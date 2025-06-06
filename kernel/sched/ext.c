/* SPDX-License-Identifier: GPL-2.0 */
/*
 * BPF extensible scheduler class: Documentation/scheduler/sched-ext.rst
 *
 * Copyright (c) 2022 Meta Platforms, Inc. and affiliates.
 * Copyright (c) 2022 Tejun Heo <tj@kernel.org>
 * Copyright (c) 2022 David Vernet <dvernet@meta.com>
 */

#include "slim.h"
#define CLUSTER_SEPARATE

enum task_event {
	PUT_PREV_TASK   = 0,
	PICK_NEXT_TASK  = 1,
	TASK_WAKE       = 2,
	TASK_MIGRATE    = 3,
	TASK_UPDATE     = 4,
	IRQ_UPDATE      = 5,
};

enum scx_internal_consts {
	SCX_WATCHDOG_MAX_TIMEOUT = 30 * HZ,
};

enum scx_ops_enable_state {
	SCX_OPS_PREPPING,
	SCX_OPS_ENABLING,
	SCX_OPS_ENABLED,
	SCX_OPS_DISABLING,
	SCX_OPS_DISABLED,
};

static inline struct task_group *css_tg(struct cgroup_subsys_state *css)
{
	return css ? container_of(css, struct task_group, css) : NULL;
}

static inline void check_class_changed(struct rq *rq, struct task_struct *p,
                         const struct sched_class *prev_class,
                         int oldprio)
{
	if (prev_class != p->sched_class) {
		if (prev_class->switched_from)
			prev_class->switched_from(rq, p);

		p->sched_class->switched_to(rq, p);
	} else if (oldprio != p->prio || dl_task(p))
		p->sched_class->prio_changed(rq, p, oldprio);
}

/*
 * sched_ext_entity->ops_state
 *
 * Used to track the task ownership between the SCX core and the BPF scheduler.
 * State transitions look as follows:
 *
 * NONE -> QUEUEING -> QUEUED -> DISPATCHING
 *   ^              |                 |
 *   |              v                 v
 *   \-------------------------------/
 *
 * QUEUEING and DISPATCHING states can be waited upon. See wait_ops_state() call
 * sites for explanations on the conditions being waited upon and why they are
 * safe. Transitions out of them into NONE or QUEUED must store_release and the
 * waiters should load_acquire.
 *
 * Tracking scx_ops_state enables sched_ext core to reliably determine whether
 * any given task can be dispatched by the BPF scheduler at all times and thus
 * relaxes the requirements on the BPF scheduler. This allows the BPF scheduler
 * to try to dispatch any task anytime regardless of its state as the SCX core
 * can safely reject invalid dispatches.
 */
enum scx_ops_state {
	SCX_OPSS_NONE,		/* owned by the SCX core */
	SCX_OPSS_QUEUEING,	/* in transit to the BPF scheduler */
	SCX_OPSS_QUEUED,	/* owned by the BPF scheduler */
	SCX_OPSS_DISPATCHING,	/* in transit back to the SCX core */

	/*
	 * QSEQ brands each QUEUED instance so that, when dispatch races
	 * dequeue/requeue, the dispatcher can tell whether it still has a claim
	 * on the task being dispatched.
	 */
	SCX_OPSS_QSEQ_SHIFT	= 2,
	SCX_OPSS_STATE_MASK	= (1LLU << SCX_OPSS_QSEQ_SHIFT) - 1,
	SCX_OPSS_QSEQ_MASK	= ~SCX_OPSS_STATE_MASK,
};

enum switch_stat {
	SCX_DISABLED,
	SCX_SWITCH_PREP,
	SCX_RQ_SWITCH_BEGIN,
	SCX_RQ_SWITCH_DONE,
	SCX_ENABLED,
};
enum switch_stat curr_ss;

/*
 * During exit, a task may schedule after losing its PIDs. When disabling the
 * BPF scheduler, we need to be able to iterate tasks in every state to
 * guarantee system safety. Maintain a dedicated task list which contains every
 * task between its fork and eventual free.
 */
DEFINE_SPINLOCK(scx_tasks_lock);
static LIST_HEAD(scx_tasks);

/* ops enable/disable */
static struct kthread_worker *scx_ops_helper;
static DEFINE_MUTEX(scx_ops_enable_mutex);
DEFINE_STATIC_PERCPU_RWSEM(scx_fork_rwsem);
static atomic_t scx_ops_enable_state_var = ATOMIC_INIT(SCX_OPS_DISABLED);
static atomic_t set_sched_clock_prepare;

static bool scx_warned_zero_slice;

DEFINE_STATIC_KEY_FALSE(scx_ops_cpu_preempt);

static atomic64_t scx_nr_rejected = ATOMIC64_INIT(0);

/*
 * The maximum amount of time in jiffies that a task may be runnable without
 * being scheduled on a CPU. If this timeout is exceeded, it will trigger
 * scx_ops_error().
 */
unsigned long scx_watchdog_timeout;

/*
 * The last time the delayed work was run. This delayed work relies on
 * ksoftirqd being able to run to service timer interrupts, so it's possible
 * that this work itself could get wedged. To account for this, we check that
 * it's not stalled in the timer tick, and trigger an error if it is.
 */
unsigned long scx_watchdog_timestamp = INITIAL_JIFFIES;

static struct delayed_work scx_watchdog_work;

/* idle tracking */
#ifdef CONFIG_SMP
#ifdef CONFIG_CPUMASK_OFFSTACK
#define CL_ALIGNED_IF_ONSTACK
#else
#define CL_ALIGNED_IF_ONSTACK __cacheline_aligned_in_smp
#endif

static struct {
	cpumask_var_t cpu;
	cpumask_var_t smt;
} idle_masks CL_ALIGNED_IF_ONSTACK;

static bool __cacheline_aligned_in_smp scx_has_idle_cpus;
#endif	/* CONFIG_SMP */

/* dispatch queues */
static struct scx_dispatch_q __cacheline_aligned_in_smp scx_dsq_global;

u32 SCX_BPF_DSQS_DEADLINE[MAX_GLOBAL_DSQS] = {0, 1, 2, 4, 6, 8, 16, 32, 64, 128};
u32 pcp_dsq_deadline = 20;
static struct scx_dispatch_q __cacheline_aligned_in_smp gdsqs[MAX_GLOBAL_DSQS];
static DEFINE_PER_CPU(struct scx_dispatch_q, pcp_ldsq);

static u64 max_ext_dsq_internal_id;

/* a dsq idx, whether task push to little domain cpu or bit domain cpu*/
#define CLUSTER_SEPARATE_IDX	(8)
#define GDSQS_ID_BASE		(3)
#define RT_DSQ_IDX		(0)
#define NON_PERIOD_START	(5)
#define NON_PERIOD_END		(MAX_GLOBAL_DSQS)
#define CREATE_DSQ_LEVEL_WITHIN	(1)

struct hmbird_sched_info {
	spinlock_t lock;
	int curr_idx[2];
	int rtime[MAX_GLOBAL_DSQS];
};

struct pcp_sched_info {
	s64 pcp_seq;
	int rtime;
	bool pcp_round;
};

/*
 * pcp_info may rw by another cpu.
 * protected by rq->lock.
 */
atomic64_t pcp_dsq_round;
static DEFINE_PER_CPU(struct pcp_sched_info, pcp_info);

static struct hmbird_sched_info sinfo;

static unsigned long pcp_dsq_quota __read_mostly = 3 * NSEC_PER_MSEC;
static unsigned long dsq_quota[MAX_GLOBAL_DSQS] = {
					0, 0, 0, 0, 0,
					32 * NSEC_PER_MSEC,
					20 * NSEC_PER_MSEC,
					14 * NSEC_PER_MSEC,
					8 * NSEC_PER_MSEC,
					6 * NSEC_PER_MSEC
};

struct cluster_ctx {
        /* cpu-dsq map must within [lower, upper) */
	int upper;
	int lower;
	int tidx;
};

enum stat_items {
	GLOBAL_STAT,
	CPU_ALLOW_FAIL,
	RT_CNT,
	KEY_TASK_CNT,
	SWITCH_IDX,
	TIMEOUT_CNT,

	TOTAL_DSP_CNT,
	MOVE_RQ_CNT,

	DWORD_STAT_END = MOVE_RQ_CNT,

	GDSQ_CNT,
	ERR_IDX,
	PCP_TIMEOUT_CNT,
	PCP_LDSQ_CNT,
	PCP_ENQL_CNT,

	MAX_ITEMS,
};
static DEFINE_SPINLOCK(stats_lock);
static char *stats_str[MAX_ITEMS] = {
	"global stat", "cpu_allow_fail", "rt_cnt", "key_task_cnt",
	"switch_idx", "timeout_cnt", "total_dsp_cnt", "move_rq_cnt",
	"gdsq_cnt", "err_idx", "pcp_timeout_cnt", "pcp_ldsq_cnt",
	"pcp_enql_cnt"
};


struct stats_struct {
	u64 global_stat[2];
	u64 cpu_allow_fail[2];
	u64 rt_cnt[2];
	u64 key_task_cnt[2];
	u64 switch_idx[2];
	u64 timeout_cnt[2];

	/* for compatible, only use [0] */
	u64 total_dsp_cnt[2];
	u64 move_rq_cnt[2];

	u64 gdsq_count[MAX_GLOBAL_DSQS][2];
	u64 err_idx[5];
	u64 pcp_timeout_cnt[NR_CPUS];
	u64 pcp_ldsq_count[NR_CPUS][2];
	u64 pcp_enql_cnt[NR_CPUS];
} stats_data;


static void slim_stats_record(enum stat_items item, int idx, int dsq_id, int cpu)
{
	unsigned long flags;
	u64 *pval;
	u64 *pbase = (u64*)&stats_data;

	if (!slim_stats)
		return;

	switch (item) {
	case GLOBAL_STAT:
		fallthrough;
	case CPU_ALLOW_FAIL:
		fallthrough;
	case RT_CNT:
		fallthrough;
	case KEY_TASK_CNT:
		fallthrough;
	case SWITCH_IDX:
		fallthrough;
	case TIMEOUT_CNT:
		fallthrough;
	case TOTAL_DSP_CNT:
		fallthrough;
	case MOVE_RQ_CNT:
		pval = pbase + item * 2 + idx;
		break;
	case GDSQ_CNT:
		pval = &stats_data.gdsq_count[dsq_id][idx];
		break;
	case ERR_IDX:
		pval = &stats_data.err_idx[idx];
		break;
	case PCP_TIMEOUT_CNT:
		pval = &stats_data.pcp_timeout_cnt[cpu];
		break;
	case PCP_LDSQ_CNT:
		pval = &stats_data.pcp_ldsq_count[cpu][idx];
		break;
	case PCP_ENQL_CNT:
		pval = &stats_data.pcp_enql_cnt[cpu];
		break;
	default:
		return;
	}

	spin_lock_irqsave(&stats_lock, flags);
	*pval += 1;
	spin_unlock_irqrestore(&stats_lock, flags);
}

#define PRINT_INTV	(5 * HZ)
void stats_print(char *buf, int len)
{
	int idx = 0, j, ret;
	int item = 0;
	u64 *pval;
	u64 *pbase = (u64*)&stats_data;


	for (item = 0; item < MAX_ITEMS; item++) {
		if (item <= DWORD_STAT_END) {
			pval = pbase + item * 2;
			ret = snprintf(&buf[idx], len - idx, "%s:%llu, %llu\n",
					stats_str[item], pval[0],  pval[1]);
			if (ret < 0 || ret >= len - idx)
				return;
			idx += ret;
		} else if (GDSQ_CNT == item) {
			for (j = 0; j < MAX_GLOBAL_DSQS; j++) {
				pval = (u64*)&stats_data.gdsq_count[j];
				ret = snprintf(&buf[idx], len - idx, "%s[%d]:%llu, %llu\n",
						stats_str[item], j, pval[0], pval[1]);
				if (ret < 0 || ret >= len - idx)
					return;
				idx += ret;
			}
		} else if (ERR_IDX == item) {
			pval = (u64*)&stats_data.err_idx;
			ret = snprintf(&buf[idx], len - idx, "%s:%llu, %llu, %llu, %llu,"
						"%llu\n", stats_str[item], pval[0],
						pval[1], pval[2], pval[3], pval[4]);
			if (ret < 0 || ret >= len - idx)
				return;
			idx += ret;
		} else if (PCP_TIMEOUT_CNT == item) {
			for (j = 0; j < nr_cpu_ids; j++) {
				pval = (u64*)&stats_data.pcp_timeout_cnt[j];
				ret = snprintf(&buf[idx], len - idx, "%s[%d]:%llu\n",
							stats_str[item], j, *pval);
				if (ret < 0 || ret >= len - idx)
					return;
				idx += ret;
			}
		} else if (PCP_LDSQ_CNT == item) {
			for (j = 0; j < nr_cpu_ids; j++) {
				pval = (u64*)&stats_data.pcp_ldsq_count[j];
				ret = snprintf(&buf[idx], len - idx, "%s[%d]:%llu,%llu\n",
						stats_str[item], j, pval[0], pval[1]);
				if (ret < 0 || ret >= len - idx)
					return;
				idx += ret;
			}
		} else if (PCP_ENQL_CNT == item) {
			for (j = 0; j < nr_cpu_ids; j++) {
				pval = (u64*)&stats_data.pcp_enql_cnt[j];
				ret = snprintf(&buf[idx], len - idx, "%s[%d]:%llu\n",
						stats_str[item], j, *pval);
				if (ret < 0 || ret >= len - idx)
					return;
				idx += ret;
			}
		}
		else {}
	}
	buf[idx] = '\0';
}

static struct {
	cpumask_var_t exclusive;
	cpumask_var_t partial;
	cpumask_var_t big;
	cpumask_var_t little;
} iso_masks;

enum cpu_type
{
	LITTLE,
	BIG,
	PARTIAL,
	EXCLUSIVE,
	INVALID
};

enum dsq_type
{
	GLOBAL_DSQ,
	PCP_DSQ,
	OTHER,
	MAX_DSQ_TYPE,
};

static enum cpu_type cpu_cluster(int cpu)
{
	int exclusive = 0;

	trace_android_vh_scx_cpu_exclusive(cpu, &exclusive);
	if (exclusive) {
		return EXCLUSIVE;
	} else {
		if (cpumask_test_cpu(cpu, iso_masks.little)) {
			return LITTLE;
		} else if (cpumask_test_cpu(cpu, iso_masks.big)) {
			return BIG;
		} else if (cpumask_test_cpu(cpu, iso_masks.partial)) {
			return PARTIAL;
		} else if (cpumask_test_cpu(cpu, iso_masks.exclusive)) {
			return EXCLUSIVE;
		}
	}
	return INVALID;
}

static enum dsq_type get_dsq_type(struct scx_dispatch_q *dsq)
{
	if (!dsq)
		return OTHER;

	if ((dsq->id & SCX_DSQ_FLAG_BUILTIN) &&
		((dsq->id & 0xff) >= GDSQS_ID_BASE) &&
		((dsq->id & 0xff) < MAX_GLOBAL_DSQS))
		return GLOBAL_DSQ;

	if ((dsq->id & SCX_DSQ_FLAG_BUILTIN) &&
		((dsq->id & 0xff) >= MAX_GLOBAL_DSQS) &&
		((dsq->id & 0xff) < max_ext_dsq_internal_id))
		return PCP_DSQ;

	return OTHER;
}

static int dsq_id_to_internal(struct scx_dispatch_q *dsq)
{
	enum dsq_type type;

	type = get_dsq_type(dsq);
	switch (type) {
	case GLOBAL_DSQ:
	case PCP_DSQ:
		return (dsq->id & 0xff) - GDSQS_ID_BASE;
	default:
		return -1;
	}
	return -1;
}

/* Noting!!!!!!!!
 * idle_masks.cpu does not accurately reflect the idle status of the CPU.
 * Just provide a core selection tendency.
 */
static void update_partial_idle(bool idle)
{
	if (idle)
		cpumask_or(idle_masks.cpu, idle_masks.cpu, iso_masks.partial);
	else
		cpumask_andnot(idle_masks.cpu, idle_masks.cpu, iso_masks.partial);
}

/*
 * Need more synchronization for these two variables?
 * I choose not to.
 */
static int l_need_rescue, b_need_rescue;
static void set_partial_status(bool enable, bool little, bool big)
{
	WRITE_ONCE(partial_enable, enable);
	WRITE_ONCE(l_need_rescue, little);
	WRITE_ONCE(b_need_rescue, big);
}

static bool is_little_need_rescue(void)
{
	return READ_ONCE(l_need_rescue);
}

static bool is_big_need_rescue(void)
{
	return READ_ONCE(b_need_rescue);
}

static bool is_partial_enabled(void)
{
	return READ_ONCE(partial_enable);
}

static bool is_partial_cpu(int cpu)
{
	return cpumask_test_cpu(cpu, iso_masks.partial);
}

static bool skip_update_idle(void)
{
	int cpu = smp_processor_id();
	enum cpu_type type = cpu_cluster(cpu);

	if (type == EXCLUSIVE ||
		/* partial enable may changed during idle, it doesn't matter. */
		(!is_partial_enabled() && type == PARTIAL))
		return true;

	return false;
}

static void init_isolate_cpus(void)
{
	WARN_ON(!alloc_cpumask_var(&iso_masks.partial, GFP_KERNEL));
	WARN_ON(!alloc_cpumask_var(&iso_masks.exclusive, GFP_KERNEL));
	WARN_ON(!alloc_cpumask_var(&iso_masks.big, GFP_KERNEL));
	WARN_ON(!alloc_cpumask_var(&iso_masks.little, GFP_KERNEL));
	cpumask_set_cpu(0, iso_masks.little);
	cpumask_set_cpu(1, iso_masks.little);
	cpumask_set_cpu(2, iso_masks.big);
	cpumask_set_cpu(3, iso_masks.big);
	cpumask_set_cpu(4, iso_masks.exclusive);
	cpumask_set_cpu(5, iso_masks.partial);
	cpumask_set_cpu(6, iso_masks.partial);
	cpumask_set_cpu(7, iso_masks.exclusive);
}


/* Should do get/put to guarantee exists,TODO. */
static struct cgroup *cgroup_ancestor_l1(struct cgroup *cgrp)
{
	int i;
	struct cgroup *anc;

	for (i = 0; i < cgrp->level; i++) {
		anc = cgrp->ancestors[i];
		if (CREATE_DSQ_LEVEL_WITHIN != anc->level)
			continue;
		return anc;
	}
	pr_err("<slim_sched><error>:error cgroup = %s\n", cgrp->kn->name);
	return NULL;
}

#define PCP_IDX_BIT    (1 << 31)

static bool is_pcp_rt(struct task_struct *p)
{
	return rt_prio(p->prio) && (1 == p->nr_cpus_allowed);
}

static bool is_pcp_idx(int idx)
{
	return idx & PCP_IDX_BIT;
}

static int find_idx_from_task(struct task_struct *p)
{
	int idx, cpu;
	int sp_dl;
	struct task_group *tg = p->sched_task_group;

	if (1 == p->nr_cpus_allowed || is_migration_disabled(p)) {
		cpu = cpumask_any(p->cpus_ptr);
		idx = cpu | PCP_IDX_BIT;
		return idx;
	}

	sp_dl = p->scx->sched_prop & SCHED_PROP_DEADLINE_MASK;
	if (sp_dl) {
		idx = sp_dl;
		goto done;
	}

	/*
	 * Those rt threads(prio>=50) which we can not recognized,
	 * put them into SCHED_PROP_DEADLINE_LEVEL3.
	 */
	if (rt_prio(p->prio)) {
		idx = SCHED_PROP_DEADLINE_LEVEL3;
		goto done;
	}

	if (tg && tg->css.cgroup) {
		idx = tg->scx_deadline_idx;
	} else {
		idx = DEFAULT_CGROUP_DL_IDX;
	}

done:
	/* Must check whether cpu_allowed match cluster. TODO. */

	if (idx < 0 || idx >= MAX_GLOBAL_DSQS) {
		pr_err("<slim_sched><error> : idx error, idx = %d-----\n", idx);
		idx = DEFAULT_CGROUP_DL_IDX;
	}
	return idx;
}

static struct scx_dispatch_q* find_dsq_from_task(struct task_struct *p)
{
        int idx;
	unsigned long flags;
	struct scx_dispatch_q *dsq;

        if (!p)
                return NULL;

        idx = find_idx_from_task(p);
	if(is_pcp_idx(idx)) {
		idx &= ~PCP_IDX_BIT;
                dsq = &per_cpu(pcp_ldsq, idx);
		p->scx->gdsq_idx = dsq_id_to_internal(dsq);
		slim_stats_record(PCP_LDSQ_CNT, 0, 0, idx);
	} else {
		dsq = &gdsqs[idx];
                p->scx->gdsq_idx = idx;
		slim_stats_record(GDSQ_CNT, 0, idx, 0);
	}

	raw_spin_lock_irqsave(&dsq->lock, flags);
        if (list_empty(&dsq->fifo)) {
                dsq->last_consume_at = jiffies;
        }
	raw_spin_unlock_irqrestore(&dsq->lock, flags);

        return dsq;
}


bool consume_dispatch_q(struct rq *rq, struct rq_flags *rf,
                               struct scx_dispatch_q *dsq);

static void set_partial_rescue(bool p_state, bool l_over, bool b_over)
{
	set_partial_status(p_state, l_over, b_over);
	update_partial_idle(p_state);
	scx_internal_systrace("C|9999|partial_enable|%d\n", is_partial_enabled());
	scx_internal_systrace("C|9999|l_need_rescue|%d\n", is_little_need_rescue());
	scx_internal_systrace("C|9999|b_need_rescue|%d\n", is_big_need_rescue());
}

void partial_dynamic_ctrl(void)
{
	int cpu;
	u64 ratio, util = 0;
	u64 lmax = 0, bmax = 0;
	bool l_over, l_under, b_over, b_under;
	static bool last_l_over = false, last_b_over = false;

	for_each_cpu(cpu, iso_masks.little) {
		trace_android_vh_get_util(cpu, NULL, &util);
		ratio = util * 100 / arch_scale_cpu_capacity(cpu);
		if (ratio > lmax)
			lmax = ratio;
	}
	l_over = lmax >= cpuctrl_high_ratio;
	l_under = lmax < cpuctrl_low_ratio;
	for_each_cpu(cpu, iso_masks.big) {
		trace_android_vh_get_util(cpu, NULL, &util);
		ratio = util * 100 / arch_scale_cpu_capacity(cpu);
		if (ratio > bmax)
			bmax = ratio;
	}
	b_over = bmax >= cpuctrl_high_ratio;
	b_under = bmax < cpuctrl_low_ratio;

	if (is_partial_enabled() && (l_over || b_over)) {
		if (last_l_over != l_over || last_b_over != b_over)
			set_partial_rescue(true, l_over, b_over);
	}
	else if (!is_partial_enabled() && (l_over || b_over)) {
		set_partial_rescue(true, l_over, b_over);
	} else if (is_partial_enabled() && l_under && b_under) {
		set_partial_rescue(false, false, false);
	} else {}
	last_l_over = l_over;
	last_b_over = b_over;
}

static inline void slim_trace_show_cpu_consume_dsq_idx(unsigned int cpu, unsigned int idx)
{
	scx_info_systrace("C|9999|Cpu%d_dsq_id|%d\n", cpu, idx);
}

static int consume_target_dsq(struct rq *rq, struct rq_flags *rf, unsigned int idx)
{
	if (idx < 0 || idx >= MAX_GLOBAL_DSQS)
		return 0;

	if (consume_dispatch_q(rq, rf, &gdsqs[idx])) {
		slim_stats_record(GDSQ_CNT, 1, idx, 0);
		return 1;
	}
	return 0;
}

static int consume_period_dsq(struct rq *rq, struct rq_flags *rf)
{
        int i;

        for (i = 0; i < NON_PERIOD_START; i++) {
                if (consume_dispatch_q(rq, rf, &gdsqs[i])) {
			slim_stats_record(GDSQ_CNT, 1, i, 0);
                        return 1;
                }
        }
        return 0;
}

static void update_timeout_stats(struct rq *rq, struct scx_dispatch_q *dsq, u64 deadline)
{
	struct sched_ext_entity *entity;
	unsigned long flags;

	raw_spin_lock_irqsave(&dsq->lock, flags);
	if (list_empty(&dsq->fifo))
		goto clear_timeout;

	entity = list_first_entry(&dsq->fifo, struct sched_ext_entity, dsq_node.fifo);
	if (time_before_eq(jiffies, entity->runnable_at + msecs_to_jiffies(deadline)))
		goto clear_timeout;

	raw_spin_unlock_irqrestore(&dsq->lock, flags);
	scx_info_trace("<hmbird_sched><timeout>dsq[%d] still timeout task-%s, "
				"jiffies = %lu, deadline = %lu, runnable at = %lu\n",
				dsq_id_to_internal(dsq), entity->task->comm,
				jiffies, msecs_to_jiffies(deadline), entity->runnable_at);
	scx_info_systrace("C|9999|dsq_%d_timeout|%d\n", dsq_id_to_internal(dsq), 1);
	return;

clear_timeout:
	scx_info_trace("<hmbird_sched><timeout>dsq[%d] clear timeout\n",
				dsq_id_to_internal(dsq));
	scx_info_systrace("C|9999|dsq_%d_timeout|%d\n", dsq_id_to_internal(dsq), 0);
	dsq->is_timeout = false;
	slim_stats_record(PCP_TIMEOUT_CNT, 0, 0, cpu_of(rq));
	raw_spin_unlock_irqrestore(&dsq->lock, flags);
}

static void systrace_output_rtime_state(struct scx_dispatch_q *dsq, int rtime)
{
	scx_info_systrace("C|9999|dsq%d_rtime|%d\n", dsq_id_to_internal(dsq), rtime);
}

static int consume_pcp_dsq(struct rq *rq, struct rq_flags *rf, bool any)
{
        bool is_timeout;
        int cpu = cpu_of(rq);
        unsigned long flags;
        struct scx_dispatch_q *dsq = &per_cpu(pcp_ldsq, cpu);

        raw_spin_lock_irqsave(&dsq->lock, flags);
        is_timeout = dsq->is_timeout;
        raw_spin_unlock_irqrestore(&dsq->lock, flags);

	/*
	 * dsq->is_timeout may change here, let it be.
	 * it won't cause serious problems.
	 * the same for consume_dispatch_q later.
	 */
	if (!is_timeout && !any)
		return 0;

	if (consume_dispatch_q(rq, rf, dsq)) {
		if (is_timeout) {
			scx_info_trace("<hmbird_sched><timeout>dsq[%d]"
					" consume a pcp timeout task\n",
						dsq_id_to_internal(dsq));
			update_timeout_stats(rq, dsq, pcp_dsq_deadline);
			slim_stats_record(PCP_TIMEOUT_CNT, 0, 0, cpu);
		}
		slim_stats_record(PCP_LDSQ_CNT, 1, 0, cpu);
		return 1;
	}
	/*
	 * No pcp task, clear quota.
	 */
	if (any) {
		if (per_cpu(pcp_info, cpu_of(rq)).pcp_round) {
			per_cpu(pcp_info, cpu).rtime = 0;
			per_cpu(pcp_info, cpu).pcp_round = false;
			scx_info_systrace("C|9999|pcp_%d_round|%d\n", cpu, false);
			systrace_output_rtime_state(&per_cpu(pcp_ldsq, cpu),
					per_cpu(pcp_info, cpu).rtime);
		}
	}
        return 0;
}

static int consume_timeout_dsq(struct rq *rq, struct rq_flags *rf, struct cluster_ctx *ctx)
{
	int i;
	bool is_timeout;
	unsigned long flags;

	/* Third param <false> means only consume timeout pcp dsq. */
	if (consume_pcp_dsq(rq, rf, false))
		return 1;

	for(i = ctx->lower; i < ctx->upper; i++) {
		raw_spin_lock_irqsave(&gdsqs[i].lock, flags);
                is_timeout = gdsqs[i].is_timeout;
                raw_spin_unlock_irqrestore(&gdsqs[i].lock, flags);
		/* gdsqs[i].is_timeout may change here, let it be... */
		if (is_timeout) {
			/*
			 * consume_dispatch_q will acquire dsq-lock,
			 * So cannot keep lock here, annoy enough.
			 * may rewrite a consume_dispatch_q_locked, TODO.
			 */
                        if (consume_dispatch_q(rq, rf, &gdsqs[i])) {
				scx_info_trace("<hmbird_sched><timeout>dsq[%d]"
						" consume a timeout task\n", i);
                                slim_stats_record(TIMEOUT_CNT, ctx->tidx, 0, 0);
				update_timeout_stats(rq, &gdsqs[i], SCX_BPF_DSQS_DEADLINE[i]);
                                return 1;
                        }
                }
	}
	return 0;
}

static int check_pcp_dsq_round(struct rq *rq, struct rq_flags *rf)
{
	if (per_cpu(pcp_info, cpu_of(rq)).pcp_round) {
		if (consume_pcp_dsq(rq, rf, true))
			return 1;
	}
	return 0;
}

static int check_non_period_dsq_phase(struct rq *rq, struct rq_flags *rf,
						     int tmp, int cidx, int tidx)
{
	unsigned long flags;

	if (consume_dispatch_q(rq, rf, &gdsqs[tmp])) {
		if (tmp != cidx) {
			spin_lock(&sinfo.lock);
			sinfo.curr_idx[tidx] = tmp;
			spin_unlock(&sinfo.lock);
			scx_info_systrace("C|9999|cidx_%d|%d\n", tidx, sinfo.curr_idx[tidx]);
			slim_stats_record(SWITCH_IDX, 0, 0, 0);
		}
		slim_stats_record(GDSQ_CNT, 1, tmp, 0);

		raw_spin_lock_irqsave(&gdsqs[tmp].lock, flags);
		gdsqs[tmp].last_consume_at = jiffies;
		raw_spin_unlock_irqrestore(&gdsqs[tmp].lock, flags);
		return 1;
	}
	return 0;

}

static int get_cidx(struct cluster_ctx *ctx)
{
	int cidx;

	spin_lock(&sinfo.lock);
	cidx = sinfo.curr_idx[ctx->tidx];
	if (cidx < ctx->lower || cidx >= ctx->upper) {
		sinfo.curr_idx[ctx->tidx] = ctx->lower;
		scx_info_systrace("C|9999|cidx_%d|%d\n", ctx->tidx, sinfo.curr_idx[ctx->tidx]);
		slim_stats_record(ERR_IDX, ctx->tidx + 3, 0, 0);
		cidx = sinfo.curr_idx[ctx->tidx];
	}
	spin_unlock(&sinfo.lock);

	return cidx;
}


static int gen_cluster_ctx(struct cluster_ctx* ctx, enum cpu_type type)
{
	switch (type) {
	case PARTIAL:
		if (!is_partial_enabled())
			return -1;
		fallthrough;
#ifdef CLUSTER_SEPARATE
        case BIG:
                ctx->lower = NON_PERIOD_START;
                ctx->upper = CLUSTER_SEPARATE_IDX;
                ctx->tidx = 0;
                break;
        case LITTLE:
                ctx->lower = CLUSTER_SEPARATE_IDX;
                ctx->upper = NON_PERIOD_END;
                ctx->tidx = 1;
                break;
#else
	case BIG:
/*		ctx->lower = NON_PERIOD_START;
                ctx->upper = NON_PERIOD_END;
                ctx->tidx = 0;
		break;
*/
	case LITTLE:
		ctx->lower = NON_PERIOD_START;
		ctx->upper = NON_PERIOD_END;
		ctx->tidx = 0;
		break;
#endif
        default:
                WARN_ON(1);
                return -1;
        }
	return 0;
}

static int consume_non_period_dsq(struct rq *rq, struct rq_flags *rf, enum cpu_type type)
{
	struct cluster_ctx ctx;
        int cidx;
        int tmp;

	if (gen_cluster_ctx(&ctx, type))
		return 0;

	if (consume_timeout_dsq(rq, rf, &ctx))
		return 1;

	cidx = get_cidx(&ctx);
	tmp = cidx;
	do {
		if (check_pcp_dsq_round(rq, rf))
			return 1;
		if (check_non_period_dsq_phase(rq, rf, tmp, cidx, ctx.tidx))
			return 1;
		spin_lock(&sinfo.lock);
		sinfo.rtime[tmp] = 0;
		systrace_output_rtime_state(&gdsqs[tmp], sinfo.rtime[tmp]);
		tmp++;
		if (tmp >= ctx.upper) {
			atomic64_inc(&pcp_dsq_round);
			scx_info_systrace("C|9999|pcp_dsq_round|%lld\n", atomic64_read(&pcp_dsq_round));
			tmp = ctx.lower;
		}
		spin_unlock(&sinfo.lock);
	} while (tmp != cidx);

	return consume_pcp_dsq(rq, rf, true);
}

static bool consume_hmbird_global_dsq(struct rq *rq, struct rq_flags *rf)
{
	enum cpu_type type = cpu_cluster(cpu_of(rq));
	int period_allowed = 1;
	int non_period_allowed = 1;

	trace_android_vh_scx_consume_dsq_allowed(rq, rf, SCHED_EXT_DSQ_TYPE_PERIOD, &period_allowed);
	trace_android_vh_scx_consume_dsq_allowed(rq, rf, SCHED_EXT_DSQ_TYPE_NON_PERIOD, &non_period_allowed);

	switch (type) {
		case EXCLUSIVE:
			if (consume_pcp_dsq(rq, rf, true))
				return 1;
			return 0;

		case PARTIAL:
			if (!is_partial_enabled()) {
				if (consume_pcp_dsq(rq, rf, true))
					return 1;
				return 0;
			}
			if (is_little_need_rescue()) {
				if (consume_target_dsq(rq, rf, RT_DSQ_IDX))
					return 1;
				if (consume_target_dsq(rq, rf, SCHED_PROP_DEADLINE_LEVEL1))
					return 1;
				if (consume_target_dsq(rq, rf, SCHED_PROP_DEADLINE_LEVEL2))
					return 1;

				if (consume_target_dsq(rq, rf, SCHED_PROP_DEADLINE_LEVEL3))
					return 1;
				if (non_period_allowed && consume_non_period_dsq(rq, rf, LITTLE))
					return 1;
			}
			if (is_big_need_rescue()) {
				if (period_allowed && consume_period_dsq(rq, rf))
					return 1;
				if (non_period_allowed && consume_non_period_dsq(rq, rf, BIG))
					return 1;
			}
			break;

		case BIG:
			if (period_allowed && consume_period_dsq(rq, rf))
				return 1;
			if (non_period_allowed && consume_non_period_dsq(rq, rf, type))
				return 1;
			break;

		case LITTLE:
			if (consume_target_dsq(rq, rf, RT_DSQ_IDX))
				return 1;
			if (consume_target_dsq(rq, rf, SCHED_PROP_DEADLINE_LEVEL1))
				return 1;
			if (consume_target_dsq(rq, rf, SCHED_PROP_DEADLINE_LEVEL2))
				return 1;

			if (consume_target_dsq(rq, rf, SCHED_PROP_DEADLINE_LEVEL3))
				return 1;

			if (non_period_allowed && consume_non_period_dsq(rq, rf, type))
				return 1;
			break;

		default:
			break;
	}

	return 0;
}

static int consume_dispatch_global(struct rq *rq, struct rq_flags *rf)
{
	return consume_hmbird_global_dsq(rq, rf);
}


static void update_runningtime(struct rq *rq, struct task_struct *p, unsigned long exec_time)
{
	int idx;

	/* which dsq belongs to while task enqueue, task will consume its running time. */
	idx = p->scx->gdsq_idx;
	/* Only non-period dsq share running time between each other. */
	if (idx < NON_PERIOD_START || idx >= max_ext_dsq_internal_id)
		return;

	if (idx >= MAX_GLOBAL_DSQS) {
		per_cpu(pcp_info, cpu_of(rq)).rtime += exec_time;
		systrace_output_rtime_state(&per_cpu(pcp_ldsq, cpu_of(rq)),
					per_cpu(pcp_info, cpu_of(rq)).rtime);
	}
	else {
		spin_lock(&sinfo.lock);
		sinfo.rtime[idx] += exec_time;
		spin_unlock(&sinfo.lock);
		systrace_output_rtime_state(&gdsqs[idx], sinfo.rtime[idx]);
	}
}

static void update_dsq_idx(struct rq *rq, struct task_struct *p, enum cpu_type type)
{
	int cidx;
	struct cluster_ctx ctx;
	int cpu = cpu_of(rq);

	if (gen_cluster_ctx(&ctx, type))
		return;

	spin_lock(&sinfo.lock);
	cidx = sinfo.curr_idx[ctx.tidx];
	if (cidx < ctx.lower || cidx >= ctx.upper) {
		sinfo.curr_idx[ctx.tidx] = ctx.lower;
		scx_info_systrace("C|9999|cidx_%d|%d\n", ctx.tidx, sinfo.curr_idx[ctx.tidx]);
		slim_stats_record(ERR_IDX, ctx.tidx, 0, 0);
		cidx = sinfo.curr_idx[ctx.tidx];
	}

	while(1) {
		if (per_cpu(pcp_info, cpu).pcp_round) {
			if (per_cpu(pcp_info, cpu).rtime >= pcp_dsq_quota) {
				scx_info_trace("<hmbird_sched><non> : cpu[%d] pcp_dsq_round is full,"
						" rtime = %d \n", cpu, per_cpu(pcp_info, cpu).rtime);
				per_cpu(pcp_info, cpu).rtime = 0;
				per_cpu(pcp_info, cpu).pcp_round = false;
				scx_info_systrace("C|9999|pcp_%d_round|%d\n", cpu, false);
				systrace_output_rtime_state(&per_cpu(pcp_ldsq, cpu),
						per_cpu(pcp_info, cpu_of(rq)).rtime);
			}
		}
		if (sinfo.rtime[cidx] < dsq_quota[cidx])
			break;

		/* clear current dsq rtime */
		sinfo.rtime[cidx] = 0;
		systrace_output_rtime_state(&gdsqs[cidx], sinfo.rtime[cidx]);

		sinfo.curr_idx[ctx.tidx]++;
		scx_info_systrace("C|9999|cidx_%d|%d\n", ctx.tidx, sinfo.curr_idx[ctx.tidx]);
		if (sinfo.curr_idx[ctx.tidx] >= ctx.upper) {
			atomic64_inc(&pcp_dsq_round);
			scx_info_systrace("C|9999|pcp_dsq_round|%lld\n", atomic64_read(&pcp_dsq_round));
			sinfo.curr_idx[ctx.tidx] = ctx.lower;
			scx_info_systrace("C|9999|cidx_%d|%d\n", ctx.tidx, sinfo.curr_idx[ctx.tidx]);
		}
		cidx = sinfo.curr_idx[ctx.tidx];
		slim_stats_record(SWITCH_IDX, 1, 0, 0);
	}
	spin_unlock(&sinfo.lock);
}


static void update_dispatch_dsq_info(struct rq *rq, struct task_struct *p)
{
	enum cpu_type type;
	if (!rq || !p)
		return;

	type = cpu_cluster(cpu_of(rq));
	switch(type) {
	case PARTIAL:
                if (is_partial_enabled())
                        break;
		fallthrough;
	case EXCLUSIVE:
		return;
	default:
		break;
        }
	update_dsq_idx(rq, p, type);
}


static bool scan_dsq_timeout(struct rq *rq, struct scx_dispatch_q *dsq, u64 deadline)
{
	struct sched_ext_entity *entity;

	raw_spin_lock(&dsq->lock);
	if (list_empty(&dsq->fifo) || dsq->is_timeout) {
		raw_spin_unlock(&dsq->lock);
		return false;
	}

	entity = list_first_entry(&dsq->fifo, struct sched_ext_entity, dsq_node.fifo);
	if (!entity) {
		WARN_ON(1);
		raw_spin_unlock(&dsq->lock);
		return false;
	}

	if (time_before_eq(jiffies, entity->runnable_at + msecs_to_jiffies(deadline))) {
		raw_spin_unlock(&dsq->lock);
		return false;
	}

	dsq->is_timeout = true;
	scx_info_trace("<hmbird_sched><timeout>dsq[%d] has timeout task-%s, "
				"jiffies = %lu, runnable at = %lu\n",
				dsq_id_to_internal(dsq), entity->task->comm,
				jiffies, entity->runnable_at);
	scx_info_systrace("C|9999|dsq_%d_timeout|%d\n", dsq_id_to_internal(dsq), 1);
	raw_spin_unlock(&dsq->lock);

	return true;
}

void scan_timeout(struct rq *rq)
{
	int i;
	int cpu = cpu_of(rq);
	struct scx_dispatch_q *dsq;
	static u64 last_scan_at = 0;
	static DEFINE_PER_CPU(u64, pcp_last_scan_at);

	if (time_before_eq(jiffies, (unsigned long)per_cpu(pcp_last_scan_at, cpu)))
		return;
	per_cpu(pcp_last_scan_at, cpu) = jiffies;

	dsq = &per_cpu(pcp_ldsq, cpu);
	scan_dsq_timeout(rq, dsq, pcp_dsq_deadline);

	if (time_before_eq(jiffies, (unsigned long)last_scan_at))
		return;
	last_scan_at = jiffies;

	for (i = NON_PERIOD_START; i < NON_PERIOD_END; i++) {
		dsq = &gdsqs[i];
		scan_dsq_timeout(rq, dsq, SCX_BPF_DSQS_DEADLINE[i]);
	}
}

/*******************************Initialize***********************************/

void init_dsq(struct scx_dispatch_q *dsq, u64 dsq_id);
static void init_dsq_at_boot(void)
{
	int i, cpu;

	for (i = 0; i < MAX_GLOBAL_DSQS; i++) {
		init_dsq(&gdsqs[i], (u64)SCX_DSQ_FLAG_BUILTIN |
				(GDSQS_ID_BASE + i));
        }
	for_each_possible_cpu(cpu)
		init_dsq(&per_cpu(pcp_ldsq, cpu), (u64)SCX_DSQ_FLAG_BUILTIN |
				(GDSQS_ID_BASE + i + cpu));

	max_ext_dsq_internal_id = GDSQS_ID_BASE + i + cpu;
	spin_lock_init(&sinfo.lock);
}


static int cgrp_name_to_idx(struct cgroup *cgrp)
{
	int idx;

	if (!cgrp)
		return -1;

	if (!strcmp(cgrp->kn->name, "display")
			|| !strcmp(cgrp->kn->name, "multimedia"))
		idx = 5; /* 8ms */
	else if (!strcmp(cgrp->kn->name, "top-app")
			|| !strcmp(cgrp->kn->name, "ss-top"))
		idx = 6; /* 16ms */
	else if (!strcmp(cgrp->kn->name, "ssfg")
			|| !strcmp(cgrp->kn->name, "foreground"))
		idx = 7; /* 32ms */
	else if (!strcmp(cgrp->kn->name, "bg")
			|| !strcmp(cgrp->kn->name, "log")
			|| !strcmp(cgrp->kn->name, "dex2oat")
			|| !strcmp(cgrp->kn->name, "background"))
		idx = 9; /* 128ms */
	else
		idx = DEFAULT_CGROUP_DL_IDX; /* 64ms */

	return idx;
}

static void init_root_tg(struct cgroup  *cgrp, struct task_group *tg)
{
	if (!cgrp || !tg)
		return;
	tg->scx_deadline_idx = DEFAULT_CGROUP_DL_IDX;
}

static void init_level1_tg(struct cgroup *cgrp, struct task_group *tg)
{
	if (!cgrp || !tg)
		return;

	if (-1 == tg->scx_deadline_idx)
		tg->scx_deadline_idx = cgrp_name_to_idx(cgrp);
}

static void init_child_tg(struct cgroup *cgrp, struct task_group *tg)
{
	struct cgroup *l1cgrp;

	if (!cgrp || !tg)
		return;

	l1cgrp = cgroup_ancestor_l1(cgrp);
	if (NULL == l1cgrp) {
		pr_err("<slim_sched><error> : can't find l1 parent cgrp\n");
		tg->scx_deadline_idx = DEFAULT_CGROUP_DL_IDX;
		return;
	}
	tg->scx_deadline_idx = cgrp_name_to_idx(l1cgrp);
}

static void cgrp_dsq_idx_init(struct cgroup *cgrp, struct task_group *tg)
{
	switch (cgrp->level) {
	case 0:
		init_root_tg(cgrp, tg);
		break;
	case 1:
		init_level1_tg(cgrp, tg);
		break;
	default:
		init_child_tg(cgrp, tg);
		break;
	}
}

/**************************************************************************/

static u64 tick_sched_clock;
static DECLARE_COMPLETION(tick_sched_clock_completion);

void scx_scheduler_tick(void)
{
	int cpu = smp_processor_id();
	struct rq *rq = cpu_rq(cpu);

	if (!atomic_cmpxchg(&set_sched_clock_prepare, true, false))
		return;

	if(unlikely(!tick_sched_clock)) {
		/*
		 * Let the window begin 20us prior to the tick,
		 * that way we are guaranteed a rollover when the tick occurs.
		 * Use rq->clock directly instead of rq_clock() since
		 * we do not have the rq lock and
		 * rq->clock was updated in the tick callpath.
		 */
		if (cmpxchg64(&tick_sched_clock, 0, rq->clock - 20000))
			return;
		for_each_possible_cpu(cpu) {
			struct scx_sched_rq_stats *srq = &per_cpu(scx_sched_rq_stats, cpu);
			srq->window_start = tick_sched_clock;
		}
		complete(&tick_sched_clock_completion);
	}
}


struct scx_task_iter {
	struct sched_ext_entity		cursor;
	struct task_struct		*locked;
	struct rq			*rq;
	struct rq_flags			rf;
};

/**
 * scx_task_iter_init - Initialize a task iterator
 * @iter: iterator to init
 *
 * Initialize @iter. Must be called with scx_tasks_lock held. Once initialized,
 * @iter must eventually be exited with scx_task_iter_exit().
 *
 * scx_tasks_lock may be released between this and the first next() call or
 * between any two next() calls. If scx_tasks_lock is released between two
 * next() calls, the caller is responsible for ensuring that the task being
 * iterated remains accessible either through RCU read lock or obtaining a
 * reference count.
 *
 * All tasks which existed when the iteration started are guaranteed to be
 * visited as long as they still exist.
 */
static void scx_task_iter_init(struct scx_task_iter *iter)
{
	lockdep_assert_held(&scx_tasks_lock);

	iter->cursor = (struct sched_ext_entity){ .flags = SCX_TASK_CURSOR };
	list_add(&iter->cursor.tasks_node, &scx_tasks);
	iter->locked = NULL;
}

/**
 * scx_task_iter_exit - Exit a task iterator
 * @iter: iterator to exit
 *
 * Exit a previously initialized @iter. Must be called with scx_tasks_lock held.
 * If the iterator holds a task's rq lock, that rq lock is released. See
 * scx_task_iter_init() for details.
 */
static void scx_task_iter_exit(struct scx_task_iter *iter)
{
	struct list_head *cursor = &iter->cursor.tasks_node;

	lockdep_assert_held(&scx_tasks_lock);

	if (iter->locked) {
		task_rq_unlock(iter->rq, iter->locked, &iter->rf);
		iter->locked = NULL;
	}

	if (list_empty(cursor))
		return;

	list_del_init(cursor);
}

/**
 * scx_task_iter_next - Next task
 * @iter: iterator to walk
 *
 * Visit the next task. See scx_task_iter_init() for details.
 */
static struct task_struct *scx_task_iter_next(struct scx_task_iter *iter)
{
	struct list_head *cursor = &iter->cursor.tasks_node;
	struct sched_ext_entity *pos;

	lockdep_assert_held(&scx_tasks_lock);

	list_for_each_entry(pos, cursor, tasks_node) {
		if (&pos->tasks_node == &scx_tasks)
			return NULL;
		if (!(pos->flags & SCX_TASK_CURSOR)) {
			list_move(cursor, &pos->tasks_node);
			return pos->task;
		}
	}

	/* can't happen, should always terminate at scx_tasks above */
	WARN_ON(1);
	return NULL;
}

/**
 * scx_task_iter_next_filtered - Next non-idle task
 * @iter: iterator to walk
 *
 * Visit the next non-idle task. See scx_task_iter_init() for details.
 */
static struct task_struct *
scx_task_iter_next_filtered(struct scx_task_iter *iter)
{
	struct task_struct *p;

	while ((p = scx_task_iter_next(iter))) {
		if (!is_idle_task(p))
			return p;
	}
	return NULL;
}

/**
 * scx_task_iter_next_filtered_locked - Next non-idle task with its rq locked
 * @iter: iterator to walk
 *
 * Visit the next non-idle task with its rq lock held. See scx_task_iter_init()
 * for details.
 */
static struct task_struct *
scx_task_iter_next_filtered_locked(struct scx_task_iter *iter)
{
	struct task_struct *p;

	if (iter->locked) {
		task_rq_unlock(iter->rq, iter->locked, &iter->rf);
		iter->locked = NULL;
	}

	p = scx_task_iter_next_filtered(iter);
	if (!p)
		return NULL;

	iter->rq = task_rq_lock(p, &iter->rf);
	iter->locked = p;
	return p;
}

static enum scx_ops_enable_state scx_ops_enable_state(void)
{
	return atomic_read(&scx_ops_enable_state_var);
}

static enum scx_ops_enable_state
scx_ops_set_enable_state(enum scx_ops_enable_state to)
{
	return atomic_xchg(&scx_ops_enable_state_var, to);
}

static bool scx_ops_tryset_enable_state(enum scx_ops_enable_state to,
					enum scx_ops_enable_state from)
{
	int from_v = from;

	return atomic_try_cmpxchg(&scx_ops_enable_state_var, &from_v, to);
}

static bool scx_ops_disabling(void)
{
	return false;
/*	return unlikely(scx_ops_enable_state() == SCX_OPS_DISABLING);*/
}

/**
 * wait_ops_state - Busy-wait the specified ops state to end
 * @p: target task
 * @opss: state to wait the end of
 *
 * Busy-wait for @p to transition out of @opss. This can only be used when the
 * state part of @opss is %SCX_QUEUEING or %SCX_DISPATCHING. This function also
 * has load_acquire semantics to ensure that the caller can see the updates made
 * in the enqueueing and dispatching paths.
 */
static void wait_ops_state(struct task_struct *p, u64 opss)
{
	do {
		cpu_relax();
	} while (atomic64_read_acquire(&p->scx->ops_state) == opss);
}


static void update_curr_scx(struct rq *rq)
{
	struct task_struct *curr = rq->curr;
	u64 now = rq_clock_task(rq);
	u64 delta_exec;

	if (time_before_eq64(now, curr->se.exec_start))
		return;

	delta_exec = now - curr->se.exec_start;
	curr->se.exec_start = now;
	update_runningtime(rq, curr, delta_exec);
	curr->se.sum_exec_runtime += delta_exec;
	account_group_exec_runtime(curr, delta_exec);
	cgroup_account_cputime(curr, delta_exec);

	if (curr->scx->slice != SCX_SLICE_INF)
		curr->scx->slice -= min(curr->scx->slice, delta_exec);

	trace_sched_stat_runtime(curr, delta_exec, 0);
}

static bool scx_dsq_priq_less(struct rb_node *node_a,
			      const struct rb_node *node_b)
{
	const struct sched_ext_entity *a =
		container_of(node_a, struct sched_ext_entity, dsq_node.priq);
	const struct sched_ext_entity *b =
		container_of(node_b, struct sched_ext_entity, dsq_node.priq);

	return time_before64(a->dsq_vtime, b->dsq_vtime);
}

static void dispatch_enqueue(struct scx_dispatch_q *dsq, struct task_struct *p,
			     u64 enq_flags)
{
	bool is_local = dsq->id == SCX_DSQ_LOCAL;
	unsigned long flags;

	WARN_ON_ONCE(p->scx->dsq || !list_empty(&p->scx->dsq_node.fifo));
	WARN_ON_ONCE((p->scx->dsq_flags & SCX_TASK_DSQ_ON_PRIQ) ||
		     !RB_EMPTY_NODE(&p->scx->dsq_node.priq));

	if (!is_local) {
		raw_spin_lock_irqsave(&dsq->lock, flags);
		if (unlikely(dsq->id == SCX_DSQ_INVALID)) {
			scx_ops_error("attempting to dispatch to a destroyed dsq");
			/* fall back to the global dsq */
			raw_spin_unlock_irqrestore(&dsq->lock, flags);
			dsq = &scx_dsq_global;
			raw_spin_lock_irqsave(&dsq->lock, flags);
		}
	}

	if (enq_flags & SCX_ENQ_DSQ_PRIQ) {
		p->scx->dsq_flags |= SCX_TASK_DSQ_ON_PRIQ;
		rb_add_cached(&p->scx->dsq_node.priq, &dsq->priq,
			      scx_dsq_priq_less);
	} else {
		if (enq_flags & (SCX_ENQ_HEAD | SCX_ENQ_PREEMPT))
			list_add(&p->scx->dsq_node.fifo, &dsq->fifo);
		else
			list_add_tail(&p->scx->dsq_node.fifo, &dsq->fifo);
	}
	dsq->nr++;
	p->scx->dsq = dsq;

	/*
	 * We're transitioning out of QUEUEING or DISPATCHING. store_release to
	 * match waiters' load_acquire.
	 */
	if (enq_flags & SCX_ENQ_CLEAR_OPSS)
		atomic64_set_release(&p->scx->ops_state, SCX_OPSS_NONE);

	if (is_local) {
		struct scx_rq *scx = container_of(dsq, struct scx_rq, local_dsq);
		struct rq *rq = scx->rq;
		bool preempt = false;

		if ((enq_flags & SCX_ENQ_PREEMPT) && p != rq->curr &&
		    rq->curr->sched_class == &ext_sched_class) {
			rq->curr->scx->slice = 0;
			preempt = true;
		}

		if (preempt || sched_class_above(&ext_sched_class,
						 rq->curr->sched_class))
			resched_curr(rq);
	} else {
		raw_spin_unlock_irqrestore(&dsq->lock, flags);
	}
}

static void task_unlink_from_dsq(struct task_struct *p,
				 struct scx_dispatch_q *dsq)
{
	if (p->scx->dsq_flags & SCX_TASK_DSQ_ON_PRIQ) {
		rb_erase_cached(&p->scx->dsq_node.priq, &dsq->priq);
		RB_CLEAR_NODE(&p->scx->dsq_node.priq);
		p->scx->dsq_flags &= ~SCX_TASK_DSQ_ON_PRIQ;
	} else {
		list_del_init(&p->scx->dsq_node.fifo);
	}
}

static bool task_linked_on_dsq(struct task_struct *p)
{
	return !list_empty(&p->scx->dsq_node.fifo) ||
		!RB_EMPTY_NODE(&p->scx->dsq_node.priq);
}

static void dispatch_dequeue(struct scx_rq *scx_rq, struct task_struct *p)
{
	unsigned long flags;
	struct scx_dispatch_q *dsq = p->scx->dsq;
	bool is_local = dsq == &scx_rq->local_dsq;

	if (!dsq) {
		WARN_ON_ONCE(task_linked_on_dsq(p));
		/*
		 * When dispatching directly from the BPF scheduler to a local
		 * DSQ, the task isn't associated with any DSQ but
		 * @p->scx->holding_cpu may be set under the protection of
		 * %SCX_OPSS_DISPATCHING.
		 */
		if (p->scx->holding_cpu >= 0)
			p->scx->holding_cpu = -1;
		return;
	}

	if (!is_local)
		raw_spin_lock_irqsave(&dsq->lock, flags);

	/*
	 * Now that we hold @dsq->lock, @p->holding_cpu and @p->scx->dsq_node
	 * can't change underneath us.
	*/
	if (p->scx->holding_cpu < 0) {
		/* @p must still be on @dsq, dequeue */
		WARN_ON_ONCE(!task_linked_on_dsq(p));
		task_unlink_from_dsq(p, dsq);
		dsq->nr--;
	} else {
		/*
		 * We're racing against dispatch_to_local_dsq() which already
		 * removed @p from @dsq and set @p->scx->holding_cpu. Clear the
		 * holding_cpu which tells dispatch_to_local_dsq() that it lost
		 * the race.
		 */
		WARN_ON_ONCE(task_linked_on_dsq(p));
		p->scx->holding_cpu = -1;
	}
	p->scx->dsq = NULL;

	if (!is_local)
		raw_spin_unlock_irqrestore(&dsq->lock, flags);
}


static bool test_rq_online(struct rq *rq)
{
#ifdef CONFIG_SMP
	return rq->online;
#else
	return true;
#endif
}


static void do_enqueue_task(struct rq *rq, struct task_struct *p, u64 enq_flags,
			    int sticky_cpu)
{
	struct scx_dispatch_q* d;
	WARN_ON_ONCE(!test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags));

	if (is_pcp_rt(p)) {
		/* Enqueue percpu rt task to local directly. */
		/* Or cause a bug when disable dispatch. */
		if (cpumask_test_cpu(cpu_of(rq), p->cpus_ptr))
			enq_flags |= SCX_ENQ_LOCAL;
	}

	if (test_bit(ffs(SCX_TASK_ENQ_LOCAL), (unsigned long*)&p->scx->flags)) {
		enq_flags |= SCX_ENQ_LOCAL;
		clear_bit(ffs(SCX_TASK_ENQ_LOCAL), (unsigned long*)&p->scx->flags);
	}
	/* rq migration */
	if (sticky_cpu == cpu_of(rq))
		goto local_norefill;
	/*
	 * If !rq->online, we already told the BPF scheduler that the CPU is
	 * offline. We're just trying to on/offline the CPU. Don't bother the
	 * BPF scheduler.
	 */
	if (unlikely(!test_rq_online(rq)))
		goto local;

	/* see %SCX_OPS_ENQ_EXITING */
	if (unlikely(p->flags & PF_EXITING))
		goto local;

	/* see %SCX_OPS_ENQ_LAST */
	if (enq_flags & SCX_ENQ_LAST)
		goto local;

	if (enq_flags & SCX_ENQ_LOCAL)
		goto local;
	else
		goto global;

local:
	/*
	 * For task-ordering, slice refill must be treated as implying the end
	 * of the current slice. Otherwise, the longer @p stays on the CPU, the
	 * higher priority it becomes from scx_prio_less()'s POV.
	 */
	p->scx->slice = SCX_SLICE_DFL;
local_norefill:
	dispatch_enqueue(&rq->scx->local_dsq, p, enq_flags);
	slim_stats_record(PCP_ENQL_CNT, 0, 0, cpu_of(rq));
	return;

global:
	d = find_dsq_from_task(p);
	if (d) {
		p->scx->slice = SCX_SLICE_DFL;
		dispatch_enqueue(d, p, enq_flags);
		return;
	}
	slim_stats_record(GLOBAL_STAT, 0, 0, 0);
	p->scx->slice = SCX_SLICE_DFL;
	dispatch_enqueue(&scx_dsq_global, p, enq_flags);
}

static bool watchdog_task_watched(const struct task_struct *p)
{
	return !list_empty(&p->scx->watchdog_node);
}

static void watchdog_watch_task(struct rq *rq, struct task_struct *p)
{
	lockdep_assert_rq_held(rq);
	if (test_bit(ffs(SCX_TASK_WATCHDOG_RESET), (unsigned long*)&p->scx->flags))
		p->scx->runnable_at = jiffies;
	clear_bit(ffs(SCX_TASK_WATCHDOG_RESET), (unsigned long*)&p->scx->flags);
	list_add_tail(&p->scx->watchdog_node, &rq->scx->watchdog_list);
}

static void watchdog_unwatch_task(struct task_struct *p, bool reset_timeout)
{
	list_del_init(&p->scx->watchdog_node);
	if (reset_timeout)
		set_bit(ffs(SCX_TASK_WATCHDOG_RESET), (unsigned long*)&p->scx->flags);
}

static void enqueue_task_scx(struct rq *rq, struct task_struct *p, int enq_flags)
{
	int sticky_cpu = p->scx->sticky_cpu;

	enq_flags |= rq->scx->extra_enq_flags;

	if (sticky_cpu >= 0)
		p->scx->sticky_cpu = -1;

	/*
	 * Restoring a running task will be immediately followed by
	 * set_next_task_scx() which expects the task to not be on the BPF
	 * scheduler as tasks can only start running through local DSQs. Force
	 * direct-dispatch into the local DSQ by setting the sticky_cpu.
	 */
	if (unlikely(enq_flags & ENQUEUE_RESTORE) && task_current(rq, p))
		sticky_cpu = cpu_of(rq);

	if (test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags)) {
		WARN_ON_ONCE(!watchdog_task_watched(p));
		return;
	}

	watchdog_watch_task(rq, p);
	set_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags);
	rq->scx->nr_running++;
	add_nr_running(rq, 1);

	do_enqueue_task(rq, p, enq_flags, sticky_cpu);
}

static void ops_dequeue(struct task_struct *p, u64 deq_flags)
{
	u64 opss;

	watchdog_unwatch_task(p, false);

	/* acquire ensures that we see the preceding updates on QUEUED */
	opss = atomic64_read_acquire(&p->scx->ops_state);

	switch (opss & SCX_OPSS_STATE_MASK) {
	case SCX_OPSS_NONE:
		break;
	case SCX_OPSS_QUEUEING:
		/*
		 * QUEUEING is started and finished while holding @p's rq lock.
		 * As we're holding the rq lock now, we shouldn't see QUEUEING.
		 */
		WARN_ON(1);
		break;
	case SCX_OPSS_QUEUED:
		if (atomic64_try_cmpxchg(&p->scx->ops_state, &opss,
					 SCX_OPSS_NONE))
			break;
		fallthrough;
	case SCX_OPSS_DISPATCHING:
		/*
		 * If @p is being dispatched from the BPF scheduler to a DSQ,
		 * wait for the transfer to complete so that @p doesn't get
		 * added to its DSQ after dequeueing is complete.
		 *
		 * As we're waiting on DISPATCHING with the rq locked, the
		 * dispatching side shouldn't try to lock the rq while
		 * DISPATCHING is set. See dispatch_to_local_dsq().
		 *
		 * DISPATCHING shouldn't have qseq set and control can reach
		 * here with NONE @opss from the above QUEUED case block.
		 * Explicitly wait on %SCX_OPSS_DISPATCHING instead of @opss.
		 */
		wait_ops_state(p, SCX_OPSS_DISPATCHING);
		WARN_ON(atomic64_read(&p->scx->ops_state) != SCX_OPSS_NONE);
		break;
	}
}

static void dequeue_task_scx(struct rq *rq, struct task_struct *p, int deq_flags)
{
	struct scx_rq *scx_rq = rq->scx;

	if (!test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags)) {
		WARN_ON_ONCE(watchdog_task_watched(p));
		return;
	}

	ops_dequeue(p, deq_flags);

	if(task_current(rq, p))
		trace_android_vh_hmbird_update_load(p, rq, PUT_PREV_TASK, rq->clock);

	if (deq_flags & SCX_DEQ_SLEEP)
		set_bit(ffs(SCX_TASK_DEQD_FOR_SLEEP), (unsigned long*)&p->scx->flags);
	else
		clear_bit(ffs(SCX_TASK_DEQD_FOR_SLEEP), (unsigned long*)&p->scx->flags);

	clear_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags);
	WARN_ON(!scx_rq->nr_running);
	scx_rq->nr_running--;
	sub_nr_running(rq, 1);
	dispatch_dequeue(scx_rq, p);
}

static void yield_task_scx(struct rq *rq)
{
	struct task_struct *p = rq->curr;

	p->scx->slice = 0;
}

static bool yield_to_task_scx(struct rq *rq, struct task_struct *to)
{
	return false;
}

#ifdef CONFIG_SMP
/**
 * move_task_to_local_dsq - Move a task from a different rq to a local DSQ
 * @rq: rq to move the task into, currently locked
 * @p: task to move
 * @enq_flags: %SCX_ENQ_*
 *
 * Move @p which is currently on a different rq to @rq's local DSQ. The caller
 * must:
 *
 * 1. Start with exclusive access to @p either through its DSQ lock or
 *    %SCX_OPSS_DISPATCHING flag.
 *
 * 2. Set @p->scx->holding_cpu to raw_smp_processor_id().
 *
 * 3. Remember task_rq(@p). Release the exclusive access so that we don't
 *    deadlock with dequeue.
 *
 * 4. Lock @rq and the task_rq from #3.
 *
 * 5. Call this function.
 *
 * Returns %true if @p was successfully moved. %false after racing dequeue and
 * losing.
 */
static bool move_task_to_local_dsq(struct rq *rq, struct task_struct *p,
				   u64 enq_flags)
{
	struct rq *task_rq;

	lockdep_assert_rq_held(rq);

	/*
	 * If dequeue got to @p while we were trying to lock both rq's, it'd
	 * have cleared @p->scx->holding_cpu to -1. While other cpus may have
	 * updated it to different values afterwards, as this operation can't be
	 * preempted or recurse, @p->scx->holding_cpu can never become
	 * raw_smp_processor_id() again before we're done. Thus, we can tell
	 * whether we lost to dequeue by testing whether @p->scx->holding_cpu is
	 * still raw_smp_processor_id().
	 *
	 * See dispatch_dequeue() for the counterpart.
	 */
	if (unlikely(p->scx->holding_cpu != raw_smp_processor_id()))
		return false;

	/* @p->rq couldn't have changed if we're still the holding cpu */
	task_rq = task_rq(p);
	lockdep_assert_rq_held(task_rq);
	deactivate_task(task_rq, p, 0);
	set_task_cpu(p, cpu_of(rq));
	p->scx->sticky_cpu = cpu_of(rq);

	/*
	 * We want to pass scx-specific enq_flags but activate_task() will
	 * truncate the upper 32 bit. As we own @rq, we can pass them through
	 * @rq->scx->extra_enq_flags instead.
	 */
	WARN_ON_ONCE(rq->scx->extra_enq_flags);
	rq->scx->extra_enq_flags = enq_flags;
	activate_task(rq, p, 0);
	rq->scx->extra_enq_flags = 0;

	return true;
}

#endif	/* CONFIG_SMP */

static int task_fits_cpu_scx(struct task_struct *p, int cpu)
{
	int fitable = 1;

	trace_android_vh_task_fits_cpu_scx(p, cpu, &fitable);

	return fitable;
}

static int check_misfit_task_on_little(struct task_struct *p, struct rq *rq, struct scx_dispatch_q *dsq)
{
	bool dsq_misfit;
	int cpu = cpu_of(rq);
	u64 task_util = 0;
	struct cluster_ctx ctx;
	int dsq_int = dsq_id_to_internal(dsq);

	trace_android_vh_get_util(-1, p, &task_util);
	gen_cluster_ctx(&ctx, BIG);
	dsq_misfit = (dsq_int >= SCHED_PROP_DEADLINE_LEVEL1 &&
				dsq_int <= SCHED_PROP_DEADLINE_LEVEL3);
#ifdef CLUSTER_SEPARATE
	dsq_misfit |= (dsq_int >= ctx.lower && dsq_int < ctx.upper);
#endif
	if (cpumask_test_cpu(cpu, iso_masks.little)
                && task_util > misfit_ds && dsq_misfit) {
                scx_info_trace("<hmbird_sched><filter>:task %s "
				"can't run on cpu%d, util = %llu\n",
				p->comm, cpu, task_util);
                return true;
        }
	return false;
}

static bool task_can_run_on_rq(struct task_struct *p, struct rq *rq, struct scx_dispatch_q *dsq)
{
	if (!task_fits_cpu_scx(p, cpu_of(rq))) {
		return false;
	}

	if (check_misfit_task_on_little(p, rq, dsq))
		return false;

	return likely(test_rq_online(rq)) && !is_migration_disabled(p);
}

bool consume_dispatch_q(struct rq *rq, struct rq_flags *rf,
			       struct scx_dispatch_q *dsq)
{
	struct scx_rq *scx_rq = rq->scx;
	struct sched_ext_entity *entity;
	struct task_struct *p;
	struct rb_node *rb_node;
	struct rq *task_rq;
	unsigned long flags;
	bool moved = false;

	slim_stats_record(TOTAL_DSP_CNT, 0, 0, 0);

retry:
	if (list_empty(&dsq->fifo) && !rb_first_cached(&dsq->priq))
		return false;

	raw_spin_lock_irqsave(&dsq->lock, flags);

	list_for_each_entry(entity, &dsq->fifo, dsq_node.fifo) {
		p = entity->task;
		task_rq = task_rq(p);
		if (rq == task_rq)
			goto this_rq;
		if (task_can_run_on_rq(p, rq, dsq))
			goto remote_rq;
	}

	for (rb_node = rb_first_cached(&dsq->priq); rb_node;
	     rb_node = rb_next(rb_node)) {
		entity = container_of(rb_node, struct sched_ext_entity, dsq_node.priq);
		p = entity->task;
		task_rq = task_rq(p);
		if (rq == task_rq)
			goto this_rq;
		if (task_can_run_on_rq(p, rq, dsq))
			goto remote_rq;
	}

	raw_spin_unlock_irqrestore(&dsq->lock, flags);
	return false;

this_rq:
	/* @dsq is locked and @p is on this rq */
	WARN_ON_ONCE(p->scx->holding_cpu >= 0);
	task_unlink_from_dsq(p, dsq);
	list_add_tail(&p->scx->dsq_node.fifo, &scx_rq->local_dsq.fifo);
	dsq->nr--;
	scx_rq->local_dsq.nr++;
	p->scx->dsq = &scx_rq->local_dsq;
	raw_spin_unlock_irqrestore(&dsq->lock, flags);
	return true;

remote_rq:
#ifdef CONFIG_SMP
	slim_stats_record(MOVE_RQ_CNT, 0, 0, 0);
	/*
	 * @dsq is locked and @p is on a remote rq. @p is currently protected by
	 * @dsq->lock. We want to pull @p to @rq but may deadlock if we grab
	 * @task_rq while holding @dsq and @rq locks. As dequeue can't drop the
	 * rq lock or fail, do a little dancing from our side. See
	 * move_task_to_local_dsq().
	 */
	WARN_ON_ONCE(p->scx->holding_cpu >= 0);
	task_unlink_from_dsq(p, dsq);
	dsq->nr--;
	p->scx->holding_cpu = raw_smp_processor_id();
	raw_spin_unlock_irqrestore(&dsq->lock, flags);

	rq_unpin_lock(rq, rf);
	double_lock_balance(rq, task_rq);
	rq_repin_lock(rq, rf);

	moved = move_task_to_local_dsq(rq, p, 0);

	double_unlock_balance(rq, task_rq);
#endif /* CONFIG_SMP */
	if (likely(moved))
		return true;
	goto retry;
}


static int balance_one(struct rq *rq, struct task_struct *prev,
		       struct rq_flags *rf, bool local)
{
	struct scx_rq *scx_rq = rq->scx;
	bool prev_on_scx = prev->sched_class == &ext_sched_class;

	lockdep_assert_rq_held(rq);

	if (static_branch_unlikely(&scx_ops_cpu_preempt) &&
	    unlikely(rq->scx->cpu_released)) {
		/*
		 * If the previous sched_class for the current CPU was not SCX,
		 * notify the BPF scheduler that it again has control of the
		 * core. This callback complements ->cpu_release(), which is
		 * emitted in scx_notify_pick_next_task().
		 */
		rq->scx->cpu_released = false;
	}

	if (prev_on_scx) {
		WARN_ON_ONCE(local && test_bit(ffs(SCX_TASK_BAL_KEEP), (unsigned long*)&prev->scx->flags));
		update_curr_scx(rq);

		/*
		 * If @prev is runnable & has slice left, it has priority and
		 * fetching more just increases latency for the fetched tasks.
		 * Tell put_prev_task_scx() to put @prev on local_dsq. If the
		 * BPF scheduler wants to handle this explicitly, it should
		 * implement ->cpu_released().
		 *
		 * See scx_ops_disable_workfn() for the explanation on the
		 * disabling() test.
		 *
		 * When balancing a remote CPU for core-sched, there won't be a
		 * following put_prev_task_scx() call and we don't own
		 * %SCX_TASK_BAL_KEEP. Instead, pick_task_scx() will test the
		 * same conditions later and pick @rq->curr accordingly.
		 */
		if (test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&prev->scx->flags) &&
		    prev->scx->slice && !scx_ops_disabling()) {
			if (local)
				set_bit(ffs(SCX_TASK_BAL_KEEP), (unsigned long*)&prev->scx->flags);
			return 1;
		}
	}
	/* if there already are tasks to run, nothing to do */
	if (scx_rq->local_dsq.nr)
		return 1;

	if (consume_dispatch_q(rq, rf, &scx_dsq_global)) {
		slim_stats_record(GLOBAL_STAT, 1, 0, 0);
		return 1;
	}

	if (consume_dispatch_global(rq, rf))
		return 1;

	return 0;
}

static int balance_scx(struct rq *rq, struct task_struct *prev,
		       struct rq_flags *rf)
{
	return balance_one(rq, prev, rf, true);
}

static void set_next_task_scx(struct rq *rq, struct task_struct *p, bool first)
{
	u64 util = 0;
	if (test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags)) {
		/*
		 * Core-sched might decide to execute @p before it is
		 * dispatched. Call ops_dequeue() to notify the BPF scheduler.
		 */
		ops_dequeue(p, SCX_DEQ_CORE_SCHED_EXEC);
		dispatch_dequeue(rq->scx, p);
	}

	p->se.exec_start = rq_clock_task(rq);

	if(test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags))
		trace_android_vh_hmbird_update_load(p, rq, PICK_NEXT_TASK, rq->clock);

	watchdog_unwatch_task(p, true);
	slim_trace_show_cpu_consume_dsq_idx(smp_processor_id(), p->scx->gdsq_idx);
	if (debug_enabled()) {
		trace_android_vh_get_util(-1, p, &util);
		scx_info_systrace("C|9999|cpu_%d_ds|%llu\n", cpu_of(rq), util);
	}
	/*
	 * @p is getting newly scheduled or got kicked after someone updated its
	 * slice. Refresh whether tick can be stopped. See can_stop_tick_scx().
	 */
	if ((p->scx->slice == SCX_SLICE_INF) !=
	    (bool)(rq->scx->flags & SCX_RQ_CAN_STOP_TICK)) {
		if (p->scx->slice == SCX_SLICE_INF)
			rq->scx->flags |= SCX_RQ_CAN_STOP_TICK;
		else
			rq->scx->flags &= ~SCX_RQ_CAN_STOP_TICK;

		sched_update_tick_dependency(rq);
	}
}

static void put_prev_task_scx(struct rq *rq, struct task_struct *p)
{
	update_curr_scx(rq);

	update_dispatch_dsq_info(rq, p);

	if(test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags))
		trace_android_vh_hmbird_update_load(p, rq, PUT_PREV_TASK, rq->clock);

	slim_trace_show_cpu_consume_dsq_idx(smp_processor_id(), 0);
	/*
	 * If we're being called from put_prev_task_balance(), balance_scx() may
	 * have decided that @p should keep running.
	 */
	if (test_bit(ffs(SCX_TASK_BAL_KEEP), (unsigned long*)&p->scx->flags)) {
		clear_bit(ffs(SCX_TASK_BAL_KEEP), (unsigned long*)&p->scx->flags);
		watchdog_watch_task(rq, p);
		/*
		 * Orig enq_flag is SCX_ENQ_HEAD, when we implement preempt, put
		 * a higher prio task A to local(assume curr is B), and then
		 * trigger a preempt event,
		 * if we use SCX_ENQ_HEAD, it will push curr(B) to the first one
		 * of local q, A is behind B after enqueue, cause next task will
		 * still be curr(B), so we set the enq flag to empty.
		 * for this problem, another way is to set curr->slice = 0, but
		 * it's unfair to curr task.
		 */
		dispatch_enqueue(&rq->scx->local_dsq, p, 0);
		return;
	}

	if (test_bit(ffs(SCX_TASK_QUEUED), (unsigned long*)&p->scx->flags)) {
		watchdog_watch_task(rq, p);

		if (sched_prop_get_top_thread_id(p)) {
			do_enqueue_task(rq, p, SCX_ENQ_LOCAL, -1);
			return;
		}

		/*
		 * If @p has slice left and balance_scx() didn't tag it for
		 * keeping, @p is getting preempted by a higher priority
		 * scheduler class or core-sched forcing a different task. Leave
		 * it at the head of the local DSQ.
		 */
		if (p->scx->slice && !scx_ops_disabling()) {
			dispatch_enqueue(&rq->scx->local_dsq, p, SCX_ENQ_HEAD);
			return;
		}

		/*
		 * If we're in the pick_next_task path, balance_scx() should
		 * have already populated the local DSQ if there are any other
		 * available tasks. If empty, tell ops.enqueue() that @p is the
		 * only one available for this cpu. ops.enqueue() should put it
		 * on the local DSQ so that the subsequent pick_next_task_scx()
		 * can find the task unless it wants to trigger a separate
		 * follow-up scheduling event.
		 */
		if (list_empty(&rq->scx->local_dsq.fifo)) {
			do_enqueue_task(rq, p, SCX_ENQ_LAST | SCX_ENQ_LOCAL, -1);
		} else
			do_enqueue_task(rq, p, 0, -1);
	}
}

static struct task_struct *first_local_task(struct rq *rq)
{
	struct rb_node *rb_node;
	struct sched_ext_entity *entity;

	if (!list_empty(&rq->scx->local_dsq.fifo)) {
		entity = list_first_entry(&rq->scx->local_dsq.fifo, struct sched_ext_entity, dsq_node.fifo);
		return entity->task;
	}

	rb_node = rb_first_cached(&rq->scx->local_dsq.priq);
	if (rb_node) {
		entity = container_of(rb_node, struct sched_ext_entity, dsq_node.priq);
		return entity->task;
	}
	return NULL;
}

static struct task_struct *pick_next_task_scx(struct rq *rq)
{
	struct task_struct *p;

	p = first_local_task(rq);
	if (!p)
		return NULL;

	if (unlikely(!p->scx->slice)) {
		if (!scx_ops_disabling() && !scx_warned_zero_slice) {
			printk_deferred(KERN_WARNING "sched_ext: %s[%d] has zero slice in pick_next_task_scx()\n",
					p->comm, p->pid);
			scx_warned_zero_slice = true;
		}
		p->scx->slice = SCX_SLICE_DFL;
	}

	set_next_task_scx(rq, p, true);

	return p;
}

void __scx_notify_pick_next_task(struct rq *rq, struct task_struct *task,
				 const struct sched_class *active)
{
	lockdep_assert_rq_held(rq);

	/*
	 * The callback is conceptually meant to convey that the CPU is no
	 * longer under the control of SCX. Therefore, don't invoke the
	 * callback if the CPU is is staying on SCX, or going idle (in which
	 * case the SCX scheduler has actively decided not to schedule any
	 * tasks on the CPU).
	 */
	if (likely(active >= &ext_sched_class))
		return;

	/*
	 * At this point we know that SCX was preempted by a higher priority
	 * sched_class, so invoke the ->cpu_release() callback if we have not
	 * done so already. We only send the callback once between SCX being
	 * preempted, and it regaining control of the CPU.
	 *
	 * ->cpu_release() complements ->cpu_acquire(), which is emitted the
	 *  next time that balance_scx() is invoked.
	 */
	if (!rq->scx->cpu_released)
		rq->scx->cpu_released = true;
}

#ifdef CONFIG_SMP

static bool test_and_clear_cpu_idle(int cpu)
{
	if (cpumask_test_and_clear_cpu(cpu, idle_masks.cpu)) {
		if (cpumask_empty(idle_masks.cpu))
			scx_has_idle_cpus = false;
		return true;
	} else {
		return false;
	}
}

static s32 scx_pick_idle_cpu(const struct cpumask *cpus_allowed)
{
	int cpu;

	do {
		cpu = cpumask_any_and_distribute(idle_masks.cpu, cpus_allowed);
		if (cpu >= nr_cpu_ids)
			return -EBUSY;
	} while (!test_and_clear_cpu_idle(cpu));

	return cpu;
}


static bool prev_cpu_misfit(int prev)
{
	if (!is_partial_enabled() && is_partial_cpu(prev))
		return true;

	return false;
}

static int heavy_rt_placement(struct task_struct *p, int prev)
{
	struct cpumask tmp;
	int cpu;
	u16 ds = p->scx->sts.demand_scaled;
	if (!rt_prio(p->prio))
		return -EFAULT;

	if ((ds < misfit_ds))
		return -EFAULT;

	if (is_partial_enabled())
		cpumask_or(&tmp, iso_masks.big, iso_masks.partial);
	else
		cpumask_copy(&tmp, iso_masks.big);

	cpu = scx_pick_idle_cpu(&tmp);
	if (cpu >= 0)
                return cpu;

	if (cpumask_test_cpu(prev, iso_masks.big) ||
		(is_partial_enabled() && is_partial_cpu(prev)))
		return prev;

	return cpumask_first(&tmp);
}

static int spec_task_before_pick_idle(struct task_struct *p, int prev)
{
	int cpu;

	cpu = heavy_rt_placement(p, prev);
	if (cpu >= 0)
		return cpu;
	return -EFAULT;
}

static int repick_fallback_cpu(void)
{
	/*
	 * partial cpu follow big cluster's Scheduling policy,
	 * simply return first bit cpu.
	 */
	return cpumask_first(iso_masks.big);
}

static s32 scx_select_cpu_dfl(struct task_struct *p, s32 prev_cpu, u64 wake_flags)
{
	s32 cpu = -1;

	trace_android_vh_scx_select_cpu_dfl(p, &cpu);
	if (cpu >= 0) {
		set_bit(ffs(SCX_TASK_ENQ_LOCAL), (unsigned long*)&p->scx->flags);
		return cpu;
	}

	if (sched_prop_get_top_thread_id(p)) {
		set_bit(ffs(SCX_TASK_ENQ_LOCAL), (unsigned long*)&p->scx->flags);
		cpu = scx_pick_idle_cpu(iso_masks.big);
		if (cpu >= 0)
			return cpu;
		if (cpumask_test_cpu(prev_cpu, iso_masks.big))
			return prev_cpu;
		else
			return cpumask_first(iso_masks.big);
	}

	if (p->nr_cpus_allowed == 1) {
		cpu = cpumask_any(p->cpus_ptr);
                return cpu;
        }

	partial_dynamic_ctrl();

	cpu = spec_task_before_pick_idle(p, prev_cpu);
	if (cpu >= 0) {
		scx_info_trace("<hmbird_sched><select_cpu>:heavy task %s pick cpu %d\n", p->comm, cpu);
		return cpu;
	}

	/* if the previous CPU is idle, dispatch directly to it */
	if (test_and_clear_cpu_idle(prev_cpu))
		return prev_cpu;

	cpu = scx_pick_idle_cpu(cpu_possible_mask);
	if (cpu >= 0)
		return cpu;

	if (prev_cpu_misfit(prev_cpu)) {
		scx_info_trace("<hmbird_sched><select_cpu>:prev cpu%d is misfit for %s, repick cpu \n", cpu, p->comm);
		return repick_fallback_cpu();
	}

	return prev_cpu;
}

static int select_task_rq_scx(struct task_struct *p, int prev_cpu, int wake_flags)
{
	return scx_select_cpu_dfl(p, prev_cpu, wake_flags);
}

static void set_cpus_allowed_scx(struct task_struct *p,
				 const struct cpumask *new_mask,
				u32 flags)
{
	set_cpus_allowed_common(p, new_mask, flags);
}

static void reset_idle_masks(void)
{
	cpumask_or(idle_masks.cpu, idle_masks.cpu, iso_masks.little);
	cpumask_or(idle_masks.cpu, idle_masks.cpu, iso_masks.big);
	if (is_partial_enabled())
		cpumask_or(idle_masks.cpu, idle_masks.cpu, iso_masks.partial);
	scx_has_idle_cpus = true;
}

void __scx_update_idle(struct rq *rq, bool idle)
{
	int cpu = cpu_of(rq);
	struct cpumask *sib_mask = topology_sibling_cpumask(cpu);

	if (skip_update_idle())
		return;

	if (idle) {
		cpumask_set_cpu(cpu, idle_masks.cpu);
		if (!scx_has_idle_cpus)
			scx_has_idle_cpus = true;

		/*
		 * idle_masks.smt handling is racy but that's fine as it's only
		 * for optimization and self-correcting.
		 */
		for_each_cpu(cpu, sib_mask) {
			if (!cpumask_test_cpu(cpu, idle_masks.cpu))
				return;
		}
		cpumask_or(idle_masks.smt, idle_masks.smt, sib_mask);
	} else {
		cpumask_clear_cpu(cpu, idle_masks.cpu);
		if (scx_has_idle_cpus && cpumask_empty(idle_masks.cpu))
			scx_has_idle_cpus = false;

		cpumask_andnot(idle_masks.smt, idle_masks.smt, sib_mask);
	}
}

#else /* !CONFIG_SMP */

static bool test_and_clear_cpu_idle(int cpu) { return false; }
static s32 scx_pick_idle_cpu(const struct cpumask *cpus_allowed) { return -EBUSY; }
static void reset_idle_masks(void) {}

#endif /* CONFIG_SMP */

static bool check_rq_for_timeouts(struct rq *rq)
{
	struct sched_ext_entity *entity;
	struct task_struct *p;
	struct rq_flags rf;
	bool timed_out = false;
	struct task_struct *tp;
	int total_count = 0;
	int lvl_cnt[4] = {0};
	int sum = 0, i;

	rq_lock_irqsave(rq, &rf);
	list_for_each_entry(entity, &rq->scx->watchdog_list, watchdog_node) {
		unsigned long last_runnable;

		p = entity->task;
		last_runnable = p->scx->runnable_at;

		if (unlikely(time_after(jiffies,
					last_runnable + scx_watchdog_timeout))) {
			u32 dur_ms = jiffies_to_msecs(jiffies - last_runnable);

			scx_ops_error_type(SCX_EXIT_ERROR_STALL,
					   "%s[%d] failed to run for %u.%03us,"
						"p->scx->dsq->id = %llu,"
						"p->cpumask = %*pb",
					   	p->comm, p->pid,
					   	dur_ms / 1000, dur_ms % 1000,
						p->scx->dsq ? p->scx->dsq->id : 0,
						cpumask_pr_args(&p->cpus_mask));
			timed_out = true;
			tp = p;
			break;
		}
	}
	rq_unlock_irqrestore(rq, &rf);

	if (timed_out) {
		rq_lock_irqsave(rq, &rf);
		list_for_each_entry(entity, &rq->scx->watchdog_list, watchdog_node) {
			unsigned long last_runnable;

			p = entity->task;
			last_runnable = p->scx->runnable_at;

			total_count++;
			if (time_before(jiffies, last_runnable + 5 * HZ)) {
				lvl_cnt[0]++;
			} else if (time_before(jiffies, last_runnable + 15 * HZ)) {
				lvl_cnt[1]++;
			} else if (time_before(jiffies, last_runnable + 30 * HZ)) {
				lvl_cnt[2]++;
			} else {
				lvl_cnt[3]++;
			}
		}
		pr_err("total watchdog_list count = %d\n", total_count);
		pr_err("lvl_cnt = %d, %d, %d, %d\n", lvl_cnt[0], lvl_cnt[1],
							lvl_cnt[2], lvl_cnt[3]);
		pr_err("task:%s, dsq_id = %llu, mask = %*pb\n",
				tp->comm, tp->scx->dsq ? tp->scx->dsq->id : 0,
				cpumask_pr_args(&tp->cpus_mask));
		pr_err("nr_running = %u\n", rq->scx->nr_running);
		for_each_online_cpu(i)
			sum += cpu_rq(i)->nr_running;
		pr_err("sum = %d\n", sum);

		rq_unlock_irqrestore(rq, &rf);
	}
	return timed_out;
}

static void scx_watchdog_workfn(struct work_struct *work)
{
	int cpu;

	scx_watchdog_timestamp = jiffies;

	for_each_online_cpu(cpu) {
		if (unlikely(check_rq_for_timeouts(cpu_rq(cpu))))
			break;

		cond_resched();
	}
	queue_delayed_work(system_unbound_wq, to_delayed_work(work),
			   scx_watchdog_timeout / 2);
}

static void set_pcp_round(struct rq *rq)
{
	int cpu = cpu_of(rq);

	if (atomic64_read(&pcp_dsq_round) != per_cpu(pcp_info, cpu).pcp_seq) {
		per_cpu(pcp_info, cpu).pcp_seq = atomic64_read(&pcp_dsq_round);
		per_cpu(pcp_info, cpu).pcp_round = true;
		scx_info_systrace("C|9999|pcp_%d_round|%d\n", cpu, true);
		per_cpu(pcp_info, cpu).rtime = 0;
		systrace_output_rtime_state(&per_cpu(pcp_ldsq, cpu),
                                        per_cpu(pcp_info, cpu).rtime);
	}
}


/*
 * Just for debug: output ext on/off state per 10s.
 */
#define OUTPUT_INTVAL	(msecs_to_jiffies(10 * 1000))
static void inform_ext_onoff_from_systrace(void)
{
	static unsigned long __read_mostly next_print;

	if (time_before(jiffies, next_print))
		return;

	next_print = jiffies + OUTPUT_INTVAL;
	scx_internal_systrace("C|9999|scx_status|%d\n", curr_ss);
}

static void task_tick_scx(struct rq *rq, struct task_struct *curr, int queued)
{
	update_curr_scx(rq);

	set_pcp_round(rq);

	trace_android_vh_hmbird_update_load(curr, rq, TASK_UPDATE, rq->clock);
	/*
	 * While disabling, always resched and refresh core-sched timestamp as
	 * we can't trust the slice management or ops.core_sched_before().
	 */
	if (scx_ops_disabling())
		curr->scx->slice = 0;

	if (!curr->scx->slice)
		resched_curr(rq);

	inform_ext_onoff_from_systrace();
}

static int scx_ops_prepare_task(struct task_struct *p, struct task_group *tg)
{
	WARN_ON_ONCE(test_bit(ffs(SCX_TASK_OPS_PREPPED), (unsigned long*)&p->scx->flags));

	p->scx->disallow = false;

	trace_android_vh_hmbird_init_task(p);

	set_bit(ffs(SCX_TASK_OPS_PREPPED), (unsigned long*)&p->scx->flags);
	set_bit(ffs(SCX_TASK_WATCHDOG_RESET), (unsigned long*)&p->scx->flags);
	return 0;
}

static void scx_ops_enable_task(struct task_struct *p)
{
	lockdep_assert_rq_held(task_rq(p));
	WARN_ON_ONCE(!test_bit(ffs(SCX_TASK_OPS_PREPPED), (unsigned long*)&p->scx->flags));

	clear_bit(ffs(SCX_TASK_OPS_PREPPED), (unsigned long*)&p->scx->flags);
	set_bit(ffs(SCX_TASK_OPS_ENABLED), (unsigned long*)&p->scx->flags);
}

static void scx_ops_disable_task(struct task_struct *p)
{
	lockdep_assert_rq_held(task_rq(p));

	if (test_bit(ffs(SCX_TASK_OPS_PREPPED), (unsigned long*)&p->scx->flags)) {
		clear_bit(ffs(SCX_TASK_OPS_PREPPED), (unsigned long*)&p->scx->flags);
	} else if (test_bit(ffs(SCX_TASK_OPS_ENABLED), (unsigned long*)&p->scx->flags)) {
		clear_bit(ffs(SCX_TASK_OPS_ENABLED), (unsigned long*)&p->scx->flags);
	}
}

static void set_task_scx_weight(struct task_struct *p)
{
	u32 weight = sched_prio_to_weight[p->static_prio - MAX_RT_PRIO];

	p->scx->weight = sched_weight_to_cgroup(weight);
}

/**
 * refresh_scx_weight - Refresh a task's ext weight
 * @p: task to refresh ext weight for
 *
 * @p->scx->weight carries the task's static priority in cgroup weight scale to
 * enable easy access from the BPF scheduler. To keep it synchronized with the
 * current task priority, this function should be called when a new task is
 * created, priority is changed for a task on sched_ext, and a task is switched
 * to sched_ext from other classes.
 */
static void refresh_scx_weight(struct task_struct *p)
{
	lockdep_assert_rq_held(task_rq(p));
	set_task_scx_weight(p);
}

int scx_pre_fork(struct task_struct *p)
{
	int ret = 0;

	p->scx = kmalloc(sizeof(struct sched_ext_entity), GFP_KERNEL);
	if (!p->scx) {
		ret = -1;
		goto lock;
	}

	p->scx->dsq              = NULL;
	INIT_LIST_HEAD(&p->scx->dsq_node.fifo);
	RB_CLEAR_NODE(&p->scx->dsq_node.priq);
	INIT_LIST_HEAD(&p->scx->watchdog_node);
	p->scx->flags            = 0;
	p->scx->weight           = 0;
	p->scx->sticky_cpu       = -1;
	p->scx->holding_cpu      = -1;
	p->scx->kf_mask          = 0;
	atomic64_set(&p->scx->ops_state, 0);
	p->scx->runnable_at      = INITIAL_JIFFIES;
	p->scx->slice            = SCX_SLICE_DFL;
	p->scx->task             = p;
	sched_set_sched_prop(p, 0);

	/*
	 * BPF scheduler enable/disable paths want to be able to iterate and
	 * update all tasks which can become complex when racing forks. As
	 * enable/disable are very cold paths, let's use a percpu_rwsem to
	 * exclude forks.
	 */
lock:
	percpu_down_read(&scx_fork_rwsem);

	return ret;
}

int scx_fork(struct task_struct *p)
{
	percpu_rwsem_assert_held(&scx_fork_rwsem);

	if (scx_enabled())
		return scx_ops_prepare_task(p, task_group(p));
	else
		return 0;
}

void scx_post_fork(struct task_struct *p)
{
	if (scx_enabled()) {
		struct rq_flags rf;
		struct rq *rq;

		rq = task_rq_lock(p, &rf);
		/*
		 * Set the weight manually before calling ops.enable() so that
		 * the scheduler doesn't see a stale value if they inspect the
		 * task struct. We'll invoke ops.set_weight() afterwards, as it
		 * would be odd to receive a callback on the task before we
		 * tell the scheduler that it's been fully enabled.
		 */
		set_task_scx_weight(p);
		scx_ops_enable_task(p);
		refresh_scx_weight(p);
		task_rq_unlock(rq, p, &rf);
	}

	spin_lock_irq(&scx_tasks_lock);
	list_add_tail(&p->scx->tasks_node, &scx_tasks);
	spin_unlock_irq(&scx_tasks_lock);

	percpu_up_read(&scx_fork_rwsem);
}

void scx_cancel_fork(struct task_struct *p)
{
	if (scx_enabled())
		scx_ops_disable_task(p);

	if (p->scx) {
		kfree(p->scx);
		p->scx = NULL;
	}
	percpu_up_read(&scx_fork_rwsem);
}

void sched_ext_free(struct task_struct *p)
{
	unsigned long flags;

	spin_lock_irqsave(&scx_tasks_lock, flags);
	list_del_init(&p->scx->tasks_node);
	spin_unlock_irqrestore(&scx_tasks_lock, flags);

	/*
	 * @p is off scx_tasks and wholly ours. scx_ops_enable()'s PREPPED ->
	 * ENABLED transitions can't race us. Disable ops for @p.
	 */
	if (test_bit(ffs(SCX_TASK_OPS_PREPPED), (unsigned long*)&p->scx->flags) ||
		test_bit(ffs(SCX_TASK_OPS_ENABLED), (unsigned long*)&p->scx->flags)) {
		struct rq_flags rf;
		struct rq *rq;

		rq = task_rq_lock(p, &rf);
		scx_ops_disable_task(p);
		task_rq_unlock(rq, p, &rf);
	}
	if (p->scx) {
		kfree(p->scx);
		p->scx = NULL;
	}
}

static void prio_changed_scx(struct rq *rq, struct task_struct *p, int oldprio)
{
}


static void check_preempt_curr_scx(struct rq *rq, struct task_struct *p, int wake_flags)
{
	int check_result = -1;
	enum cpu_type type;
	int sp_dl;

	trace_android_vh_check_preempt_curr_scx(rq, p, wake_flags, &check_result);

	if (check_result > 0) {
		goto preempt;
	}

	sp_dl = p->scx->sched_prop & SCHED_PROP_DEADLINE_MASK;
	if (!sp_dl || (sp_dl >= SCHED_PROP_DEADLINE_LEVEL3))
		return;

	type = cpu_cluster(cpu_of(rq));
	if (type == EXCLUSIVE || ((type == PARTIAL) && !is_partial_enabled()))
		return;

	if (rq->curr->prio > p->prio)
		goto preempt;

	return;
preempt:
	resched_curr(rq);
}
static void switched_to_scx(struct rq *rq, struct task_struct *p) {}

int scx_check_setscheduler(struct task_struct *p, int policy)
{
	lockdep_assert_rq_held(task_rq(p));

	/* if disallow, reject transitioning into SCX */
	if (scx_enabled() && READ_ONCE(p->scx->disallow) &&
	    p->policy != policy && policy == SCHED_EXT)
		return -EACCES;

	return 0;
}

#ifdef CONFIG_NO_HZ_FULL
bool scx_can_stop_tick(struct rq *rq)
{
	struct task_struct *p = rq->curr;

	if (scx_ops_disabling())
		return false;

	if (p->sched_class != &ext_sched_class)
		return true;

	/*
	 * @rq can dispatch from different DSQs, so we can't tell whether it
	 * needs the tick or not by looking at nr_running. Allow stopping ticks
	 * iff the BPF scheduler indicated so. See set_next_task_scx().
	 */
	return rq->scx->flags & SCX_RQ_CAN_STOP_TICK;
}
#endif

int scx_tg_online(struct task_group *tg)
{
	tg->scx_deadline_idx = -1;
	return 0;
}

/*
 * Omitted operations:
 *
 * - check_preempt_curr: NOOP as it isn't useful in the wakeup path because the
 *   task isn't tied to the CPU at that point. Preemption is implemented by
 *   resetting the victim task's slice to 0 and triggering reschedule on the
 *   target CPU.
 *
 * - migrate_task_rq: Unncessary as task to cpu mapping is transient.
 *
 * - task_fork/dead: We need fork/dead notifications for all tasks regardless of
 *   their current sched_class. Call them directly from sched core instead.
 *
 * - task_woken, switched_from: Unnecessary.
 */
DEFINE_SCHED_CLASS(ext) = {
	.enqueue_task		= enqueue_task_scx,
	.dequeue_task		= dequeue_task_scx,
	.yield_task		= yield_task_scx,
	.yield_to_task		= yield_to_task_scx,

	.check_preempt_curr	= check_preempt_curr_scx,

	.pick_next_task		= pick_next_task_scx,

	.put_prev_task		= put_prev_task_scx,
	.set_next_task          = set_next_task_scx,

#ifdef CONFIG_SMP
	.balance		= balance_scx,
	.select_task_rq		= select_task_rq_scx,
	.set_cpus_allowed	= set_cpus_allowed_scx,
#endif
	.task_tick		= task_tick_scx,

	.switched_to		= switched_to_scx,
	.prio_changed		= prio_changed_scx,

	.update_curr		= update_curr_scx,

#ifdef CONFIG_UCLAMP_TASK
	.uclamp_enabled		= 0,
#endif
};

/*
 * Must with rq lock held.
 */
bool task_is_scx(struct task_struct *p)
{
	return p->sched_class == &ext_sched_class;
}
EXPORT_SYMBOL_GPL(task_is_scx);


void init_dsq(struct scx_dispatch_q *dsq, u64 dsq_id)
{
	memset(dsq, 0, sizeof(*dsq));

	raw_spin_lock_init(&dsq->lock);
	INIT_LIST_HEAD(&dsq->fifo);
	dsq->id = dsq_id;
}

static int scx_cgroup_init(void)
{
	struct cgroup_subsys_state *css;

	css_for_each_descendant_pre(css, &root_task_group.css) {
		struct task_group *tg = css_tg(css);
		cgrp_dsq_idx_init(css->cgroup, tg);
	}
	return 0;
}

/*
 * Used by sched_fork() and __setscheduler_prio() to pick the matching
 * sched_class. dl/rt are already handled.
 */
bool task_on_scx(struct task_struct *p)
{
	return scx_enabled();
}

static void update_load_enable(bool enable)
{
	static bool prev = false;

	if (prev == enable)
		return;

	prev = enable;
	WRITE_ONCE(tick_sched_clock, 0);
	if (enable)
		atomic_set_release(&set_sched_clock_prepare, true);
	trace_android_vh_hmbird_update_load_enable(enable);
}

static void __setscheduler_prio(struct task_struct *p, int prio)
{
	bool on_scx = task_on_scx(p);

	if (p->sched_class == &stop_sched_class) {}
	else if (dl_prio(prio))
		p->sched_class = &dl_sched_class;
	else if (on_scx && rt_prio(prio))
		p->sched_class = &ext_sched_class;
	else if (rt_prio(prio))
		p->sched_class = &rt_sched_class;
	else if (on_scx)
		p->sched_class = &ext_sched_class;
	else
		p->sched_class = &fair_sched_class;

	p->prio = prio;
}

static void scx_switch_log(enum switch_stat ss, bool finish, bool enable)
{
	char *s1 = finish ? "finished" : "failed";
	char *s2 = enable ? "enabled" : "disabled";
	scx_internal_systrace("C|9999|scx_status|%d\n", ss);

	if (ss == SCX_DISABLED || ss == SCX_ENABLED)
		scx_debug("<hmbird_sched><switch>: ext %s %s at "
				"jiffies = %lu, clock = %lu\n",
				s2, s1, jiffies, (unsigned long)sched_clock());
	curr_ss = ss;
}

static void scx_ops_disable_workfn(struct kthread_work *work)
{
	struct scx_task_iter sti;
	struct task_struct *p;
	int cpu;

	scx_switch_log(SCX_SWITCH_PREP, 0, 0);
	cancel_delayed_work_sync(&scx_watchdog_work);

	switch (scx_ops_set_enable_state(SCX_OPS_DISABLING)) {
	case SCX_OPS_DISABLED:
		WARN_ON_ONCE(scx_ops_set_enable_state(SCX_OPS_DISABLED) !=
			     SCX_OPS_DISABLING);
		scx_switch_log(SCX_ENABLED, 0, 0);
		return;
	case SCX_OPS_PREPPING:
		goto forward_progress_guaranteed;
	case SCX_OPS_DISABLING:
		/* shouldn't happen but handle it like ENABLING if it does */
		WARN_ONCE(true, "sched_ext: duplicate disabling instance?");
		fallthrough;
	case SCX_OPS_ENABLING:
	case SCX_OPS_ENABLED:
		break;
	}

/*
	spin_lock_irq(&scx_tasks_lock);
	scx_task_iter_init(&sti);
	while ((p = scx_task_iter_next_filtered_locked(&sti))) {
		if (READ_ONCE(p->__state) != TASK_DEAD) {
			SCHED_CHANGE_BLOCK(task_rq(p), p,
					   DEQUEUE_SAVE | DEQUEUE_MOVE) {
			}
		}
	}
	scx_task_iter_exit(&sti);
	spin_unlock_irq(&scx_tasks_lock);
*/

	/* kick all CPUs to restore ticks */
	for_each_possible_cpu(cpu)
		resched_cpu(cpu);

forward_progress_guaranteed:
	/*
	 * Here, every runnable task is guaranteed to make forward progress and
	 * we can safely use blocking synchronization constructs. Actually
	 * disable ops.
	 */
	mutex_lock(&scx_ops_enable_mutex);


	/* avoid racing against fork and cgroup changes */
	cpus_read_lock();
	percpu_down_write(&scx_fork_rwsem);

	scx_switch_log(SCX_RQ_SWITCH_BEGIN, 0, 0);
	spin_lock_irq(&scx_tasks_lock);
	atomic_set(&__scx_ops_enabled, false);
	scx_task_iter_init(&sti);
	while ((p = scx_task_iter_next_filtered_locked(&sti))) {
		const struct sched_class *old_class = p->sched_class;
		struct rq *rq = task_rq(p);
		bool alive = READ_ONCE(p->__state) != TASK_DEAD;

		update_rq_clock(rq);

		SCHED_CHANGE_BLOCK(rq, p, DEQUEUE_SAVE | DEQUEUE_MOVE |
				   DEQUEUE_NOCLOCK) {
			p->scx->slice = min_t(u64, p->scx->slice, SCX_SLICE_DFL);

			__setscheduler_prio(p, p->prio);
		}

		if (alive)
			check_class_changed(task_rq(p), p, old_class, p->prio);

		scx_ops_disable_task(p);
	}
	scx_task_iter_exit(&sti);
	spin_unlock_irq(&scx_tasks_lock);
	scx_switch_log(SCX_RQ_SWITCH_DONE, 0, 0);

	atomic_set(&non_ext_task, true);
	/* no task is on scx, turn off all the switches and flush in-progress calls */
	static_branch_disable_cpuslocked(&scx_ops_cpu_preempt);
	synchronize_rcu();

	percpu_up_write(&scx_fork_rwsem);
	cpus_read_unlock();

	update_load_enable(false);

	mutex_unlock(&scx_ops_enable_mutex);

	WARN_ON_ONCE(scx_ops_set_enable_state(SCX_OPS_DISABLED) !=
		     SCX_OPS_DISABLING);
	scx_switch_log(SCX_DISABLED, 1, 0);
}

static DEFINE_KTHREAD_WORK(scx_ops_disable_work, scx_ops_disable_workfn);

static void schedule_scx_ops_disable_work(void)
{
	struct kthread_worker *helper = READ_ONCE(scx_ops_helper);

	/*
	 * We may be called spuriously before the first bpf_sched_ext_reg(). If
	 * scx_ops_helper isn't set up yet, there's nothing to do.
	 */
	if (helper)
		kthread_queue_work(helper, &scx_ops_disable_work);
}

static void scx_ops_disable(enum scx_exit_type type)
{
	if (WARN_ON_ONCE(type == SCX_EXIT_NONE || type == SCX_EXIT_DONE))
		type = SCX_EXIT_ERROR;

	schedule_scx_ops_disable_work();
}

static void scx_ops_error_irq_workfn(struct irq_work *irq_work)
{
	schedule_scx_ops_disable_work();
}

static DEFINE_IRQ_WORK(scx_ops_error_irq_work, scx_ops_error_irq_workfn);

__printf(2, 3) void scx_ops_error_type(enum scx_exit_type type,
				       const char *fmt, ...)
{
	irq_work_queue(&scx_ops_error_irq_work);
}

static struct kthread_worker *scx_create_rt_helper(const char *name)
{
	struct kthread_worker *helper;

	helper = kthread_create_worker(0, name);
	if (helper)
		sched_set_fifo(helper->task);
	return helper;
}

static inline void set_audio_thread_sched_prop(struct task_struct *p)
{
	struct cgroup_subsys_state *css;

	if (likely(p->prio >= MAX_RT_PRIO))
		return;

	rcu_read_lock();
	css = task_css(p, cpuset_cgrp_id);
	if (!css) {
		rcu_read_unlock();
		return;
	}
	rcu_read_unlock();

	if (!strcmp(css->cgroup->kn->name, "audio-app"))
		sched_set_sched_prop(p, SCHED_PROP_DEADLINE_LEVEL1);

	return;
}

static int scx_ops_enable(void *unused)
{
	struct scx_task_iter sti;
	struct task_struct *p;
	int ret;
	int tcnt = 0;
	unsigned long long start = 0;

	scx_switch_log(SCX_SWITCH_PREP, 0, 0);
	mutex_lock(&scx_ops_enable_mutex);

	if (!scx_ops_helper) {
		WRITE_ONCE(scx_ops_helper,
			   scx_create_rt_helper("sched_ext_ops_helper"));
		if (!scx_ops_helper) {
			ret = -ENOMEM;
			goto err_unlock;
		}
	}

	if (scx_ops_enable_state() != SCX_OPS_DISABLED) {
		ret = -EBUSY;
		goto err_unlock;
	}

	update_load_enable(true);

	WARN_ON_ONCE(scx_ops_set_enable_state(SCX_OPS_PREPPING) !=
		     SCX_OPS_DISABLED);

	scx_warned_zero_slice = false;

	atomic64_set(&scx_nr_rejected, 0);

	/*
	 * Keep CPUs stable during enable so that the BPF scheduler can track
	 * online CPUs by watching ->on/offline_cpu() after ->init().
	 */
	cpus_read_lock();

	scx_watchdog_timeout = SCX_WATCHDOG_MAX_TIMEOUT;

	scx_watchdog_timestamp = jiffies;
	queue_delayed_work(system_unbound_wq, &scx_watchdog_work,
			   scx_watchdog_timeout / 2);

	/*
	 * Lock out forks, cgroup on/offlining and moves before opening the
	 * floodgate so that they don't wander into the operations prematurely.
	 */
	percpu_down_write(&scx_fork_rwsem);

	reset_idle_masks();

	/*
	 * All cgroups should be initialized before letting in tasks. cgroup
	 * on/offlining and task migrations are already locked out.
	 */
	ret = scx_cgroup_init();
	if (ret)
		goto err_disable_unlock;


	wait_for_completion_interruptible(&tick_sched_clock_completion);
	atomic64_set(&scx_irq_work_lastq_ws, tick_sched_clock);

	/*
	 * Enable ops for every task. Fork is excluded by scx_fork_rwsem
	 * preventing new tasks from being added. No need to exclude tasks
	 * leaving as sched_ext_free() can handle both prepped and enabled
	 * tasks. Prep all tasks first and then enable them with preemption
	 * disabled.
	 */
	spin_lock_irq(&scx_tasks_lock);

	atomic_set(&non_ext_task, false);
	atomic_set(&__scx_ops_enabled, true);

	scx_task_iter_init(&sti);
	while ((p = scx_task_iter_next_filtered(&sti))) {
		scx_ops_prepare_task(p, task_group(p));
	}
	scx_task_iter_exit(&sti);

	/*
	 * All tasks are prepped but are still ops-disabled. Ensure that
	 * %current can't be scheduled out and switch everyone.
	 * preempt_disable() is necessary because we can't guarantee that
	 * %current won't be starved if scheduled out while switching.
	 */
	preempt_disable();

	/*
	 * From here on, the disable path must assume that tasks have ops
	 * enabled and need to be recovered.
	 */
	if (!scx_ops_tryset_enable_state(SCX_OPS_ENABLING, SCX_OPS_PREPPING)) {
		preempt_enable();
		spin_unlock_irq(&scx_tasks_lock);
		ret = -EBUSY;
		goto err_disable_unlock;
	}

	/*
	 * We're fully committed and can't fail. The PREPPED -> ENABLED
	 * transitions here are synchronized against sched_ext_free() through
	 * scx_tasks_lock.
	 */
	start = sched_clock();
	scx_switch_log(SCX_RQ_SWITCH_BEGIN, 0, 0);
	scx_task_iter_init(&sti);
	while ((p = scx_task_iter_next_filtered_locked(&sti))) {
		if (reject_change_to_scx(p, p->prio))
			continue;

		tcnt++;
		if (READ_ONCE(p->__state) != TASK_DEAD) {
			const struct sched_class *old_class = p->sched_class;
			struct rq *rq = task_rq(p);

			set_audio_thread_sched_prop(p);
			update_rq_clock(rq);

			SCHED_CHANGE_BLOCK(rq, p, DEQUEUE_SAVE | DEQUEUE_MOVE |
					   DEQUEUE_NOCLOCK) {
				scx_ops_enable_task(p);
				__setscheduler_prio(p, p->prio);
			}

			check_class_changed(task_rq(p), p, old_class, p->prio);
		} else {
			scx_ops_disable_task(p);
		}
	}
	scx_task_iter_exit(&sti);

	spin_unlock_irq(&scx_tasks_lock);
	scx_switch_log(SCX_RQ_SWITCH_DONE, 0, 0);
	preempt_enable();
	percpu_up_write(&scx_fork_rwsem);

	if (!scx_ops_tryset_enable_state(SCX_OPS_ENABLED, SCX_OPS_ENABLING)) {
		ret = -EBUSY;
		goto err_disable;
	}

	cpus_read_unlock();
	mutex_unlock(&scx_ops_enable_mutex);
	scx_switch_log(SCX_ENABLED, 1, 1);

	return 0;

err_unlock:
	mutex_unlock(&scx_ops_enable_mutex);
	return ret;

err_disable_unlock:
	percpu_up_write(&scx_fork_rwsem);
err_disable:
	cpus_read_unlock();
	mutex_unlock(&scx_ops_enable_mutex);
	/* must be fully disabled before returning */
	scx_ops_disable(SCX_EXIT_ERROR);
	kthread_flush_work(&scx_ops_disable_work);
	scx_switch_log(SCX_DISABLED, 0, 1);
	return ret;
}

#ifdef CONFIG_SCHED_DEBUG
static const char *scx_ops_enable_state_str[] = {
	[SCX_OPS_PREPPING]	= "prepping",
	[SCX_OPS_ENABLING]	= "enabling",
	[SCX_OPS_ENABLED]	= "enabled",
	[SCX_OPS_DISABLING]	= "disabling",
	[SCX_OPS_DISABLED]	= "disabled",
};

static int scx_debug_show(struct seq_file *m, void *v)
{
	mutex_lock(&scx_ops_enable_mutex);
	seq_printf(m, "%-30s: %d\n", "enabled", scx_enabled());
	seq_printf(m, "%-30s: %s\n", "enable_state",
		   scx_ops_enable_state_str[scx_ops_enable_state()]);
	seq_printf(m, "%-30s: %llu\n", "nr_rejected",
		   atomic64_read(&scx_nr_rejected));
	mutex_unlock(&scx_ops_enable_mutex);
	return 0;
}

static int scx_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, scx_debug_show, NULL);
}

const struct file_operations sched_ext_fops = {
	.open		= scx_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif


static int bpf_scx_reg(void *kdata)
{
	return scx_ops_enable(kdata);
}

static void bpf_scx_unreg(void *kdata)
{
	scx_ops_disable(SCX_EXIT_UNREG);
	kthread_flush_work(&scx_ops_disable_work);
}

/*
 * MUST load ext module before enable ext scheduler
 * load track & ext gover implemented in ext module
 */
void ext_ctrl(bool enable)
{
	static bool prev_state;

	if (!atomic_read(&ext_module_loaded)) {
		pr_err("hmbird_sched : ext module not loaded, failed to enable ext!\n");
		return;
	}

	if (prev_state == enable)
		return;

	if (enable)
		bpf_scx_reg(NULL);
	else
		bpf_scx_unreg(NULL);

	prev_state = enable;
}

static void sysrq_handle_sched_ext_reset(int key)
{
	if (scx_ops_helper)
		scx_ops_disable(SCX_EXIT_SYSRQ);
	else
		pr_info("sched_ext: BPF scheduler not yet used\n");
}

static const struct sysrq_key_op sysrq_sched_ext_reset_op = {
	.handler	= sysrq_handle_sched_ext_reset,
	.help_msg	= "reset-sched-ext(S)",
	.action_msg	= "Disable sched_ext and revert all tasks to CFS",
	.enable_mask	= SYSRQ_ENABLE_RTNICE,
};

void __init init_sched_ext_class(void)
{
	int cpu;
	u32 v;

	/*
	 * The following is to prevent the compiler from optimizing out the enum
	 * definitions so that BPF scheduler implementations can use them
	 * through the generated vmlinux.h.
	 */
	WRITE_ONCE(v, SCX_WAKE_EXEC | SCX_ENQ_WAKEUP | SCX_DEQ_SLEEP |
		   SCX_KICK_PREEMPT);

	init_dsq(&scx_dsq_global, SCX_DSQ_GLOBAL);
	init_dsq_at_boot();
	init_isolate_cpus();
#ifdef CONFIG_SMP
	WARN_ON(!alloc_cpumask_var(&idle_masks.cpu, GFP_KERNEL));
	WARN_ON(!alloc_cpumask_var(&idle_masks.smt, GFP_KERNEL));
#endif

	/*
	 * we can't static init init_task's scx struct, init here.
	 * init_task->scx would not use during boot.
	 */
	init_task.scx = kmalloc(sizeof(struct sched_ext_entity), GFP_KERNEL);
	if (init_task.scx) {
		INIT_LIST_HEAD(&init_task.scx->dsq_node.fifo);
		INIT_LIST_HEAD(&init_task.scx->watchdog_node);
		init_task.scx->sticky_cpu = -1;
		init_task.scx->holding_cpu = -1;
		atomic64_set(&init_task.scx->ops_state, 0);
		init_task.scx->runnable_at = jiffies;
		init_task.scx->slice = SCX_SLICE_DFL;
		init_task.scx->task = &init_task;
		sched_set_sched_prop(&init_task, 0);
	} else {
		pr_err("fatal error : alloc init_task.scx failed!!!\n");
	}

	for_each_possible_cpu(cpu) {
		struct rq *rq = cpu_rq(cpu);

		/*
		 * exec during boot phase, no need to care about alloc faild.
		 * lifecycle same to rq, no need to free.
		 */
		rq->scx = kmalloc(sizeof(struct scx_rq), GFP_KERNEL);
		if (rq->scx)
			rq->scx->rq = rq;
		else
			pr_err("fatal error : alloc rq->scx failed!!!\n");


		init_dsq(&rq->scx->local_dsq, SCX_DSQ_LOCAL);
		INIT_LIST_HEAD(&rq->scx->watchdog_list);

		WARN_ON(!zalloc_cpumask_var(&rq->scx->cpus_to_kick, GFP_KERNEL));
		WARN_ON(!zalloc_cpumask_var(&rq->scx->cpus_to_preempt, GFP_KERNEL));
		WARN_ON(!zalloc_cpumask_var(&rq->scx->cpus_to_wait, GFP_KERNEL));
	}

	register_sysrq_key('S', &sysrq_sched_ext_reset_op);
	INIT_DELAYED_WORK(&scx_watchdog_work, scx_watchdog_workfn);
}


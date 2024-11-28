#ifndef __SLIM_H
#define __SLIM_H

extern int scx_enable;

extern int cpuctrl_high_ratio;
extern int cpuctrl_low_ratio;
extern int partial_enable;
extern int slim_stats;
extern int misfit_ds;

extern int slim_walt_ctrl;
extern int slim_walt_dump;
extern int slim_walt_policy;
extern int slim_gov_debug;
extern int hmbirdcore_debug;
extern int sched_ravg_window_frame_per_sec;

extern atomic_t __scx_ops_enabled;
extern atomic_t non_ext_task;

extern noinline int tracing_mark_write(const char *buf);
int task_top_id(struct task_struct *p);
void stats_print(char *buf, int len);
extern spinlock_t scx_tasks_lock;
#endif

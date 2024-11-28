/* SPDX-License-Identifier: GPL-2.0 */

#include "slim.h"


int scx_enable;
int cpuctrl_high_ratio = 55;
int cpuctrl_low_ratio = 40;
int partial_enable;
int slim_stats;
int hmbirdcore_debug = DEBUG_INTERNAL;
int misfit_ds = 40;

int slim_gov_debug;

extern int slim_for_app;

int sched_ravg_window_frame_per_sec = 125;

#define SLIM_SCHED_CTRL		"scx_enable"
#define PAR_CTRL_NAME           "partial_ctrl"
#define CPUCTRL_HIGH_THRES      "cpuctrl_high"
#define CPUCTRL_LOW_THRES       "cpuctrl_low"
#define SLIM_STATS		"slim_stats"
#define SLIM_TRACE		"hmbirdcore_debug"
#define MISFIT_DS		"misfit_ds"

#define SCHED_EXT_DIR		"hmbird_sched"

#define SLIM_FOR_APP		"slim_for_app"

noinline int tracing_mark_write(const char *buf)
{
        trace_printk(buf);
        return 0;
}

static char *slim_config[] = {
	SLIM_SCHED_CTRL,
	PAR_CTRL_NAME,
	CPUCTRL_HIGH_THRES,
	CPUCTRL_LOW_THRES,
	SLIM_STATS,
	SLIM_TRACE,
	SLIM_FOR_APP,
	MISFIT_DS,
};

static int *slim_data[] = {
	&scx_enable,
	&partial_enable,
	&cpuctrl_high_ratio,
	&cpuctrl_low_ratio,
	&slim_stats,
	&hmbirdcore_debug,
	&slim_for_app,
	&misfit_ds,
};

static ssize_t slim_common_write(struct file *file, const char __user *buf,
                               size_t count, loff_t *ppos)
{
	int *pval = (int *)pde_data(file_inode(file));
	char kbuf[5] = {0};
	int err;

	if (count >= 5)
		return -EFAULT;

	if (copy_from_user(kbuf, buf, count)) {
		pr_err("slim_sched : Failed to copy_from_user\n");
		return -EFAULT;
	}
	err = kstrtoint(strstrip(kbuf), 0, pval);
	if (err < 0) {
		pr_err("slim_sched: Failed to exec kstrtoint\n");
		return -EFAULT;
	}

	if (pval == &scx_enable)
		ext_ctrl(*pval);

	return count;
}

static int slim_common_show(struct seq_file *m, void *v)
{
	seq_printf(m, "%d\n", *(int*) m->private);
	return 0;
}

static int slim_common_open(struct inode *inode, struct file *file)
{
	return single_open(file, slim_common_show, pde_data(inode));
}

static const struct proc_ops common_proc_ops = {
	.proc_open              = slim_common_open,
	.proc_write             = slim_common_write,
	.proc_read              = seq_read,
	.proc_lseek             = seq_lseek,
	.proc_release           = single_release,
};

#define HMBIRD_STATS	"hmbird_stats"
#define MAX_STATS_BUF	(2000)
static int hmbird_stats_show(struct seq_file *m, void *v)
{
	char buf[MAX_STATS_BUF] = {0};

	stats_print(buf, MAX_STATS_BUF);

	seq_printf(m, "%s\n", buf);
	return 0;
}

static int hmbird_stats_open(struct inode *inode, struct file *file)
{
	return single_open(file, hmbird_stats_show, inode);
}

static const struct proc_ops hmbird_stat_fops = {
	.proc_open		= hmbird_stats_open,
	.proc_read		= seq_read,
	.proc_lseek		= seq_lseek,
	.proc_release		= single_release,
};



static int __init slim_sysfs_init(void)
{
	int i;
	hmbird_dir = proc_mkdir(SCHED_EXT_DIR, NULL);
	if (hmbird_dir) {
		for (i = 0; i < ARRAY_SIZE(slim_config); i++) {
			proc_create_data(slim_config[i], S_IRUGO | S_IWUGO,
					hmbird_dir, &common_proc_ops, slim_data[i]);
		}
		proc_create(HMBIRD_STATS, S_IRUGO | S_IWUGO, hmbird_dir, &hmbird_stat_fops);
	}
	return 0;
}

__initcall(slim_sysfs_init);

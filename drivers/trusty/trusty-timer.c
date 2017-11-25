/*
 * Copyright (C) 2017 Intel, Inc.
 * Copyright (C) 2016 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/smwall.h>
#include <linux/trusty/trusty.h>

struct trusty_timer {
	struct sec_timer_state *sts;
	struct hrtimer tm;
};

struct trusty_timer_dev_state {
	struct device *dev;
	struct device *smwall_dev;
	struct device *trusty_dev;
	struct notifier_block call_notifier;
	struct trusty_timer timer;
};

static enum hrtimer_restart trusty_timer_cb(struct hrtimer *tm)
{
	struct trusty_timer_dev_state *s;

	s = container_of(tm, struct trusty_timer_dev_state, timer.tm);

	set_pending_intr_to_lk(0x31);
	trusty_enqueue_nop(s->trusty_dev, NULL);

	return HRTIMER_NORESTART;
}

static int trusty_timer_call_notify(struct notifier_block *nb,
		unsigned long action, void *data)
{
	struct trusty_timer *tt;
	struct sec_timer_state *sts;
	struct trusty_timer_dev_state *s;

	if (action != TRUSTY_CALL_RETURNED)
		return NOTIFY_DONE;

	s = container_of(nb, struct trusty_timer_dev_state, call_notifier);

	/* this notifier is executed in non-preemptible context */
	tt = &s->timer;
	sts = tt->sts;

	if (sts->tv_ns > sts->cv_ns) {
		hrtimer_cancel(&tt->tm);
	} else if (sts->cv_ns > sts->tv_ns) {
		/* need to set/reset timer */
		hrtimer_start(&tt->tm, ns_to_ktime(sts->cv_ns - sts->tv_ns),
				HRTIMER_MODE_REL_PINNED);
	}

	sts->cv_ns = 0ULL;
	sts->tv_ns = 0ULL;

	return NOTIFY_OK;
}

static int trusty_timer_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int cpu;
	struct trusty_timer_dev_state *s;
	struct trusty_timer *tt;

	dev_dbg(&pdev->dev, "%s\n", __func__);

	if (!trusty_wall_base(pdev->dev.parent)) {
		dev_notice(&pdev->dev, "smwall: is not setup by parent\n");
		return -ENODEV;
	}

	s = kzalloc(sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;

	s->dev = &pdev->dev;
	s->smwall_dev = s->dev->parent;
	s->trusty_dev = s->smwall_dev->parent;
	platform_set_drvdata(pdev, s);

	tt = &s->timer;

	hrtimer_init(&tt->tm, CLOCK_BOOTTIME, HRTIMER_MODE_REL_PINNED);
	tt->tm.function = trusty_timer_cb;
	tt->sts =
		trusty_wall_per_cpu_item_ptr(s->smwall_dev, 0,
				SM_WALL_PER_CPU_SEC_TIMER_ID,
				sizeof(*tt->sts));
	WARN_ON(!tt->sts);


	/* register notifier */
	s->call_notifier.notifier_call = trusty_timer_call_notify;
	ret = trusty_call_notifier_register(s->trusty_dev, &s->call_notifier);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to register call notifier\n");
		kfree(s);
		return ret;
	}

	dev_info(s->dev, "initialized\n");

	return 0;

}

static int trusty_timer_remove(struct platform_device *pdev)
{
	unsigned int cpu;
	struct trusty_timer_dev_state *s = platform_get_drvdata(pdev);
	struct trusty_timer *tt;


	dev_dbg(&pdev->dev, "%s\n", __func__);

	/* unregister notifier */
	trusty_call_notifier_unregister(s->trusty_dev, &s->call_notifier);

	tt = &s->timer;
	hrtimer_cancel(&tt->tm);

	/* free state */
	kfree(s);
	return 0;
}

static const struct of_device_id trusty_test_of_match[] = {
	{ .compatible = "android,trusty-timer-v1", },
	{},
};

static struct platform_driver trusty_timer_driver = {
	.probe = trusty_timer_probe,
	.remove = trusty_timer_remove,
	.driver = {
		.name = "trusty-timer",
		.owner = THIS_MODULE,
		.of_match_table = trusty_test_of_match,
	},
};

module_platform_driver(trusty_timer_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Trusty timer driver");

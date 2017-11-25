/*
 * Copyright (C) 2013 Google, Inc.
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

#include <linux/module.h>
#include <linux/cpu.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/irqdomain.h>
#include <linux/of_irq.h>
#include <linux/trusty/smcall.h>
#include <linux/trusty/sm_err.h>
#include <linux/trusty/trusty.h>

#define IRQ_VECTOR_OFFSET 0x30
#define IRQ_FOR_LK_TIMER 1

struct trusty_irq {
	struct trusty_irq_state *is;
	struct hlist_node node;
	unsigned int irq;
	bool percpu;
	bool enable;
	struct trusty_irq __percpu *percpu_ptr;
};

struct trusty_irq_irqset {
	struct hlist_head pending;
	struct hlist_head inactive;
};

struct trusty_irq_state {
	struct device *dev;
	struct device *trusty_dev;
	struct trusty_irq_irqset normal_irqs;
	spinlock_t normal_irqs_lock;
	struct trusty_irq_irqset __percpu *percpu_irqs;
	struct notifier_block trusty_call_notifier;
	struct notifier_block cpu_notifier;
};

static void trusty_irq_enable_pending_irqs(struct trusty_irq_state *is,
					   struct trusty_irq_irqset *irqset,
					   bool percpu)
{
	struct hlist_node *n;
	struct trusty_irq *trusty_irq;

	hlist_for_each_entry_safe(trusty_irq, n, &irqset->pending, node) {
		dev_dbg(is->dev,
			"%s: enable pending irq %d, percpu %d, cpu %d\n",
			__func__, trusty_irq->irq, percpu, smp_processor_id());
		if (percpu)
			enable_percpu_irq(trusty_irq->irq, 0);
		else
			enable_irq(trusty_irq->irq);
		hlist_del(&trusty_irq->node);
		hlist_add_head(&trusty_irq->node, &irqset->inactive);
	}
}

static void trusty_irq_enable_irqset(struct trusty_irq_state *is,
				      struct trusty_irq_irqset *irqset)
{
	struct trusty_irq *trusty_irq;

	hlist_for_each_entry(trusty_irq, &irqset->inactive, node) {
		if (trusty_irq->enable) {
			dev_warn(is->dev,
				 "%s: percpu irq %d already enabled, cpu %d\n",
				 __func__, trusty_irq->irq, smp_processor_id());
			continue;
		}
		dev_dbg(is->dev, "%s: enable percpu irq %d, cpu %d\n",
			__func__, trusty_irq->irq, smp_processor_id());
		enable_percpu_irq(trusty_irq->irq, 0);
		trusty_irq->enable = true;
	}
}

static void trusty_irq_disable_irqset(struct trusty_irq_state *is,
				      struct trusty_irq_irqset *irqset)
{
	struct hlist_node *n;
	struct trusty_irq *trusty_irq;

	hlist_for_each_entry(trusty_irq, &irqset->inactive, node) {
		if (!trusty_irq->enable) {
			dev_warn(is->dev,
				 "irq %d already disabled, percpu %d, cpu %d\n",
				 trusty_irq->irq, trusty_irq->percpu,
				 smp_processor_id());
			continue;
		}
		dev_dbg(is->dev, "%s: disable irq %d, percpu %d, cpu %d\n",
			__func__, trusty_irq->irq, trusty_irq->percpu,
			smp_processor_id());
		trusty_irq->enable = false;
		if (trusty_irq->percpu)
			disable_percpu_irq(trusty_irq->irq);
		else
			disable_irq_nosync(trusty_irq->irq);
	}
	hlist_for_each_entry_safe(trusty_irq, n, &irqset->pending, node) {
		if (!trusty_irq->enable) {
			dev_warn(is->dev,
				 "pending irq %d already disabled, percpu %d, cpu %d\n",
				 trusty_irq->irq, trusty_irq->percpu,
				 smp_processor_id());
		}
		dev_dbg(is->dev,
			"%s: disable pending irq %d, percpu %d, cpu %d\n",
			__func__, trusty_irq->irq, trusty_irq->percpu,
			smp_processor_id());
		trusty_irq->enable = false;
		hlist_del(&trusty_irq->node);
		hlist_add_head(&trusty_irq->node, &irqset->inactive);
	}
}

static int trusty_irq_call_notify(struct notifier_block *nb,
				  unsigned long action, void *data)
{
	struct trusty_irq_state *is;

	BUG_ON(!irqs_disabled());

	if (action != TRUSTY_CALL_PREPARE)
		return NOTIFY_DONE;

	is = container_of(nb, struct trusty_irq_state, trusty_call_notifier);

	spin_lock(&is->normal_irqs_lock);
	trusty_irq_enable_pending_irqs(is, &is->normal_irqs, false);
	spin_unlock(&is->normal_irqs_lock);
	trusty_irq_enable_pending_irqs(is, this_cpu_ptr(is->percpu_irqs), true);

	return NOTIFY_OK;
}

irqreturn_t trusty_irq_handler(int irq, void *data)
{
	struct trusty_irq *trusty_irq = data;
	struct trusty_irq_state *is = trusty_irq->is;
	struct trusty_irq_irqset *irqset;

	dev_dbg(is->dev, "%s: irq %d, percpu %d, cpu %d, enable %d\n",
		__func__, irq, trusty_irq->irq, smp_processor_id(),
		trusty_irq->enable);

	WARN_ON(irq != IRQ_FOR_LK_TIMER);

	set_pending_intr_to_lk(irq+IRQ_VECTOR_OFFSET);

	if (trusty_irq->percpu) {
		disable_percpu_irq(irq);
		irqset = this_cpu_ptr(is->percpu_irqs);
	} else {
		disable_irq_nosync(irq);
		irqset = &is->normal_irqs;
	}

	spin_lock(&is->normal_irqs_lock);
	if (trusty_irq->enable) {
		hlist_del(&trusty_irq->node);
		hlist_add_head(&trusty_irq->node, &irqset->pending);
	}
	spin_unlock(&is->normal_irqs_lock);

	trusty_enqueue_nop(is->trusty_dev, NULL);

	dev_dbg(is->dev, "%s: irq %d done\n", __func__, irq);

	return IRQ_HANDLED;
}

static void trusty_irq_cpu_up(void *info)
{
	unsigned long irq_flags;
	struct trusty_irq_state *is = info;

	dev_dbg(is->dev, "%s: cpu %d\n", __func__, smp_processor_id());

	local_irq_save(irq_flags);
	trusty_irq_enable_irqset(is, this_cpu_ptr(is->percpu_irqs));
	local_irq_restore(irq_flags);
}

static void trusty_irq_cpu_down(void *info)
{
	unsigned long irq_flags;
	struct trusty_irq_state *is = info;

	dev_dbg(is->dev, "%s: cpu %d\n", __func__, smp_processor_id());

	local_irq_save(irq_flags);
	trusty_irq_disable_irqset(is, this_cpu_ptr(is->percpu_irqs));
	local_irq_restore(irq_flags);
}

static int trusty_irq_cpu_notify(struct notifier_block *nb,
				 unsigned long action, void *hcpu)
{
	struct trusty_irq_state *is;

	is = container_of(nb, struct trusty_irq_state, cpu_notifier);

	dev_dbg(is->dev, "%s: 0x%lx\n", __func__, action);

	switch (action & ~CPU_TASKS_FROZEN) {
	case CPU_UP_PREPARE:
		trusty_irq_cpu_up(is);
		break;
	case CPU_DEAD:
		trusty_irq_cpu_down(is);
		break;
	}

	return NOTIFY_OK;
}

static int trusty_irq_create_irq_mapping(struct trusty_irq_state *is, int irq)
{
	int ret;
	int index;
	u32 irq_pos;
	u32 templ_idx;
	u32 range_base;
	u32 range_end;
	struct of_phandle_args oirq;

	/* check if "interrupt-ranges" property is present */
	if (!of_find_property(is->dev->of_node, "interrupt-ranges", NULL)) {
		/* fallback to old behavior to be backward compatible with
		 * systems that do not need IRQ domains.
		 */
		return irq;
	}

	/* find irq range */
	for (index = 0;; index += 3) {
		ret = of_property_read_u32_index(is->dev->of_node,
						 "interrupt-ranges",
						 index, &range_base);
		if (ret)
			return ret;

		ret = of_property_read_u32_index(is->dev->of_node,
						 "interrupt-ranges",
						 index + 1, &range_end);
		if (ret)
			return ret;

		if (irq >= range_base && irq <= range_end)
			break;
	}

	/*  read the rest of range entry: template index and irq_pos */
	ret = of_property_read_u32_index(is->dev->of_node,
					 "interrupt-ranges",
					 index + 2, &templ_idx);
	if (ret)
		return ret;

	/* read irq template */
	ret = of_parse_phandle_with_args(is->dev->of_node,
					 "interrupt-templates",
					 "#interrupt-cells",
					 templ_idx, &oirq);
	if (ret)
		return ret;

	WARN_ON(!oirq.np);
	WARN_ON(!oirq.args_count);

	/*
	 * An IRQ template is a non empty array of u32 values describing group
	 * of interrupts having common properties. The u32 entry with index
	 * zero contains the position of irq_id in interrupt specifier array
	 * followed by data representing interrupt specifier array with irq id
	 * field omitted, so to convert irq template to interrupt specifier
	 * array we have to move down one slot the first irq_pos entries and
	 * replace the resulting gap with real irq id.
	 */
	irq_pos = oirq.args[0];

	if (irq_pos >= oirq.args_count) {
		dev_err(is->dev, "irq pos is out of range: %d\n", irq_pos);
		return -EINVAL;
	}

	for (index = 1; index <= irq_pos; index++)
		oirq.args[index - 1] = oirq.args[index];

	oirq.args[irq_pos] = irq - range_base;

	ret = irq_create_of_mapping(&oirq);

	return (!ret) ? -EINVAL : ret;
}

static inline void trusty_irq_unmask(struct irq_data *data)
{
	return;
}

static inline void trusty_irq_mask(struct irq_data *data)
{
	return;
}

static void trusty_irq_enable(struct irq_data *data)
{
	return;
}

static void trusty_irq_disable(struct irq_data *data)
{
	return;
}

void trusty_irq_eoi(struct irq_data *data)
{
	return;
}
static struct irq_chip trusty_irq_chip = {
		.name = "TRUSY-IRQ",
		.irq_mask = trusty_irq_mask,
		.irq_unmask = trusty_irq_unmask,
		.irq_enable = trusty_irq_enable,
		.irq_disable = trusty_irq_disable,
		.irq_eoi = trusty_irq_eoi,
};

static int trusty_irq_init_normal_irq(struct trusty_irq_state *is, int tirq)
{
	int ret;
	int irq;
	unsigned long irq_flags;
	struct trusty_irq *trusty_irq;

	dev_dbg(is->dev, "%s: irq %d\n", __func__, tirq);

	irq = tirq;

	trusty_irq = kzalloc(sizeof(*trusty_irq), GFP_KERNEL);
	if (!trusty_irq)
		return -ENOMEM;

	trusty_irq->is = is;
	trusty_irq->irq = irq;
	trusty_irq->enable = true;

	spin_lock_irqsave(&is->normal_irqs_lock, irq_flags);
	hlist_add_head(&trusty_irq->node, &is->normal_irqs.inactive);
	spin_unlock_irqrestore(&is->normal_irqs_lock, irq_flags);

	ret = irq_alloc_desc_at(irq, 0);
	if (ret >= 0)
		irq_set_chip_and_handler_name(irq, &trusty_irq_chip, handle_edge_irq, "trusty-irq");
	else if (ret != -EEXIST) {
		dev_err(is->dev, "can't allocate irq desc %d\n", ret);
		goto err_request_irq;
	}

	ret = request_irq(irq, trusty_irq_handler, IRQF_NO_THREAD,
			  "trusty-irq", trusty_irq);

	if (ret) {
		dev_err(is->dev, "request_irq failed %d\n", ret);
		goto err_request_irq;
	}
	return 0;

err_request_irq:
	spin_lock_irqsave(&is->normal_irqs_lock, irq_flags);
	hlist_del(&trusty_irq->node);
	spin_unlock_irqrestore(&is->normal_irqs_lock, irq_flags);
	kfree(trusty_irq);
	return ret;
}

static int trusty_irq_init_per_cpu_irq(struct trusty_irq_state *is, int tirq)
{
	int ret;
	int irq;
	unsigned int cpu;
	struct trusty_irq __percpu *trusty_irq_handler_data;

	dev_dbg(is->dev, "%s: irq %d\n", __func__, tirq);

	irq = trusty_irq_create_irq_mapping(is, tirq);
	if (irq <= 0) {
		dev_err(is->dev,
			"trusty_irq_create_irq_mapping failed (%d)\n", irq);
		return irq;
	}

	trusty_irq_handler_data = alloc_percpu(struct trusty_irq);
	if (!trusty_irq_handler_data)
		return -ENOMEM;

	for_each_possible_cpu(cpu) {
		struct trusty_irq *trusty_irq;
		struct trusty_irq_irqset *irqset;

		if (cpu >= 32)
			return -EINVAL;
		trusty_irq = per_cpu_ptr(trusty_irq_handler_data, cpu);
		irqset = per_cpu_ptr(is->percpu_irqs, cpu);

		trusty_irq->is = is;
		hlist_add_head(&trusty_irq->node, &irqset->inactive);
		trusty_irq->irq = irq;
		trusty_irq->percpu = true;
		trusty_irq->percpu_ptr = trusty_irq_handler_data;
	}

	ret = request_percpu_irq(irq, trusty_irq_handler, "trusty",
				 trusty_irq_handler_data);
	if (ret) {
		dev_err(is->dev, "request_percpu_irq failed %d\n", ret);
		goto err_request_percpu_irq;
	}

	return 0;

err_request_percpu_irq:
	for_each_possible_cpu(cpu) {
		struct trusty_irq *trusty_irq;

		if (cpu >= 32)
			return -EINVAL;
		trusty_irq = per_cpu_ptr(trusty_irq_handler_data, cpu);
		hlist_del(&trusty_irq->node);
	}

	free_percpu(trusty_irq_handler_data);
	return ret;
}

static int trusty_smc_get_next_irq(struct trusty_irq_state *is,
				   unsigned long min_irq, bool per_cpu)
{
	return trusty_fast_call32(is->trusty_dev, SMC_FC_GET_NEXT_IRQ,
				  min_irq, per_cpu, 0);
}

static int trusty_irq_init_one(struct trusty_irq_state *is,
			       int irq, bool per_cpu)
{
	int ret;

	irq = trusty_smc_get_next_irq(is, irq, per_cpu);
	if (irq < 0)
		return irq;
	dev_info(is->dev, "irq from lk = %d\n", irq);

	WARN_ON(irq-IRQ_VECTOR_OFFSET != IRQ_FOR_LK_TIMER);

	if (per_cpu)
		ret = trusty_irq_init_per_cpu_irq(is, irq-IRQ_VECTOR_OFFSET);
	else
		ret = trusty_irq_init_normal_irq(is, irq-IRQ_VECTOR_OFFSET);

	if (ret) {
		dev_warn(is->dev,
			 "failed to initialize irq %d, irq will be ignored\n",
			 irq);
	}

	return irq + 1;
}

static void trusty_irq_free_irqs(struct trusty_irq_state *is)
{
	struct trusty_irq *irq;
	struct hlist_node *n;

	hlist_for_each_entry_safe(irq, n, &is->normal_irqs.inactive, node) {
		dev_dbg(is->dev, "%s: irq %d\n", __func__, irq->irq);
		free_irq(irq->irq, irq);
		hlist_del(&irq->node);
		kfree(irq);
	}
/*
	hlist_for_each_entry_safe(irq, n,
				  &this_cpu_ptr(is->percpu_irqs)->inactive,
				  node) {
		struct trusty_irq __percpu *trusty_irq_handler_data;

		dev_dbg(is->dev, "%s: percpu irq %d\n", __func__, irq->irq);
		trusty_irq_handler_data = irq->percpu_ptr;
		free_percpu_irq(irq->irq, trusty_irq_handler_data);
		for_each_possible_cpu(cpu) {
			struct trusty_irq *irq_tmp;

			irq_tmp = per_cpu_ptr(trusty_irq_handler_data, cpu);
			hlist_del(&irq_tmp->node);
		}
		free_percpu(trusty_irq_handler_data);
	} */
}

static int trusty_irq_probe(struct platform_device *pdev)
{
	int ret;
	int irq;
	unsigned long irq_flags;
	struct trusty_irq_state *is;

	ret = trusty_check_cpuid(NULL);
	if (ret < 0) {
		dev_err(&pdev->dev, "CPUID Error: Cannot find eVmm in trusty driver initialization!");
		return -EINVAL;
	}

	dev_dbg(&pdev->dev, "%s\n", __func__);

	is = kzalloc(sizeof(*is), GFP_KERNEL);
	if (!is) {
		ret = -ENOMEM;
		goto err_alloc_is;
	}

	is->dev = &pdev->dev;
	is->trusty_dev = is->dev->parent;
	spin_lock_init(&is->normal_irqs_lock);
	is->percpu_irqs = alloc_percpu(struct trusty_irq_irqset);
	if (!is->percpu_irqs) {
		ret = -ENOMEM;
		goto err_alloc_pending_percpu_irqs;
	}

	platform_set_drvdata(pdev, is);

	is->trusty_call_notifier.notifier_call = trusty_irq_call_notify;
	ret = trusty_call_notifier_register(is->trusty_dev,
					    &is->trusty_call_notifier);
	if (ret) {
		dev_err(&pdev->dev,
			"failed to register trusty call notifier\n");
		goto err_trusty_call_notifier_register;
	}

	for (irq = 0; irq >= 0;)
		irq = trusty_irq_init_one(is, irq, false);

	is->cpu_notifier.notifier_call = trusty_irq_cpu_notify;
	ret = register_hotcpu_notifier(&is->cpu_notifier);
	if (ret) {
		dev_err(&pdev->dev, "register_cpu_notifier failed %d\n", ret);
		goto err_register_hotcpu_notifier;
	}
	ret = on_each_cpu(trusty_irq_cpu_up, is, 0);
	if (ret) {
		dev_err(&pdev->dev, "register_cpu_notifier failed %d\n", ret);
		goto err_on_each_cpu;
	}

	return 0;

err_on_each_cpu:
	unregister_hotcpu_notifier(&is->cpu_notifier);
	on_each_cpu(trusty_irq_cpu_down, is, 1);
err_register_hotcpu_notifier:
	spin_lock_irqsave(&is->normal_irqs_lock, irq_flags);
	trusty_irq_disable_irqset(is, &is->normal_irqs);
	spin_unlock_irqrestore(&is->normal_irqs_lock, irq_flags);
	trusty_irq_free_irqs(is);
	trusty_call_notifier_unregister(is->trusty_dev,
					&is->trusty_call_notifier);
err_trusty_call_notifier_register:
	free_percpu(is->percpu_irqs);
err_alloc_pending_percpu_irqs:
	kfree(is);
err_alloc_is:
	return ret;
}

static int trusty_irq_remove(struct platform_device *pdev)
{
	int ret;
	unsigned long irq_flags;
	struct trusty_irq_state *is = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "%s\n", __func__);

	unregister_hotcpu_notifier(&is->cpu_notifier);
	ret = on_each_cpu(trusty_irq_cpu_down, is, 1);
	if (ret)
		dev_err(&pdev->dev, "on_each_cpu failed %d\n", ret);
	spin_lock_irqsave(&is->normal_irqs_lock, irq_flags);
	trusty_irq_disable_irqset(is, &is->normal_irqs);
	spin_unlock_irqrestore(&is->normal_irqs_lock, irq_flags);

	trusty_irq_free_irqs(is);

	trusty_call_notifier_unregister(is->trusty_dev,
					&is->trusty_call_notifier);
	free_percpu(is->percpu_irqs);
	kfree(is);

	return 0;
}

static const struct of_device_id trusty_test_of_match[] = {
	{ .compatible = "android,trusty-irq-v1", },
	{},
};

static struct platform_driver trusty_irq_driver = {
	.probe = trusty_irq_probe,
	.remove = trusty_irq_remove,
	.driver	= {
		.name = "trusty-irq",
		.owner = THIS_MODULE,
		.of_match_table = trusty_test_of_match,
	},
};

module_platform_driver(trusty_irq_driver);


MODULE_LICENSE("GPL v2");


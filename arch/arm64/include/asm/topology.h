/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __ASM_TOPOLOGY_H
#define __ASM_TOPOLOGY_H

#include <linux/cpumask.h>

struct cpu_topology {
	int thread_id;
	int core_id;
	int cluster_id;
	cpumask_t thread_sibling;
	cpumask_t core_sibling;
};

extern struct cpu_topology cpu_topology[NR_CPUS];

#define topology_physical_package_id(cpu)	(cpu_topology[cpu].cluster_id)
#define topology_core_id(cpu)		(cpu_topology[cpu].core_id)
#define topology_core_cpumask(cpu)	(&cpu_topology[cpu].core_sibling)
#define topology_sibling_cpumask(cpu)	(&cpu_topology[cpu].thread_sibling)

void init_cpu_topology(void);
void store_cpu_topology(unsigned int cpuid);
const struct cpumask *cpu_coregroup_mask(int cpu);

#ifdef CONFIG_NUMA

struct pci_bus;
int pcibus_to_node(struct pci_bus *bus);
#define cpumask_of_pcibus(bus)	(pcibus_to_node(bus) == -1 ?		\
				 cpu_all_mask :				\
				 cpumask_of_node(pcibus_to_node(bus)))

#endif /* CONFIG_NUMA */
struct sched_domain;
#ifdef CONFIG_CPU_FREQ
#define arch_scale_freq_capacity cpufreq_scale_freq_capacity
extern unsigned long cpufreq_scale_freq_capacity(struct sched_domain *sd, int cpu);
extern unsigned long cpufreq_scale_max_freq_capacity(int cpu);
#endif
#define arch_get_cpu_scale topology_get_cpu_scale
extern unsigned long topology_get_cpu_scale(struct sched_domain *sd, int cpu);

#include <linux/arch_topology.h>

/* Replace task scheduler's default frequency-invariant accounting */
#define arch_scale_freq_capacity topology_get_freq_scale

/* Replace task scheduler's default cpu-invariant accounting */
#define arch_scale_cpu_capacity topology_get_cpu_scale

#include <asm-generic/topology.h>

#endif /* _ASM_ARM_TOPOLOGY_H */

/*
 * (C) COPYRIGHT 2014 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU licence.
 *
 * A copy of the licence is included with the program, and can also be obtained
 * from Free Software Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 */

#include "mali_kbase_rk.h"

/**
 * Maximum frequency GPU will be clocked at. Given in kHz.
 */
#define GPU_FREQ_KHZ_MAX KBASE_RK_GPU_FREQ_KHZ_MAX

/**
 * Minimum frequency GPU will be clocked at. Given in kHz.
 */
#define GPU_FREQ_KHZ_MIN KBASE_RK_GPU_FREQ_KHZ_MIN

/**
 * A pointer to a function that calculates the CPU clock
 * speed of the platform in MHz - see
 * @ref kbase_cpuprops_clock_speed_function for the function
 * prototype.
 *
 * Attached value: A @ref kbase_cpuprops_clock_speed_function.
 * Default Value:  NA
 */
#define CPU_SPEED_FUNC (&kbase_cpuprops_get_default_clock_speed)

/**
 * A pointer to a function that calculates the GPU clock
 * speed of the platform in MHz - see
 * @ref kbase_gpuprops_clock_speed_function for the function
 * prototype.
 *
 * Attached value: A @ref kbase_gpuprops_clock_speed_function.
 * Default Value:  NA
 */
#define GPU_SPEED_FUNC (NULL)

/**
 * Power management configuration
 *
 * Attached value: pointer to @ref kbase_pm_callback_conf
 * Default value: See @ref kbase_pm_callback_conf
 */
#define POWER_MANAGEMENT_CALLBACKS (&kbase_rk_pm_callbacks)

/**
 * Platform specific configuration functions
 *
 * Attached value: pointer to @ref kbase_platform_funcs_conf
 * Default value: See @ref kbase_platform_funcs_conf
 */
#define PLATFORM_FUNCS (&kbase_rk_platform_funcs)

/**
 * Power model for IPA
 *
 * Attached value: pointer to @ref mali_pa_model_ops
 */
#define POWER_MODEL_CALLBACKS (NULL)

/**
 * Secure mode switch
 *
 * Attached value: pointer to @ref kbase_secure_ops
 */
#define SECURE_CALLBACKS (NULL)

extern struct kbase_pm_callback_conf kbase_rk_pm_callbacks;
extern struct kbase_platform_funcs_conf kbase_rk_platform_funcs;

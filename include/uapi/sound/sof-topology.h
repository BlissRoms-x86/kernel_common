/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2017 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * BSD LICENSE
 *
 * Copyright(c) 2017 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in
 *     the documentation and/or other materials provided with the
 *     distribution.
 *   * Neither the name of Intel Corporation nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */


/*
 * Topology IDs and tokens.
 *
 * ** MUST BE ALIGNED WITH TOPOLOGY CONFIGURATION TOKEN VALUES **
 */

#ifndef __INCLUDE_UAPI_SOF_TOPOLGOY_H__
#define __INCLUDE_UAPI_SOF_TOPOLOGY_H__

/*
 * Kcontrol IDs
 */
#define SOF_TPLG_KCTL_VOL_ID	256


/*
 * Tokens - must match values in topology configurations
 */

/* buffers */
#define SOF_TKN_BUF_SIZE	 		100
#define SOF_TKN_BUF_PRELOAD	 		101

/* DAI */
#define SOF_TKN_DAI_DMAC 			151
#define	SOF_TKN_DAI_DMAC_CHAN 			152
#define	SOF_TKN_DAI_DMAC_CONFIG			153

/* scheduling */
#define SOF_TKN_SCHED_DEADLINE 			200
#define SOF_TKN_SCHED_PRIORITY 			201
#define SOF_TKN_SCHED_MIPS 			202
#define SOF_TKN_SCHED_CORE 			203

/* volume */
#define SOF_TKN_VOLUME_RAMP_STEP_TYPE 		250
#define SOF_TKN_VOLUME_RAMP_STEP_MS 		251

/* SRC */
#define SOF_TKN_SRC_RATE_IN			300
#define SOF_TKN_SRC_RATE_OUT			301

#endif

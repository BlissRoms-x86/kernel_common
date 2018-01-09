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
 *         Keyon Jie <yang.jie@linux.intel.com>
 */

#ifndef __INCLUDE_UAPI_SOF_IPC_H__
#define __INCLUDE_UAPI_SOF_IPC_H__

/*
 * IPC messages have a prefixed 32 bit identifier made up as follows :-
 *
 * 	0xGCCCNNNN where
 * G is global cmd type (4 bits)
 * C is command type (12 bits)
 * I is the ID number (16 bits) - monotonic and overflows
 *
 * This is sent at the start of the IPM message in the mailbox. Messages should
 * not be sent in the doorbell (special exceptions for firmware .
 */

/* Global Message - Generic */
#define SOF_GLB_TYPE_SHIFT			28
#define SOF_GLB_TYPE_MASK			(0xf << SOF_GLB_TYPE_SHIFT)
#define SOF_GLB_TYPE(x)				(x << SOF_GLB_TYPE_SHIFT)

/* Command Message - Generic */
#define SOF_CMD_TYPE_SHIFT			16
#define SOF_CMD_TYPE_MASK			(0xfff << SOF_CMD_TYPE_SHIFT)
#define SOF_CMD_TYPE(x)				(x << SOF_CMD_TYPE_SHIFT)

/* Global Message Types */
#define SOF_IPC_GLB_REPLY			SOF_GLB_TYPE(0x1U)
#define SOF_IPC_GLB_COMPOUND			SOF_GLB_TYPE(0x2U)
#define SOF_IPC_GLB_TPLG_MSG			SOF_GLB_TYPE(0x3U)
#define SOF_IPC_GLB_PM_MSG			SOF_GLB_TYPE(0x4U)
#define SOF_IPC_GLB_COMP_MSG			SOF_GLB_TYPE(0x5U)
#define SOF_IPC_GLB_STREAM_MSG			SOF_GLB_TYPE(0x6U)
#define SOF_IPC_FW_READY			SOF_GLB_TYPE(0x7U)
#define SOF_IPC_GLB_DAI_MSG			SOF_GLB_TYPE(0x8U)

/*
 * DSP Command Message Types
 */


/* topology */
#define SOF_IPC_TPLG_COMP_NEW			SOF_CMD_TYPE(0x000)
#define SOF_IPC_TPLG_COMP_FREE			SOF_CMD_TYPE(0x001)
#define SOF_IPC_TPLG_COMP_CONNECT		SOF_CMD_TYPE(0x002)
#define SOF_IPC_TPLG_PIPE_NEW			SOF_CMD_TYPE(0x010)
#define SOF_IPC_TPLG_PIPE_FREE			SOF_CMD_TYPE(0x011)
#define SOF_IPC_TPLG_PIPE_CONNECT		SOF_CMD_TYPE(0x012)
#define SOF_IPC_TPLG_PIPE_COMPLETE		SOF_CMD_TYPE(0x013)
#define SOF_IPC_TPLG_BUFFER_NEW			SOF_CMD_TYPE(0x020)
#define SOF_IPC_TPLG_BUFFER_FREE		SOF_CMD_TYPE(0x021)

/* PM */
#define SOF_IPC_PM_CTX_SAVE			SOF_CMD_TYPE(0x000)
#define SOF_IPC_PM_CTX_RESTORE			SOF_CMD_TYPE(0x001)
#define SOF_IPC_PM_CTX_SIZE			SOF_CMD_TYPE(0x002)
#define SOF_IPC_PM_CLK_SET			SOF_CMD_TYPE(0x003)
#define SOF_IPC_PM_CLK_GET			SOF_CMD_TYPE(0x004)
#define SOF_IPC_PM_CLK_REQ			SOF_CMD_TYPE(0x005)

/* component - multiple different types */
#define SOF_IPC_COMP_SET_VOLUME			SOF_CMD_TYPE(0x000)
#define SOF_IPC_COMP_GET_VOLUME			SOF_CMD_TYPE(0x001)
#define SOF_IPC_COMP_SET_MIXER			SOF_CMD_TYPE(0x002)
#define SOF_IPC_COMP_GET_MIXER			SOF_CMD_TYPE(0x003)
#define SOF_IPC_COMP_SET_MUX			SOF_CMD_TYPE(0x004)
#define SOF_IPC_COMP_GET_MUX			SOF_CMD_TYPE(0x005)
#define SOF_IPC_COMP_SET_SRC			SOF_CMD_TYPE(0x006)
#define SOF_IPC_COMP_GET_SRC			SOF_CMD_TYPE(0x007)

/* DAI messages */
#define SOF_IPC_COMP_SSP_CONFIG			SOF_CMD_TYPE(0x000)
#define SOF_IPC_COMP_HDA_CONFIG			SOF_CMD_TYPE(0x001)
#define SOF_IPC_COMP_DMIC_CONFIG		SOF_CMD_TYPE(0x002)
#define SOF_IPC_COMP_LOOPBACK			SOF_CMD_TYPE(0x003)

/* stream */
#define SOF_IPC_STREAM_PCM_PARAMS		SOF_CMD_TYPE(0x001)
#define SOF_IPC_STREAM_PCM_PARAMS_REPLY		SOF_CMD_TYPE(0x002)
#define SOF_IPC_STREAM_PCM_FREE			SOF_CMD_TYPE(0x003)
#define SOF_IPC_STREAM_TRIG_START		SOF_CMD_TYPE(0x004)
#define SOF_IPC_STREAM_TRIG_STOP		SOF_CMD_TYPE(0x005)
#define SOF_IPC_STREAM_TRIG_PAUSE		SOF_CMD_TYPE(0x006)
#define SOF_IPC_STREAM_TRIG_RELEASE		SOF_CMD_TYPE(0x007)
#define SOF_IPC_STREAM_TRIG_DRAIN		SOF_CMD_TYPE(0x008)
#define SOF_IPC_STREAM_TRIG_XRUN		SOF_CMD_TYPE(0x009)
#define SOF_IPC_STREAM_POSITION			SOF_CMD_TYPE(0x00a)
#define SOF_IPC_STREAM_VORBIS_PARAMS		SOF_CMD_TYPE(0x010)
#define SOF_IPC_STREAM_VORBIS_FREE		SOF_CMD_TYPE(0x011)


/* Get message component id */
#define SOF_IPC_MESSAGE_ID(x)			(x & 0xffff)

/* maximum message size for mailbox Tx/Tx */
#define SOF_IPC_MSG_MAX_SIZE			128

/*
 * Command Header - Header for all IPC. Identifies IPC message.
 * The size can be greater than the structure size and that means there is
 * extended bespoke data beyond the end of the structure including variable
 * arrays.
 */

struct sof_ipc_hdr {
	uint32_t cmd;			/* SOF_IPC_GLB_ + cmd */
	uint32_t size;			/* size of structure */
}  __attribute__((packed));

/*
 * Generic reply message. Some commands override this with their own reply
 * types that must include this at start.
 */ 
struct sof_ipc_reply {
	struct sof_ipc_hdr hdr;
	int32_t error;			/* negative error numbers */
}  __attribute__((packed));


/*
 * Compound commands - SOF_IPC_GLB_COMPOUND.
 *
 * Compound commands are sent to the DSP as a single IPC operation. The
 * commands are split into blocks and each block has a header. This header
 * identifies the command type and the number of commands before the next
 * header.
 */

struct sof_ipc_compound_hdr {
	struct sof_ipc_hdr hdr;
	uint32_t count;			/* count of 0 means end of compound sequence */
}  __attribute__((packed));


/*
 * DAI Configuration.
 *
 * Each different DAI type will have it's own structure and IPC cmd.
 */

#define SOF_DAI_FMT_I2S			1 /* I2S mode */
#define SOF_DAI_FMT_RIGHT_J		2 /* Right Justified mode */
#define SOF_DAI_FMT_LEFT_J		3 /* Left Justified mode */
#define SOF_DAI_FMT_DSP_A		4 /* L data MSB after FRM LRC */
#define SOF_DAI_FMT_DSP_B		5 /* L data MSB during FRM LRC */
#define SOF_DAI_FMT_PDM			6 /* Pulse density modulation */

#define SOF_DAI_FMT_CONT		(1 << 4) /* continuous clock */
#define SOF_DAI_FMT_GATED		(0 << 4) /* clock is gated */

#define SOF_DAI_FMT_NB_NF		(0 << 8) /* normal bit clock + frame */
#define SOF_DAI_FMT_NB_IF		(2 << 8) /* normal BCLK + inv FRM */
#define SOF_DAI_FMT_IB_NF		(3 << 8) /* invert BCLK + nor FRM */
#define SOF_DAI_FMT_IB_IF		(4 << 8) /* invert BCLK + FRM */

#define SOF_DAI_FMT_CBM_CFM		(0 << 12) /* codec clk & FRM master */
#define SOF_DAI_FMT_CBS_CFM		(2 << 12) /* codec clk slave & FRM master */
#define SOF_DAI_FMT_CBM_CFS		(3 << 12) /* codec clk master & frame slave */
#define SOF_DAI_FMT_CBS_CFS		(4 << 12) /* codec clk & FRM slave */

#define SOF_DAI_FMT_FORMAT_MASK		0x000f
#define SOF_DAI_FMT_CLOCK_MASK		0x00f0
#define SOF_DAI_FMT_INV_MASK		0x0f00
#define SOF_DAI_FMT_MASTER_MASK		0xf000

/* SSP Configuration Request - SOF_IPC_DAI_SSP_CONFIG */
struct sof_ipc_dai_ssp_params {
	struct sof_ipc_hdr hdr;
	uint32_t mclk;
	uint32_t bclk;
	uint32_t fclk;
	uint16_t ssp_id;
	uint16_t mode;
	uint16_t num_slots;
	uint16_t frame_width;
	uint16_t clk_id;
	uint16_t format;	/* SOF_DAI_FMT_ */
	uint16_t mclk_master;
} __attribute__((packed));

/* HDA Configuration Request - SOF_IPC_DAI_HDA_CONFIG */
struct sof_ipc_dai_hda_params {
	struct sof_ipc_hdr hdr;
	uint32_t hda_id;
	uint32_t mclk;
	/* TODO */
} __attribute__((packed));

/* DMIC Configuration Request - SOF_IPC_DAI_DMIC_CONFIG */
struct sof_ipc_dai_dmic_params {
	struct sof_ipc_hdr hdr;
	uint32_t dmic_id;
	uint32_t mclk;
	/* TODO */
} __attribute__((packed));

/*
 * Stream configuration.
 */

#define SOF_IPC_MAX_CHANNELS			8

/* channel positions - uses same values as ALSA */
enum sof_ipc_chmap {
	SOF_CHMAP_UNKNOWN = 0,
	SOF_CHMAP_NA,		/* N/A, silent */
	SOF_CHMAP_MONO,		/* mono stream */
	SOF_CHMAP_FL,		/* front left */
	SOF_CHMAP_FR,		/* front right */
	SOF_CHMAP_RL,		/* rear left */
	SOF_CHMAP_RR,		/* rear right */
	SOF_CHMAP_FC,		/* front centre */
	SOF_CHMAP_LFE,		/* LFE */
	SOF_CHMAP_SL,		/* side left */
	SOF_CHMAP_SR,		/* side right */
	SOF_CHMAP_RC,		/* rear centre */
	SOF_CHMAP_FLC,		/* front left centre */
	SOF_CHMAP_FRC,		/* front right centre */
	SOF_CHMAP_RLC,		/* rear left centre */
	SOF_CHMAP_RRC,		/* rear right centre */
	SOF_CHMAP_FLW,		/* front left wide */
	SOF_CHMAP_FRW,		/* front right wide */
	SOF_CHMAP_FLH,		/* front left high */
	SOF_CHMAP_FCH,		/* front centre high */
	SOF_CHMAP_FRH,		/* front right high */
	SOF_CHMAP_TC,		/* top centre */
	SOF_CHMAP_TFL,		/* top front left */
	SOF_CHMAP_TFR,		/* top front right */
	SOF_CHMAP_TFC,		/* top front centre */
	SOF_CHMAP_TRL,		/* top rear left */
	SOF_CHMAP_TRR,		/* top rear right */
	SOF_CHMAP_TRC,		/* top rear centre */
	SOF_CHMAP_TFLC,		/* top front left centre */
	SOF_CHMAP_TFRC,		/* top front right centre */
	SOF_CHMAP_TSL,		/* top side left */
	SOF_CHMAP_TSR,		/* top side right */
	SOF_CHMAP_LLFE,		/* left LFE */
	SOF_CHMAP_RLFE,		/* right LFE */
	SOF_CHMAP_BC,		/* bottom centre */
	SOF_CHMAP_BLC,		/* bottom left centre */
	SOF_CHMAP_BRC,		/* bottom right centre */
	SOF_CHMAP_LAST = SOF_CHMAP_BRC,
};

/* common sample rates for use in masks */
#define SOF_RATE_8000	(1 << 0)
#define SOF_RATE_11250	(1 << 1)
#define SOF_RATE_16000	(1 << 2)
#define SOF_RATE_22500	(1 << 3)
#define SOF_RATE_24000	(1 << 4)
#define SOF_RATE_32000	(1 << 5)
#define SOF_RATE_40000	(1 << 6)
#define SOF_RATE_44100	(1 << 7)
#define SOF_RATE_48000	(1 << 8)
#define SOF_RATE_88200	(1 << 9)
#define SOF_RATE_96000	(1 << 10)
#define SOF_RATE_176400	(1 << 11)
#define SOF_RATE_192000	(1 << 12)


/* stream PCM frame format */
enum sof_ipc_frame {
	SOF_IPC_FRAME_S16_LE = 0,
	SOF_IPC_FRAME_S24_4LE,
	SOF_IPC_FRAME_S32_LE,
	SOF_IPC_FRAME_FLOAT,
	/* other formats here */
};

/* stream buffer format */
enum sof_ipc_buffer_format {
	SOF_IPC_BUFFER_INTERLEAVED,
	SOF_IPC_BUFFER_NONINTERLEAVED,
	/* other formats here */
};

/* stream direction */
enum sof_ipc_stream_direction {
	SOF_IPC_STREAM_PLAYBACK = 0,
	SOF_IPC_STREAM_CAPTURE,
};

/* stream ring info */
struct sof_ipc_host_buffer {
	uint32_t phy_addr;
	uint32_t pages;
	uint32_t size;
	uint32_t offset;
} __attribute__((packed));

struct sof_ipc_stream_params {
	struct sof_ipc_host_buffer buffer;
	enum sof_ipc_stream_direction direction;
	enum sof_ipc_frame frame_fmt;
	enum sof_ipc_buffer_format buffer_fmt;
	uint32_t rate;
	uint32_t channels;
	uint32_t sample_size;
	/* for notifying host period has completed - 0 means no period IRQ */
	uint32_t host_period_bytes;
} __attribute__((packed));

/* PCM params info - SOF_IPC_STREAM_PCM_PARAMS */
struct sof_ipc_pcm_params {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;
	struct sof_ipc_stream_params params;
	enum sof_ipc_chmap channel_map[];
}  __attribute__((packed));

/* PCM params info reply - SOF_IPC_STREAM_PCM_PARAMS_REPLY */
struct sof_ipc_pcm_params_reply {
	struct sof_ipc_reply rhdr;
	uint32_t comp_id;
	uint32_t posn_offset;
}   __attribute__((packed));

/* compressed vorbis params - SOF_IPC_STREAM_VORBIS_PARAMS */
struct sof_ipc_vorbis_params {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;
	struct sof_ipc_stream_params params;
	/* TODO */
}  __attribute__((packed));


/* free stream - SOF_IPC_STREAM_PCM_PARAMS */
struct sof_ipc_stream {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;
} __attribute__((packed));


/* flags indicating which time stamps are in sync with each other */
#define	SOF_TIME_HOST_SYNC	(1 << 0)
#define	SOF_TIME_DAI_SYNC	(1 << 1)
#define	SOF_TIME_WALL_SYNC	(1 << 2)
#define	SOF_TIME_STAMP_SYNC	(1 << 3)

/* flags indicating which time stamps are valid */
#define	SOF_TIME_HOST_VALID	(1 << 8)
#define	SOF_TIME_DAI_VALID	(1 << 9)
#define	SOF_TIME_WALL_VALID	(1 << 10)
#define	SOF_TIME_STAMP_VALID	(1 << 11)

/* flags indicating time stamps are 64bit else 3use low 32bit */
#define	SOF_TIME_HOST_64	(1 << 16)
#define	SOF_TIME_DAI_64		(1 << 17)
#define	SOF_TIME_WALL_64	(1 << 18)
#define	SOF_TIME_STAMP_64	(1 << 19)

struct sof_ipc_stream_posn {
	struct sof_ipc_reply rhdr;
	uint32_t comp_id;
	uint32_t flags;		/* SOF_TIME_ */
	uint32_t wallclock_hz;	/* frequency of wallclock in Hz */
	uint32_t timestamp_ns;	/* resolution of timestamp in ns */
	uint64_t host_posn;	/* host DMA position in bytes */
	uint64_t dai_posn;	/* DAI DMA position in bytes */
	uint64_t wallclock;	/* audio wall clock */
	uint64_t timestamp;	/* system time stamp */ 
}  __attribute__((packed));

/*
 * Component Mixers and Controls
 */

struct sof_ipc_ctrl_chan {
	enum sof_ipc_chmap channel;
	uint32_t value;
} __attribute__((packed));

struct sof_ipc_ctrl_values {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;
	uint32_t num_values;
	struct sof_ipc_ctrl_chan values[SOF_IPC_MAX_CHANNELS];
} __attribute__((packed));

struct sof_ipc_ctrl_get_values {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;
	uint32_t num_values;
} __attribute__((packed));

/*
 * Component
 */

/* types of component */
enum sof_comp_type {
	SOF_COMP_NONE = 0,
	SOF_COMP_HOST,
	SOF_COMP_DAI,
	SOF_COMP_SG_HOST,	/* scatter gather variant */
	SOF_COMP_SG_DAI,	/* scatter gather variant */
	SOF_COMP_VOLUME,
	SOF_COMP_MIXER,
	SOF_COMP_MUX,
	SOF_COMP_SRC,
	SOF_COMP_SPLITTER,
	SOF_COMP_TONE,
	SOF_COMP_SWITCH,
	SOF_COMP_BUFFER,
	SOF_COMP_EQ_IIR,
	SOF_COMP_EQ_FIR,
};

/* create new generic component - SOF_IPC_TPLG_COMP_NEW */
struct sof_ipc_comp {
	struct sof_ipc_hdr hdr;
	uint32_t id;
	enum sof_comp_type type;
	uint32_t pipeline_id;
} __attribute__((packed));

/*
 * Component Buffers
 */

/* create new component buffer - SOF_IPC_TPLG_BUFFER_NEW */
struct sof_ipc_buffer {
	struct sof_ipc_comp comp;
	uint32_t size;		/* buffer size in bytes */
} __attribute__((packed));

/* types of DAI */
enum sof_ipc_dai_type {
	SOF_DAI_INTEL_NONE = 0,
	SOF_DAI_INTEL_SSP,
	SOF_DAI_INTEL_DMIC,
	SOF_DAI_INTEL_HDA,
};

/* generic component config data */
struct sof_ipc_comp_config {
	uint32_t format;	/* data format */
	uint32_t frames;	/* number of frames to process, 0 is variable */
	uint32_t channels;	/* max number of channels */
	uint32_t frame_size;	/* sample size in bytes */
	uint32_t periods_sink;	/* 0 means variable */
	uint32_t periods_source;	/* 0 means variable */
	uint32_t preload_count;	/* how many periods to preload */
	enum sof_ipc_frame frame_fmt;
	enum sof_ipc_chmap chmap[SOF_IPC_MAX_CHANNELS];	/* channel map */
} __attribute__((packed));

/* generic host component */
struct sof_ipc_comp_host {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
	enum sof_ipc_stream_direction direction;
	uint32_t no_irq;	/* dont send periodic IRQ to host/DSP */
	uint32_t dmac_id;
	uint32_t dmac_chan;
	uint32_t dmac_config; /* DMA engine specific */
}  __attribute__((packed));

/* generic DAI component */
struct sof_ipc_comp_dai {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
	enum sof_ipc_stream_direction direction;
	uint32_t index;
	enum sof_ipc_dai_type type;
	uint32_t dmac_id;
	uint32_t dmac_chan;
	uint32_t dmac_config; /* DMA engine specific */
}  __attribute__((packed));

/* generic mixer component */
struct sof_ipc_comp_mixer {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
}  __attribute__((packed));

/* volume ramping types */
enum sof_volume_ramp {
	SOF_VOLUME_LINEAR	= 0,
	SOF_VOLUME_LOG,
	SOF_VOLUME_LINEAR_ZC,
	SOF_VOLUME_LOG_ZC,
};

/* generic volume component */
struct sof_ipc_comp_volume {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
	uint32_t channels;
	int32_t min_value;
	int32_t max_value;
	enum sof_volume_ramp ramp;
	uint32_t initial_ramp;	/* ramp space in ms */
}  __attribute__((packed));

/* generic SRC component */
struct sof_ipc_comp_src {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
	/* either source or sink rate must be non zero */
	uint32_t source_rate;	/* source rate or 0 for variable */
	uint32_t sink_rate;	/* sink rate or 0 for variable */
	uint32_t rate_mask;	/* SOF_RATE_ supported rates */
} __attribute__((packed));

/* generic MUX component */
struct sof_ipc_comp_mux {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
} __attribute__((packed));

/* generic tone generator component */
struct sof_ipc_comp_tone {
	struct sof_ipc_comp comp;
	struct sof_ipc_comp_config config;
	int32_t frequency;
	int32_t amplitude;
	int32_t freq_mult;
	int32_t ampl_mult;
	int32_t length;
	int32_t period;
	int32_t repeats;
	int32_t ramp_step;
} __attribute__((packed));

/* FIR equalizer component */
struct sof_ipc_comp_eq_fir {
       struct sof_ipc_comp comp;
       struct sof_ipc_comp_config config;
} __attribute__((packed));

/* IIR equalizer component */
struct sof_ipc_comp_eq_iir {
       struct sof_ipc_comp comp;
       struct sof_ipc_comp_config config;
} __attribute__((packed));

/* IPC to pass configuration blobs to equalizers and re-assign responses */
struct sof_ipc_eq_fir_blob {
	struct sof_ipc_comp comp;
	struct sof_ipc_host_buffer buffer;
	int32_t data[];
} __attribute__((packed));

struct sof_ipc_eq_iir_blob {
	struct sof_ipc_comp comp;
	struct sof_ipc_host_buffer buffer;
	int32_t data[];
} __attribute__((packed));

struct sof_ipc_eq_fir_switch {
	struct sof_ipc_comp comp;
	int32_t data[];
} __attribute__((packed));

struct sof_ipc_eq_iir_switch {
	struct sof_ipc_comp comp;
	int32_t data[];
} __attribute__((packed));

/* frees components, buffers and pipelines
 * SOF_IPC_TPLG_COMP_FREE, SOF_IPC_TPLG_PIPE_FREE, SOF_IPC_TPLG_BUFFER_FREE
 */
struct sof_ipc_free {
	struct sof_ipc_hdr hdr;
	uint32_t id;
} __attribute__((packed));


struct sof_ipc_comp_reply {
	struct sof_ipc_reply rhdr;
	uint32_t id;
	uint32_t offset;
} __attribute__((packed));


/*
 * Pipeline
 */

/* new pipeline - SOF_IPC_TPLG_PIPE_NEW */
struct sof_ipc_pipe_new {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;	/* component id for pipeline */
	uint32_t pipeline_id;	/* pipeline id */
	uint32_t sched_id;	/* sheduling component id */
	uint32_t core;		/* core we run on */
	uint32_t deadline;	/* execution completion deadline in us*/
	uint32_t priority;	/* priority level 0 (low) to 10 (max) */
	uint32_t mips;		/* worst case instruction count per period */
	uint32_t frames_per_sched; /* output frames of pipeline, 0 is variable */
}  __attribute__((packed));

/* pipeline construction complete - SOF_IPC_TPLG_PIPE_COMPLETE */
struct sof_ipc_pipe_ready {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;
}  __attribute__((packed));


struct sof_ipc_pipe_free {
	struct sof_ipc_hdr hdr;
	uint32_t comp_id;
}  __attribute__((packed));

/* connect two components in pipeline - SOF_IPC_TPLG_COMP_CONNECT */
struct sof_ipc_pipe_comp_connect {
	struct sof_ipc_hdr hdr;
	uint32_t source_id;
	uint32_t sink_id;
}  __attribute__((packed));


/*
 * PM
 */

/* PM context element */
struct sof_ipc_pm_ctx_elem {
	uint32_t type;
	uint32_t size;
	uint64_t addr;
}  __attribute__((packed));

/* PM context - SOF_IPC_PM_CTX_SAVE, SOF_IPC_PM_CTX_RESTORE,
 * SOF_IPC_PM_CTX_SIZE */
struct sof_ipc_pm_ctx {
	struct sof_ipc_hdr hdr;
	struct sof_ipc_host_buffer buffer;
	uint32_t num_elems;
	uint32_t size;
	struct sof_ipc_pm_ctx_elem elems[];
};

/*
 * Firmware boot and version
 */

#define SOF_IPC_MAX_ELEMS	16

/* extended data types that can be appended onto end of sof_ipc_fw_ready */
enum sof_ipc_ext_data {
	SOF_IPC_EXT_DMA_BUFFER = 0,
	SOF_IPC_EXT_WINDOW,
};

/* FW version - SOF_IPC_GLB_VERSION */
struct sof_ipc_fw_version {
	uint16_t major;
	uint16_t minor;
	uint16_t build;
	uint8_t date[12];
	uint8_t time[10];
	uint8_t tag[6];
} __attribute__((packed));

/* FW ready Message - sent by firmware when boot has completed */
struct sof_ipc_fw_ready {
	struct sof_ipc_hdr hdr;
	uint32_t inbox_offset;
	uint32_t outbox_offset;
	uint32_t inbox_size;
	uint32_t outbox_size;
	struct sof_ipc_fw_version version;
} __attribute__((packed));

/*
 * Extended Firmware data. All optional, depends on platform/arch.
 */

enum sof_ipc_region {
	SOF_IPC_REGION_DOWNBOX	= 0,
	SOF_IPC_REGION_UPBOX,
	SOF_IPC_REGION_TRACE,
	SOF_IPC_REGION_DEBUG,
	SOF_IPC_REGION_STREAM,
	SOF_IPC_REGION_REGS,
};

struct sof_ipc_ext_data_hdr {
	struct sof_ipc_hdr hdr;
	enum sof_ipc_ext_data type;			/* SOF_IPC_EXT_ */
};

struct sof_ipc_dma_buffer_elem {
	enum sof_ipc_region type;
	uint32_t id;	/* platform specific - used to map to host memory */
	struct sof_ipc_host_buffer buffer;
};

/* extended data DMA buffers for IPC, trace and debug */
struct sof_ipc_dma_buffer_data {
	struct sof_ipc_ext_data_hdr ext_hdr;
	uint32_t num_buffers;
	struct sof_ipc_dma_buffer_elem buffer[];	/* host files in buffer[n].buffer */
}  __attribute__((packed));


struct sof_ipc_window_elem {
	enum sof_ipc_region type;
	uint32_t id;	/* platform specific - used to map to host memory */
	uint32_t flags;	/* R, W, RW, etc - to define */
	uint32_t size;	/* size of region in bytes */
	uint32_t offset; /* offset in window region as windows can be partitioned */
};

/* extended data memory windows for IPC, trace and debug */
struct sof_ipc_window {
	struct sof_ipc_ext_data_hdr ext_hdr;
	uint32_t num_windows;
	struct sof_ipc_window_elem window[];
}  __attribute__((packed));

#endif

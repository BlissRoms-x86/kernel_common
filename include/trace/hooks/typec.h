/* SPDX-License-Identifier: GPL-2.0 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM typec
#define TRACE_INCLUDE_PATH trace/hooks
#if !defined(_TRACE_HOOK_TYPEC_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_HOOK_TYPEC_H
#include <linux/tracepoint.h>
#include <trace/hooks/vendor_hooks.h>
/*
 * Following tracepoints are not exported in tracefs and provide a
 * mechanism for vendor modules to hook and extend functionality
 */
struct tcpci;
struct tcpci_data;

#ifndef TYPEC_TIMER
#define TYPEC_TIMER
enum typec_timer {
	SINK_WAIT_CAP,
	SOURCE_OFF,
	CC_DEBOUNCE,
	SINK_DISCOVERY_BC12,
};
#endif

DECLARE_HOOK(android_vh_typec_tcpci_override_toggling,
	TP_PROTO(struct tcpci *tcpci, struct tcpci_data *data, int *override_toggling),
	TP_ARGS(tcpci, data, override_toggling));

DECLARE_RESTRICTED_HOOK(android_rvh_typec_tcpci_chk_contaminant,
	TP_PROTO(struct tcpci *tcpci, struct tcpci_data *data, int *ret),
	TP_ARGS(tcpci, data, ret), 1);

/*
 * This hook is for addressing hardware anomalies where TCPC_POWER_STATUS_VBUS_PRES bit can return 0
 * even before falling below sSinkDisconnect threshold.
 * Handler has to set bypass to override the value that would otherwise be returned by this
 * function.
 * Handler can set vbus or clear vbus to indicate vbus present or absent
 */
DECLARE_RESTRICTED_HOOK(android_rvh_typec_tcpci_get_vbus,
	TP_PROTO(struct tcpci *tcpci, struct tcpci_data *data, int *vbus, int *bypass),
	TP_ARGS(tcpci, data, vbus, bypass), 1);

DECLARE_HOOK(android_vh_typec_tcpm_get_timer,
	TP_PROTO(const char *state, enum typec_timer timer, unsigned int *msecs),
	TP_ARGS(state, timer, msecs));

#endif /* _TRACE_HOOK_UFSHCD_H */
/* This part must be outside protection */
#include <trace/define_trace.h>

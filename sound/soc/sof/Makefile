snd-sof-objs := core.o ops.o loader.o ipc.o pcm.o pm.o debug.o topology.o\
		control.o stream.o trace.o compressed.o
snd-sof-intel-byt-objs := hw-byt.o
snd-sof-intel-hsw-objs := hw-hsw.o
snd-sof-intel-bdw-objs := hw-bdw.o
snd-sof-intel-apl-objs := hw-apl.o

snd-sof-pci-objs := sof-pci-dev.o
snd-sof-acpi-objs := sof-acpi-dev.o
snd-sof-platform-objs := sof-platform-dev.o
snd-sof-nocodec-objs := nocodec.o

obj-$(CONFIG_SND_SOC_SOF) += snd-sof.o
obj-$(CONFIG_SND_SOC_SOF_NOCODEC) += snd-sof-nocodec.o

obj-$(CONFIG_SND_SOC_SOF_ACPI) += sof-acpi-dev.o
obj-$(CONFIG_SND_SOC_SOF_PCI) += sof-pci-dev.o
obj-$(SND_SOC_SOF_PLATFORM) += sof-platform-dev.o

obj-$(CONFIG_SND_SOC_SOF_BAYTRAIL) += snd-sof-intel-byt.o
obj-$(CONFIG_SND_SOC_SOF_HASWELL) += snd-sof-intel-hsw.o
obj-$(CONFIG_SND_SOC_SOF_BROADWELL) += snd-sof-intel-bdw.o
obj-$(CONFIG_SND_SOC_SOF_APOLLOLAKE) += snd-sof-intel-apl.o

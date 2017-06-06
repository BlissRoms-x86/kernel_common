ccflags-y := -Iinclude/drm -I$(src)/osindependent

vboxvideo-y :=  osindependent/HGSMIBase.o osindependent/Modesetting.o \
		osindependent/VBVABase.o vbox_drv.o vbox_fb.o vbox_hgsmi.o \
		vbox_irq.o vbox_main.o vbox_mode.o vbox_prime.o vbox_ttm.o

obj-$(CONFIG_DRM_VBOXVIDEO) += vboxvideo.o

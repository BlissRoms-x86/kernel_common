#
# Makefile for multiplexer devices.
#

mux-core-objs			:= core.o
mux-adg792a-objs		:= adg792a.o
mux-gpio-objs			:= gpio.o
mux-mmio-objs			:= mmio.o
mux-intel_cht_usb_mux-objs	:= intel_cht_usb_mux.o

obj-$(CONFIG_MULTIPLEXER)	+= mux-core.o
obj-$(CONFIG_MUX_ADG792A)	+= mux-adg792a.o
obj-$(CONFIG_MUX_GPIO)		+= mux-gpio.o
obj-$(CONFIG_MUX_MMIO)		+= mux-mmio.o
obj-$(CONFIG_MUX_CHT_USB_MUX)	+= mux-intel_cht_usb_mux.o

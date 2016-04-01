
obj-$(CONFIG_CHROMEOS)		+= chromeos.o
obj-$(CONFIG_CHROMEOS_OF_FIRMWARE) += chromeos_arm.o
obj-$(CONFIG_CHROMEOS_LAPTOP)	+= chromeos_laptop.o
obj-$(CONFIG_CHROMEOS_PSTORE)	+= chromeos_pstore.o
cros_ec_devs-objs		:= cros_ec_dev.o cros_ec_sysfs.o \
				   cros_ec_lightbar.o cros_ec_vbc.o
obj-$(CONFIG_CROS_EC_CHARDEV)   += cros_ec_devs.o
cros_ec_lpcs-objs		+= cros_ec_lpc.o cros_ec_lpc_reg.o
obj-$(CONFIG_CROS_EC_LPC)       += cros_ec_lpcs.o
obj-$(CONFIG_CROS_EC_LPC_MEC)	+= cros_ec_lpc_mec.o
obj-$(CONFIG_CROS_EC_PROTO)	+= cros_ec_proto.o

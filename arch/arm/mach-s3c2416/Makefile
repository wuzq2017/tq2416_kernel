# arch/arm/mach-s3c2416/Makefile
#
# Copyright 2009 Yauhen Kharuzhy <jekhor@gmail.com>
#
# Licensed under GPLv2

obj-y				:=
obj-m				:=
obj-n				:=
obj-				:=

obj-$(CONFIG_CPU_S3C2416)	+= s3c2416.o clock.o
obj-$(CONFIG_CPU_S3C2416)	+= irq.o
obj-$(CONFIG_S3C2416_PM)	+= pm.o
obj-$(CONFIG_S3C2416_DMA)	+= dma.o

# Device setup
obj-$(CONFIG_S3C2416_SETUP_SDHCI) += setup-sdhci.o
obj-$(CONFIG_S3C2416_SETUP_SDHCI_GPIO) += setup-sdhci-gpio.o

#keypad
obj-$(CONFIG_KEYBOARD_TQ2416)	+= dev-kaypad.o
# Machine support

obj-$(CONFIG_MACH_SMDK2416)	+= mach-smdk2416.o
obj-$(CONFIG_MACH_TQ2416)	+= mach-tq2416.o


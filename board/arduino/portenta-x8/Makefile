#
# Copyright 2018 NXP
#
# SPDX-License-Identifier:      GPL-2.0+
#

obj-y += portenta-x8.o

obj-y += mmc.o

ifdef CONFIG_SPL_BUILD
obj-y += spl.o
obj-y += lpddr4_timing.o
endif

obj-$(CONFIG_USB_TCPC) += tcpc.o

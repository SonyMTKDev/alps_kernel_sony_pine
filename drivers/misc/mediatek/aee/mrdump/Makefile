#
# Copyright (C) 2016 MediaTek Inc.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See http://www.gnu.org/licenses/gpl-2.0.html for more details.
#

ifeq (y, $(filter y, $(CONFIG_ARCH_MT6570) $(CONFIG_ARCH_MT6757)))
obj-$(CONFIG_MTK_AEE_FEATURE)	+= mrdump_hw.o
endif

ifeq ($(CONFIG_MTK_AEE_MRDUMP),y)
obj-y += ramdump.o
ramdump-y := mrdump_full.o mrdump_setup.o
obj-$(CONFIG_ARM) += mrdump_arm.o
obj-$(CONFIG_ARM64) += mrdump_arm64.o
endif

#MRDUMP_KEY is independent feature
#it is not depend on MRDUMP
# obj-$(CONFIG_MTK_AEE_FEATURE) += mrdump_key.o
# mrdump_key-y := mrdump_key_setup.o

obj-$(CONFIG_MTK_AEE_IPANIC) += mrdump_mini.o mrdump_control.o

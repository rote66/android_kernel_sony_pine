#
# Copyright (C) 2015 MediaTek Inc.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License version 2 as
# published by the Free Software Foundation.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
# GNU General Public License for more details.
#

# In case the platform does NOT support this type of sensors
ccflags-y += -I$(srctree)/drivers/misc/mediatek/hwmon/include

obj-y	+= step_counter.o
#[SM31][Sensors][JasonHsing] G-sensor bma422 driver porting 20161012 --
obj-$(CONFIG_MTK_BMA4XYSTC)   +=  bma4xy_stc/

ifeq ($(CONFIG_MTK_SCP_SENSORHUB_V1),y)
obj-$(CONFIG_MTK_STEPSIGNHUB)		+=	stepsignhub/
endif

ifeq ($(CONFIG_CUSTOM_KERNEL_STEP_COUNTER),"OTHER_VENDOR")
obj-y += other_vendor/
endif

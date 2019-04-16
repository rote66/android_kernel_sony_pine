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

# [Arima_8100][bozhi_lin] support type-c pd charger and set current to 1.5A 20161222 begin
ccflags-$(CONFIG_TCPC_CLASS) += -I$(srctree)/drivers/misc/mediatek/pd_rt1711/inc
# [Arima_8100][bozhi_lin] 20161222 end

#obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o pmic_chr_type_det.o mt6311.o
obj-$(CONFIG_MTK_PMIC)          += pmic.o upmu_common.o pmic_auxadc.o mt6311.o

ifeq ($(CONFIG_MTK_PMIC_DVT_SUPPORT),y)
     obj-$(CONFIG_MTK_PMIC)     += pmic_dvt.o
endif

obj-$(CONFIG_MTK_SMART_BATTERY) += battery_meter_hal.o pmic_chr_type_det.o

ifeq ($(CONFIG_MTK_BQ24261_SUPPORT),y)
     obj-$(CONFIG_MTK_SMART_BATTERY) += bq24261.o charging_hw_bq24261.o
else
     ifeq ($(CONFIG_MTK_SWCHR_SUPPORT),y)
          obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_mt_sw_charger.o
     else
          ifeq ($(CONFIG_MTK_NCP1854_SUPPORT),y)
               obj-$(CONFIG_MTK_SMART_BATTERY) += ncp1854.o charging_hw_ncp1854.o
          else
               ifeq ($(CONFIG_MTK_BQ24296_SUPPORT),y)
                    obj-$(CONFIG_MTK_SMART_BATTERY) += bq24296.o charging_hw_bq24296.o
               else
                    ifeq ($(CONFIG_MTK_FAN5405_SUPPORT),y)
                        obj-$(CONFIG_MTK_SMART_BATTERY) += fan5405.o charging_hw_fan5405.o
                    else
                         ifeq ($(CONFIG_MTK_RT9536_SUPPORT),y)
                             obj-$(CONFIG_MTK_SMART_BATTERY) += rt9536.o charging_hw_rt9536.o
                         else
# [Arima_8100][bozhi_lin] RichTek rt9458 charging ic driver porting 20160921 begin
                             ifeq ($(CONFIG_MTK_RT9458_SUPPORT),y)
                                 obj-$(CONFIG_MTK_SMART_BATTERY) += rt9458.o charging_hw_rt9458.o
                             else
                                 obj-$(CONFIG_MTK_SMART_BATTERY) += charging_hw_pmic.o
                             endif
# [Arima_8100][bozhi_lin] 20160921 end
                         endif
                    endif
               endif
          endif
     endif
endif

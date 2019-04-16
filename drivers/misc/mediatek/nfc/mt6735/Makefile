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

#obj-$(CONFIG_NFC_MT6605)   += mt6605.o
# [NFC][Kent][16090701][begin] Proting NFC PN553 
#obj-y   += mt6605.o disable mtk NFC
# [NFC][Kent][16090701][end] Proting NFC PN553 
ccflags-$(CONFIG_NFC_MT6605) +=  -DDEBUG
# [NFC][Kent][16090701][begin] Proting NFC PN553 
obj-$(CONFIG_PN553_NFC)    += pn553.o  
# [NFC][Kent][16090701][end] Proting NFC PN553 

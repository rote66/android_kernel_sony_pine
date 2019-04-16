# include $(srctree)/drivers/misc/mediatek/Makefile.custom

# Linux driver folder
ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/
ccflags-y += -I$(srctree)/drivers/misc/mediatek/include/mt-plat/$(MTK_PLATFORM)/include/
obj-$(CONFIG_TOUCHSCREEN_NT11206_1080_1920)    +=	NVTtouch_206.o \
													NVTtouch_206_ext_proc.o \
													NVTtouch_206_fw_update.o \
													NVTtouch_206_mp_ctrlram.o

# Linux driver folder
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/NT11206/
#ccflags-y += -I$(srctree)/drivers/input/touchscreen/mediatek/
#obj-y	+=  NVTtouch_206.o
#obj-y	+=  NVTtouch_206_ext_proc.o
#obj-y	+=  NVTtouch_206_fw_update.o
#obj-y	+=  NVTtouch_206_mp_ctrlram.o

#!/bin/sh
#modify by rote66
echo 536870912 > /sys/block/zram0/disksize
#echo 335544320 > /sys/block/zram0/disksize
/system/bin/mkswap /dev/block/zram0
/system/bin/swapon /dev/block/zram0

#Added by rote66 for customized EMP kernel
#set swappiness
echo 100 > /proc/sys/vm/swappiness

#Enable Ksm
echo 100 > /sys/kernel/mm/ksm/pages_to_scan
echo 500 > /sys/kernel/mm/ksm/sleep_millisecs
echo 1 > /sys/kernel/mm/ksm/run

#Zswap
echo 24 > /sys/module/zswap/parameters/max_pool_percent
echo Y > /sys/module/zswap/parameters/enabled
#END

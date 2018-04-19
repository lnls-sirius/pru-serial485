#!/usr/bin/env sh

 #DDR size for PRU serial interface: 2 097 152 bytes
modprobe uio_pruss extram_pool_sz=0x200000

echo PRUserial485 > /sys/devices/bone_capemgr.9/slots
echo ADDRserial485 > /sys/devices/bone_capemgr.9/slots

cat /sys/devices/bone_capemgr.9/slots


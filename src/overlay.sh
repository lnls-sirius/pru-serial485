#!/usr/bin/env sh
/sbin/modprobe uio_pruss
echo PRUserial485 > /sys/devices/bone_capemgr.9/slots

cat /sys/devices/bone_capemgr.9/slots


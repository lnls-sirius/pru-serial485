#!/usr/bin/env sh
/sbin/modprobe uio_pruss
echo pru_485_enable > /sys/devices/bone_capemgr.9/slots

cat /sys/devices/bone_capemgr.9/slots


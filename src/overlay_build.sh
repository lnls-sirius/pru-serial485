#!/bin/sh

export SLOTS=/sys/devices/bone_capemgr.9/slots
export PINS=/sys/kernel/debug/pinctrl/44e10800.pinmux/pins

echo "Compilando Device Tree Overlay..."

dtc -O dtb -o pru_485_enable-00A0.dtbo -b 0 -@ pru_485_enable-00A0.dts

cp pru_485_enable-00A0.dtbo /lib/firmware

rm pru_485_enable-00A0.dtbo

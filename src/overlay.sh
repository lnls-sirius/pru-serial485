#!/bin/sh


#DDR size for PRU serial interface: 2 097 152 bytes
modprobe uio_pruss extram_pool_sz=0x200000

KERNEL_VERSION=`uname -r`;
if [ "${KERNEL_VERSION%.*}" = "3.8" ]; then
    echo "Configuring pins for kernel 3.8.x"
    echo PRUserial485 > /sys/devices/bone_capemgr.9/slots
    echo ADDRserial485 > /sys/devices/bone_capemgr.9/slots

elif [ "${KERNEL_VERSION%.*}" = "4.14" ]; then
    echo "Configuring pins for kernel 4.14.x"
    # PRUserial485 pins
    config-pin P8_46 pruin          # MISO
    config-pin P8_39 pruin          # IRQ
    config-pin P8_27 pruin          # SYNC
    config-pin P8_45 pruout         # CLK
    config-pin P8_43 pruout         # CS
    config-pin P8_41 pruout         # MOSI
    config-pin P8_29 pruout         # LED WRITE
    config-pin P8_40 pruout         # LED READ
    # ADDRserial485 PRU address
    config-pin P8_31 in_pd          # SW address 1
    config-pin P8_32 in_pd          # SW address 2
    config-pin P8_33 in_pd          # SW address 3
    config-pin P8_34 in_pd          # SW address 4
    config-pin P8_35 in_pd          # SW address 5
else
    echo "Version not supported"
fi


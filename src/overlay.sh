#!/bin/sh


#DDR size for PRU serial interface: 2 097 152 bytes
DESIRED_POOL_SIZE="0x00200000"
MAP1_SIZE_FILE="/sys/class/uio/uio0/maps/map1/size"

if [ -f "$MAP1_SIZE_FILE" ]; then
    CURRENT_POOL_SIZE=$(cat "$MAP1_SIZE_FILE")
    if [ "$CURRENT_POOL_SIZE" = "$DESIRED_POOL_SIZE" ]; then
        echo "uio_pruss extram pool already correctly sized ($CURRENT_POOL_SIZE), skipping reload."
    else
        echo "uio_pruss extram pool wrong size ($CURRENT_POOL_SIZE), forcing reload..."
        rmmod -f uio_pruss
        modprobe uio_pruss extram_pool_sz=0x200000
    fi
else
    echo "uio_pruss not loaded yet, loading with correct pool size..."
    modprobe uio_pruss extram_pool_sz=0x200000
fi

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
    config-pin P9_17 pru_uart
    config-pin P9_17 pru_uart
    config-pin P9_24 pru_uart
    config-pin P9_26 pru_uart
    config-pin P9_27 pruout
else
    echo "Version not supported"
fi


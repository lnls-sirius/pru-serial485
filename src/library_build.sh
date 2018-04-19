#!/usr/bin/env sh
cd library


KERNEL_VERSION=`uname -r`;
if [ "${KERNEL_VERSION%.*}" = "3.8" ]; then
    echo "."
    echo ".."
    echo "..."
    echo "Building Device Tree Overlay..."
    ./overlay_build.sh
    echo "OK"
fi

echo "."
echo ".."
echo "..."
echo "Building and installing C library..."
gcc -mfloat-abi=hard -Wall -fPIC -O2 -mtune=cortex-a8 -march=armv7-a -I/usr/include -c -o PRUserial485.o PRUserial485.c
ar -rv libPRUserial485.a PRUserial485.o
gcc -shared -Wl,-soname, -o libPRUserial485.so PRUserial485.o
pasm -V3 -b PRUserial485.p

install -m0755 libPRUserial485.a libPRUserial485.so /usr/lib
ldconfig -n /usr/lib/libPRUserial485.*
install -m0755 PRUserial485.h /usr/include

mv PRUserial485.bin /usr/bin
rm PRUserial485.o libPRUserial485.so libPRUserial485.a
echo "OK"


echo "."
echo ".."
echo "..."
echo "Building and installing Python library..."
./setup.py install
echo "OK"

echo "."
echo ".."
echo "..."
echo "Done!"

cd ..

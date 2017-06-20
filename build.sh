#!/bin/sh

export ARCH=arm 
#export CROSS_COMPILE=arm-linux-gnueabihf-

export RELEASE_DIR=./deploy
#CPU_JOB_NUM=$(grep processor /proc/cpuinfo | awk '{field=$NF};END{print field+1}')

#make m0skt_00_defconfig
make kcppk_defconfig

CPU_JOB_NUM=2
echo make -j$CPU_JOB_NUM
make -j$CPU_JOB_NUM 

echo "Make modules"

make -j1 zImage LOCALVERSION=
make -j1 modules_install INSTALL_MOD_PATH=$RELEASE_DIR/mod LOCALVERSION=

echo
echo "Copying..."$RELEASE_DIR
cp -f ./arch/arm/boot/zImage $RELEASE_DIR/zImage
echo
echo "Copy done"


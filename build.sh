## Config
# make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi- tq2440_defconfig

## Build and deploy
make ARCH=arm CROSS_COMPILE=arm-linux-gnueabi-  -j8 uImage &&  cp arch/arm/boot/uImage ~/tftpboot/uImage-3.7

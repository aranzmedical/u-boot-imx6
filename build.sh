ARCH=arm CROSS_COMPILE=arm-fslc-linux-gnueabi- make clean
ARCH=arm CROSS_COMPILE=arm-fslc-linux-gnueabi- make mrproper
ARCH=arm CROSS_COMPILE=arm-fslc-linux-gnueabi- make silhouettestar2_defconfig
ARCH=arm CROSS_COMPILE=arm-fslc-linux-gnueabi- make

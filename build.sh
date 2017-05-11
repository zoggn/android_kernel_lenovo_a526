#!/bin/bash

#get changes from  GitHub 
#echo 'Get changes from GitHub'
#git pull
#echo 'Start building...'

# Set options
export ARCH=arm CROSS_COMPILE=../android_toolchain_hf/bin/arm-cortex_a7-linux-gnueabihf-
export KBUILD_BUILD_USER=assusdan
export KBUILD_BUILD_HOST=SRT

    #For checking errors
echo 'Remove kernel...'
rm -rf arch/arm/boot/zImage


echo 'Configure Pure F'
make aeon6582_wt_nj_l_defconfig >/dev/null

echo 'Building Pure F...'
make -j4 zImage >/dev/null 2>buildlog.log

    #check errors
if [ ! -f arch/arm/boot/zImage ]
then
    echo "BUID ERRORS!"

else
    #if OK
echo 'Moving Pure F'
mv arch/arm/boot/zImage /var/www/compiled/zImage-puref

fi


#write worktime
echo $[$SECONDS / 60]' minutes'
echo $[$SECONDS % 60]' seconds' 


#echo 'Configure A'
#make zerasra_defconfig >/dev/null
#echo 'Build A'
#make -j4 zImage >/dev/null 2>buildlog.log
#echo 'Moving A'
#mv arch/arm/boot/zImage /var/www/html/zImage_A 

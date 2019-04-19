#!/bin/bash
echo "For L1 source clean"
make mrproper -k -i
make clean -k -i
find -name ".*.su" | xargs rm -f
#rm .*.su
#rm build.log
#rm arch/arm64/boot/Image.gz-dtb
#rm install_kernel/zImage
#rm install_kernel/oldimg
echo "done"

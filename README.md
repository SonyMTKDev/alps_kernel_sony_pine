WHAT IS THIS?
=============

Linux Kernel source code done by @jmpfbmx for the devices:
* Sony XPERIA L1


BUILD INSTRUCTIONS?
===================

Specific sources are separated by releases with it's corresponding number. First, you should
clone the project:

        $ git clone -b ALPS-MP_O1-SONY https://github.com/PineDevelopment/alps_kernel_sony_pine kernel

At the same level of the "kernel" directory:

Download a prebuilt gcc

        $ git clone https://android.googlesource.com/platform/prebuilts/gcc/linux-x86/aarch64/aarch64-linux-android-4.9 -b oreo-release 

Create KERNEL_OUT dir:

        $ mkdir KERNEL_OUT 
  
Your directory tree should look like this:
* kernel
* aarch64-linux-android-4.9
* KERNEL_OUT

## Building
Finally, build the kernel according the next table of product names:

| device                    | product                 |
| --------------------------|-------------------------|
| Sony XPERIA L1            | pine                    |
| Sony XPERIA L1 DEBUG      | pine_debug              |
| Sony XPERIA L1 TWRP       | pine_twrp               |


        $ make -C kernel  O=../KERNEL_OUT  ARCH=arm64 CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-android- <product>_defconfig
        $ make O=../KERNEL_OUT/ -C kernel ARCH=arm64  CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-android-
    
You can specify "-j CORES" argument to speed-up your compilation, example:

        $ make O=../KERNEL_OUT/ -C kernel ARCH=arm64  CROSS_COMPILE=../aarch64-linux-android-4.9/bin/aarch64-linux-android- -j28

### Acknowledgements:
* [@jmpfbmx](https://github.com/jmpfbmx)
* [@Ruben1863](https://github.com/Ruben1863)

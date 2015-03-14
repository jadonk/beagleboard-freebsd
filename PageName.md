# Introduction #

Very brief instructions on how to build the FreeBSD kernel using the code in the repository and then booting it on the BeagleBoard.


# Disclaimer #

These instructions work for me, however it's entirely likely that I may have missed some files that I should have uploaded to the repo, or my build system produces different binaries.

In addition, I build my kernel on OSX using a modified CodeSourcery ARM toolchain, I haven't tried building the kernel on an actual FreeBSD system.


# Building the Kernel #

Copy the files from the repo into the FreeBSD tree, I suggest you use some graphical merge program as some of my changes might not be applicable and unnecessary.  To build type the following command.

<pre>   make buildkernel KERNCONF=BEAGLEBOARD TARGET=arm MACHINE_ARCH=arm</pre>

If you've already built the kernel once and just modified one or two files you can run the following commands which just builds the changed files and re-links.

<pre>   make buildkernel KERNFAST=BEAGLEBOARD TARGET=arm MACHINE_ARCH=arm</pre>

The build process should produce a **kernel.bin** binary file and a **kernel** ELF file.


# Running the Kernel #

## Running the flat binary kernel ##

Copy the **kernel.bin** file onto the root directory of a MMC or SD card and chuck it in the BeagleBoard. At the u-boot prompt, type the following:

<pre>
*OMAP3 beagleboard.org #* mmc init<br>
*OMAP3 beagleboard.org #* fatload mmc 0 0x80200000 kernel.bin<br>
*OMAP3 beagleboard.org #* go 0x80200000<br>
</pre>

_Note:_ On older versions of the Beagleboard and u-boot the _mmc init_ command might be just _mmcinit_. i.e.

<pre>
*OMAP3 beagleboard.org #* mmcinit<br>
*OMAP3 beagleboard.org #* fatload mmc 0 0x80200000 kernel.bin<br>
*OMAP3 beagleboard.org #* go 0x80200000<br>
</pre>


After that you should see the kernel start-up messages coming out the serial port.

## Running the ELF kernel ##

The process is a bit more complex, because u-boot expects a special
header in front of ELF files. You will need the mkimage binary that
can be installed from ports in _devel/u-boot. To get it installed,
type in the following commands:_

<pre>
*host %* cd /usr/ports/devel/u-boot<br>
*host %* sudo make install<br>
</pre>

Once mkimage is installed, you can convert your **kernel** image file
into an image that u-boot can load and run. Since u-boot doesn't
support FreeBSD images, the following command instructs it to treat
the kernel as a NetBSD image. Luckily, the behavior of u-boot in the
case of a NetBSD image is generic enough so that things work anyway.

<pre>
*host %* mkimage -A arm -O netbsd -T kernel -C none -a 80200000 -e 802000e0 -n "FreeBSD" -d _path_to_kernel_/kernel ukernel<br>
</pre>

You should obtain a **ukernel** file, that you can stick into the FAT
partition of your SD card. Once you get to the u-boot prompt, type in
the following commands:

<pre>
*OMAP3 beagleboard.org #* mmcinit<br>
*OMAP3 beagleboard.org #* fatload mmc 0:1 81000000 ukernel<br>
*OMAP3 beagleboard.org #* bootm 81000000<br>
</pre>

You should see the kernel traces as well.

## Reducing the amount of typing ##

Instead of typing this each time, it is of course possible to set the _bootcmd_ environment variable whose content u-boot will execute if the _boot_ command is used.

At the U-boot prompt, just type:
```
OMAP3 beagleboard.org # setenv bootcmd 'mmcinit ; fatload mmc 0:1 81000000 ukernel ; bootm 81000000'
```
Beware, on some versions of U-boot, spaces are important. Then, save this environment variable to persistent storage so that it's available across reboots by typing:
```
OMAP3 beagleboard.org # saveenv
Saving Environment to NAND...
Erasing Nand...Writing to Nand... done
```
Now, and each time you start the board, you only have to type:
```
OMAP3 beagleboard.org # boot
```
U-boot also supports automatically executing the content of _bootcmd_ when starting up, but it is really not recommended at this stage of the development process.
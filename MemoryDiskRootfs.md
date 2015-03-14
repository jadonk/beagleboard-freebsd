# Introduction #

This page explains how to build a ram/memory disk image for FreeBSD on the BeagleBoard.


# Pitfalls #

  * When using an uncompressed kernel image, the kernel is _huge_ (several megabytes, just for a fairly limited system based on the 'rescue' binary). Because the kernel load address is at 80200000, and because U-boot is relocating an [ELF kernel](PageName.md) from it load address to 80200000, you must be careful that the two images do not overlap during the relocation process.

# Creating a disk image #

Start by allocating the space for the image file by filling it with zeroes.

```
host % dd if=/dev/zero of=rootfs.img bs=1M count=n
```

Where _n_ is the number of megabytes your image is. If the image is too big, it's going to take a lot of time to load, and if it's too small it won't hold the base system. 5 Megabytes is good for the moment.

Then, you must create a file system image on it.

```
host % mdconfig -a -t vnode -f rootfs.img
md0
host % bsdlabel -w /dev/md0 auto
host % newfs /dev/md0a
/dev/md0a: 5.0MB (10224 sectors) block size 16384, fragment size 2048
        using 4 cylinder groups of 1.25MB, 80 blks, 192 inodes.
super-block backups (for fsck -b #) at:
 160, 2720, 5280, 7840
host % mount /dev/md0a /mnt/loop
```

Your disk image is now mounted into _/mnt/loop_. The system can therefore be copied there. Remember, the image is fairly small so it is recommended to build a small file system based on _/rescue_.

Once the file system has been created, un-mount it:

```
host % umount /dev/md0a
```

Then proceed to activate support for it in the kernel. Edit _sys/arm/conf/BEAGLEBOARD_ and uncomment the following lines:

```
#options		MD_ROOT
#options		MD_ROOT_SIZE=5120
#makeoptions     	MFS_IMAGE=_path_to_your_ramdisk_image_
#options         	ROOTDEVNAME=\"/dev/md0\"
#env			BEAGLEBOARD.env
```

Of course, _path\_to\_your\_ramdisk\_image_ and the size should be updated. Once this is done, the kernel needs to be recompiled.

# References #
[1](http://www.freebsd.org/doc/handbook/disks-virtual.html) The FreeBSD handbook, 18.13 Network, Memory, and File-Backed File Systems. Check the usage of _mdmfs_ for an easier way to do this.
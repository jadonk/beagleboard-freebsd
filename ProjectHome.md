# Update #

As part of the work I'm doing to unify the omap3 and omap4 trees (for Beagleboard and Pandaboard) I've move my work to gitorious. Unfortunately I don't have the time to keep both this page and the gitorious one up to date, so for now all my updates are going onto the following site:
```
https://gitorious.org/+freebsd-omap-team/freebsd/freebsd-omap/
```
I hope to eventually pull the changes back here once the omap3/beagleboard side of the tree is back to an equivalent state (which is close now).


# Overview #

This project contains a public repository of my work on porting FreeBSD to the BeagleBoard (TI OMAP3530).  It contains a partially working port of some of the files required to boot FreeBSD on the BeagleBoard, however it is very much a work in progress.



# How to Use #

The project is not in a state where you can just pick up the code and get a BeagleBoard booted and running with FreeBSD.  To start with you'll need to manually merge the code in the SVN repo into a FreeBSD tree, then you'll most likely have to spend a while debugging it to it get working correctly.

Check out the BuildingAndBooting wiki page for some basic information on how to get started.

An important thing to bare in mind is that I still don't have working pmap code, this means that the kernel will boot and the drivers will load, however userspace apps crash, this includes the init process ... this obviously limits the usability of the system.

_Please bare in mind that this is work in progress and some files may have been committed to the SVN repo without any form of testing, use with caution._



# Status #

### 24 January 2011 ###
Currently a number of the on-chip modules have drivers and are more or less working, some have know issues and all haven't received a lot of testing.  Check out this project's **SOC Components** wiki page for more details on state of various drivers.

The main issue at the moment is the pmap code.  The current head version of FreeBSD for ARM doesn't support ARMv7 MMU's.  I've done some work and got basic support up and running, this is enough to develop the drivers with, but is lacking when it comes to supporting userspace interactions (I believe the main problem being mmap used when loading shared libraries).

### 31 May 2011 ###
Started working on the Pandaboard and most of the latest developments have moved to gitorious here
```
https://gitorious.org/+freebsd-omap-team/freebsd/freebsd-omap/commits/bengray-wip
```
the code there is currently not compatible with the beagleboard but the eventual goal is to provide a single tree for both OMAP3 and OMAP4 chips.
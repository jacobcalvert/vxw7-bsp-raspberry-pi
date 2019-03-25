VxWorks® 7 Raspberry Pi 3B/3B+ unsupported BSP
===
---

# Overview

This document describes the features of the rpi_3 BSP/PSL, which is designed
to run on the Raspberry Pi 3 Model B/B+ board. This is an ARM Cortex-A53
processor-based platform.

# Project License

Copyright (c) 2019 Wind River Systems, Inc.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1) Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.

2) Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.

3) Neither the name of Wind River Systems nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.

# Prerequisite(s)

* You must have Wind River® VxWorks® 7 SR0610 released source code and
  development environment to support "Raspberry Pi 3B/B+ unsupported BSP".

# Building and Using

### Preparation

IMPORTANT: Your existing VxWorks 7 installation may already contains the rpi_3 BSP 
and bcm2837 PSL, please check following installation location to see whether they 
exist already. This Open Source BSP can exist in parallel with your existing 
installation by applying different layer versions. By default, all layers downloaded
here has a "rpi-dev" suffix to avoid collision with any existing installation.  

1. Download all three layers from Github.
```Bash
git clone -b rpi-dev https://github.com/Wind-River/vx7-bsp-raspberry-pi.git
cd vx7-bsp-raspberry-pi
```

2. Prepare the USB layer. The downloaded usb-W.X.Y.Z.rpidev is not a complete layer, 
but just contains the incremental modification needed by the Open Source BSP, thus user
shall first make a copy of the original usb-W.X.Y.Z layer (IMPORTANT: the version must
match), rename it to usb-W.X.Y.Z.rpidev, then copy the usb-W.X.Y.Z.rpidev.overwrite 
content into it:
```Bash
cp -r ***installDir***/vxworks-7/pkgs_v2/connectivity/usb-W.X.Y.Z ./usb-W.X.Y.Z.rpidev
cp -r ./usb-W.X.Y.Z.rpidev.overwrite/* ./usb-W.X.Y.Z.rpidev/
```

### Installation

There are two ways to install this BSP: inside existing VxWorks-7 Installation or outside
existing VxWorks-7 Installation.

#### Install into the source tree

All layers in this BSP goes to their respective destination among the existing installation. 
The advantage is the BSP will always be accessible once you complete the installation. The 
disadvantage is you can't shut down this BSP unless you manually delete all the installed 
layers among the source tree.

Here's how it’s done:

```Bash
cp -r rpi_3-W.X.Y.Z.rpidev ***installDir***/vxworks-7/pkges_v2/os/board/unsupported/
cp –r bcm2837-W.X.Y.Z.rpidev ***installDir***/vxworks-7/pkgs_v2/os/psl/unsupported/
cp –r usb-W.X.Y.Z.rpidev ***installDir***/vxworks-7/pkgs_v2/connectivity/
```

#### Install beside the source tree

All layers are copied in a place that's outside the default scanning folder, i.e., 
vxworks-7/pkgs_v2, and when launching the Development Shell or Workbench, the path containing 
this BSP is provided to the development environment. The advantage of this method is obvious, 
the package can be easily turn on or off, and the source code stays in one unified location 
external to the default installation, which makes it easier to manage.

Suppose all three packages are in /home/rpidev/, then do the following when launching wrenv
or Workbench:

```Bash
export WIND_LAYER_PATHS=/home/rpidev
export WIND_BSP_PATHS=/home/rpidev
```
then 
```Bash
./wrenv.sh -p vxworks-7
```
or
```Bash
./startWorkBench.sh
```

### Building

The building process comprises three parts, U-Boot, VSB project and VIP project.
U-Boot is open source code that can be got from related official server.
The building process of these three parts are all described in target.txt.
Target.txt can be found under os/board/unsupported/rpi_3 directory.

### Drivers

By now, VxWorks 7 based released SR0610 is supported by this BSP, and some
common drivers are also supported by PSL as below:

| Hardware Interface | Controller | Driver/Component | Status |
| ------ | ------ | ------ | ------ |
Mini UART | on-chip | DRV_SIO_FDT_NS16550 | SUPPORTED 
BCM2837 L1 INTCTLR | on-chip    | DRV_INTCTLR_FDT_BCM2837_L1_INTC | SUPPORTED 
BCM2837 INTCTLR    | on-chip    | DRV_INTCTLR_FDT_BCM2837_INTC    | SUPPORTED
ARM Generic Timer  | on-chip    | DRV_ARM_GEN_TIMER               | SUPPORTED
System Timer       | on-chip    | DRV_TIMER_FDT_BCM2837_SYSTIMER  | SUPPORTED
Clock              | on-chip    |                                 | UNSUPPORTED
GPIO               | on-chip    | DRV_GPIO_FDT_BCM2837            | SUPPORTED
I2C                | on-chip    | DRV_I2C_FDT_BCM2837             | SUPPORTED
PinMux             | on-chip    |                                 | UNSUPPORTED
SPI                | on-chip    | DRV_SPI_FDT_BCM2837             | SUPPORTED
USB HOST           | on-chip    | USB Host Stack(DWC2DR)          | SUPPORTED
USB ETHERNET       | on-chip    | INCLUDE_USB_GEN2_LAN78XX        | SUPPORTED
Audio              | on-chip    |                                 | UNSUPPORTED
HDMI               | on-chip    |                                 | UNSUPPORTED
Watchdog Module    | on-chip    |                                 | UNSUPPORTED
EMMC               | on-chip    |                                 | UNSUPPORTED
Wireless LAN       | unknown    |                                 | UNSUPPORTED
Bluetooth          | unknown    |                                 | UNSUPPORTED

The detailed introduction of these drivers and usage can also be found in
target.txt. It is similar as other BSP in VxWorks 7 SR0610.

# Legal Notices

All product names, logos, and brands are property of their respective owners. 
All company, product and service names used in this software are for identification 
purposes only. Wind River and VxWorks are registered trademarks of Wind River 
Systems. Raspberry Pi are registered trademarks of the Raspberry Pi Foundation.

Disclaimer of Warranty / No Support: Wind River does not provide support and 
maintenance services for this software, under Wind River’s standard Software 
Support and Maintenance Agreement or otherwise. Unless required by applicable 
law, Wind River provides the software (and each contributor provides its 
contribution) on an “AS IS” BASIS, WITHOUT WARRANTIES OF ANY KIND, either express 
or implied, including, without limitation, any warranties of TITLE, NONINFRINGEMENT, 
MERCHANTABILITY, or FITNESS FOR A PARTICULAR PURPOSE. You are solely responsible 
for determining the appropriateness of using or redistributing the software and 
assume ay risks associated with your exercise of permissions under the license.


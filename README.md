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

### Setup

Checkout feature of GitHub to place the contents of this repository 
in your VxWorks® install tree at an appropriate location:
```Bash
cp -r os/board/unsupported/rpi_3 ***installDir***/vxworks-7/pkgs_v2/os/board/unsupported

cp -r os/psl/unsupported/bcm2837  ***installDir***/vxworks-7/pkgs_v2/os/psl/unsupported

```
### Building

The building process comprises three parts, U-Boot, VSB project and VIP project.
U-Boot is open source code that can be got from related official server.
The building process of these three parts are all described in target.txt.
Target.txt can be found under os/board/unsupported/rpi_3 directory.

### Drivers

By now, VxWorks 7 based released SR0610 is supported by this BSP, and some
common drivers are also supported by PSL as below:
Hardware Interface | Controller | Driver/Component                | Status
/-------------------------------------------------------------------------------
Mini UART          | on-chip    | DRV_SIO_FDT_NS16550             | SUPPORTED
BCM2837 L1 INTCTLR | on-chip    | DRV_INTCTLR_FDT_BCM2837_L1_INTC | SUPPORTED
BCM2837 INTCTLR    | on-chip    | DRV_INTCTLR_FDT_BCM2837_INTC    | SUPPORTED
ARM Generic Timer  | on-chip    | DRV_ARM_GEN_TIMER               | SUPPORTED
System Timer       | on-chip    | DRV_TIMER_FDT_BCM2837_SYSTIMER  | SUPPORTED
Clock              | on-chip    |                                 | UNSUPPORTED
GPIO               | on-chip    | DRV_GPIO_FDT_BCM2837            | SUPPORTED
I2C                | on-chip    | DRV_I2C_FDT_BCM2837             | SUPPORTED
PinMux             | on-chip    |                                 | UNSUPPORTED
SPI                | on-chip    | DRV_SPI_FDT_BCM2837             | SUPPORTED
Audio              | on-chip    |                                 | UNSUPPORTED
HDMI               | on-chip    |                                 | UNSUPPORTED
Watchdog Module    | on-chip    |                                 | UNSUPPORTED
EMMC               | on-chip    |                                 | UNSUPPORTED
Wireless LAN       | unknown    |                                 | UNSUPPORTED
Bluetooth          | unknown    |                                 | UNSUPPORTED

The detailed introduction of these drivers and usage can also be found in
target.txt. It is similar as other BSP in VxWorks 7 SR0610.


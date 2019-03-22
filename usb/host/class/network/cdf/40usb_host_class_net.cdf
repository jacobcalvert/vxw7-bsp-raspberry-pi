/* 40usb_host_class_net.cdf - USB host class network component description file */

/*
 * Copyright (c) 2014, 2017, 2019 Wind River Systems, Inc.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met: 
 *
 * 1) Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer. 
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation and/or 
 * other materials provided with the distribution. 
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors may be 
 * used to endorse or promote products derived from this software without specific 
 * prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE 
 * USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 */

/*
modification history
--------------------
29jan19,npc  added Lan78xx usb ethernet card support (F11409)
06jun17,whu  support to finish transfer with a zero-length packet (V7CON-493)
05jun14,s_z  Add INCLUDE_END as dependence (V7CON-148)
15feb14,j_x  remove USB_GEN2_END_MAX_DEVS parameter (VXW7-1884)
07jan14,j_x  create
*/

/*
DESCRIPTION
This file contains descriptions for the USB host class network components.
*/

/* USB host class network configuration */

Folder  FOLDER_USB_GEN2_END_CONTROLLERS {
    NAME            USB GEN2 END Controllers
    SYNOPSIS        USB Generation 2 END Class Controllers
    _CHILDREN       FOLDER_USB_GEN2_DEVICES
    CHILDREN        INCLUDE_USB_GEN2_PEGASUS      \
                    INCLUDE_USB_GEN2_DM960X       \
                    INCLUDE_USB_GEN2_ASIX         \
                    INCLUDE_USB_GEN2_LAN78XX
}

Component INCLUDE_USB_GEN2_PEGASUS {
    NAME            Pegasus Serial Controllers
    SYNOPSIS        USB Generation 2 Pegasus serial controllers
    MODULES         usb2Pgs.o usb2End.o
    REQUIRES        INCLUDE_USB                \
                    INCLUDE_USB_GEN2_HELPER
}

Component INCLUDE_USB_GEN2_DM960X {
    NAME            Dm960x Serial Controllers
    SYNOPSIS        USB Generation 2 Dm960x serial controllers
    MODULES         usb2Dm960x.o usb2End.o
    REQUIRES        INCLUDE_USB \
                    INCLUDE_USB_GEN2_HELPER
}

Component INCLUDE_USB_GEN2_ASIX {
    NAME            Asix Serial Controllers
    SYNOPSIS        USB Generation 2 Asix  serial controllers
    MODULES         usb2Asix.o usb2End.o
    REQUIRES        INCLUDE_USB \
                    INCLUDE_USB_GEN2_HELPER
}

Component INCLUDE_USB_GEN2_LAN78XX {
    NAME            Lan78xx Serial Controllers
    SYNOPSIS        USB Generation 2 Lan78xx serial controllers
    MODULES         usb2Lan78xx.o usb2End.o
    REQUIRES        INCLUDE_USB \
                    INCLUDE_USB_GEN2_HELPER
}

Component INCLUDE_USB_GEN2_END_INIT {
    NAME            USB GEN2 END Device Init
    SYNOPSIS        USB GEN2 END Device Component Initialization
    _CHILDREN       FOLDER_USB_GEN2_DEVICES_INIT
    REQUIRES        INCLUDE_USB_INIT          \
                    INCLUDE_END               \
                    INCLUDE_USB_HOST_CLASS_INIT
    CFG_PARAMS      USB_GEN2_END_NAME       \
                    USB_GEN2_END_IP_ADDRESS \
                    USB_GEN2_END_NET_MASK   \
                    USB_GEN2_END_COMMON_TASK_PRIORITY \
                    USB_GEN2_END_ZERO_LENGTH_PACKET
}

Parameter  USB_GEN2_END_IP_ADDRESS {
    NAME            USB GEN2 END Device IP Address
    SYNOPSIS        USB GEN2 END Device IP Address
    DEFAULT         {"90.0.0.50"}
}

Parameter  USB_GEN2_END_NET_MASK {
    NAME            USB GEN2 END Device Net Mask
    SYNOPSIS        USB GEN2 END Device Net Mask
    DEFAULT         {0xffff0000}
}

Parameter USB_GEN2_END_NAME {
    NAME            USB GEN2 End driver base name
    SYNOPSIS        Drive Base Name associated to the GEN2 End device
    DEFAULT         "usb2End"
}

Parameter USB_GEN2_END_COMMON_TASK_PRIORITY {
    NAME            USB GEN2 End Common Task priority
    SYNOPSIS        USB GEN2 End Common Task priority
    DEFAULT         100
}

Parameter USB_GEN2_END_ZERO_LENGTH_PACKET {
    NAME            USB GEN2 End Zero Length Packet Support
    SYNOPSIS        USB GEN2 End Zero Length Packet Support
    TYPE            bool
    DEFAULT         FALSE
}

/* debug information */

Component INCLUDE_USB_TRACK_GEN2_END {
    NAME            END
    SYNOPSIS        USB GEN2 END Driver Tracking
    _CHILDREN       FOLDER_USB_TRACK_GEN2
    CONFIGLETTES    usrUsbDebug.c
    REQUIRES        INCLUDE_USB_GEN2_PEGASUS
    CFG_PARAMS      USB_TRACK_GEN2_END
}

Parameter USB_TRACK_GEN2_END {
    NAME            USB GEN2 END Debug Level BitMap
    SYNOPSIS        USB GEN2 END Debug Level BitMap Enable By Set (Bit 0:INFO, Bit 1:ERROR, Bit 2:WARNING, Bit 3:NORMAL, Bit 4:VERBOSE)
    DEFAULT         1
}
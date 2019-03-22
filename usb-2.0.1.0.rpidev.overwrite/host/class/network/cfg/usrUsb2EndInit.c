/* usrUsn2EndInit.c - Generation 2 USB-Ethernet class driver configlette file */

/*
 * Copyright (c) 2008-2010, 2017, 2019 Wind River Systems, Inc.  
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met: 
 *
 * 1) Redistributions of source code must retain the above copyright notice, 
 * this list of conditions and the following disclaimer. 
 *
 * 2) Redistributions in binary form must reproduce the above copyright notice, 
 * this list of conditions and the following disclaimer in the documentation 
 * and/or other materials provided with the distribution. 
 *
 * 3) Neither the name of Wind River Systems nor the names of its contributors 
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission. 
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE 
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE 
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
modification history
--------------------
29jan19,npc  added Lan78xx usb ethernet card support (F11409)
06jun17,whu  support to finish transfer with a zero-length packet (V7CON-493)
01sep10,ghs  Change default ip address (WIND00231161)
13jan10,ghs  vxWorks 6.9 LP64 adapting
02jul09,s_z  move the hardware related initialization routine to this file
12nov08,s_z  written

*/
#ifndef __INCusrUsb2EndInitc
#define __INCusrUsb2EndInitc


#ifdef __cplusplus
extern "C" {
#endif

/* includes */
#include <usb2End.h>

#ifdef INCLUDE_USB_GEN2_PEGASUS
#include <usb2Pgs.h>
#endif

#ifdef INCLUDE_USB_GEN2_DM960X
#include <usb2Dm960x.h>
#endif

#ifdef INCLUDE_USB_GEN2_ASIX
#include <usb2Asix.h>
#endif

#ifdef INCLUDE_USB_GEN2_LAN78XX
#include <usb2Lan78xx.h>
#endif

/* defines */

/* Default device name */
#ifndef USB_GEN2_END_NAME
#define USB_GEN2_END_NAME                  "usb2End"
#endif

/* Default IP address and netmask */

/* TODO: to support more pegasus devices */


#ifndef USB_GEN2_END_IP_ADDRESS
#define USB_GEN2_END_IP_ADDRESS           {"90.0.0.50"}
#endif

#ifndef USB_GEN2_END_NET_MASK
#define USB_GEN2_END_NET_MASK             {0xFFFF0000}
#endif

#ifndef USB_GEN2_END_COMMON_TASK_PRIORITY
#define USB_GEN2_END_COMMON_TASK_PRIORITY (100)
#endif

#ifndef USB_GEN2_END_MAX_DEVS
#define USB_GEN2_END_MAX_DEVS             (1)
#endif

#ifndef USB_GEN2_END_ZERO_LENGTH_PACKET
#define USB_GEN2_END_ZERO_LENGTH_PACKET             (FALSE)
#endif

/*
 * Define the globles for the parameters defined by the workbench or
 * the configuration, which will be used by usb2End.c
 */

const char * g_UsbGen2EndName          = USB_GEN2_END_NAME;
const INT8   g_UsbGen2EndNameLen       = (sizeof(USB_GEN2_END_NAME) + 1 );

const char * g_UsbGen2EndIpAddress[USB_GEN2_END_MAX_DEVS] = USB_GEN2_END_IP_ADDRESS;

const UINT32 g_UsbGen2EndNetMask[USB_GEN2_END_MAX_DEVS] = USB_GEN2_END_NET_MASK;

const INT8   g_UsbGen2EndMaxDevs = USB_GEN2_END_MAX_DEVS;

const UINT8  g_UsbGen2EndCommonTaskPriority  = USB_GEN2_END_COMMON_TASK_PRIORITY;

const BOOL   g_UsbGen2EndZlpSupport = USB_GEN2_END_ZERO_LENGTH_PACKET;

/*******************************************************************************
*
* usrUsb2PgsInit - initializes the GEN2 Usb-Ehternet class driver.
*
* This function is initializing the GEN2 Usb-Ehternet class driver.
* If called automatically by the project facility
*
* RETURNS: If the Usb-Ehternet driver could be initialized or not
*
* ERRNO: N/A
*/

STATUS usrUsb2EndInit()
    {

    #ifdef INCLUDE_USB_GEN2_PEGASUS
    usb2PgsInit(NULL);
    #endif

    #ifdef INCLUDE_USB_GEN2_DM960X
    usb2Dm960xInit(NULL);
    #endif

    #ifdef INCLUDE_USB_GEN2_ASIX
    usb2AsixInit(NULL);
    #endif

    #ifdef INCLUDE_USB_GEN2_LAN78XX
    usb2Lan78xxInit(NULL);
    #endif

    return usb2EndInit(USB_GEN2_END_NAME);
    }


#ifdef __cplusplus
}
#endif
#endif  /* __INCusrUsb2EndInitc */






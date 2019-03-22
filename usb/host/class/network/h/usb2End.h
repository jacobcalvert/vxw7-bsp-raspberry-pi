/* usb2End.h - Generation 2 USB End driver */

/*
 * Copyright (c) 2008-2011, 2014-2015, 2018-2019 Wind River Systems, Inc.
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
24may18,whu  add commonTaskId in USB2_END_DEVICE (V7CON-633)
25feb15,r_w  merge VXW6-83171 to vxWorks 7 (V7CON-236)
01sep14,lan  add a semaphore to protect the usb2EndUnbind (VXW6-83171)
10aug11,s_z  Replace netPoolInit by endPoolCreate (WIND00286518)
18jul11,s_z  Deal with the network packet which has PAD data at the end
             (WIND00288104)
06jul11,ghs  Use USBD match function for matching device ID (WIND00183500)
07jan11,ghs  Clean up compile warnings (WIND00247082)
29apr10,pad  Moved extern C statement after include statements.
27apr10,j_x  Import usb2EndIoctl for usbGen2Show (WIND00184395)
30mar10,w_x  Back port GEN2 class drivers for old VxWorks (WIND00192167)
23mar10,j_x  Changed for USB debug (WIND00184542)
13jan10,ghs  vxWorks 6.9 LP64 adapting
21oct09,w_x  Modifications for Kernel API references (WIND00187268)
11sep09,ghs  Remove warning message
27aug09,w_x  Unify log mechanism and variable naming with some compiler
             warning removal in USB GEN2 class drivers (WIND00178936)
19aug09,s_z  Change usb2EndLoad routine as an external interface
22arp09,s_z  Remove the gun compiler warning and code cleaning
12nov08,s_z  written

*/
#ifndef __INCusb2Endh
#define __INCusb2Endh

/* includes */
#include <vxWorks.h>
#include <iosLib.h>
#include <semLib.h>
#include <logLib.h>
#include <errno.h>
#include <errnoLib.h>
#include <etherMultiLib.h> /* multicast stuff. */
#include <endLib.h>
#include <usb2Helper.h>

#ifdef __cplusplus
extern "C" {
#endif

/* define */

/* Default buffer sizes */
#ifndef USB2_END_DEFAULT_OUT_BUFFER_SIZE
#define USB2_END_DEFAULT_OUT_BUFFER_SIZE    2048 /*1550*/
#endif

#ifndef USB2_END_DEFAULT_IN_BUFFER_SIZE
#define USB2_END_DEFAULT_IN_BUFFER_SIZE     2048 /*1550*/
#endif

#define USB2_END_INTERRUPT_IN_BUFFER_SIZE   8

/*
#define USB2_END_POOL_BUFSIZ                ((USB2_END_DEFAULT_IN_BUFFER_SIZE + 3) \
                                             & 0xFFFFFFFC)
*/

#define USB2_END_POOL_BUFSIZ                ((USB2_END_DEFAULT_OUT_BUFFER_SIZE + 3) \
                                             & 0xFFFFFFFC)

#define USB2_END_MIN_BUF                    (1514)    /* min first buffer size */
#define USB2_END_TUPLE_CNT                  (256)


/* Default poly for hash */
#define USB2_END_POLY                       0xEDB88320
#define USB2_END_BITS                       0x06

/*
 * Define different iConfigureFlag according devices chipset serial
 * we use the first byte to record different kind of chipset
 * the last byte for future using, such as status checking
 */


#define USB2_END_PEGASUS_II                         (0x01)
#define USB2_END_ASIX                               (0x02)
#define USB2_END_ASIX_AM88772                       (0x03)
#define USB2_END_DM960X                             (0x04)
#define USB2_END_LAN78XX                            (0x05)

/* Define End speed parameter*/

#define USB2_END_SPEED_10M                  10000000        /* 10Mbs */
#define USB2_END_SPEED_100M                 100000000       /* 10Mbs */

/*
 * Shortcut for getting the hardware address
 * from the MIB II stuff.
 */
#define END_HADDR(pEnd)     \
    ((pEnd)->mib2Tbl.ifPhysAddress.phyAddress)

#define END_HADDR_LEN(pEnd) \
    ((pEnd)->mib2Tbl.ifPhysAddress.addrLength)

/*
 * USB2_END_MAX_LIST_NUMBER is the max length of the global list table,
 * can be expanded when the list is not enough
 */
#define USB2_END_MAX_LIST_NUMBER                16
#define USB2_END_MAX_VENDOR_DEVICE_NUMBER       100

/*
 *
 *
 */

#define USB2_END_DEFAULT_TIMEOUT                1000
#define USB2_END_DEFAULT_TIMEOUT_PLUS           2000

/*Vendor ID defination */

/* ADMtek */
#define ADMTEK_VENDOR_ID                        0x07A6

/* Sohoware */
#define SOHOWARE_VENDOR_ID                      0x15E8

/* D-Link */
#define DLINK_VENDOR_ID                         0x2001

/* Belkin */
#define BELKIN_VENDOR_ID                        0x050D

/* Netgear */
#define NETGEAR_VENDOR_ID                       0x0846

/* IO Data */
#define IODATA_VENDOR_ID                        0x04BB

/* 3COM */
#define _3COM_VENDOR_ID                         0x0506

/* SIEMENS */
#define SIEMENS_VENDOR_ID                       0x067C

/* ABOCOM */
#define ABOCOM_VENDOR_ID                        0x07B8

/* ACCTON */
#define ACCTON_VENDOR_ID                        0x083A

/* AEILAB */
#define AEILAB_VENDOR_ID                        0x3334

/* ALLIEDTEL */
#define ALLIEDTEL_VENDOR_ID                     0x07C9

/* ATEN */
#define ATEN_VENDOR_ID                          0x0557

/* BILLIONTON */
#define BILLIONTON_VENDOR_ID                    0x08DD

/* COMPAQ */
#define COMPAQ_VENDOR_ID                        0x049F

/* COREGA */
#define COREGA_VENDOR_ID                        0x07AA

/* ELCON */
#define ELCON_VENDOR_ID                         0x0dB7

/* ELECOM */
#define ELECOM_VENDOR_ID                        0x056E

/* ELSA */
#define ELSA_VENDOR_ID                          0x05CC

/* GIGABYTE */
#define GIGABYTE_VENDOR_ID                      0x1044

/* HAWKING */
#define HAWKING_VENDOR_ID                       0x0E66

/* HP */
#define HP_VENDOR_ID                            0x03F0

/* KINGSTON */
#define KINGSTON_VENDOR_ID                      0x0951

/* LANEED */
#define LANEED_VENDOR_ID                        0x056E

/* LINKSYS */
#define LINKSYS_VENDOR_ID                       0x066B

/* LINKSYS2 */
#define LINKSYS2_VENDOR_ID                      0x077B

/* MELCO */
#define MELCO_VENDOR_ID                         0x0411

/* MICROSOFT */
#define MICROSOFT_VENDOR_ID                     0x045E

/* MOBILITY */
#define MOBILITY_VENDOR_ID                      0x1342

/* OCT */
#define OCT_VENDOR_ID                           0x0B39

/* SMARTBRIDGES */
#define SMARTBRIDGES_VENDOR_ID                  0x08D1

/* SMC */
#define SMC_VENDOR_ID                           0x0707

/* typedef */

/* Usb-Ethernet Device Configure Flag */
typedef struct usb2_end_adapter_configure
    {
    UINT8          uChipsetType;
    UINT8          uHeader;
    UINT8          uLinkStatusOffset;
    UINT8          uLinkStatusMask;
    }USB2_END_CONFIG_FLAG,*pUSB2_END_CONFIG_FLAG;

/*
 * Usb2 end private functions, which need be accompished in
 * the chipset previate files
 */
typedef struct usb2_end_private_funcs
    {
    STATUS (*usb2EndSetFlag)
        (
        END_OBJ *             endObj
        );
    STATUS (*usb2EndMCastFilterSet)
        (
        END_OBJ *             endObj
        );
    STATUS (*usb2EndHWConfig)
        (
        VOID *                pDev
        );
    VOID (*usb2EndStatusCheck)
        (
        VOID *                pDev
        );
    VOID (* usb2EndDataPacking)
        (
        UINT8 *               pBuf,
        UINT32 *              pActLen
        );
    UINT32 (* usb2EndDataUnPacking)
        (
        unsigned char **      ppBuf,
        UINT32 *              pPacketLen,
        UINT32 *              pActLen
        );
    VOID (* quirkySupportFunc)
        (
        pUSB2_END_CONFIG_FLAG pConfigFlag
        );
    VOID (* usrApplication)
        (
        void
        );
    } USB2_END_PRIVATE_FUNCS;


#define USB2_END_SET_CONFIG(uChipsetType,uHeader,uLinkStatusOffset,          \
                         uLinkStatusMask)                                    \
{                                                                            \
uChipsetType,                                                                \
uHeader,                                                                     \
uLinkStatusOffset,                                                           \
uLinkStatusMask                                                              \
}

/* Usb-Ethernet Device Information */
typedef struct usb2_end_adapter_info
    {
    USBHST_DEVICE_ID         deviceId;      /* Store device id */
    const char *             deviceName;    /* device name */
    USB2_END_PRIVATE_FUNCS * pPrivateFuncs; /* Device private Function table */
    pUSB2_END_CONFIG_FLAG    pConfigFlag;   /*Configure Flag*/

    }USB2_END_ADAPTER_INFO,*pUSB2_END_ADAPTER_INFO;

#define USB2_END_ADAPTER(uVendorID,uProductID,deviceName,pPrivateFuncs,     \
                         pConfigFlag)                                       \
{                                                                           \
    {                                                                       \
    {NULL}, USBD_MATCH_NOTIFY_AS_DEVICE,                                    \
    uVendorID, uProductID, 0, 0, 0,                                         \
    0, 0, 0,                                                                \
    0, 0, 0,                                                                \
    USBD_MATCH_ID_VENDORID | USBD_MATCH_ID_PRODUCTID, 0                     \
    },                                                                      \
deviceName,                                                                 \
pPrivateFuncs,                                                              \
pConfigFlag                                                                 \
}

typedef struct usb2_end_adapter_device_list
    {
    char *                  pTeamName;        /* Team Name */
    USB2_END_ADAPTER_INFO * pAdapterTeamList; /* Team List */
    UINT16                  nTeamMembers;     /* Numbers of the Team member*/

    }USB2_END_DEVICE_LIST,*pUSB2_END_DEVICE_LIST;

typedef struct usb2_end_device
    {
    END_OBJ             endObj;                  /* must be first field */
    USB2_CLASS_DEVICE * pUsb2ClassDevice;        /* Generic class device info */

    SEM_ID              pBulkOutBinarySema;      /* signaling sem, Bulk Out transfer*/
    SEM_ID              pBulkInBinarySema;       /* signaling sem, Bulk In transfer */
    SEM_ID              pInterruptInBinarySema;  /* signaling sem, Interrupt in transfer */

    SEM_ID              pBulkOutUrbMutex;        /* Bulk Out transfer mutex */
    SEM_ID              pBulkInUrbMutex;         /* Bulk In transfer mutex */
    SEM_ID              pInterruptInUrbMutex;    /* Interrupt In transfer mutex */
    SEM_ID              pEndUnbindMutex;         /* End unbind mutex */

    USBHST_URB *        pBulkOutUrb;             /* Bulk Out transfer URB */
    USBHST_URB *        pBulkInUrb;              /* Bulk In transfer URB */
    USBHST_URB *        pInterruptInUrb;         /* Interrupt In transfer URB */

    UINT32              uBulkOutBufferSize;      /* Bulk Out buffer size  */
    UINT32              uBulkInBufferSize;       /* Bulk In buffer size  */
    UINT32              uInterruptInBufferSize;  /* Interrupt In buffer size  */

    unsigned char *     pBulkOutBuffer;          /* Bulk Out buffer point*/
    unsigned char *     pBulkInBuffer;           /* Bulk In buffer */
    unsigned char *     pInterruptInBuffer;      /* Interrupt In buffer */

    UINT8               bBulkOutEndPointAddr;    /* Bulk Out endpoint addr */
    UINT8               bBulkInEndPointAddr;     /* Bulk In endpoint addr */
    UINT8               bInterruptInEndPointAddr;/* Interrupt In endpoint addr */

    char *              pDeviceGeneralName;      /* Devcie General Name */
    UINT32              uDeviceIndex;            /* Device index in the list */

    int                 deviceTimeout;           /* Timeout (ms) to wait reqs. */

    UINT8               macAddress[6];           /* Mac address */
    MSG_Q_ID            hCommonTaskMsgQ;         /* MsgQ ID for common task */
    TASK_ID             commonTaskId;

    USB2_END_ADAPTER_INFO *pAdapterInfo;         /* Adapter infomation */
    UINT16              wMaxPacketSizeOut;       /* Max packet size */
    BOOL                bLinked;                 /* Net cable linked*/
    BOOL                bFirstConnected;         /* The first time connection */
    BOOL                bNetworkBinded;          /* Bind to Network */

    } USB2_END_DEVICE;


typedef struct usb2_end_data_received_info
    {
    USB2_END_DEVICE * pDevice;     /* Device point */
    UINT8 *           pBuffer;     /* Received data buffer point or NULL */
    UINT32            cmdOractLen; /* parameter or received data length */
    } USB2_END_DATA_RECEIVED_INFO;

typedef enum data_received_info_parameter
    {
    USB2_END_KILL_THE_TASK = 0,
    USB2_END_DATA_TRANSFER_REQUEST ,
    USB2_END_STATUS_CHECK_REQUEST ,
    USB2_END_BIND_TO_NETWORK,
    USB2_END_UNBIND_FROM_NETWORK
    } DATA_RECEIVED_INFO_PARAMETER;

/*
 * Exported functions
 */

IMPORT int endMultiLstCnt
    (
    END_OBJ * pEnd
    );

IMPORT END_OBJ * usb2EndLoad
    (
    char *    initString,
    void *    pDev
    );

IMPORT STATUS usb2EndInit
    (
    char *    pName
    );

IMPORT void usb2EndDeInit
    (
    void
    );

IMPORT STATUS usb2EndDeviceListAdd
    (
    char                  * pTeamName,
    USB2_END_ADAPTER_INFO * pAdapterTeamList,
    UINT16                  nTeamMenbers
    );

IMPORT VOID usb2EndDeviceListRemove
    (
    USB2_END_ADAPTER_INFO * pAdapterTeamList
    );

IMPORT int usb2EndIoctl
    (
    USB2_END_DEVICE * pDevice,    /* device receiving command */
    int               function,   /* Function to use */
    void *            arg         /* Argument to use */
    );

#ifdef __cplusplus
}
#endif
#endif  /* __INCusb2Endh */


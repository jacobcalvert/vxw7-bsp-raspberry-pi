/* usb2Smsc95xx.c - USB SMSC95XX device class driver using the WRS USB2 API */

/*
 * Copyright (c) 2019 Wind River Systems, Inc.  
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
20may19,jnc  create, ported from Lan78XX driver

*/

/*
DESCRIPTION

This module works as a hardware configure layer for USB-Ethernet adapters
built with SMSC95XX Chipsets.

Since the module use the interface offered by usb2End.c, we only need to add
the supported device list into the the global device list g_usb2EndAdapterList
defined in usb2End.c. In the device list information structure, the
pPrivateFuncs table is very important, such as the hardware configuration
usb2Smsc95xxHWConfig.

\cs
USB2_END_PRIVATE_FUNCS usb2Smsc95xxPrivteFuncs =
    {
    usb2Smsc95xxSetFlag,
    NULL,
    usb2Smsc95xxHWConfig,
    usb2Smsc95xxStatusCheck,
    usb2Smsc95xxDataPacking,
    usb2Smsc95xxDataUnPacking,
    NULL,
    NULL
    };
\ce

INITIALIZATION

To initialize the Smsc95xx devices support, usb2Smsc95xxInit needs to be called to
add the supported device list to the global device list g_usb2EndAdapterList
defined in usb2End.c

Furthermore, usb2Smsc95xxInit() needs to be called before the usb2EndInit(), 
which will guarantee the device can be recognized if the devices have already
been connected when booting up.

INCLUDE FILES: vxWorks.h semLib.h logLib.h errno.h errnoLib.h
               etherMultiLib.h endLib.h
               usbHst.h usbd.h usb2End.h usb2Smsc95xx.h vxFdtLib.h
*/

/* includes */

#include <vxWorks.h>
#include <semLib.h>
#include <logLib.h>
#include <errno.h>
#include <errnoLib.h>
#include <etherMultiLib.h>
#include <endLib.h>
#include <usbHst.h>
#include <usbd.h>
#include <usb2End.h>
#include <usb2Smsc95xx.h>
#include <vxFdtLib.h>

/* defines */

#undef SMSC95XX_DBG
#ifdef SMSC95XX_DBG
USB2_CLASS_DEVICE * gpUsb2ClassDevice;

UINT8 * gtxbuf[10000] = {0};
UINT8 * grxbuf[10000] = {0};
#endif

/* imports */

IMPORT void sysUsDelay (int delay);

/* locals */

/* forward declarations */

LOCAL STATUS usb2Smsc95xxSetFlag
    (
    END_OBJ *       endObj
    );

LOCAL STATUS usb2Smsc95xxHWConfig
    (
    VOID *          pDev
    );

LOCAL VOID usb2Smsc95xxDataPacking
    (
    UINT8 *         pBuf,
    UINT32 *        pActLen
    );

LOCAL UINT32 usb2Smsc95xxDataUnPacking
    (
    unsigned char ** ppBuf,
    UINT32 *         pPacketLen,
    UINT32 *         pActLen
    );

LOCAL VOID usb2Smsc95xxStatusCheck
    (
    VOID *                   pDev
    );

/* Smsc95xx device private function table */

LOCAL USB2_END_PRIVATE_FUNCS usb2Smsc95xxPrivteFuncs =
    {
    usb2Smsc95xxSetFlag,
    NULL,
    usb2Smsc95xxHWConfig,
    usb2Smsc95xxStatusCheck,
    usb2Smsc95xxDataPacking,
    usb2Smsc95xxDataUnPacking,
    NULL,
    NULL
    };

/* Configuration */

LOCAL USB2_END_CONFIG_FLAG usb2Smsc95xxConfigFlag =
    {
    USB2_END_SMSC95XX,
    USB2_END_SMSC75X_HEADER,
    USB2_SMSC75X_LINK_STATUS_OFFSET,
    USB2_SMSC75X_LINK_STATUS_MASK
    };

/* Adapter list */

LOCAL USB2_END_ADAPTER_INFO Usb2Smsc95xxAdapterList[] =
    {
    USB2_END_ADAPTER (0x0424,
                      SMSC95XX_CHIP_ID_9512,
                      "SMSC9512 USB Ethernet",
                      &usb2Smsc95xxPrivteFuncs,
                      &usb2Smsc95xxConfigFlag
                      )
    };

/*******************************************************************************
*
* usb2Smsc95xxGetRegisters - get the registers value from the Smsc95xx device
*
* This routine requests data from the indexed registers.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
*/

LOCAL STATUS usb2Smsc95xxGetRegister
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice, /* Device pointer */
    UINT32              index,            /* Register Index Start to read */
    UINT32 *            pBuffer           /* Pointer to store the data */
    )
    {
    USBHST_STATUS       hstStatus = USBHST_FAILURE;
    UINT16              transferLength = 4; /* fixed register width */

    hstStatus = usb2VendorClassSpecific (
                    pUsb2ClassDevice,
                    USB2_RT_DEV_TO_HOST| USB2_RT_VENDOR | USB2_RT_DEVICE,
                    USB2_SMSC95XX_GET_REGISTER_VENDOR_RQ,    /* request */
                    0,                                      /* value */
                    (UINT16) index,                         /* index */
                    &transferLength,                        /* length */
                    (pUINT8) pBuffer,                       /* pBfr */
                    USB2_DEFAULT_TIMEOUT);                  /* timeout */

    USB2_END_INFO ("usb2Smsc95xxGetRegister(): index %d ret %d, data 0x%x\n",
                   index, hstStatus, *pBuffer, 4, 5, 6);

    if (hstStatus == USBHST_SUCCESS)
        {
        return OK;
        }
    else
        {
        return ERROR;
        }
    }

/*******************************************************************************
*
* usb2Smsc95xxSetRegister - set a value to a single register
*
* This routine sets the right value to the register.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxSetRegister
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice, /* Device pointer */
    UINT32              index,            /* Register Index */
    UINT32              data              /* Value to be set */
    )
    {
    USBHST_STATUS       hstStatus = USBHST_FAILURE;
    UINT16              uLength = 4;    /* fixed register width */

    USB2_END_INFO ("usb2Smsc95xxSetRegister(): index %d, data 0x%x",
                   index, data, 3, 4, 5, 6);

    hstStatus = usb2VendorClassSpecific (
                        pUsb2ClassDevice,
                        USB2_RT_HOST_TO_DEV | USB2_RT_VENDOR | USB2_RT_DEVICE,
                        USB2_LAN78XX_SET_REGISTER_VENDOR_RQ,    /* request */
                        0,                                      /* value */
                        (UINT16) index,                         /* index */
                        &uLength,                               /* pActLen */
                        (pUINT8) &data   ,                      /* pBfr */
                        USB2_DEFAULT_TIMEOUT);                  /* timeoutMs */

    USB2_END_INFO (" ret %d\n",
                   hstStatus, 2, 3, 4, 5, 6);

    if (hstStatus == USBHST_SUCCESS)
        {
        return OK;
        }
    else
        {
        return ERROR;
        }
    }

#ifdef LAN78XX_DBG
void testGetReg
    (
    UINT32 reg
    )
    {
    UINT32 data = 0;
    
    usb2Smsc95xxGetRegister (gpUsb2ClassDevice, reg, &data);
    printf ("reg %x, data %x\n", reg, data);
    }

void testSetReg
    (
    UINT32 reg,
    UINT32 data
    )
    {
    usb2Smsc95xxSetRegister (gpUsb2ClassDevice, reg, data);
    }
#endif

/*******************************************************************************
*
* usb2Smsc95xxWaitRegisterBit - wait specific value for single register bits
*
* This routine will read the register continuously until the value of specified
* bits are matched or timeout.
*
* RETURNS: OK returned, or ERROR if it was timeout.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxWaitRegisterBit
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice,   /* Device pointer */
    UINT32              index,              /* Register Index */
    UINT32              bitMask,            /* Matched bits mask */
    UINT32              bitValue,           /* Matched bits value */
    UINT32              timeout             /* Timeout time in millisecond */
    )
    {
    UINT32              count = 0;
    UINT32              data  = 0;

    while (count < timeout)
        {
        if (OK != usb2Smsc95xxGetRegister (pUsb2ClassDevice,
                                          index,
                                          &data))
            {
            USB2_END_ERR ("usb2Smsc95xxWaitRegisterBit(): "
                          "get reg index 0x%x fail\n",
                          index, 2, 3, 4, 5, 6);
            return ERROR;
            }

        if (bitValue == (data & bitMask))
            {
            return OK;
            }

        count++;
        sysUsDelay (1000);
        }

    return ERROR;   /* timeout */
    }

/*******************************************************************************
*
* usb2Smsc95xxPhyBusyWait - wait PHY available status
*
* This routine waits PHY available status.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
*/

LOCAL STATUS usb2Smsc95xxPhyBusyWait
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice  /* Device pointer */
    )
    {
    if (OK != usb2Smsc95xxWaitRegisterBit (pUsb2ClassDevice,
                                          LAN78XX_MII_ACC,
                                          BIT (SMSC95XX_MII_ACC_BUSY_BIT),
                                          0 << SMSC95XX_MII_ACC_BUSY_BIT,
                                          1000))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyBusyWait(): "
                      "wait SMSC95XX_MII_ACC_BUSY_BIT timeout\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxPhyRead - read the contents of the PHY registers
*
* This routine reads the register contents of PHY a device.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxPhyRead
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice, /* Device pointer */
    UINT8               offSet,           /* Offset of the MII register */
    UINT16 *            phyWord           /* Pointer to store the value */
    )
    {
    UINT32 data = 0;

    /* wait for PHY not busy */ 
    if(OK != usb2Smsc95xxPhyBusyWait(pUsb2ClassDevice))
    {
        USB2_END_ERR ("usb2Smsc95xxPhyRead(): phy busy wait fail\n",
                      1, 2, 3, 4, 5, 6);
	return ERROR;
    }

    data = (USB2_SMSC95XX_PHY_ADDR << SMSC95XX_MII_ACC_PHYADDR_BIT) |
           ((UINT32) offSet << SMSC95XX_MII_ACC_REG_BIT) |
           SMSC95XX_MII_ACC_MII_READ |
           BIT (SMSC95XX_MII_ACC_BUSY_BIT);

    if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                      SMSC95XX_MII_ACC,
                                      data))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyRead(): set SMSC95XX_MII_ACC fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    if (OK != usb2Smsc95xxPhyBusyWait (pUsb2ClassDevice))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyRead(): phy busy wait fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    if (OK != usb2Smsc95xxGetRegister (pUsb2ClassDevice,
                                      SMSC95XX_MII_DATA,
                                      &data))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyRead(): get SMSC95XX_MII_DATA fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    *phyWord = data & 0xFFFF;

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxPhyWrite - write a word into the PHY register
*
* This routine writes the word into the register of the PHY register.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxPhyWrite
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice, /* Device pointer */
    UINT8               offSet,           /* Offset of the MII register */
    UINT16              phyWord           /* Data to be written */
    )
    {
    UINT32 data = 0;

    if (OK != usb2Smsc95xxPhyBusyWait (pUsb2ClassDevice))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyWrite(): phy busy wait fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                      SMSC95XX_MII_DATA,
                                      phyWord))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyWrite(): set SMSC95XX_MII_DATA fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    data = (USB2_SMSC95XX_PHY_ADDR << SMSC95XX_MII_ACC_PHYADDR_BIT) |
           ((UINT32) offSet << SMSC95XX_MII_ACC_REG_BIT) |
           SMSC95XX_MII_ACC_MII_WRITE |
           BIT (SMSC95XX_MII_ACC_BUSY_BIT);

    if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                      SMSC95XX_MII_ACC,
                                      data))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyWrite(): set SMSC95XX_MII_ACC fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    if (OK != usb2Smsc95xxPhyBusyWait (pUsb2ClassDevice))
        {
        USB2_END_ERR ("usb2Smsc95xxPhyWrite(): write timeout\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxWriteMac - write a word into the PHY register
*
* This routine writes the word into the register of the PHY register.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxWriteMac
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice, /* pointer to  device */
    UINT8 *             macAddr           /* Data to be written */
    )
    {
    /* TODO: finish up this routine */
    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxEepromRead - read the data from EEPROM
*
* This routine reads the contents of external EEPROM.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxEepromRead
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice,   /* Device pointer */
    UINT32              offset,             /* Offset of the read data */
    UINT32              len,                /* Length of the read data */
    UINT8 *             buffer              /* Pointer to the read buffer*/
    )
    {
    UINT32              i;
    UINT32              data;

    /* check EEPROM is not busy */
    
    if (OK != usb2Smsc95xxWaitRegisterBit (pUsb2ClassDevice,
                                          LAN78XX_E2P_CMD,
                                          BIT (SMSC95XX_E2P_CMD_EPC_BUSY_BIT),
                                          0 << SMSC95XX_E2P_CMD_EPC_BUSY_BIT,
                                          1000))
        {
        USB2_END_ERR ("usb2Smsc95xxEepromRead(): "
                      "wait SMSC95XX_E2P_CMD_EPC_BUSY_BIT timeout\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR; 
        }

    for (i = 0; i < len; i++)
        {
        data = (offset & SMSC95XX_E2P_CMD_EPC_ADDR_MASK) | 
               SMSC95XX_EPC_CMD_READ_CMD | BIT(SMSC95XX_E2P_CMD_EPC_BUSY_BIT);
        if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                          SMSC95XX_E2P_CMD,
                                          data))
            {
            return ERROR;
            }

        if (OK != usb2Smsc95xxWaitRegisterBit (pUsb2ClassDevice,
                                              LAN78XX_E2P_CMD,
                                              BIT (SMSC95XX_E2P_CMD_EPC_BUSY_BIT),
                                              0 << SMSC95XX_E2P_CMD_EPC_BUSY_BIT,
                                              1000))
            {
            USB2_END_ERR ("usb2Smsc95xxEepromRead(): "
                          "wait SMSC95XX_E2P_CMD_EPC_BUSY_BIT timeout\n",
                          1, 2, 3, 4, 5, 6);
            return ERROR;
            }

        data = 0;
        if (OK != usb2Smsc95xxGetRegister (pUsb2ClassDevice,
                                          SMSC95XX_E2P_DATA,
                                          &data))
            {
            return ERROR;
            }
        buffer[i] = data & 0xFF;
        offset++;
        }

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxReadEepromMac - read the MAC address from EEPROM
*
* This function get the MAC address from Smsc95xx external EEPROM. 
*
* RETURNS: OK, or ERROR if cannnot read the valid MAC address.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxReadEepromMac
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice,
    UINT8               macAddress[6]
    )
    {
    UINT8               indicator = 0;

    memset(macAddress, 0x0, 6);

    /* check the EEPROM indicator first */

    if (OK != usb2Smsc95xxEepromRead (pUsb2ClassDevice, 0, 1, &indicator))
        {
        return ERROR;
        }

    if (indicator != 0xA5)
        {
        USB2_END_ERR ("usb2Smsc95xxReadEepromMac(): cannot find EEPROM"
                      "indicator:0x%x\n", indicator, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* read the MAC address */

    if (OK != usb2Smsc95xxEepromRead (pUsb2ClassDevice, 1, 6, macAddress))
        {
        return ERROR;
        }

    /* check the MAC address */

    if ((macAddress[0] & 0x01) == 0x01 || 
        (macAddress[0] == 0 
         && macAddress[1] == 0
         && macAddress[2] == 0
         && macAddress[3] == 0
         && macAddress[4] == 0
         && macAddress[5] == 0)) /* multcast or all zero address is invalid */
        {
        USB2_END_ERR ("usb2Smsc95xxReadEepromMac(): "
                      "invalid EEPROM MAC:%02x:%02x:%02x:%02x:%02x:%02x\n",
                      macAddress[0], macAddress[1], macAddress[2], 
                      macAddress[3], macAddress[4], macAddress[5]);
        memset(macAddress, 0x0, 6);
        return ERROR;
        }

    USB2_END_INFO ("usb2Smsc95xxReadEepromMac(): "
                   "read EEPROM MAC:%02x:%02x:%02x:%02x:%02x:%02x\n",
                   macAddress[0], macAddress[1], macAddress[2], 
                   macAddress[3], macAddress[4], macAddress[5]);

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxOtpRead - read the data from OTP
*
* This routine reads the contents of OTP.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxOtpRead
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice,   /* Device pointer */
    UINT32              offset,             /* Offset of the read data */
    UINT32              len,                /* Length of the read data */
    UINT8 *             buffer              /* Pointer to the read buffer*/
    )
    {
    UINT32              i;
    UINT32              data;
    
    if (OK != usb2Smsc95xxGetRegister (pUsb2ClassDevice,
                                      LAN78XX_OTP_PWR_DN,
                                      &data))
        {
        return ERROR;
        }

    if (data & BIT (LAN78XX_OTP_PWR_DN_PWRDN_N_BIT))
        {
        data &= ~ BIT (LAN78XX_OTP_PWR_DN_PWRDN_N_BIT);
        if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice, 
                                          LAN78XX_OTP_PWR_DN,
                                          data))
            {
            return ERROR;
            }

        if (OK != usb2Smsc95xxWaitRegisterBit(pUsb2ClassDevice,
                                             LAN78XX_OTP_PWR_DN,
                                             BIT (LAN78XX_OTP_PWR_DN_PWRDN_N_BIT),
                                             0 << LAN78XX_OTP_PWR_DN_PWRDN_N_BIT,
                                             1000))
            {
            USB2_END_ERR ("usb2Smsc95xxOtpRead(): "
                          "wait LAN78XX_OTP_PWR_DN_PWRDN_N_BIT timeout\n",
                          1, 2, 3, 4, 5, 6);
            return ERROR;
            }
        }

    for (i = 0; i < len; i++)
        {
        data = (offset >> 8) & LAN78XX_OTP_ADDR1_MASK;
        if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                          LAN78XX_OTP_ADDR1,
                                          data))
            {
            return ERROR;
            }

        data = offset & LAN78XX_OTP_ADDR2_MASK;
        if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                          LAN78XX_OTP_ADDR2,
                                          data))
            {
            return ERROR;
            }

        if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                          LAN78XX_OTP_FUNC_CMD,
                                          LAN78XX_OTP_FUNC_CMD_READ))
            {
            return ERROR;
            }

        if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                          LAN78XX_OTP_CMD_GO,
                                          BIT (LAN78XX_OTP_CMD_GO_GO_BIT)))
            {
            return ERROR;
            }

        if (OK != usb2Smsc95xxWaitRegisterBit (pUsb2ClassDevice,
                                              LAN78XX_OTP_STATUS,
                                              BIT (LAN78XX_OTP_STATUS_BUSY_BIT),
                                              0 << LAN78XX_OTP_STATUS_BUSY_BIT,
                                              1000))
            {
            USB2_END_ERR ("usb2Smsc95xxOtpRead(): "
                          "wait LAN78XX_OTP_STATUS_BUSY_BIT timeout\n",
                          1, 2, 3, 4, 5, 6);
            return ERROR;
            }

        data = 0;
        if (OK != usb2Smsc95xxGetRegister (pUsb2ClassDevice,
                                          LAN78XX_OTP_RD_DATA,
                                          &data))
            {
            return ERROR;
            }

        buffer[i] = data & 0xFF;
        offset++;
        }

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxReadOtpMac - read the MAC address from OTP
*
* This function get the MAC address from Smsc95xx OTP. 
*
* RETURNS: OK, or ERROR if cannnot read the valid MAC address.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxReadOtpMac
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice,
    UINT8               macAddress[6]
    )
    {
    /* TODO: finish this routine */
    return ERROR;
    }

/*******************************************************************************
*
* usb2Smsc95xxReadMac - read the MAC address from HW
*
* This function get the MAC address from Smsc95xx chip. 
* First read from external EEPROM.
* If it is not valid, then read it from OTP.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxReadMac
    (
    USB2_END_DEVICE *   pDevice       /* Device pointer */
    )
    {
    UINT32              hwCfg;
    UINT32              data;
    UINT32              chipID;

    chipID = pDevice->pAdapterInfo->deviceId.uProductID;

    /*
     * Get MAC address from ERPROM.
     * For Smsc95xx, some EEPROM pins are muxed with LED function.
     * Disable LED function first.
     */

    if (chipID == LAN78XX_CHIP_ID_7800 || chipID == LAN78XX_CHIP_ID_7850)
        {
        if (OK != usb2Smsc95xxGetRegister (pDevice->pUsb2ClassDevice,
                                          LAN78XX_HW_CFG,
                                          &hwCfg))
        {
        return ERROR;
        }
        data = hwCfg;
        data &= ~(BIT (LAN78XX_HW_CFG_LED0_EN_BIT) | 
                  BIT (LAN78XX_HW_CFG_LED1_EN_BIT));
        if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice, 
                                          LAN78XX_HW_CFG,
                                          data))
            {
            return ERROR;
            }
        }

    /* read MAC address from EEPROM */
    
    if (OK == usb2Smsc95xxReadEepromMac (pDevice->pUsb2ClassDevice,
                                        pDevice->macAddress))
        {
        
        /* re-enable LED pins functions */
    
        if (chipID == LAN78XX_CHIP_ID_7800 || chipID == LAN78XX_CHIP_ID_7850)
            {
            if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice, 
                                              LAN78XX_HW_CFG,
                                              hwCfg))
                {
                return ERROR;
                }
            }

        return OK;
        }

    /* if EEPROM MAC address is not ok, read it from OTP. */

    if (OK != usb2Smsc95xxReadOtpMac (pDevice->pUsb2ClassDevice,
                                     pDevice->macAddress))
        {
        return ERROR;
        }

    return OK;
    }
    
/*******************************************************************************
*
* usb2Smsc95xxReadDtbMac - read the MAC address from DTB file
*
* This function get the MAC address from DTB file. 
*
* RETURNS: OK returned, or ERROR if cannot get the MAC address.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxReadDtbMac
    (
    UINT8           macAddress[6]
    )
    {
    INT32           offset;
    INT32           valLen;
    const void *    pValue = NULL;

    offset = vxFdtNodeOffsetByCompatible(0, "usb424,7800");
    if(offset < 0)
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset: find usb netcard node fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;;
        }

    pValue = vxFdtPropGet (offset, "local-mac-address", &valLen);
    if ((pValue != NULL) && (valLen == 6))
        {
        memcpy (macAddress, (UINT8 *) pValue, 6);
        USB2_END_INFO ("usb2Smsc95xxReadDtbMac(): "
                       "read DTB MAC:%02x:%02x:%02x:%02x:%02x:%02x\n",
                       macAddress[0], macAddress[1], macAddress[2], 
                       macAddress[3], macAddress[4], macAddress[5]);
        return OK;
        }
    else 
        {
        return ERROR;
        }
    }

/*******************************************************************************
*
* usb2Smsc95xxHWReset - reset the Smsc95xx Chip
*
* This function reset the Smsc95xx chip, include MAC and PHY.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxHWReset
    (
    USB2_CLASS_DEVICE * pUsb2ClassDevice  /* Device pointer */
    )
    {
    UINT32              data;

    /* reset MAC */

    if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice, 
                                      SMSC95XX_HW_CFG,
                                      BIT (SMSC95XX_HW_CFG_LRST_BIT)))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset: set SMSC95XX_HW_CFG fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    if (OK != usb2Smsc95xxWaitRegisterBit (pUsb2ClassDevice, 
                                          SMSC95XX_HW_CFG,
                                          BIT (SMSC95XX_HW_CFG_LRST_BIT),
                                          0 << SMSC95XX_HW_CFG_LRST_BIT,
                                          1000))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset():"
                      " wait SMSC95XX_HW_CFG_LRST_BIT timeout\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* reset PHY */

    if (OK != usb2Smsc95xxGetRegister (pUsb2ClassDevice,
                                      SMSC95XX_PMT_CTL,
                                      &data))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset(): get SMSC95XX_PMT_CTL fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }
    data |= BIT (SMSC95XX_PMT_CTL_PHY_RST_BIT);
    if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                      SMSC95XX_PMT_CTL,
                                      data))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset(): set SMSC95XX_PMT_CTL fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    if (OK != usb2Smsc95xxWaitRegisterBit (pUsb2ClassDevice,
                                          SMSC95XX_PMT_CTL,
                                          BIT (SMSC95XX_PMT_CTL_PHY_RST_BIT),
                                          0 << SMSC95XX_PMT_CTL_PHY_RST_BIT,
                                          1000))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset():"
                      " wait SMSC95XX_PMT_CTL_PHY_RST_BIT timeout\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    if (OK != usb2Smsc95xxWaitRegisterBit (pUsb2ClassDevice,
                                          SMSC95XX_PMT_CTL,
                                          BIT (SMSC95XX_PMT_CTL_READY_BIT),
                                          BIT (SMSC95XX_PMT_CTL_READY_BIT),
                                          1000))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset():"
                      " wait SMSC95XX_PMT_CTL_READY_BIT timeout\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

   if (OK != usb2Smsc95xxGetRegister (pUsb2ClassDevice,
                                      SMSC95XX_HW_CFG,
                                      &data))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset(): get SMSC95XX_HW_CFG fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }
    data |= BIT (SMSC95XX_HW_CFG_BIR_BIT);
    if (OK != usb2Smsc95xxSetRegister (pUsb2ClassDevice,
                                      SMSC95XX_HW_CFG,
                                      data))
        {
        USB2_END_ERR ("usb2Smsc95xxHWReset(): set SMSC95XX_HW_CFG fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxStatusCheck - get the Smsc95xx link status
*
* This routine gets the link status of the device, since some of such devices
* can not support Interrupt In transfer, even they claim to have Interrupt
* In endpoint.
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usb2Smsc95xxStatusCheck
    (
    VOID *                   pDev
    )
    {
    UINT16                   phyWord = 0;
    USB2_END_DEVICE *        pDevice = NULL;
    pUSB2_END_CONFIG_FLAG    pConfigFlag = NULL;

    pDevice = (USB2_END_DEVICE *) pDev;

    /* Parameter valudify */

    /* TODO */
    return;
    }

/*******************************************************************************
*
* usb2Smsc95xxDataPacking - wrap the packet to transfer
*
* This function wrap the packet according the chipset spec
*
* RETURNS: N/A
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL VOID usb2Smsc95xxDataPacking
    (
    UINT8 *     pBuf,
    UINT32 *    pActLen
    )
    {
    UINT32      len = * pActLen;
    UINT32 *    pBufHead = (UINT32 *) pBuf;
    
    /*
     * (Required by the device)configure the first 8 Bytes,
     * TX command A and TX command B
     */

    /* TX command A setting */

    pBufHead[0] = (UINT32) ((len & USB2_LAN78XX_TXCOMMANDA_LEN_MASK) |
                            (BIT (USB2_LAN78XX_TXCOMMANDA_FCS_BIT)));

    /* TX command B setting */

    pBufHead[1] = (UINT32) 0;

    *pActLen += 8;
    
#ifdef LAN78XX_DBG
    memcpy (gtxbuf, pBuf, len);
#endif

    return ;
    }


/*******************************************************************************
*
* usb2Smsc95xxDataUnPacking - unwrap the packet received
*
* This routine unwraps the packet recevied according to the chipset spec.
*
* RETURNS: The PAD data length.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL UINT32 usb2Smsc95xxDataUnPacking
    (
    unsigned char **    ppBuf,
    UINT32 *            pPacketLen,
    UINT32 *            pActLen
    )
    {
    UINT32              len = 0;
    UINT32 *            pBuffer = (UINT32 *) *ppBuf;
    UINT32              revLen  = *pPacketLen;

    if (revLen < 4)
        {
        USB2_END_ERR ("usb2Smsc95xxDataUnPacking: invalid revLen %d\n",
                      revLen, 2, 3, 4, 5, 6);
        *pActLen = 0;
        *pPacketLen = 0;
        return 0;
        }

    if (0 != (pBuffer[0] & BIT (USB2_LAN78XX_RXCOMMANDA_RXE_BIT)))
        {
        USB2_END_ERR ("usb2Smsc95xxDataUnPacking: RX error\n",
                      1, 2, 3, 4, 5, 6);
        *pActLen = 0;
        *pPacketLen = 0;
        }
    else
        {
        len = (pBuffer[0] & USB2_LAN78XX_RXCOMMANDA_LEN_MASK);
        if (len > revLen - 4)
            {
            USB2_END_ERR ("usb2Smsc95xxDataUnPacking: "
                          "len (%d) > revLen (%d) - 4\n",
                          len, revLen, 3, 4, 5, 6);
            *pActLen = 0;
            *pPacketLen = 0;
            }
        else
            {
            *pActLen = len;
            *pPacketLen = 0;
            *ppBuf += 10; /* skip rxcmd A, rxcmd B and rxx cmd C */
            
#ifdef LAN78XX_DBG
            memcpy (grxbuf, pBuffer, len + 10);
#endif
            }
        }

    return 0;
    }

/*******************************************************************************
*
* usb2Smsc95xxHWConfig - configure the Smsc95xx Chip
*
* This function initializes the device specific registers for Smsc95xx chip.
* This function will reset the MAC and loads the Ethernet ID read from
* EEPROM.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxHWConfig
    (
    VOID *              pDev
    )
    {
    UINT32              data;
    UINT16              phyWord;
    USB2_END_DEVICE *   pDevice = NULL;
    BOOL                getUbootMac = FALSE;

    /* Get the device information, for the descriptors */

    pDevice = (USB2_END_DEVICE *) pDev;
    if (NULL == pDevice)
        {
        USB2_END_ERR ("usb2Smsc95xxHWConfig(): The USB-Ethernet device "
                      "not connected\n", 1, 2, 3, 4, 5, 6);
        return ERROR;
        }

#ifdef SMSC95XX_DBG
    gpUsb2ClassDevice = pDevice->pUsb2ClassDevice;
#endif

    /* get MAC address from U-Boot initialized register first */

    if (OK != usb2Smsc95xxGetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_RX_ADDRL,
                                      &data))
        {
        return ERROR;
        }
    memcpy (pDevice->macAddress, &data, 4);
                                      
    if (OK != usb2Smsc95xxGetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_RX_ADDRH,
                                      &data))
        {
        return ERROR;
        }
    memcpy(pDevice->macAddress + 4, &data, 2);

    /* check the MAC address, multcast or all zero address is invalid */

    if ((pDevice->macAddress[0] & 0x01) == 0x01 || 
        (pDevice->macAddress[0] == 0 
          && pDevice->macAddress[1] == 0
          && pDevice->macAddress[2] == 0
          && pDevice->macAddress[3] == 0
          && pDevice->macAddress[4] == 0
          && pDevice->macAddress[5] == 0))
        {
        getUbootMac = FALSE;
        
        USB2_END_ERR ("usb2Smsc95xxReadMac(): "
                      "read invalid Uboot init MAC:"
                      "%02x:%02x:%02x:%02x:%02x:%02x\n",
                      pDevice->macAddress[0], pDevice->macAddress[1], 
                      pDevice->macAddress[2], pDevice->macAddress[3], 
                      pDevice->macAddress[4], pDevice->macAddress[5]);
        }
    else
        {
        getUbootMac = TRUE;

        USB2_END_INFO ("usb2Smsc95xxReadMac(): "
                       "read Uboot init MAC:%02x:%02x:%02x:%02x:%02x:%02x\n",
                       pDevice->macAddress[0], pDevice->macAddress[1], 
                       pDevice->macAddress[2], pDevice->macAddress[3], 
                       pDevice->macAddress[4], pDevice->macAddress[5]);
        }

    /* reset the chip */

    if (OK != usb2Smsc95xxHWReset (pDevice->pUsb2ClassDevice))
        {
        USB2_END_ERR ("usb2Smsc95xxHWConfig(): The USB-Ethernet device "
                      "not connected\n", 1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* if cannot get Uboot init MAC address, try to get it from EEPROM&OTP */
    
    if (!getUbootMac)
        {
        if (OK != usb2Smsc95xxReadMac (pDevice))
            {

            /* get from EEPROM/OTP fail, then get the MAC address from DTB */

            if (OK != usb2Smsc95xxReadDtbMac (pDevice->macAddress))
                {
                
                /* get the MAC address fail, using the debug MAC address */

                USB2_END_ERR ("usb2Smsc95xxHWConfig(): Get MAC address fail\n",
                              1, 2, 3, 4, 5, 6);

                pDevice->macAddress[0] = 0x00;
                pDevice->macAddress[1] = 0x11;
                pDevice->macAddress[2] = 0x22;
                pDevice->macAddress[3] = 0x33;
                pDevice->macAddress[4] = 0x44;
                pDevice->macAddress[5] = 0x55;
                }
            }
        }
#if 0
    if (OK != usb2Smsc95xxWriteMac (pDevice->pUsb2ClassDevice,
                                   pDevice->macAddress))
        {
        return ERROR;
        }

    /* init PHY */

    phyWord = 0x1E1;
    if (OK != usb2Smsc95xxPhyWrite (pDevice->pUsb2ClassDevice,
                                   LAN78XX_PHY_AN_ADV,
                                   phyWord))
        {
        return ERROR;
        }

    phyWord = 0x300;
    if (OK != usb2Smsc95xxPhyWrite (pDevice->pUsb2ClassDevice,
                                   LAN78XX_PHY_1000BASE_T_CTRL,
                                   phyWord))
        {
        return ERROR;
        }

    phyWord = 0x1340;
    if (OK != usb2Smsc95xxPhyWrite (pDevice->pUsb2ClassDevice,
                                   LAN78XX_PHY_MCR,
                                   phyWord))
        {
        return ERROR;
        }

    /* init MAC */

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_INT_STS, 0xFFFFFFFF))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_BURST_CAP, 0))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_BULK_IN_DLY, 0x800))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                       LAN78XX_RFE_CTL,
                                       BIT (LAN78XX_RFE_CTL_BCAST_EN_BIT) | 
                                       BIT (LAN78XX_RFE_CTL_DA_PERFECT_BIT)))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_FCT_RX_FIFO_END,
                                      (LAN78XX_MAX_RX_FIFO_SIZE - 512) / 512))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_FCT_TX_FIFO_END,
                                      (LAN78XX_MAX_TX_FIFO_SIZE - 512) / 512))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_FLOW, 0))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxGetRegister (pDevice->pUsb2ClassDevice, 
                                      LAN78XX_MAC_CR, &data))
        {
        return ERROR;
        }
    data |= BIT (LAN78XX_MAC_CR_AUTO_DUPLEX_BIT) | 
            BIT (LAN78XX_MAC_CR_AUTO_SPEED_BIT) | 
            BIT (LAN78XX_MAC_CR_ADP_BIT);
    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_MAC_CR, data))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_MAC_TX, 
                                      BIT (LAN78XX_MAC_TX_TXEN_BIT)))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_FCT_TX_CTL, 
                                      BIT (LAN78XX_FCT_TX_CTL_EN_BIT)))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_MAC_RX,
                                      LAN78XX_MAC_RX_MAX_SIZE << 16 |
                                      BIT (LAN78XX_MAC_RX_FCS_STRIP_BIT) | 
                                      BIT (LAN78XX_MAC_RX_RXEN_BIT)))
        {
        return ERROR;
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      LAN78XX_FCT_RX_CTL, 
                                      BIT (LAN78XX_FCT_RX_CTL_EN_BIT)))
        {
        return ERROR;
        }
#endif
    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxSetFlag - reconfigure the interface under us
*
* Reconfigure the interface setting promiscuous/ broadcast etc modes, and
* changing the multicast interface list.
*
* RETURNS: OK returned, or ERROR if unsuccessfully.
*
* ERRNO: N/A
*
* \NOMANUAL
*/

LOCAL STATUS usb2Smsc95xxSetFlag
    (
    END_OBJ *           endObj
    )
    {
    USB2_END_DEVICE *   pDevice = NULL;
    UINT32              data;

    if (endObj == NULL)
        {
        return ERROR;
        }

    pDevice = (USB2_END_DEVICE *) endObj;

    if (OK != usb2Smsc95xxGetRegister (pDevice->pUsb2ClassDevice,
                                      SMSC95XX_MAC_CR,
                                      &data))
        {
        USB2_END_ERR ("usb2Smsc95xxSetFlag(): get LAN78XX_RFE_CTL fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    /* Set the modes asked for */

    if (END_FLAGS_GET (&pDevice->endObj) & IFF_PROMISC)
        {
        data |= BIT(SMSC95XX_MAC_CR_PROMISC_BIT);
	data &= ~BIT(SMSC95XX_MAC_CR_ALLMULTI_BIT);
        }
    else if (END_FLAGS_GET (&pDevice->endObj) & IFF_ALLMULTI)
        {
        data |= BIT(SMSC95XX_MAC_CR_ALLMULTI_BIT);
	data &= ~BIT(SMSC95XX_MAC_CR_PROMISC_BIT);
        }
    else
        {
	data &= ~(BIT(SMSC95XX_MAC_CR_PROMISC_BIT) | BIT(SMSC95XX_MAC_CR_ALLMULTI_BIT));
       // data |= BIT (LAN78XX_RFE_CTL_BCAST_EN_BIT);
        }

    if (OK != usb2Smsc95xxSetRegister (pDevice->pUsb2ClassDevice,
                                      SMSC95XX_MAC_CR,
                                      data))
        {
        USB2_END_ERR ("usb2Smsc95xxSetFlag(): set SMSC95XX_MAC_CR fail\n",
                      1, 2, 3, 4, 5, 6);
        return ERROR;
        }

    return OK;
    }

/*******************************************************************************
*
* usb2Smsc95xxInit - register Smsc95xx support to general USB-Ethernet class driver
*
* This routine is called once to add the support for Smsc95xx devcie.
* Only need to add the supported devcie table to the USB-Ethernet global list.
*
* RETURNS: OK, or ERROR if the the driver could not be initialized.
*
* ERRNO: N/A
*/

STATUS usb2Smsc95xxInit
    (
    char * pName /* Default base name that will be used, or NULL if no names */
    )

    {
     STATUS status;

    /*
     * If you devcie did not included in Usb2Smsc95xxAdapterList, and it also
     * uses Smsc95xx serial chipset, you can add the device information to
     * the list mamually.
     */

     /* Add the Smsc95xx device table to the USB-Ethernet global list */

     if (NULL == pName)
        {
        status = usb2EndDeviceListAdd ("usbSmsc95xx", Usb2Smsc95xxAdapterList, 
                 sizeof(Usb2Smsc95xxAdapterList) / sizeof(USB2_END_ADAPTER_INFO));
        }
     else
        {
        status = usb2EndDeviceListAdd (pName, Usb2Smsc95xxAdapterList, 
                 sizeof(Usb2Smsc95xxAdapterList) / sizeof(USB2_END_ADAPTER_INFO));
        }

     return status;
    }

/*******************************************************************************
*
* usb2Smsc95xxDeInit - unregister Smsc95xx support from general USB-Ethernet 
*                     class driver
*
* This routine is to de-initialize the Smsc95xx driver from supported from the
* general USB-Ethernet class driver by removing the Smsc95xx table.
*
* RETURNS: N/A
*
* ERRNO: N/A
*/

void usb2Smsc95xxDeInit (void)
    {

    /*
     * DeInit to unsupported the Smsc95xx devices
     * Sample remove the supported table from the global device list
     */

    usb2EndDeviceListRemove (Usb2Smsc95xxAdapterList);
    }


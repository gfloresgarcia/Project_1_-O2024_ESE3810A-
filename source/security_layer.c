/*
 * security_layer.c
 *
 *  Created on: 10 sep. 2024
 *      Author: Gustavo Flores
 */

#include "security_layer.h"
#include "security_layer_cfg.h"
#include "fsl_debug_console.h"
#include "Driver_ETH_MAC.h"
#include "fsl_enet.h"
#include "fsl_enet_cmsis.h"
#include "fsl_enet_phy_cmsis.h"
#include "fsl_phy.h"

#include "aes.h"
#include "fsl_crc.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/
uint8_t g_macAddr[6] = MAC_ADDRESS;

framesTxRx encrypted;
framesTxRx encryptedWithCRC;

uint8_t flagRX = 0, flagTx = 0;

void ENET_SignalEvent_t(uint32_t event)
{
    if (event == ARM_ETH_MAC_EVENT_RX_FRAME)
    {
        uint32_t size;
        uint32_t len;

        /* Get the Frame size */
        size = EXAMPLE_ENET.GetRxFrameSize();
        /* Call ENET_ReadFrame when there is a received frame. */
        if (size != 0)
        {
            /* Received valid frame. Deliver the rx buffer with the size equal to length. */
            uint8_t *data = (uint8_t *)malloc(size);
            if (data)
            {
                len = EXAMPLE_ENET.ReadFrame(data, size);
                if (size == len)
                {
                    //I can set a flag to indicate that a message was received
                	flagRX = 1;
                }
                free(data);
            }
        }
    }
    if (event == ARM_ETH_MAC_EVENT_TX_FRAME)
    {
    	//I can set a flag to evaluate if message was sent
    	flagTx = 1;
    }
}

void ENET_Initialization(void) {
	ARM_ETH_LINK_INFO linkInfo;

    EXAMPLE_ENET.Initialize(ENET_SignalEvent_t);
    EXAMPLE_ENET.PowerControl(ARM_POWER_FULL);
    EXAMPLE_ENET.SetMacAddress((ARM_ETH_MAC_ADDR *)g_macAddr);

    PRINTF("Wait for PHY initialization...\r\n");
    while (EXAMPLE_ENET_PHY.PowerControl(ARM_POWER_FULL) != ARM_DRIVER_OK)
    {
        PRINTF("PHY Auto-negotiation failed, please check the cable connection and link partner setting.\r\n");
    }

    EXAMPLE_ENET.Control(ARM_ETH_MAC_CONTROL_RX, 1);
    EXAMPLE_ENET.Control(ARM_ETH_MAC_CONTROL_TX, 1);
    PRINTF("Wait for PHY link up...\r\n");
    do
    {
        if (EXAMPLE_ENET_PHY.GetLinkState() == ARM_ETH_LINK_UP)
        {
            linkInfo = EXAMPLE_ENET_PHY.GetLinkInfo();
            EXAMPLE_ENET.Control(ARM_ETH_MAC_CONFIGURE, linkInfo.speed << ARM_ETH_MAC_SPEED_Pos |
                                                            linkInfo.duplex << ARM_ETH_MAC_DUPLEX_Pos |
                                                            ARM_ETH_MAC_ADDRESS_BROADCAST);
            break;
        }
    } while (1);
}

void encryptPackage(uint8_t* toEncrypt) {
	uint8_t key[] = KEY;
	uint8_t iv[]  = IV;

	struct AES_ctx ctx;

	size_t toEncryptLength;
	encrypted.length = 0;
	memset(encrypted.frame, 0, sizeof(encrypted.frame));

	AES_init_ctx_iv(&ctx, key, iv);

	toEncryptLength = strlen(toEncrypt);
	encrypted.length = toEncryptLength + (16 - (toEncryptLength % 16));
	memcpy(encrypted.frame, toEncrypt, toEncryptLength);

	AES_CBC_encrypt_buffer(&ctx, encrypted.frame, encrypted.length);
}

static void InitCrc32(CRC_Type *base, uint32_t seed) {
    crc_config_t config;

    config.polynomial         = 0x04C11DB7U;
    config.seed               = seed;
    config.reflectIn          = true;
    config.reflectOut         = true;
    config.complementChecksum = true;
    config.crcBits            = kCrcBits32;
    config.crcResult          = kCrcFinalChecksum;

    CRC_Init(base, &config);
}

uint32_t calculateCRC32(uint8_t* toCalculate, size_t length) {
	CRC_Type *base = CRC0;
	uint32_t crc32 = 0;

	InitCrc32(base, 0xFFFFFFFFU);
	CRC_WriteData(base, (uint8_t *)&toCalculate[0], length);
	crc32 = CRC_Get32bitResult(base);

	return crc32;
}

void addCRC32(uint8_t* toAdd, size_t length, uint32_t add) {
	memcpy(encryptedWithCRC.frame, toAdd, length);
	encryptedWithCRC.frame[length] = (uint8_t)((add & 0xFF000000) >> 24);
	encryptedWithCRC.frame[length+1] = (uint8_t)((add & 0x00FF0000) >> 16);
	encryptedWithCRC.frame[length+2] = (uint8_t)((add & 0x0000FF00) >> 8);
	encryptedWithCRC.frame[length+3] = (uint8_t)((add & 0x000000FF));
	encryptedWithCRC.length+=4;
}

SL_result sendPackageWithSecurityLayer(uint8_t* message) {
	uint32_t crc32 = 0;
	flagRX = 0, flagTx = 0;

	encryptPackage(message);
	crc32 = calculateCRC32(encrypted.frame, encrypted.length);
	addCRC32(encrypted.frame, encrypted.length, crc32);

	if (EXAMPLE_ENET.SendFrame(&encryptedWithCRC.frame[0], encryptedWithCRC.length, ARM_ETH_MAC_TX_FRAME_EVENT) == ARM_DRIVER_OK && flagTx == 1) {
		return packageSent_OK;
		//SDK_DelayAtLeastUs(1000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
	}
	else {
		return packageSent_ERROR;
	}
}

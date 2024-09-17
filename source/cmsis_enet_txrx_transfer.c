/*
 * Copyright 2017-2020 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*  Standard C Included Files */
#include <string.h>
/*  SDK Included Files */
//#include "Driver_ETH_MAC.h"
#include "pin_mux.h"
#include "board.h"
#include "fsl_debug_console.h"
#include "stdlib.h"
#include "security_layer.h"

#include "fsl_common.h"
#include "fsl_sysmpu.h"
#include "fsl_phyksz8081.h"
#include "fsl_enet_mdio.h"
#include "RTE_Device.h"


/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
extern uint8_t flagRx;
extern uint8_t flagTx;

/*******************************************************************************
 * Code
 ******************************************************************************/
mdio_handle_t mdioHandle = {.ops = &enet_ops};
phy_handle_t phyHandle   = {.phyAddr = RTE_ENET_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &phyksz8081_ops};

uint32_t ENET0_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_CoreSysClk);
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Hardware Initialization. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Disable SYSMPU. */
    SYSMPU_Enable(SYSMPU, false);

    mdioHandle.resource.base        = ENET;
    mdioHandle.resource.csrClock_Hz = ENET0_GetFreq();

    PRINTF("\r\nENET example start.\r\n");

    /* Initialize the ENET module. */
    ENET_Initialization();

#if defined(PHY_STABILITY_DELAY_US) && PHY_STABILITY_DELAY_US
    /* Wait a moment for PHY status to be stable. */
    SDK_DelayAtLeastUs(PHY_STABILITY_DELAY_US, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
#endif

    while (1)
    {
    	SDK_DelayAtLeastUs(2000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

    	//In this section of the code i will to call function to send and receive
    	//Ethernet packages using the library security_layer
    	if (sendPackageWithSecurityLayer("Envia paquete 1 para prueba de cifrado y CRC32") == packageSent_OK) {
    		//Check flag RX
    		while(flagRx == 0); {
    			SDK_DelayAtLeastUs(500, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
    		}

    		receivePackageWithSecurityLayer();
    	}


    }
}

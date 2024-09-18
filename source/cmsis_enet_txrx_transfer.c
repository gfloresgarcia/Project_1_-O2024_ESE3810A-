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
#include "stdio.h"
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

//Structure to create 8 messages with different length and data
framesTxRx Test[8];


/*******************************************************************************
 * Code
 ******************************************************************************/
mdio_handle_t mdioHandle = {.ops = &enet_ops};
phy_handle_t phyHandle   = {.phyAddr = RTE_ENET_PHY_ADDRESS, .mdioHandle = &mdioHandle, .ops = &phyksz8081_ops};

uint32_t ENET0_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_CoreSysClk);
}

//Create function to construct test messages
static void fill_Messages(void) {
	Test[0].length = sprintf(Test[0].frame, "Prueba");
	Test[1].length = sprintf(Test[1].frame, "Derf");
	Test[2].length = sprintf(Test[2].frame, "Tripiante");
	Test[3].length = sprintf(Test[3].frame, "Flumpo");
	Test[4].length = sprintf(Test[4].frame, "Sistemas de comunicion para sistemas embebidos e IoT");
	Test[5].length = sprintf(Test[5].frame, "   ");
	Test[6].length = sprintf(Test[6].frame, "Oye tranquilo viejo. Bendiciones matinales. La UEMAFEM. Tripiante. Flumpo.");
	Test[7].length = sprintf(Test[7].frame, "El hermano de calceto, corbato. La UEMAFEM.");
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
    	fill_Messages();

    	for (int i = 0; i < 8; i++) {
    		PRINTF("\r\n\r\nEnvio de frame %d ...\r\n", i + 1);

        	SDK_DelayAtLeastUs(2000000, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);

        	//In this section of the code i will to call function to send and receive
        	//Ethernet packages using the library security_layer
        	if (sendPackageWithSecurityLayer(&Test[i]) == packageSent_OK) {
        		//Check flag RX
        		while(flagRx == 0) {
        			SDK_DelayAtLeastUs(500, SDK_DEVICE_MAXIMUM_CPU_CLOCK_FREQUENCY);
        		}

        		receivePackageWithSecurityLayer();
        	}
    	}

    	while(1);
    }
}

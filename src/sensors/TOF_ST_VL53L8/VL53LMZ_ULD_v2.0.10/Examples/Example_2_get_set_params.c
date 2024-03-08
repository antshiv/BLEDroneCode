/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


/***********************************/
/*   VL53LMZ ULD get/set params   */
/***********************************/
/*
* This example shows the possibility of VL53LMZ to get/set params. It
* initializes the VL53LMZ ULD, set a configuration, and starts
* a ranging to capture 10 frames.
*
* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53lmz_api.h"

int example2(void)
{

	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	uint32_t 				integration_time_ms;
	VL53LMZ_Configuration 	Dev;			/* Sensor configuration */
	VL53LMZ_ResultsData 	Results;		/* Results data from VL53LMZ */

	
	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	* example, only the I2C address is used.
	*/
	Dev.platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	//Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	* from the default one (filled with 0x20 for this example).
	*/
	//status = vl53lmz_set_i2c_address(&Dev, 0x20);
	

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53LMZ sensor connected */
	status = vl53lmz_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53LMZ not detected at requested address\n");
		return status;
	}

	/* (Mandatory) Init VL53LMZ sensor */
	status = vl53lmz_init(&Dev);
	if(status)
	{
		printf("VL53LMZ ULD Loading failed\n");
		return status;
	}

	printf("VL53LMZ ULD ready ! (Version : %s)\n",
			VL53LMZ_API_REVISION);
			

	/*********************************/
	/*        Set some params        */
	/*********************************/

	/* Set resolution in 8x8. WARNING : As others settings depend to this
	 * one, it must be the first to use.
	 */
	status = vl53lmz_set_resolution(&Dev, VL53LMZ_RESOLUTION_8X8);
	if(status)
	{
		printf("vl53lmz_set_resolution failed, status %u\n", status);
		return status;
	}

	/* Set ranging frequency to 10Hz.
	 * Using 4x4, min frequency is 1Hz and max is 60Hz
	 * Using 8x8, min frequency is 1Hz and max is 15Hz
	 */
	status = vl53lmz_set_ranging_frequency_hz(&Dev, 10);
	if(status)
	{
		printf("vl53lmz_set_ranging_frequency_hz failed, status %u\n", status);
		return status;
	}

	/* Set target order to closest */
	status = vl53lmz_set_target_order(&Dev, VL53LMZ_TARGET_ORDER_CLOSEST);
	if(status)
	{
		printf("vl53lmz_set_target_order failed, status %u\n", status);
		return status;
	}

	/* Get current integration time */
	status = vl53lmz_get_integration_time_ms(&Dev, &integration_time_ms);
	if(status)
	{
		printf("vl53lmz_get_integration_time_ms failed, status %u\n", status);
		return status;
	}
	printf("Current integration time is : %d ms\n", (int)integration_time_ms);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53lmz_start_ranging(&Dev);

	loop = 0;
	while(loop < 10)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A3
		 * (GPIO 1) when a new measurement is ready */
 
		status = vl53lmz_check_data_ready(&Dev, &isReady);

		if(isReady)
		{
			vl53lmz_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 8x8 mode, we have a total
			 * of 64 zones to print. For this example, only the data of
			 * first zone are print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 64; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm\n",
					i,
					Results.target_status[VL53LMZ_NB_TARGET_PER_ZONE*i],
					Results.distance_mm[VL53LMZ_NB_TARGET_PER_ZONE*i]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&(Dev.platform), 5);
	}

	status = vl53lmz_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}

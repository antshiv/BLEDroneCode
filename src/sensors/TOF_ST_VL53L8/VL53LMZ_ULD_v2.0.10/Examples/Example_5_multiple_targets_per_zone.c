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
/*  VL53LMZ ULD multiple targets  */
/***********************************/
/*
* This example shows the possibility of VL53LMZ to get/set params. It
* initializes the VL53LMZ ULD, set a configuration, and starts
* a ranging to capture 10 frames.
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53lmz_api.h"

int example5(void)
{
	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i, j;
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
	/*	Set nb target per zone       */
	/*********************************/

	/* Each zone can output between 1 and 4 targets. By default the output
	 * is set to 1 targets, but user can change it using macro
	 * VL53LMZ_NB_TARGET_PER_ZONE located in file 'platform.h'.
	 */

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

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++)
			{
				/* Print per zone results. These results are the same for all targets */
				printf("Zone %3u : %2u, %6lu, %6lu, ",
					i,
					Results.nb_target_detected[i],
					Results.ambient_per_spad[i],
					Results.nb_spads_enabled[i]);

				for(j = 0; j < VL53LMZ_NB_TARGET_PER_ZONE; j++)
				{
					/* Print per target results. These results depends of the target nb */
					uint16_t idx = VL53LMZ_NB_TARGET_PER_ZONE * i + j;
					printf("Target[%1u] : %2u, %4d, %6lu, %3u, ",
						j,
						Results.target_status[idx],
						Results.distance_mm[idx],
						Results.signal_per_spad[idx],
						Results.range_sigma_mm[idx]);
				}
				printf("\n");
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

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
/*   VL53LMZ ULD calibrate Xtalk  */
/***********************************/
/*
* This example shows the possibility of VL53LMZ to calibrate Xtalk. It
* initializes the VL53LMZ ULD, perform a Xtalk calibration, and starts
* a ranging to capture 10 frames.

* In this example, we also suppose that the number of target per zone is
* set to 1 , and all output are enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "vl53lmz_api.h"
#include "vl53lmz_plugin_xtalk.h"

int example7(void)
{

	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	VL53LMZ_Configuration 	Dev;			/* Sensor configuration */
	VL53LMZ_ResultsData 	Results;		/* Results data from VL53LMZ */
	
	/* Buffer containing Xtalk data */
	uint8_t					xtalk_data[VL53LMZ_XTALK_BUFFER_SIZE];

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
	/*    Start Xtalk calibration    */
	/*********************************/

	/* Start Xtalk calibration with a 3% reflective target at 600mm for the
	 * sensor, using 4 samples.
	 */
	printf("Running Xtalk calibration...\n");

	status = vl53lmz_calibrate_xtalk(&Dev, 3, 4, 600);
	if(status)
	{
		printf("vl53lmz_calibrate_xtalk failed, status %u\n", status);
		return status;
	}else
	{
		printf("Xtalk calibration done\n");

		/* Get Xtalk calibration data, in order to use them later */
		status = vl53lmz_get_caldata_xtalk(&Dev, xtalk_data);

		/* Set Xtalk calibration data */
		status = vl53lmz_set_caldata_xtalk(&Dev, xtalk_data);
	}
	
	
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
			 * of 16 zones to print. For this example, only the data of first zone are 
			 * print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for(i = 0; i < 16; i++)
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

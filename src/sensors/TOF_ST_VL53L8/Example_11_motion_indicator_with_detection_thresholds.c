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


/************************************************************/
/*  VL53LMZ ULD motion indicator with detection thresholds */
/************************************************************/
/*
* This example shows how to use the motion indicator with detection threshold.
* This kind of configuration might be used for user detection applications.
* To use this example, user needs to be sure that macro
* VL53LMZ_DISABLE_MOTION_INDICATOR is NOT enabled (see file platform.h).
*/

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "tof_vl53l8.h"
#include "VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/inc/vl53lmz_api.h"
#include "VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/inc/vl53lmz_plugin_motion_indicator.h"
#include "VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/inc/vl53lmz_plugin_detection_thresholds.h"

// #define UNUSED(x) (void)(x)

/* This function needs to be filled by the customer. It allows knowing when
 * the VL53LMZ interrupt is raised on GPIO1. This is the only way to use detection thresholds.
 */
/*
int WaitForL5Interrupt(VL53LMZ_Configuration * pDev) {

	//Add your implementation here ...
	UNUSED(pDev);

	return 0;
}
*/
extern int WaitForL5Interrupt(VL53LMZ_Configuration * pDev);

int example12(const struct i2c_dt_spec *i2cSpec) {
	uint8_t 				status, loop, isAlive, isReady, i;
	VL53LMZ_Configuration 	*Dev;			/* Sensor configuration */
	VL53LMZ_Motion_Configuration 	motion_config;	/* Motion configuration*/
	VL53LMZ_ResultsData 	Results;		/* Results data from VL53LMZ */
	uint8_t device_id;
	uint8_t revision_id;
	Dev = (VL53LMZ_Configuration *)k_calloc(1, sizeof(VL53LMZ_Configuration));
	if(!Dev) {
		printk("Memory allocation failed\n");
		return -1;
	}
	Dev->revision_id = 22; 
	//memset(Dev, 0, sizeof(VL53LMZ_Configuration));
	printk("Loop starts");
	//Dev.platform.i2cSpec = i2cSpec;
	//status = vl53lmz_is_alive(Dev, &isAlive);
	status = WrByte(&(Dev->platform), 0x7fff, 0x00);
	status |= RdByte(&(Dev->platform), 1, &(Dev->revision_id));
	status |= RdByte(&(Dev->platform), 0, &(Dev->device_id));

	int RegisterAdress = 0x7fff;
	uint8_t p_value[1];
    //status = mcu_i2cTransfer(&vl53l8_1, &RegisterAdress, 1, p_value, 1);
	//Dev.platform.i2cSpec = i2cSpec;
	k_free(Dev);
	return 0;
}

int example11(const struct i2c_dt_spec *i2cSpec)
{
	/*********************************/
	/*   VL53LMZ ranging variables  */
	/*********************************/

	uint8_t 				status, loop, isAlive, isReady, i;
	VL53LMZ_Configuration 	Dev;			/* Sensor configuration */
	VL53LMZ_Motion_Configuration 	motion_config;	/* Motion configuration*/
	VL53LMZ_ResultsData 	Results;		/* Results data from VL53LMZ */

	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	* example, only the I2C address is used.
	*/
	Dev.platform.address = VL53LMZ_DEFAULT_I2C_ADDRESS;
	Dev.platform.i2cSpec = i2cSpec;

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

	//i2c_validation(&Dev);
    //status = vl53l5cx_test_i2c(&Dev);
	
	/*status = vl53lmz_is_alive(&Dev, &isAlive);
	if(!isAlive || status)
	{
		printf("VL53LMZ not detected at requested address\n");
		return status;
	}
	*/
	/* (Mandatory) Init VL53LMZ sensor */
	//printk('VL53LMZ ULD example11 : Initializing\n');
	status = vl53lmz_init(&Dev);
	if(status)
	{
		printk("VL53LMZ ULD Loading failed\n");
		return status;
	}

	printk("VL53LMZ ULD ready ! (Version : %s)\n",
			VL53LMZ_API_REVISION);


	/*********************************/
	/*   Program motion indicator    */
	/*********************************/

	/* Create motion indicator with resolution 8x8 */
	status = vl53lmz_motion_indicator_init(&Dev, &motion_config, VL53LMZ_RESOLUTION_8X8);
	if(status)
	{
		printf("Motion indicator init failed with status : %u\n", status);
		return status;
	}

	/* (Optional) Change the min and max distance used to detect motions. The
	 * difference between min and max must never be >1500mm, and minimum never be <400mm,
	 * otherwise the function below returns error 127 */
	status = vl53lmz_motion_indicator_set_distance_motion(&Dev, &motion_config, 1000, 2000);
	if(status)
	{
		printf("Motion indicator set distance motion failed with status : %u\n", status);
		return status;
	}

	/* If user want to change the resolution, he also needs to update the motion indicator resolution */
	//status = vl53lmz_set_resolution(&Dev, VL53LMZ_RESOLUTION_4X4);
	//status = vl53lmz_motion_indicator_set_resolution(&Dev, &motion_config, VL53LMZ_RESOLUTION_4X4);


	/* Set the device in AUTONOMOUS and set a small integration time to reduce power consumption */
	status = vl53lmz_set_resolution(&Dev, VL53LMZ_RESOLUTION_8X8);
	status = vl53lmz_set_ranging_mode(&Dev, VL53LMZ_RANGING_MODE_AUTONOMOUS);
	status = vl53lmz_set_ranging_frequency_hz(&Dev, 2);
	status = vl53lmz_set_integration_time_ms(&Dev, 10);


	/*********************************/
	/*  Program detection thresholds */
	/*********************************/

	/* In this example, we want 1 thresholds per zone for a 8x8 resolution */
	/* Create array of thresholds (size cannot be changed) */
	VL53LMZ_DetectionThresholds thresholds[VL53LMZ_NB_THRESHOLDS];

	/* Set all values to 0 */
	memset(&thresholds, 0, sizeof(thresholds));

	/* Add thresholds for all zones (64 zones in resolution 4x4, or 64 in 8x8) */
	for(i = 0; i < 64; i++){
		thresholds[i].zone_num = i;
		thresholds[i].measurement = VL53LMZ_MOTION_INDICATOR;
		thresholds[i].type = VL53LMZ_GREATER_THAN_MAX_CHECKER;
		thresholds[i].mathematic_operation = VL53LMZ_OPERATION_NONE;

		/* The value 44 is given as example. All motion above 44 will be considered as a movement */
		thresholds[i].param_low_thresh = 44;
		thresholds[i].param_high_thresh = 44;
	}

	/* The last thresholds must be clearly indicated. As we have 64
	 * checkers, the last one is the 63 */
	thresholds[63].zone_num = VL53LMZ_LAST_THRESHOLD | thresholds[63].zone_num;

	/* Send array of thresholds to the sensor */
	vl53lmz_set_detection_thresholds(&Dev, thresholds);

	/* Enable detection thresholds */
	vl53lmz_set_detection_thresholds_enable(&Dev, 1);


	/*********************************/
	/*         Ranging loop          */
	/*********************************/

	status = vl53lmz_start_ranging(&Dev);
	printf("Waiting for a movement into the FOV between 1m and 2m...\n");

	loop = 0;
	while(loop < 10)
	{
		/* Function WaitForL5Interrupt() does not exists, and must be
		 * implemented by user. It allows catching the interrupt raised on
		 * pin A3 (INT), when the checkers detect the programmed
		 * conditions.
		 */

		isReady = WaitForL5Interrupt(&Dev);

		if(isReady)
		{
			vl53lmz_get_ranging_data(&Dev, &Results);

			/* As the sensor is set in 8x8 mode by default, we have a total
			 * of 64 zones to print. For this example, only the data of first zone are
			 * print */
			for(i = 0; i < 64; i++)
			{
				if(Results.motion_indicator.motion[motion_config.map_id[i]] >= 44)
				{
					printf(" Movement detected in this zone : %3d !\n", i);
				}
			}
			printf("\n");
			loop++;
		}

	}

	status = vl53lmz_stop_ranging(&Dev);
	printf("End of ULD demo\n");
	return status;
}

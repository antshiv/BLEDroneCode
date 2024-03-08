/*******************************************************************************
 * Copyright (c) 2020, STMicroelectronics - All Rights Reserved
 *
 * This file is part of the VL53L8CX Ultra Lite Driver and is dual licensed,
 * either 'STMicroelectronics Proprietary license'
 * or 'BSD 3-clause "New" or "Revised" License' , at your option.
 *
 ********************************************************************************
 *
 * 'STMicroelectronics Proprietary license'
 *
 ********************************************************************************
 *
 * License terms: STMicroelectronics Proprietary in accordance with licensing
 * terms at www.st.com/sla0081
 *
 * STMicroelectronics confidential
 * Reproduction and Communication of this document is strictly prohibited unless
 * specifically authorized in writing by STMicroelectronics.
 *
 *
 ********************************************************************************
 *
 * Alternatively, the VL53L8CX Ultra Lite Driver may be distributed under the
 * terms of 'BSD 3-clause "New" or "Revised" License', in which case the
 * following provisions apply instead of the ones mentioned above :
 *
 ********************************************************************************
 *
 * License terms: BSD 3-clause "New" or "Revised" License.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 * list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software
 * without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *
 *******************************************************************************/

/***********************************/
/*   VL53L8CX ULD basic example    */
/***********************************/
/*
 * This example is the most basic. It initializes the VL53L8CX ULD, and starts
 * a ranging to capture 10 frames.
 *
 * By default, ULD is configured to have the following settings :
 * - Resolution 4x4
 * - Ranging period 1Hz
 *
 * In this example, we also suppose that the number of target per zone is
 * set to 1 , and all output are enabled (see file platform.h).
 */

#include <stdlib.h>
#include <string.h>
#include <stdio.h>
// #include "vl53l8cx_api.h"
#include "VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/inc/vl53l8cx_api.h"
#include "VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/inc/vl53l8cx_plugin_motion_indicator.h"
#include "VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/inc/vl53l8cx_plugin_detection_thresholds.h"
static uint8_t init_complete = false;

int example1(const struct i2c_dt_spec *i2cSpec)
{

	/*********************************/
	/*   VL53L8CX ranging variables  */
	/*********************************/

	uint8_t status, loop, isAlive, isReady, i;
	VL53L8CX_Configuration Dev;	  /* Sensor configuration */
	VL53L8CX_ResultsData Results; /* Results data from VL53L8CX */
	uint8_t resolution;

	/*********************************/
	/*      Customer platform        */
	/*********************************/

	/* Fill the platform structure with customer's implementation. For this
	 * example, only the I2C address is used.
	 */
	Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
	Dev.platform.i2cSpec = i2cSpec;
	vl53l5cx_test_i2c(&Dev);
	return;

	/* (Optional) Reset sensor toggling PINs (see platform, not in API) */
	// Reset_Sensor(&(Dev.platform));

	/* (Optional) Set a new I2C address if the wanted address is different
	 * from the default one (filled with 0x20 for this example).
	 */
	// status = vl53l8cx_set_i2c_address(&Dev, 0x20);

	/*********************************/
	/*   Power on sensor and init    */
	/*********************************/

	/* (Optional) Check if there is a VL53L8CX sensor connected */
	if (!init_complete)
	{
		status = vl53l8cx_is_alive(&Dev, &isAlive);
		if (!isAlive || status)
		{
			printf("VL53L8CX not detected at requested address\n");
			return status;
		}

		/* (Mandatory) Init VL53L8CX sensor */
		status = vl53l8cx_init(&Dev);
		if (status)
		{
			printf("VL53L8CX ULD Loading failed\n");
			return status;
		}
		init_complete = true;
		printf("VL53L8CX ULD ready ! (Version : %s)\n",
			   VL53L8CX_API_REVISION);

		printk("setting power mode to wakeup\n");
		// status = vl53l8cx_set_power_mode(&Dev, VL53L8CX_POWER_MODE_WAKEUP); // Set mode standby
		// if (status)
		//	printf("Error in setting power mode\n");

		/*********************************/
		/*         Ranging loop          */
		/*********************************/

		status = vl53l8cx_set_ranging_frequency_hz(&Dev, 2); // Set 2Hz ranging frequency
		if (status)
			printf("Error in setting frequency\n");
		status = vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_CONTINUOUS); // Set mode continuous
		if (status)
			printf("Error in setting mode\n");
	}
	uint8_t p_power_mode;
	status = vl53l8cx_get_power_mode(
		&Dev,
		&p_power_mode);
	if (status)
		printf("Error in getting power mode\n");
	printf("Power mode is %d\n", p_power_mode);
	printf("Ranging starts\n", status);
	status = vl53l8cx_start_ranging(&Dev);
	printf("Data read size %d \n", Dev.data_read_size);
	status = vl53l8cx_get_resolution(&Dev, &resolution);
	if (status)
		printf("Error in getting resolution\n");

	loop = 0;
	while (loop < 200)
	{
		/* Use polling function to know when a new measurement is ready.
		 * Another way can be to wait for HW interrupt raised on PIN A1
		 * (INT) when a new measurement is ready */

		status = vl53l8cx_check_data_ready(&Dev, &isReady);

		if (isReady)
		{
			printf("Data read size %d \n", Dev.data_read_size);
			memset(&Results, 0, sizeof(Results));
			status = vl53l8cx_get_ranging_data(&Dev, &Results);
			if (status)
			{
				printf("Error in getting ranging data status is %d\n", status);
				continue;
			}
			printf("Status is : %u and resolution is %d\n", status, resolution);

			/* As the sensor is set in 4x4 mode by default, we have a total
			 * of 16 zones to print. For this example, only the data of first zone are
			 * print */
			printf("Print data no : %3u\n", Dev.streamcount);
			for (i = 0; i < resolution; i++)
			{
				printf("Zone : %3d, Status : %3u, Distance : %4d mm, Nb targets : %2u, Ambient : %4lu Kcps/spads\n",
					   i,
					   Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
					   Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i],
					   Results.nb_target_detected[i],
					   Results.ambient_per_spad[i]);
			}
			printf("\n");
			loop++;
		}

		/* Wait a few ms to avoid too high polling (function in platform
		 * file, not in API) */
		WaitMs(&(Dev.platform), 2000);

	}

	status = vl53l8cx_stop_ranging(&Dev);
	printf("End of ULD demo status %d\n", status);
	return status;
}

/*
 * Running in continous mode
 */
void continous_mode(const struct i2c_dt_spec *i2cSpec)
{
	/* USER CODE BEGIN PV */
	int status = 0;
	volatile int IntCount;
	uint8_t p_data_ready;
	VL53L8CX_Configuration Dev;
	VL53L8CX_ResultsData Results;
	uint8_t resolution, isAlive;
	uint16_t idx;
	Dev.platform.address = VL53L8CX_DEFAULT_I2C_ADDRESS;
	Dev.platform.i2cSpec = i2cSpec;

	// Reset_Sensor(&(Dev.platform));

	status = vl53l8cx_is_alive(&Dev, &isAlive);
	if (!isAlive || status)
	{
		printf("VL53L8CX not detected at requested address\n");
		return status;
	}
	// printf("Sensor initializing, please wait few seconds\n");
	/* (Mandatory) Init VL53L8CX sensor */
	status = vl53l8cx_init(&Dev);
	if (status)
	{
		printf("VL53L8CX ULD Loading failed\n");
		return status;
	}
	status |= vl53l8cx_set_ranging_frequency_hz(&Dev, 15);						 // Set 2Hz ranging frequency
	status |= vl53l8cx_set_ranging_mode(&Dev, VL53L8CX_RANGING_MODE_CONTINUOUS); // Set mode continuous
	printf("Status is %d Ranging starts\n", status);
	status = vl53l8cx_start_ranging(&Dev);
	get_data_by_polling(&Dev);
}

void get_data_by_polling(VL53L8CX_Configuration *p_dev)
{
	uint8_t p_data_ready;
	int status;
	uint8_t resolution, isAlive;
	VL53L8CX_ResultsData Results;
	do
	{
		status = vl53l8cx_check_data_ready(p_dev, &p_data_ready);
		if (p_data_ready)
		{
			status = vl53l8cx_get_resolution(p_dev, &resolution);
			status = vl53l8cx_get_ranging_data(p_dev, &Results);

			for (int i = 0; i < resolution; i++)
			{
				/* Print per zone results */
				printf("Zone : %2d, Nb targets : %2u, Ambient : %4lu Kcps/spads, ",
					   i,
					   Results.nb_target_detected[i],
					   Results.ambient_per_spad[i]);

				/* Print per target results */
				if (Results.nb_target_detected[i] > 0)
				{
					printf("Target status : %3u, Distance : %4d mm\n",
						   Results.target_status[VL53L8CX_NB_TARGET_PER_ZONE * i],
						   Results.distance_mm[VL53L8CX_NB_TARGET_PER_ZONE * i]);
				}
				else
				{
					printf("Target status : 255, Distance : No target\n");
				}
			}
			printf("\n");
		}
		else
		{
			k_msleep(2500);
		}
		k_msleep(2500);
	} while (1);
}
/**
 *
 * Copyright (c) 2021 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "platform.h"
extern const struct i2c_dt_spec vl53l8_1;

uint8_t RdByte(
	VL53L8CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_value)
{
	uint8_t status = 255;
	uint8_t txBuf[2] = {0};	
	txBuf[1] = RegisterAdress & 0xFF;
	txBuf[0] = RegisterAdress >> 8;

	/* Need to be implemented by customer. This function returns 0 if OK */
	if(p_platform->i2c_spi == 0)
		status = mcu_i2cTransfer(p_platform->i2cSpec, txBuf, 2, p_value, 1);
	else {
		uint8_t rx[3];
		status = spi_write_msg(p_platform->spi_tof_cfg, 3, txBuf, rx);
		spi_release(p_platform->spi_dev, p_platform->spi_tof_cfg);
		*p_value = rx[2];
	}

	/* Need to be implemented by customer. This function returns 0 if OK */

	return status;
}

uint8_t WrByte(
	VL53L8CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t value)
{
	uint8_t status = 255;
    uint8_t txBuf[3] = {0}; //All writable registers are 2 bytes
	txBuf[1] = RegisterAdress & 0xFF;
	txBuf[0] = RegisterAdress >> 8;
	txBuf[2] = value;
	/* Need to be implemented by customer. This function returns 0 if OK */
	if(p_platform->i2c_spi == 0)
		status = mcu_i2cTransfer(p_platform->i2cSpec, txBuf, 3, NULL, 0);
	else {
		txBuf[0] |= 0b10000000;
		status = spi_write_msg(p_platform->spi_tof_cfg, 3, txBuf, NULL);
		spi_release(p_platform->spi_dev, p_platform->spi_tof_cfg);
	}

	/* Need to be implemented by customer. This function returns 0 if OK */

	return status;
}

uint8_t WrMulti(
	VL53L8CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_values,
	uint32_t size)
{
	uint8_t status = 255;
	uint8_t *txBuf = k_malloc(size + 2);
	if (txBuf == NULL)
	{
		printf("Failed to allocate memory for txBuf of size %d\n", size + 2);
		return 1;
	}
	txBuf[1] = RegisterAdress & 0xFF;
	txBuf[0] = RegisterAdress >> 8;
	bytecpy(&txBuf[2], p_values, size);
	/* Need to be implemented by customer. This function returns 0 if OK */
	if(p_platform->i2c_spi == 0)
		status = mcu_i2cTransfer(p_platform->i2cSpec, txBuf, size + 2, NULL, 0);
	else {
		txBuf[0] |= 0b10000000;
		status = spi_write_msg(p_platform->spi_tof_cfg, size + 2, txBuf, NULL);
		spi_release(p_platform->spi_dev, p_platform->spi_tof_cfg);
	}
	k_free(txBuf);

	/* Need to be implemented by customer. This function returns 0 if OK */

	return status;
}

uint8_t RdMulti(
	VL53L8CX_Platform *p_platform,
	uint16_t RegisterAdress,
	uint8_t *p_values,
	uint32_t size)
{
	uint8_t status = 255;
	uint8_t txBuf[2] = {0};
	txBuf[1] = RegisterAdress & 0xFF;
	txBuf[0] = RegisterAdress >> 8;
	if(p_platform->i2c_spi == 0)
		status = mcu_i2cTransfer(p_platform->i2cSpec, txBuf, 2, p_values, size);
	else {
		uint8_t *rx_buf = k_malloc(size + 2);
		status = spi_write_msg(p_platform->spi_tof_cfg, size + 2, txBuf, rx_buf);
		spi_release(p_platform->spi_dev, p_platform->spi_tof_cfg);
		memcpy(p_values, rx_buf + 2, size);	
		k_free(rx_buf);
	}

	/* Need to be implemented by customer. This function returns 0 if OK */

	return status;
}

uint8_t Reset_Sensor(
	VL53L8CX_Platform *p_platform)
{
	uint8_t status = 0;

	/* (Optional) Need to be implemented by customer. This function returns 0 if OK */

	/* Set pin LPN to LOW */
	/* Set pin AVDD to LOW */
	/* Set pin VDDIO  to LOW */
	/* Set pin CORE_1V8 to LOW */
	WaitMs(p_platform, 100);

	/* Set pin LPN to HIGH */
	/* Set pin AVDD to HIGH */
	/* Set pin VDDIO to HIGH */
	/* Set pin CORE_1V8 to HIGH */
	WaitMs(p_platform, 100);

	return status;
}

void SwapBuffer(
	uint8_t *buffer,
	uint16_t size)
{
	uint32_t i, tmp;

	/* Example of possible implementation using <string.h> */
	for (i = 0; i < size; i = i + 4)
	{
		tmp = (buffer[i] << 24) | (buffer[i + 1] << 16) | (buffer[i + 2] << 8) | (buffer[i + 3]);

		memcpy(&(buffer[i]), &tmp, 4);
	}
}

uint8_t WaitMs(
	VL53L8CX_Platform *p_platform,
	uint32_t TimeMs)
{
	uint8_t status = 255;

	/* Need to be implemented by customer. This function returns 0 if OK */
	status = k_msleep(TimeMs);

	return status;
}

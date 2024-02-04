/*
 *  Include Generic Header Files Here
 */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "mcu.h"

/*
 *  Include MCU Specific Header Files Here
 */

/*
 *  Include MCU Specific Header Files Here
 */

/********* MCU SPECIFIC I2C CODE STARTS HERE **********/

void mcu_i2cInit(uint8_t busId)
{
    /* Add MCU specific init necessary for I2C to be used */
}

// int8_t mcu_i2cTransfer( uint8_t busId, uint8_t i2cAddr,
int8_t mcu_i2cTransfer(const struct i2c_dt_spec *i2cSpec,
                       uint8_t *dataToWrite, uint8_t writeLength,
                       uint8_t *dataToRead, uint8_t readLength)
{
    /*
     *  Add MCU specific I2C read/write code here.
     */
    // uint8_t config[2] = {0x03,0x8C};
    int ret;
    ret = i2c_write_read_dt(i2cSpec, dataToWrite, writeLength, dataToRead, readLength);

    /*
     *  Add MCU specific return code for error handling
     */
    if (ret != 0)
    {
        printk("Failed to write/read I2C device address %x at Reg. %x \n\r", i2cSpec->addr, dataToWrite[0]);
    }

    return (0);
}
/********* MCU SPECIFIC I2C CODE ENDS HERE **********/

/********* MCU SPECIFIC DELAY CODE STARTS HERE ************/
void mcu_msWait(uint16_t msWait)
{
    /*
     *  Add MCU specific wait loop for msWait. The unit is in milli-seconds
     */
}

void mcu_usWait(uint16_t usWait)
{
    /*
     *  Add MCU specific wait loop for usWait. The unit is in micro-seconds
     */
}
/********* MCU SPECIFIC DELAY CODE ENDS HERE ************/

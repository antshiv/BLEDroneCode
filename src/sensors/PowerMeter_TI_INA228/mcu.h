/*
 *  ======== mcu.h ========
 *  MCU hardware abstraction used sensor API implementations
 */
#ifndef ti_sensors_MCU__include
#define ti_sensors_MCU__include 1

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../../includes.h"

/* support C++ sources */
#ifdef __cplusplus
extern "C" {
#endif


/*
 *  ======== mcu_i2cInit ========
 *  Initialize the specified I2C bus for first use
 */
extern void mcu_i2cInit(uint8_t busId);

/*
 *  ======== mcu_i2cTransfer ========
 *  Transfer data to and from an I2C slave
 *
 *  If writeLength is non-zero, mcu_i2cTransfer always performs the write
 *  transfer first.
 *
 *  @param busId         id of an I2C bus to access for the transfer
 *  @param sensorAddress I2C address of peripheral to access
 *  @param dataToWrite   non-NULL pointer to a buffer of at least writeLength
 *                       bytes; may be NULL if writeLength = 0.
 *  @param writeLength   number of bytes to write from the dataToWrite array
 *  @param dataToRead    non-NULL pointer to a buffer of at least readLength
 *                       bytes; may be NULL if readLength = 0.
 *  @param readLength    number of bytes to read into dataToRead array
 *
 *  @return              0 if successful, otherwise non-zero
 */
extern int8_t mcu_i2cTransfer( const struct i2c_dt_spec *i2cSpec,
                              uint8_t *dataToWrite, uint8_t writeLength,
                              uint8_t *dataToRead,  uint8_t readLength);

/*
 *  ======== mcu_msWait ========
 *  Delay CPU for at least the specified number of milliseconds
 *
 *  @param msWait - number of milliseconds to delay, a value of 0 causes
 *                  this function to return immediately.
 */
extern void mcu_msWait(uint16_t msWait);

/*
 *  ======== mcu_usWait ========
 *  Delay CPU for at least the specified number of microseconds
 *
 *  @param msWait - number of microseconds to delay, a value of 0 causes
 *                  this function to return immediately.
 */
extern void mcu_usWait(uint16_t usWait);

/* support C++ sources */
#ifdef __cplusplus
}
#endif
#endif /* ti_sensors_MCU__include */


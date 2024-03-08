/*
 *  ======== INA228.c ========
 *  INA228 APIs for initialization and use of the INA228 peripheral
 *
 *  DO NOT EDIT - This file is generated by the SysConfig tool for
 *  the TI Sensors in this application.
 */

#include <stddef.h>
#include <stdint.h>

#include "INA228.h"
#include "../../drivers/TWIM/mcu.h"
#include "config.h" /* for sensor handle names */

#define MSB(u16) (((u16) & 0xFF00U) >> 8)
#define LSB(u16) ((u16) & 0xFFU)

#define maxRegAddress 0x3F

const struct i2c_dt_spec ina228 = I2C_DT_SPEC_GET(INA228_NODE);

// Register size in bytes
const uint8_t INA228_regSize[maxRegAddress+1] = {
                                            2,2,2,2,3,3,2,3,\
                                            3,5,5,2,2,2,2,2,\
                                            2,2,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,0,0,\
                                            0,0,0,0,0,0,2,2
};

/*
 *  ======== INA228_writeReg ========
 * Write register
 */
void INA228_writeReg(INA228_Handle sensor, uint8_t regAddr, uint16_t value)
{
    uint8_t txBuf[3] = {0}; //All writable registers are 2 bytes

    txBuf[0] = regAddr;
    txBuf[1] = MSB(value);
    txBuf[2] = LSB(value);
    mcu_i2cTransfer(&ina228, txBuf, 3, NULL, 0);

    //check for change in ADCRANGE 
    if(regAddr == INA228_config_register)
    {
        sensor->adcrange = value & INA228_config_register_adcrange_4096mV;
    }

}

/*
 *  ======== INA228_config ========
 * Configure device with current settings.
 */
void INA228_config(INA228_Handle sensor)
{
    //Initialize the bus containing this sensor
    mcu_i2cInit(sensor->busId);

    //Write sensor Configuration Registers
    INA228_writeReg(sensor, INA228_config_register, sensor->configRegister);
    INA228_writeReg(sensor, INA228_adc_config_register, sensor->adcConfigRegister);
    INA228_writeReg(sensor, INA228_shunt_cal_register, sensor->shuntCalRegister);
    INA228_writeReg(sensor, INA228_shunt_tempco_register, sensor->shuntTempcoRegister);
    INA228_writeReg(sensor, INA228_diag_alrt_register, sensor->diagAlrtRegister);
    INA228_writeReg(sensor, INA228_sovl_register, sensor->sovlRegister);
    INA228_writeReg(sensor, INA228_suvl_register, sensor->suvlRegister);
    INA228_writeReg(sensor, INA228_bovl_register, sensor->bovlRegister);
    INA228_writeReg(sensor, INA228_buvl_register, sensor->buvlRegister);
    INA228_writeReg(sensor, INA228_temp_limit_register, sensor->tempLimitRegister);
    INA228_writeReg(sensor, INA228_pwr_limit_register, sensor->pwrLimitRegister);

}

/*
 *  ======== INA228_setCURRENT_LSB ========
 *  Set the CURRENT_LSB value used for calculations
 */
void INA228_setCURRENT_LSB(INA228_Handle sensor, float CURRENT_LSB)
{
    sensor->currentlsb = CURRENT_LSB;
}

/*
 *  ======== INA228_readReg ========
 *  Read register
 */
uint64_t INA228_readReg(INA228_Handle sensor, uint8_t regAddr)
{
    uint64_t value;
    int i;
    
    uint8_t txBuf[1] = {0};
    uint8_t rxBuf[5] = {0}; //max buffer size

    txBuf[0] = regAddr;

    //Read register
    mcu_i2cTransfer(&ina228, txBuf, 1, rxBuf, INA228_regSize[regAddr]);

    //Combine bytes
    value = rxBuf[0];
    for(i = 1; i < INA228_regSize[regAddr]; i++)
    {
        value = (value << 8) | rxBuf[i];
    }

    return value;
}

/*
 *  ======== INA228_getVSHUNT_mV ========
 *  Get VSHUNT value (mV)
 */
float INA228_getVSHUNT_mV(INA228_Handle sensor)
{
    uint64_t value = INA228_readReg(sensor, INA228_vshunt_register);
    float data;

    //Remove reserved bits
    value = value >> 4;

    //Convert for 2's compliment and signed value
    if(value > 0x7FFFF)
    {
        data = (float)value - 0x100000;
    }
    else
    {
        data = (float)value;
    }

    //Convert to mV

    if(sensor->adcrange == INA228_config_register_adcrange_4096mV)
    {
        data = (data * 78.125) / 1000000;
    }
    else
    {
        data = (data * 312.5) / 1000000;
    }

    return data;
}

/*
 *  ======== INA228_getVBUS_V ========
 *  Get VBUS value (V)
 */
float INA228_getVBUS_V(INA228_Handle sensor)
{
    uint64_t value = INA228_readReg(sensor, INA228_vbus_register);
    float data;

    //Remove reserved bits
    value = value >> 4;

    //Convert for 2's compliment and signed value (though always positive)
    if(value > 0x7FFFF)
    {
        data = (float)value - 0x100000; //left for redundancy and error checking, should never get used
    }
    else
    {
        data = (float)value;
    }

    //Convert to V
    data = (data * 195.3125) / 1000000;

    return data;
}

/*
 *  ======== INA228_getDIETEMP_C ========
 *  Get DIETMEP value (C)
 */
float INA228_getDIETEMP_C(INA228_Handle sensor)
{
    uint64_t value = INA228_readReg(sensor, INA228_dietemp_register);
    float data;

    //Convert for 2's compliment and signed value
    if(value > 0x7FFF)
    {
        data = (float)value - 0x10000; 
    }
    else
    {
        data = (float)value;
    }

    //Convert to C
    data = (data * 7.8125) / 1000;

    return data;
}

/*
 *  ======== INA228_getDIETEMP_F ========
 *  Get DIETMEP value (F)
 */
float INA228_getDIETEMP_F(INA228_Handle sensor)
{
    float data = INA228_getDIETEMP_C(sensor);
    
    //Convert to F
    data = (data * (9/5)) + 32;

    return data;
}

/*
 *  ======== INA228_getCURRENT_signedLSB ========
 *  Get CURRENT value (signed value in LSBs)
 */
float INA228_getCURRENT_signedLSB(INA228_Handle sensor)
{
    uint64_t value = INA228_readReg(sensor, INA228_current_register);
    float data;

    //Remove reserved bits
    value = value >> 4;

    //Convert for 2's compliment and signed value 
    if(value > 0x7FFFF)
    {
        data = (float)value - 0x100000;
    }
    else
    {
        data = (float)value;
    }

    return data;
}

/*
 *  ======== INA228_getCURRENT_A ========
 *  Get CURRENT value (A)
 */
float INA228_getCURRENT_A(INA228_Handle sensor)
{
    float data = INA228_getCURRENT_signedLSB(sensor);

    data = data * sensor->currentlsb;

    return data;
}

/*
 *  ======== INA228_getPOWER_signedLSB ========
 *  Get POWER value (signed value in LSBs)
 */
float INA228_getPOWER_signedLSB(INA228_Handle sensor)
{
    uint64_t value = INA228_readReg(sensor, INA228_power_register);
    float data;

    data = (float)value;

    return data;
}

/*
 *  ======== INA228_getPOWER_W ========
 *  Get POWER value (W)
 */
float INA228_getPOWER_W(INA228_Handle sensor)
{
    float data = INA228_getPOWER_signedLSB(sensor);

    data = data * sensor->currentlsb * 3.2;

    return data;
}

/*
 *  ======== INA228_getENERGY_signedLSB ========
 *  Get ENERGY value (signed value in LSBs)
 */
double INA228_getENERGY_signedLSB(INA228_Handle sensor)
{
    uint64_t value = INA228_readReg(sensor, INA228_energy_register);
    double data;

    data = (double)value;

    return data;
}

/*
 *  ======== INA228_getENERGY_J ========
 *  Get ENERGY value (J)
 */
double INA228_getENERGY_J(INA228_Handle sensor)
{
    double data = INA228_getENERGY_signedLSB(sensor);

    data = data * sensor->currentlsb * 51.2;

    return data;
}

/*
 *  ======== INA228_getCHARGE_signedLSB ========
 *  Get CHARGE value (signed value in LSBs)
 */
double INA228_getCHARGE_signedLSB(INA228_Handle sensor)
{
    uint64_t value = INA228_readReg(sensor, INA228_charge_register);
    double data;

    //Convert for 2's compliment and signed value 
    if(value > 0x7FFFFFFFFF)
    {
        data = (double)value - 0x10000000000;
    }
    else
    {
        data = (double)value;
    }

    return data;
}

/*
 *  ======== INA228_getCHARGE_C ========
 *  Get CHARGE value (C)
 */
double INA228_getCHARGE_C(INA228_Handle sensor)
{
    double data = INA228_getCHARGE_signedLSB(sensor);

    data = data * sensor->currentlsb;

    return data;
}

void power_monitor_thread() {
    int rc;
    float v_bus, power, current, die_temp, shunt_v, energy, charge;
    INA228_config(INA228);
    for (;;) { 
        die_temp = INA228_getDIETEMP_C(INA228);
        v_bus = INA228_getVBUS_V(INA228);
        current = INA228_getCURRENT_A(INA228);
        power = INA228_getPOWER_W(INA228);
        shunt_v = INA228_getVSHUNT_mV(INA228);
        energy = INA228_getENERGY_J(INA228);
        charge = INA228_getCHARGE_C(INA228);

        printf("Die Temp: %f [C] -- "
            "Bus: %f [V] -- "
            "Power: %f [W] -- "
            "Current: %f [A]\n",
               die_temp,
               v_bus,
               power,
               current);
        printf("Shunt Voltage: %f [mV] -- "
            "Energy: %f [J] -- "
            "Charge: %f [C]\n",
               shunt_v,
               energy,
               charge);       
        k_yield();
        k_sleep(K_MSEC(1000)); 
    }
}

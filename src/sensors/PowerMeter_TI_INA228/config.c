/*
 *  ======== config.c ========
 *  Configuration-specific definitions
 *
 *  DO NOT EDIT - This file is generated by the SysConfig tool for
 *  the TI Sensors in this application.
 */
#include <stdint.h>

#include "INA228.h"

#include "config.h"

/*
 *  ======== INA228 ========
 *  INA228 Sensor configuration settings
 */

static INA228_State INA228_state = {

    /* Configuration and Settings */
    .configRegister = (INA228_config_register_rst_NormalOperation | \
                       INA228_config_register_rstacc_NormalOperation | \
                       0x0000U | \
                       INA228_config_register_tempcomp_Shunttemperaturecompensationdisabled | \
                       INA228_config_register_adcrange_16384mV),
    .adcConfigRegister = (INA228_adc_config_register_mode_Continuousbusvoltageshuntvoltageandtemperature | \
                          INA228_adc_config_register_vbusct_1052us | \
                          INA228_adc_config_register_vshct_1052us | \
                          INA228_adc_config_register_vtct_1052us | \
                          INA228_adc_config_register_avg_1),
    .shuntCalRegister = 0x1000U,
    .shuntTempcoRegister = 0x0000U, /* TEMPCO is 0 ppm/°C */
    .diagAlrtRegister = (INA228_diag_alrt_register_alatch_Transparent | \
                         INA228_diag_alrt_register_cnvr_DisableconversionreadyflagonALERTpin | \
                         INA228_diag_alrt_register_slowalert_ALERTcomparisononnonaveragedADCvalue | \
                         INA228_diag_alrt_register_apol_Normalactivelowopendrain),
    .sovlRegister = 0x7FFFU,
    .suvlRegister = 0x8000U,
    .bovlRegister = 0x7FFFU,
    .buvlRegister = 0x0000U,
    .tempLimitRegister = 0x7FFFU,
    .pwrLimitRegister = 0xFFFFU,

    .adcrange = INA228_config_register_adcrange_16384mV, 
    .currentlsb = 0.00001,

    /* Sensor's I2C bus ID and address */
    .busId = 0,
    .devAddr = 0x45U,

};
const INA228_Handle INA228 = &INA228_state;



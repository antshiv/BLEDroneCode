#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
)
target_sources(app PRIVATE src/drivers/PWM/pwm_local.c)
target_sources(app PRIVATE src/drivers/SPIM/spim_local.c)
target_sources(app PRIVATE src/drivers/TWIM/twim_local.c)
target_sources(app PRIVATE src/drivers/TWIM/mcu.c)
target_sources(app PRIVATE src/drivers/GPIO_INPUT/gpio_input_local.c)

# Include sensors 
#target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/ceva_fsm300.c)
target_sources(app PRIVATE src/commands.c)
target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/ceva_fsmSH2.c)
#target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/cal_app.c)
#target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/demo_app.c)
# Include SH2
#target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/sh2/shtp.c)
#target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/sh2/sh2.c)
target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/sh2/sh2_util.c)
target_sources(app PRIVATE src/sensors/IMU_CEVA_FSM300/sh2/sh2_SensorValue.c)

#Include Power Meter
target_sources(app PRIVATE src/sensors/PowerMeter_TI_INA228/INA228.c)
target_sources(app PRIVATE src/sensors/PowerMeter_TI_INA228/config.c)

#Include Absolute Barometric Pressure Sensor
target_sources(app PRIVATE src/sensors/AbsoluteBarometricPressure_BMP585/common.c)
target_sources(app PRIVATE src/sensors/AbsoluteBarometricPressure_BMP585/BMP5-Sensor-API/bmp5.c)
target_sources(app PRIVATE src/sensors/AbsoluteBarometricPressure_BMP585/read_sensor_data_normal_mode.c)

#Include TOF sensor
target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/tof_vl53l8.c)
target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53LMZ_ULD_v2.0.10/Platform/platform.c)

#target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/src/vl53lmz_api.c)
#target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/src/vl53lmz_plugin_cnh.c)
#target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/src/vl53lmz_plugin_detection_thresholds.c)
#target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/src/vl53lmz_plugin_motion_indicator.c)
#target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/src/vl53lmz_plugin_xtalk.c)
#target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/Example_11_motion_indicator_with_detection_thresholds.c)

target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/src/vl53l8cx_api.c)
target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/src/vl53l8cx_plugin_detection_thresholds.c)
target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/src/vl53l8cx_plugin_motion_indicator.c)
target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/src/vl53l8cx_plugin_xtalk.c)
target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/Example_11_motion_indicator_with_detection_thresholds_VL53L8CX.c)
target_sources(app PRIVATE src/sensors/TOF_ST_VL53L8/Example_1_ranging_basic.c)

# Include UART ASYNC API adapter
target_sources_ifdef(CONFIG_BT_NUS_UART_ASYNC_ADAPTER app PRIVATE
  src/uart_async_adapter.c
)

# NORDIC SDK APP END

zephyr_library_include_directories(.)

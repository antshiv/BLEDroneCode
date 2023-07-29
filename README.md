# Drone code Using NordicSemi NRF5340 with Bluetooth BLE

-------------------------------------------------------

This repository contains the code for the drone. The code runs on the zephyr RTOS and uses the NordicSemi Peripheral UART as the base code base.

the IMU sensor used is the CEVA FSM300. Currently this is the only external Peripheral used.

The connection diagram is as follows:
![Connection Diagram](https://github.com/antshiv/BLEDroneCode/blob/master/assets/connection_diagram.jpg)

# Connecting the E.S.C - Electronic Speed Controller
The E.S.C is connected to the NRF5340 using the PWM pins. The PWM pins are connected to the NRF5340 using the following pins:
```
PWM0 - P0.06
PWM1 - P0.07
PWM2 - P0.08
PWM3 - P0.09
```
![ESC Connection Diagram](https://github.com/antshiv/BLEDroneCode/blob/master/assets/ESC_connection.jpg)

The key connection is to make sure the grounds are common. You want the E.S.C and the NRF5340 to have the same ground. This is because the PWM signal is a differential signal and the ground is used as a reference. If the grounds are not common then the PWM signal will not be read correctly.

The timing settings for the PWM signal is as follows:
```
PWM period: 20ms
PWM pulse width: 1ms - 2ms
```

![ESC Connection Diagram](https://github.com/antshiv/BLEDroneCode/blob/master/assets/ESC_timing_sequence.jpg)

The PWM signal is a 20ms signal with a pulse width of 1ms to 2ms. The pulse width is used to control the speed of the motor. The pulse width is controlled using the PWM duty cycle. The duty cycle is a percentage of the pulse width. For example a duty cycle of 50% will give a pulse width of 1.5ms. This will give a medium speed to the motor. A duty cycle of 100% will give a pulse width of 2ms. This will give the maximum speed to the motor. A duty cycle of 0% will give a pulse width of 1ms. This will give the minimum speed to the motor.

# Connecting the IMU - Inertial Measurement Unit
The IMU is connected to the NRF5340 using the SPI pins. The SPI pins are connected to the NRF5340 using the following pins:
```
SPI0 SCK - P0.06
SPI0 MOSI - P0.07
SPI0 MISO - P0.25
SPI0 CSN - P0.26
```

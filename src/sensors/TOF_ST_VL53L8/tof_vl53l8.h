//#include "VL53LMZ_ULD_v2.0.10/VL53LMZ_ULD_API/inc/vl53lmz_api.h"
#include "VL53L8CX_ULD_driver_1.2.1/VL53L8CX_ULD_API/inc/vl53l8cx_api.h"
#include "../../includes.h"
#include "../../drivers/GPIO_INPUT/gpio_input_local.h"
#include "../../drivers/SPIM/spim_local.h"

#define GPIO_FOR_LPn_PIN 1
#define GPIO_FOR_PWR_EN 1
extern const struct gpio_dt_spec spi_i2c_n;
extern const struct gpio_dt_spec LPn;
extern const struct gpio_dt_spec pwren;
#define VL53L8_1_NODE DT_NODELABEL(vl53l8_1)
extern const struct i2c_dt_spec vl53l8_1;

extern int example11(const struct i2c_dt_spec *i2cSpec);
extern int example1(const struct i2c_dt_spec *i2cSpec);
void continous_mode(const struct i2c_dt_spec *i2cSpec);
void tof_vl5318_thread(void);

int WaitForL5Interrupt(VL53L8CX_Configuration * pDev);
uint8_t vl53l5cx_test_i2c(VL53L8CX_Configuration *p_dev);
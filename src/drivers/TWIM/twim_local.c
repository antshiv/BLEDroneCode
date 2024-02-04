#include "twim_local.h"

const struct i2c_dt_spec dev_i2c1 = I2C_DT_SPEC_GET(INA228_NODE);

void twim_init(void)
{
    int ret;
    
    if (!device_is_ready(dev_i2c1.bus))
    {
        printk("TWI master device not ready!\n");
    } else {
        printk("TWI master device ready!\n");
    }
}

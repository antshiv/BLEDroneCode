#include "../../includes.h"

#define INA228_NODE DT_NODELABEL(ina228)
extern const struct i2c_dt_spec dev_i2c1;
void i2c_init(void);
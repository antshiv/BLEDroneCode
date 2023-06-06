#include "ceva_fsm300.h"


int interrupt_flag = 0;

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins)
{
    int err = 0;
    interrupt_flag = 1;
	//printk("FSM has an interrup triggered at %" PRIu32 "\n", k_cycle_get_32());
}

void FSM_thread(void) {
    int err = 0;
    for (;;) {
        if (interrupt_flag == 1) {
            //printk("FSM thread running %" PRIu32 "\n", k_cycle_get_32());
            /* Interfacting with sensor using SPI */
            err = gpio_pin_toggle_dt(&spi4_cs);
            if (err < 0)
            {
                return;
            } 
            //k_msleep(SLEEP_TIME_MS);
            spi_write_test_msg();
            err = gpio_pin_toggle_dt(&spi4_cs);
            /* end sending info using SPI */	
            interrupt_flag = 0;
            k_yield();
        }
    }
}
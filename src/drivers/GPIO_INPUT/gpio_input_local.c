#include "../../includes.h"
#include "gpio_input_local.h"


void button_pressed(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins)
{
	printk("Button pressed at %" PRIu32 "\n", k_cycle_get_32());
}

void disableInts() {
    printk("disable interrupts \n");
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_DISABLE);
}

void enableInts() {
    printk("enabling interrupts \n");
	gpio_pin_interrupt_configure_dt(&button, GPIO_INT_EDGE_TO_ACTIVE);
}
#include "../../includes.h"
#include "gpio_input_local.h"


const struct gpio_dt_spec button = GPIO_DT_SPEC_GET_OR(SW0_NODE, gpios,
															  {0});
/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
struct gpio_dt_spec led = GPIO_DT_SPEC_GET_OR(DT_ALIAS(led2), gpios,
													 {0});

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
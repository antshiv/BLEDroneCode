#include "../../includes.h"

/*** Initilize GPIO input */
/*
 * Get button configuration from the devicetree sw0 alias. This is mandatory.
 */
#define SW0_NODE DT_ALIAS(sw0)
#if !DT_NODE_HAS_STATUS(SW0_NODE, okay)
#error "Unsupported board: sw0 devicetree alias is not defined"
#endif

extern const struct gpio_dt_spec button;
static struct gpio_callback button_cb_data;

/*
 * The led0 devicetree alias is optional. If present, we'll use it
 * to turn on the LED whenever the button is pressed.
 */
extern struct gpio_dt_spec led;


void button_pressed(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins);

void disableInts();

void enableInts();

 #define GPIO_PIN_RESET 0
 #define GPIO_PIN_SET 1
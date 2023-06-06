#include "../../includes.h"
#include "../../drivers/SPIM/spim_local.h"

#define SLEEP_TIME_MS 1

extern int interrupt_flag;

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins);


void FSM_thread(void);
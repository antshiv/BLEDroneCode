#include "../../includes.h"
#include "../../drivers/SPIM/spim_local.h"
#include "sh2/sh2_hal.h"

#define SLEEP_TIME_MS 1

extern int interrupt_flag;

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins);


extern const struct gpio_dt_spec fsmrrstn;
extern const struct gpio_dt_spec fsmbootn;
void FSM_thread(void);
void FSM_process_data();
void FSM_init();
sh2_Hal_t *sh2_hal_init(void);
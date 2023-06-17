#include "../../includes.h"
#include "../../drivers/SPIM/spim_local.h"

extern int interrupt_flag;

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins);


extern const struct gpio_dt_spec fsmrrstn;
extern const struct gpio_dt_spec fsmbootn;
void FSM_thread(void);

// Maximum SHTP Transfer and Payload sizes
#define SH2_HAL_MAX_TRANSFER_OUT (128)
#define SH2_HAL_MAX_PAYLOAD_OUT  (128)
#define SH2_HAL_MAX_TRANSFER_IN  (1024)

// ------------------------------------------------------------------------
// Private type definitions

#define CHAN_EXECUTABLE_DEVICE    (1)
#define CHAN_SENSORHUB_CONTROL    (2)
#define CHAN_SENSORHUB_INPUT      (3)
#define CHAN_SENSORHUB_INPUT_WAKE (4)
#define CHAN_SENSORHUB_INPUT_GIRV (5)
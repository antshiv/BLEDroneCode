#include "../../includes.h"
#include "sh2/sh2.h"
#include "../../drivers/SPIM/spim_local.h"

extern int interrupt_flag;
#define ARRAY_LEN(a) ((sizeof(a))/(sizeof(a[0])))

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
					uint32_t pins);


extern const struct gpio_dt_spec fsmrrstn;
extern const struct gpio_dt_spec fsmbootn;
extern const struct gpio_dt_spec fsmwaken;
void FSM_thread(void);

typedef enum SpiState_e
{
    SPI_INIT,
    SPI_DUMMY,
    SPI_DFU,
    SPI_IDLE,
    SPI_RD_HDR,
    SPI_RD_BODY,
    SPI_WRITE
} SpiState_t;

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

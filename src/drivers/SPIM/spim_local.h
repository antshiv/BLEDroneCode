#include "../../includes.h"

// SPI master functionality

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
extern const struct device *spi_dev;
static struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);
static const struct gpio_dt_spec spi4_cs = GPIO_DT_SPEC_GET(DT_ALIAS(spi4_cs), gpios);

extern struct spi_cs_control spim_cs;

void spi_init(void);
int spi_write_test_msg(void);
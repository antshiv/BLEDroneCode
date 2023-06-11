#include "../../includes.h"

// SPI master functionality

#define MY_SPI_MASTER DT_NODELABEL(my_spi_master)
extern const struct device *spi_dev;
extern struct k_poll_signal spi_done_sig;
extern const struct gpio_dt_spec spi4_cs;
extern const struct spi_config spi_cfg;

extern struct spi_cs_control spim_cs;

void spi_init(void);
int spi_write_test_msg(void);
void spi_write_msg(uint16_t len, uint8_t *tx_data, uint8_t *rx_data);
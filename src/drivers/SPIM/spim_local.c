#include "../../includes.h"
#include "spim_local.h"

const struct device *spi_dev;
struct k_poll_signal spi_done_sig = K_POLL_SIGNAL_INITIALIZER(spi_done_sig);
const struct gpio_dt_spec spi4_cs = GPIO_DT_SPEC_GET(DT_ALIAS(spi4_cs), gpios);
struct spi_cs_control spim_cs = {
    .gpio = SPI_CS_GPIOS_DT_SPEC_GET(DT_NODELABEL(reg_my_spi_master)),
    .delay = 0,
};

const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
                 SPI_MODE_CPOL | SPI_MODE_CPHA | SPI_HOLD_ON_CS,
    .frequency = 500000,//1000000,
    .slave = 0,
    .cs = &spim_cs,
};

void spi_init(void)
{
    int ret;
    spi_dev = DEVICE_DT_GET(MY_SPI_MASTER);
    if (!device_is_ready(spi_dev))
    {
        printk("SPI master device not ready!\n");
    }
    if (!device_is_ready(spim_cs.gpio.port))
    {
        printk("SPI master chip select device not ready!\n");
    }
	if (!gpio_is_ready_dt(&spi4_cs))
    {
        printk("Error: SPI chip select device %s is not ready\n",
               spi4_cs.port->name);
        return 0;
    }

	ret = gpio_pin_configure_dt(&spi4_cs, GPIO_OUTPUT_HIGH);
	if (ret != 0)
	{
		printk("Error %d: failed to configure %s pin %d\n",
			   ret, spi4_cs.port->name, spi4_cs.pin);
		return 0;
	}
}

int spi_write_test_msg(void)
{
    static uint8_t counter = 0;
    static uint8_t tx_buffer[2];
    static uint8_t rx_buffer[2];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = sizeof(tx_buffer)};
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1};

    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = sizeof(rx_buffer),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1};

    // Update the TX buffer with a rolling counter
    // tx_buffer[0] = counter++;
    tx_buffer[0] = 0x02;
    tx_buffer[1] = 0x02;
    printk("SPI TX: 0x%.2x, 0x%.2x\n", tx_buffer[0], tx_buffer[1]);

    // Reset signal
    k_poll_signal_reset(&spi_done_sig);

    // Start transaction
    int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
    if (error != 0)
    {
        printk("SPI transceive error: %i\n", error);
        return error;
    }

    // Wait for the done signal to be raised and log the rx buffer
    int spi_signaled, spi_result;
    do
    {
        k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
    } while (spi_signaled == 0);
    printk("SPI RX: 0x%.2x, 0x%.2x\n", rx_buffer[0], rx_buffer[1]);
    return 0;
}

void spi_write_msg(uint16_t len ,uint8_t *tx_buffer, uint8_t *rx_buffer)
{
    //printk("SPI TX len %d\n", len);

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = len};
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1};

    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = len,
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1};

    // Reset signal
    k_poll_signal_reset(&spi_done_sig);

    // Start transaction
    int error = spi_transceive_async(spi_dev, &spi_cfg, &tx, &rx, &spi_done_sig);
    if (error != 0)
    {
        printk("SPI transceive error: %i\n", error);
        return error;
    }
    // Wait for the done signal to be raised and log the rx buffer
    int spi_signaled, spi_result;
    do
    {
        k_poll_signal_check(&spi_done_sig, &spi_signaled, &spi_result);
    } while (spi_signaled == 0);
    /*
    for (int i = 0; i < len; i++)
    {
        printk("SPI RX[%d]: 0x%.2x\n", i, rx_buffer[i]);
    }
    */
    return 0;
}

/*
 * A build error on this line means your board is unsupported.
 * See the sample documentation for information on how to fix this.
 */
// static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/*************OLD SPI *************************************/
/* static const struct spi_config spi_cfg = {
    .operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB |
             SPI_MODE_CPOL | SPI_MODE_CPHA,
    .frequency = 4000000,
    .slave = 0,
};

const struct device * spi_dev;

static void spi_init(void)
{
    const char* const spiName = "SPI_4";
    int err, ret;

    printk("SPI4 example application\n");

    spi_dev = DEVICE_DT_GET(DT_NODELABEL(arduino_spi));
    if (!device_is_ready(spi4_cs.port)) {
        printk("spi4 cs is not ready\n");
        return;
    }

    ret = gpio_pin_configure_dt(&spi4_cs, GPIO_OUTPUT_INIT_LOW);
    if (ret < 0) {
        printk("spi4 cs config failed\n");
        return;
    }

    if (spi_dev == NULL) {
        printk("Could not get %s device\n", spiName);
        return;
    }
}

void spi_test_send(void)
{
    int err;
    static uint8_t tx_buffer[1];
    static uint8_t rx_buffer[1];

    const struct spi_buf tx_buf = {
        .buf = tx_buffer,
        .len = sizeof(tx_buffer)
    };
    const struct spi_buf_set tx = {
        .buffers = &tx_buf,
        .count = 1
    };

    struct spi_buf rx_buf = {
        .buf = rx_buffer,
        .len = sizeof(rx_buffer),
    };
    const struct spi_buf_set rx = {
        .buffers = &rx_buf,
        .count = 1
    };

    err = spi_transceive(spi_dev, &spi_cfg, &tx, &rx);
    if (err) {
        printk("SPI error: %d\n", err);
    } else {
         Connect MISO to MOSI for loopback
        printk("TX sent: %x\n", tx_buffer[0]);
        printk("RX recv: %x\n", rx_buffer[0]);
        tx_buffer[0]++;
    }
} */
/*******************END OLD SPI ******************************/

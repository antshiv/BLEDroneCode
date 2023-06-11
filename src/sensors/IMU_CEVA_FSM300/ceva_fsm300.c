#include "ceva_fsm300.h"

#include "sh2/sh2.h"
#include "sh2/sh2_util.h"
#include "sh2/sh2_err.h"
#include "sh2/sh2_SensorValue.h"
//#include "sh2/sh2_hal_init.h"


// ------------------------------------------------------------------------
// Private data

// Dummy transmit data for SPI reads
static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER_IN] = {0};
const struct gpio_dt_spec fsmrstn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmrstn), gpios);
const struct gpio_dt_spec fsmbootn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmbootn), gpios);

// Timer handle
//static TIM_HandleTypeDef tim2;

// SPI Peripheral, SPI1
//static SPI_HandleTypeDef spi;

// SPI Bus access state machine state
//static SpiState_t spiState = SPI_INIT;

// Timestamp
static volatile uint32_t rxTimestamp_us;

// true from time SH is put in reset until first INTN indication
static volatile bool inReset;

// set true when INTN is observed, until RX operation starts
static volatile bool rxReady;

// Receive support
static uint8_t rxBuf[SH2_HAL_MAX_TRANSFER_IN];
static volatile uint32_t rxBufLen;
static volatile bool rxDataReady;

// Transmit support
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_OUT];
static uint32_t txBufLen;

// Instances of the SPI HAL for SH2 and DFU
static sh2_Hal_t sh2Hal;
static sh2_Hal_t dfuHal;

static bool isOpen = false;

// ------------------------------------------------------------------------

int interrupt_flag = 0;

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    int err = 0;
    interrupt_flag = 1;
    // printk("FSM has an interrup triggered at %" PRIu32 "\n", k_cycle_get_32());
}

void FSM_process_data() {
            int ret, err;
     // printk("FSM thread running %" PRIu32 "\n", k_cycle_get_32());
            /* Interfacting with sensor using SPI */
            //gpio_pin_set_dt(&spi4_cs, GPIO_OUT_PIN0_Low);

            if (err < 0)
            {
                return;
            }
            // k_msleep(SLEEP_TIME_MS);
            //  spi_write_test_msg();
            static uint8_t *tx_buffer;
            static uint8_t *rx_buffer;
            int len = 4;
            uint16_t packet_length = 0;
            uint8_t packet_hdr_length = 4;

            tx_buffer = k_calloc(len, sizeof(uint8_t));
            rx_buffer = k_calloc(len, sizeof(uint8_t));

            /* send an header to see how much is the packet load */
            err = gpio_pin_toggle_dt(&spi4_cs);
            spi_write_msg(packet_hdr_length, tx_buffer, rx_buffer);
            packet_length = rx_buffer[1] << 8 | rx_buffer[0];
            printk("Packet length: %d\n", packet_length);

            k_free(tx_buffer);
            k_free(rx_buffer);
            
            
            //k_msleep(SLEEP_TIME_MS * 0.06);

            /* the paylaod data */
            tx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));
            rx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));

            spi_write_msg(packet_length - packet_hdr_length , tx_buffer, rx_buffer);
            err = gpio_pin_toggle_dt(&spi4_cs);
            printk("Packet data[0]: %d\n", rx_buffer[0]);

            k_free(tx_buffer);
            k_free(rx_buffer);
            spi_release(spi_dev, &spi_cfg);

            //gpio_pin_set_dt(&spi4_cs, GPIO_OUT_PIN0_High);
}

void FSM_init() {
    /* call process data 3 times to init the sensor*/
    printk("FSM init start\n");
    FSM_process_data();
    k_msleep(SLEEP_TIME_MS);
    FSM_process_data();
    k_msleep(SLEEP_TIME_MS);
    FSM_process_data();
    printk("FSM init end\n");
}

void FSM_thread(void)
{
    int err = 0;
    for (;;)
    {
        if (interrupt_flag == 1)
        {
            // printk("FSM thread running %" PRIu32 "\n", k_cycle_get_32());
            /* Interfacting with sensor using SPI */
            // k_msleep(SLEEP_TIME_MS);
            //  spi_write_test_msg();
            static uint8_t *tx_buffer;
            static uint8_t *rx_buffer;
            int len = 4;
            uint16_t packet_length = 0;
            uint8_t packet_hdr_length = 4;

            tx_buffer = k_calloc(len, sizeof(uint8_t));
            rx_buffer = k_calloc(len, sizeof(uint8_t));

            /* send an header to see how much is the packet load */
            spi_write_msg(len, tx_buffer, rx_buffer);
            packet_length = rx_buffer[1] << 8 | rx_buffer[0];
            printk("Packet length: %d\n", packet_length);

            k_free(tx_buffer);
            k_free(rx_buffer);
            
            
            k_msleep(SLEEP_TIME_MS * 0.06);

            /* the paylaod data */
            tx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));
            rx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));

            spi_write_msg(packet_length - packet_hdr_length , tx_buffer, rx_buffer);
            printk("Packet data[0]: %d\n", rx_buffer[0]);

            k_free(tx_buffer);
            k_free(rx_buffer);

            spi_release(spi_dev, &spi_cfg);

            /* end sending info using SPI */
            interrupt_flag = 0;
            k_yield();
        }
    }
}

// ------------------------------------------------------------------------
// Private methods

static void bootn(bool state)
{
    int ret;
    //HAL_GPIO_WritePin(BOOTN_PORT, BOOTN_PIN, 
    //                  state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    ret =  gpio_pin_set_dt(&fsmbootn, (int)state);
    if (ret < 0)
    {
        printk("Error setting state %d GPIO: %d\n", state , ret);
    }
}

static void rstn(bool state)
{
    int ret;
    ret =  gpio_pin_set_dt(&fsmrstn, (int)state);
    if (ret < 0)
    {
        printk("Error setting state %d GPIO: %d\n", state , ret);
    }
}




// ------------------------------------------------------------------------
// SH2 SPI Hal Methods

static int sh2_spi_hal_open(sh2_Hal_t *self)
{

}

static void sh2_spi_hal_close(sh2_Hal_t *self)
{

}

static int sh2_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{

}

static int sh2_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{

}

static uint32_t sh2_spi_hal_getTimeUs(sh2_Hal_t *self)
{

}

// ------------------------------------------------------------------------
// Public methods

sh2_Hal_t *sh2_hal_init(void)
{
    // Set up the HAL reference object for the client
    sh2Hal.open = sh2_spi_hal_open;
    sh2Hal.close = sh2_spi_hal_close;
    sh2Hal.read = sh2_spi_hal_read;
    sh2Hal.write = sh2_spi_hal_write;
    sh2Hal.getTimeUs = sh2_spi_hal_getTimeUs;

    return &sh2Hal;
}

sh2_Hal_t *dfu_hal_init(void)
{
    // Set up the HAL reference object for the client
    //dfuHal.open = dfu_spi_hal_open;
    //dfuHal.close = dfu_spi_hal_close;
    //dfuHal.read = dfu_spi_hal_read;
    //dfuHal.write = dfu_spi_hal_write;
    //dfuHal.getTimeUs = dfu_spi_hal_getTimeUs;

    //return &dfuHal;
}


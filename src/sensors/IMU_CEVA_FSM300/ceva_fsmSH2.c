/*
    SH2 Open Source Driver Project

*/
#include "ceva_fsmSH2.h"
const struct gpio_dt_spec fsmrstn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmrstn), gpios);
const struct gpio_dt_spec fsmbootn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmbootn), gpios);

#define RESET_DELAY_US (10000)
#define SLEEP_TIME_MS 1
int interrupt_flag = 0;
int resetComplete = false;

static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER_IN] = {0};

// Receive support
static uint8_t rxBuf[SH2_HAL_MAX_TRANSFER_IN];
static volatile uint32_t rxBufLen;
static volatile bool rxDataReady;

// Transmit support
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_OUT];
static uint32_t txBufLen;

static void bootn(bool state)
{
    int ret;
    ret = gpio_pin_set_dt(&fsmbootn, (int)state);
    if (ret < 0)
    {
        printk("Error setting state %d GPIO: %d\n", state, ret);
    }
    ret = gpio_pin_get_dt(&fsmbootn);
}

static void rstn(bool state)
{
    int ret;
    ret = gpio_pin_set_dt(&fsmrstn, (int)state);
    if (ret < 0)
    {
        printk("Error setting state %d GPIO: %d\n", state, ret);
    }
}

static void ceva_fsmSH2_dummy() {
    bootn(false);
    rstn(false);
    printk("ceva_fsmSH2_Open\n");
    /* send dummy SPI */
    uint8_t tx_buffer[1] = {0xAA};
    uint8_t rx_buffer[1] = {0x00};
    spi_write_msg(1, tx_buffer, rx_buffer);
    spi_release(spi_dev, &spi_cfg);
        k_usleep(RESET_DELAY_US);

    // Deassert reset, boot in non-DFU mode
    bootn(true);
    rstn(true);
    //enable interrupts
    enableInts();
    printk("ceva_fsmSH2_Open done\n");

}


void ceva_fsmSH2_Open() {
    ceva_fsmSH2_dummy();
}

static void fsmProcessInit() {
    if (resetComplete) {
        return;
    }
    /* extract channel id from packet */
    int channel_id = rxBuf[2];
    switch (channel_id)
    {
    case CHAN_EXECUTABLE_DEVICE: 
        if (rxBuf[4] == 0x01) {
            printk("FSM is ready\n");
            resetComplete = true;
        }
        /* code */
        break;
    default:
        break;
    }
}

void FSM_thread(void) {
    for (;;) {
        if (interrupt_flag) {
            interrupt_flag = 0;
            int len = 4;
            uint16_t packet_length = 0;
            uint8_t packet_hdr_length = 4;
            memset(rxBuf, 0, SH2_HAL_MAX_TRANSFER_IN);
            memset(txBuf, 0, SH2_HAL_MAX_TRANSFER_OUT);

            /* send an header to see how much is the packet load */
            spi_write_msg(len, txBuf, rxBuf);
            packet_length = rxBuf[1] << 8 | rxBuf[0];
            printk("Packet length: %d\n", packet_length);

            k_msleep(SLEEP_TIME_MS * 0.01);

            /* the paylaod data */

            spi_write_msg(packet_length - packet_hdr_length, txBuf, rxBuf+4);
            printk("Packet data[0]: %x\n", rxBuf[4]);
            spi_release(spi_dev, &spi_cfg);
            fsmProcessInit();

            /* end sending info using SPI */
            interrupt_flag = 0;
        }
        k_yield();
    }
}

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    int err = 0;
    interrupt_flag = 1;
    //rxTimestamp_us = k_uptime_get_32() * 1000;
    //inReset = false;
    //rxReady = true;
    // Start read if possible
    //spiActivate();
    // printk("FSM has an interrup triggered at %" PRIu32 "\n", k_cycle_get_32());
}
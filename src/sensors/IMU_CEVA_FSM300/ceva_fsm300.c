#include "ceva_fsm300.h"

#include "sh2/sh2.h"
#include "sh2/sh2_util.h"
#include "sh2/sh2_err.h"
#include "sh2/sh2_SensorValue.h"
// #include "sh2/sh2_hal_init.h"

// ------------------------------------------------------------------------
// Private data

// Dummy transmit data for SPI reads
static const uint8_t txZeros[SH2_HAL_MAX_TRANSFER_IN] = {0};
const struct gpio_dt_spec fsmrstn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmrstn), gpios);
const struct gpio_dt_spec fsmbootn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmbootn), gpios);

// Timer handle
// static TIM_HandleTypeDef tim2;

// SPI Peripheral, SPI1
// static SPI_HandleTypeDef spi;

// SPI Bus access state machine state
// static SpiState_t spiState = SPI_INIT;

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

// Keep reset asserted this long.
// (Some targets have a long RC decay on reset.)
#define RESET_DELAY_US (10000)

// Wait up to this long to see first interrupt from SH
#define START_DELAY_US (2000000)

// How many bytes to read when reading the length field
#define READ_LEN (4)

// ------------------------------------------------------------------------
// Private types

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

// ------------------------------------------------------------------------

// SPI Bus access state machine state
static SpiState_t spiState = SPI_INIT;

// ------------------------------------------------------------------------

int interrupt_flag = 0;


void FSM_process_data()
{
    int ret, err;
    // printk("FSM thread running %" PRIu32 "\n", k_cycle_get_32());
    /* Interfacting with sensor using SPI */
    // gpio_pin_set_dt(&spi4_cs, GPIO_OUT_PIN0_Low);

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
    memcpy(rxBuf, rx_buffer, packet_hdr_length);

    k_free(tx_buffer);
    k_free(rx_buffer);

    // k_msleep(SLEEP_TIME_MS * 0.06);

    /* the paylaod data */
    tx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));
    rx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));

    spi_write_msg(packet_length - packet_hdr_length, tx_buffer, rx_buffer);
    err = gpio_pin_toggle_dt(&spi4_cs);
    printk("Packet data[0]: %d\n", rx_buffer[0]);
    memcpy(rxBuf + READ_LEN, rx_buffer, packet_length);
    rxBufLen = packet_length;
    printk("rx buf length: %d\n", rxBufLen);
    k_free(tx_buffer);
    k_free(rx_buffer);
    spi_release(spi_dev, &spi_cfg);

    // gpio_pin_set_dt(&spi4_cs, GPIO_OUT_PIN0_High);
}

void FSM_init()
{
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
            printk("FSM thread running interrupt \n");
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
            memcpy(rxBuf, rx_buffer, packet_hdr_length);
            //txBufLen = 4;
            //if (txBufLen < 1)
            //{
              //  printk("txBufLen is 0\n");
              //  return;
                
            //}
            //printk("send length: %d\n", txBufLen);
            //spi_write_msg(txBufLen, txBuf, rxBuf);
            //uint16_t rxLen = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;
            packet_length = rx_buffer[1] << 8 | rx_buffer[0];
            //uint16_t rxLen = rxBuf[1] << 8 | rxBuf[0];
            printk("Packet length: %d\n", packet_length);

            k_free(tx_buffer);
            k_free(rx_buffer);

            k_msleep(SLEEP_TIME_MS * 0.06);

            /* the paylaod data */
            tx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));
            rx_buffer = k_calloc(packet_length - packet_hdr_length, sizeof(uint8_t));
            //spi_write_msg(rxLen-READ_LEN, txZeros, rxBuf+READ_LEN);

            spi_write_msg(packet_length - packet_hdr_length, tx_buffer, rx_buffer);
            printk("Packet data[0]: %x\n", rx_buffer[0]);
            memcpy(rxBuf + READ_LEN, rx_buffer, packet_length);
            rxBufLen = packet_length;
            printk("rx buf length: %d\n", rxBufLen);

            k_free(tx_buffer);
            k_free(rx_buffer);

            spi_release(spi_dev, &spi_cfg);

            /* end sending info using SPI */
            interrupt_flag = 0;
        }
        if (txBufLen > 0) {
            printk("FSM thread running txBufLen > 0 \n");
            spiState = SPI_WRITE;
            printk("txBufLen: %d and txbuf[0] = %d\n", txBufLen, txBuf[0]);
            spi_write_msg(txBufLen, txBuf, rxBuf);
            uint16_t rxLen = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;
            rxBufLen = (txBufLen < rxLen) ? txBufLen : rxLen;
            // Tx buffer is empty now.
            txBufLen = 0;
            // transition back to idle state
            spiState = SPI_IDLE;
            spi_release(spi_dev, &spi_cfg);
        }
        k_yield();
    }
}

// ------------------------------------------------------------------------
// Private methods

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

static void spiDummyOp(void)
{
    // We need to establish SCLK in proper initial state.
    // Do one SPI operation with reset asserted and no CS asserted to get clock sorted.
    uint8_t dummyTx[1];
    uint8_t dummyRx[1];

    memset(dummyTx, 0xAA, sizeof(dummyTx));
    spi_write_msg(1, dummyTx, dummyRx);
    spi_release(spi_dev, &spi_cfg);
}

static void spiActivate(void)
{
        if ((spiState == SPI_IDLE) && (rxBufLen == 0))
    {
        if (rxReady)
        {
            // reset flag that was set with INTN
            rxReady = false;
            
            // assert CSN
            //csn(false);

            if (txBufLen > 0)
            {
                spiState = SPI_WRITE;
                
                // Start operation to write (and, incidentally, read)
                //HAL_SPI_TransmitReceive_IT(&spi, txBuf, rxBuf, txBufLen);
                spi_write_msg(txBufLen, txBuf, rxBuf);
                //spi_release(spi_dev, &spi_cfg);

                // Deassert Wake
                //ps0_waken(true);
            }
            else
            {
                spiState = SPI_RD_HDR;
                
                // Start SPI operation to read header (writing zeros)
                //HAL_SPI_TransmitReceive_IT(&spi, (uint8_t *)txZeros, rxBuf, READ_LEN);
                spi_write_msg(READ_LEN, txZeros, rxBuf);
            }
        }
    }
}


// Handle the end of a SPI operation.
// This can be done from interrupt context or with interrupts disabled.
// Depending on spiState, it may start a follow-up operation or transition
// to idle.  In the latter case, it will call spiActivate
static void spiCompleted(void)
{
    // Get length of payload available
    uint16_t rxLen = (rxBuf[0] + (rxBuf[1] << 8)) & ~0x8000;
        
    // Truncate that to max len we can read
    if (rxLen > sizeof(rxBuf))
    {
        rxLen = sizeof(rxBuf);
    }

    if (spiState == SPI_DUMMY)
    {
        // SPI Dummy operation completed, transition now to idle
        spiState = SPI_IDLE;
    }
    else if (spiState == SPI_RD_HDR)
    {
        // We read a header

        if (rxLen > READ_LEN) {
            // There is more to read

            // Transition to RD_BODY state
            spiState = SPI_RD_BODY;
        
            // Start a read operation for the remaining length.  (We already read the first READ_LEN bytes.)
            //HAL_SPI_TransmitReceive_IT(&spi, (uint8_t *)txZeros, rxBuf+READ_LEN, rxLen-READ_LEN);
            spi_write_msg(rxLen-READ_LEN, txZeros, rxBuf+READ_LEN);
        }
        else
        {
            // No SHTP payload was received, this operation is done
            //csn(true);            // deassert CSN
            rxBufLen = 0;         // no rx data available
            spiState = SPI_IDLE;  // back to idle state
            spiActivate();        // activate next operation, if any.
            spiCompleted();
        }
    }
    else if (spiState == SPI_RD_BODY)
    {
        // We completed the read or write of a payload
        // deassert CSN.
        //csn(true);

        // Check len of data read and set rxBufLen
        rxBufLen = rxLen;

        // transition back to idle state
        spiState = SPI_IDLE;

        // Activate the next operation, if any.
        spiActivate();
        spiCompleted();
    }
    else if (spiState == SPI_WRITE)
    {
        // We completed the read or write of a payload
        // deassert CSN.
        //csn(true);

        // Since operation was a write, transaction was for txBufLen bytes.  So received
        // data len is, at a maximum, txBufLen.
        rxBufLen = (txBufLen < rxLen) ? txBufLen : rxLen;

        // Tx buffer is empty now.
        txBufLen = 0;
        
        // transition back to idle state
        spiState = SPI_IDLE;

        // Activate the next operation, if any.
        spiActivate();
        spiCompleted();
    }
}


void FSM_thread1(void) {
    for(;;) {
        if (interrupt_flag) {
            printk("FSM has an interrupt\n");
            interrupt_flag = 0;
            spiActivate();
            spiCompleted();
            spi_release(spi_dev, &spi_cfg);
        }
        k_yield();
        k_msleep(1);
    }
}


void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    int err = 0;
    interrupt_flag = 1;
    rxTimestamp_us = k_uptime_get_32() * 1000;
    inReset = false;
    rxReady = true;
    // Start read if possible
    //spiActivate();
    // printk("FSM has an interrup triggered at %" PRIu32 "\n", k_cycle_get_32());
}


// ------------------------------------------------------------------------
// SH2 SPI Hal Methods

void resetDelayUs(uint32_t delay)
{
    volatile uint32_t now = k_uptime_get() * 1000;
    uint32_t start = now;
    while (((now - start) < delay) && (inReset))
    {
        now = k_uptime_get() * 1000;
    }
}

static int sh2_spi_hal_open(sh2_Hal_t *self)
{
    //printk('sh2_spi_hal_open \n');
    int retval = SH2_OK;

    if (isOpen)
    {
        // Can't open if another instance is already open
        return SH2_ERR;
    }

    isOpen = true;

    // Hold in reset
    bootn(false);
    rstn(false);
    // Clear rx, tx buffers
    rxBufLen = 0;
    txBufLen = 0;
    rxDataReady = false;
    rxReady = false;

    inReset = true; // will change back to false when INTN serviced

    // Do dummy SPI operation
    // (First SPI op after reconfig has bad initial state of signals
    // so this is a throwaway operation.  Afterward, all is well.)
    spiState = SPI_DUMMY;
    spiDummyOp();
    spiState = SPI_IDLE;

    // Delay for RESET_DELAY_US to ensure reset takes effect
    k_usleep(RESET_DELAY_US);

    // Deassert reset, boot in non-DFU mode
    bootn(true);
    rstn(true);
  
    //enable interrupts
    enableInts();

    //k_usleep(START_DELAY_US);
    // Wait for INTN to be asserted
    resetDelayUs(START_DELAY_US);
    k_usleep(START_DELAY_US);
    printk("delay done \n"); 
    return retval;
}

static void sh2_spi_hal_close(sh2_Hal_t *self)
{
        // Disable interrupts
    disableInts();
    
    // Set state machine to INIT state
    spiState = SPI_INIT;
    
    // Hold sensor hub in reset
    rstn(false);
    


    // No longer open
    isOpen = false;
}

static int sh2_spi_hal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len, uint32_t *t)
{
     int retval = 0;
     //printk("sh2_spi_hal_read \n");

        // If there is received data available...
    if (rxBufLen > 0)
    {
        printk("rxBufLen > 0  %d \n", rxBufLen);
        // And if the data will fit in this buffer...
        if (len >= rxBufLen)
        {
            // Copy data to the client buffer
            memcpy(pBuffer, rxBuf, rxBufLen);
            retval = rxBufLen;

            // Set timestamp of that data
            *t = rxTimestamp_us;

            // Clear rxBuf so we can receive again
            rxBufLen = 0;
        }
        else
        {
            // Discard what was read and return error because buffer was too small.
            retval = SH2_ERR_BAD_PARAM;
            rxBufLen = 0;
        }
        
        // Now that rxBuf is empty, activate SPI processing to send any
        // potential write that was blocked.
        //disableInts();
        //spiActivate();
        //spiCompleted();
        //spi_release(spi_dev, &spi_cfg);
        //enableInts();
        return retval; 
    }
    k_yield();

}

static int sh2_spi_hal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len)
{
    printk("sh2_spi_hal_write \n");
    int retval = SH2_OK;
    // Validate parameters
    if ((self == 0) || (len > sizeof(txBuf)) ||
        ((len > 0) && (pBuffer == 0)))
    {
        return SH2_ERR_BAD_PARAM;
    }

    // If tx buffer is not empty, return 0
    if (txBufLen != 0)
    {
        return 0;
    }

    // Copy data to tx buffer
    memcpy(txBuf, pBuffer, len);
    printk("HAL write txBuf is %d and length is %d \n", txBuf[0], len);
    txBufLen = len;
    retval = len;
    return retval;
}

static uint32_t sh2_spi_hal_getTimeUs(sh2_Hal_t *self)
{
    //printk("sh2_spi_hal_getTimeUs \n");
    uint32_t time = k_uptime_get() * 1000;
    //printk("time is %d \n", time);
    return time;
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
    // dfuHal.open = dfu_spi_hal_open;
    // dfuHal.close = dfu_spi_hal_close;
    // dfuHal.read = dfu_spi_hal_read;
    // dfuHal.write = dfu_spi_hal_write;
    // dfuHal.getTimeUs = dfu_spi_hal_getTimeUs;

    // return &dfuHal;
}

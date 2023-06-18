/*
    SH2 Open Source Driver Project

*/
#include "ceva_fsmSH2.h"
const struct gpio_dt_spec fsmrstn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmrstn), gpios);
const struct gpio_dt_spec fsmbootn = GPIO_DT_SPEC_GET(DT_ALIAS(fsmbootn), gpios);
const struct gpio_dt_spec fsmwaken = GPIO_DT_SPEC_GET(DT_ALIAS(fsmwaken), gpios);

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
static uint8_t txBuf[SH2_HAL_MAX_TRANSFER_IN];
static uint32_t txBufLen;

// SPI Bus access state machine state
static SpiState_t spiState = SPI_INIT;

// ------------------------------------------------------------------------
// Set Sensor Config

// Report definitions
// Bit fields for Feature Report flags
#define FEAT_CHANGE_SENSITIVITY_MODE_BIT   0
#define FEAT_CHANGE_SENSITIVITY_ENABLE_BIT 1
#define FEAT_WAKEUP_ENABLE_BIT             2
#define FEAT_ALWAYS_ON_ENABLE_BIT          3
#define FEAT_SNIFF_ENABLE_BIT              4
#define FEAT_CHANGE_SENSITIVITY_RELATIVE   (1 << FEAT_CHANGE_SENSITIVITY_MODE_BIT)
#define FEAT_CHANGE_SENSITIVITY_ABSOLUTE   (0 << FEAT_CHANGE_SENSITIVITY_MODE_BIT)
#define FEAT_CHANGE_SENSITIVITY_ENABLED    (1 << FEAT_CHANGE_SENSITIVITY_ENABLE_BIT)
#define FEAT_CHANGE_SENSITIVITY_DISABLED   (0 << FEAT_CHANGE_SENSITIVITY_ENABLE_BIT)
#define FEAT_WAKE_ENABLED                  (1 << FEAT_WAKEUP_ENABLE_BIT)
#define FEAT_WAKE_DISABLED                 (0 << FEAT_WAKEUP_ENABLE_BIT)
#define FEAT_ALWAYS_ON_ENABLED             (1 << FEAT_ALWAYS_ON_ENABLE_BIT)
#define FEAT_ALWAYS_ON_DISABLED            (0 << FEAT_ALWAYS_ON_ENABLE_BIT)
#define FEAT_SNIFF_ENABLED                 (1 << FEAT_SNIFF_ENABLE_BIT)
#define FEAT_SNIFF_DISABLED                (0 << FEAT_SNIFF_ENABLE_BIT)

// SENSORHUB_SET_FEATURE_CMD
#define SENSORHUB_SET_FEATURE_CMD    (0xFD)
typedef struct PACKED_STRUCT {
    uint8_t reportId;             // 0xFD
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} SetFeatureReport_t;

/******************************/
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

static void waken(bool state)
{
    int ret;
    ret = gpio_pin_set_dt(&fsmwaken, (int)state);
    if (ret < 0)
    {
        printk("Error setting state %d GPIO: %d\n", state, ret);
    }
}

/******************************/

    /**
 * @brief Set sensor configuration. (e.g enable a sensor at a particular rate.)
 *
 * @param  sensorId Which sensor to configure.
 * @param  pConfig Pointer to structure holding sensor configuration.
 * @return SH2_OK (0), on success.  Negative value from sh2_err.h on error.
 */
int sh2_setSensorConfig(sh2_SensorId_t sensorId, const sh2_SensorConfig_t *pConfig)
{
    SetFeatureReport_t report;
    int status;

    report.reportId = SENSORHUB_SET_FEATURE_CMD;
    report.featureReportId = sensorId;
    report.flags = 0;
    if (pConfig->changeSensitivityEnabled)
    {
        report.flags |= FEAT_CHANGE_SENSITIVITY_ENABLED;
    }
    if (pConfig->wakeupEnabled)
    {
        report.flags |= FEAT_WAKE_ENABLED;
    }
    if (pConfig->changeSensitivityRelative)
    {
        report.flags |= FEAT_CHANGE_SENSITIVITY_RELATIVE;
    }
    if (pConfig->alwaysOnEnabled)
    {
        report.flags |= FEAT_ALWAYS_ON_ENABLED;
    }
    if (pConfig->sniffEnabled)
    {
        report.flags |= FEAT_SNIFF_ENABLED;
    }
    report.changeSensitivity = pConfig->changeSensitivity;
    report.reportInterval_uS = pConfig->reportInterval_us;
    report.batchInterval_uS = pConfig->batchInterval_us;
    report.sensorSpecific = pConfig->sensorSpecific;
            
    memset(rxBuf, 0, SH2_HAL_MAX_TRANSFER_IN);
    memset(txBuf, 0, SH2_HAL_MAX_TRANSFER_IN);
    uint8_t reportLen = sizeof(report);
    uint16_t hdrLen = 4 * sizeof(uint8_t);
    txBuf[0] = 21;
    txBuf[2] = 2;//CHAN_SENSORHUB_CONTROL;
    txBuf[3] = 0; 
    memcpy(&txBuf[4], &report, 17);
    //printk('write to spi'); 
    spi_write_msg(17 + 4, txBuf, rxBuf);
    spi_release(spi_dev, &spi_cfg);
    spiState = SPI_IDLE;
    //k_sleep(K_MSEC(1U));
    //waken(false);
}

// Configure one sensor to produce periodic reports
static void startReports()
{
    static sh2_SensorConfig_t config;
    int status;
    int sensorId;
    static const int enabledSensors[] =
    {
        SH2_GAME_ROTATION_VECTOR,
        // SH2_RAW_ACCELEROMETER,
        // SH2_RAW_GYROSCOPE,
        // SH2_ROTATION_VECTOR,
        // SH2_GYRO_INTEGRATED_RV,
        // SH2_IZRO_MOTION_REQUEST,
        // SH2_SHAKE_DETECTOR,
    };

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.sniffEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    // Select a report interval.
    // config.reportInterval_us = 100000;  // microseconds (10 Hz)
    // config.reportInterval_us = 40000;  // microseconds (25 Hz)
    config.reportInterval_us = 10000;  // microseconds (100 Hz)
    // config.reportInterval_us = 2500;   // microseconds (400 Hz)
    // config.reportInterval_us = 1000;   // microseconds (1000 Hz)

    for (int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorId = enabledSensors[n];
        status = sh2_setSensorConfig(sensorId, &config);
        if (status != 0) {
            printk("Error while enabling sensor %d\n", sensorId);
        }
    }
    
}

// ------------------------------------------------------------------------
// Get FRS.




static void ceva_fsmSH2_dummy() {
    bootn(false);
    rstn(false);
    printk("ceva_fsmSH2 Dummy\n");
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
    printk("ceva_fsmSH2 Dummy done\n");

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
            /* need to send start Report*/
            gpio_pin_configure_dt(&fsmwaken, GPIO_OUTPUT);
            waken(false);
            spiState = SPI_WRITE; 
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
            if (spiState == SPI_WRITE) {
                startReports();
            } else {
            int len = 4;
            uint16_t packet_length = 0;
            uint8_t packet_hdr_length = 4;
            memset(rxBuf, 0, SH2_HAL_MAX_TRANSFER_IN);
            memset(txBuf, 0, SH2_HAL_MAX_TRANSFER_IN);

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
            }
            interrupt_flag = 0;
        } else {
           // printk('buffer lenght is %d', txBuf[0]);
        }

        k_yield();
    }
}

void fsm_interrupt_triggered(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    int err = 0;
    interrupt_flag = 1;
    waken(true);
    //rxTimestamp_us = k_uptime_get_32() * 1000;
    //inReset = false;
    //rxReady = true;
    // Start read if possible
    //spiActivate();
    // printk("FSM has an interrup triggered at %" PRIu32 "\n", k_cycle_get_32());
}
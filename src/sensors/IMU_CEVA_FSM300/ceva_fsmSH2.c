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
int wakenComplete = true;

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
#define FEAT_CHANGE_SENSITIVITY_MODE_BIT 0
#define FEAT_CHANGE_SENSITIVITY_ENABLE_BIT 1
#define FEAT_WAKEUP_ENABLE_BIT 2
#define FEAT_ALWAYS_ON_ENABLE_BIT 3
#define FEAT_SNIFF_ENABLE_BIT 4
#define FEAT_CHANGE_SENSITIVITY_RELATIVE (1 << FEAT_CHANGE_SENSITIVITY_MODE_BIT)
#define FEAT_CHANGE_SENSITIVITY_ABSOLUTE (0 << FEAT_CHANGE_SENSITIVITY_MODE_BIT)
#define FEAT_CHANGE_SENSITIVITY_ENABLED (1 << FEAT_CHANGE_SENSITIVITY_ENABLE_BIT)
#define FEAT_CHANGE_SENSITIVITY_DISABLED (0 << FEAT_CHANGE_SENSITIVITY_ENABLE_BIT)
#define FEAT_WAKE_ENABLED (1 << FEAT_WAKEUP_ENABLE_BIT)
#define FEAT_WAKE_DISABLED (0 << FEAT_WAKEUP_ENABLE_BIT)
#define FEAT_ALWAYS_ON_ENABLED (1 << FEAT_ALWAYS_ON_ENABLE_BIT)
#define FEAT_ALWAYS_ON_DISABLED (0 << FEAT_ALWAYS_ON_ENABLE_BIT)
#define FEAT_SNIFF_ENABLED (1 << FEAT_SNIFF_ENABLE_BIT)
#define FEAT_SNIFF_DISABLED (0 << FEAT_SNIFF_ENABLE_BIT)

// SENSORHUB_SET_FEATURE_CMD
#define SENSORHUB_SET_FEATURE_CMD (0xFD)
typedef PACKED_STRUCT
{
    uint8_t reportId;        // 0xFD
    uint8_t featureReportId; // sensor id
    uint8_t flags;           // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
}
SetFeatureReport_t;

// Lengths of reports by report id.
static const sh2_ReportLen_t sh2ReportLens[] = {
    // Sensor reports
    {.id = SH2_ACCELEROMETER, .len = 10},
    {.id = SH2_GYROSCOPE_CALIBRATED, .len = 10},
    {.id = SH2_MAGNETIC_FIELD_CALIBRATED, .len = 10},
    {.id = SH2_LINEAR_ACCELERATION, .len = 10},
    {.id = SH2_ROTATION_VECTOR, .len = 14},
    {.id = SH2_GRAVITY, .len = 10},
    {.id = SH2_GYROSCOPE_UNCALIBRATED, .len = 16},
    {.id = SH2_GAME_ROTATION_VECTOR, .len = 12},
    {.id = SH2_GEOMAGNETIC_ROTATION_VECTOR, .len = 14},
    {.id = SH2_PRESSURE, .len = 8},
    {.id = SH2_AMBIENT_LIGHT, .len = 8},
    {.id = SH2_HUMIDITY, .len = 6},
    {.id = SH2_PROXIMITY, .len = 6},
    {.id = SH2_TEMPERATURE, .len = 6},
    {.id = SH2_MAGNETIC_FIELD_UNCALIBRATED, .len = 16},
    {.id = SH2_TAP_DETECTOR, .len = 5},
    {.id = SH2_STEP_COUNTER, .len = 12},
    {.id = SH2_SIGNIFICANT_MOTION, .len = 6},
    {.id = SH2_STABILITY_CLASSIFIER, .len = 6},
    {.id = SH2_RAW_ACCELEROMETER, .len = 16},
    {.id = SH2_RAW_GYROSCOPE, .len = 16},
    {.id = SH2_RAW_MAGNETOMETER, .len = 16},
    {.id = SH2_STEP_DETECTOR, .len = 8},
    {.id = SH2_SHAKE_DETECTOR, .len = 6},
    {.id = SH2_FLIP_DETECTOR, .len = 6},
    {.id = SH2_PICKUP_DETECTOR, .len = 8},
    {.id = SH2_STABILITY_DETECTOR, .len = 6},
    {.id = SH2_PERSONAL_ACTIVITY_CLASSIFIER, .len = 16},
    {.id = SH2_SLEEP_DETECTOR, .len = 6},
    {.id = SH2_TILT_DETECTOR, .len = 6},
    {.id = SH2_POCKET_DETECTOR, .len = 6},
    {.id = SH2_CIRCLE_DETECTOR, .len = 6},
    {.id = SH2_HEART_RATE_MONITOR, .len = 6},
    {.id = SH2_ARVR_STABILIZED_RV, .len = 14},
    {.id = SH2_ARVR_STABILIZED_GRV, .len = 12},
    {.id = SH2_GYRO_INTEGRATED_RV, .len = 14},
    {.id = SH2_IZRO_MOTION_REQUEST, .len = 6},
    {.id = SH2_RAW_OPTICAL_FLOW, .len = 24},
    {.id = SH2_DEAD_RECKONING_POSE, .len = 60},
    {.id = SH2_WHEEL_ENCODER, .len = 12},

    // Other response types
    {.id = SENSORHUB_FLUSH_COMPLETED, .len = 2},
    {.id = SENSORHUB_COMMAND_RESP, .len = 16},
    {.id = SENSORHUB_FRS_READ_RESP, .len = 16},
    {.id = SENSORHUB_FRS_WRITE_RESP, .len = 4},
    {.id = SENSORHUB_PROD_ID_RESP, .len = 16},
    {.id = SENSORHUB_TIMESTAMP_REBASE, .len = 5},
    {.id = SENSORHUB_BASE_TIMESTAMP_REF, .len = 5},
    {.id = SENSORHUB_GET_FEATURE_RESP, .len = 17},
};

// ------------------------------------------------------------------------

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
    txBuf[2] = 2; // CHAN_SENSORHUB_CONTROL;
    txBuf[3] = 0;
    memcpy(&txBuf[4], &report, 17);
    // printk('write to spi');
    spi_write_msg(17 + 4, txBuf, rxBuf);
    spi_release(spi_dev, &spi_cfg);
    spiState = SPI_IDLE;
    // k_sleep(K_MSEC(1U));
    // waken(false);
}

// Configure one sensor to produce periodic reports
static void startReports()
{
    // printk('sending config\n');
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
            // SH2_ARVR_STABILIZED_RV,
            // SH2_ARVR_STABILIZED_GRV,
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
    config.reportInterval_us = 40000; // microseconds (25 Hz)
    // config.reportInterval_us = 10000; // microseconds (100 Hz)
    // config.reportInterval_us = 2500;   // microseconds (400 Hz)
    // config.reportInterval_us = 1000;   // microseconds (1000 Hz)

    for (int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorId = enabledSensors[n];
        status = sh2_setSensorConfig(sensorId, &config);
        if (status != 0)
        {
            printk("Error while enabling sensor %d\n", sensorId);
        }
    }
}

// ------------------------------------------------------------------------
// Get FRS.

static void ceva_fsmSH2_dummy()
{
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
    // enable interrupts
    enableInts();
    printk("ceva_fsmSH2 Dummy done\n");
}

void ceva_fsmSH2_Open()
{
    ceva_fsmSH2_dummy();
}

static void fsmProcessInit()
{
    /* extract channel id from packet */
    int channel_id = rxBuf[2];
    switch (channel_id)
    {
    case CHAN_EXECUTABLE_DEVICE:
        if (rxBuf[4] == 0x01)
        {
            printk("FSM is ready\n");
            resetComplete = true;
            /* need to send start Report*/
            gpio_pin_configure_dt(&fsmwaken, GPIO_OUTPUT);
            waken(false);
            wakenComplete = false;
            spiState = SPI_WRITE;
            // k_usleep(1000);
            // startReports();
        }
    case CHAN_SENSORHUB_CONTROL:
    {
        // printk('chan sensorhub control\n');
    }
    /* code */
    break;
    default:
        break;
    }
}

// Produce 64-bit microsecond timestamp for a sensor event
static uint64_t touSTimestamp(uint32_t hostInt, int32_t referenceDelta, uint16_t delay)
{
    static uint32_t lastHostInt = 0;
    static uint32_t rollovers = 0;
    uint64_t timestamp;

    // Count times hostInt timestamps rolled over to produce upper bits
    if (hostInt < lastHostInt)
    {
        rollovers++;
    }
    lastHostInt = hostInt;

    timestamp = ((uint64_t)rollovers << 32);
    timestamp += hostInt + (referenceDelta + delay) * 100;

    return timestamp;
}

// Print a sensor event to the console
static void printEvent(const sh2_SensorEvent_t *event)
{
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float r, i, j, k, acc_deg, x, y, z;
    float t;
    static int skip = 0;

    rc = sh2_decodeSensorEvent(&value, event);
    if (rc != SH2_OK)
    {
        printk("Error decoding sensor event: %d\n", rc);
        return;
    }

    t = value.timestamp / 1000000.0; // time in seconds.
    printk("value sensor id %d\n", value.sensorId);
    switch (value.sensorId)
    {
    case SH2_RAW_ACCELEROMETER:
        printk("%8.4f Raw acc: %d %d %d\n",
               t,
               value.un.rawAccelerometer.x,
               value.un.rawAccelerometer.y,
               value.un.rawAccelerometer.z);
        break;

    case SH2_ACCELEROMETER:
        printk("%8.4f Acc: %f %f %f\n",
               t,
               value.un.accelerometer.x,
               value.un.accelerometer.y,
               value.un.accelerometer.z);
        break;

    case SH2_RAW_GYROSCOPE:
        printk("%8.4f Raw gyro: x:%d y:%d z:%d temp:%d time_us:%d\n",
               t,
               value.un.rawGyroscope.x,
               value.un.rawGyroscope.y,
               value.un.rawGyroscope.z,
               value.un.rawGyroscope.temperature,
               value.un.rawGyroscope.timestamp);
        break;

    case SH2_ROTATION_VECTOR:
        r = value.un.rotationVector.real;
        i = value.un.rotationVector.i;
        j = value.un.rotationVector.j;
        k = value.un.rotationVector.k;
        acc_deg = scaleRadToDeg *
                  value.un.rotationVector.accuracy;
        printk("%8.4f Rotation Vector: "
               "r:%0.6f i:%0.6f j:%0.6f k:%0.6f (acc: %0.6f deg)\n",
               t,
               r, i, j, k, acc_deg);
        break;
    case SH2_GAME_ROTATION_VECTOR:
        r = value.un.gameRotationVector.real;
        i = value.un.gameRotationVector.i;
        j = value.un.gameRotationVector.j;
        k = value.un.gameRotationVector.k;
        printf("%8.4f GRV: "
               "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\n",
               t,
               r, i, j, k);
        break;
    case SH2_GYROSCOPE_CALIBRATED:
        x = value.un.gyroscope.x;
        y = value.un.gyroscope.y;
        z = value.un.gyroscope.z;
        printk("%8.4f GYRO: "
               "x:%0.6f y:%0.6f z:%0.6f\n",
               t,
               x, y, z);
        break;
    case SH2_GYROSCOPE_UNCALIBRATED:
        x = value.un.gyroscopeUncal.x;
        y = value.un.gyroscopeUncal.y;
        z = value.un.gyroscopeUncal.z;
        printk("%8.4f GYRO_UNCAL: "
               "x:%0.6f y:%0.6f z:%0.6f\n",
               t,
               x, y, z);
        break;
    case SH2_GYRO_INTEGRATED_RV:
        // These come at 1kHz, too fast to print all of them.
        // So only print every 10th one
        skip++;
        if (skip == 10)
        {
            skip = 0;
            r = value.un.gyroIntegratedRV.real;
            i = value.un.gyroIntegratedRV.i;
            j = value.un.gyroIntegratedRV.j;
            k = value.un.gyroIntegratedRV.k;
            x = value.un.gyroIntegratedRV.angVelX;
            y = value.un.gyroIntegratedRV.angVelY;
            z = value.un.gyroIntegratedRV.angVelZ;
            printk("%8.4f Gyro Integrated RV: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f x:%0.6f y:%0.6f z:%0.6f\n",
                   t,
                   r, i, j, k,
                   x, y, z);
        }
        break;
    case SH2_IZRO_MOTION_REQUEST:
        printk("IZRO Request: intent:%d, request:%d\n",
               value.un.izroRequest.intent,
               value.un.izroRequest.request);
        break;
    case SH2_SHAKE_DETECTOR:
        printk("Shake Axis: %c%c%c\n",
               (value.un.shakeDetector.shake & SHAKE_X) ? 'X' : '.',
               (value.un.shakeDetector.shake & SHAKE_Y) ? 'Y' : '.',
               (value.un.shakeDetector.shake & SHAKE_Z) ? 'Z' : '.');

        break;
    default:
        printk("Unknown sensor: %d\n", value.sensorId);
        break;
    }
}

static uint8_t getReportLen(uint8_t reportId)
{
    for (unsigned n = 0; n < ARRAY_LEN(sh2ReportLens); n++)
    {
        if (sh2ReportLens[n].id == reportId)
        {
            return sh2ReportLens[n].len;
        }
    }

    return 0;
}

static void SHInputHandler(uint8_t *payload, uint16_t len, uint32_t timestamp)
{
    sh2_SensorEvent_t event;
    uint16_t cursor = 0;

    int32_t referenceDelta = 0;

    while (cursor < len)
    {
        // Get next report id
        uint8_t reportId = payload[cursor];

        // Determine report length
        uint8_t reportLen = getReportLen(reportId);
        if (reportLen == 0)
        {
            // An unrecognized report id
            // pSh2->unknownReportIds++;
            printk("Unknown report id %d\n", reportId);
            return;
        }
        else
        {
            if (reportId == SENSORHUB_BASE_TIMESTAMP_REF)
            {
                const BaseTimestampRef_t *rpt = (const BaseTimestampRef_t *)(payload + cursor);

                // store base timestamp reference
                referenceDelta = -rpt->timebase;
            }
            else if (reportId == SENSORHUB_TIMESTAMP_REBASE)
            {
                const TimestampRebase_t *rpt = (const TimestampRebase_t *)(payload + cursor);

                referenceDelta += rpt->timebase;
            }
            else if (reportId == SENSORHUB_FLUSH_COMPLETED)
            {
                // Route this as if it arrived on command channel.
                printk("Flush completed\n");
                // opRx(pSh2, payload+cursor, reportLen);
            }
            else
            {
                // Sensor event.  Call callback
                uint8_t *pReport = payload + cursor;
                uint16_t delay = ((pReport[2] & 0xFC) << 6) + pReport[3];
                event.timestamp_uS = touSTimestamp(timestamp, referenceDelta, delay);
                event.delay_uS = (referenceDelta + delay) * 100;
                event.reportId = reportId;
                memcpy(event.report, pReport, reportLen);
                event.len = reportLen;
                printk("next packet info : %x, len %d\n", payload[cursor], event.len);
                printEvent(&event);
                // if (pSh2->sensorCallback != 0) {
                // pSh2->sensorCallback(pSh2->sensorCookie, &event);
                //}
            }

            // Move to next report in the payload
            cursor += reportLen;
        }
    }
}

void FSM_thread(void)
{
    for (;;)
    {
        if (interrupt_flag)
        {
            if (spiState == SPI_WRITE)
            {
                if (!wakenComplete)
                {
                    waken(true);
                    wakenComplete = true;
                }
                // printk('SPI_WRITE Sent report\n');
                startReports();
            }
            else
            {
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
                spi_write_msg(packet_length - packet_hdr_length, txBuf, rxBuf + 4);
                // printk("Packet data[0]: %x\n", rxBuf[4]);
                spi_release(spi_dev, &spi_cfg);
                if (resetComplete)
                {
                    printk("Event received: rxBuf[4]: %x\n", rxBuf[4]);
                    SHInputHandler(&rxBuf[4], packet_length - 4, k_uptime_get());
                    // printEvent(&rxBuf[4]);
                }
                else
                {
                    fsmProcessInit();
                }
            }
            interrupt_flag = 0;
        }
        else
        {
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
    // rxTimestamp_us = k_uptime_get_32() * 1000;
    // inReset = false;
    // rxReady = true;
    //  Start read if possible
    // spiActivate();
    //  printk("FSM has an interrup triggered at %" PRIu32 "\n", k_cycle_get_32());
}
#include "../../includes.h"
#include "sh2/sh2.h"
#include "sh2/sh2_SensorValue.h"
#include "../../drivers/SPIM/spim_local.h"
#include "sh2/sh2_err.h"
#include "sh2/sh2_util.h"

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

#if defined(_MSC_VER)
#define PACKED_STRUCT struct
#pragma pack(push, 1)
#elif defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__((packed))
#else 
#define PACKED_STRUCT __packed struct
#endif


// ------------------------------------------------------------------------
// Private type definitions

#define CHAN_EXECUTABLE_DEVICE    (1)
#define CHAN_SENSORHUB_CONTROL    (2)
#define CHAN_SENSORHUB_INPUT      (3)
#define CHAN_SENSORHUB_INPUT_WAKE (4)
#define CHAN_SENSORHUB_INPUT_GIRV (5)

// executable/device channel responses
#define EXECUTABLE_DEVICE_CMD_RESET (1)
#define EXECUTABLE_DEVICE_CMD_ON    (2)
#define EXECUTABLE_DEVICE_CMD_SLEEP (3)

// executable/device channel responses
#define EXECUTABLE_DEVICE_RESP_RESET_COMPLETE (1)

// Tags for sensorhub app advertisements.
#define TAG_SH2_VERSION (0x80)
#define TAG_SH2_REPORT_LENGTHS (0x81)

// Max length of sensorhub version string.
#define MAX_VER_LEN (16)

// Max number of report ids supported
#define SH2_MAX_REPORT_IDS (64)

#if defined(_MSC_VER)
#define PACKED_STRUCT struct
#pragma pack(push, 1)
#elif defined(__GNUC__)
#define PACKED_STRUCT struct __attribute__((packed))
#else 
#define PACKED_STRUCT __packed struct
#endif

#define ADVERT_TIMEOUT_US (200000)

// Command and Subcommand values
#define SH2_CMD_ERRORS                 1
#define SH2_CMD_COUNTS                 2
#define     SH2_COUNTS_GET_COUNTS          0
#define     SH2_COUNTS_CLEAR_COUNTS        1
#define SH2_CMD_TARE                   3
#define     SH2_TARE_TARE_NOW              0
#define     SH2_TARE_PERSIST_TARE          1
#define     SH2_TARE_SET_REORIENTATION     2
#define SH2_CMD_INITIALIZE             4
#define     SH2_INIT_SYSTEM                1
#define     SH2_INIT_UNSOLICITED           0x80
// #define SH2_CMD_FRS                    5 /* Depreciated */
#define SH2_CMD_DCD                    6
#define SH2_CMD_ME_CAL                 7
#define SH2_CMD_DCD_SAVE               9
#define SH2_CMD_GET_OSC_TYPE           0x0A
#define SH2_CMD_CLEAR_DCD_AND_RESET    0x0B
#define SH2_CMD_CAL                    0x0C
#define     SH2_CAL_START                   0
#define     SH2_CAL_FINISH                  1
#define SH2_CMD_BOOTLOADER             0x0D     /* SH-2 Reference Manual 6.4.12 */
#define     SH2_BL_MODE_REQ                 0
#define     SH2_BL_STATUS_REQ               1
#define SH2_CMD_INTERACTIVE_ZRO        0x0E     /* SH-2 Reference Manual 6.4.13 */
#define SH2_CMD_WHEEL_REQ              0x0F
#define SH2_CMD_DR_CAL_SAVE            0x10

// SENSORHUB_COMMAND_REQ
#define SENSORHUB_COMMAND_REQ        (0xF2)
#define COMMAND_PARAMS (9)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t p[COMMAND_PARAMS];
} CommandReq_t;

// SENSORHUB_COMMAND_RESP
#define SENSORHUB_COMMAND_RESP       (0xF1)
#define RESPONSE_VALUES (11)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t seq;
    uint8_t command;
    uint8_t commandSeq;
    uint8_t respSeq;
    uint8_t r[RESPONSE_VALUES];
} CommandResp_t;

// SENSORHUB_PROD_ID_REQ
#define SENSORHUB_PROD_ID_REQ        (0xF9)
typedef PACKED_STRUCT {
    uint8_t reportId;  
    uint8_t reserved;
} ProdIdReq_t;

// SENSORHUB_PROD_ID_RESP
#define SENSORHUB_PROD_ID_RESP       (0xF8)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t resetCause;
    uint8_t swVerMajor;
    uint8_t swVerMinor;
    uint32_t swPartNumber;
    uint32_t swBuildNumber;
    uint16_t swVerPatch;
    uint8_t reserved0;
    uint8_t reserved1;
} ProdIdResp_t;

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


// GET_FEATURE_REQ
#define SENSORHUB_GET_FEATURE_REQ    (0xFE)
typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;
} GetFeatureReq_t;

// SENSORHUB_GET_FEATURE_RESP
#define SENSORHUB_GET_FEATURE_RESP   (0xFC)
typedef PACKED_STRUCT{
    uint8_t reportId;
    uint8_t featureReportId;      // sensor id
    uint8_t flags;                // FEAT_... values
    uint16_t changeSensitivity;
    uint32_t reportInterval_uS;
    uint32_t batchInterval_uS;
    uint32_t sensorSpecific;
} GetFeatureResp_t;


typedef struct sh2_s sh2_t;

typedef int (sh2_OpStart_t)(sh2_t *pSh2);
typedef void (sh2_OpRx_t)(sh2_t *pSh2, const uint8_t *payload, uint16_t len);

typedef struct sh2_Op_s {
    uint32_t timeout_us;
    sh2_OpStart_t *start;
    sh2_OpRx_t *rx;
} sh2_Op_t;

// Parameters and state information for the operation in progress
typedef union {
    struct {
        CommandReq_t req;
    } sendCmd;
    struct {
        sh2_ProductIds_t *pProdIds;
        uint8_t nextEntry;
        uint8_t expectedEntries;
    } getProdIds;
    struct {
        sh2_SensorConfig_t *pConfig;
        sh2_SensorId_t sensorId;
    } getSensorConfig;
    struct {
        const sh2_SensorConfig_t *pConfig;
        sh2_SensorId_t sensorId;
    } setSensorConfig;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t *pWords;
        uint16_t nextOffset;
    } getFrs;
    struct {
        uint16_t frsType;
        uint32_t *pData;
        uint16_t words;
        uint16_t offset;
    } setFrs;
    struct {
        uint8_t severity;
        sh2_ErrorRecord_t *pErrors;
        uint16_t *pNumErrors;
        uint16_t errsRead;
    } getErrors;
    struct {
        sh2_SensorId_t sensorId;
        sh2_Counts_t *pCounts;
    } getCounts;
    struct {
        uint8_t sensors;
    } calConfig;
    struct {
        uint8_t *pSensors;
    } getCalConfig;
    struct {
        sh2_SensorId_t sensorId;
    } forceFlush;
    struct {
        sh2_OscType_t *pOscType;
    } getOscType;
    struct {
        uint32_t interval_us;
    } startCal;
    struct {
        sh2_CalStatus_t status;
    } finishCal;
    struct {
        uint8_t wheelIndex;
        uint32_t timestamp;
        int16_t wheelData;
        uint8_t dataType;
    } wheelRequest;
} sh2_OpData_t;

// Max length of an FRS record, words.
#define MAX_FRS_WORDS (72)

struct sh2_s {
    // Pointer to the SHTP HAL
    sh2_Hal_t *pHal;

    // associated SHTP instance
    void *pShtp;
    
    volatile bool resetComplete;
    char version[MAX_VER_LEN+1];

    // Multi-step operation support
    const sh2_Op_t *pOp;
    int opStatus;
    sh2_OpData_t opData;
    uint8_t lastCmdId;
    uint8_t cmdSeq;
    uint8_t nextCmdSeq;
    
    // Event callback and it's cookie
    sh2_EventCallback_t *eventCallback;
    void * eventCookie;

    // Sensor callback and it's cookie
    sh2_SensorCallback_t *sensorCallback;
    void * sensorCookie;

    // Storage space for reading sensor metadata
    uint32_t frsData[MAX_FRS_WORDS];
    uint16_t frsDataLen;

    // Stats
    uint32_t execBadPayload;
    uint32_t emptyPayloads;
    uint32_t unknownReportIds;

};

#define SENSORHUB_BASE_TIMESTAMP_REF (0xFB)
typedef PACKED_STRUCT {
    uint8_t reportId;
    int32_t timebase;
} BaseTimestampRef_t;

#define SENSORHUB_TIMESTAMP_REBASE   (0xFA)
typedef PACKED_STRUCT {
    uint8_t reportId;
    int32_t timebase;
} TimestampRebase_t;

// SENSORHUB_FORCE_SENSOR_FLUSH
#define SENSORHUB_FORCE_SENSOR_FLUSH (0xF0)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushReq_t;

// SENSORHUB_FLUSH_COMPLETED    
#define SENSORHUB_FLUSH_COMPLETED    (0xEF)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t sensorId;
} ForceFlushResp_t;

typedef struct sh2_ReportLen_s {
    uint8_t id;
    uint8_t len;
} sh2_ReportLen_t;

// SENSORHUB_FRS_WRITE_REQ
#define SENSORHUB_FRS_WRITE_REQ      (0xF7)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t length;
    uint16_t frsType;
} FrsWriteReq_t;

// SENSORHUB_FRS_WRITE_DATA_REQ
#define SENSORHUB_FRS_WRITE_DATA_REQ (0xF6)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t offset;
    uint32_t data0;
    uint32_t data1;
} FrsWriteDataReq_t;

// FRS write status values
#define FRS_WRITE_STATUS_RECEIVED (0)
#define FRS_WRITE_STATUS_UNRECOGNIZED_FRS_TYPE (1)
#define FRS_WRITE_STATUS_BUSY (2)
#define FRS_WRITE_STATUS_WRITE_COMPLETED (3)
#define FRS_WRITE_STATUS_READY (4)
#define FRS_WRITE_STATUS_FAILED (5)
#define FRS_WRITE_STATUS_NOT_READY (6) // data received when not in write mode
#define FRS_WRITE_STATUS_INVALID_LENGTH (7)
#define FRS_WRITE_STATUS_RECORD_VALID (8)
#define FRS_WRITE_STATUS_INVALID_RECORD (9)
#define FRS_WRITE_STATUS_DEVICE_ERROR (10)
#define FRS_WRITE_STATUS_READ_ONLY (11)

// SENSORHUB_FRS_WRITE_RESP
#define SENSORHUB_FRS_WRITE_RESP     (0xF5)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t status;
    uint16_t wordOffset;
} FrsWriteResp_t;

// RESP_FRS_READ_REQ
#define SENSORHUB_FRS_READ_REQ       (0xF4)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t reserved;
    uint16_t readOffset;
    uint16_t frsType;
    uint16_t blockSize;
} FrsReadReq_t;

// Get Datalen portion of len_status field
#define FRS_READ_DATALEN(x) ((x >> 4) & 0x0F)

// Get status portion of len_status field
#define FRS_READ_STATUS(x) ((x) & 0x0F)

// Status values
#define FRS_READ_STATUS_NO_ERROR                        0
#define FRS_READ_STATUS_UNRECOGNIZED_FRS_TYPE           1
#define FRS_READ_STATUS_BUSY                            2
#define FRS_READ_STATUS_READ_RECORD_COMPLETED           3
#define FRS_READ_STATUS_OFFSET_OUT_OF_RANGE             4
#define FRS_READ_STATUS_RECORD_EMPTY                    5
#define FRS_READ_STATUS_READ_BLOCK_COMPLETED            6
#define FRS_READ_STATUS_READ_BLOCK_AND_RECORD_COMPLETED 7
#define FRS_READ_STATUS_DEVICE_ERROR                    8

// SENSORHUB_FRS_READ_RESP
#define SENSORHUB_FRS_READ_RESP      (0xF3)
typedef PACKED_STRUCT {
    uint8_t reportId;
    uint8_t len_status;  // See FRS_READ... macros above
    uint16_t wordOffset;
    uint32_t data0;
    uint32_t data1;
    uint16_t frsType;
    uint8_t reserved0;
    uint8_t reserved1;
} FrsReadResp_t;

// ------------------------------------------------------------------------
// Private data

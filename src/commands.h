#include "includes.h"
#include "drivers/PWM/pwm_local.h"

extern uint8_t *ble_received_data;
extern uint16_t ble_received_data_len;
extern bool commandReceived;
extern bool bleISRComplete;

void process_command_thread(void);
void process_command(void);

#define HARD_STOP 0x01
#define FETCH_RAW_READINGS 0x02
#define FETCH_GYRP_READINGS 0x03
#define FETCH_ACCL_READINGS 0x04
#define TEST_MOTOR_1 0x05
#define TEST_MOTOR_2 0x06
#define TEST_MOTOR_3 0x07
#define TEST_MOTOR_4 0x08
#define HOVER 0x09
#define MOVE_RIGHT 0x0A
#define MOVE_LEFT 0x0B
#define MOVE_FORWARD 0x0C
#define MOVE_BACKWARD 0x0D
#define LOWER 0x0E
#define STOP_SENSOR_STREAN 0x0F
#define TOGGLE_LOGGING 0x2f
#define TEST_MODE_ON 0x3f
#define TEST_MODE_OFF 0x4f
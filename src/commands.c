#include "commands.h"

uint8_t *ble_received_data;
uint16_t ble_received_data_len;
bool commandReceived = false;
bool bleISRComplete = false;
static bool testMode = false;

void process_command_thread(void)
{
    printf('process_command_thread \n');
    while (1)
    {
        printk('processCommands \n');
        if (commandReceived)
        {
            commandReceived = false;
            // processCommand(ble_received_data, blen_reciveed_data_len);
            printk('processCommand len %d \n', ble_received_data_len);
        }
        k_msleep(1000);
        k_yield();
    }
}

static uint8_t getPWM(uint8_t *token)
{
    int count = 0;
    while (token != NULL)
    {
        if (count)
        {
            printf("The pwm speed is 0x%x or %d% \n", atoi(token), atoi(token));
            return atoi(token);
        }
        count++;
        token = strtok(NULL, " ");
    }
}

void process_command(void)
{
    printk("Process command thread\n");
    while (1)
    {
        if (commandReceived && bleISRComplete)
        {
            printf("processCommand len %d \n", ble_received_data_len);
            commandReceived = false;
            bleISRComplete = false;
            if (ble_received_data)
            {
                printf("the data received is %s \n", ble_received_data);
                char *token = strtok(ble_received_data, " ");
                switch (atoi(token))
                {
                case HARD_STOP:
                    printf("the data received is 0x01 - Hard Stop \n");
                    setDuty(pwm_led0, MAX_PERIOD, (MAX_PERIOD / 10U));
                    setDuty(pwm_led1, MAX_PERIOD, (MAX_PERIOD / 10U));
                    setDuty(pwm_led2, MAX_PERIOD, (MAX_PERIOD / 10U));
                    setDuty(pwm_led3, MAX_PERIOD, (MAX_PERIOD / 10U));
                    break;
                case TEST_MODE_ON:
                    printf("Test mode on");
                    testMode = true;
                    break;
                case TEST_MODE_OFF:
                    printf("Test mode off");
                    setDuty(pwm_led0, MAX_PERIOD, MAX_PERIOD / 20U);
                    setDuty(pwm_led1, MAX_PERIOD, MAX_PERIOD / 20U);
                    setDuty(pwm_led2, MAX_PERIOD, MAX_PERIOD / 20U);
                    setDuty(pwm_led3, MAX_PERIOD, MAX_PERIOD / 20U);
                    testMode = false;
                    break;
                case TEST_MOTOR_1:
                    if (testMode)
                    {
                        setDuty(pwm_led0, MAX_PERIOD, (MAX_PERIOD * getPWM(token)) / 100U);
                    }
                    break;
                case TEST_MOTOR_2:
                    if (testMode)
                    {
                        setDuty(pwm_led1, MAX_PERIOD, (MAX_PERIOD * getPWM(token)) / 100U);
                    }
                    break;
                case TEST_MOTOR_3:
                    if (testMode)
                    {
                        setDuty(pwm_led2, MAX_PERIOD, (MAX_PERIOD * getPWM(token)) / 100U);
                    }
                    break;
                case TEST_MOTOR_4:
                    if (testMode)
                    {
                        setDuty(pwm_led3, MAX_PERIOD, (MAX_PERIOD * getPWM(token)) / 100U);
                    }
                    break;
                case FETCH_RAW_READINGS:
                    printf("the data received is 0x02 - Fetch Raw Readings \n");
                    break;
                default:
                    break;
                }
            }
            // processCommand(ble_received_data, blen_reciveed_data_len);
        }
        // k_msleep(1000);
        k_yield();
    }
}
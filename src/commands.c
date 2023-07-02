#include "commands.h"

uint8_t *ble_received_data;
uint16_t ble_received_data_len;
bool commandReceived = false;
bool bleISRComplete = false;

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

void thread0(void)
{
    printk("thread0\n");
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
                    break;
                case TEST_MOTOR_1:
                    while (token != NULL)
                    {
                        printf("The pwm speed is 0x%x or %d% \n", atoi(token), atoi(token));
                        token = strtok(NULL, " ");
                    }
                case TEST_MOTOR_2:
                    while (token != NULL)
                    {
                        printf("The pwm speed is 0x%x or %d% \n", atoi(token), atoi(token));
                        token = strtok(NULL, " ");
                    }
                case TEST_MOTOR_3:
                    while (token != NULL)
                    {
                        printf("The pwm speed is 0x%x or %d% \n", atoi(token), atoi(token));
                        token = strtok(NULL, " ");
                    }
                case TEST_MOTOR_4:
                    while (token != NULL)
                    {
                        printf("The pwm speed is 0x%x or %d% \n", atoi(token), atoi(token));
                        token = strtok(NULL, " ");
                    }
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
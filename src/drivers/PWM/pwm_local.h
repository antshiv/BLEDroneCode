#include <zephyr/drivers/pwm.h>

extern const struct pwm_dt_spec pwm_led0;
extern const struct pwm_dt_spec pwm_led1;
extern const struct pwm_dt_spec pwm_led2;
extern const struct pwm_dt_spec pwm_led3;
/*
 * For the ESC, the duty cycle is set to 8000, which is the minimum value
 * for the ESC to recognize the signal. The frequency is set to 50Hz, which
 * is the standard frequency for ESC.
 *
 */
#define PWM_CH0_DUTY 8000
#define PWM_CH1_DUTY 8000
#define PWM_CH2_DUTY 8000
#define PWM_CH3_DUTY 8000


#define FREQUENCY 50U

#define MIN_PERIOD PWM_SEC(1U) / 128U
#define MAX_PERIOD PWM_SEC(1U) / FREQUENCY


void pwm_check_ready();
void setDutyTest(const struct pwm_dt_spec pwm, uint16_t max_period);
void setDuty(const struct pwm_dt_spec pwm, uint32_t period, uint32_t duty_cycle);

void calibratePWM(int *period);
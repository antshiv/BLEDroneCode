#include "../../includes.h"
#include "pwm_local.h"

/*
 * Initialize the device driver for PWM_0, PWM_1, PWM_2, PWM_3
 * PWM signal is required to control the ESC of the drone.
 */
const struct pwm_dt_spec pwm_led0 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
const struct pwm_dt_spec pwm_led1 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led1));
const struct pwm_dt_spec pwm_led2 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led2));
const struct pwm_dt_spec pwm_led3 = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led3));

uint16_t pwm_seq[4] = {PWM_CH0_DUTY, PWM_CH1_DUTY, PWM_CH2_DUTY, PWM_CH3_DUTY};

void pwm_check_ready()
{
    if (!device_is_ready(pwm_led0.dev))
    {
        printk("Error: PWM device %s is not ready\n",
               pwm_led0.dev->name);
        return;
    }
    if (!device_is_ready(pwm_led1.dev))
    {
        printk("Error: PWM device %s is not ready\n",
               pwm_led1.dev->name);
        return;
    }
    if (!device_is_ready(pwm_led2.dev))
    {
        printk("Error: PWM device %s is not ready\n",
               pwm_led2.dev->name);
        return;
    }
    if (!device_is_ready(pwm_led3.dev))
    {
        printk("Error: PWM device %s is not ready\n",
               pwm_led3.dev->name);
        return;
    }
}

void setDutyTest(const struct pwm_dt_spec pwm, uint16_t max_period)
{
    /*
     * In case the default MAX_PERIOD value cannot be set for
     * some PWM hardware, decrease its value until it can
     *
     * Keep its value at least MIN_PERIOD * 4 to make sure
     * the sample changes frequency at least once.
     */
    printk("Calibrating for channel %d...\n", pwm.channel);
    max_period = MAX_PERIOD;
    while (pwm_set_dt(&pwm, max_period, max_period / 2U))
    {
        max_period /= 2U;
        if (max_period < (4U * MIN_PERIOD))
        {
            printk("Error: PWM device "
                   "does not support a period at least %lu\n",
                   4U * MIN_PERIOD);
            return;
        }
    }
}

void calibratePWM(int *period)
{
    /*
        if (!device_is_ready(dev_i2c.bus)) {
            printk("I2C bus %s is not ready!\n\r",dev_i2c.bus->name);
            return;
        }
    */
    /*
     * In case the default MAX_PERIOD value cannot be set for
     * some PWM hardware, decrease its value until it can.
     *
     * Keep its value at least MIN_PERIOD * 4 to make sure
     * the sample changes frequency at least once.
     */
    printk("Calibrating for channel %d...\n", pwm_led0.channel);
    int max_period = MAX_PERIOD;
    while (pwm_set_dt(&pwm_led0, max_period, max_period / 2U))
    {
        max_period /= 2U;
        if (max_period < (4U * MIN_PERIOD))
        {
            printk("Error: PWM device "
                   "does not support a period at least %lu\n",
                   4U * MIN_PERIOD);
            return;
        }
    }
    // setDuty(pwm_led0, max_period);
    setDutyTest(pwm_led1, max_period);
    setDutyTest(pwm_led2, max_period);
    setDutyTest(pwm_led3, max_period);

    printk("Done calibrating; maximum/minimum periods %u/%lu usec\n",
           max_period, MIN_PERIOD);

    *period = max_period;
}

void setDuty(const struct pwm_dt_spec pwm, uint32_t period, uint32_t duty_cycle)
{
    int ret;
    uint16_t duty = pwm_seq[pwm.channel];
    printk("Setting duty cycle to %u usec\n", duty);
    ret = pwm_set_dt(&pwm, period, duty_cycle);
    if (ret)
    {
        printk("Error %d: failed to set duty cycle\n", ret);
        return;
    }
    printk("PWM period %u usec\n", period);
}

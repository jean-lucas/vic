#include <wiringPi.h>
#include <unistd.h>

#include "motor_speed_controller.h"
#include "vic_hardware.h"

#define MOTOR_PIN BCM_PIN_18

void vichw_init_speed(void)
{
	pinMode(MOTOR_PIN, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(PWM_CLOCK);
	pwmSetRange(PWM_RANGE);
}

void vichw_set_speed(double speed)
{
	int pwm = DEFAULT_PWM;
	if (speed < 0.0) {
		pwm = 100;
	} else if (speed > 0.0) {
		pwm = DEFAULT_PWM + 50.0/3 * speed;
	} else {
		pwm = DEFAULT_PWM;
	}

	if (pwm > MAX_SPEED_PWN) pwm = MAX_SPEED_PWN;
	if (pwm < MIN_SPEED_PWM) pwm = MIN_SPEED_PWM;

	pwmWrite(MOTOR_PIN, pwm);
}


/* TODO: implement me */
double vichw_get_speed() {
	return 0;
}

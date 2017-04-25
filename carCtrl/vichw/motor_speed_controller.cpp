#include <pigpio.h>
#include <unistd.h>

#include "motor_speed_controller.h"
#include "vic_hardware.h"

#define MOTOR_PIN 18

void vichw_init_speed(void)
{
	gpioSetMode(MOTOR_PIN, PI_OUTPUT);
	vichw_set_speed(0);
}

void vichw_set_speed(double speed)
{
	int pwm = DEFAULT_PWM_SPDC;
	if (speed < 0.0) {
		pwm = 1000;
	} else if (speed > 0.0) {
		pwm = DEFAULT_PWM_SPDC + 500.0/3 * speed;
	} else {
		pwm = DEFAULT_PWM_SPDC;
	}

	if (pwm > MAX_SPEED_PWN) pwm = MAX_SPEED_PWN;
	if (pwm < MIN_SPEED_PWM) pwm = MIN_SPEED_PWM;

	gpioServo(MOTOR_PIN, pwm);
}


/* TODO: implement me */
double vichw_get_speed()
{
	return 0;
}

#include <pigpio.h>
#include <unistd.h>

#include "servo_controller.h"
#include "vic_hardware.h"

#define SERVO_PIN 13

void vichw_init_servo(void)
{
	gpioSetMode(SERVO_PIN, PI_OUTPUT);
	vichw_set_angle(0);
}

void vichw_set_angle(double angle)
{
	int pwm = DEFAULT_PWM_SRVO;
	if (angle < 0.0) {
		pwm = DEFAULT_PWM_SRVO + 300.0/45.0 * (-angle);
	} else if (angle > 0.0) {
		pwm = DEFAULT_PWM_SRVO - 300.0/45.0 * angle; //should be 400
	} else {
		pwm = DEFAULT_PWM_SRVO;
	}

	if (pwm > MAX_SERVO_PWN) pwm = MAX_SERVO_PWN;
	if (pwm < MIN_SERVO_PWM) pwm = MIN_SERVO_PWM;
	
	gpioServo(SERVO_PIN, pwm);
}

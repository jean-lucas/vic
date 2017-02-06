#include <wiringPi.h>
#include <unistd.h>

#include "servo_controller.h"
#include "vic_hardware.h"


void set_angle(double angle)
{
	int pwm = DEFAULT_PWM;
	if (angle < 0.0) {
		pwm = DEFAULT_PWM + 30.0/45.0 * (-angle);
	} else if (angle > 0.0) {
		pwm = DEFAULT_PWM - 40.0/45.0 * angle;
	} else {
		pwm = DEFAULT_PWM;
	}

	if (pwm > MAX_SERVO_PWN) pwm = MAX_SERVO_PWN;
	if (pwm < MIN_SERVO_PWM) pwm = MIN_SERVO_PWM;

	pinMode(BCM_PIN_13, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(PWM_CLOCK);
	pwmSetRange(PWM_RANGE);
	pwmWrite(BCM_PIN_13, pwm);
}

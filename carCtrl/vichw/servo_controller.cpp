#include <pigpio.h>
#include <unistd.h>

#include "servo_controller.h"
#include "vic_hardware.h"

#define SERVO_PIN 13

void vichw_init_servo(void) {
	gpioSetMode(SERVO_PIN, PI_OUTPUT);
	vichw_set_angle(DEFAULT_PWM);
}

void vichw_set_angle(double pwm) {

	if (pwm > MAX_SERVO_PWN) pwm = MAX_SERVO_PWN;
	if (pwm < MIN_SERVO_PWM) pwm = MIN_SERVO_PWM;
	
	gpioServo(SERVO_PIN, pwm);
}

#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "motor_speed_controller.h"

int main()
{
	wiringPiSetup();
	set_speed(0);
	sleep(1);
	set_speed(0.5);
	sleep(3);
	set_speed(0.7);
	sleep(3);
	set_speed(0.8);
	sleep(5);
	set_speed(-1);
	sleep(1);
	set_speed(0);

	return 0;
}

void set_speed(double speed)
{
	int pwm = 150;
	if (speed < 0.0) {
		pwm = 100;
	} else if (speed > 0.0) {
		pwm = 150.0 + 50.0/3 * speed;
	} else {
		pwm = 150;
	}

	if (pwm > 200) pwm = 200;
	if (pwm < 100) pwm = 100;

	pinMode(1, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(192);
	pwmSetRange(2000);
	pwmWrite(1, pwm);
}

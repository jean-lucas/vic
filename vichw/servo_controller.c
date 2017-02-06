#include <wiringPi.h>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "servo_controller.h"

int main()
{
	wiringPiSetup();
	set_angle(0);
	sleep(1);
	set_angle(-30);
	sleep(1);
	set_angle(30);

	return 0;
}

void set_angle(double angle)
{
	int pwm = 150;
	if (angle < 0.0) {
		pwm = 150.0 + 30.0/45.0 * (-angle);
	} else if (angle > 0.0) {
		pwm = 150.0 - 40.0/45.0 * angle;
	} else {
		pwm = 150;
	}

	if (pwm > 180) pwm = 180;
	if (pwm < 110) pwm = 110;

	pinMode(23, PWM_OUTPUT);
	pwmSetMode(PWM_MODE_MS);
	pwmSetClock(192);
	pwmSetRange(2000);
	pwmWrite(23, pwm);
}

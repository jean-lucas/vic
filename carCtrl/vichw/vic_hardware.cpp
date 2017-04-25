#include "vic_hardware.h"
#include <pigpio.h>
#include <stdio.h>
#include <unistd.h>
#include "servo_controller.h"
#include "motor_speed_controller.h"
#include "ultrasonic.h"
#include <iostream>
#include <fstream>

using namespace std;

/*  constants */
int DEFAULT_PWM_SPDC = 1500;
int DEFAULT_PWM_SRVO = 1500;


/* speed controller  constants */
const int MAX_SPEED_PWN = 2000;
const int MIN_SPEED_PWM = 1000;

/* servo controller  constants */
const int MAX_SERVO_PWN = 1800;
const int MIN_SERVO_PWM = 1100;

static void config()
{
	int offset = 0;
	ifstream f("/etc/vic.conf");
	if (f.is_open()) {
		f >> offset;
	}
	printf("offset = %d\n", offset);
	DEFAULT_PWM_SRVO += offset;
}

int vichw_init()
{
	gpioInitialise();
	config();
    vichw_init_speed();
    vichw_init_servo();
	vichw_init_ultrasonic();
	return 1;
}

void vichw_deinit(void) {
	vichw_set_speed(0);
	vichw_set_angle(0);
	sleep(1);
    gpioTerminate();
}
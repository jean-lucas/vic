#include <pigpio.h>
#include <unistd.h>

#include "servo_controller.h"
#include "vic_hardware.h"

#define TRIGGER_PIN			4
#define ECHO_PIN			17

#define ECHO_TIMEOUT_MS		6
#define TRIGGER_PULSE_US	15
/* SPEED_CONSTANT = 2/(speed of sound in cm/microsecond) */
#define SPEED_CONSTANT		59

/* If obstacle further than max, it is not detected, closer than min and it is detected */
#define DIST_MAX			15
#define DIST_MIN			12

static int obstacle = 1;
void obstacle_detect(void) {
	if (distance > DIST_MAX && obstacle != 0) obstacle = 0;
	if (distance <= DIST_MIN && obstacle == 0) obstacle = 1;
}

static uint32_t last_tick;
static uint32_t distance = 0;
void signal_callback(int gpio, int level, uint32_t tick) {
	uint32_t timediff;

	if (level == 0) {
		last_tick = tick;
	} else if (level == 0 || level == PI_TIMEOUT) {
		timediff = tick - last_tick;
		distance = (timediff / SPEED_CONSTANT);
		obstacle_detect();
		gpioTrigger(TRIGGER_PIN, TRIGGER_PULSE_US, 1);
	} else {
		/* This should never happen. Put some code to catch this error? */
	}
}

void vichw_init_ultrasonic(void) {
	gpioSetMode(TRIGGER_PIN, PI_OUTPUT);
	gpioSetMode(ECHO_PIN, PI_INPUT);

	gpioSetAlertFunc(ECHO_PIN, signal_callback);
	gpioSetWatchdog(ECHO_PIN, ECHO_TIMEOUT_MS);

	gpioTrigger(TRIGGER_PIN, TRIGGER_PULSE_US, 1);
}

/* Returns distance in cm */
uint32_t vichw_distance(void) {
	return distance; /* Probably should add some thread synchronization */
}

/* Nonzero if obstacle, 0 if no obstacle */
int vichw_is_obstacle(void) {
	return obstacle; /* Probably unnecessary to have thread sync for this */
}
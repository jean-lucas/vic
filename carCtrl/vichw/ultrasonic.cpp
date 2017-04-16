#include <pigpio.h>
#include <unistd.h>

#include "servo_controller.h"
#include "vic_hardware.h"

#define TRIGGER_PIN			4
#define ECHO_PIN			17

#define ECHO_TIMEOUT_US		6
#define TRIGGER_PULSE_US	15
/* SPEED_CONSTANT = 2/(speed of sound in cm/microsecond) */
#define SPEED_CONSTANT		59

static uint32_t last_tick;
static uint32_t distance = 0;
void signal_callback(int gpio, int level, uint32_t tick) {
	uint32_t timediff;

	if (level == 0) {
		last_tick = tick;
	} else if (level == 0 || level == PI_TIMEOUT) {
		timediff = tick - last_tick;
		distance = (timediff / SPEED_CONSTANT);
		gpioTrigger(TRIGGER_PIN, TRIGGER_PULSE_US, 1);
	} else {
		/* This should never happen. Put some code to catch this error? */
	}
}

void vichw_init_ultrasonic(void) {
	gpioSetMode(TRIGGER_PIN, PI_OUTPUT);
	gpioSetMode(ECHO_PIN, PI_INPUT);

	gpioSetAlertFunc(ECHO_PIN, signal_callback);
	gpioSetWatchdog(ECHO_PIN, ECHO_TIMEOUT_US);

	gpioTrigger(TRIGGER_PIN, TRIGGER_PULSE_US, 1);
}

/* Returns distance in cm */
uint32_t vichw_distance(void) {
	return distance; /* Probably should add some thread synchronization */
}
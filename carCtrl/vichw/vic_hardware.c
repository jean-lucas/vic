#include "vic_hardware.h"
#include "servo_controller.h"
#include "motor_speed_controller.h"

void vichw_init(void)
{
	vichw_init_speed();
	vichw_init_servo();
}
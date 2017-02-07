#include <time.h>

#include "vic_hardware.h"
#include "servo_controller.h"
#include "motor_speed_controller.h"

int main(void)
{
	int angleX10 = -300;
	int speedX100 = 0;
	vichw_init();
	
	while (angleX100 <= 300) {
		if (speedX100 > 80) {
			speedX100 = -10;
		}
		
		vichw_set_angle(angleX10 / 10.0);
		vichw_set_speed(speedX100 / 100.0);
		angleX10++;
		speedX100++;
		usleep(50000);
	}
	
	vichw_set_angle(0);
	vichw_set_speed(-1);
	
	return 0;
}
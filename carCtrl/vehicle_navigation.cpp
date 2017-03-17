#include <stdio.h>
#include <algorithm>
#include <math.h>

#include "vehicle_navigation.h"
#include "vic_types.h"
#include "vichw/servo_controller.h"
#include "vichw/motor_speed_controller.h"

int update_navigation(struct ImageData *img,  struct CarStatus *car){

	double speed_ok = 0;
	double angle_ok = 0;

	//Update steering angle
	double fix_value = img->fix;
	double new_angle = 0;

	if (fix_value >= 20) {
		new_angle = fix_value/5;
	}

	printf("setting new angle %f \n",new_angle);


	if(new_angle > MAX_ANGLE){
		new_angle = MAX_ANGLE;
	}else if(new_angle < -1*MAX_ANGLE){
		new_angle = -1*MAX_ANGLE;
	}


	car->current_wheel_angle = new_angle;
	vichw_set_angle(new_angle);
	angle_ok = 1;

	//TODO: Update vehicle speed
	double new_speed = 0;

	new_speed = car->current_speed;
	if(new_speed > MAX_SPEED){
		new_speed = MAX_SPEED;
	}

//	printf("setting speed = %f \n\n", new_speed);
	car->current_speed = new_speed;
	vichw_set_speed(new_speed);
	speed_ok = 1;
	
	return speed_ok*angle_ok;
}



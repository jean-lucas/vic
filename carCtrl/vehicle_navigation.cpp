#include <stdio.h>
#include <algorithm>
#include <math.h>

#include "vehicle_navigation.h"
#include "vic_types.h"
#include "vichw/servo_controller.h"
#include "vichw/motor_speed_controller.h"

int update_navigation(ImageData *img, CarStatus *car){

	double speed_ok = 0;
	double angle_ok = 0;

	//Update steering angle
	double angle_diff_abs = abs(img->avg_left_angle - img->avg_right_angle);
	double new_angle;

	if(angle_diff_abs >= ANGLE_THRESHOLD){

		double angle_diff =  img->avg_left_angle - img->avg_right_angle;
		new_angle = angle_diff/2;

	}else{
		new_angle = car->current_wheel_angle;
	}

	if(new_angle > MAX_ANGLE){
		new_angle = MAX_ANGLE;
	}else if(new_angle < -1*MAX_ANGLE){
		new_angle = -1*MAX_ANGLE;
	}

	printf("setting angle= %f \n", new_angle);

	car->current_wheel_angle = new_angle;
	vichw_set_angle(new_angle);
	angle_ok = 1;

	//TODO: Update vehicle speed
	double new_speed;

	new_speed = car->current_speed;
	if(new_speed > MAX_SPEED){
		new_speed = MAX_SPEED;
	}

	printf("setting speed = %f \n", new_speed);
	car->current_speed = new_speed;
	vichw_set_speed(new_speed);
	speed_ok = 1;
	
	return speed_ok*angle_ok;
}

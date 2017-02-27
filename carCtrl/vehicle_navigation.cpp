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
	double angle_diff = img->avg_left_angle - img->avg_right_angle;
	double length_diff = img->left_line_length - img->right_line_length;
	double new_angle;

	if(fabs(angle_diff) >= ANGLE_THRESHOLD){
		angle_diff =  img->avg_left_angle - img->avg_right_angle;
		new_angle = angle_diff/3;
	}

	else if( fabs(angle_diff) < ANGLE_THRESHOLD && car->current_wheel_angle > 0) {
		new_angle = 0;
	}

	else{

		new_angle = car->current_wheel_angle;
	}

	printf("setting angle= %f \n", new_angle);


	if(new_angle > MAX_ANGLE){
		new_angle = MAX_ANGLE;
	}else if(new_angle < -1*MAX_ANGLE){
		new_angle = -1*MAX_ANGLE;
	}


	car->current_wheel_angle = new_angle;
	vichw_set_angle(new_angle);
	angle_ok = 1;

	//TODO: Update vehicle speed
	double new_speed;

	new_speed = car->current_speed;
	if(new_speed > MAX_SPEED){
		new_speed = MAX_SPEED;
	}

	printf("setting speed = %f \n\n", new_speed);
	car->current_speed = new_speed;
	vichw_set_speed(new_speed);
	speed_ok = 1;
	
	return speed_ok*angle_ok;
}

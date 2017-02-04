#include <stdio.h>
#include <algorithm>
#include <math.h>

#include "vehicle_navigation.h"
#include "vic_types.h"
#include "servo_controller.h"

int update_navigation(ImageData *img, CarStatus *car){
	
	double speed_ok = 0;
	double angle_ok = 0;
	
	//TODO: Update vehicle speed
	speed_ok = 1;
	
	//Update steering angle
	double angle_diff_abs = abs(img->avg_left_angle - img->avg_right_angle);
	
	if(angle_diff_abs >= ANGLE_THRESHOLD){
		
		double angle_diff =  img->avg_left_angle - img->avg_right_angle;
		double new_angle = angle_diff/2;
		
		if(angle_diff > MAX_ANGLE){
			new_angle = MAX_ANGLE;
		}else if(angle_diff < -1*MAX_ANGLE){
			new_angle = -1*MAX_ANGLE;
		}
		
		car->current_wheel_angle = new_angle;
		set_angle(new_angle);
		angle_ok = 1;
		
		return speed_ok*angle_ok;
	}
	
}

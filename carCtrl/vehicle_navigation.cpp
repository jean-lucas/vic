#include <stdio.h>
#include <algorithm>
#include <math.h>

#include "vehicle_navigation.h"
#include "vic_types.h"
#include "vichw/servo_controller.h"
#include "vichw/motor_speed_controller.h"
#include "vichw/vic_hardware.h"
#include "pid.h"

struct pid_context pid;

void init_navigation(double time_period) {
	pid_tune(&pid, 2, 10, 0, DEFAULT_PWM, time_period);
	pid_set_clipping(&pid, MAX_SERVO_PWN, MIN_SERVO_PWM);
	pid_set(&pid, 0);
}

int update_navigation(struct ImageData *img,  struct CarStatus *car){

	double angle_diff  = img->avg_left_angle - img->avg_right_angle;
	double length_diff = img->left_line_length - img->right_line_length;
	double new_angle   = 0;


	if( abs(angle_diff) >= ANGLE_THRESHOLD) {
		new_angle = angle_diff/3.0;
	}

	else if( abs(angle_diff) < ANGLE_THRESHOLD && car->current_wheel_angle > 0) {
		new_angle = 0;
	}
	
	else{
		if (length_diff > LENGTH_THRESHOLD) {
			new_angle = CENTER_ADJUST_ANGLE;
		}
		else {
			new_angle = car->current_wheel_angle;
		}
	}

	if ( abs(new_angle - car->current_wheel_angle) > 35) {
		printf("\t\t\t\t\thmm\n");
		new_angle = car->current_wheel_angle;
	}

	

	printf("setting angle= \t\t%f\n", new_angle);
	car->current_wheel_angle = new_angle;

	if(new_angle > MAX_ANGLE){
		new_angle = MAX_ANGLE;
	}else if(new_angle < -1*MAX_ANGLE){
		new_angle = -1*MAX_ANGLE;
	}

	vichw_set_angle(new_angle);

	double angle_ok = 1;
	// double pwm = pid_update(&pid, -1*new_angle);
	// printf("setting pwm \t\t %f\n\n", pwm );
	// vichw_set_angle2(pwm);


	//TODO: Update vehicle speed
	double new_speed = 0;

	if (img->go_slow) {
		new_speed = car->current_speed/1.2;
	}
	else {
		new_speed = 0.5;
	}
	
	new_speed = car->current_speed;
	if(new_speed > MAX_SPEED){
		new_speed = MAX_SPEED;
	}

	car->current_speed = new_speed;
	vichw_set_speed(new_speed);
	double speed_ok = 1;
	
	return speed_ok*angle_ok;
}



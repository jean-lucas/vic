#include <stdio.h>
#include <algorithm>
#include <math.h>

#include "vehicle_navigation.h"
#include "vic_types.h"
#include "vichw/servo_controller.h"
#include "vichw/motor_speed_controller.h"
#include "vichw/vic_hardware.h"
#include "pid.h"

int large_delta_count = 0;

double pp = 0;
double dd = 0;
double qq = 0;



double hard_right = 45;
double hard_left  = -45;
double mild_right = 30;
double mild_left  = -30;



int sign(double val);
void make_left_turn(int type);
void make_right_turn(int type);
void follow_path(struct ImageData *img);
double lengthExpert(double avg_left, double avg_right);
double slopeExpert(double prev_slope, double curr_slope);

static int count = 0;
static double setting_speed = 0;
static double setting_angle = 0;

int update_navigation(struct ImageData *img,  struct CarStatus *car, double p1, double d2, double q3) {
	pp = p1;
	dd = d2;
	qq = q3;


	int direction = car->travel_direction;

	if (direction > 0 && count == 0) { 
		make_right_turn(img->intersection_type);
	}

	else if (direction < 0 && count == 0) {
		make_left_turn(img->intersection_type);
	}

	else if (count == 0) {
		follow_path(img);
	} 

	else {
		count--;
		setting_speed = LOW_SPEED;
		if  (count == 0) {
			printf("reset straight\n");
			car->travel_direction = STRAIGHT_PATH;
			setting_speed = NORMAL_SPEED;
		}
	}
	


	//slow down if approaching intersection
	if (car->intersection_detected) 
		setting_speed = LOW_SPEED;
	else if (car->intersection_stop)
		setting_speed = STOP_SPEED;
	else
		setting_speed NORMAL_SPEED;


	car->current_speed 		 = setting_speed;
	car->current_wheel_angle = setting_angle;

	vichw_set_speed(setting_speed);
	vichw_set_angle(setting_angle);
	

	return 1;
}

/* Gives access to the main loop to stop the car when desired */
void stop_car() {
	setting_speed = STOP_SPEED;
	vichw_set_speed(setting_speed);
}


/* 
	Attempt to make a right turn at a given intersection type
	TODO: calibrate the turning angles and count values
*/
void make_right_turn(int type) {
	printf("start right turn\n");
	if (type)
		setting_angle = mild_right;
	else 
		setting_angle = hard_right;

	setting_speed = LOW_SPEED;
	count = 50;
}


/* 
	Attempt to make a left turn at a given intersection type
	TODO: calibrate the turning angles and count values
*/
void make_left_turn(int type) {
	printf("start left turn\n");
	if (type)
		setting_angle = hard_left;
	else 
		setting_angle = mild_left;
	

	setting_speed = LOW_SPEED;
	count = 70;
}


/* 
	Follow the path given by the ImageData 
	The setting_angle is based on the contributions from the
	slope expert and length expert
*/
void follow_path(struct ImageData *img) {
	double ang1 = 0,  ang2 = 0, ang = 0;
	double current_angle = car->current_wheel_angle;

	ang1 = lengthExpert(img->left_line_length, img->right_line_length);
	ang2 = slopeExpert(img->old_slope, img->avg_slope);


	ang = (ang1/dd + ang2 )/2.0;

	if (sign(ang) != sign(current_angle) && abs(ang - current_angle) > 20 && current_angle != 0  && large_delta_count != 1) {
		ang = current_angle;
		large_delta_count = 1;
	}
	else {
		large_delta_count = 0;
	}


	setting_angle = ang;
}




double slopeExpert(double prev_slope, double curr_slope) {

	double new_angle = 0;

	if (sign(prev_slope) == sign(curr_slope)) {
		new_angle = curr_slope/qq;
	}
	else {
		//average out the slopes
		printf("=========== TAKING AVERAGE (2) =========\n");
		new_angle = (curr_slope + (prev_slope*0.6))/(2.0);
		new_angle = new_angle/qq;
	}
	return new_angle;
}


double lengthExpert(double avg_left, double avg_right) {

	double new_angle = 0;	
	
	if (avg_left < 170 && avg_left > 0) {
		new_angle = 35;
		count = 3;
	}
	else if  (avg_right < 170 && avg_right > 0) {
		new_angle = -35;
		count = 3;
	}

	return new_angle;
}




int sign(double val) {
	if (val > 0) return 1;
	if (val < 0) return -1;
	return 1;
}




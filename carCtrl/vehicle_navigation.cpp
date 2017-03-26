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
	// pid_tune(&pid, 2, 10, 0, DEFAULT_PWM, time_period);
	// pid_set_clipping(&pid, MAX_SERVO_PWN, MIN_SERVO_PWM);
	// pid_set(&pid, 0);

}


//get the sign of a value
int sign(double val) {
	if (val > 0) return 1;
	if (val < 0) return -1;
	return 1;
}


int reduce_speed = 0;


int update_navigation(struct ImageData *img,  struct CarStatus *car, double p, double d){

	
	//holy long line batman!
	double ang = calculate_angle(img->avg_left_angle, img->avg_right_angle, img->trajectory_angle,  \
								car->current_wheel_angle, img->left_line_length, img->right_line_length, p ,d);


	printf("setting angle= \t\t%f\n\n", ang);
	car->current_wheel_angle = ang;
	vichw_set_angle(ang);


	//TODO: Update vehicle speed
	double new_speed = 0;

	if (reduce_speed) {
		new_speed = 0.48;
		reduce_speed = 0;
	}
	else {
		new_speed = 0.50;
	}
	
	if(new_speed > MAX_SPEED){
		new_speed = MAX_SPEED;
	}

	car->current_speed = new_speed;
	vichw_set_speed(new_speed);

	return 1;
}

void set_speed(double speed) {
	vichw_set_speed(speed);
}



//given the angles and line lengths calculate a new desired angle to follow.
//under certain conditions, this method may advise the car should slow down.
double calculate_angle(double theta1, double theta2, double theta3, double current_angle, double left_len, double right_len, double p, double d) {

	double new_angle = 0;
	double angle_diff = theta1 - theta2;

	if (sign(angle_diff) == sign(theta3)) { // take their avg
		new_angle = ((angle_diff)/p + theta3/d)/1;
	}

	else if (theta3 != 0 && theta1*theta2 == 0) {
		new_angle = theta3/d;
	} 

	else if (theta3 == 0 && theta1*theta2 != 0) {
		if (angle_diff > ANGLE_THRESHOLD) {
			new_angle = angle_diff/p;
		}
		else {
			new_angle = current_angle;
		}
	}

	else {
		new_angle = current_angle/3;
		reduce_speed = 1;
	}

	//should we check for first offense?
	if (sign(new_angle) != sign(current_angle) && abs(new_angle - current_angle) > 20) {
		new_angle = new_angle/3;
		printf(" ===== angle  reduction =====\n");
	}


	//check clipping
	if(new_angle > MAX_ANGLE){
		new_angle = MAX_ANGLE;
	}
	else if(new_angle < -1*MAX_ANGLE){
		new_angle = -1*MAX_ANGLE;
	}


	return new_angle;
}	





	// if (img->theta3 != 0) {
	// 	new_angle = img->theta3/d;
	// }
	// else if( abs(angle_diff) >= 5) {
	// 	new_angle = angle_diff/p;
	// 	new_angle += (img->theta3)/d;
	// }

	// else if( abs(angle_diff) < ANGLE_THRESHOLD && car->current_wheel_angle != 0) {
	// 	// new_angle = 0;
	// 	new_angle = img->theta3/d;
	// }
	
	// else{
	// 	printf("easy\n");
	// 	if (length_diff > LENGTH_THRESHOLD) {
	// 		new_angle = CENTER_ADJUST_ANGLE;
	// 	}
	// 	else {
	// 		new_angle = car->current_wheel_angle;
	// 	}
	// }

	

	// if ( new_angle > 0 && car->current_wheel_angle < 0 && abs(abs(new_angle) - abs(car->current_wheel_angle)) > 10) {
	// 	printf("\t\t\t\t\thmm================\n");
	// 	new_angle = car->current_wheel_angle;
	// }
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


int reduce_speed = 0;
int left_votes     = 0;
int right_votes    = 0;
int straight_votes = 0;
int large_delta_count = 0;

int p = 0;
int d = 0;

int sign(double val);

double calculate_angle(double theta1, double theta2, double theta3, double current_angle, double left_len, double right_len, double p, double d);

double slopeExpert(double prev_slope, double curr_slope);
double lengthExpert(double avg_left, double avg_right);
double angleExpert(double theta1, double theta2, double theta3, double current_angle);
double poll_votes();




void init_navigation(double time_period) {
	// pid_tune(&pid, 2, 10, 0, DEFAULT_PWM, time_period);
	// pid_set_clipping(&pid, MAX_SERVO_PWN, MIN_SERVO_PWM);
	// pid_set(&pid, 0);

}








int update_navigation(struct ImageData *img,  struct CarStatus *car, double p1, double d2, double q3){

	p = p1;
	d = d2;
	q = q3;
	left_votes = 0;
	right_votes = 0;
	straight_votes = 0;

	double current_angle = car->current_wheel_angle;
	double ang1 = angleExpert(img->avg_left_angle, img->avg_right_angle, img->trajectory_angle, current_angle);
	double ang2 = slopeExpert(img_data->old_slope, img_data->avg_slope);
	double ang3 = lengthExpert(img_data->left_line_length, img_data->right_line_length); 

	printf("ang1: %f\t ang2: %f\t ang3: %f\n",ang1,ang2,ang3);
	printf("voteL: %d\t voteC: %d\t voteR: %d\n",left_votes, straight_votes, right_votes);

	double ang = (ang1 + ang2 + ang3)/3.0;

	if (sign(ang) != sign(current_angle) && abs(ang - current_angle) > 20 && current_angle != 0  && large_delta_count != 1) {
		printf("before reduction %f\n", ang );
		ang = current_angle;
		printf(" ===== angle  set previous =====\n");
		large_delta_count = 1;
	}
	else {
		large_delta_count = 0;
	}



	printf("setting angle= \t\t%f\n\n", ang);
	car->current_wheel_angle = ang;
	vichw_set_angle(ang);


	//TODO: Update vehicle speed
	double new_speed = 0;

	if (reduce_speed) {
		new_speed = 0.45;
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


void vote(double val) {
	if (sign(val) > 0) 
		right_votes++;
	else if (sign(val) < 0)
		left_votes++;
	else 
		straight_votes++;
}



double slopeExpert(double prev_slope, double curr_slope) {

	double new_angle = 0;
	if (sign(prev_slope) == sign(curr_slope)) {
		new_angle = curr_slope/q;
	}
	else {
		//average out the slopes
		printf("=========== TAKING AVERAGE (2) =========\n");
		new_angle = (curr_slope + old_slope)/(q*2.0);
	}

	vote(new_angle);

	return new_angle;
}


double lengthExpert(double avg_left, double avg_right) {

	double new_angle = 0;

	if (avg_left < avg_right/1.5)
		vote(-1);
	if (avg_right < avg_left/1.5)
		vote(1);
	else
		vote(0);

	if (avg_left - avg_right > -1*LENGTH_THRESHOLD) {
		new_angle = CENTER_ADJUST_ANGLE;
	}
	else if (avg_left - avg_right > LENGTH_THRESHOLD) {
		new_angle = -1*CENTER_ADJUST_ANGLE;
	}
	vote(new_angle);
}

double angleExpert(double theta1, double theta2, double theta3, double current_angle, double curr_theta3) {


	//first clean up the angles
	if (theta1 == 0 && theta3 == 0 && theta2 != 0) { //turning right, and lost track of left lane
        theta3 = (theta2 - 90)/3.0;
        printf("!!!!!!!!!!!\n");

    }
    else if (theta2 == 0 && theta3 == 0 && theta1 != 0) { //turning left, and lost track of right lane
        theta3 = (90 - theta1)/3.0;
        printf("???????????\n");

    }
    else if (theta1 != 0 && theta2 != 0 && theta3 == 0) { //cant find trajectory point, use average of sides
        printf("############\n");
        theta3 = (theta1 -theta2)/2.0;
    }

    if (theta1 == 0) 
        theta1 = theta2/4.0;
    
    if (theta2 == 0)
        theta2 = theta1/4.0;
    
    if (theta3 == 0) //if everything fails attempt to use our previous angle value
        theta3 = current_angle/3.0;
    



	double new_angle = 0;
	double angle_diff = theta1 - theta2;

	
	//both going the same direction
	if (sign(angle_diff) == sign(theta3) && abs(angle_diff) > ANGLE_THRESHOLD) { 
		new_angle = angle_diff/p + theta3/d;
	}
	else if (sign(angle_diff) == sign(theta3)) {
		new_angle =  theta3/d;
	}
	else {
		printf("=========== TAKING AVERAGE (1) =========\n");
		new_angle = (angle_diff/p + theta3/d)/2.0;
	}

	//vote
	vote(new_angle);


	return new_angle;
}	



//get the sign of a value
int sign(double val) {
	if (val > 0) return 1;
	if (val < 0) return -1;
	return 1;
}







	// double ang = calculate_angle(img->avg_left_angle, img->avg_right_angle, img->trajectory_angle,  \
	// 							car->current_wheel_angle, img->left_line_length, img->right_line_length, p ,d);


// //given the angles and line lengths calculate a new desired angle to follow.
// //under certain conditions, this method may advise the car should slow down.
// double calculate_angle(double theta1, double theta2, double theta3, double current_angle, double left_len, double right_len, double p, double d) {

// 	double new_angle = 0;
// 	double angle_diff = theta1 - theta2;

	

// 	if (sign(angle_diff) == sign(theta3)) { 
// 		new_angle = angle_diff/p + theta3/d;
// 	}

// 	else if (theta3 != 0 && theta1*theta2 == 0) {
// 		new_angle = theta3/d;
// 	} 

// 	else if (theta3 == 0 && theta1*theta2 != 0) {
// 		if (angle_diff > ANGLE_THRESHOLD) {
// 			new_angle = angle_diff/p;
// 		}
// 		else {
// 			new_angle = current_angle;
// 		}
// 	}

// 	else {
// 		new_angle = current_angle/3;
// 		reduce_speed = 1;
// 	}

// 	//should we check for first offense?
// 	if (sign(new_angle) != sign(current_angle) && abs(new_angle - current_angle) > 20 && current_angle != 0 ) {
// 		printf("before reduction %f\n", new_angle );
// 		// new_angle = new_angle/5;
// 		new_angle = current_angle;
// 		printf(" ===== angle  set previous =====\n");
// 	}


// 	//check clipping
// 	if(new_angle > MAX_ANGLE){
// 		new_angle = MAX_ANGLE;
// 	}
// 	else if(new_angle < -1*MAX_ANGLE){
// 		new_angle = -1*MAX_ANGLE;
// 	}


// 	return new_angle;
// }	

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

double pp = 0;
double dd = 0;
double qq = 0;
int sign(double val);

double calculate_angle(double theta1, double theta2, double theta3, double current_angle, double left_len, double right_len, double p1, double d2, double q3);

double slopeExpert(double prev_slope, double curr_slope);
double lengthExpert(double avg_left, double avg_right);
double angleExpert(double theta1, double theta2, double theta3, double current_angle);
double poll_votes();




void init_navigation(double time_period) {
	// pid_tune(&pid, 2, 10, 0, DEFAULT_PWM, time_period);
	// pid_set_clipping(&pid, MAX_SERVO_PWN, MIN_SERVO_PWM);
	// pid_set(&pid, 0);

}


void vote(double val) {
	if (sign(val) > 0) 
		right_votes++;
	else if (sign(val) < 0)
		left_votes++;
	else 
		straight_votes++;
}







int update_navigation(struct ImageData *img,  struct CarStatus *car, double p1, double d2, double q3){

	pp = p1;
	dd = d2;
	qq = q3;
	left_votes = 0;
	right_votes = 0;
	straight_votes = 0;
	double ang1 = 0,  ang2 = 0, ang3 = 0;
	double current_angle = car->current_wheel_angle;


	if (img->left_line_length < 150 && img->left_line_length > 10) {
		ang1 = 45;
		vote(1);
	}
	else if  (img->right_line_length < 150 && img->right_line_length > 10) {
		ang1 = -45;
		vote(-1);
	}
	else if (img->left_line_length > 160 && img->right_line_length > 160	) {
		// ang1 = angleExpert(img->avg_left_angle, img->avg_right_angle, img->trajectory_angle, current_angle);
	}



	ang2 = slopeExpert(img->old_slope, img->avg_slope);
	// ang3 = lengthExpert(img->left_line_length, img->right_line_length); 

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





double slopeExpert(double prev_slope, double curr_slope) {

	printf("curr_slope %f\n", curr_slope);
	double new_angle = 0;
	if (sign(prev_slope) == sign(curr_slope)) {
		new_angle = curr_slope/qq;
	}
	else {
		//average out the slopes
		printf("=========== TAKING AVERAGE (2) =========\n");
		new_angle = (curr_slope + prev_slope)/(qq*2.0);
	}

	vote(new_angle);

	return new_angle;
}


double lengthExpert(double avg_left, double avg_right) {

	double new_angle = 0;	
	printf("left-len: %f \t right-len: %f\n", avg_left, avg_right);
	if (avg_left < avg_right/1.5)
		vote(-1);
	if (avg_right < avg_left/1.5)
		vote(1);
	else
		vote(0);

	if (avg_left - avg_right < -1*LENGTH_THRESHOLD) {
		new_angle = CENTER_ADJUST_ANGLE;
	}
	else if (avg_left - avg_right > LENGTH_THRESHOLD) {
		new_angle = -1*CENTER_ADJUST_ANGLE;
	}
	vote(new_angle);
}

double angleExpert(double theta1, double theta2, double theta3, double current_angle) {


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
        theta3 = 0;
        // theta3 = (theta1 -theta2)/2.0;
    }

    if (theta1 == 0) 
        theta1 = theta2/4.0;
    
    if (theta2 == 0)
        theta2 = theta1/4.0;
    
    if (theta3 == 0) //if everything fails attempt to use our previous angle value
        theta3 = current_angle/3.0;
    


    printf("Theta1: %f \tTheta2: %f\tTheta3: %f\t\n", theta1, theta2, theta3);

	double new_angle = 0;
	double angle_diff = theta1 - theta2;

	
	//both going the same direction
	if (sign(angle_diff) == sign(theta3) && abs(angle_diff) > ANGLE_THRESHOLD) { 
		new_angle = angle_diff/pp + theta3/dd;
	}
	else if (sign(angle_diff) == sign(theta3)) {
		new_angle =  theta3/dd;
	}
	else {
		printf("=========== TAKING AVERAGE (1) =========\n");
		new_angle = (angle_diff/pp + theta3/dd)/2.0;
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
	// 							car->current_wheel_angle, img->left_line_length, img->right_line_length, pp ,dd);


// //given the angles and line lengths calculate a new desired angle to follow.
// //under certain conditions, this method may advise the car should slow down.
// double calculate_angle(double theta1, double theta2, double theta3, double current_angle, double left_len, double right_len, double pp, double dd) {

// 	double new_angle = 0;
// 	double angle_diff = theta1 - theta2;

	

// 	if (sign(angle_diff) == sign(theta3)) { 
// 		new_angle = angle_diff/pp + theta3/dd;
// 	}

// 	else if (theta3 != 0 && theta1*theta2 == 0) {
// 		new_angle = theta3/dd;
// 	} 

// 	else if (theta3 == 0 && theta1*theta2 != 0) {
// 		if (angle_diff > ANGLE_THRESHOLD) {
// 			new_angle = angle_diff/pp;
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

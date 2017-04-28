#ifndef VIC_TYPES_H
#define VIC_TYPES_H

/* Constants */ 

//response signals from bluetooth
const int PROCEED_RESP 		  = 0;
const int STOP_RESP 		  = 1;
const int EMERGENCY_STOP_RESP = 2;

//error messages
const int NO_ERROR 		= 1;
const int HALT_SYSTEM 	= 0;
const int CORRUPT_IMAGE = -1;

//vehicle speed values
const double STOP_SPEED   = 0;
const double NORMAL_SPEED = 0.50;
const double LOW_SPEED 	  = NORMAL_SPEED - 0.02;

//vehicle travel direction
const int STRAIGHT_PATH = 0;
const int RIGHT_PATH 	= 1;
const int LEFT_PATH		= -1;



struct ImageData {
	double avg_slope;
	double old_slope;
	double left_line_length;
	double right_line_length;
	double intersection_distance;
	int intersection_detected;
	int intersection_stop;
	int intersection_colour;
	int obstacle_detected;
	int intersection_type;	
};

struct CarStatus {
	double current_speed;
	double current_wheel_angle;
	int car_id;
	int obstacle_stop;
	int current_lane;
	int travel_direction; //0 straight, 1 right, -1 left
	int drive_thru;	//0 no, 1 yes
};


struct SignalRequest {
	int port;
};

struct SignalResponse {
	int val;
};



#endif

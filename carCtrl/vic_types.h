#ifndef VIC_TYPES_H
#define VIC_TYPES_H

/* Constants */ 

//response signals from bluetooth
const int PROCEED_RESP = 0;
const int STOP_RESP = 1;
const int EMERGENCY_STOP_RESP = 2;



/* structs */
struct ImageData {
	double fix;
	double avg_left_angle;
	double avg_right_angle;
	double left_line_length;
	double right_line_length;
	double intersection_distance;
	int intersection_detected;
	int obstacle_detected;
};

struct CarStatus {
	double current_speed;
	double current_wheel_angle;
	int car_id;
	int intersection_stop;
	int obstacle_stop;
	int current_lane;
};

struct SignalRequest {
	int port;
};

struct SignalResponse {
	int val;
};



#endif

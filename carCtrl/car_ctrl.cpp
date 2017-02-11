#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include "car_ctrl.h"
#include "laneDetect.h"
#include "carComms.h"
#include "vic_types.h"
#include "vehicle_navigation.h"
#include "vichw/vic_hardware.h"




CarStatus car_stat;
ImageData img_data;


int main(int argc, char** argv) {

	if (!init()) {
		printf("failed to init\n");
		exit(0);
	}

	run();

	return 0;
}


/* Initialize necessary modules, and test their are working */
int init () {

	unsigned int status = 1;

	car_stat.current_speed 		 = 0.6;
	car_stat.current_wheel_angle = 0;
	car_stat.car_id 			 = CAR_ID;
	car_stat.intersection_stop 	 = 0;
	car_stat.obstacle_stop 		 = 0;
	car_stat.current_lane 		 = OUTER_LANE;


	img_data.avg_left_angle		   = 0;
	img_data.avg_right_angle 	   = 0;
	img_data.left_line_length 	   = 0;
	img_data.right_line_length 	   = 0;
	img_data.intersection_distance = -1;
	img_data.intersection_detected = 0;
	img_data.obstacle_detected 	   = 0;
	

	status &= test_camera(); 

	printf("Calling vichw_init from main\n");
	status &= vichw_init();

	printf("status from vichw_init = %d \n", status);


	return status;
}

int run() {

	int valid = 1;
	 while (valid) {
		valid = get_lane_status(&img_data);
		valid = update_navigation(&img_data, &car_stat);
		usleep(500000);
	}

	printf(" %f\n %f\n %f\n %f\n %f\n %d\n %d\n", img_data.avg_left_angle, img_data.avg_right_angle, \
		img_data.left_line_length, img_data.right_line_length, img_data.intersection_distance, img_data.intersection_detected, \
		img_data.obstacle_detected);
	sleep(2);
	vichw_deinit();

	return valid;

}


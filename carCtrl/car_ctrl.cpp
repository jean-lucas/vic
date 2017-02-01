#include <stdio.h>
#include <stdlib.h>

#include "car_ctrl.h"
#include "laneDetect.h"
#include "carComms.h"
#include "vic_types.h"




int main(int argc, char** argv) {

	printf("This is from main\n");


	//int k = get_lane_status();
	//int i =	sendToIC("blah blah\n");


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

	status &= test_camera();

	return status;

}
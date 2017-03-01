#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>
#include "opencv2/highgui/highgui.hpp"

#include "car_ctrl.h"
#include "laneDetect.h"
#include "carComms.h"
#include "vic_types.h"
#include "vehicle_navigation.h"
#include "vichw/vic_hardware.h"




CarStatus car_stat;
ImageData img_data, img_data_prev;
cv::VideoCapture cap;


int main(int argc, char** argv) {
    if (argc > 1) {
            cap = test_camera();
            capture_lane(&cap);
            cap.release();
            return 0;
    }
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

    car_stat.current_speed           = 0.5;
    car_stat.current_wheel_angle     = 0;
    car_stat.car_id                  = CAR_ID;
    car_stat.intersection_stop       = 0;
    car_stat.obstacle_stop           = 0;
    car_stat.current_lane            = OUTER_LANE;


    img_data.avg_left_angle            = 0;
    img_data.avg_right_angle           = 0;
    img_data.left_line_length          = 0;
    img_data.right_line_length         = 0;
    img_data.intersection_distance     = -1;
    img_data.intersection_detected     = 0;
    img_data.obstacle_detected         = 0;

    copyStruct(&img_data, &img_data_prev);

    cap = test_camera();
    status &= vichw_init();


    pthread_t  thread_server;
    pthread_create(&thread_server, NULL, recvFromIC, NULL);

    return status;
}







int run() {

	int valid = 1;

	struct timeval t;
	gettimeofday(&t, NULL);
	double ms = (t.tv_sec)*1000 + (t.tv_usec)/1000;
	double old = ms;
	printf("startTime (ms) = %f \n", ms);

	while (valid) {
	    gettimeofday(&t, NULL);
        old = (t.tv_sec)*1000 + (t.tv_usec)/1000;

        valid &= get_lane_status(&img_data, &cap);
        valid &= update_navigation(&img_data, &img_data_prev, &car_stat);
        copyStruct(&img_data, &img_data_prev);

        gettimeofday(&t, NULL);
        ms = (t.tv_sec)*1000 + (t.tv_usec)/1000;

//        printf(" diff = %f \n", ms-old);


	}

	/*      printf(" left theta: %f\n right theta %f\n left len %f\n right len %f\n int len%f\n inter? %d\n obstacle? %d\n", img_data.avg_left_angle, img_data.avg_right_angle, \
	        img_data.left_line_length, img_data.right_line_length, img_data.intersection_distance, img_data.intersection_detected, \
	        img_data.obstacle_detected);
	*/

	cap.release();
	vichw_deinit();

	return valid;

}



void copyStruct(struct ImageData *img1, struct ImageData *img2) {
	img2->avg_left_angle            = img1->avg_left_angle;
	img2->avg_right_angle           = img1->avg_right_angle;
	img2->left_line_length          = img1->left_line_length;
	img2->right_line_length         = img1->right_line_length;
	img2->intersection_distance     = img1->intersection_distance;
	img2->intersection_detected     = img1->intersection_detected;
	img2->obstacle_detected         = img1->obstacle_detected;
}

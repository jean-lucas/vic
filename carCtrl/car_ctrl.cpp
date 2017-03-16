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
ImageData img_data, img_data_msa;
SignalResponse *sig_resp;
cv::VideoCapture cap;

const int MSA_MAX_STEP = 3;
int msa_curr_step = 1;



int main(int argc, char** argv) {
    if (argc > 1) {
            cap = test_camera();
           // capture_lane(&cap);
	    get_lane_statusv2(&cap);
	    cap.release();
            return 0;
    }
    if (!init()) {
            printf("failed to init\n");
            exit(0);
    }

    run();
    cleanup();
    return 0;
}


/* Initialize necessary modules, and test their are working */
int init () {

    unsigned int status = 1;

    car_stat.current_speed           = 0.53;
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


    sig_resp = (struct SignalResponse*) calloc(1, sizeof(*sig_resp));
    sig_resp->val = DEFAULT_RESP;

    // memcpy(&img_data_msa, &img_data, sizeof(img_data));

    cap = test_camera();
    status &= vichw_init();

    pthread_t  server_thread;
    pthread_create(&server_thread, NULL, recvFromIC, (void*) sig_resp);


    return status;
}







double getMsTime() {
    struct timeval t;
    gettimeofday(&t, NULL);
    double ms = (t.tv_sec)*1000 + (t.tv_usec)/1000;
    return ms;
}


//Moving sum average - resets when step is one
void update_msa(struct ImageData *img, struct ImageData *img_msa, int step) {

    //if step is 1 (first step) use the current data
    if (step == 1) {
        memcpy(&img_msa, &img, sizeof(img));
    }

    else {
        img_msa->avg_left_angle            = (img->avg_left_angle    + (step*img_msa->avg_left_angle))    / (step + 1);
        img_msa->avg_right_angle           = (img->avg_right_angle   + (step*img_msa->avg_right_angle))   / (step + 1);
        img_msa->left_line_length          = (img->left_line_length  + (step*img_msa->left_line_length))  / (step + 1);
        img_msa->right_line_length         = (img->right_line_length + (step*img_msa->right_line_length)) / (step + 1);
        img_msa->intersection_distance     = img->intersection_distance;
        img_msa->intersection_detected     = img->intersection_detected;
        img_msa->obstacle_detected         = img->obstacle_detected;
    }
}



int run() {

	int valid = 1;

    int step = 1;
	while (valid) {

        //update_msa(&img_data, &img_data_msa, step);
        valid &= get_lane_status(&img_data, &cap);
        valid &= update_navigation(&img_data, &car_stat);
        // if (step > MSA_MAX_STEP) {
        //     step = 0;
        // }
        // step += 1;


        // printf("sig  response = %d \n", sig_resp->val);
         if (sig_resp->val == EMERGENCY_STOP_RESP) {
             valid = 0;
             break;
         }
	}

	      // printf(" left theta: %f\n right theta %f\n left len %f\n right len %f\n int len%f\n inter? %d\n obstacle? %d\n\n", img_data_msa.avg_left_angle, img_data_msa.avg_right_angle, \
	      //   img_data_msa.left_line_length, img_data_msa.right_line_length, img_data_msa.intersection_distance, img_data_msa.intersection_detected, \
	      //   img_data_msa.obstacle_detected);

	return valid;

}



void cleanup() {
    printf("Cleaning up services\n");
    cap.release();
    vichw_deinit();
    free(sig_resp);
}




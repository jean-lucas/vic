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
ImageData img_data;
SignalResponse *sig_resp;
cv::VideoCapture cap;






int main(int argc, char** argv) {

    int start_on_bt_cmd =  0;


    if (argc < 1) {
        printf("invalid arg\n");
        return 0;
    }

    int argval = atoi(argv[1]):

    //test lane detect only
    if (argval == 1) {
        cap = test_camera();
	    get_lane_statusv2(&img_data, &cap);
	    cap.release();
        return 0;
    }

    //start operating only from BT commands
    else if (argval == 2 ) {
        start_on_bt_cmd = 1;
    }

    if (!init(start_on_bt_cmd)) {
        printf("failed to init\n");
        exit(0);
    
    }

    run();
    cleanup();
    return 0;
}


/* Initialize necessary modules, and test their are working */
int init (int quickstart_mode) {

    unsigned int status = 1;

    car_stat.current_speed           = 0.5;
    car_stat.current_wheel_angle     = 0;
    car_stat.car_id                  = CAR_ID;
    car_stat.intersection_stop       = 0;
    car_stat.obstacle_stop           = 0;
    car_stat.current_lane            = OUTER_LANE;

    img_data.fix                       = 0;
    img_data.avg_left_angle            = 0;
    img_data.avg_right_angle           = 0;
    img_data.left_line_length          = 0;
    img_data.right_line_length         = 0;
    img_data.intersection_distance     = -1;
    img_data.intersection_detected     = 0;
    img_data.obstacle_detected         = 0;


    sig_resp = (struct SignalResponse*) calloc(1, sizeof(*sig_resp));
    //make sure car does not start driving until we want it to
    if (quickstart_mode) {
        sig_resp->val = PROCEED_RESP;
    }
    else {
        sig_resp->val = STOP_RESP;

    }
    
     

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



int run() {

	int valid = 1;
	while (valid) {

        if (sig_resp->val != PROCEED_RESP) {
            if (sig_resp->val == EMERGENCY_STOP_RESP) {
                printf("EMERGENCY_STOP_RESP\n");
                valid = 0;
                break;
            }
            //otherwise a stop signal has been called
            pause();
        }

        valid &= get_lane_statusv2(&img_data, &cap);
        valid &= update_navigation(&img_data, &car_stat);
           
	}

	return valid;

}


//when called, program will remain paused until necessary BT responses are received
void pause() {
    printf("Entering pause state\n");
    while (sig_resp->val != PROCEED_RESP);
    printf("Leaving pause state\n");
    run();
}

void cleanup() {
    printf("Cleaning up services\n");
    cap.release();
    vichw_deinit();
    free(sig_resp);
}




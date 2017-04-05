#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <pigpio.h>

#include "car_ctrl.h"
#include "laneDetect.h"
#include "carComms.h"
#include "vic_types.h"
#include "vehicle_navigation.h"
#include "vichw/vic_hardware.h"
#include "opencv2/highgui/highgui.hpp"


CarStatus car_stat;
ImageData img_data;
SignalResponse *sig_resp;
cv::VideoCapture cap;


double p = 3;
double d = 1.5;    
double q = 2;
double starting_speed = 0.48;


void stop_at_intersection();


double getMsTime() {
    struct timeval t;
    gettimeofday(&t, NULL);
    double ms = (t.tv_sec)*1000 + (t.tv_usec)/1000;
    return ms;
}


int main(int argc, char** argv) {
    int quickstart_mode = 1;  


    if (argc < 1) {
        printf("invalid arg\n");
        return 0;
    }

    int argval = atoi(argv[1]);

    //test lane detect only
    if (argval == 1) {
        cap = test_camera();
        get_lane_statusv3(&img_data, &cap);
        // calibrate_camera(&cap);
	    cap.release();
        return 0;
    }

    //start operating only from BT commands
    if (argval == 2 ) {
        quickstart_mode = 0;
    }

    //start car as soon as it can
    if (argc > 2) {
        p = atof(argv[2]);
        d = atof(argv[3]);
        q = atof(argv[4]);
        printf("p = %f \t d= %f\t q= %f\n",p,d,q );
    }

    if (!init(quickstart_mode)) {
        printf("failed to init\n");
        return 0;
    
    }

    run();

    cleanup();
    
    return 0;
}


/* Initialize necessary modules, and test their are working */
int init(int quickstart_mode) {

    int status = 1;

    car_stat.current_speed           = starting_speed;
    car_stat.current_wheel_angle     = 0;
    car_stat.car_id                  = CAR_ID;
    car_stat.intersection_stop       = 0;
    car_stat.obstacle_stop           = 0;

    img_data.trajectory_angle          = 0;
    img_data.avg_left_angle            = 0;
    img_data.avg_right_angle           = 0; 
    img_data.avg_slope                 = 0;
    img_data.old_slope                 = 0;
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




int run() {

	int status = 1;
    double  t1 = 0;
    double time_diff = 0;


    double running_time = 0;
    int iterations = 0;
    double time_start = 0, time_end = 0;

    while (status != HALT_SYSTEM) {

        time_start  = getMsTime();
        //check for IC response
        if (sig_resp->val != PROCEED_RESP) {
            if (sig_resp->val == EMERGENCY_STOP_RESP) {
                printf("EMERGENCY_STOP_RESP\n");
                status = HALT_SYSTEM;
                break;                
            }
            else {
                pause_sys();
            }
        } 


        status = get_lane_statusv3(&img_data, &cap);


        //check if an intersection has been detected. If so, do the right thing
        if (img_data.intersection_detected && time_diff > 10000) {
            t1 = getMsTime();
            img_data.intersection_detected = 0;
            stop_at_intersection();
            status = get_lane_statusv3(&img_data, &cap);
            time_diff = getMsTime() - t1;
        }

        if (status == CORRUPT_IMAGE) {
            continue;
        }

        status = update_navigation(&img_data, &car_stat, p, d, q);

        time_diff = getMsTime() - t1;
        

        time_end = getMsTime();
        iterations += 1;
        running_time += (time_end - time_start);
    }

    printf("ending with status %d\n", status);

    printf("In %d iterations avg running time was %f\n", iterations,running_time/iterations);

}



//TODO: PLEASE FIX ME :(

//create message, stop the car from proceeding and enter pause state
void stop_at_intersection() {
    set_speed(0);
    sig_resp->val = STOP_RESP;

    //build msg
    char* msg = (char*) malloc(sizeof(int)*8);
    msg = "1_2_1_12345";

    int sent = sendToIC(msg);
    printf("sent a message with size %d\n",sent );

    while (sig_resp->val == STOP_RESP){};
    
    printf("Leaving intersection\n");
}


//when called, program will remain paused until necessary BT responses are received
void pause_sys() {
    printf("Entering pause state\n");
    set_speed(0);
    while (sig_resp->val == STOP_RESP);
    if (sig_resp->val == EMERGENCY_STOP_RESP) {
        cleanup();
        exit(0);
    }
    printf("Leaving pause state\n");
}





void cleanup() {
    printf("Cleaning up services\n");
    vichw_deinit();
    cap.release();
    free(sig_resp);
}






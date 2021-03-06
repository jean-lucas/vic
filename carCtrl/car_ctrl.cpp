#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <time.h>
#include <sys/time.h>
#include <pigpio.h>
#include <iostream>
#include <fstream>
#include <raspicam/raspicam_cv.h>

#include "car_ctrl.h"
#include "laneDetect.h"
#include "carComms.h"
#include "vic_types.h"
#include "vehicle_navigation.h"
#include "vichw/vic_hardware.h"


    

CarStatus car_stat;
ImageData img_data;
SignalResponse *sig_resp;
raspicam::RaspiCam_Cv cap;

pthread_cond_t  cond  = PTHREAD_COND_INITIALIZER;
pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;





double p = 3;
double d = 1.5;    
double q = 2;


int wake_thread = 0;
int kill_send_thread = 0;


unsigned long long getMsTime() {
    struct timeval t;
    gettimeofday(&t, NULL);
    unsigned long long  ms = (t.tv_sec)*1000 + (t.tv_usec)/1000;
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
        cap.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
        cap.set( CV_CAP_PROP_FRAME_WIDTH, 640);
        cap.set( CV_CAP_PROP_FRAME_HEIGHT, 480);
        cap.open();
        calibrate_raspicam(&cap);
        get_lane_statusv3(&img_data, &cap);
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

    /* get config values */
    int car_id     = 0;
    int pwm_offset = 0;
    std::ifstream f("/etc/vic.conf");
    if (f.is_open()) {
        f >> car_id;
        f >> pwm_offset;
    }
    else {
        status = 0; //failed to open config
    }


    /* init car */
    car_stat.current_speed           = NORMAL_SPEED;
    car_stat.current_wheel_angle     = 0;
    car_stat.car_id                  = car_id;
    car_stat.obstacle_stop           = 0;
    car_stat.travel_direction        = 0;
    car_stat.drive_thru              = 0;

    /* init image data */
    img_data.avg_slope                 = 0;
    img_data.old_slope                 = 0;
    img_data.left_line_length          = 0;
    img_data.right_line_length         = 0;
    img_data.intersection_distance     = -1;
    img_data.intersection_detected     = 0;
    img_data.intersection_stop         = 0;
    img_data.intersection_colour       = -1;
    img_data.obstacle_detected         = 0;
    img_data.intersection_type         = 0;

    /* init video capture */
    cap.set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    cap.set( CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set( CV_CAP_PROP_FRAME_HEIGHT, 480);
    cap.open();

    /* init bluetooth server */
    sig_resp = (struct SignalResponse*) calloc(1, sizeof(*sig_resp));
    if (quickstart_mode) 
        sig_resp->val = PROCEED_RESP;
    else
        sig_resp->val = STOP_RESP;

    pthread_t server_thread;
    pthread_t sender_thread;
    pthread_create(&server_thread, NULL, recvFromIC, (void*) sig_resp);
    pthread_create(&sender_thread, NULL, threaded_send, NULL);

    /* init hardware */
    status &= vichw_init(pwm_offset);

    return status;
}





int run() {

	int status = 1;
    int iterations = 0;
    unsigned long long t1 = 0, time_diff = 0;
    unsigned long long running_time = 0, time_start = 0, time_end = 0;


    while (status != HALT_SYSTEM) {
       

        //check for IC response
        if (sig_resp->val != PROCEED_RESP) {
            if (sig_resp->val == EMERGENCY_STOP_RESP) {
                printf("EMERGENCY_STOP_RESP\n");
                status = HALT_SYSTEM;
                kill_send_thread = 1;
                break;                
            }
            else {
                pause_sys();
            }
        } 
        status = get_lane_statusv3(&img_data, &cap);
        //send a  DEPARTURE message 
        if (img_data.intersection_stop == 1  && car_stat.drive_thru == 1 && time_diff > 5000 && !car_stat.obstacle_stop) {
            t1 = getMsTime();
            car_stat.drive_thru = 0;
            img_data.intersection_stop = 0;

            pthread_mutex_lock(&mutex);
            wake_thread = 1;
            pthread_cond_signal(&cond);
            pthread_mutex_unlock(&mutex);
            printf("%llu\n", time_diff);

            time_diff = getMsTime() - t1;

        }

        //send an ARRIVAL message
        if (img_data.intersection_stop == 1 && car_stat.drive_thru == 0 && time_diff > 6000 && !car_stat.obstacle_stop) {
            t1 = getMsTime();            
            car_stat.travel_direction = 0;
            stop_at_intersection();
            status = get_lane_statusv3(&img_data, &cap);
            printf("drive_thru\n");
            time_diff = getMsTime() - t1;   
            
        }

        if (status == CORRUPT_IMAGE) {
            continue;
        }

        status = update_navigation(&img_data, &car_stat, p, d, q);

        time_diff = getMsTime() - t1;
        
    }


    kill_send_thread = 1;
    printf("ending with status %d\n", status);



    return 1;
}




void* threaded_send(void* arg) {

    while (!kill_send_thread) {
        pthread_mutex_lock(&mutex);
        while(!wake_thread) {
            pthread_cond_wait(&cond, &mutex);
        }

        //all that matters in this msg is the first and last values.
        char msg[1024];
        sprintf(msg, "%d_%d_%d_%d_%d", 1, 0, 0, 0, car_stat.car_id);

        int sent = sendToIC(msg);
        printf("\x1b[33m DEPARTURE msg: (%s) with size of %d\x1b[0m \n ", msg, sent );

        wake_thread = 0;

        pthread_mutex_unlock(&mutex);

    }

    return 0;
}






//create message, stop the car from proceeding and enter pause state
void stop_at_intersection() {
    
    stop_car();
    reset_wheel();
    sig_resp->val = STOP_RESP;


    char msg[1024];
    sprintf(msg, "%d_%d_%d_%d_%d", 0, img_data.intersection_colour,  car_stat.travel_direction, 1, car_stat.car_id);

    int sent = sendToIC(msg);
    printf("\x1b[36m ARRIVAL msg: (%s) with size of %d\x1b[0m \n ", msg, sent );

    while (sig_resp->val == STOP_RESP){};

    img_data.intersection_detected = 0;
    img_data.intersection_type     = 0;
    img_data.intersection_stop     = 0;
    img_data.intersection_distance = -1;
    img_data.intersection_colour   = -1;
    car_stat.drive_thru            = 1;
    
}


//when called, program will remain paused until necessary BT responses are received
void pause_sys() {

    printf("Entering pause state\n");
    stop_car();

    while (sig_resp->val == STOP_RESP);

    if (sig_resp->val == EMERGENCY_STOP_RESP) {
        kill_send_thread = 1;
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






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
#include <raspicam/raspicam_cv.h>

    

CarStatus car_stat;
ImageData img_data;
SignalResponse *sig_resp;
raspicam::RaspiCam_Cv cap;


double p = 3;
double d = 1.5;    
double q = 2;




unsigned long long getMsTime() {
    struct timeval t;
    gettimeofday(&t, NULL);
         long  ms = (t.tv_sec)*1000 + (t.tv_usec)/1000;
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
        printf("%llu\n", getMsTime());
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

    /* init car */
    car_stat.current_speed           = NORMAL_SPEED;
    car_stat.current_wheel_angle     = 0;
    car_stat.car_id                  = CAR_ID;
    car_stat.obstacle_stop           = 0;
    car_stat.travel_direction        = 0;

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
    pthread_create(&server_thread, NULL, recvFromIC, (void*) sig_resp);


    /* init hardware */
    status &= vichw_init();

    return status;
}




int run() {

	int status = 1;
    unsigned long long  t1 = 0;
    unsigned long long time_diff = 0;


    unsigned long long running_time = 0;
    int iterations = 0;
    unsigned long long time_start = 0, time_end = 0;

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
        printf("time_diff = %llu\n", time_diff);
        if (img_data.intersection_stop == 2 && time_diff > 10000) {
        // if (img_data.intersection_stop == 1) {
            t1 = getMsTime();            
            car_stat.travel_direction = 0;
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

    return 1;
}



//TODO: PLEASE FIX ME :(

//create message, stop the car from proceeding and enter pause state
void stop_at_intersection() {

    stop_car();
    sig_resp->val = STOP_RESP;

    //build msg "ID_ComingFrom_GoingTo_Time"
    char* msg = (char*) malloc(sizeof(char)*50);
    sprintf(msg, "%d_%d_%d_%d_%d", car_stat.car_id, img_data.intersection_colour,  car_stat.travel_direction,1, 0);

    int sent = sendToIC(msg);
    printf("\x1b[36m sent msg: (%s) with size of %d\x1b[0m \n ", msg, sent );

    while (sig_resp->val == STOP_RESP){};

    img_data.intersection_detected = 0;
    img_data.intersection_type     = 0;
    img_data.intersection_stop     = 0;
    img_data.intersection_distance = -1;
    img_data.intersection_colour   = -1;
    free(msg);
    
}


//when called, program will remain paused until necessary BT responses are received
void pause_sys() {

    printf("Entering pause state\n");
    stop_car();

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






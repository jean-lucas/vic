#ifndef LANEDETECT_H    
#define LANEDETECT_H

#include "vic_types.h"
#include <raspicam/raspicam_cv.h>

//constants
const int DEFAULT_CAMERA_ID = 0;
//maximum height difference between two points of a line to consider it straight
const float STRAIGHT_LINE_THRESHOLD = 100;  

//multiple a pixel distance to get a meter distance
const float PIXEL_TO_METER_FACTOR = 0.5;


void test_camera(raspicam::RaspiCam_Cv* cap);
void calibrate_raspicam(raspicam::RaspiCam_Cv *cap);
int get_lane_statusv3(struct ImageData *img_data, raspicam::RaspiCam_Cv *cap);

#endif


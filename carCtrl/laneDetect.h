#ifndef LANEDETECT_H    
#define LANEDETECT_H

#include "vic_types.h"

//constants
const int DEFAULT_CAMERA_ID = 0;
//maximum height difference between two points of a line to consider it straight
const float STRAIGHT_LINE_THRESHOLD = 50;  

//multiple a pixel distance to get a meter distance
const float PIXEL_TO_METER_FACTOR = 0.5;


int get_lane_status(struct ImageData *img_data);
int test_camera();



#endif

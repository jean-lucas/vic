#ifndef LANEDETECT_H    
#define LANEDETECT_H

#include "vic_types.h"

//constants
const int DEFAULT_CAMERA_ID = 0;



int get_lane_status(struct Imageta *img_data);
int test_camera();



#endif

#ifndef VEHICLE_NAVIGATION_H
#define VEHICLE_NAVIGATION_H

#include "vic_types.h"
//Constants
const double MAX_SPEED = 1.4; //metres per second
const double MAX_ANGLE = 45; //degrees
const double ANGLE_THRESHOLD =  12; //degrees
const double CENTER_ADJUST_ANGLE = 10;
int update_navigation(struct ImageData *img, struct CarStatus *car);

#endif

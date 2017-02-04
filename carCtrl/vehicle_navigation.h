#ifndef VEHICLE_NAVIGATION_H
#define VEHICLE_NAVIGATION_H

//Constants
const double MAX_SPEED = 1.4; //metres per second
const double MAX_ANGLE = 45; //degrees
const double ANGLE_THRESHOLD =  15; //degrees

int update_navigation(struct *ImageData, struct *CarStatus);

#endif
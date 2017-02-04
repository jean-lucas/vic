#ifndef VEHICLE_NAVIGATOIN_H
#define VEHICLE_NAVIGATION_H

//Constants
double MAX_SPEED = 1.4; //metres per second
double MAX_ANGLE = 45; //degrees
double ANGLE_THRESHOLD =  15; //degrees

int update_navigation(struct *ImageData, struct *CarStatus);

#endif
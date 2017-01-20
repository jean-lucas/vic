#ifndef LANEDETECT_H    
#define LANEDETECT_H


//constants
const int DEFAULT_CAMERA_ID = 0;



int getLaneStatus();

//get midpoint of a line 
Point getMidpoint(Point a, Point b);
double calculateAvgAngle(vector<Point> vec, Point center);
double calculateAvgLineSize(vector<Point> vec, Point center);
double lineLength(Point a, Point b);



#endif
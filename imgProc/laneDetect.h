#ifndef LANEDETECT_H    
#define LANEDETECT_H



int getLaneStatus(const char* pathName);

//get midpoint of a line 
Point getMidpoint(Point a, Point b);
double calculateAvgAngle(vector<Point> vec, Point center);
double calculateAvgLineSize(vector<Point> vec, Point center);
double lineLength(Point a, Point b);



#endif
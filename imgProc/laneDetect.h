#ifndef LANEDETECT_H    
#define LANEDETECT_H


//constants
const int DEFAULT_CAMERA_ID = 0;



//digital information pertaining to the image capture
struct imageData {

	float avgLeftAngle, avgRightAngle;
	float avgLeftDistance, avgRightDistance;
	float distanceToObject;

	bool intersectionDetected;
	bool objectDetected;
};



int getLaneStatus();




#endif

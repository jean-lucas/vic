#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <time.h>
#include "laneDetect.h"

using namespace cv;
using namespace std;

//this percentage will be cutoff from the top of the image
const double CUT_OFF_HEIGHT_FACTOR = 0.2;


Point2d getMidpoint(Point2d a, Point2d b);
double calculateAvgAngle(vector<Point2d> vec, Point2d center);
double calculateAvgLineSize(vector<Point2d> vec, Point2d center);
double lineLength(Point2d a, Point2d b);
double getDistanceToLine(Point2d a, Point2d b);


/* 
	Test if camera can capture images/videos 
	If successful return Point2der to VideoCapture object
	Else return null Point2der
*/
VideoCapture test_camera() {

	VideoCapture cap(DEFAULT_CAMERA_ID);

	if (!cap.isOpened()) {
		printf("failed to open capture\n");
		exit(0);
	}
	return cap;
}




int get_lane_status(struct ImageData *img_data, VideoCapture *cap) {

	if (!(cap->isOpened())) {
		printf("failed to open capture\n");
		cap->release();
		return 0;
	}


	Mat capMat, croppedMat, cannyMat, houghMat;

	//retrieve the current frame
	cap->read(capMat);

	//cropping region
	Size size_uncropped      = capMat.size();
	int img_height_uncropped = size.height;
	int img_width_uncropped  = size.width;
	Rect cropRect = Rect(0, img_height_uncropped, img_width_uncropped, img_height_uncropped*CUT_OFF_HEIGHT_FACTOR);
	croppedMat = img(Rect);



	//finds edges in the capMatMath  via the Canny Edge detection algorithm, and puts 
	//result in cannyMat
	//Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
	Canny(croppedMat, cannyMat, 50, 200, 3);
//	Canny(capMat, cannyMat, 55, 110, 3);

	//converts img in cannyMat to another colour space and puts it in houghMat
	cvtColor(cannyMat, houghMat, CV_GRAY2BGR);



	//basic information of the image
	Size size = houghMat.size();
	int imgHeight = size.height;
	int imgWidth  = size.width;
	Point2d centerPoint = Point2d(imgWidth/2.0, imgHeight);
	circle(houghMat,centerPoint,5,Scalar(255,150,50),2,LINE_8,0);

	//draw the cutoff point, can be removed after.
//	line(houghMat, Point2d(0,imgHeight*CUT_OFF_HEIGHT_FACTOR), Point2d(imgWidth, imgHeight*CUT_OFF_HEIGHT_FACTOR), Scalar(255,255,0),5,8);


	vector<Vec4f> lines;
	vector<Point2d> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
	vector<Point2d> rightLines;


	//(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
	HoughLinesP(cannyMat, lines, 1, CV_PI/180,50,2,1);

	//to  detect an intersection, we can look for the horizontal lines across the lane.
	//for  this we can just compare the y values between two points of a line.
	bool intersectionDetected = false;
	double estimatedIntersectionDistance = 0;


	//to detect an obstacle, we can try to find a countour  within our image, find the area of the given contour
	//and thus determnine if an obstacle  is ahead or not.
	//Instead of  using contours we can follow a similar approach to the intersection detection above
	bool  obstacleDetected = false;

	//draw detected lines
	for (size_t i = 0; i < lines.size(); i++) {
		Point2d a = Point2d(lines[i][0],lines[i][1]);
		Point2d b = Point2d(lines[i][2],lines[i][3]);

		// //if either Point A or Point B lie above the cutoff section we can ignore it.
		// if (a.y <= imgHeight*CUT_OFF_HEIGHT_FACTOR || b.y <= imgHeight*CUT_OFF_HEIGHT_FACTOR) {
		// 	continue;
		// }


		Point2d mid = getMidpoint(a,b);


		//if we have already detected an intersection in the current frame
		//no point of continuing to check
/*		if (!intersectionDetected) {
			if ( fabs(a.y-b.y) <= STRAIGHT_LINE_THRESHOLD) {
				printf("found intersection at points (%f %f) to (%f %f)\n", a.x, a.y, b.x, b.y);
				circle(houghMat,a,10,Scalar(255,60,200));
				circle(houghMat,b,10,Scalar(255,60,200));
				intersectionDetected = true;
			}
		}

		else if (estimatedIntersectionDistance <= 0) {
			estimatedIntersectionDistance = getDistanceToLine(mid, centerPoint);
			printf("estimatedDistance to intersection = %f pixels\n", estimatedIntersectionDistance);
		} 

*/
		//if an end-point of a line plus the midpoint are to one side of the img center,
		// than consider which side it is on,
		//else IGNORE THE LINE (may need to fix this)
		if (mid.x <= centerPoint.x && (a.x <= centerPoint.x || b.x <= centerPoint.x)) {
			leftLines.push_back(mid);
		}

		else if (mid.x >= centerPoint.x && (a.x >= centerPoint.x || b.x >= centerPoint.x)) {
			rightLines.push_back(mid);
		}


	}



	double theta1, theta2;
	theta1 = calculateAvgAngle(leftLines, centerPoint)*(180.0/CV_PI);
	theta2 = calculateAvgAngle(rightLines,centerPoint)*(180.0/CV_PI);

	double avgLeftSize, avgRightSize;
	avgLeftSize = calculateAvgLineSize(leftLines, centerPoint);
	avgRightSize = calculateAvgLineSize(rightLines, centerPoint);

//	printf("Theta1: %f \tTheta2: %f \n", calculateAvgAngle(leftLines, centerPoint), theta2);
	//printf("leftLine: %f \trightLine: %f \n",avgLeftSize, avgRightSize);

	img_data->avg_left_angle 	= theta1;
	img_data->avg_right_angle 	= theta2;
	img_data->left_line_length 	= avgLeftSize;
	img_data->right_line_length 	= avgRightSize;
	img_data->intersection_distance = estimatedIntersectionDistance;
	img_data->intersection_detected = intersectionDetected;
	img_data->obstacle_detected 	= obstacleDetected;


//	imshow("capMat", capMat);
//	imshow("Canny", cannyMat);
//	imshow("Hough", houghMat);
//	waitKey();
	return 1;
}




double calculateAvgLineSize(vector<Point2d> vec, Point2d center) {
	double current=0;
	int n = vec.size();

	if (n < 1) return 0;

	for (int i = 0; i < n; i++) {
		current += sqrt(pow(vec[i].x - center.x, 2) + pow(vec[i].y - center.y, 2));
	}
	return current/(double)n;
}

double calculateAvgAngle(vector<Point2d> vec, Point2d center) {
	double currAngle = 0, top = 0, bot = 0, frac = 0, temp = 0;
	int n = vec.size();

	if (n < 1) return 0;

	for (int i = 0; i < n; i++) {
		top = fabs(vec[i].x - center.x);
		bot = sqrt( pow(vec[i].x - center.x, 2) + pow(vec[i].y - center.y, 2));
		frac = top/bot;
		if (frac < -1.0) {
			frac = -1.0;
		}
		else if (frac > 1.0) {
			frac = 1.0;
		}
		temp = acos(frac);
		if (temp <= 3.2) {
			currAngle = currAngle + temp;
		}
	}
	return currAngle/((double) n);

}


Point2d getMidpoint(Point2d a, Point2d b) {
	double midX = (a.x + b.x)/2.0;
	double midY = (a.y + b.y)/2.0;
	return Point2d(midX, midY);
}

double getDistanceToLine(Point2d a, Point2d b) {
	double distance = sqrt(pow(  a.x - b.x, 2) + pow(  a.y - b.y, 2));
	return distance;
}




/**
	For testing purposes.
	It will take an image of the track, and show all the lines it has captured
	Image will be saved into lanecap.png
*/
int capture_lane(VideoCapture *cap) {

		if (!(cap->isOpened())) {
			printf("failed to open capture\n");
			cap->release();
			return 0;
			
		}


		Mat capMat, cannyMat, houghMat, transCap, transMat;
		cap->read(capMat);



		//finds edges in the capMatMath  via the Canny Edge detection algorithm, and puts 
		//result in cannyMat
		//Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
		Canny(capMat, cannyMat, 80, 200, 3);
	//	Canny(capMat, cannyMat, 55, 110, 3);

		//converts img in cannyMat to another colour space and puts it in houghMat
		cvtColor(cannyMat, houghMat, CV_GRAY2BGR);



		//basic information of the image
		Size size = houghMat.size();
		int imgHeight = size.height;
		int imgWidth  = size.width;
		Point2d centerPoint = Point2d( imgWidth/2.0, imgHeight);
		circle(houghMat,centerPoint,5,Scalar(255,150,50),2,LINE_8,0);

		//draw the cutoff point, can be removed after.
		line(houghMat, Point2d(0,imgHeight*CUT_OFF_HEIGHT_FACTOR), Point2d(imgWidth,imgHeight*CUT_OFF_HEIGHT_FACTOR), Scalar(255,155,0),5,8);

		vector<Vec4f> lines;
		vector<Point2d> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
		vector<Point2d> rightLines;



		//(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
		HoughLinesP(cannyMat, lines, 1, CV_PI/180,50,3,5);


		//to  detect an intersection, we can look for the horizontal lines across the lane.
		//for  this we can just compare the y values between two points of a line.
		bool intersectionDetected = false;
		double estimatedIntersectionDistance = 0;

		//to detect an obstacle, we can try to find a countour  within our image, find the area of the given contour
		//and thus determnine if an obstacle  is ahead or not.
		//Instead of  using contours we can follow a similar approach to the intersection detection above
		bool  obstacleDetected = false;


		//draw detected lines
		for (size_t i = 0; i < lines.size(); i++) {
			Point2d a = Point2d( lines[i][0], lines[i][1]);
			Point2d b = Point2d( lines[i][2], lines[i][3]);


			//if either Point A or Point B lie above the cutoff section we can ignore it.
                	if (a.y <= imgHeight*CUT_OFF_HEIGHT_FACTOR || b.y <= imgHeight*CUT_OFF_HEIGHT_FACTOR) {
                 		continue;
                	}

			Point2d mid = getMidpoint(a,b);

			//if we have already detected an intersection in the current frame
			//no point of continuing to check
			if (!intersectionDetected) {
				if ( fabs(a.y-b.y) <= STRAIGHT_LINE_THRESHOLD) {
					printf("found intersection at points (%f %f) to (%f %f)\n", a.x, a.y, b.x, b.y);
					circle(houghMat,a,10,Scalar(255,60,200));
					circle(houghMat,b,10,Scalar(255,60,200));
					line(houghMat, a,b, Scalar(255,50,215),5,8);
					intersectionDetected = true;
				}
			}

			else if (estimatedIntersectionDistance <= 0) {
				estimatedIntersectionDistance = getDistanceToLine(mid, centerPoint);
				printf("estimatedDistance to intersection = %f pixels\n", estimatedIntersectionDistance);
			} 


			//if an end-point of a line plus the midpoint are to one side of the img center,
			// than consider which side it is on,
			//else IGNORE THE LINE (may need to fix this)
			if (mid.x <= centerPoint.x && (a.x <= centerPoint.x || b.x <= centerPoint.x)) {
				leftLines.push_back(mid);
			}

			else if (mid.x >= centerPoint.x && (a.x >= centerPoint.x || b.x >= centerPoint.x)) {
				rightLines.push_back(mid);
			}
			else {
				printf("ignoring a line...\n");
			}


			circle(houghMat,mid,5,Scalar(255,100,0));

			line(houghMat, mid, centerPoint, Scalar(0,255,0),1,8);
			line(houghMat, a, b, Scalar(0,0,255),3,8);


		}

		// Point2f  preTrans[4];
		// preTrans[0] = Point2f(topLeft.x, topLeft.y);
		// preTrans[1] = Point2f(topRight.x,topRight.y);
		// preTrans[2] = Point2f(0,imgHeight);
		// preTrans[3] = Point2f(imgWidth,imgHeight);


		// Point2f  postTrans[4];
  //       postTrans[0] = Point2f(0,0);
  //       postTrans[1] = Point2f(imgWidth,0);
  //       postTrans[2] = Point2f(0,imgHeight);
  //       postTrans[3] = Point2f(imgWidth,imgHeight);


		
		// transCap = getPerspectiveTransform(preTrans,postTrans);
//		warpPerspective(cannyMat, transMat,transCap,size); 
		imwrite("../../lanecap_canny.png", cannyMat);
		imwrite("../../lanecap.png", houghMat);
		// imwrite("../../lanecap_transform.png", transMat);
		return 0;
}

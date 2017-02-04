#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <algorithm>
#include <math.h>

#include "laneDetect.h"

using namespace cv;
using namespace std;


Point getMidpoint(Point a, Point b);
double calculateAvgAngle(vector<Point> vec, Point center);
double calculateAvgLineSize(vector<Point> vec, Point center);
double lineLength(Point a, Point b);
double getDistanceToLine(Point a, Point b);
/* Test if camera can capture images/videos */
int test_camera() {

	VideoCapture cap(DEFAULT_CAMERA_ID);

	if (!cap.isOpened()) {
		printf("failed to open capture\n");
		return 0;
	}
	return 1;
}


int get_lane_status(struct ImageData *img_data) {

//	const char* imgFile = pathName;
//	const char* imgFile = "road.jpg";
//	const char* imgFile = "straightRoad.jpg";
//	const char* imgFile = "roadWithStop.jpg";
//	const char* imgFile = "roadWithInt.jpg";
	const char* imgFile = "track1.jpg";

//	Mat capMat = imread(imgFile, -1);

//	if (capMat.empty()) {
//		printf("could not load img \n");
//		exit(0);
//	}

//	VideoCapture cap(DEFAULT_CAMERA_ID);

//	if (!cap.isOpened()) {
//		printf("failed to open capture\n");
//		return 0;
//	}


	Mat capMat, cannyMat, houghMat;

	capMat = imread(imgFile, -1);
	//retrieve the current frame
//	cap >> capMat;

	//close camera
//	cap.release();



	//finds edges in the capMatMath  via the Canny Edge detection algorithm, and puts 
	//result in cannyMat
	//Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
	Canny(capMat, cannyMat, 50, 200, 3);
//	Canny(capMat, cannyMat, 55, 110, 3);

	//converts img in cannyMat to another colour space and puts it in houghMat
	cvtColor(cannyMat, houghMat, CV_GRAY2BGR);



	//basic information of the image
	Size size = houghMat.size();
	int imgHeight = size.height;
	int imgWidth  = size.width;
	Point centerPoint = Point(imgWidth/2.0, imgHeight);
	circle(houghMat,centerPoint,5,Scalar(255,150,50),2,LINE_8,0);


	vector<Vec4f> lines;
	vector<Point> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
	vector<Point> rightLines;
	
	vector<vector<Point> > contours;
	vector<Vec4i> contour_hier;

	//(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
	HoughLinesP(cannyMat, lines, 1, CV_PI/180,50,5,1);

	//
	findContours(cannyMat, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

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
		Point a = Point(lines[i][0],lines[i][1]);
		Point b = Point(lines[i][2],lines[i][3]);
		Point mid = getMidpoint(a,b);

		//if we have already detected an intersection in the current frame
		//no point of continuing to check
		if (!intersectionDetected) {
			if ( fabs(a.y-b.y) <= STRAIGHT_LINE_THRESHOLD) {
				printf("found intersection at points (%d %d) to (%d %d)\n", a.x,a.y,b.x,b.y);
				circle(houghMat,a,10,Scalar(255,60,200));
				circle(houghMat,b,10,Scalar(255,60,200));
				intersectionDetected = true;
			}
		}

		else if (estimatedIntersectionDistance <= 0) {
			estimatedIntersectionDistance = getDistanceToLine(mid, centerPoint) * PIXEL_TO_METER_FACTOR;
			printf("estimatedDistance to intersection = %f meters or %f pixels\n", estimatedIntersectionDistance, getDistanceToLine(mid, centerPoint));
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

		//necessary lines
		line(houghMat, mid, centerPoint, Scalar(0,255,0),1,8);
		line(houghMat, a, b, Scalar(0,0,255),3,8);


	}


	//detected contours
	for (size_t i = 0; i < contours.size(); i++) {

//		if (isContourConvex(contours[i])) {
			double area = contourArea(contours[i]);
			printf("area = %f\t", area);
			printf("position (%d , %d)\n", contours[i][0].x, contours[i][0].y);
			circle(houghMat,Point(contours[i][0].x, contours[i][0].y),15,Scalar(255,200,0));
//		}
	}


	double theta1, theta2;
	theta1 = calculateAvgAngle(leftLines, centerPoint);
	theta2 = calculateAvgAngle(rightLines,centerPoint);


	double avgLeftSize, avgRightSize;
	avgLeftSize = calculateAvgLineSize(leftLines, centerPoint);
	avgRightSize = calculateAvgLineSize(rightLines, centerPoint);

	printf("Theta1: %f \nTheta2: %f \n", theta1, theta2);
	printf("leftLine: %f \trightLine: %f \n",avgLeftSize, avgRightSize);

	img_data->avg_left_angle 	= theta1;
	img_data->avg_right_angle 	= theta2;
	img_data->left_line_length 	= avgLeftSize;
	img_data->right_line_length 	= avgRightSize;
	img_data->intersection_distance = estimatedIntersectionDistance;
	img_data->intersection_detected = intersectionDetected;
	img_data->obstacle_detected 	= obstacleDetected;


//	imshow("capMat", capMat);
//	imshow("Canny", cannyMat);
	imshow("Hough", houghMat);
	waitKey();

	return 0;
}




double calculateAvgLineSize(vector<Point> vec, Point center) {
	double current;
	int n = vec.size();

	if (n < 1) return 0;

	for (int i = 0; i < n; i++) {
		current += sqrt(pow(vec[i].x - center.x, 2) + pow(vec[i].y - center.y, 2));
	}
	return current/(double)n;
}

double calculateAvgAngle(vector<Point> vec, Point center) {
	double currAngle, top, bot;
	int n = vec.size();

	if (n < 1) return 0;

	for (int i = 0; i < n; i++) {
		top = fabs(vec[i].x - center.x);
		bot = sqrt( pow(vec[i].x - center.x, 2) + pow(vec[i].y - center.y, 2));
		currAngle += acos(top/bot);
	}

	return currAngle/((double) n);

}


Point getMidpoint(Point a, Point b) {
	double midX = (a.x + b.x)/2.0;
	double midY = (a.y + b.y)/2.0;
	return Point(midX, midY);
}

double getDistanceToLine(Point a, Point b) {

	double distance = sqrt(pow( (double) a.x - b.x, 2) + pow( (double) a.y - b.y, 2));
	return distance;
}





/*
// for contours
	vector<vector<Point> > contours;
	vector<Vec4i> hier;
	findContours(cannyMat, contours, hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	Mat drawing = Mat::zeros(cannyMat.size(), CV_8UC3);

	printf("Calculating arc length of contours \n\n");
	double arc = 0;
	for (int i = 0; i < contours.size(); i++) {
		arc = arcLength(contours[i],false);
		if (arc > 100) {
		  for (int j = 0; j < contours[i].size(); j++) {
			printf("( %d, %d )    ", contours[i][j].x, contours[i][j].y);
   		  }
		  printf("\n\n");
		  printf("(x, y) -> ( x2, y2) (%d, %d) ->  (%d, %d) \t", contours[i][0].x, contours[i][0].y, contours[i][1].x, contours[i][1].y);
		  printf("i = %d \t arcLength = %f \n",i,arc);
		  Scalar color = Scalar(255,i*20,255);
		  drawContours(drawing, contours, i, color, 2,8,hier,0,Point());
		}
	}

		imshow("Contour", drawing);
	*/

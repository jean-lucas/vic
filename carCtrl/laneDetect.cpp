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

int getLaneStatus(const char* pathName) {

	const char* imgFile = pathName;
//	const char* imgFile = "road.jpg";
//	const char* imgFile = "straightRoad.jpg";
//	const char* imgFile = "roadWithStop.jpg";

	Mat source = imread(imgFile, -1);

	if (source.empty()) {
		printf("could not load img \n");
		exit(0);
	}

	Mat cannyMat, houghMat;


	//finds edges in the sourceMath  via the Canny Edge detection algorithm, and puts 
	//result in cannyMat
	//Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
	Canny(source, cannyMat, 65, 100, 3);

	//converts img in cannyMat to another colour space and puts it in houghMat
	cvtColor(cannyMat, houghMat, CV_GRAY2BGR);

	
	//use prob Hough trans

	//basic information of the image
	Size size = houghMat.size();
	int imgHeight = size.height;
	int imgWidth  = size.width;
	Point centerPoint = Point(imgWidth/2, imgHeight);
	circle(houghMat,centerPoint,5,Scalar(255,150,50),2,LINE_8,0);


	vector<Vec4i> lines;
	vector<Point> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
	vector<Point> rightLines;
	Scalar color;

	//(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
	HoughLinesP(cannyMat, lines, 1, CV_PI/180,50,100,50);


	for (size_t i = 0; i < lines.size(); i++) {
		Point a = Point(lines[i][0],lines[i][1]);
		Point b = Point(lines[i][2],lines[i][3]);
		Point mid = getMidpoint(a,b);

		
		//if an end-point of a line plus the midpoint are to one side of the img center,
		// than consider which side it is on,
		//else IGNORE THE LINE (may need to fix this)
		if (mid.x <= centerPoint.x && (a.x <= centerPoint.x || b.x <= centerPoint.x)) {
			leftLines.push_back(mid);
			color = Scalar(255,0,150);
		}

		else if (mid.x >= centerPoint.x && (a.x >= centerPoint.x || b.x >= centerPoint.x)) {
			rightLines.push_back(mid);
			color = Scalar(255,80,50);

		}
		else {
			printf("ignoring a line...\n");
			color = Scalar(255,110,150);

		}

		circle(houghMat,mid,5,Scalar(255,100,0));

		//necessary lines
		line(houghMat, mid, centerPoint, color,1,8);
		line(houghMat, a, b, Scalar(0,0,255),3,8);

	}

	printf("left lines: %d \t right lines: %d \n", leftLines.size(), rightLines.size());

	double theta1 = calculateAvgAngle(leftLines, centerPoint);
	double theta2 = calculateAvgAngle(rightLines,centerPoint);

	printf("Theta1: %f \t Theta2: %f \n", theta1, theta2);


	//if there is about the same number of lines on both sides, consider measuring the avg line length to
	//get the lane position
	double avgLeftSize, avgRightSize;
//	if ( abs(leftLines.size() - rightLines.size())  <= 10) {
		avgLeftSize = calculateAvgLineSize(leftLines, centerPoint);
		avgRightSize = calculateAvgLineSize(rightLines, centerPoint);

		printf("avg line sizes:  left: %f \t right: %f \n", avgLeftSize, avgRightSize);
//	}
//	else {
//		printf("Line sizes not calculated \n");
//	}


//	imshow("source", source);
//	imshow("Canny", cannyMat);
	imshow("Hough", houghMat);
	waitKey();
	return 0;
}



double lineLength(Point a, Point b) {
	return sqrt( pow(a.x - b.x, 2) + pow(a.y - b.y, 2) );
}

double calculateAvgLineSize(vector<Point> vec, Point center) {
	double current;
	int n = vec.size();
	for (int i = 0; i < n; i++) {
		current += sqrt( pow(vec[i].x - center.x, 2) + pow(vec[i].y - center.y, 2));
		//current += lineLength(vec[i], center);
	}
	return current/((double) n);
}

double calculateAvgAngle(vector<Point> vec, Point center) {
	double currAngle, top, bot;
	int n = vec.size();
	for (int i = 0; i < n; i++) {
		top = fabs(vec[i].x - center.x);
		bot = sqrt( pow(vec[i].x - center.x, 2) + pow(vec[i].y - center.y, 2));
		currAngle += acos(top/bot);
		// printf("angle (rad)  (deg)= %f   %f \n", top/bot, (180/CV_PI)*top/bot);
	}

	return currAngle/((double) n);

}


Point getMidpoint(Point a, Point b) {
	return Point( (a.x+b.x)/2, (a.y+b.y)/2 );
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

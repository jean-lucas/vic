#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <algorithm>
#include <math.h>
using namespace cv;
using namespace std;

//get midpoint of a line 
Point getMidpoint(Point a, Point b);
double calculateAvgAngle(vector<Point> vec, Point center, int n);


int main(int argc, char** argv) {

	const char* imgFile = "road.jpg";
//	const char* imgFile = "straightRoad.jpg";
//	const char* imgFile = "roadWithStop.jpg";

	Mat source = imread(imgFile, -1);

	if (source.empty()) {
		printf("could not load img \n");
		exit(0);
	}

	Mat cannyDetect1, houghTrans;
	Canny(source, cannyDetect1, 65, 100, 3);


	cvtColor(cannyDetect1, houghTrans, CV_GRAY2BGR);

	// for contours
	vector<vector<Point> > contours;
	vector<Vec4i> hier;
	findContours(cannyDetect1, contours, hier, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0,0));
	Mat drawing = Mat::zeros(cannyDetect1.size(), CV_8UC3);

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

	//use prob Hough trans
		Size size = houghTrans.size();
		int imgHeight = size.height;
		int imgWidth  = size.width;
		Point centerPoint = Point(imgWidth/2, imgHeight);
		circle(houghTrans,centerPoint,5,Scalar(255,150,50),2,LINE_8,0);

		Mat color_dst;
		vector<Vec4i> lines;
		vector<Point> leftLines;
		vector<Point> rightLines;
		HoughLinesP(cannyDetect1, lines, 1, CV_PI/180,50,10,40);
		for (size_t i = 0; i < lines.size(); i++) {
			Point a = Point(lines[i][0],lines[i][1]);
			Point b = Point(lines[i][2],lines[i][3]);
			Point mid = getMidpoint(a,b);

			//if the max X value of a point is less than width/2
			//that means the line comes from the left side of the center
			//draw center circle
			if (max(a.x, b.x) <= imgWidth/2)
				leftLines.push_back(mid);
			else
				rightLines.push_back(mid);
			circle(houghTrans,mid,5,Scalar(255,100,0));

			//necessary lines
			line(houghTrans, mid, centerPoint, Scalar(0,255,0),1,8);
			line(houghTrans, a, b, Scalar(0,0,255),3,8);


		}

		double theta1 = calculateAvgAngle(leftLines, centerPoint, min(leftLines.size(), rightLines.size()));
		double theta2 = calculateAvgAngle(rightLines,centerPoint,  min(leftLines.size(), rightLines.size()));

		printf("Theta1: %f \t Theta2: %f \n", theta1, theta2);


//	imshow("source", source);
//	imshow("Canny", cannyDetect1);
	imshow("Hough", houghTrans);
	waitKey();
	return 0;
}



double calculateAvgAngle(vector<Point> vec, Point center, int n) {
	double avgAngle,currAngle;
	for (int i = 0; i < n; i++) {
		double numerator = fabs(vec[i].x - center.x);
		double denom = sqrt( pow(vec[i].x - center.x, 2) + pow(vec[i].y - center.y, 2));
		currAngle += acos(numerator/denom);

		printf("angle (rad)  (deg)= %f   %f \n", numerator/denom, (180/CV_PI)*numerator/denom);
	}

	return currAngle/((double) n);

}


Point getMidpoint(Point a, Point b) {
	return Point( (a.x+b.x)/2, (a.y+b.y)/2 );
}


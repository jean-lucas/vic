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

struct avgLine {
	Point a,b;
};

int main(int argc, char** argv) {

	const char* imgFile = "road.jpg";
//	const char* imgFile = "straightRoad.jpg";
//	const char* imgFile = "roadWithStop.jpg";

	Mat source = imread(imgFile, -1);
	bool doProb = false;

	if (source.empty()) {
		printf("could not load img \n");
		exit(0);
	}

	Mat cannyDetect1, houghTrans;
	Canny(source, cannyDetect1, 65, 100, 3);


	cvtColor(cannyDetect1, houghTrans, CV_GRAY2BGR);

	if (doProb) {
	//Do hough line stuff on  cannyDetect
	vector<Vec2f> lines;

	HoughLines(cannyDetect1, lines, 1, CV_PI/180, 95, 0, 0);

	int lSize = lines.size();
	double rhoTot, thetaTot;


	for (size_t i = 0; i <  lines.size(); i++) {
		float rho = lines[i][0];
		float theta = lines[i][1];

		rhoTot += rho;
		thetaTot += theta;


		//draw the lines in the lines vector
		Point pt1, pt2;
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
		pt1.x = cvRound(x0 + 1000*(-b));
		pt1.y = cvRound(y0 + 1000*(a));
		pt2.x = cvRound(x0 - 1000*(-b));
		pt2.y = cvRound(y0 - 1000*(a));

		line(houghTrans, pt1, pt2, Scalar(0,0,255), 3, CV_AA);

		printf("Distance rho: %f \t Angle theta: %f\n", rho, theta);
		//imshow("Canny Lines", cannyDetect1);
		//imshow("Hough", houghTrans);

		//waitKey(2500);
	}

	//draw the average of the lines
	Point pt1, pt2;
	float rho = rhoTot/lSize;
	float theta = thetaTot/lSize;

        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));

        line(houghTrans, pt1, pt2, Scalar(0,255,0), 3, CV_AA);

	}

	//use prob Hough trans
	else {
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

	}

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


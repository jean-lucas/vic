#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
//#include <pthread.h>

using namespace cv;
using namespace std;


int main(int argc, char** argv) {

	const char* imgFile = "road.jpg";

	Mat source = imread(imgFile, -1);


	if (source.empty()) {
		printf("could not load img \n");
		exit(0);
	}

	Mat cannyDetect1, houghTrans;
	Canny(source, cannyDetect1, 65, 100, 3);

	
	cvtColor(cannyDetect1, houghTrans, CV_GRAY2BGR);


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
//		imshow("Canny Lines", cannyDetect1);
//		imshow("Hough", houghTrans);
		
//		waitKey(2000);
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


	imshow("source", source);
	imshow("Canny Lines 1", cannyDetect1);
	imshow("Hough", houghTrans);

	
	waitKey();



	
	return 0;


}

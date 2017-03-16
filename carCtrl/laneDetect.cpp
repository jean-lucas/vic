
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
const double CUT_OFF_HEIGHT_FACTOR = 0.40;
const double CUT_OFF_HEIGHT_FACTOR_BOTTOM = 0.03;

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


/*
usage:
string ty =  type2str( cannyMat.type() );
printf("Matrix: %s %dx%d \n", ty.c_str(), cannyMat.cols, cannyMat.rows );		
*/
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

int get_lane_statusv2( VideoCapture *cap) {

    if (!(cap->isOpened())) {
            printf("failed to open capture\n");
            cap->release();
            return 0;
    }


    Mat capMat, croppedMat, cannyMat, blurMat;

    //retrieve the current frame
    cap->read(capMat);


        //cropping region
	Size size_uncropped = capMat.size();
	int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
	Rect cropRect = Rect(0,new_height, size_uncropped.width, size_uncropped.height-new_height);
	croppedMat = capMat(cropRect);
	blur(croppedMat,blurMat,Size(3,3));



    imwrite("../../lanecap_blur.png", blurMat);

    Canny(blurMat, cannyMat, 50, 200, 3);

        
    cvtColor(cannyMat, cannyMat, CV_GRAY2BGR);
    cannyMat.convertTo(cannyMat, CV_32F);
    Size xx = cannyMat.size();

    printf(" CannyMat Size (%d %d) \n", xx.width, xx.height);

    //check positions - can rm after
    circle(cannyMat,Point(0,0),10,Scalar(240,0,0));
    circle(cannyMat,Point(xx.width,xx.height),10,Scalar(2,0,250));
    
    int i = 0;
    int j = 0;

    int numRows = xx.height;
    int num_segs = numRows/20;
    printf("num_segs = %d\n", num_segs );
    Vec3f colour;

    vector<Point2d> detectedPts
    //the value of j can start halfway of the width
    for ( i = 0; i < xx.height; i += num_segs) {
        printf(" On row %d\n", i );
        for ( j = 0; j < xx.width; j++) {

            colour = cannyMat.at<Vec3f>(Point(i,j));

            if (colour.val[0] + colour.val[1] + colour.val[2] >= 350) {
            	printf("detected at ( %d , %d )\n",i,j);
				circle(cannyMat,Point(i,j),5,Scalar(240,0,100));
				detectedPts.push_back(Point2d(i,j));
            }
        }
    }

    double tot = 0;
    double avg = 0;
    //get avg distance from center img to detected pts
    for (size_t k = 0; k < detectedPts.size(); k++) {
    	tot += abs(detectedPts[k].x - xx.width/2.0);
    }

    avg = tot/detectedPts.size();
    printf("avg distance is %f\n", avg);


	imwrite("../../lanecap_canny.png", cannyMat);

    return 0;
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
        int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
        Rect cropRect = Rect(0,new_height, size_uncropped.width, size_uncropped.height-new_height);
        croppedMat = capMat(cropRect);




        //finds edges in the capMatMath  via the Canny Edge detection algorithm, and puts
        //result in cannyMat
        //Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
        Canny(croppedMat, cannyMat, 50, 200, 3);
//      Canny(capMat, cannyMat, 55, 110, 3);

        //converts img in cannyMat to another colour space and puts it in houghMat
        cvtColor(cannyMat, houghMat, CV_GRAY2BGR);



        //basic information of the image
        Size size = houghMat.size();
        int imgHeight = size.height;
        int imgWidth  = size.width;
        Point2d camera_center_point = Point2d(imgWidth/2.0, imgHeight);



        vector<Vec4f> lines;
        vector<Point2d> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
        vector<Point2d> rightLines;


        //(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
        HoughLinesP(cannyMat, lines, 1, CV_PI/180,50,2,0);

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
                //      continue;
                // }


                Point2d mid = getMidpoint(a,b);


                //if we have already detected an intersection in the current frame
                //no point of continuing to check
/*              if (!intersectionDetected) {
                        if ( fabs(a.y-b.y) <= STRAIGHT_LINE_THRESHOLD) {
                                printf("found intersection at points (%f %f) to (%f %f)\n", a.x, a.y, b.x, b.y);
                                circle(houghMat,a,10,Scalar(255,60,200));
                                circle(houghMat,b,10,Scalar(255,60,200));
                                intersectionDetected = true;
                        }
                }

                else if (estimatedIntersectionDistance <= 0) {
                        estimatedIntersectionDistance = getDistanceToLine(mid, camera_center_point);
                        printf("estimatedDistance to intersection = %f pixels\n", estimatedIntersectionDistance);
                }

*/
                //if an end-point of a line plus the midpoint are to one side of the img center,
                // than consider which side it is on,
                //else IGNORE THE LINE (may need to fix this)
                if (mid.x <= camera_center_point.x && (a.x <= camera_center_point.x || b.x <= camera_center_point.x)) {
                        leftLines.push_back(mid);
                }

                else if (mid.x >= camera_center_point.x && (a.x >= camera_center_point.x || b.x >= camera_center_point.x)) {
                        rightLines.push_back(mid);
                }


        }



        double theta1, theta2;
        theta1 = calculateAvgAngle(leftLines, camera_center_point)*(180.0/CV_PI);
        theta2 = calculateAvgAngle(rightLines,camera_center_point)*(180.0/CV_PI);

        double avgLeftSize, avgRightSize;
        avgLeftSize = calculateAvgLineSize(leftLines, camera_center_point);
        avgRightSize = calculateAvgLineSize(rightLines, camera_center_point);

//      printf("Theta1: %f \tTheta2: %f \n", calculateAvgAngle(leftLines, camera_center_point), theta2);
        //printf("leftLine: %f \trightLine: %f \n",avgLeftSize, avgRightSize);

        img_data->avg_left_angle        = theta1;
        img_data->avg_right_angle       = theta2;
        img_data->left_line_length      = avgLeftSize;
        img_data->right_line_length     = avgRightSize;
        img_data->intersection_distance = estimatedIntersectionDistance;
        img_data->intersection_detected = intersectionDetected;
        img_data->obstacle_detected     = obstacleDetected;


//      imshow("capMat", capMat);
//      imshow("Canny", cannyMat);
//      imshow("Hough", houghMat);
//      waitKey();
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


        Mat capMat, croppedMat, cannyMat, houghMat;

        //retrieve the current frame
        cap->read(capMat);

        //cropping region
        Size size_uncropped      = capMat.size();
        int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
        Rect cropRect = Rect(0,new_height, size_uncropped.width, size_uncropped.height-new_height);
        croppedMat = capMat(cropRect);



        //finds edges in the capMatMath  via the Canny Edge detection algorithm, and puts
        //result in cannyMat
        //Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
        Canny(croppedMat, cannyMat, 50, 200, 3);
//      Canny(capMat, cannyMat, 55, 110, 3);

        //converts img in cannyMat to another colour space and puts it in houghMat
        cvtColor(cannyMat, houghMat, CV_GRAY2BGR);



        //basic information of the image
        Size size = houghMat.size();
        int imgHeight = size.height;
        int imgWidth  = size.width;
        Point2d camera_center_point = Point2d(imgWidth/2.0, imgHeight);




                vector<Vec4f> lines;
                vector<Point2d> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
                vector<Point2d> rightLines;



                //(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
                HoughLinesP(cannyMat, lines, 1, CV_PI/180,50,2,0);


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
                                estimatedIntersectionDistance = getDistanceToLine(mid, camera_center_point);
                                printf("estimatedDistance to intersection = %f pixels\n", estimatedIntersectionDistance);
                        }


                        //if an end-point of a line plus the midpoint are to one side of the img center,
                        // than consider which side it is on,
                        //else IGNORE THE LINE (may need to fix this)
                        if (mid.x <= camera_center_point.x && (a.x <= camera_center_point.x || b.x <= camera_center_point.x)) {
                                leftLines.push_back(mid);
                        }

                        else if (mid.x >= camera_center_point.x && (a.x >= camera_center_point.x || b.x >= camera_center_point.x)) {
                                rightLines.push_back(mid);
                        }
                        else {
                                printf("ignoring a line...\n");
                        }


                        circle(houghMat,mid,5,Scalar(255,100,0));

                        line(houghMat, mid, camera_center_point, Scalar(0,255,0),1,8);
                        line(houghMat, a, b, Scalar(0,0,255),3,8);


                }

                imwrite("../../lanecap_canny.png", cannyMat);
                imwrite("../../lanecap.png", houghMat);
                return 0;
}

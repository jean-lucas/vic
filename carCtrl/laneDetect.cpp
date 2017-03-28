
#include "laneDetect.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

using namespace cv;
using namespace std;



/* Function declarations */
Point2d get_midpoint(Point2d a, Point2d b);
double calculateAvgAngle(vector<Point2d> vec, Point2d center);
double calculateAvgLineSize(vector<Point2d> vec, Point2d center);
double lineLength(Point2d a, Point2d b);
double getDistanceToLine(Point2d a, Point2d b);
double get_slope(Point2d a, Point2d b);
dounle get_line_length(Point2d a, Point2d b);



/* Constants */

//this percentage will be cutoff from the top of the image
const double CUT_OFF_HEIGHT_FACTOR = 0.45;
const double MIN_LINE_LENGTH = 10;
const double HORIZONTAL_SLOPE_VAL = 0.5;




/*
    Test if camera can capture images/videos
    If successful return pointer to VideoCapture object
    Else return null pointer
*/
VideoCapture test_camera() {

    VideoCapture cap(DEFAULT_CAMERA_ID);

    if (!cap.isOpened()) {
            printf("failed to open capture\n");
            return HALT_SYSTEM;
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



Point2d get_left_intersection(double height, int x0, int xf, Mat cannyMat) {
    unsigned char colour = 0;
    int x;
    Point2d detected_left_point;
    for ( x = x0; x > xf; x--) {
        colour = cannyMat.at<unsigned char>(Point(x,height));
        if (colour > 200) {
            detected_left_point = Point2d(x,height);
            break;
        }
    }
    return detected_left_point;
}


Point2d get_right_intersection(double height, int x0, int xf, Mat cannyMat) {
    unsigned char colour = 0;
    int x;
    Point2d detected_right_point;
    for ( x = x0; x < xf; x++ ) {
        colour = cannyMat.at<unsigned char>(Point(x,height));
        if (colour > 200) {
            detected_right_point = Point2d(x,height);
            break;
        }
    }
    return detected_right_point;
}


int get_lane_statusv3(struct ImageData *img_data, VideoCapture *cap) {

    if (!(cap->isOpened())) {
        printf("failed to open capture\n");
        cap->release();
        return HALT_SYSTEM;
    }


    Mat capMat, croppedMat, cannyMat, houghMat;

    //retrieve the current frame
    cap->read(capMat);

    if (capMat.data == NULL) {
        return CORRUPT_IMAGE;
    }

    //cropping image
    Size size_uncropped = capMat.size();
    int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
    Rect cropRect = Rect(0, new_height, size_uncropped.width, size_uncropped.height-new_height);
    croppedMat = capMat(cropRect);




    //finds edges in the capMatMath  via the Canny Edge detection algorithm, and puts
    //result in cannyMat
    //Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
    Canny(croppedMat, cannyMat, 50, 200, 3);
//  Canny(capMat, cannyMat, 55, 110, 3);



    //basic information of the image
    Size size = cannyMat.size();
    int imgHeight = size.height;
    int imgWidth  = size.width;
    Point2d camera_center_point = Point2d(imgWidth/2.0, imgHeight);




    int height_check = imgHeight/6; // where should we place the trajectory point
    Point2d detected_left_point;
    Point2d detected_right_point;

    //try 2 attempts to get a trajectory point. At each failed instance drop height by 1/5
    detected_left_point  = get_left_intersection(height_check, imgWidth/2, 0, cannyMat);
    detected_right_point = get_right_intersection(height_check, imgWidth/2, imgWidth, cannyMat);

    int num_tries = 0;
    while (detected_left_point.y != detected_right_point.y && num_tries < 4) {
        printf("num_tries = %d\n",num_tries );
        height_check = height_check + imgHeight/6;
        detected_left_point  = get_left_intersection(height_check, imgWidth/2, 0, cannyMat);
        detected_right_point = get_right_intersection(height_check, imgWidth/2, imgWidth, cannyMat);
        num_tries++;
    }

    


    double distLeft = 0, distRight = 0;
    distLeft  = abs(detected_left_point.x - imgWidth/2);
    distRight = abs(detected_right_point.x - imgWidth/2);
    double lane_width = detected_right_point.x - detected_left_point.x;

    //can remove for production
    cvtColor(cannyMat, houghMat, CV_GRAY2BGR);



    //get theta 3
    double theta3 = 0; // trajectory angle
    double offset = 0; // distance from car center to lane center (roughly)

    if ( (distLeft + distRight) > 200 && (detected_left_point.y == detected_right_point.y) && lane_width > 0) {
        if (distLeft > distRight) {
            offset = distLeft - lane_width/2.0;
            theta3 = atan(offset/(imgHeight- height_check))*(-180.0/CV_PI);
            //trajectory
            line(houghMat,Point2d(detected_left_point.x+ distLeft - offset, height_check), Point2d(imgWidth/2, imgHeight), Scalar(55,150,115),5,8);
            circle(houghMat,Point2d(detected_left_point.x+ distLeft - offset, height_check) ,10,Scalar(20,60,200));
        }
        else {
            offset = distRight - lane_width/2.0;
            theta3 = atan(offset/(imgHeight- height_check))*(180.0/CV_PI);
            //trajectory
            line(houghMat,Point2d(detected_left_point.x+ distLeft + offset, height_check), Point2d(imgWidth/2, imgHeight), Scalar(55,150,115),5,8);
            circle(houghMat,Point2d(detected_left_point.x+ distLeft + offset, height_check) ,10,Scalar(20,60,200));
        }
        
        printf("left point (%f %f) \t rgiht point (%f  %f)\t width %f\n", detected_left_point.x, detected_left_point.y, detected_right_point.x, detected_right_point.y, lane_width );
        printf("offset %f \t theta3 %f\ndistLeft %f \t distRight %f\n", offset, theta3, distLeft, distRight);


    }
    else {
        //everything failed. set 0, and fix it later based on theta1 and theta2
        theta3 = 0;
    }

    vector<Vec4f> lines;
    vector<Point2d> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
    vector<Point2d> rightLines;


    //(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
    HoughLinesP(cannyMat, lines, 1, CV_PI/180,38,10,1);
    // HoughLinesP(cannyMat, lines, 1, CV_PI/180,30,5,10);

    //TODO: calculate these values
    bool intersectionDetected = false;
    double estimatedIntersectionDistance = 0;
    bool  obstacleDetected = false;

    Point2d a,b,mid;

    double line_length = 0;
    double slope_tot = 0;
    int slope_count = 0;

    //draw detected lines
    for (size_t i = 0; i < lines.size(); i++) {
        a = Point2d(lines[i][0],lines[i][1]);
        b = Point2d(lines[i][2],lines[i][3]);
        mid = get_midpoint(a,b);

        line_length = get_line_length(a,b);
        if (line_length > MIN_LINE_LENGTH) {
            printf("Point (%f %f) to Point (%f %f) gave slope degree of %f\n", a.x,a.y,b.x,b.y, get_slope(a,b));
            slope_tot += get_slope(a,b);
            slope_count++;
        }

        //if an end-point of a line plus the midpoint are to one side of the img center,
        // than consider which side it is on
        if (mid.x <= camera_center_point.x && (a.x <= camera_center_point.x || b.x <= camera_center_point.x)) {
            leftLines.push_back(mid);
        }

        else if (mid.x >= camera_center_point.x && (a.x >= camera_center_point.x || b.x >= camera_center_point.x)) {
            rightLines.push_back(mid);
        }

        line(houghMat, mid, camera_center_point, Scalar(0,255,0),1,8);
        line(houghMat, a, b, Scalar(0,0,255),3,8);

    }



    double theta1, theta2;
    theta1 = calculateAvgAngle(leftLines, camera_center_point)*(180.0/CV_PI);
    theta2 = calculateAvgAngle(rightLines,camera_center_point)*(180.0/CV_PI);

    double avgLeftSize, avgRightSize;
    avgLeftSize = calculateAvgLineSize(leftLines, camera_center_point);
    avgRightSize = calculateAvgLineSize(rightLines, camera_center_point);

    double avgSlope;
    avgSlope = slope_tot/slope_count;

    
    

    printf("Theta1: %f \tTheta2: %f\tTheta3: %f\t Slope: %f\n", theta1, theta2, theta3, avgSlope);

    img_data->avg_left_angle        = theta1;
    img_data->avg_right_angle       = theta2;
    img_data->trajectory_angle      = theta3;
    img_data->old_slope             = img_data->avg_slope;
    img_data->avg_slope             = avgSlope;
    img_data->left_line_length      = avgLeftSize;
    img_data->right_line_length     = avgRightSize;
    img_data->intersection_distance = estimatedIntersectionDistance;
    img_data->intersection_detected = intersectionDetected;
    img_data->obstacle_detected     = obstacleDetected;
   imwrite("../../hough.png",houghMat);

    return NO_ERROR;
}




double get_slope(Point2d a, Point2d b) {
    double hor_dist = (b.x - a.x);
    double ver_dist = (b.y - a.y);
    if (ver_dist == 0 && hor_dist == 0) {
        printf("HOLY MOLY!\n");
        exit(0);
    }
    return atan2(hor_dist/ver_dist); 
}

//get line length between two points
double get_line_length(Point2d a, Point2d b) {
    return sqrt(pow(a.x - b.x, 2) + pow(a.y - b.y, 2));
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


Point2d get_midpoint(Point2d a, Point2d b) {
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

    imwrite("../../cap.png", capMat);

    //cropping region
    Size size_uncropped      = capMat.size();
    int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
    Rect cropRect = Rect(0,new_height, size_uncropped.width, size_uncropped.height-new_height);
    croppedMat = capMat(cropRect);



    //finds edges in the capMatMath  via the Canny Edge detection algorithm, and puts
    //result in cannyMat
    //Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
    Canny(croppedMat, cannyMat, 80, 250, 3);
    //Canny(capMat, cannyMat, 55, 110, 3);

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
    HoughLinesP(cannyMat, lines, 1, CV_PI/180,30,5,10);


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


        Point2d mid = get_midpoint(a,b);

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

    double theta1, theta2;
    theta1 = calculateAvgAngle(leftLines, camera_center_point)*(180.0/CV_PI);
    theta2 = calculateAvgAngle(rightLines,camera_center_point)*(180.0/CV_PI);

    double avgLeftSize, avgRightSize;
    avgLeftSize = calculateAvgLineSize(leftLines, camera_center_point);
    avgRightSize = calculateAvgLineSize(rightLines, camera_center_point);

    printf("Theta1: %f \tTheta2: %f \n", theta1, theta2);
    printf("leftLine: %f \trightLine: %f \n",avgLeftSize, avgRightSize);



    imwrite("../../lanecap_canny.png", cannyMat);
    imwrite("../../lanecap.png", houghMat);
    return 0;
}

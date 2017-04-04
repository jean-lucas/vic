
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
double get_line_length(Point2d a, Point2d b);



/* Constants */
//this percentage will be cutoff from the top of the image
const double CUT_OFF_HEIGHT_FACTOR = 0.35;
const double MIN_LINE_LENGTH = 5;
const double INVALID_SLOPE = 200;





/*
    Test if camera can capture images/videos
    If successful return pointer to VideoCapture object
    Else return null pointer

    The VideoCapture returned will be used for constant streaming.
*/
VideoCapture test_camera() {

    VideoCapture cap(DEFAULT_CAMERA_ID);
    cap.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    if (!cap.isOpened()) {
            printf("failed to open capture\n");
            return HALT_SYSTEM;
    }
    return cap;
}


/*
    This is for 'calibrating' the camera on the tower.
    Allows us to quickly see if the camera should be turned up or down.  
*/
void calibrate_camera(VideoCapture *cap) {

    if (!(cap->isOpened())) {
        printf("failed to open capture\n");
        cap->release();
        return;
    }
    Mat capMat;
    cap->read(capMat);
    printf("image printed to cap_mat.png\n");
    imwrite("../../cap_mat.png",capMat);
}




/** 
    This is a temporary function.
    Ideally we would want this to detect any arbitrary colour, 
    as well as running a different thread
*/
int detect_red(int y0, int yf, int x0, int xf, Mat mat) {

    Vec3b colour;
    int x = 0, y = 0, vote = 0;
    for (y = y0; y < yf; y+=5) {
        for (x = x0; x < xf; x+=5) {
            colour = mat.at<Vec3b>(Point(x,y));
            if (colour[2] > 200 && (colour [0] + colour[1]) < 200) {
                vote++;
                // printf("(%d, %d) %d %d %d\n", x,y,colour[0], colour[1],colour[2]);
            }
            if (vote > 10) {
                return 1;
            }

        }
    }
    return 0;
}





int get_lane_statusv3(struct ImageData *img_data, VideoCapture *cap) {

    if (!(cap->isOpened())) {
        printf("failed to open capture\n");
        cap->release();
        return HALT_SYSTEM;
    }


    Mat capMat, croppedMat, cannyMat, houghMat, contourMat, contourCanny;

    //retrieve the current frame from video stream
    cap->read(capMat);

    if (capMat.data == NULL) {
        return CORRUPT_IMAGE;
    }

    //cropping image
    Size size_uncropped = capMat.size();
    int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
    Rect cropRect = Rect(0, new_height, size_uncropped.width, size_uncropped.height-new_height);
    croppedMat = capMat(cropRect);
    Size cropped = croppedMat.size();



    // Try to detect an intersection, this should be moved later
    int detected_red =0;
    detected_red = detect_red(cropped.height/4, 3*cropped.height/4, cropped.width/3, 2*cropped.width/3, croppedMat);

    if (detected_red) {
        printf("intersection found, stopping car.\n");
        img_data->intersection_detected = detected_red;
        return NO_ERROR;
    }

    

    //Create binary image
    cvtColor(croppedMat, croppedMat,CV_BGR2GRAY);
    threshold(croppedMat, croppedMat, 200, 255, THRESH_BINARY);
    

    //Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
    Canny(croppedMat, cannyMat, 50, 255, 3);


    //basic information of the image
    Size size = cannyMat.size();
    int imgHeight = size.height;
    int imgWidth  = size.width;
    Point2d camera_center_point = Point2d(imgWidth/2.0, imgHeight);



    //THese vector of lines will hold the results from Hough Transform
    vector<Vec4f> lines;
    vector<Point2d> leftLines; //left lines and right lines are based on the center point (CAREFUL!)
    vector<Point2d> rightLines;


    //(inputMat, output vector N x 4, distance resolution of accumulator, angle of accumulator, threshold, minLineLength, maxLineGap )
    HoughLinesP(cannyMat, lines, 1, CV_PI/180,38,10,1);



    Point2d a,b,mid;

    double line_length = 0;
    double slope_tot = 0;
    int slope_count = 0;
    double slope_ang = 0;


    int col = 0;


    //logic on the detected lines from hough transform
    for (size_t i = 0; i < lines.size(); i++) {
        a = Point2d(lines[i][0],lines[i][1]);
        b = Point2d(lines[i][2],lines[i][3]);
        mid = get_midpoint(a,b);   


        line_length = get_line_length(a,b);
        if (line_length > MIN_LINE_LENGTH) {

            slope_ang = get_slope(a,b);

            if (slope_ang != INVALID_SLOPE) {
                slope_tot   += slope_ang;
                slope_count += 1;
                // line(houghMat, a, b, Scalar(0,0,col),3,8);
                // printf("pass: Point (%f %f) to Point (%f %f) gave slope degree of %f\n", a.x,a.y,b.x,b.y,slope_ang);
            }
        }

        //if an end-point of a line plus the midpoint are to one side of the img center,
        // than consider which side it is on
        if (mid.x <= camera_center_point.x && (a.x <= camera_center_point.x || b.x <= camera_center_point.x)) {
            leftLines.push_back(mid);
        }

        else {
            rightLines.push_back(mid);
        }


        // line(houghMat, mid, camera_center_point, Scalar(0,255,0),1,8);
        // line(houghMat, a, b, Scalar(0,0,col),3,8);

    }




    double avgLeftSize, avgRightSize;
    avgLeftSize = calculateAvgLineSize(leftLines, camera_center_point);
    avgRightSize = calculateAvgLineSize(rightLines, camera_center_point);


    double avgSlope = 0;
    if (slope_count > 0) 
        avgSlope = slope_tot/slope_count;
    else 
        avgSlope = 0;

    printf("avgSlope = %f \n",avgSlope );



    img_data->old_slope             = img_data->avg_slope;
    img_data->avg_slope             = avgSlope;
    img_data->left_line_length      = avgLeftSize;
    img_data->right_line_length     = avgRightSize;
    img_data->intersection_detected = intersectionDetected;

   // imwrite("../../hough.png",houghMat);

    return NO_ERROR;
}







/*
    Calculate the angle the line (a,b) creates to a vertical line. 
    Angles in the FIRST  quadrant will be positive -> right turn
    Angles in the SECOND quadrant will be negative -> left turn. 
*/
double get_slope(Point2d a, Point2d b) {
    double hor_dist = (a.x - b.x);
    double ver_dist = (a.y - b.y);

    if (abs(ver_dist) <= 0.0001 || abs(hor_dist) <= 0.0001) {
        return INVALID_SLOPE;
    }

    double angle = atan(hor_dist/ver_dist)*(-180.0/CV_PI);

    if (abs(angle) > 50) {
        return INVALID_SLOPE;
    }

    return angle;
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


//find the Euclidean distance from point a to point b 
double getDistanceToLine(Point2d a, Point2d b) {
    double distance = sqrt(pow(  a.x - b.x, 2) + pow(  a.y - b.y, 2));
    return distance;
}














/*
usage:
string ty =  type2str( cannyMat.type() );
printf("Matrix: %s %dx%d \n", ty.c_str(), cannyMat.cols, cannyMat.rows );       

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
*/
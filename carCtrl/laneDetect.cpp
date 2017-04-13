
#include "laneDetect.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>

#include <raspicam/raspicam_cv.h>

using namespace cv;
using namespace std;
using namespace raspicam;



/* private function declarations */
static Point2d get_midpoint(Point2d a, Point2d b);
static double calculateAvgAngle(vector<Point2d> vec, Point2d center);
static double calculateAvgLineSize(vector<Point2d> vec, Point2d center);
static double lineLength(Point2d a, Point2d b);
static double getDistanceToLine(Point2d a, Point2d b);
static double get_slope(Point2d a, Point2d b);
static double get_line_length(Point2d a, Point2d b);
static int get_intersection_type(Mat mat);


/* Constants */
//this percentage will be cutoff from the top of the image
const double CUT_OFF_HEIGHT_FACTOR = 0.55;
const double MIN_LINE_LENGTH = 5;
const double INVALID_SLOPE = 200;
const double MIN_INTERSECTION_DISTANCE = 100;

static struct int_info {
    int type     = 0;
    int detected = 0;
    double dist  = -1;
} info;





/*
    Test if camera can capture images/videos
    If successful return pointer to RaspiCam_Cv object
    Else return null pointer

    The RaspiCam_Cv returned will be used for constant streaming.
*/
void test_camera(raspicam::RaspiCam_Cv* cap) {
    // RaspiCam_Cv *cap = (RaspiCam_Cv *) malloc(sizeof(RaspiCam_Cv));
    cap->set( CV_CAP_PROP_FORMAT, CV_8UC3 );
    cap->set( CV_CAP_PROP_FRAME_WIDTH, 640);
    cap->set( CV_CAP_PROP_FRAME_HEIGHT, 480);

    cap->open();
    if (cap->isOpened()) {
        printf("open success\n");
    }
    else {
        printf("open not success\n");
    }
    sleep(1);
    
}

void calibrate_raspicam(RaspiCam_Cv *cap) {

    sleep(1);
    Mat capMat;
    cap->grab();
    cap->retrieve(capMat);
    imwrite("../../raspicam.png", capMat);
    printf("image printed to raspicam.png\n");

}




/** 
    This is a temporary function.
    Ideally we would want this to detect any arbitrary colour, 
    as well as running a different thread BGR
*/
int_info detect_intersection(int y0, int yf, int x0, int xf, Mat mat) {

    Vec3b colour;
    int x = 0, y = 0, vote = 0;
    double detected_ypos = 0;

    int_info inter_info;
    inter_info.detected = 0;
    inter_info.dist     = -1;

    for (y = y0; y < yf; y+=5) {
        for (x = x0; x < xf; x+=5) {
            colour = mat.at<Vec3b>(Point(x,y));
            if (colour[2] > 200 && (colour [0] + colour[1]) < 200) {
                vote++;
                detected_ypos += y;
            }
            else if (colour[0] > 200 && (colour[1] + colour[2]) < 200) {
                vote++;
                detected_ypos += y;
            }
            if (vote > 5) {
                inter_info.detected = 1;
                inter_info.dist     = detected_ypos/vote;
                return inter_info;
            }

        }
    }

    return inter_info;
}




int get_lane_statusv3(struct ImageData *img_data, RaspiCam_Cv *cap) {

    if (!(cap->isOpened())) {
        printf("failed to open capture\n");
        cap->release();
        return HALT_SYSTEM;
    }


    Mat capMat, croppedMat, cannyMat, houghMat, contourMat, contourCanny;

    //retrieve the current frame from video stream


    cap->grab();
    cap->retrieve(capMat);

    if (capMat.data == NULL) {
        return CORRUPT_IMAGE;
    }

    Size size_uncropped = capMat.size();


    // Try to detect an intersection
    info = detect_intersection(size_uncropped.height/4, 3*size_uncropped.height/4, 
                                size_uncropped.width/3, 2*size_uncropped.width/3, capMat);

    // printf("\nDistance to intersection %f, and detected %d\n", info.dist, info.detected );

    if (info.detected) {
        img_data->intersection_detected = info.detected;
        img_data->intersection_distance = info.dist;
        img_data->intersection_type     = info.type;
        if (info.dist < MIN_INTERSECTION_DISTANCE && info.dist >= 0) {
            img_data->intersection_stop = 1;
            printf("intersection found of type %d, stopping car.\n", info.type);
            return NO_ERROR;
        }
        
    }


    // imwrite("../../step1_bareCap.png",capMat);

    //cropping image
    int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
    Rect cropRect = Rect(0, new_height, size_uncropped.width, size_uncropped.height-new_height);
    croppedMat = capMat(cropRect);
    Size cropped = croppedMat.size();

    // imwrite("../../step2_crop.png",croppedMat);

    

    

    //Create binary image
    cvtColor(croppedMat, croppedMat, CV_BGR2GRAY);
    threshold(croppedMat, croppedMat, 180, 255, THRESH_BINARY);
    
    // imwrite("../../step3_binary.png",croppedMat);

    //Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
    Canny(croppedMat, cannyMat, 50, 255, 3);

    // imwrite("../../lul.png",cannyMat);

    //basic information of the image
    Size size = cannyMat.size();
    int imgHeight = size.height;
    int imgWidth  = size.width;
    Point2d camera_center_point = Point2d(imgWidth/2.0, imgHeight);

    cvtColor(cannyMat, houghMat, CV_GRAY2BGR);



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


    int col = 255;


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

    //imwrite("../../step5_hough.png",houghMat);


    double avgLeftSize, avgRightSize;
    avgLeftSize = calculateAvgLineSize(leftLines, camera_center_point);
    avgRightSize = calculateAvgLineSize(rightLines, camera_center_point);


    double avgSlope = 0;
    if (slope_count > 0) 
        avgSlope = slope_tot/slope_count;
    else 
        avgSlope = 0;

    printf("avgSlope = %f \n",avgSlope );
    printf("L = %f \t R = %f\n", avgLeftSize, avgRightSize );


    img_data->old_slope             = img_data->avg_slope;
    img_data->avg_slope             = avgSlope;
    img_data->left_line_length      = avgLeftSize;
    img_data->right_line_length     = avgRightSize;
    img_data->intersection_detected = info.detected ;
    img_data->intersection_type     = info.type;


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

    if (angle > 50) {
        info.type = 1; 
        return INVALID_SLOPE;
    }
    if (angle < -50) {
        info.type = 2;
        return INVALID_SLOPE;
    }

    return angle;
}


double get_distance_to_intersection(Point start_point) {

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
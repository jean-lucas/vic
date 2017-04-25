
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

#define PINK   0
#define BLUE   1
#define GREEN  2
#define ORANGE 3




/* private function declarations */
static Point2d get_midpoint(Point2d a, Point2d b);
static double calculateAvgAngle(vector<Point2d> vec, Point2d center);
static double calculateAvgLineSize(vector<Point2d> vec, Point2d center);
static double lineLength(Point2d a, Point2d b);
static double getDistanceToLine(Point2d a, Point2d b);
static double get_slope(Point2d a, Point2d b);
static double get_line_length(Point2d a, Point2d b);


/* Constants */
//this percentage will be cutoff from the top of the image
const double CUT_OFF_HEIGHT_FACTOR = 0.45;
const double CUT_OFF_WIDTH_FACTOR  = 0.08; // from both sides
const double MIN_LINE_LENGTH = 5;
const double INVALID_SLOPE = 200;
const double MIN_INTERSECTION_DISTANCE = 140;


static struct int_info {
    int type     = 0;
    int detected = 0;
    int colour   = -1;   
    double dist  = -1;
} info;

#define MAX(x, y) (((x) > (y)) ? (x) : (y))
#define MIN(x, y) (((x) < (y)) ? (x) : (y))





void calibrate_raspicam(RaspiCam_Cv *cap) {

    sleep(1);
    Mat capMat;
    cap->grab();
    cap->retrieve(capMat);
    imwrite("../../raspicam.png", capMat);
    printf("image printed to raspicam.png\n");

}

// from http://coecsl.ece.illinois.edu/ge423/spring05/group8/finalproject/hsv_writeup.pdf
// r,g,b values are from 0 to 1
// h = [0,360], s = [0,1], v = [0,1]
// if s == 0, then h = -1 (undefined)
void RGBtoHSV( float r, float g, float b, float *h, float *s, float *v ) {
    float min, max, delta;
    min = MIN(MIN( r, g), b );
    max = MAX(MAX( r, g), b );
    *v = max; // v
    delta = max - min;
    if( max != 0 )
        *s = delta / max; // s
    else {
        *s = 0;
        *h = -1;
        return;
    }
    if( r == max )
        *h = ( g - b ) / delta; // between yellow & magenta
    else if( g == max )
        *h = 2 + ( b - r ) / delta; // between cyan & yellow
    else
        *h = 4 + ( r - g ) / delta; // between magenta & cyan
    
    *h *= 60; // degrees
    if( *h < 0 )
        *h += 360;
}


/** 
    Detect a intersection colour segment.
    And the distance to the colour segment
*/
int_info detect_intersection(int y0, int yf, int x0, int xf, Mat mat) {

    int x = 0, y = 0, tot_vote = 0;
    int step = 10;
    double detected_ypos = 0;
    float red = 0, blue = 0, green = 0;
    float h = 0, s = 0, v = 0;
    int colour_votes[4] = {0};
    
    Vec3b colour;
    int_info inter_info;
    inter_info.detected = 0;
    inter_info.dist     = -1;
    inter_info.colour   = -1;

    for (y = y0; y < yf; y += step) {
        for (x = x0; x < xf; x += step) {
            colour = mat.at<Vec3b>(Point(x,y));
            red =   (float) colour[2] / 255;
            green = (float) colour[1] / 255;
            blue =  (float) colour[0] / 255;
        
            RGBtoHSV(red, green, blue, &h, &s, &v);

            if (s > 0.2 && v > 0.5) {

                if (h > 300 && h < 360) {           //detect pink
                    ++tot_vote;
                    ++colour_votes[PINK];
                    detected_ypos += y;
                }
                else if (h > 0 && h < 60) {         //detect orange
                    ++tot_vote;
                    ++colour_votes[ORANGE];
                    detected_ypos += y;
                }
                else if (h > 190 && h < 260) {      //detect blue
                    ++tot_vote;
                    ++colour_votes[BLUE];
                    detected_ypos += y;
                }
                else if (h > 80 && h < 160) {       //detect green
                    ++tot_vote;
                    ++colour_votes[GREEN];
                    detected_ypos += y;
                }
            }
                       
            if (tot_vote > 3) {
                int max_vote_colour = -1;
                for (int i = 0; i < 4; i++) {
                    if (max_vote_colour < colour_votes[i])
                        max_vote_colour = i;
                }
                printf("COLOR DETECTED %d\n", max_vote_colour );
                inter_info.detected = 1;
                inter_info.dist     = detected_ypos/tot_vote;
                inter_info.colour   = max_vote_colour;
                return inter_info;
            }



        }
    }

    return inter_info;
}


void draw_grid(int y0, int yf, int x0, int xf, Mat mat) {


    int x,y;
    int step =  10;
    for (y = y0; y < yf; y+=step) {
        line(mat, Point(x0,y), Point(xf,y), Scalar(0,255,255),1,8);
    }
    for (x = x0; x < xf; x+=step) {
        line(mat, Point(x,y0), Point(x,yf), Scalar(0,255,255),1,8);
    }
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


    // Try to detect an intersection (y0, yf, x0, xf, mat)
    info = detect_intersection( size_uncropped.height/5, 
                                size_uncropped.height, 
                                size_uncropped.width/3, 
                                2*size_uncropped.width/3, 
                                capMat);

    // info = detect_intersection( size_uncropped.height/5, 
    //                             size_uncropped.height, 
    //                             size_uncropped.width/3, 
    //                             2*size_uncropped.width/3, 
    //                             capMat);

    // draw_grid(  size_uncropped.height/5, 
    //             size_uncropped.height, 
    //             size_uncropped.width/3, 
    //             2*size_uncropped.width/3, 
    //             capMat);
    

    if (info.detected) {
        info.dist = size_uncropped.height - info.dist;
    
        printf("\nDistance to intersection %f, and detected %d\n", info.dist, info.detected );
        img_data->intersection_detected = info.detected;
        img_data->intersection_distance = info.dist;
        img_data->intersection_colour   = info.colour;
        if (info.dist < MIN_INTERSECTION_DISTANCE) {
            img_data->intersection_stop = 1;
            printf("intersection found of type %d, stopping car.\n", info.type);
            return NO_ERROR;
        }
        
    }

    if (!info.detected && img_data->intersection_distance > 0) {
        img_data->intersection_detected = 1;
        img_data->intersection_distance = 0;
        img_data->intersection_stop     = 1;
        img_data->intersection_colour   = info.colour;
        printf("intersection found of type %d, stopping car (2).\n", info.type);
        return NO_ERROR;
    }



    //cropping image
    int top_y      = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
    int top_x      = size_uncropped.width*CUT_OFF_WIDTH_FACTOR;
    int new_width  = size_uncropped.width  - 2*top_x;
    int new_height = size_uncropped.height - top_y;

    Rect cropRect = Rect(top_x, top_y, new_width, new_height);
    croppedMat = capMat(cropRect);
    
    Size cropped = croppedMat.size();


    

    

    //Create binary image
    cvtColor(croppedMat, croppedMat, CV_BGR2GRAY);
    threshold(croppedMat, croppedMat, 170, 255, THRESH_BINARY);
    
    // imwrite("../../step3_binary.png",croppedMat);

    //Canny(inputMay, outputMat, threshold_1, threshold_2, apertureSize, L2Gradient )
    Canny(croppedMat, cannyMat, 50, 255, 3);

    // imwrite("../../canny_cropped.png",cannyMat);

    //basic information of the image
    Size size = cannyMat.size();
    int imgHeight = size.height;
    int imgWidth  = size.width;
    Point2d camera_center_point = Point2d(imgWidth/2.0, imgHeight);


    //houghmat needs to be BGR in order to apply hough transform
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


    }

    // imwrite("../../step5_hough.png",houghMat);


    double avgLeftSize, avgRightSize;
    avgLeftSize = calculateAvgLineSize(leftLines, camera_center_point);
    avgRightSize = calculateAvgLineSize(rightLines, camera_center_point);


    double avgSlope = 0;
    if (slope_count > 0) 
        avgSlope = slope_tot/slope_count;
    else 
        avgSlope = 0;

    // printf("avgSlope = %f \nL = %f \t R = %f\n",avgSlope, avgLeftSize, avgRightSize );


    img_data->old_slope             = img_data->avg_slope;
    img_data->avg_slope             = avgSlope;
    img_data->left_line_length      = avgLeftSize;
    img_data->right_line_length     = avgRightSize;
    img_data->intersection_detected = info.detected;
    img_data->intersection_distance = info.dist;
    img_data->intersection_type     = info.type;
    img_data->intersection_colour   = info.colour;
    img_data->intersection_stop     = 0;


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
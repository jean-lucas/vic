
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <algorithm>
#include <math.h>
#include <time.h>
#include "laneDetect.h"


#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/time.h>

using namespace cv;
using namespace std;

//this percentage will be cutoff from the top of the image
const double CUT_OFF_HEIGHT_FACTOR = 0.30;
const double LANE_WIDTH = 550.0;
const int DETECTED_PTS_MIN = 3;

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

double getMS() {
    struct timeval t;
    gettimeofday(&t, NULL);
    double ms = (t.tv_sec)*1000 + (t.tv_usec)/1000;
    return ms;
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

int get_lane_statusv2(struct ImageData *img_data, VideoCapture *cap) {

    if (!(cap->isOpened())) {
            printf("failed to open capture\n");
            cap->release();
            return 0;
    }

    int go_slow = 0;

    Mat capMat, croppedMat, cannyMat;

    //retrieve the current frame
    cap->read(capMat);


    // imwrite("../../cap.png", capMat);  
    //cropping region
	Size size_uncropped = capMat.size();
	int new_height = size_uncropped.height*CUT_OFF_HEIGHT_FACTOR;
	Rect cropRect = Rect(0,new_height, size_uncropped.width, size_uncropped.height-new_height);
	croppedMat = capMat(cropRect);
	blur(croppedMat,croppedMat,Size(3,3));




    Canny(croppedMat, cannyMat, 150, 240, 3);

    Size sz = cannyMat.size();
    int half_wid = sz.width/2;

    int y = 0;
    int x = 0;

    int numRows = sz.height;
    int num_segs = numRows/20;
    
    unsigned char colour;
    vector<Point2d> detected_pts_left;
    vector<Point2d> detected_pts_right;

    // Only check the bottom 4/5ths of the image. This can be changed
    for ( y = 2*sz.height/5; y < 3*sz.height/5; y+=1) {

        //get points on the right side
        for ( x = half_wid; x < sz.width; x++) {
            colour = cannyMat.at<unsigned char>(Point(x,y));
            if (colour > 200) {
				circle(cannyMat,Point(x,y),5,Scalar(100,0,90));
				detected_pts_right.push_back(Point2d(x,y));
                break;
            }
        }

        //get points on the left side
        for ( x = half_wid; x > 0; x--) {
            colour = cannyMat.at<unsigned char>(Point(x,y));
            if (colour > 200) {
                circle(cannyMat,Point(x,y),5,Scalar(100,0,90));
                detected_pts_left.push_back(Point2d(x,y));
                break;
            }
        }
    }


    double distLeft = 0, distRight = 0;
    int left_count = detected_pts_left.size();
    int right_count = detected_pts_right.size();


    //if the # of detected pts on a given side is less than three
    //consider an alternative approach.

    int i = 0;
    for (i = left_count-1; i >= 0; i--) {
        distLeft += abs(detected_pts_left[i].x - half_wid);
    }
    for (i = right_count-1; i >= 0; i--) {
        distRight += abs(detected_pts_right[i].x - half_wid);
    }

    //get avg distances
    distLeft = distLeft/left_count;
    distRight = distRight/right_count;

    double desired_change = 0;
    printf("desired left %f (%d)\t desired right %f (%d) \n",distLeft, left_count, distRight,right_count );

    if (left_count > DETECTED_PTS_MIN && right_count > DETECTED_PTS_MIN ) {
        desired_change = distRight - distLeft;
    }
    else if (left_count <= DETECTED_PTS_MIN && right_count > DETECTED_PTS_MIN) { //likely due to not seeing the left lane
        desired_change = distRight - LANE_WIDTH/2.0;
    }
    else if (right_count <= DETECTED_PTS_MIN && left_count > DETECTED_PTS_MIN) {
        desired_change = LANE_WIDTH/2.0 - distLeft;
    }
    else {
        // weird? keep ongoing straight and slow down
        desired_change = 0;
        go_slow = 1;
    }
    printf(" fix \t\t %f \n",desired_change );
   
    // printf("desired change is  %f\n", desired_change);
    // char* filename = (char*)malloc(sizeof(char)*100);
    // sprintf(filename,"%s-%f%s","../../caps/lanecap_canny",getMS(),".png");
	// imwrite(filename, cannyMat);
    // printf("%s\n",filename );
    // imwrite("../../lanecap_canny.png", cannyMat);
	
	//colour detection for detecting stop sign
	// Mat HSV;
	// Mat colorThreshold;
	// cvtColor(capMat,HSV,CV_BGR2HSV);
	// inRange(HSV,Scalar(15,15,100),Scalar(145,145,250),colorThreshold);
	// imwrite("../../colour_threshold_mask.png",colorThreshold);
	Mat maskMat;
	inRange(capMat,Scalar(15,15,100),Scalar(45,45,255),maskMat);
	imwrite("../../colour_threshold_mask.png",maskMat);


    img_data->fix                   = desired_change;
    img_data->avg_left_angle        = 0;
    img_data->avg_right_angle       = 0;
    img_data->left_line_length      = 0;
    img_data->right_line_length     = 0;
    img_data->intersection_distance = 0;
    img_data->intersection_detected = 0;
    img_data->obstacle_detected     = 0;
    img_data->go_slow             = go_slow;

    return 1;
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
//  Canny(capMat, cannyMat, 55, 110, 3);

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
        Point2d a = Point2d(lines[i][0],lines[i][1]);
        Point2d b = Point2d(lines[i][2],lines[i][3]);

        Point2d mid = getMidpoint(a,b);

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

     printf("Theta1: %f \tTheta2: %f \n", theta1, theta2);
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

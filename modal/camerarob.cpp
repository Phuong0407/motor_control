#include "define.h"
int x;
extern int run;
//extern int argc;
//extern char * argv[];


const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
/*int low_H = 0, low_S = 0, low_V = 0;
int high_H = max_value_H, high_S = max_value, high_V = max_value;
static void on_low_H_thresh_trackbar(int, void *)
{
 low_H = min(high_H-1, low_H);
 setTrackbarPos("Low H", window_detection_name, low_H);
}
static void on_high_H_thresh_trackbar(int, void *)
{
 high_H = max(high_H, low_H+1);
 setTrackbarPos("High H", window_detection_name, high_H);
}
static void on_low_S_thresh_trackbar(int, void *)
{
 low_S = min(high_S-1, low_S);
 setTrackbarPos("Low S", window_detection_name, low_S);
}
static void on_high_S_thresh_trackbar(int, void *)
{
 high_S = max(high_S, low_S+1);
 setTrackbarPos("High S", window_detection_name, high_S);
}
static void on_low_V_thresh_trackbar(int, void *)
{
 low_V = min(high_V-1, low_V);
 setTrackbarPos("Low V", window_detection_name, low_V);
}
static void on_high_V_thresh_trackbar(int, void *)
{
 high_V = max(high_V, low_V+1);
 setTrackbarPos("High V", window_detection_name, high_V);
}*/
void * recuperation_barycentre(void * b) {
 VideoCapture cap(0);
 //namedWindow(window_capture_name);
 //namedWindow(window_detection_name);
 // Trackbars to set thresholds for HSV values
 /*createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
 createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
 createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
 createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
 createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
 createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);*/
 Mat frame, frame_HSV, frame_threshold;
 while (run) {
 cap >> frame;
 if(frame.empty())
 {
 break;
 }
 // Convert from BGR to HSV colorspace
 cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
 // Detect the object based on HSV Range Values
 inRange(frame_HSV, Scalar(0, 83, 113), Scalar(25, 255, 255), frame_threshold);
 
 // detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
findContours(frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
// draw contours on the original image
Mat image_copy = frame.clone();
//drawContours(image_copy, contours, -1, Scalar(0, 255, 0), 2);

// find moments of the image
Mat image_copy2 = frame.clone();
Moments m = moments(frame_threshold,true);
Point p(m.m10/m.m00, m.m01/m.m00);
x=m.m10/m.m00;
 
// coordinates of centroid
cout<< x<< endl;
 
// show the image with a point mark at the centroid
//circle(image_copy2, p, 5, Scalar(128,0,0), -1);
//imshow("Image with center",image_copy2);

//imshow("None approximation", image_copy);
//imshow(window_detection_name, frame_threshold);
//waitKey(0);
//imwrite("contours_none_image1.jpg", image_copy);
//destroyAllWindows();


 // Show the frames
 //imshow(window_capture_name, frame);
 
 char key = (char) waitKey(30);
 if (key == 'q' || key == 27)
 {
 break;
 }
 }
 
 return 0;
}


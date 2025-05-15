#include "define.h"
int x;
int y;
int pixel;
extern int run;
extern int low_H;
extern int low_S;
extern int low_V;
extern int high_H;
extern int high_S;
extern int high_V;
extern int erosion_size;
extern int dilation_size;

const int max_value_H = 360/2;
const int max_value = 255;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";
Mat erosion_dst, dilation_dst;

void * recup_bary_seuil(void * b) {
int pix=0;
int i=0;
int j =0;

 VideoCapture cap(0);
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
 inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
 
//erosion
 Mat element = getStructuringElement( MORPH_ELLIPSE, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
 erode( frame_threshold, erosion_dst, element );
//dilation
 Mat element1 = getStructuringElement( MORPH_ELLIPSE, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
 dilate( erosion_dst, dilation_dst, element1 );
 
// find moments of the image
Moments m = moments(dilation_dst,true);
Point p(m.m10/m.m00, m.m01/m.m00);
x=m.m10/m.m00;
y=m.m01/m.m00;

// coordinates of centroid
cout<< x<< endl;
cout<< y<< endl;
 
// number of pixel
 int pix=0;
 for (i=0; i<1280; ++i){
	 for (j=0;j<960;++j){
		 if (dilation_dst.at<int>(j,i)!=0){
			 ++pix;}
	}
}
pixel=pix;
cout<< pixel<< endl;

 char key = (char) waitKey(30);
 if (key == 'q' || key == 27)
 {
 break;
 }
 }
 return NULL;
}

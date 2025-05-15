#include <opencv2/imgproc.hpp>
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";

Mat frame, frame_HSV, frame_threshold;
Mat erosion_dst, dilation_dst;
int pix=0;
int i=0;
int j =0;
int erosion_elem = 0;
int erosion_size = 0;
int dilation_elem = 0;
int dilation_size = 0;
int const max_elem = 2;
int const max_kernel_size = 21;

void Erosion( int, void* );
void Dilation( int, void* );

const int max_value_H = 360/2;
const int max_value = 255;
int low_H = 0, low_S = 0, low_V = 0;
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
}

int main(int argc, char* argv[])
{
 VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
 namedWindow(window_capture_name);
 namedWindow(window_detection_name);
 // Trackbars to set thresholds for HSV values
 createTrackbar("Low H", window_detection_name, &low_H, max_value_H, on_low_H_thresh_trackbar);
 createTrackbar("High H", window_detection_name, &high_H, max_value_H, on_high_H_thresh_trackbar);
 createTrackbar("Low S", window_detection_name, &low_S, max_value, on_low_S_thresh_trackbar);
 createTrackbar("High S", window_detection_name, &high_S, max_value, on_high_S_thresh_trackbar);
 createTrackbar("Low V", window_detection_name, &low_V, max_value, on_low_V_thresh_trackbar);
 createTrackbar("High V", window_detection_name, &high_V, max_value, on_high_V_thresh_trackbar);
 while (true) {
 cap >> frame;
 if(frame.empty())
 {
 break;
 }
 
 // Convert from BGR to HSV colorspace
 cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
 // Detect the object based on HSV Range Values
 inRange(frame_HSV, Scalar(low_H, low_S, low_V), Scalar(high_H, high_S, high_V), frame_threshold);
 
 // detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
vector<vector<Point> > contours;
vector<Vec4i> hierarchy;
findContours(frame_threshold, contours, hierarchy, RETR_TREE, CHAIN_APPROX_NONE);
// draw contours on the original image
Mat image_copy = frame.clone();
drawContours(image_copy, contours, -1, Scalar(0, 255, 0), 2);

// find moments of the image
Mat image_copy2 = frame.clone();
Moments m = moments(dilation_dst,true);
Point p(m.m10/m.m00, m.m01/m.m00);
 
// coordinates of centroid
cout<< Mat(p)<< endl;
 cout<< frame_threshold.size().width<< endl;
// show the image with a point mark at the centroid
circle(image_copy2, p, 5, Scalar(128,0,0), -1);
imshow("Image with center",image_copy2);

imshow("None approximation", image_copy);
imshow(window_detection_name, frame_threshold);
printf("%d\n",m.m10/m.m00);
//waitKey(0);
//imwrite("contours_none_image1.jpg", image_copy);
//destroyAllWindows();

 namedWindow( "Erosion Demo", WINDOW_AUTOSIZE );
 namedWindow( "Dilation Demo", WINDOW_AUTOSIZE );
 moveWindow( "Dilation Demo", erosion_dst.cols, 0 );
  createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Erosion Demo",
 &erosion_elem, max_elem,
 Erosion );
 createTrackbar( "Kernel size:\n 2n +1", "Erosion Demo",
 &erosion_size, max_kernel_size,
 Erosion );
 createTrackbar( "Element:\n 0: Rect \n 1: Cross \n 2: Ellipse", "Dilation Demo",
 &dilation_elem, max_elem,
 Dilation );
 createTrackbar( "Kernel size:\n 2n +1", "Dilation Demo",
 &dilation_size, max_kernel_size,
 Dilation );
 Erosion( 0, 0 );
 Dilation( 0, 0 );
 pix=0;
 for (i=0; i<1280; ++i){
	 for (j=0;j<960;++j){
		 if (dilation_dst.at<int>(j,i)!=0){
			 ++pix;}
	}
}
printf("nombre de pixels : %d\n", pix);
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

void Erosion( int, void* )
{
 int erosion_type = 0;
erosion_type = MORPH_ELLIPSE; 
 Mat element = getStructuringElement( erosion_type, Size( 2*erosion_size + 1, 2*erosion_size+1 ), Point( erosion_size, erosion_size ) );
 erode( frame_threshold, erosion_dst, element );
 imshow( "Erosion Demo", erosion_dst );
}
void Dilation( int, void* )
{
 int dilation_type = 0;
dilation_type = MORPH_ELLIPSE;
 Mat element = getStructuringElement( dilation_type, Size( 2*dilation_size + 1, 2*dilation_size+1 ), Point( dilation_size, dilation_size ) );
 dilate( erosion_dst, dilation_dst, element );
 imshow( "Dilation Demo", dilation_dst );
}


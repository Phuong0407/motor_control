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
int const max_elem = 2;
int const max_kernel_size = 21;
void Erosion( int, void* );
void Dilation( int, void* );
int pix=0;
int i=0;
int j =0;
int erosion_size=4;
int dilation_size=21;


int main(int argc, char* argv[])
{
 VideoCapture cap(argc > 1 ? atoi(argv[1]) : 0);
 namedWindow(window_capture_name);
 namedWindow(window_detection_name);
 while (true) {
 cap >> frame;
 if(frame.empty())
 {
 break;
 }
 
 // Convert from BGR to HSV colorspace
 cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
 // Detect the object based on HSV Range Values
 inRange(frame_HSV, Scalar(0, 112, 129), Scalar(6, 215, 254), frame_threshold);
 
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

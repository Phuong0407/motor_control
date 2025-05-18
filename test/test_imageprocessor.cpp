#include "camera.h"
#include "imageprocessor.h"
#include <pthread.h>
#include <iostream>

// lccv::PiCamera   cam;

int main() {
    
    // pthread_t CameraController;
	// pthread_create(&CameraController, NULL, computeBarycenter, NULL);
	// pthread_join(CameraController,NULL);
    
    lccv::PiCamera   cam;    
    cam.options->video_width    = framewidth;
    cam.options->video_height   = frameheight;
    cam.options->framerate      = framerate;
    cam.options->verbose        = verbose;
    cam.startVideo();

    int ch = 0;
    // startCamera();
//     ImageProcessor a;
//     cv::Mat image1(frameheight, framewidth, CV_8UC3);

//     while (ch != 27) {
//         if (!cam.getVideoFrame(image1, 1000)) {
//             std::cout << "Timeout error while grabbing frame." << std::endl;
//             continue;
//         }
//         cv::imshow("raw", image1);

//         // a.processImage(image1);
//         // cv::Mat img = a.getOutputImage();
//         // cv::imshow("processed", img);
//         // ch = cv::waitKey(5);
//     }
}
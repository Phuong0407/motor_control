#include "camera.h"
#include "imageprocessor.h"
#include <pthread.h>
#include <iostream>

int main() {
    
    // pthread_t CameraController;
	// pthread_create(&CameraController, NULL, computeBarycenter, NULL);
	// pthread_join(CameraController,NULL);
    
    
    int ch = 0;
    startCamera();
    ImageProcessor a;
    cv::Mat image1(frameheight, framewidth, CV_8UC3);

    while (ch != 27) {
        if (!cam.getVideoFrame(image1, 1000)) {
            std::cout << "Timeout error while grabbing frame." << std::endl;
            continue;
        }
        a.processImage(image1);
        cv::Mat img = a.getOutputImage();
        cv::imshow("processed", img);
        ch = cv::waitKey(5);
    }
}

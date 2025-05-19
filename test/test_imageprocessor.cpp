// #include "camera.h"
#include "imageprocessor.h"
#include <pthread.h>
#include <iostream>
#include <chrono>

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
    ImageProcessor a;
    cv::Mat image1(frameheight, framewidth, CV_8UC3);

    while (ch != 27) {
        auto start_time = std::chrono::high_resolution_clock::now();
        if (!cam.getVideoFrame(image1, 1000)) {
            std::cout << "Timeout error while grabbing frame." << std::endl;
            continue;
        }
        a.processImage(image1);
        cv::Mat img = a.getOutputImage();
        cv::imshow("processed", img);

        auto end_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed_seconds = end_time - start_time;
        std::cout << "Elapsed time: " << elapsed_seconds.count() << "s\n";
        ch = cv::waitKey(5);
    }
}
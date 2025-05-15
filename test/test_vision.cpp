#include "image_processor.hpp"
#include <opencv2/opencv.hpp>

#include <iostream>

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();
    
    cv::Mat image1(480, 640, CV_8UC3);
    cv::Mat image2(480, 640, CV_8UC3);

    ImageProcessor<4> vision;

    int ch = 0;
    while (ch != 27) {
        if (!cam.getVideoFrame(image1, 1000)) {
            std::cout << "Timeout error while grabbing frame." << std::endl;
            continue;
        }
        int direction = vision.processImage(image1, image2);
        if (direction * 0.1 > 5)
            printf("TURN LEFT\n");
        else if (direction * 0.1 < -5)
            printf("TURN RIGHT\n");
        else
            printf("GO STRAIGHT\n");
        cv::imshow("processed", image2);
        ch = cv::waitKey(5);
    }
}

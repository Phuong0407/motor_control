#include "./include/navigation/navigation.hpp"

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();
    
    cv::Mat image1(480, 640, CV_8UC3);

    Navigation naviation(1.0, 2.0, 0.08, 0.08, 0.05, 0.02);

    int ch = 0;
    while (ch != 27) {
        cam.getVideoFrame(image1, 1000);
        naviation.navigate(image1);
    }
}
#include "../include/vision/image_processor.hpp"

int main() {
    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cv::Mat image(480, 640, CV_8UC3);

    ImageProcessor a;

    int ch = 0;
    while (ch != 27) {
        
        if (!cam.getVideoFrame(image, 1000)) {
            std::cout << "Timeout error while grabbing frame." << std::endl;
            continue;
        }
        a.image = image;
        a.detectAndDrawContour();
        cv::imshow("Processed", image);
        ch = cv::waitKey(5);
    }
}
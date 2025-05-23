#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

std::vector<cv::Point2f> clickedPoints;

void onMouse(int event, int x, int y, int, void*) {
    if (event == cv::EVENT_LBUTTONDOWN && clickedPoints.size() < 4) {
        clickedPoints.emplace_back(x, y);
        std::cout << "Clicked Point " << clickedPoints.size() << ": (" << x << ", " << y << ")" << std::endl;
    }
}

int main() {

    lccv::PiCamera cam;
    cam.options->video_width = 640;
    cam.options->video_height = 480;
    cam.options->framerate = 30;
    cam.options->verbose = true;
    cam.startVideo();

    // cv::VideoCapture cap(0);
    // if (!cap.isOpened()) {
    //     std::cerr << "Camera not detected!" << std::endl;
    //     return -1;
    // }

    std::cout << "== Dual-Axis Calibration ==\n";
    std::cout << "Click 2 points horizontally (e.g. ruler along the ground).\n";
    std::cout << "Then click 2 points vertically (e.g. on a vertical stick or edge).\n";

    cv::namedWindow("Calibration", cv::WINDOW_NORMAL);
    cv::setMouseCallback("Calibration", onMouse, nullptr);

    cv::Mat frame;
    while (clickedPoints.size() < 4) {
        cap >> frame;
        if (frame.empty()) continue;

        for (const auto& pt : clickedPoints) {
            cv::circle(frame, pt, 5, cv::Scalar(0, 255, 0), -1);
        }

        cv::imshow("Calibration", frame);
        if (cv::waitKey(10) == 27) break;  // ESC to exit early
    }

    cap.release();
    cv::destroyAllWindows();

    if (clickedPoints.size() == 4) {
        float knownLengthX, knownLengthY;

        std::cout << "\nEnter the real-world horizontal length in cm (between point 1 and 2): ";
        std::cin >> knownLengthX;
        std::cout << "Enter the real-world vertical length in cm (between point 3 and 4): ";
        std::cin >> knownLengthY;

        float pixelDistanceX = cv::norm(clickedPoints[0] - clickedPoints[1]);
        float pixelDistanceY = cv::norm(clickedPoints[2] - clickedPoints[3]);

        float pixelsPerCmX = pixelDistanceX / knownLengthX;
        float pixelsPerCmY = pixelDistanceY / knownLengthY;

        std::cout << "\n--- Calibration Results ---\n";
        std::cout << "Horizontal pixel distance: " << pixelDistanceX << " px\n";
        std::cout << "Vertical pixel distance: " << pixelDistanceY << " px\n";
        std::cout << "Pixels per cm (X): " << pixelsPerCmX << "\n";
        std::cout << "Pixels per cm (Y): " << pixelsPerCmY << "\n";
        std::cout << "CM per pixel (X): " << (1.0f / pixelsPerCmX) << "\n";
        std::cout << "CM per pixel (Y): " << (1.0f / pixelsPerCmY) << "\n";
    } else {
        std::cout << "Not enough points selected for calibration.\n";
    }

    return 0;
}

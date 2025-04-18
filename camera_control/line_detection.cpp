#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>


cv::Mat extractRedMask(const cv::Mat& image) {
    cv::Mat redMask, hsvImage, binaryMask;

    cv::cvtColor(image, hsvImage, cv::COLOR_BGR2HSV);

    cv::Scalar lowerRed1(0, 120, 70), upperRed1(10, 255, 255);
    cv::Scalar lowerRed2(170, 120, 70), upperRed2(180, 255, 255);

    cv::Mat mask1, mask2;
    cv::inRange(hsvImage, lowerRed1, upperRed1, mask1);
    cv::inRange(hsvImage, lowerRed2, upperRed2, mask2);

    binaryMask = mask1 | mask2;

    return binaryMask;
}



int main() {
	uint32_t num_cams = LibcameraApp::GetNumberCameras();
	std::cout << "Found " << num_cams << " cameras." << std::endl;

    uint32_t height = 480;
    uint32_t width = 640;
    std::cout<<"Sample program for LCCV video capture"<<std::endl;
    std::cout<<"Press ESC to stop."<<std::endl;
    cv::Mat image = cv::Mat(height, width, CV_8UC3);
    cv::Mat image2 = cv::Mat(height, width, CV_8UC3);
    lccv::PiCamera cam;
    cam.options->video_width=width;
    cam.options->video_height=height;
    cam.options->framerate=30;
    cam.options->verbose=true;

    cv::namedWindow("Video",cv::WINDOW_NORMAL);
    cam.startVideo();

    int ch=0;
    while(ch!=27){
        if (!cam.getVideoFrame(image,1000)){
            std::cout<<"Timeout error"<<std::endl;
        } else {
			cv::Mat redBinary = extractRedMask(image); // Get red-only binary matrix
			int height = redBinary.rows;
			int width = redBinary.cols;

			cv::Mat roi_bottom = redBinary(cv::Rect(0, height * 5 / 6, width, height / 12));
			cv::Mat roi_top    = redBinary(cv::Rect(0, height * 4 / 6, width, height / 12));

			cv::Moments M_bot = cv::moments(roi_bottom, true);
			cv::Moments M_top = cv::moments(roi_top, true);

			cv::Point2f pt_bot(-1, -1);
			cv::Point2f pt_top(-1, -1);

			if (M_bot.m00 > 0 && M_top.m00 > 0) {
			    pt_bot = cv::Point2f(M_bot.m10 / M_bot.m00, height * 5 / 6 + height / 24);
			    pt_top = cv::Point2f(M_top.m10 / M_top.m00, height * 4 / 6 + height / 24);

			    float dx = pt_bot.x - pt_top.x;
			    float dy = pt_bot.y - pt_top.y;
			    float angle = std::atan2(dy, dx) * 180.0 / CV_PI;

			    std::cout << "Red line angle: " << angle << " degrees" << std::endl;
    
			    cv::line(image, pt_bot, pt_top, cv::Scalar(255, 255, 0), 2);
			    cv::circle(image, pt_bot, 5, cv::Scalar(0, 255, 0), -1);
			    cv::circle(image, pt_top, 5, cv::Scalar(0, 255, 0), -1);
		    } else {
			    std::cout << "Red line not detected clearly in both regions." << std::endl;
}



			cv::imshow("Red Mask", redBinary);
			cv::imshow("Video", image);
		}
        ch=cv::waitKey(5);
    }

    cam.stopVideo();
	cv::destroyAllWindows();
}


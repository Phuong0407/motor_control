#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <csignal>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include "control_motores.h"

using namespace std;
using namespace cv;

// Global Variables
int Speed = 50;
int w = 640;
int h = 240;
int bytesPerFrame = w * h;
int fps = 40;
int lateral_search = 20;
int start_height = h - 5;
int no_points_count = 0;
vector<Mat> frames;
pid_t cameraProcessPID;

// Function Prototypes
void cleanupFinish();
void signalHandler(int sig);
void startCamera();
void processFrame(Mat& frame);

int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <direction divisor>" << endl;
        return -1;
    }

    int divisor = atoi(argv[1]);

    signal(SIGINT, signalHandler);

    MotorsSetup();
    BaseSpeed(Speed);

    startCamera();

    while (true) {
        FILE* pipe = popen("raspividyuv -w 640 -h 240 --output - --timeout 0 -vs -co 50 -br 50 --framerate 40 --luma --nopreview", "r");

        if (!pipe) {
            cerr << "Error: Could not open camera stream!" << endl;
            break;
        }

        vector<unsigned char> buffer(bytesPerFrame);
        size_t bytesRead = fread(buffer.data(), 1, bytesPerFrame, pipe);

        if (bytesRead != bytesPerFrame) {
            cerr << "Error: Camera stream closed unexpectedly!" << endl;
            pclose(pipe);
            break;
        }

        Mat frame(h, w, CV_8UC1, buffer.data());
        processFrame(frame);

        if (no_points_count > 50 || Speed <= 0) {
            cout << "Line lost or speed too low, exiting..." << endl;
            pclose(pipe);
            break;
        }

        pclose(pipe);
    }

    cleanupFinish();
    return 0;
}

void processFrame(Mat& frame) {
    int middle;
    Mat frame_rgb;
    cvtColor(frame, frame_rgb, COLOR_GRAY2BGR);

    Mat thresh;
    adaptiveThreshold(frame, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 11, 2);

    Mat row = thresh.row(start_height);
    Mat diff;
    absdiff(row.colRange(1, row.cols), row.colRange(0, row.cols - 1), diff);

    vector<int> points;
    for (int i = 0; i < diff.cols; i++) {
        if (diff.at<uchar>(0, i) > 200) {
            points.push_back(i);
        }
    }

    line(frame_rgb, Point(0, start_height), Point(w, start_height), Scalar(0, 255, 0), 1);

    if (points.size() >= 2) {
        if (GetSpeed() == 0) {
            BaseSpeed(Speed);
        }

        middle = (points[0] + points[1]) / 2;

        circle(frame_rgb, Point(points[0], start_height), 2, Scalar(255, 0, 0), -1);
        circle(frame_rgb, Point(points[1], start_height), 2, Scalar(255, 0, 0), -1);
        circle(frame_rgb, Point(middle, start_height), 2, Scalar(0, 0, 255), -1);

        int direction = (middle - 320) / divisor;
        Direction(direction);
        cout << "Direction: " << direction << endl;

    } else {
        start_height = (start_height - 5) % h;
        no_points_count++;
        Speed -= 1;
        BaseSpeed(Speed);
    }

    frames.push_back(frame_rgb);
    frames.push_back(thresh);

    if (frames.size() > 100) {
        frames.erase(frames.begin());
    }
}

void startCamera() {
    cout << "Recording..." << endl;
}

void cleanupFinish() {
    MotorsStop();

    cout << "Writing frames to disk..." << endl;
    VideoWriter writer("drive_test.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 5, Size(w, h));

    for (const auto& frame : frames) {
        writer.write(frame);
    }

    writer.release();
    cout << "Recording saved to drive_test.avi" << endl;
}

void signalHandler(int sig) {
    cout << "Terminating..." << endl;
    cleanupFinish();
    exit(0);
}

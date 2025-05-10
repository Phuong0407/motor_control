#ifndef CAMERA_HPP
#define CAMERA_HPP

#include <lccv.hpp>
#include <libcamera_app.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

/**
 * @class Camera
 * @brief Interface with the Raspberry Pi camera using LCCV and OpenCV.
 * @brief The camera is initialized and start simultanuously;
 * @brief While it is stopped seperately.
 */
class Camera{
private:
    lccv::PiCamera cam;

public:
    Camera(
        int frame_width,
        int frame_height,
        int framerate,
        bool verbose
    ) {
        cam.options->video_width = frame_width;
        cam.options->video_height = frame_height;
        cam.options->framerate = framerate;
        cam.options->verbose = verbose;

        int ret_code = cam.startVideo();
        if (ret_code) printf("[ERROR] Failed to initialize camera with error code: %d\n", ret_code);
        else printf("[INFO] Camera initialized successfully with resolution %dx%d.\n", frame_width, frame_height);
    }

    void stopVideo() {
        cam.stopVideo();
        printf("[INFO] Camera stops recording video.\n");
    }

    /**
     * @brief Captures a single frame from the video source and stores it in the specified `image` matrix.
     * @param image Reference to a `cv::Mat` object where the captured frame will be stored.
     *              The matrix must be pre-allocated with the size `(video_height, video_width)`.
     * @param timeout Time in milliseconds to wait for the frame. It is recommended to set this value to 
     *                at least 500 ms to avoid potential timeout errors.
     * @note The `image` matrix must be properly initialized with the expected size. 
     * @note A timeout value of less than 500 ms may increase the likelihood of a capture failure.
     * @return `true` if a frame is successfully captured and stored in `image`; `false` if a timeout or other error occurs.
     */
    bool captureFrame(cv::Mat &image, int timeout) {
        if (!cam.getVideoFrame(image, timeout)) {
            printf("[ERROR] Timeout while grabbing frame.\n");
            return false;
        }
        return true;
    }

    inline int getVideoWidth() const { return cam.options->video_width; }
    inline int getVideoHeight() const { return cam.options->video_height; }
};

#endif // CAMERA_HPP
#include "camera.h"
#include "vision.h"
#include <pthread.h>
#include <iostream>

int main() {
    startCamera();
    pthread_t CameraController;
	pthread_create(&CameraController, NULL, computeBaryCenter, NULL);
	pthread_join(CameraController,NULL);


    // int ch = 0;
    // while (ch != 27) {
    //     if (!cam.getVideoFrame(image1, 1000)) {
    //         std::cout << "Timeout error while grabbing frame." << std::endl;
    //         continue;
    //     }
    //     int direction = vision.processImage(image1, image2);
    //     if (direction * 0.2 < -5.0)
    //         printf("TURN LEFT\n");
    //     else if (direction * 0.2 > 5.0)
    //         printf("TURN RIGHT\n");
    //     else
    //         printf("GO STRAIGHT\n");
    //     cv::imshow("processed", image2);
    //     ch = cv::waitKey(5);
    // }
}

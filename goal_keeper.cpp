#include "camera.h"
#include "vision.h"
#include "control.h"

int main() {
    startCamera();

    pthread_t Vision;
    pthread_t Control;

    pthread_create(&Vision, 		NULL, extractBallCenter, 		NULL);
	pthread_create(&Control, 		NULL, controlRobotGoalKeeper, 	NULL);

    pthread_join(Vision, 		NULL);
	pthread_join(Control, 		NULL);

    if (TERMINATE_PROGRAM) {
        printf("\nPROGRAM STOP NOW.\n");
		stopAllMotors();
		cv::destroyAllWindows();
		exit(0);
    }
    return 0;
}
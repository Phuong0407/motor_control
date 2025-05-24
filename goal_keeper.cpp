#include "camera.h"
#include "vision.h"
#include "control.h"

int main() {
    startCamera();

    pthread_t Vision;
    pthread_t Control;
    pthread_t NoBallHandler;

    pthread_create(&Vision, 		NULL, extractBallCenter, 		NULL);
	pthread_create(&Control, 		NULL, controlRobotGoalKeeper, 	NULL);
    pthread_create(&NoBallHandler, 	NULL, handleNoBallFound, 		NULL);

    pthread_join(Vision,            NULL);
	pthread_join(Control, 		    NULL);
	pthread_join(NoBallHandler, 	NULL);

    if (TERMINATE_PROGRAM) {
        printf("\nPROGRAM STOP NOW.\n");
		stopAllMotors();
		cv::destroyAllWindows();
		exit(0);
    }
    return 0;
}
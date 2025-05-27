// #define NOLOADED_RUN
#define LOADED_RUN

#include "camera.h"
#include "vision.h"
#include "velocity.h"
#include "encoder.h"
#include "control.h"
#include "motor.h"

#include <string>
#include <stdio.h>

int main(int argc, char *argv[]) {
	startCamera();
    startEncoders();

	pthread_t Vision;
    pthread_t Velocity;
    pthread_t Control1;
    pthread_t Control2;
    pthread_t Control3;
    pthread_t StuckSolver;
    pthread_t SpeedMonitor;

	pthread_create(&Vision, 		NULL, computeBarycenter, 		NULL);
	pthread_create(&Velocity, 		NULL, computeRefTPSFromVision, 	NULL);
	pthread_create(&Control1, 		NULL, controlMotor1, 			NULL);
	pthread_create(&Control2, 		NULL, controlMotor2, 			NULL);
	pthread_create(&Control3, 		NULL, controlMotor3, 			NULL);
	pthread_create(&StuckSolver, 	NULL, overcomeStuckState, 		NULL);
	pthread_create(&SpeedMonitor, 	NULL, monitorMotorsSpeed, 		NULL);

	pthread_join(Vision, 		NULL);
	pthread_join(Velocity, 		NULL);
	pthread_join(Control1, 		NULL);
	pthread_join(Control2, 		NULL);
	pthread_join(Control3, 		NULL);
	pthread_join(StuckSolver, 	NULL);
	pthread_join(SpeedMonitor, 	NULL);

	if (TERMINATE_PROGRAM) {
		printf("\n[INFO] PROGRAM STOP NOW.\n");
		cv::destroyAllWindows();
		stopAllMotors();
	}

	return 0;
}
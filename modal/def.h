#ifndef DEFINE_h
#define DEFINE_h
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <unistd.h>
#include <pthread.h>
#include <errno.h>
#include <wiringPi.h>
#include <stdlib.h>
#include <signal.h>
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <math.h>
using namespace std;
using namespace cv;

void * recup_bary_seuil(void * b);

#endif


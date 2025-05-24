#ifndef CAMERA_H
#define CAMERA_H

#include "robot.h"

void startCamera() {
    cam.options->video_width    = FRAMEWIDTH;
    cam.options->video_height   = FRAMEHEIGHT;
    cam.options->framerate      = FRAMERATE;
    cam.options->verbose        = VERBOSE;
    cam.startVideo();
}

#endif // CAMERA_H
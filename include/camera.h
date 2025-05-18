#ifndef CAMERA_H
#define CAMERA_H

#include "robot.h"

void startCamera() {
    cam.options->video_width    = framewidth;
    cam.options->video_height   = frameheight;
    cam.options->framerate      = framerate;
    cam.options->verbose        = verbose;
    cam.startVideo();
}

#endif // CAMERA_H
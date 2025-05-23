#include "kasilas/kasilas.h"

int main() {
    cam.options->video_width    = FRAME_WIDTH;
    cam.options->video_height   = FRAME_HEIGHT;
    cam.options->framerate      = FRAMERATE;
	cam.options->verbose        = VERBOSE;
    cam.startVideo();

    while (true) {
        
    }
}
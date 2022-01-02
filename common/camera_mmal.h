#ifndef CAMERA_MMAL_H
#define CAMERA_MMAL_H

#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <time.h>

//camera/mmal/raspberry specific libraries
#include "bcm_host.h"
#include "mmal.h"
#include "util/mmal_default_components.h"
#include "util/mmal_util.h"
#include "util/mmal_connection.h"
#include "util/mmal_util_params.h"
#include "interface/vcos/vcos.h"

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

//source: https://github.com/raspberrypi/userland/blob/master/interface/mmal/test/examples/example_basic_2.c
#define CHECK_STATUS(status, msg) if (status != MMAL_SUCCESS) { fprintf(stderr, msg"\n\r");}

sem_t semaphore_cam_buffer;

void framebuffer_init(char **framebuffer_out, uint *screen_size_x, uint *screen_size_y);
void camera_mmal_init(MMAL_PORT_T **video_port, MMAL_POOL_T **pool, uint camera_resolution_x, uint camera_resolution_y, uint shutter_speed, uint framerate, uint awb_mode);
void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

#endif
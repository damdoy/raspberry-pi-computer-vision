#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <semaphore.h>
#include <time.h>
#include <math.h>

//camera/mmal/raspberry specific libraries
#include "bcm_host.h"
#include "mmal.h"
#include "util/mmal_default_components.h"
#include "util/mmal_util.h"
#include "util/mmal_connection.h"
#include "util/mmal_util_params.h"
#include "interface/vcos/vcos.h"

#include "../common/camera_mmal.h"
#include "../common/image.h"
#include "../common/edge_detect.h"

//will affect framerate, it seems that if framerate is higher than possible shutter speed, it will be automatically lowered
#define CAMERA_SHUTTER_SPEED 24000

//framerate above 30 only possible for some resolution, depends on the camera
//can also reduce the displayed portion of the camera on screen
#define MAX_CAMERA_FRAMERATE 30

//resolution needs to be smaller than the screen size
#define CAMERA_RESOLUTION_X 800
#define CAMERA_RESOLUTION_Y 600

#define CHECK_STATUS(status, msg) if (status != MMAL_SUCCESS) { fprintf(stderr, msg"\n\r");}

char *fbp;
uint32_t screen_size_x = 0;
uint32_t screen_size_y = 0;

static int cur_sec;

extern sem_t semaphore_cam_buffer;

void init_time_keeping();
float get_cur_time();

void sobel_edge_detect(image_grayscale_t *img_in, image_grayscale_t *out);
void edge_thinning(image_grayscale_t *img_in, image_grayscale_t *out);

void main(void){
    //sets up the framebuffer, will draw
    framebuffer_init(&fbp, &screen_size_x, &screen_size_y);

    MMAL_PORT_T *video_port;
    MMAL_POOL_T *pool;
    MMAL_BUFFER_HEADER_T *buffer;

    camera_mmal_init(&video_port, &pool, CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y, CAMERA_SHUTTER_SPEED, MAX_CAMERA_FRAMERATE, MMAL_PARAM_AWBMODE_INCANDESCENT);

    float time_since_report = 0.0f;
    int count_frames = 0;

    float start_time;
    float end_time;
    float start_profiling_time;
    float end_profiling_time;

    init_time_keeping();

    image_rgb_t img;
    img.width = CAMERA_RESOLUTION_X;
    img.height = CAMERA_RESOLUTION_Y;

    image_grayscale_t img_gray;
    image_grayscale_t img_edge;

    while(1){
        start_time = get_cur_time();

        //wait until a buffer has been received
        sem_wait(&semaphore_cam_buffer);

        buffer = mmal_queue_get(pool->queue);

        img.img = buffer->data;

        image_convert_to_grayscale(&img, &img_gray);

        sobel_edge_detect(&img_gray, &img_edge);

        // save to raw file
        // save_image_grayscale_to_file(&img_edge, "img.raw");

        //Send back the buffer to the port to be filled with an image again
        mmal_port_send_buffer(video_port, buffer);

        image_draw_grayscale(&img_edge, fbp, screen_size_x);


        end_time = get_cur_time();
        float seconds = (float)(end_time - start_time);
        time_since_report += seconds;
        count_frames++;

        if(time_since_report > 1.0f){
            float framerate = count_frames/time_since_report;
            printf("frequency: %fHz\n\r", framerate);
            time_since_report = 0;
            count_frames = 0;
        }

        // printf("profiling time: %f\n\r", end_profiling_time-start_profiling_time);

        free(img_edge.img);
    }

    //todo free the mmal and framebuffer ressources cleanly
}

//clock_gettime is a better time keeping mechanism than other on the raspberry pi
void init_time_keeping(){
    struct timespec time_read;
    clock_gettime(CLOCK_REALTIME, &time_read);
    cur_sec = time_read.tv_sec; //global
}

float get_cur_time(){
    struct timespec time_read;
    clock_gettime(CLOCK_REALTIME, &time_read);
    return (time_read.tv_sec-cur_sec)+time_read.tv_nsec/1000000000.0f;
}
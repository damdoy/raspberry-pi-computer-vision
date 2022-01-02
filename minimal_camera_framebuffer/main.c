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

#include "../common/camera_mmal.h"

//will affect framerate, it seems that if framerate is higher than possible shutter speed, it will be automatically lowered
#define CAMERA_SHUTTER_SPEED 25000

//framerate above 30 only possible for some resolution, depends on the camera
//can also reduce the displayed portion of the camera on screen
#define MAX_CAMERA_FRAMERATE 40

//resolution needs to be smaller than the screen size
#define CAMERA_RESOLUTION_X 1440
#define CAMERA_RESOLUTION_Y 1080

char *fbp;
uint32_t screen_size_x = 0;
uint32_t screen_size_y = 0;

static int cur_sec;

extern sem_t semaphore_cam_buffer;

void init_time_keeping();
float get_cur_time();

void main(void){
    MMAL_PORT_T *video_port;
    MMAL_POOL_T *pool;
    MMAL_BUFFER_HEADER_T *buffer;

    //sets up the framebuffer, will draw
    framebuffer_init(&fbp, &screen_size_x, &screen_size_y);

    camera_mmal_init(&video_port, &pool, CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y, CAMERA_SHUTTER_SPEED, MAX_CAMERA_FRAMERATE, MMAL_PARAM_AWBMODE_INCANDESCENT);

    // printf("out\n");
    float time_since_report = 0.0f;
    int count_frames = 0;

    float start_time;
    float end_time;
    float start_copy_time;
    float end_copy_time;

    init_time_keeping();

    while(1){
        start_time = get_cur_time();

        //wait until a buffer has been received
        sem_wait(&semaphore_cam_buffer);

        buffer = mmal_queue_get(pool->queue);

        start_copy_time = get_cur_time();

        //draw the image on the top left corner of the framebuffer
        //would be less costly to limit frambuffer size and just do a memcpy
        int img_idx = 0;
        int framebuffer_idx = 0;
        for(int i = 0; i < CAMERA_RESOLUTION_Y; i++){
           for(int j = 0; j < CAMERA_RESOLUTION_X; j++){
               //seem that R and B components are inverted
               fbp[framebuffer_idx] = buffer->data[img_idx+2];
               fbp[framebuffer_idx+1] = buffer->data[img_idx+1];
               fbp[framebuffer_idx+2] = buffer->data[img_idx+0];
               img_idx +=  3;
               framebuffer_idx += 3;
           }
           framebuffer_idx = i*screen_size_x*3;
        }

        end_copy_time = get_cur_time();
        // printf("frame copy time: %f\n\r", end_copy_time-start_copy_time);

        //Send back the buffer to the port to be filled with an image again
        mmal_port_send_buffer(video_port, buffer);

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

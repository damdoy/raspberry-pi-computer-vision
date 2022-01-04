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
#include "../common/image.h"

//will affect framerate, it seems that if framerate is higher than possible shutter speed, it will be automatically lowered
#define CAMERA_SHUTTER_SPEED 24000

//framerate above 30 only possible for some resolution, depends on the camera
//can also reduce the displayed portion of the camera on screen
#define MAX_CAMERA_FRAMERATE 30

//resolution needs to be smaller than the screen size
#define CAMERA_RESOLUTION_X 800
#define CAMERA_RESOLUTION_Y 600

char *fbp;
uint32_t screen_size_x = 0;
uint32_t screen_size_y = 0;

static int cur_sec;

typedef struct feature_point_t{
    int x;
    int y;
    int level;
} feature_point_t;

extern sem_t semaphore_cam_buffer;

void init_time_keeping();
float get_cur_time();

uint check_fast_point(image_grayscale_t *image, float thresh, uint pos_x, uint pos_y);
void find_fast_points(image_grayscale_t image, feature_point_t *points, uint *nb_points, uint granularity, uint max_points, float threshold);

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

    while(1){
        start_time = get_cur_time();

        //wait until a buffer has been received
        sem_wait(&semaphore_cam_buffer);

        buffer = mmal_queue_get(pool->queue);

        image_rgb_t img;
        img.width = CAMERA_RESOLUTION_X;
        img.height = CAMERA_RESOLUTION_Y;
        img.img = buffer->data;

        const uint TOTAL_LEVELS_PYRAMID = 3;
        const float THRESHOLD_DETECTION = 0.07f;
        const uint PYRAMID_BLUR = 1; //means box blur filter of size (PYRAMID_BLUR*2+1)
        const uint MAX_POINTS_PER_PYRAMID = 64;

        image_grayscale_t img_gray[TOTAL_LEVELS_PYRAMID];
        image_grayscale_t img_gray_blurred[TOTAL_LEVELS_PYRAMID];

        image_convert_to_grayscale(&img, &img_gray[0]); //13ms

        uint factor = 1;
        for (size_t i = 0; i < TOTAL_LEVELS_PYRAMID; i++)
        {
            blur_grayscale_image(&img_gray[i], &img_gray_blurred[i], PYRAMID_BLUR); //38ms

            feature_point_t points[64];
            uint nb_points;
            uint granularity_search = 3;

            // start_profiling_time = get_cur_time();
            find_fast_points(img_gray_blurred[i], points, &nb_points, granularity_search, MAX_POINTS_PER_PYRAMID, THRESHOLD_DETECTION); // 34ms
            // end_profiling_time = get_cur_time();

            //draw the feature points on the image with appropriate size
            for (size_t ipt = 0; ipt < nb_points; ipt++)
            {
                draw_circle(points[ipt].x*factor, points[ipt].y*factor, 4*factor, &img);
            }


            if(i < TOTAL_LEVELS_PYRAMID-1){
                downscale_gray_image(&img_gray_blurred[i], &img_gray[i+1]); //1ms
            }

            factor*=2;
        }

        // image_draw_grayscale(&img_gray[0], fbp, screen_size_x);

        image_draw(&img, fbp, screen_size_x); //11ms

        // save to raw file
        // save_image_rgb_to_file(&img, "img.raw");

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


        // printf("profiling time: %f\n\r", end_profiling_time-start_profiling_time);

        //destroy buffers
        for (size_t i = 0; i < TOTAL_LEVELS_PYRAMID; i++){
            free(img_gray[i].img);
            free(img_gray_blurred[i].img);
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

//apply FAST algorithm to find if current position is a feature
//circle around centre has radius of 3
uint check_fast_point(image_grayscale_t *image, float thresh, uint pos_x, uint pos_y){
    //all relative positions (x,y) around centre, 16 points
    static int lst_offset_fast[] = {0, -3, 1, -3, 2, -2, 3, -1, 3, 0, 3, 1, 2, 2, 1, 3, 0, 3, -1, 3, -2, 2, -3, 1, -3, 0, -3, -1, -2, -2, -1, -3};

    uint val_cur_pix = image_grayscale_get(image, pos_x, pos_y);

    float thresh_val_pos_f = val_cur_pix+255.0f*thresh;
    float thresh_val_neg_f = val_cur_pix-255.0f*thresh;

    int thresh_val_pos = thresh_val_pos_f;
    int thresh_val_neg = thresh_val_neg_f;

    if(thresh_val_pos_f > 255){
        thresh_val_pos = 255;
    }
    if(thresh_val_neg_f < 0){
        thresh_val_neg = 0;
    }

    int count_bigger = 0;
    int count_lower = 0;

    uint array_pixel_bigger[16] = {0};
    uint array_pixel_smaller[16] = {0};

    for(int i = 0; i < 32; i+=8){ //only check 4 points
        if(image_grayscale_get(image, pos_x+lst_offset_fast[i], pos_y+lst_offset_fast[i+1]) > thresh_val_pos){
            count_bigger++;
            array_pixel_bigger[i/2] = 1;
        }

        if(image_grayscale_get(image, pos_x+lst_offset_fast[i], pos_y+lst_offset_fast[i+1]) < thresh_val_neg){
            count_lower++;
            array_pixel_smaller[i/2] = 1;
        }
    }

    //if less than 3 points are not above threshold, then no need to check for 12 continuous points
    if(count_bigger>=3){
        for(int i = 0; i < 32; i+=2){
            if(image_grayscale_get(image, pos_x+lst_offset_fast[i], pos_y+lst_offset_fast[i+1]) > thresh_val_pos){
                array_pixel_bigger[i/2] = 1;
            }
        }
    }

    if(count_lower>=3){
        for(int i = 0; i < 32; i+=2){
            if(image_grayscale_get(image, pos_x+lst_offset_fast[i], pos_y+lst_offset_fast[i+1]) < thresh_val_neg){
                array_pixel_smaller[i/2] = 1;
            }
        }
    }

    int max_continuous_bigger = 0;
    int max_continuous_smaller = 0;
    int continuous_bigger = 0;
    int continuous_smaller = 0;

    //go two times around the circle to check for continuous pixels above threshold
    for (size_t i = 0; i < 32; i++) {
        if(array_pixel_bigger[i%16]){
            continuous_bigger++;
        }
        else{
            if(continuous_bigger > max_continuous_bigger){
                max_continuous_bigger = continuous_bigger;
            }
            continuous_bigger = 0;
        }
        if(array_pixel_smaller[i%16]){
            continuous_smaller++;
        }
        else{
            if(continuous_smaller > max_continuous_smaller){
                max_continuous_smaller = continuous_smaller;
            }
            continuous_smaller = 0;
        }
    }

    if(continuous_bigger > max_continuous_bigger){
        max_continuous_bigger = continuous_bigger;
    }
    if(continuous_smaller > max_continuous_smaller){
        max_continuous_smaller = continuous_smaller;
    }

    return  max_continuous_bigger>=12 || max_continuous_smaller>=12;
}

//check in all image, given granularty for fast points
void find_fast_points(image_grayscale_t image, feature_point_t *points, uint *nb_points, uint granularity, uint max_points, float threshold){

    uint current_head = 0;

    for (size_t i = 10; i < image.height-10; i+=granularity)
        {
            for (size_t j = 10; j < image.width-10; j+=granularity)
            {
                if(check_fast_point(&image, threshold, j, i) && current_head < max_points){
                    points[current_head].x = j;
                    points[current_head].y = i;
                    points[current_head].level = 0;
                    current_head++;
                }
            }
        }

    *nb_points = current_head;
}
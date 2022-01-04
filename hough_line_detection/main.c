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
#define CAMERA_SHUTTER_SPEED 15000

//framerate above 30 only possible for some resolution, depends on the camera
//can also reduce the displayed portion of the camera on screen
#define MAX_CAMERA_FRAMERATE 30

//resolution needs to be smaller than the screen size
#define CAMERA_RESOLUTION_X 480
#define CAMERA_RESOLUTION_Y 480

#define CHECK_STATUS(status, msg) if (status != MMAL_SUCCESS) { fprintf(stderr, msg"\n\r");}

char *fbp;
uint32_t screen_size_x = 0;
uint32_t screen_size_y = 0;

static int cur_sec;

extern sem_t semaphore_cam_buffer;

void init_time_keeping();
float get_cur_time();

void get_hough_transform(image_grayscale_t *img_in, image_grayscale32_t *img_out, uint size_hough_x, uint size_hough_y);
void increment_hough_value(image_grayscale32_t *img, uint x, uint y, uint diag);

void draw_inverse_hough_transform(image_rgb_t *img_out, image_grayscale32_t *hough_transf, uint threshold);
void draw_inverse_hough_transform_point(image_rgb_t *img_out, float theta, float rho);

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
    image_grayscale_t img_canny;

    image_grayscale32_t img_hough_trans;

    //a size of hough transform similar to the size of the image
    //give the best results
    #define SIZE_HOUGH_X 400
    #define SIZE_HOUGH_Y 400

    while(1){
        start_time = get_cur_time();

        //wait until a buffer has been received
        sem_wait(&semaphore_cam_buffer);

        buffer = mmal_queue_get(pool->queue);

        img.img = buffer->data;

        image_convert_to_grayscale(&img, &img_gray);

        get_canny(&img_gray, &img_canny);

        //will transform the edge image (canny) into the hough tranform
        get_hough_transform(&img_canny, &img_hough_trans, SIZE_HOUGH_X, SIZE_HOUGH_Y);

        uint hough_threshold = img.height/2;
        draw_inverse_hough_transform(&img, &img_hough_trans, hough_threshold);


        // save to raw file
        // save_image_rgb_to_file(&img, "img.raw");

        image_draw(&img, fbp, screen_size_x);

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

        free(img_gray.img);
        free(img_canny.img);
        free(img_hough_trans.img);

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

void get_hough_transform(image_grayscale_t *img_in, image_grayscale32_t *hough_img_out, uint size_hough_x, uint size_hough_y)
{
    hough_img_out->width = size_hough_x;
    hough_img_out->height = size_hough_y;
    hough_img_out->img = (uint32_t*)malloc(hough_img_out->width*hough_img_out->height*sizeof(uint32_t));

    //set the hough image to 0
    memset(hough_img_out->img, 0, hough_img_out->width*hough_img_out->height*sizeof(uint32_t));

    uint diag = sqrt(img_in->width*img_in->width+img_in->height*img_in->height);

    //for each pixels in the input image calculate the hough curve if it is an edge
    for (uint y = 0; y < img_in->height; y++)
    {
        for (uint x = 0; x < img_in->width; x++)
        {
            if(image_grayscale_get(img_in, x, y) > 0)
            {
                increment_hough_value(hough_img_out, x, y, diag);
            }
        }

    }

}

void increment_hough_value(image_grayscale32_t *hough_img, uint pix_x, uint pix_y, uint diag)
{
    float theta_increment = 3.141592f/hough_img->width;
    float rho_increment = 2*diag/hough_img->height;
    float current_val_theta = 0;

    //calculate value of rho for first pixel on x axis
    float rho_val = pix_x*cos(current_val_theta)+pix_y*sin(current_val_theta);
    uint y_coord = hough_img->height/2 + rho_val/(rho_increment);
    uint y_coord_prev = y_coord;
    image_grayscale32_increment_pix(hough_img, 0, y_coord);

    //increment x, find rho value and draw it
    for(uint x = 1; x < hough_img->width; x++){
        current_val_theta+=theta_increment;
        rho_val = pix_x*cos(current_val_theta)+pix_y*sin(current_val_theta);
        y_coord = hough_img->height/2 + rho_val/(rho_increment);

        // //will make lines of the hough transform thicker
        // if(y_coord <= y_coord_prev){
        //     for(uint i = y_coord; i < y_coord_prev; i++){

        //         if(i >= 0 && i < hough_img->height)
        //         {
        //             image_grayscale32_increment_pix(hough_img, x-1, i);
        //         }
        //     }
        // }
        if(y_coord >= 0 && y_coord < hough_img->height)
        {
            image_grayscale32_increment_pix(hough_img, x, y_coord);
        }
        y_coord_prev = y_coord;
    }
}

void draw_inverse_hough_transform(image_rgb_t *img_out, image_grayscale32_t *hough_transf, uint threshold)
{
    uint diag = sqrt(img_out->width*img_out->width+img_out->height*img_out->height);

    float theta_increment = 3.141592f/hough_transf->width;
    float rho_increment = 2*diag/hough_transf->height;

    //check all pixels in the hough transform image, if higher than threshold, draw the line
    for (int y = 3; y < hough_transf->height-3; y++)
    {
        for (int x = 3; x < hough_transf->width-3; x++)
        {

            uint32_t current_hough_val = image_grayscale32_get(hough_transf, x, y);

            if(current_hough_val >= threshold)
            {
                //check that there is no higher values in the vicinity
                uint is_largest = 1;
                for (int j = -3; j < 3; j++)
                {
                    for (int i = -3; i < 3; i++)
                    {
                        if(is_largest && current_hough_val < image_grayscale32_get(hough_transf, x+i, y+j)){
                            is_largest = 0;
                        }
                    }
                }

                if(is_largest)
                {
                    //transform from hough image to rho/theta values
                    float theta = x*theta_increment;
                    float rho = ((int)y-((int)hough_transf->height/2))*rho_increment;
                    draw_inverse_hough_transform_point(img_out, theta, rho);
                }
            }
        }

    }

}

void draw_inverse_hough_transform_point(image_rgb_t *img_out, float theta, float rho)
{
    int start_x = -1;
    int end_x = -1;

    int start_y = -1;
    int end_y = -1;

    //iterate over all x, find the first valid value in image, and the last one, this should be the line to draw
    for (size_t x = 0; x < img_out->width; x++)
    {
        float val_y = (rho-x*cos(theta) )/sin(theta);

        //first valid point
        if( (val_y >= 0 && val_y < img_out->height) && start_y == -1){
            start_x = x;
            start_y = val_y;
        }

        //last valid point
        if( (val_y < 0 || val_y >= img_out->height) && start_y != -1 && end_y == -1){
            val_y = (rho-(x-1)*cos(theta) )/sin(theta);
            end_x = x-1;
            end_y = val_y;
        }

    }

    float val_y = (rho-(img_out->width-1)*cos(theta) )/sin(theta);

    //checks that line is still within the image at the end of the sweep ==> add it as last point
    if( (val_y >= 0 || val_y < img_out->height) && start_y != -1 && end_y == -1){
        end_x = (img_out->width-1);
        end_y = val_y;
    }

    if(start_x == -1 || end_x == -1 || start_y == -1 || end_y == -1)
    {
        start_x = start_y = end_x = end_y = 0;
        return;
    }

    uint colour[3] = {0, 255, 0};
    draw_line(start_x, start_y, end_x, end_y, img_out, colour);
}
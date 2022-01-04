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

//will affect framerate, it seems that if framerate is higher than possible shutter speed, it will be automatically lowered
#define CAMERA_SHUTTER_SPEED 15000

//framerate above 30 only possible for some resolution, depends on the camera
//can also reduce the displayed portion of the camera on screen
#define MAX_CAMERA_FRAMERATE 30

//resolution needs to be smaller than the screen size
#define CAMERA_RESOLUTION_X 640
#define CAMERA_RESOLUTION_Y 480

#define CHECK_STATUS(status, msg) if (status != MMAL_SUCCESS) { fprintf(stderr, msg"\n\r");}

char *fbp;
uint32_t screen_size_x = 0;
uint32_t screen_size_y = 0;

static int cur_sec;

extern sem_t semaphore_cam_buffer;

void init_time_keeping();
float get_cur_time();

float calc_dx(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y);
float calc_dy(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y);
float calc_dt(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y);

void calc_optical_flow(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y, float *u_x, float *u_y);

void main(void){
    //sets up the framebuffer, will draw
    framebuffer_init(&fbp, &screen_size_x, &screen_size_y);

    MMAL_PORT_T *video_port;
    MMAL_POOL_T *pool;
    MMAL_BUFFER_HEADER_T *buffer;
    MMAL_BUFFER_HEADER_T *prev_buffer;

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

    image_rgb_t img_prev;
    img_prev.width = CAMERA_RESOLUTION_X;
    img_prev.height = CAMERA_RESOLUTION_Y;

    //allocate mem for previous image, it will just be memcpyied every image from the current image
    img_prev.img = malloc(img.width*img.height*3);

    image_grayscale_t img_gray_prev;
    image_grayscale_t img_gray;

    sem_wait(&semaphore_cam_buffer);

    buffer = mmal_queue_get(pool->queue);

    img.img = buffer->data;

    image_convert_to_grayscale(&img, &img_gray);

    while(1){
        start_time = get_cur_time();

        //wait until a buffer has been received
        sem_wait(&semaphore_cam_buffer);

        buffer = mmal_queue_get(pool->queue);

        //keep old image in memory
        memcpy(img_prev.img, img.img, img.width*img.height*3);

        //update current image
        img.img = buffer->data;

        img_gray_prev = img_gray;
        image_convert_to_grayscale(&img, &img_gray);

        uint col[3] = {0, 255, 0};

        const uint space_between_points = 10;

        //start and end *2 to avoid seg fault
        for (uint i = space_between_points*2; i < img.width-space_between_points*2; i+=space_between_points)
        {
            for (uint j = space_between_points*2; j < img.height-space_between_points*2; j+=space_between_points){
                //flow vector variables
                float u_x, u_y;
                calc_optical_flow(&img_gray, &img_gray_prev, i, j, &u_x, &u_y);

                //convert flow vector to polar for simpler handling
                float angle = atan2(u_y, u_x);
                float mag = sqrt(u_x*u_x+u_y*u_y);

                //clamp crazy vectors to avoid mess in the display
                if(mag > space_between_points)
                {
                    mag = space_between_points;
                }

                //only draw significant vectors
                if(mag >= 2){
                    //draw a starting point in red
                    for (int k = -1; k <= 1; k++)
                    {
                        for (int l = -1; l <= 1; l++)
                        {
                            image_set(&img_prev, i+k, j+l, 0, 255);
                            image_set(&img_prev, i+k, j+l, 1, 0);
                            image_set(&img_prev, i+k, j+l, 2, 0);
                        }

                    }

                    //draw the flow vector
                    uint col_green[3] = {0, 255, 0};

                    int iu_x = mag*cos(angle);
                    int iu_y = mag*sin(angle);
                    draw_line(i, j, i+iu_x, j+iu_y, &img_prev, col_green);
                }
            }
        }

        image_draw(&img_prev, fbp, screen_size_x);

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

        //free buffers
        free(img_gray_prev.img);
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

float calc_dx(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y)
{
    float val = 0;
    // val += image_grayscale_get(img, pos_x+1, pos_y-1);
    // val += image_grayscale_get(img, pos_x+1, pos_y);
    // val += image_grayscale_get(img, pos_x+1, pos_y+1);
    // val -= image_grayscale_get(img, pos_x-1, pos_y-1);
    // val -= image_grayscale_get(img, pos_x-1, pos_y);
    // val -= image_grayscale_get(img, pos_x-1, pos_y+1);

    val += image_grayscale_get(img_prev, pos_x+1, pos_y-1);
    val += image_grayscale_get(img_prev, pos_x+1, pos_y);
    val += image_grayscale_get(img_prev, pos_x+1, pos_y+1);
    val -= image_grayscale_get(img_prev, pos_x-1, pos_y-1);
    val -= image_grayscale_get(img_prev, pos_x-1, pos_y);
    val -= image_grayscale_get(img_prev, pos_x-1, pos_y+1);

    return val/6.0f;
}

float calc_dy(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y)
{
    float val = 0;
    // val += image_grayscale_get(img, pos_x-1, pos_y+1);
    // val += image_grayscale_get(img, pos_x, pos_y+1);
    // val += image_grayscale_get(img, pos_x+1, pos_y+1);
    // val -= image_grayscale_get(img, pos_x-1, pos_y-1);
    // val -= image_grayscale_get(img, pos_x, pos_y-1);
    // val -= image_grayscale_get(img, pos_x+1, pos_y-1);

    val += image_grayscale_get(img_prev, pos_x-1, pos_y+1);
    val += image_grayscale_get(img_prev, pos_x, pos_y+1);
    val += image_grayscale_get(img_prev, pos_x+1, pos_y+1);
    val -= image_grayscale_get(img_prev, pos_x-1, pos_y-1);
    val -= image_grayscale_get(img_prev, pos_x, pos_y-1);
    val -= image_grayscale_get(img_prev, pos_x+1, pos_y-1);

    return val/6.0f;
}

float calc_dt(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y)
{
    float val = 0;
    for (size_t i = pos_x-1; i <= pos_x+1; i++)
    {
        for (size_t j = pos_y-1; j <= pos_y+1; j++)
        {
            val += image_grayscale_get(img, i, j);
            val -= image_grayscale_get(img_prev, i, j);
        }
    }

    return val/9.0f;
}

void calc_optical_flow(image_grayscale_t *img, image_grayscale_t *img_prev, uint pos_x, uint pos_y, float *u_x, float *u_y)
{
    //MUST BE AN ODD VALUE
    #define WINDOW_SIZE 5
    float dx[WINDOW_SIZE*WINDOW_SIZE];
    float dy[WINDOW_SIZE*WINDOW_SIZE];
    float dt[WINDOW_SIZE*WINDOW_SIZE];

    //find dx dy and dt for pixels around the interest point
    uint counter = 0;
    for(uint j = pos_y-(WINDOW_SIZE/2); j <= pos_y+(WINDOW_SIZE/2); j++)
    {
        for(uint i = pos_x-(WINDOW_SIZE/2); i <= pos_x+(WINDOW_SIZE/2); i++)
        {
            dx[counter] = calc_dx(img, img_prev, i, j)/255.0f;
            dy[counter] = calc_dy(img, img_prev, i, j)/255.0f;
            dt[counter] = calc_dt(img, img_prev, i, j)/255.0f;

            counter++;
        }
    }

    //the Ax=b
    float A_mat[4] = {0, 0, 0, 0};
    float B_mat[2] = {0, 0};

    for (size_t i = 0; i < (WINDOW_SIZE*WINDOW_SIZE); i++)
    {
        A_mat[0] += dx[i]*dx[i];
        A_mat[1] += dx[i]*dy[i];
        A_mat[2] += dx[i]*dy[i];
        A_mat[3] += dy[i]*dy[i];

        B_mat[0] -= dx[i]*dt[i];
        B_mat[1] -= dy[i]*dt[i];
    }

    float det = A_mat[0]*A_mat[3]-A_mat[1]*A_mat[2];

    // can be a way to not take small flow into account
    // if(abs(det) < 0.001f)
    if(0) //not used now
    {
        *u_x = 0;
        *u_y = 0;
    }
    else
    {
        //calculate inverse of A
        float A_inv[4];

        A_inv[0] = (1.0f/det)*A_mat[3];
        A_inv[1] = -(1.0f/det)*A_mat[1];
        A_inv[2] = -(1.0f/det)*A_mat[2];
        A_inv[3] = (1.0f/det)*A_mat[0];

        //solve Au=B
        *u_x = A_inv[0]*B_mat[0]+A_inv[1]*B_mat[1];
        *u_y = A_inv[2]*B_mat[0]+A_inv[3]*B_mat[1];
    }
}
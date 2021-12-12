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

#include "../common/image.h"

#define MMAL_CAMERA_PREVIEW_PORT 0
#define MMAL_CAMERA_VIDEO_PORT 1
#define MMAL_CAMERA_CAPTURE_PORT 2

//will affect framerate, it seems that if framerate is higher than possible shutter speed, it will be automatically lowered
#define CAMERA_SHUTTER_SPEED 15000

//framerate above 30 only possible for some resolution, depends on the camera
//can also reduce the displayed portion of the camera on screen
#define CAMERA_FRAMERATE 30

//resolution needs to be smaller than the screen size
#define CAMERA_RESOLUTION_X 800
#define CAMERA_RESOLUTION_Y 600

#define CHECK_STATUS(status, msg) if (status != MMAL_SUCCESS) { fprintf(stderr, msg"\n\r");}

char *fbp;
uint32_t screen_size_x = 0;
uint32_t screen_size_y = 0;

static int cur_sec;

sem_t semaphore;

void framebuffer_init();
void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer);

void init_time_keeping();
float get_cur_time();


void edge_thinning(image_grayscale_t *img_in, image_grayscale_t *out);
void single_thresholding(image_grayscale_t *img_in, uint8_t thresh, uint8_t val_thresh);
void double_thresholding(image_grayscale_t *img_in, uint8_t thresh_low, uint8_t thresh_high, uint8_t val_weak, uint8_t val_high);
void canny_hysteresis(image_grayscale_t *img_in, uint8_t val_weak, uint8_t val_high);

void main(void){
    //sets up the framebuffer, will draw
    framebuffer_init();

    MMAL_STATUS_T status = MMAL_EINVAL;
    MMAL_COMPONENT_T *camera;
    MMAL_PORT_T *video_port;
    MMAL_ES_FORMAT_T *format;
    MMAL_POOL_T *pool;

    sem_init(&semaphore, 0, 0);

    bcm_host_init();

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    CHECK_STATUS(status, "failed to create decoder");

    status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_SHUTTER_SPEED, CAMERA_SHUTTER_SPEED);
    CHECK_STATUS(status, "failed to set shutter speed");

    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

    format = video_port->format;
    format->encoding = MMAL_ENCODING_RGB24;
    format->es->video.width = VCOS_ALIGN_UP(CAMERA_RESOLUTION_X, 32);
    format->es->video.height = VCOS_ALIGN_UP(CAMERA_RESOLUTION_Y, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = CAMERA_RESOLUTION_X;
    format->es->video.crop.height = CAMERA_RESOLUTION_Y;

    printf("Camera: resolution %dx%d\n\r", CAMERA_RESOLUTION_X, CAMERA_RESOLUTION_Y);

    status = mmal_port_format_commit(video_port);
    CHECK_STATUS(status, "failed to commit format");

    //second paramter of the second parameter is the denominator for the framerate
    MMAL_PARAMETER_FRAME_RATE_T framerate_param = {{MMAL_PARAMETER_VIDEO_FRAME_RATE, sizeof(framerate_param)}, {CAMERA_FRAMERATE, 0}};
    status = mmal_port_parameter_set(video_port, &framerate_param.hdr);
    CHECK_STATUS(status, "failed to set framerate");

    //two buffers seem a good compromise, more will cause some latency
    video_port->buffer_num = 2;
    pool = mmal_port_pool_create(video_port, video_port->buffer_num, video_port->buffer_size);

    video_port->userdata = (void *)pool->queue;

    status = mmal_component_enable(camera);
    CHECK_STATUS(status, "failed to enable camera");

    //will call the callback function everytime there is an image available
    status = mmal_port_enable(video_port, output_callback);
    CHECK_STATUS(status, "failed to enable video port");

    usleep(250);

    //necessary parameter to get the RGB data out of the video port
    status = mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_CAPTURE, 1);
    CHECK_STATUS(status, "failed to set parameter capture");

    //need to provide the buffers to the port
    int queue_length = mmal_queue_length(pool->queue);
    for(int i = 0; i < queue_length; i++){
        MMAL_BUFFER_HEADER_T *buffer = mmal_queue_get(pool->queue);
        if(buffer == NULL){
            printf("problem to get the buffer\n\r");
        }

        status = mmal_port_send_buffer(video_port, buffer);
        CHECK_STATUS(status, "could not send buffer");
    }

    MMAL_BUFFER_HEADER_T *buffer;
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
    image_grayscale_t img_edge_thin;

    while(1){

        start_time = get_cur_time();

        //wait until a buffer has been received
        sem_wait(&semaphore);

        buffer = mmal_queue_get(pool->queue);

        img.img = buffer->data;

        image_convert_to_grayscale(&img, &img_gray);

        sobel_edge_detect(&img_gray, &img_edge);

        //because sobel edge are thick, make them thin
        edge_thinning(&img_edge, &img_edge_thin);

        //apply some thresholding to keep only significan edges
        single_thresholding(&img_edge_thin, 48, 255);

        //produces results a bit more noisy, for a lot of processing
        // double_thresholding(&img_edge_thin, 48, 64, 64, 255);
        // canny_hysteresis(&img_edge_thin, 64, 255);

        // save to raw file
        // FILE *fp = fopen("img.raw", "wb");
        // for (size_t y = 0; y < img.height; y++)
        // {
        //     for (size_t x = 0; x < img.width; x++)
        //     {
        //         fputc(image_grayscale_get(&img_edge, x, y), fp);
        //     }
        // }

        // fclose(fp);


        //Send back the buffer to the port to be filled with an image again
        mmal_port_send_buffer(video_port, buffer);

        image_draw_grayscale(&img_edge_thin, fbp, screen_size_x);


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
        free(img_edge_thin.img);
    }

    //todo free the mmal and framebuffer ressources cleanly
}

void framebuffer_init(){
    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;

    int fb_d = open("/dev/fb0", O_RDWR);
    ioctl(fb_d, FBIOGET_FSCREENINFO, &finfo);
    ioctl(fb_d, FBIOGET_VSCREENINFO, &vinfo);

    printf("Framebuffer: resolution %dx%d with %dbpp\n\r", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
    screen_size_x = vinfo.xres;
    screen_size_y = vinfo.yres;

    fbp = (char*)mmap(0, screen_size_x*screen_size_y*4, PROT_READ | PROT_WRITE, MAP_SHARED, fb_d, 0);
    //draw a gradient background
    for(int i = 0; i < screen_size_y; i++){
        for(int j = 0; j < screen_size_x*4; j+=4){
            int idx = i*screen_size_x*4+j;
            fbp[idx] = (i*255)/screen_size_y;
            fbp[idx+1] = (j*255)/(screen_size_x*4);
            fbp[idx+2] = 128;
            fbp[idx+3] = 0;
        }
    }
}

void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer){
   struct MMAL_QUEUE_T *queue = (struct MMAL_QUEUE_T *)port->userdata;

   mmal_queue_put(queue, buffer);

   sem_post(&semaphore);

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

//will only keep the highest value pixel in all possible orientation
//in a 3x3 area around each pixels
void edge_thinning(image_grayscale_t *img_in, image_grayscale_t *out)
{
    out->width = img_in->width;
    out->height = img_in->height;
    out->img = malloc(out->width*out->height);

    memset(out->img, 0, out->width*out->height);

    for(int j = 1; j < img_in->height-1; j++)
    {
        for(int i = 1; i < img_in->width-1; i++)
        {
            uint8_t current_pix = image_grayscale_get(img_in, i, j);

            //see if the current pixel is the highest valued on in every directions

            uint8_t prev_pix_0 = image_grayscale_get(img_in, i-1, j);
            uint8_t next_pix_0 = image_grayscale_get(img_in, i+1, j);

            if(prev_pix_0 < current_pix && next_pix_0 < current_pix)
            {
                image_grayscale_set(out, i, j, current_pix);
                continue;
            }

            uint8_t prev_pix_45 = image_grayscale_get(img_in, i-1, j+1);
            uint8_t next_pix_45 = image_grayscale_get(img_in, i+1, j-1);

            if(prev_pix_45 < current_pix && next_pix_45 < current_pix)
            {
                image_grayscale_set(out, i, j, current_pix);
                continue;
            }

            uint8_t prev_pix_90 = image_grayscale_get(img_in, i, j-1);
            uint8_t next_pix_90 = image_grayscale_get(img_in, i, j+1);

            if(prev_pix_90 < current_pix && next_pix_90 < current_pix)
            {
                image_grayscale_set(out, i, j, current_pix);
                continue;
            }

            uint8_t prev_pix_135 = image_grayscale_get(img_in, i-1, j-1);
            uint8_t next_pix_135 = image_grayscale_get(img_in, i+1, j+1);

            if(prev_pix_135 < current_pix && next_pix_135 < current_pix)
            {
                image_grayscale_set(out, i, j, current_pix);
                continue;
            }
        }
    }
}

void single_thresholding(image_grayscale_t *img_in, uint8_t thresh, uint8_t val_thresh)
{
    for(int j = 1; j < img_in->height-1; j++)
    {
        for(int i = 1; i < img_in->width-1; i++)
        {
            if(image_grayscale_get(img_in, i, j) >= thresh)
            {
                image_grayscale_set(img_in, i, j, val_thresh);
            }
            else
            {
                image_grayscale_set(img_in, i, j, 0);
            }
        }
    }
}

//double thresholding can be used in canny
void double_thresholding(image_grayscale_t *img_in, uint8_t thresh_low, uint8_t thresh_high, uint8_t val_weak, uint8_t val_high){
    for(int j = 1; j < img_in->height-1; j++)
    {
        for(int i = 1; i < img_in->width-1; i++)
        {
            uint8_t current_pix = image_grayscale_get(img_in, i, j);

            if(current_pix >= thresh_low && current_pix < thresh_high)
            {
                image_grayscale_set(img_in, i, j, val_weak);
            }
            else if(current_pix >= thresh_high)
            {
                image_grayscale_set(img_in, i, j, val_high);
            }
            else
            {
                image_grayscale_set(img_in, i, j, 0);
            }
        }
    }

}

//will keep weak pixels if they are in the vicinity of a high-strong one in a 3x3 area around each pixel
void canny_hysteresis(image_grayscale_t *img_in, uint8_t val_weak, uint8_t val_high)
{
    for(int j = 1; j < img_in->height-1; j++)
    {
        for(int i = 1; i < img_in->width-1; i++)
        {
            uint8_t current_pix = image_grayscale_get(img_in, i, j);

            if(current_pix == val_weak)
            {
                uint8_t has_high_pixel = 0;

                for (int l = -1; l <= 1; l++)
                {
                    for (int k = -1; k <= 1; k++)
                    {
                        if(image_grayscale_get(img_in, i+k, j+l) == val_high){
                            has_high_pixel = 1;
                        }
                    }
                }

                if(has_high_pixel)
                {
                    image_grayscale_set(img_in, i, j, val_high);
                }
                else
                {
                    image_grayscale_set(img_in, i, j, 0);
                }
            }
        }
    }
}
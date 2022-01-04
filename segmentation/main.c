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
#define CAMERA_SHUTTER_SPEED 25000

//framerate above 30 only possible for some resolution, depends on the camera
//can also reduce the displayed portion of the camera on screen
#define MAX_CAMERA_FRAMERATE 30

//resolution needs to be smaller than the screen size
#define CAMERA_RESOLUTION_X 640
#define CAMERA_RESOLUTION_Y 480

char *fbp;
uint32_t screen_size_x = 0;
uint32_t screen_size_y = 0;

static int cur_sec;

extern sem_t semaphore_cam_buffer;

void init_time_keeping();
float get_cur_time();

typedef struct cluster_point_t{
    uint8_t r;
    uint8_t g;
    uint8_t b;

    //normalised positions
    float x_norm;
    float y_norm;

    //real position in the image
    uint x;
    uint y;

    uint cluster_id;
} cluster_point_t;

typedef struct centroid_t{
    float r;
    float g;
    float b;

    float x;
    float y;

    uint cluster_id;
} centroid_t;

float assign_points_to_centroid(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids);
float assign_points_to_centroid_xy(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids);
float calculate_new_centroids(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids);
float calculate_new_centroids_xy(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids);
void draw_rgb_clusters(image_rgb_t *img_draw, cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids);
void k_means_clustering(image_rgb_t *img_in, image_rgb_t *img_out, uint nb_cluster, uint nb_cycles);

void main(void){
    MMAL_PORT_T *video_port;
    MMAL_POOL_T *pool;
    MMAL_BUFFER_HEADER_T *buffer;

    //sets up the framebuffer, will draw
    framebuffer_init(&fbp, &screen_size_x, &screen_size_y);

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

    image_rgb_t img_cluster;

    while(1){
        start_time = get_cur_time();

        //wait until a buffer has been received
        sem_wait(&semaphore_cam_buffer);

        buffer = mmal_queue_get(pool->queue);

        img.img = buffer->data;

        const uint NB_CLUSTERS = 8;
        const uint NB_CYCLES = 8;
        k_means_clustering(&img, &img_cluster, NB_CLUSTERS, NB_CYCLES);

        // save to raw file
        // save_image_rgb_to_file(&img_cluster, "img_cluster.raw");

        //Send back the buffer to the port to be filled with an image again
        mmal_port_send_buffer(video_port, buffer);

        image_draw(&img_cluster, fbp, screen_size_x);


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

        free(img_cluster.img);
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

void k_means_clustering(image_rgb_t *img_in, image_rgb_t *img_out, uint nb_cluster, uint nb_cycles){
    img_out->width = img_in->width;
    img_out->height = img_in->height;

    //create a list of all "points" to cluster that is pixels
    uint number_points = img_in->width*img_in->height;
    cluster_point_t *lst_points = malloc(number_points*sizeof(cluster_point_t));

    //keep centroids between frames
    static centroid_t *lst_centroids = NULL;

    uint current_cluster_point_idx = 0;

    //initialise each point a value (rgb)
    for (size_t y = 0; y < img_in->height; y++)
    {
        for (size_t x = 0; x < img_in->width; x++)
        {
            cluster_point_t pt;
            pt.x = x;
            pt.y = y;
            pt.r = image_get(img_in, x, y, 0);
            pt.g = image_get(img_in, x, y, 1);
            pt.b = image_get(img_in, x, y, 2);

            //normalise the positions in a similar way to the colours
            pt.x_norm = (float)x/(float)img_in->width*255;
            pt.y_norm = (float)y/(float)img_in->height*255;

            pt.cluster_id = 0;

            lst_points[current_cluster_point_idx] = pt;

            current_cluster_point_idx++;
        }
    }

    //keep centroid values between images, since they shouldn't move that much between two frames
    if(lst_centroids == NULL){
        lst_centroids = malloc(nb_cluster*sizeof(centroid_t));

        //assign each centroid a random value
        for (size_t i = 0; i < nb_cluster; i++)
        {
            uint8_t r = 128;
            uint8_t g = 128;
            uint8_t b = 128;
            uint x = img_in->width/2;
            uint y = img_in->height/2;
            centroid_t pt;
            pt.r = r;
            pt.g = g;
            pt.b = b;
            pt.x = img_in->width/2;
            pt.y = img_in->height/2;
            lst_centroids[i] = pt;

            //uint8_t will take care of the modulo
            r+=15;
            g+=85;
            b+=155;
            x = (x+22)%img_in->width;
            y = (y+33)%img_in->height;
        }
    }

    float centroid_move = 0;

    for (size_t i = 0; i < nb_cycles; i++)
    {
        // assign_points_to_centroid(lst_points, number_points, lst_centroids, nb_cluster);
        assign_points_to_centroid_xy(lst_points, number_points, lst_centroids, nb_cluster);

        // centroid_move = calculate_new_centroids(lst_points, number_points, lst_centroids, nb_cluster);
        centroid_move = calculate_new_centroids_xy(lst_points, number_points, lst_centroids, nb_cluster);

        //if move not significant, stop
        if(centroid_move < 1.0f){
            break;
        }
    }

    img_out->img = malloc(img_in->width*img_in->height*3);

    draw_rgb_clusters(img_out, lst_points, number_points, lst_centroids, nb_cluster);

    free(lst_points);
}

float assign_points_to_centroid(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids){
    float avg_distance = 0;

    //for each point, find closest centroid and assign point to it
    for (size_t pt_idx = 0; pt_idx < nb_points; pt_idx++)
    {
        float min_dist_to_centroid = 10000000000; //safe max value
        uint selected_centroid = 0;

        for (size_t i = 0; i < nb_centroids; i++)
        {
            float dist_to_centroid = 0;

            dist_to_centroid += (lst_points[pt_idx].r-lst_centroids[i].r)*(lst_points[pt_idx].r-lst_centroids[i].r);
            dist_to_centroid += (lst_points[pt_idx].g-lst_centroids[i].g)*(lst_points[pt_idx].g-lst_centroids[i].g);
            dist_to_centroid += (lst_points[pt_idx].b-lst_centroids[i].b)*(lst_points[pt_idx].b-lst_centroids[i].b);

            dist_to_centroid = sqrt(dist_to_centroid);

            if(dist_to_centroid < min_dist_to_centroid)
            {
                min_dist_to_centroid = dist_to_centroid;
                selected_centroid = i;
            }
        }

        avg_distance += min_dist_to_centroid/nb_points;
        lst_points[pt_idx].cluster_id = selected_centroid;
    }

    return avg_distance;
}

float assign_points_to_centroid_xy(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids){
    float avg_distance = 0;

    //for each point, find closest centroid and assign point to it
    for (size_t pt_idx = 0; pt_idx < nb_points; pt_idx++)
    {
        float min_dist_to_centroid = 10000000000; //safe max value
        uint selected_centroid = 0;

        for (size_t i = 0; i < nb_centroids; i++)
        {
            float dist_to_centroid = 0;

            dist_to_centroid += (lst_points[pt_idx].r-lst_centroids[i].r)*(lst_points[pt_idx].r-lst_centroids[i].r);
            dist_to_centroid += (lst_points[pt_idx].g-lst_centroids[i].g)*(lst_points[pt_idx].g-lst_centroids[i].g);
            dist_to_centroid += (lst_points[pt_idx].b-lst_centroids[i].b)*(lst_points[pt_idx].b-lst_centroids[i].b);
            dist_to_centroid += (lst_points[pt_idx].x_norm-lst_centroids[i].x)*(lst_points[pt_idx].x_norm-lst_centroids[i].x);
            dist_to_centroid += (lst_points[pt_idx].y_norm-lst_centroids[i].y)*(lst_points[pt_idx].y_norm-lst_centroids[i].y);

            dist_to_centroid = sqrt(dist_to_centroid);

            if(dist_to_centroid < min_dist_to_centroid)
            {
                min_dist_to_centroid = dist_to_centroid;
                selected_centroid = i;
            }
        }
        avg_distance += min_dist_to_centroid/nb_points;
        lst_points[pt_idx].cluster_id = selected_centroid;
    }

    return avg_distance;
}


float calculate_new_centroids(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids){

    struct intermediate_centroid_t{
        uint total_r;
        uint total_g;
        uint total_b;
        uint nb_points_in_cluster;
    };

    //keep accumulation values in a list for each centroid
    struct intermediate_centroid_t *lst_intermediate_centroid = malloc(nb_centroids*sizeof(struct intermediate_centroid_t));
    float total_move = 0;

    for (size_t i = 0; i < nb_centroids; i++)
    {
        lst_intermediate_centroid[i].total_r = 0;
        lst_intermediate_centroid[i].total_g = 0;
        lst_intermediate_centroid[i].total_b = 0;
        lst_intermediate_centroid[i].nb_points_in_cluster = 0;
    }


    //for each point, add their value in their respective centroid
    for (size_t pt_idx = 0; pt_idx < nb_points; pt_idx++)
    {
        uint centroid_idx = lst_points[pt_idx].cluster_id;

        lst_intermediate_centroid[centroid_idx].total_r += lst_points[pt_idx].r;
        lst_intermediate_centroid[centroid_idx].total_g += lst_points[pt_idx].g;
        lst_intermediate_centroid[centroid_idx].total_b += lst_points[pt_idx].b;
        lst_intermediate_centroid[centroid_idx].nb_points_in_cluster++;
    }

    //for each centroid, update their value by calculating the mean
    for (size_t i = 0; i < nb_centroids; i++)
    {
        uint nb_points_in_cluster = lst_intermediate_centroid[i].nb_points_in_cluster;
        if(nb_points_in_cluster > 0)
        {
            float old_r = lst_centroids[i].r;
            float old_g = lst_centroids[i].g;
            float old_b = lst_centroids[i].b;
            lst_centroids[i].r = (float)lst_intermediate_centroid[i].total_r/(float)nb_points_in_cluster;
            lst_centroids[i].g = (float)lst_intermediate_centroid[i].total_g/(float)nb_points_in_cluster;
            lst_centroids[i].b = (float)lst_intermediate_centroid[i].total_b/(float)nb_points_in_cluster;

            total_move += (old_r-lst_centroids[i].r)*(old_r-lst_centroids[i].r);
            total_move += (old_g-lst_centroids[i].g)*(old_g-lst_centroids[i].g);
            total_move += (old_b-lst_centroids[i].b)*(old_b-lst_centroids[i].b);
        }
    }

    free(lst_intermediate_centroid);

    return total_move;
}

float calculate_new_centroids_xy(cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids){

    struct intermediate_centroid_t{
        uint total_r;
        uint total_g;
        uint total_b;
        float total_x;
        float total_y;
        uint nb_points_in_cluster;
    };

    //keep accumulation values in a list for each centroid
    struct intermediate_centroid_t *lst_intermediate_centroid = malloc(nb_centroids*sizeof(struct intermediate_centroid_t));
    float total_move = 0;

    for (size_t i = 0; i < nb_centroids; i++)
    {
        lst_intermediate_centroid[i].total_r = 0;
        lst_intermediate_centroid[i].total_g = 0;
        lst_intermediate_centroid[i].total_b = 0;
        lst_intermediate_centroid[i].total_x = 0;
        lst_intermediate_centroid[i].total_y = 0;
        lst_intermediate_centroid[i].nb_points_in_cluster = 0;
    }

    //for each point, add their value in their respective centroid
    for (size_t pt_idx = 0; pt_idx < nb_points; pt_idx++)
    {
        uint centroid_idx = lst_points[pt_idx].cluster_id;

        lst_intermediate_centroid[centroid_idx].total_r += lst_points[pt_idx].r;
        lst_intermediate_centroid[centroid_idx].total_g += lst_points[pt_idx].g;
        lst_intermediate_centroid[centroid_idx].total_b += lst_points[pt_idx].b;
        lst_intermediate_centroid[centroid_idx].total_x += lst_points[pt_idx].x_norm;
        lst_intermediate_centroid[centroid_idx].total_y += lst_points[pt_idx].y_norm;
        lst_intermediate_centroid[centroid_idx].nb_points_in_cluster++;
    }

    //for each centroid, update their value by calculating the mean
    for (size_t i = 0; i < nb_centroids; i++)
    {
        uint nb_points_in_cluster = lst_intermediate_centroid[i].nb_points_in_cluster;
        if(nb_points_in_cluster > 0)
        {
            float old_r = lst_centroids[i].r;
            float old_g = lst_centroids[i].g;
            float old_b = lst_centroids[i].b;
            float old_x = lst_centroids[i].x;
            float old_y = lst_centroids[i].y;


            lst_centroids[i].r = (float)lst_intermediate_centroid[i].total_r/(float)nb_points_in_cluster;
            lst_centroids[i].g = (float)lst_intermediate_centroid[i].total_g/(float)nb_points_in_cluster;
            lst_centroids[i].b = (float)lst_intermediate_centroid[i].total_b/(float)nb_points_in_cluster;
            lst_centroids[i].x = (float)lst_intermediate_centroid[i].total_x/(float)nb_points_in_cluster;
            lst_centroids[i].y = (float)lst_intermediate_centroid[i].total_y/(float)nb_points_in_cluster;

            total_move += (old_r-lst_centroids[i].r)*(old_r-lst_centroids[i].r);
            total_move += (old_g-lst_centroids[i].g)*(old_g-lst_centroids[i].g);
            total_move += (old_b-lst_centroids[i].b)*(old_b-lst_centroids[i].b);
            total_move += (old_x-lst_centroids[i].x)*(old_x-lst_centroids[i].x);
            total_move += (old_y-lst_centroids[i].y)*(old_y-lst_centroids[i].y);
        }
    }

    free(lst_intermediate_centroid);

    return total_move;
}

void draw_rgb_clusters(image_rgb_t *img_draw, cluster_point_t *lst_points, uint nb_points, centroid_t *lst_centroids, uint nb_centroids){

    //for each point retrieve the x,y values and the linked centroid rgb value
    for (size_t pt_idx = 0; pt_idx < nb_points; pt_idx++)
    {
        image_set(img_draw, lst_points[pt_idx].x, lst_points[pt_idx].y, 0, lst_centroids[lst_points[pt_idx].cluster_id].r);
        image_set(img_draw, lst_points[pt_idx].x, lst_points[pt_idx].y, 1, lst_centroids[lst_points[pt_idx].cluster_id].g);
        image_set(img_draw, lst_points[pt_idx].x, lst_points[pt_idx].y, 2, lst_centroids[lst_points[pt_idx].cluster_id].b);
    }
}
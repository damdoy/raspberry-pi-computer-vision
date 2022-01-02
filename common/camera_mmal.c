#include "camera_mmal.h"

void framebuffer_init(char **framebuffer_out, uint *screen_size_x, uint *screen_size_y){

    struct fb_var_screeninfo vinfo;
    struct fb_fix_screeninfo finfo;
    char *framebuffer;

    int fb_d = open("/dev/fb0", O_RDWR);
    ioctl(fb_d, FBIOGET_FSCREENINFO, &finfo);
    ioctl(fb_d, FBIOGET_VSCREENINFO, &vinfo);

    vinfo.bits_per_pixel = 24;
    printf("Framebuffer: setting depth to %dbpp\n\r", vinfo.bits_per_pixel);
    ioctl(fb_d, FBIOPUT_VSCREENINFO, &vinfo);

    printf("Framebuffer: resolution %dx%d with %dbpp\n\r", vinfo.xres, vinfo.yres, vinfo.bits_per_pixel);
    *screen_size_x = vinfo.xres;
    *screen_size_y = vinfo.yres;

    framebuffer = (char*)mmap(0, vinfo.xres*vinfo.yres*(vinfo.bits_per_pixel/8), PROT_READ | PROT_WRITE, MAP_SHARED, fb_d, 0);
    //draw a gradient background
    for(int i = 0; i < vinfo.yres; i++){
        for(int j = 0; j < vinfo.xres*3; j+=3){
            int idx = i*vinfo.xres*3+j;
            framebuffer[idx] = (i*255)/vinfo.yres;
            framebuffer[idx+1] = (j*255)/(vinfo.xres*3);
            framebuffer[idx+2] = 128;
        }
    }

    *framebuffer_out = framebuffer;
}

void camera_mmal_init(MMAL_PORT_T **video_port_out, MMAL_POOL_T **pool_out, uint camera_resolution_x, uint camera_resolution_y, uint shutter_speed, uint framerate, uint awb_mode){
    MMAL_STATUS_T status = MMAL_EINVAL;
    MMAL_COMPONENT_T *camera;
    MMAL_ES_FORMAT_T *format;
    MMAL_PORT_T *video_port;
    MMAL_POOL_T *pool;

    sem_init(&semaphore_cam_buffer, 0, 0);

    bcm_host_init();

    status = mmal_component_create(MMAL_COMPONENT_DEFAULT_CAMERA, &camera);
    CHECK_STATUS(status, "failed to create decoder");

    status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_SHUTTER_SPEED, shutter_speed);
    CHECK_STATUS(status, "failed to set shutter speed");

    /* possible values for the AWB
        MMAL_PARAM_AWBMODE_OFF,
        MMAL_PARAM_AWBMODE_AUTO,
        MMAL_PARAM_AWBMODE_SUNLIGHT,
        MMAL_PARAM_AWBMODE_CLOUDY,
        MMAL_PARAM_AWBMODE_SHADE,
        MMAL_PARAM_AWBMODE_TUNGSTEN,
        MMAL_PARAM_AWBMODE_FLUORESCENT,
        MMAL_PARAM_AWBMODE_INCANDESCENT,
        MMAL_PARAM_AWBMODE_FLASH,
        MMAL_PARAM_AWBMODE_HORIZON,
        MMAL_PARAM_AWBMODE_GREYWORLD
    */
    status = mmal_port_parameter_set_uint32(camera->control, MMAL_PARAMETER_AWB_MODE, awb_mode);
    CHECK_STATUS(status, "failed to set AWB");

    video_port = camera->output[MMAL_CAMERA_VIDEO_PORT];

    format = video_port->format;
    format->encoding = MMAL_ENCODING_RGB24;
    format->es->video.width = VCOS_ALIGN_UP(camera_resolution_x, 32);
    format->es->video.height = VCOS_ALIGN_UP(camera_resolution_y, 16);
    format->es->video.crop.x = 0;
    format->es->video.crop.y = 0;
    format->es->video.crop.width = camera_resolution_x;
    format->es->video.crop.height = camera_resolution_y;

    printf("Camera: resolution %dx%d\n\r", camera_resolution_x, camera_resolution_y);

    status = mmal_port_format_commit(video_port);
    CHECK_STATUS(status, "failed to commit format");

    //second paramter of the second parameter is the denominator for the framerate
    MMAL_PARAMETER_FRAME_RATE_T framerate_param = {{MMAL_PARAMETER_VIDEO_FRAME_RATE, sizeof(framerate_param)}, {framerate, 0}};
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

    status = mmal_port_parameter_set_boolean(video_port, MMAL_PARAMETER_ZERO_COPY, MMAL_TRUE);
    CHECK_STATUS(status, "failed to set zero copy");

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

    *video_port_out = video_port;
    *pool_out = pool;
}

void output_callback(MMAL_PORT_T *port, MMAL_BUFFER_HEADER_T *buffer){
    struct MMAL_QUEUE_T *queue = (struct MMAL_QUEUE_T *)port->userdata;

    mmal_queue_put(queue, buffer);

    sem_post(&semaphore_cam_buffer);
}
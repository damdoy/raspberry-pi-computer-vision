#include "image.h"

#include <math.h>
#include <string.h>

inline uint8_t image_get(image_rgb_t *img, int x, int y, int channel){
    return img->img[ (y*img->width+x)*3+channel ];
}

inline void image_set(image_rgb_t *img, int x, int y, int channel, uint8_t val){
    img->img[ (y*img->width+x)*3+channel ] = val;
}

inline uint8_t image_grayscale_get(image_grayscale_t *img, int x, int y){
    return img->img[ (y*img->width+x) ];
}

inline void image_grayscale_set(image_grayscale_t *img, int x, int y, uint8_t val){
    img->img[ (y*img->width+x) ] = val;
}

void image_draw(image_rgb_t *img, char *framebuffer, uint framebuffer_width){
    int offset_data = 0;
    for(int i = 0; i < img->height; i++){
        for(int j = 0; j < img->width*4; j+=4){
            int idx = i*framebuffer_width*4+j;
            //seem that R and B components are inverted
            framebuffer[idx] = img->img[offset_data+2];
            framebuffer[idx+1] = img->img[offset_data+1];
            framebuffer[idx+2] = img->img[offset_data+0];
            framebuffer[idx+3] = 0;
            offset_data += 3;
        }
    }
}

void image_draw_grayscale(image_grayscale_t *img, char *framebuffer, uint framebuffer_width){
    int offset_data = 0;
    for(int i = 0; i < img->height; i++){
        for(int j = 0; j < img->width*4; j+=4){
            int idx = i*framebuffer_width*4+j;
            framebuffer[idx] = img->img[offset_data];
            framebuffer[idx+1] = img->img[offset_data];
            framebuffer[idx+2] = img->img[offset_data];
            framebuffer[idx+3] = 0;
            offset_data++;
        }
    }
}

void image_convert_to_grayscale(image_rgb_t *source, image_grayscale_t *dest){
    dest->width = source->width;
    dest->height = source->height;

    dest->img = malloc(dest->width*dest->width);

    for (size_t i = 0; i < source->height; i++)
    {
        for (size_t j = 0; j < source->width; j++)
        {
            uint idx_source = (i*dest->width+j)*3;
            uint idx_dest = i*dest->width+j;
            //classical luminance calculation, without floats
            uint dst = source->img[idx_source]*76+source->img[idx_source+1]*150+source->img[idx_source+2]*29;
            dest->img[idx_dest] = (uint8_t)(dst/255);
        }
    }
}

//use a simple box blur filter
//use fact box filter is seperable to compute in two steps
void blur_grayscale_image(image_grayscale_t *image_source, image_grayscale_t *image_dest, uint kernel_size){
    image_dest->height = image_source->height;
    image_dest->width = image_source->width;
    image_dest->img = malloc(image_source->width*image_source->height);

    uint16_t *img_temp = malloc(image_source->width*image_source->height*sizeof(uint16_t));

    float total_kernel_size = (2*kernel_size+1)*(2*kernel_size+1);

    //use separable property of box filter
    for (int y = 0; y < image_source->height; y++)
    {
        for (int x = 0; x < image_source->width; x++)
        {
            uint16_t total = 0.0f;
            for (int ki = -((int)kernel_size); ki <= ((int)kernel_size); ki++)
            {
                //ignore everything out of the image
                if(x+ki >= 0 && x+ki < image_source->width){
                    total += image_grayscale_get(image_source, x+ki, y);
                }
            }
            img_temp[y*image_source->width+x] = total;
        }
    }

    for (int y = 0; y < image_source->height; y++)
    {
        for (int x = 0; x < image_source->width; x++)
        {
            uint16_t total = 0.0f;
            for (int kj = -((int)kernel_size); kj <= ((int)kernel_size); kj++)
            {
                if(y+kj >= 0 && y+kj < image_source->height){
                    total += img_temp[(y+kj)*image_source->width+x];
                }
            }
            image_grayscale_set(image_dest, x, y, total/total_kernel_size);
        }
    }

    free(img_temp);

}

//divides by two on each axis the resolution of the image
void downscale_gray_image(image_grayscale_t *image_source, image_grayscale_t *image_dest){
    image_dest->height = image_source->height/2;
    image_dest->width = image_source->width/2;
    image_dest->img = malloc(image_dest->width*image_dest->height);
    // memcpy(image_dest->img, image_source->img, image_source->width*image_source->height);

    for (int y = 0; y < image_dest->height; y++)
    {
        for (int x = 0; x < image_dest->width; x++)
        {
            image_grayscale_set(image_dest, x, y, image_grayscale_get(image_source, x*2, y*2));
        }
    }
}

//bresenham algorithm for circle
void draw_circle(uint x, uint y, uint radius, image_rgb_t *img){
    int curx = x;
    int cury = y-radius;
    int offsetx = curx-x;
    int offsety = cury-y;

    int color[] = {255, 0, 0};

    image_set(img, curx, cury, 0, color[0]);
    image_set(img, curx, cury, 1, color[1]);
    image_set(img, curx, cury, 2, color[2]);

    image_set(img, curx, cury+radius*2, 0, color[0]);
    image_set(img, curx, cury+radius*2, 1, color[1]);
    image_set(img, curx, cury+radius*2, 2, color[2]);
    image_set(img, curx+radius, cury+radius, 0, color[0]);
    image_set(img, curx+radius, cury+radius, 1, color[1]);
    image_set(img, curx+radius, cury+radius, 2, color[2]);
    image_set(img, curx-radius, cury+radius, 0, color[0]);
    image_set(img, curx-radius, cury+radius, 1, color[1]);
    image_set(img, curx-radius, cury+radius, 2, color[2]);

    while(offsetx+offsety < 0){
        curx = curx+1;
        offsetx = curx-x;
        offsety = cury-y;

        if( offsetx*offsetx+offsety*offsety>(radius+0.5)*(radius+0.5)){
            cury = cury+1;
        }

        offsetx = curx-x;
        offsety = cury-y;

        image_set(img, curx, cury, 0, color[0]);
        image_set(img, curx, cury, 1, color[1]);
        image_set(img, curx, cury, 2, color[2]);

        image_set(img, x+offsetx, y-offsety, 0, color[0]);
        image_set(img, x+offsetx, y-offsety, 1, color[1]);
        image_set(img, x+offsetx, y-offsety, 2, color[2]);

        image_set(img, x-offsetx, y+offsety, 0, color[0]);
        image_set(img, x-offsetx, y+offsety, 1, color[1]);
        image_set(img, x-offsetx, y+offsety, 2, color[2]);
        image_set(img, x-offsetx, y-offsety, 0, color[0]);
        image_set(img, x-offsetx, y-offsety, 1, color[1]);
        image_set(img, x-offsetx, y-offsety, 2, color[2]);

        image_set(img, x+offsety, y+offsetx, 0, color[0]);
        image_set(img, x+offsety, y+offsetx, 1, color[1]);
        image_set(img, x+offsety, y+offsetx, 2, color[2]);
        image_set(img, x-offsety, y-offsetx, 0, color[0]);
        image_set(img, x-offsety, y-offsetx, 1, color[1]);
        image_set(img, x-offsety, y-offsetx, 2, color[2]);
        image_set(img, x-offsety, y+offsetx, 0, color[0]);
        image_set(img, x-offsety, y+offsetx, 1, color[1]);
        image_set(img, x-offsety, y+offsetx, 2, color[2]);
        image_set(img, x+offsety, y-offsetx, 0, color[0]);
        image_set(img, x+offsety, y-offsetx, 1, color[1]);
        image_set(img, x+offsety, y-offsetx, 2, color[2]);
    }
}

void draw_line(uint x1, uint y1, uint x2, uint y2, image_rgb_t *img, uint colour[3])
{
    uint smallest_y = y1;
    uint biggest_y = y2;
    int direction_y = 1;
    if(y2 < y1)
    {
        smallest_y = y2;
        biggest_y = y1;
        direction_y = -1;
    }

    int direction_x = 1;
    if(x2 < x1)
    {
        direction_x = -1;
    }

    float slope = 0;
    //avoid a /0 by having a very steep slope
    if(abs(x2-x1) == 0)
    {
        slope = 10000*direction_y;
    }
    else
    {
        slope = ((float)abs(y2-y1))/((float)abs(x2-x1));
        slope = slope*direction_x*direction_y;
    }

    uint cur_y = y1;

    for(uint i = x1; i != x2+direction_x; i+= direction_x)
    {
        float y = slope*((int)i-(int)x1)+y1;
        if(fabs(y-cur_y) < 1.0f)
        {
            image_set(img, i, cur_y, 0, colour[0]);
            image_set(img, i, cur_y, 1, colour[1]);
            image_set(img, i, cur_y, 2, colour[2]);
        }

        float next_y = slope*(((int)i+direction_x)-(int)x1)+(int)y1;
        uint cond;
        if(direction_y == 1)
        {
            cond = next_y > cur_y;
        }
        else
        {
            cond = next_y < cur_y;
        }

        while(cond && cur_y >= smallest_y && cur_y <= biggest_y)
        {
            image_set(img, i, cur_y, 0, colour[0]);
            image_set(img, i, cur_y, 1, colour[1]);
            image_set(img, i, cur_y, 2, colour[2]);
            cur_y += direction_y;
            if(direction_y == 1)
            {
                cond = next_y > cur_y;
            }
            else
            {
                cond = next_y < cur_y;
            }
        }
    }
}

//very naive implementation, very slow
void sobel_edge_detect_naive(image_grayscale_t *img_in, image_grayscale_t *out){
    out->width = img_in->width;
    out->height = img_in->height;
    out->img = malloc(out->width*out->height);

    memset(out->img, 0, out->width*out->height);

    //never get to the frame pixels as they would cause seg fault
    for(int j = 1; j < out->height-1; j++)
    {
        for(int i = 1; i < out->width-1; i++)
        {
            int conv_calc_x = 0;

            conv_calc_x += image_grayscale_get(img_in, i-1, j-1);
            conv_calc_x += 2*image_grayscale_get(img_in, i-1, j);
            conv_calc_x += image_grayscale_get(img_in, i-1, j+1);

            conv_calc_x -= image_grayscale_get(img_in, i+1, j-1);
            conv_calc_x -= 2*image_grayscale_get(img_in, i+1, j);
            conv_calc_x -= image_grayscale_get(img_in, i+1, j+1);

            int conv_calc_y = 0;

            conv_calc_y += image_grayscale_get(img_in, i-1, j-1);
            conv_calc_y += 2*image_grayscale_get(img_in, i, j-1);
            conv_calc_y += image_grayscale_get(img_in, i+1, j-1);

            conv_calc_y -= image_grayscale_get(img_in, i-1, j+1);
            conv_calc_y -= 2*image_grayscale_get(img_in, i, j+1);
            conv_calc_y -= image_grayscale_get(img_in, i+1, j+1);

            int value = sqrt(conv_calc_x*conv_calc_x+conv_calc_y*conv_calc_y);

            //clamp
            if(value > 255)
            {
                value = 255;
            }

            image_grayscale_set(out, i, j, value);
        }
    }
}

//use separation of the sobel kernel, saves a bit of processing, but not much
void sobel_edge_detect(image_grayscale_t *img_in, image_grayscale_t *out){
    out->width = img_in->width;
    out->height = img_in->height;
    out->img = malloc(out->width*out->height);

    memset(out->img, 0, out->width*out->height);

    //temporary matrices
    int *temp_mat_x = malloc(out->width*out->height*sizeof(int));
    int *temp_mat_y = malloc(out->width*out->height*sizeof(int));

    //calculate first slice of the separated matrix
    for(int j = 1; j < out->height-1; j++)
    {
        for(int i = 1; i < out->width-1; i++)
        {
            uint idx = j*out->width+i;

            temp_mat_x[idx] += image_grayscale_get(img_in, i-1, j);
            // temp_mat_x[idx] += 0*image_grayscale_get(img_in, i, j);
            temp_mat_x[idx] -= image_grayscale_get(img_in, i+1, j);

            temp_mat_y[idx] += image_grayscale_get(img_in, i, j-1);
            temp_mat_y[idx] += 2*image_grayscale_get(img_in, i, j);
            temp_mat_y[idx] += image_grayscale_get(img_in, i, j+1);

        }
    }

    for(int j = 2; j < out->height-2; j++)
    {
        for(int i = 2; i < out->width-2; i++)
        {
            uint idx = j*out->width+i;
            uint idx_y_prev = (j-1)*out->width+i;
            uint idx_y_next = (j+1)*out->width+i;

            int conv_calc_x = 0;
            int conv_calc_y = 0;

            conv_calc_x += temp_mat_x[idx-1];
            conv_calc_x += 2*temp_mat_x[idx];
            conv_calc_x += temp_mat_x[idx+1];

            conv_calc_y += temp_mat_y[idx_y_prev];
            // conv_calc_y += 0*temp_mat_y[idx];
            conv_calc_y -= temp_mat_y[idx_y_next];

            int value = sqrt(conv_calc_x*conv_calc_x+conv_calc_y*conv_calc_y);
            // int value = sqrt(conv_calc_y*conv_calc_y);

            if(value > 255)
            {
                value = 255;
            }

            image_grayscale_set(out, i, j, value);
        }
    }

    free(temp_mat_x);
    free(temp_mat_y);
}
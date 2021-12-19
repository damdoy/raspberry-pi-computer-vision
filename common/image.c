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

inline uint32_t image_grayscale32_get(image_grayscale32_t *img, int x, int y){
    return img->img[ (y*img->width+x) ];
}

inline void image_grayscale32_increment_pix(image_grayscale32_t *img, int x, int y){
    // img->img[ (y*img->width+x) ]+=0x70000000;
    img->img[ (y*img->width+x) ]++;
}

inline void image_grayscale32_set(image_grayscale32_t *img, int x, int y, uint32_t val){
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

void image_draw_grayscale32(image_grayscale32_t *img, char *framebuffer, uint framebuffer_width){
    int offset_data = 0;
    for(int i = 0; i < img->height; i++){
        for(int j = 0; j < img->width*4; j+=4){
            int idx = i*framebuffer_width*4+j;
            framebuffer[idx] = (img->img[offset_data]&0xFF000000)>>24;
            framebuffer[idx+1] = (img->img[offset_data]&0xFF000000)>>24;
            framebuffer[idx+2] = (img->img[offset_data]&0xFF000000)>>24;
            framebuffer[idx+3] = 0;
            offset_data++;
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
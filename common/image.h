#ifndef IMAGE_H
#define IMAGE_H

#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>

typedef struct image_rgb_t{
    int width;
    int height;
    uint8_t *img;
} image_rgb_t;

typedef struct image_grayscale_t{
    int width;
    int height;
    uint8_t *img;
} image_grayscale_t;

void image_draw(image_rgb_t *img, char *framebuffer, uint framebuffer_width);
void image_draw_grayscale(image_grayscale_t *img, char *framebuffer, uint framebuffer_width);
uint8_t image_get(image_rgb_t *img, int x, int y, int channel);
void image_set(image_rgb_t *img, int x, int y, int channel, uint8_t val);
uint8_t image_grayscale_get(image_grayscale_t *img, int x, int y);
void image_grayscale_set(image_grayscale_t *img, int x, int y, uint8_t val);
void image_convert_to_grayscale(image_rgb_t *source, image_grayscale_t *dest);
void blur_grayscale_image(image_grayscale_t *image_source, image_grayscale_t *image_dest, uint kernel_size);
void downscale_gray_image(image_grayscale_t *image_source, image_grayscale_t *image_dest);

void draw_circle(uint x, uint y, uint radius, image_rgb_t *img);
void draw_line(uint x1, uint y1, uint x2, uint y2, image_rgb_t *img, uint colour[3]);

void sobel_edge_detect(image_grayscale_t *img_in, image_grayscale_t *out);

#endif
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

typedef struct image_grayscale32_t{
    int width;
    int height;
    uint32_t *img;
} image_grayscale32_t;

void image_draw(image_rgb_t *img, char *framebuffer, uint framebuffer_width);
uint8_t image_get(image_rgb_t *img, int x, int y, int channel);
void image_set(image_rgb_t *img, int x, int y, int channel, uint8_t val);

void save_image_rgb_to_file(image_rgb_t *img, char *filename);
void save_image_grayscale_to_file(image_grayscale_t *img, char *filename);

void image_draw_grayscale(image_grayscale_t *img, char *framebuffer, uint framebuffer_width);
uint8_t image_grayscale_get(image_grayscale_t *img, int x, int y);
void image_grayscale_set(image_grayscale_t *img, int x, int y, uint8_t val);
void image_convert_to_grayscale(image_rgb_t *source, image_grayscale_t *dest);
void blur_grayscale_image(image_grayscale_t *image_source, image_grayscale_t *image_dest, uint kernel_size);
void downscale_gray_image(image_grayscale_t *image_source, image_grayscale_t *image_dest);

void image_draw_grayscale32(image_grayscale32_t *img, char *framebuffer, uint framebuffer_width);
uint32_t image_grayscale32_get(image_grayscale32_t *img, int x, int y);
void image_grayscale32_increment_pix(image_grayscale32_t *img, int x, int y);
void image_grayscale32_set(image_grayscale32_t *img, int x, int y, uint32_t val);

void draw_circle(uint x, uint y, uint radius, image_rgb_t *img);
void draw_line(uint x1, uint y1, uint x2, uint y2, image_rgb_t *img, uint colour[3]);

#endif
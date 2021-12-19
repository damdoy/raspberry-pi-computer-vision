#ifndef EDGE_DETECT_H
#define EDGE_DETECT_H

#include "image.h"

void sobel_edge_detect(image_grayscale_t *img_in, image_grayscale_t *out);

void edge_thinning(image_grayscale_t *img_in, image_grayscale_t *out);
void single_thresholding(image_grayscale_t *img_in, uint8_t thresh, uint8_t val_thresh);
void double_thresholding(image_grayscale_t *img_in, uint8_t thresh_low, uint8_t thresh_high, uint8_t val_weak, uint8_t val_high);
void canny_hysteresis(image_grayscale_t *img_in, uint8_t val_weak, uint8_t val_high);

void get_canny(image_grayscale_t *img_in, image_grayscale_t *out);
#endif
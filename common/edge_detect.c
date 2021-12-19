#include "edge_detect.h"

#include <string.h>
#include <math.h>

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

void get_canny(image_grayscale_t *img_in, image_grayscale_t *out)
{
    image_grayscale_t temp;

    sobel_edge_detect(img_in, &temp);

    //because sobel edge are thick, make them thin
    edge_thinning(&temp, out);

    //apply some thresholding to keep only significan edges
    single_thresholding(out, 48, 255);

    free(temp.img);
}
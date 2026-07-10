/**
 * Unified postprocess header - supports both 1-class (int8) and 25-class (float) models
 * Auto-selects based on model type at runtime
 */

#ifndef POSTPROCESS_H_
#define POSTPROCESS_H_

#include <stdint.h>
#include <vector>
#include <iomanip>
#include <sstream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "common.h"

// 1-class model parameters (light_det*)
#define OBJ_NAME_MAX_SIZE_1 16
#define OBJ_NUMB_MAX_SIZE_1 64
#define OBJ_CLASS_NUM_1 1
#define NMS_THRESH_1 0.45
#define BOX_THRESH_1 0.45
#define PROP_BOX_SIZE_1 (5 + OBJ_CLASS_NUM_1)

// 25-class model parameters (aug_enhanced*)
#define OBJ_NAME_MAX_SIZE_25 32
#define OBJ_NUMB_MAX_SIZE_25 128
#define OBJ_CLASS_NUM_25 25
#define NMS_THRESH_25 0.45
#define BOX_THRESH_25 0.45
#define PROP_BOX_SIZE_25 (5 + OBJ_CLASS_NUM_25)

// 1-class postprocess (int8_t quantized output)
int post_process_1(int8_t *input0, int8_t *input1, int8_t *input2, int model_in_h, int model_in_w,
                   float conf_threshold, float nms_threshold, BOX_RECT pads, float scale_w, float scale_h,
                   std::vector<int32_t> &qnt_zps, std::vector<float> &qnt_scales,
                   DetectResultsGroup *group);
void deinitPostProcess_1();

// 25-class postprocess (float32 output)
int post_process_25_f(float *input0, float *input1, float *input2, int model_in_h, int model_in_w,
                      float conf_threshold, float nms_threshold, BOX_RECT pads, float scale_w, float scale_h,
                      DetectResultsGroup *group);
int post_process_25(int8_t *input0, int8_t *input1, int8_t *input2, int model_in_h, int model_in_w,
                    float conf_threshold, float nms_threshold, BOX_RECT pads, float scale_w, float scale_h,
                    std::vector<int32_t> &qnt_zps, std::vector<float> &qnt_scales,
                    DetectResultsGroup *group);
void deinitPostProcess_25();

// Common drawing utilities (shared between both models)
int draw_image_detect(cv::Mat &cur_img, std::vector<DetectionBox> &results, int cur_frame_id);
void show_draw_results(DetectResultsGroup &results_group);

#endif // POSTPROCESS_H_

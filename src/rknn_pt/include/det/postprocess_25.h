#ifndef POSTPROCESS_25_H_
#define POSTPROCESS_25_H_

#include <stdint.h>
#include <vector>

#include <iomanip>
#include <sstream>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "common.h"

#define OBJ_NAME_MAX_SIZE 16
#define OBJ_NUMB_MAX_SIZE 64
#define OBJ_CLASS_NUM 25
#define NMS_THRESH 0.45
#define BOX_THRESH 0.45
#define PROP_BOX_SIZE (5 + OBJ_CLASS_NUM)


int post_process_25(int8_t *input0, int8_t *input1, int8_t *input2, int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold, BOX_RECT pads, float scale_w, float scale_h,
                 std::vector<int32_t> &qnt_zps, std::vector<float> &qnt_scales,
                 DetectResultsGroup *group);

void deinitPostProcess_25();

int draw_image_detect_25(cv::Mat &cur_img, std::vector<DetectionBox> &results, int cur_frame_id);

void show_draw_results_25(DetectResultsGroup &results_group);


#endif //POSTPROCESS_25_H_

/**
 * 25类检测后处理实现 - float输入版本 (want_float=1)
 * 包含 sigmoid，直接处理 float32 NPU 输出
 */

#include "postprocess.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <iostream>
#include <set>
#include <vector>

static const char *labels[OBJ_CLASS_NUM_25] = {
    "0","1","2","3","4",
    "5","6","7","8","9",
    "electrodrill","headphones","keyboard","mobile_phone","monitor",
    "mouse","multimeter","oscillograph","pliers","printer",
    "screwdriver","soldering_iron","speaker","tape_measure","wrench",
};

const int anchor0[6] = {10, 13, 16, 30, 33, 23};
const int anchor1[6] = {30, 61, 62, 45, 59, 119};
const int anchor2[6] = {116, 90, 156, 198, 373, 326};

static const int cnum = 25;
static cv::Scalar_<int> randColor[cnum];
static bool init_colors = false;

inline static int clamp(float val, int min, int max) { return val > min ? (val < max ? val : max) : min; }

static float sigmoid(float x) { return 1.0f / (1.0f + expf(-x)); }

static float CalculateOverlap(float xmin0, float ymin0, float xmax0, float ymax0, float xmin1, float ymin1, float xmax1,
                              float ymax1)
{
  float w = fmax(0.f, fmin(xmax0, xmax1) - fmax(xmin0, xmin1) + 1.0);
  float h = fmax(0.f, fmin(ymax0, ymax1) - fmax(ymin0, ymin1) + 1.0);
  float i = w * h;
  float u = (xmax0 - xmin0 + 1.0) * (ymax0 - ymin0 + 1.0) + (xmax1 - xmin1 + 1.0) * (ymax1 - ymin1 + 1.0) - i;
  return u <= 0.f ? 0.f : (i / u);
}

static int nms(int validCount, std::vector<float> &outputLocations, std::vector<int> classIds, std::vector<int> &order,
               int filterId, float threshold)
{
  for (int i = 0; i < validCount; ++i) {
    if (order[i] == -1) continue;
    int n = order[i];
    if (classIds[n] != filterId) continue;
    for (int j = i + 1; j < validCount; ++j) {
      int m = order[j];
      if (m == -1 || classIds[m] != filterId) continue;
      float xmin0 = outputLocations[n * 4 + 0];
      float ymin0 = outputLocations[n * 4 + 1];
      float xmax0 = outputLocations[n * 4 + 0] + outputLocations[n * 4 + 2];
      float ymax0 = outputLocations[n * 4 + 1] + outputLocations[n * 4 + 3];
      float xmin1 = outputLocations[m * 4 + 0];
      float ymin1 = outputLocations[m * 4 + 1];
      float xmax1 = outputLocations[m * 4 + 0] + outputLocations[m * 4 + 2];
      float ymax1 = outputLocations[m * 4 + 1] + outputLocations[m * 4 + 3];
      float iou = CalculateOverlap(xmin0, ymin0, xmax0, ymax0, xmin1, ymin1, xmax1, ymax1);
      if (iou > threshold) order[j] = -1;
    }
  }
  return 0;
}

static int quick_sort_indice_inverse(std::vector<float> &input, int left, int right, std::vector<int> &indices)
{
  float key;
  int key_index;
  int low = left;
  int high = right;
  if (left < right) {
    key_index = indices[left];
    key = input[left];
    while (low < high) {
      while (low < high && input[high] <= key) high--;
      input[low] = input[high];
      indices[low] = indices[high];
      while (low < high && input[low] >= key) low++;
      input[high] = input[low];
      indices[high] = indices[low];
    }
    input[low] = key;
    indices[low] = key_index;
    quick_sort_indice_inverse(input, left, low - 1, indices);
    quick_sort_indice_inverse(input, low + 1, right, indices);
  }
  return low;
}

static int process_float(float *input, int *anchor, int grid_h, int grid_w, int height, int width, int stride,
                         std::vector<float> &boxes, std::vector<float> &objProbs, std::vector<int> &classId,
                         float threshold)
{
  int validCount = 0;
  int grid_len = grid_h * grid_w;
  int prop_size = PROP_BOX_SIZE_25;  // 30

  for (int a = 0; a < 3; a++) {
    for (int i = 0; i < grid_h; i++) {
      for (int j = 0; j < grid_w; j++) {
        int offset = (prop_size * a) * grid_len + i * grid_w + j;
        float *in_ptr = input + offset;

        float box_x_logit  = in_ptr[0 * grid_len];
        float box_y_logit  = in_ptr[1 * grid_len];
        float box_w_logit  = in_ptr[2 * grid_len];
        float box_h_logit  = in_ptr[3 * grid_len];
        float box_conf_logit = in_ptr[4 * grid_len];

        float box_conf = sigmoid(box_conf_logit);
        if (box_conf < threshold) continue;

        float box_x = sigmoid(box_x_logit) * 2.0f - 0.5f;
        float box_y = sigmoid(box_y_logit) * 2.0f - 0.5f;
        float box_w = sigmoid(box_w_logit) * 2.0f;
        float box_h = sigmoid(box_h_logit) * 2.0f;

        box_x = (box_x + j) * (float)stride;
        box_y = (box_y + i) * (float)stride;
        box_w = box_w * box_w * (float)anchor[a * 2];
        box_h = box_h * box_h * (float)anchor[a * 2 + 1];
        box_x -= (box_w / 2.0f);
        box_y -= (box_h / 2.0f);

        float maxClassProb = sigmoid(in_ptr[5 * grid_len]);
        int maxClassId = 0;
        for (int k = 1; k < OBJ_CLASS_NUM_25; ++k) {
          float prob = sigmoid(in_ptr[(5 + k) * grid_len]);
          if (prob > maxClassProb) {
            maxClassId = k;
            maxClassProb = prob;
          }
        }
        // 手机(13)、耳机(11)、音响(22)使用更高阈值
        float class_threshold = threshold;
        if (maxClassId == 11 || maxClassId == 13 || maxClassId == 22) {
          class_threshold = 0.80f;
        }
        if (maxClassProb > class_threshold) {
          objProbs.push_back(maxClassProb * box_conf);
          classId.push_back(maxClassId);
          validCount++;
          boxes.push_back(box_x);
          boxes.push_back(box_y);
          boxes.push_back(box_w);
          boxes.push_back(box_h);
        }
      }
    }
  }
  return validCount;
}

int post_process_25_f(float *input0, float *input1, float *input2, int model_in_h, int model_in_w,
                       float conf_threshold, float nms_threshold, BOX_RECT pads, float scale_w, float scale_h,
                       DetectResultsGroup *group)
{
  group->dets.clear();
  std::vector<float> filterBoxes, objProbs;
  std::vector<int> classId;

  int stride0 = 8, stride1 = 16, stride2 = 32;
  int grid_h0 = model_in_h / stride0, grid_w0 = model_in_w / stride0;
  int grid_h1 = model_in_h / stride1, grid_w1 = model_in_w / stride1;
  int grid_h2 = model_in_h / stride2, grid_w2 = model_in_w / stride2;

  int validCount0 = process_float(input0, (int *)anchor0, grid_h0, grid_w0, model_in_h, model_in_w, stride0,
                                  filterBoxes, objProbs, classId, conf_threshold);
  int validCount1 = process_float(input1, (int *)anchor1, grid_h1, grid_w1, model_in_h, model_in_w, stride1,
                                  filterBoxes, objProbs, classId, conf_threshold);
  int validCount2 = process_float(input2, (int *)anchor2, grid_h2, grid_w2, model_in_h, model_in_w, stride2,
                                  filterBoxes, objProbs, classId, conf_threshold);

  int validCount = validCount0 + validCount1 + validCount2;
  if (validCount <= 0) return 0;

  std::vector<int> indexArray;
  for (int i = 0; i < validCount; ++i) indexArray.push_back(i);
  quick_sort_indice_inverse(objProbs, 0, validCount - 1, indexArray);

  std::set<int> class_set(std::begin(classId), std::end(classId));
  for (auto c : class_set) nms(validCount, filterBoxes, classId, indexArray, c, nms_threshold);

  for (int i = 0; i < validCount; ++i) {
    if (indexArray[i] == -1) continue;
    int n = indexArray[i];
    float x1 = filterBoxes[n * 4 + 0] - pads.left;
    float y1 = filterBoxes[n * 4 + 1] - pads.top;
    float x2 = x1 + filterBoxes[n * 4 + 2];
    float y2 = y1 + filterBoxes[n * 4 + 3];
    int id = classId[n];
    DetectionBox new_box;
    new_box.class_id = id;
    new_box.box = cv::Rect_<int>(
        (int)(clamp(x1, 0, model_in_w) / scale_w),
        (int)(clamp(y1, 0, model_in_h) / scale_h),
        (int)(clamp(x2, 0, model_in_w) / scale_w) - (int)(clamp(x1, 0, model_in_w) / scale_w),
        (int)(clamp(y2, 0, model_in_h) / scale_h) - (int)(clamp(y1, 0, model_in_h) / scale_h));
    new_box.score = objProbs[i];
    new_box.det_name = labels[id];
    group->dets.push_back(new_box);
  }
  return 0;
}

// Stub for backward compat
int post_process_25(int8_t *input0, int8_t *input1, int8_t *input2, int model_in_h, int model_in_w,
                 float conf_threshold, float nms_threshold, BOX_RECT pads, float scale_w, float scale_h,
                 std::vector<int32_t> &qnt_zps, std::vector<float> &qnt_scales, DetectResultsGroup *group)
{
  printf("ERROR: post_process_25(int8_t) not supported after float update!\n");
  return -1;
}

int draw_image_detect(cv::Mat &cur_img, std::vector<DetectionBox> &results, int cur_frame_id) {
  char text[256];
  for (const auto& res : results) {
    sprintf(text, "%s %.2f", res.det_name.c_str(), res.score);
    cv::rectangle(cur_img, res.box, cv::Scalar(256, 0, 0, 256), 3);
    cv::putText(cur_img, text, cv::Point(res.box.x, res.box.y + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
  }
  return 0;
}

static void initializeRandColors() {
  cv::RNG rng(0xFFFFFFFF);
  for (int i = 0; i < cnum; i++) rng.fill(randColor[i], cv::RNG::UNIFORM, 0, 256);
  init_colors = true;
}

void show_draw_results(DetectResultsGroup &results_group) {
  if (!init_colors) initializeRandColors();
  char text[256];
  for (const auto& res : results_group.dets) {
    int color_idx = 0;
    for (int i = 0; res.det_name.c_str()[i] != '\0'; i++)
      color_idx = (color_idx * 31 + res.det_name.c_str()[i]) % cnum;
    sprintf(text, "%s %.2f", res.det_name.c_str(), res.score);
    cv::rectangle(results_group.cur_img, res.box, randColor[color_idx], 2, 8, 0);
    cv::putText(results_group.cur_img, text, cv::Point(res.box.x, res.box.y + 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255, 255, 255));
    int cx = res.box.x + res.box.width / 2;
    int cy = res.box.y + res.box.height / 2;
    cv::circle(results_group.cur_img, cv::Point(cx, cy), 10, cv::Scalar(0, 0, 255), -1, cv::LINE_AA);
  }
}

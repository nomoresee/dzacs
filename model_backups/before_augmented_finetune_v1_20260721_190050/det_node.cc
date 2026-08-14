#include <stdio.h>
#include <memory>
#include <chrono>
#include <sys/time.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/String.h>
// #include <sensor_msgs/

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "rkpt.hpp"
#include "rknnPool.hpp"
#include "rknn_pt/SwitchModel.h"
#include <cstdlib>
#include <cstdint>
#include <algorithm>
#include <string>
#include <map>
#include <mutex>
#include <set>
#include <sstream>

namespace
{

cv::Mat latest_ros_frame;
uint64_t latest_frame_sequence = 0;
std::mutex latest_frame_mtx;

const std::map<std::string, std::string> model_map = {
    {"light_det2", "/home/duzhong/dzacs/src/rknn_pt/model/light_det2.rknn"},
    {"aug_enhanced_v5s_opt2_v3", "/home/duzhong/dzacs/src/rknn_pt/model/aug_enhanced_v5s_opt2_v3.rknn"}
};
std::string pending_model_name;
std::string pending_model_path;
bool need_swap = false;
std::mutex swap_mtx;

std_msgs::Int32MultiArray make_offset_message(int offset_x = 0,
                                               int offset_y = 0,
                                               int detected = 0)
{
  std_msgs::Int32MultiArray msg;
  msg.layout.dim.resize(1);
  msg.layout.dim[0].size = 3;
  msg.layout.dim[0].stride = 3;
  msg.layout.dim[0].label = "target offset [x, y, detected]";
  msg.data = {offset_x, offset_y, detected};
  return msg;
}

std_msgs::String make_visual_class_message(
    const std::vector<DetectionBox> &detections)
{
  std_msgs::String msg;

  if (detections.empty())
    return msg;

  // 按置信度从高到低排序，只保留前2个
  std::vector<const DetectionBox*> sorted;
  for (const auto &det : detections)
    sorted.push_back(&det);
  std::sort(sorted.begin(), sorted.end(),
      [](const DetectionBox *a, const DetectionBox *b) {
        return a->score > b->score;
      });
  if (sorted.size() > 2)
    sorted.resize(2);

  std::vector<int> digits;
  int item_id = -1;
  for (const auto *det : sorted)
  {
    if (det->class_id >= 0 && det->class_id <= 9)
    {
      digits.push_back(det->class_id);
    }
    else if (det->class_id >= 10)
    {
      item_id = det->class_id;
    }
  }

  std::sort(digits.begin(), digits.end());
  digits.erase(std::unique(digits.begin(), digits.end()), digits.end());

  if (digits.size() >= 2)
  {
    msg.data = std::to_string(digits[0]) + "_" + std::to_string(digits[1]);
  }
  else if (!digits.empty())
  {
    msg.data = std::to_string(digits[0]);
  }
  else if (item_id >= 0)
  {
    msg.data = std::to_string(item_id);
  }

  return msg;
}

std_msgs::String make_detected_objects_message(
    const std::vector<DetectionBox> &detections)
{
  if (detections.empty())
  {
    std_msgs::String msg;
    return msg;
  }

  // 按置信度从高到低排序，只保留前2个
  std::vector<const DetectionBox*> sorted;
  for (const auto &detection : detections)
    sorted.push_back(&detection);
  std::sort(sorted.begin(), sorted.end(),
      [](const DetectionBox *a, const DetectionBox *b) {
        return a->score > b->score;
      });
  if (sorted.size() > 2)
    sorted.resize(2);

  std::set<std::string> unique_classes;
  for (const auto *det : sorted)
    unique_classes.insert(det->det_name);

  std::ostringstream joined;
  for (auto it = unique_classes.begin(); it != unique_classes.end(); ++it)
  {
    if (it != unique_classes.begin())
      joined << ',';
    joined << *it;
  }

  std_msgs::String msg;
  msg.data = joined.str();
  return msg;
}

bool resolve_model(const std::string &request, std::string &model_name,
                   std::string &model_path, std::string &error)
{
  auto it = model_map.find(request);
  if (it != model_map.end())
  {
    model_name = it->first;
    model_path = it->second;
    return true;
  }

  for (const auto &entry : model_map)
  {
    if (request == entry.second || request == entry.first + ".rknn")
    {
      model_name = entry.first;
      model_path = entry.second;
      return true;
    }
  }

  error = "Unknown model '" + request +
          "'. Use light_det2 or aug_enhanced_v5s_opt2_v3";
  return false;
}

bool queue_model_switch(const std::string &request, std::string &message)
{
  std::string model_name;
  std::string model_path;
  if (!resolve_model(request, model_name, model_path, message))
    return false;

  {
    std::lock_guard<std::mutex> lock(swap_mtx);
    pending_model_name = model_name;
    pending_model_path = model_path;
    need_swap = true;
  }

  message = "Queued model switch to " + model_name;
  return true;
}

void handle_switch_model_topic(const std_msgs::String::ConstPtr &msg)
{
  std::string message;
  if (!queue_model_switch(msg->data, message))
  {
    ROS_WARN("%s", message.c_str());
    return;
  }
  ROS_INFO("%s", message.c_str());
}

// Keep a service interface for compatibility. Topic switching is the primary
// interface and is advertised as /switch_model.
bool handle_switch_model(rknn_pt::SwitchModel::Request &req,
                         rknn_pt::SwitchModel::Response &res)
{
  res.success = queue_model_switch(req.model_name, res.message);
  if (res.success)
    ROS_INFO("%s", res.message.c_str());
  else
    ROS_WARN("%s", res.message.c_str());
  return true;
}

} // namespace

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    cv::Mat new_frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
    std::lock_guard<std::mutex> lock(latest_frame_mtx);
    latest_ros_frame = std::move(new_frame);
    ++latest_frame_sequence;
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("No image convert to : %s", e.what());
    return;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "det_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");

  ros::Publisher det_pub = nh.advertise<std_msgs::Int32MultiArray>("offset_center", 1);
  ros::Publisher objects_pub =
      nh.advertise<std_msgs::String>("/detected_objects", 1);
  ros::Publisher visual_class_pub =
      nh.advertise<std_msgs::String>("/dzatornode2", 1);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

  ros::Subscriber switch_sub =
      nh.subscribe<std_msgs::String>("/switch_model", 1, handle_switch_model_topic);
  ros::ServiceServer switch_srv =
      nh.advertiseService("/switch_model_srv", handle_switch_model);
  ros::Publisher current_model_pub =
      nh.advertise<std_msgs::String>("/current_model", 1, true);

  // 图像和模型切换回调独立运行。主线程中的推理、绘框或 imshow 卡顿时，
  // 相机回调仍会持续覆盖最新帧，不再被 ros::spinOnce 的调用时机限制。
  ros::AsyncSpinner callback_spinner(2);
  callback_spinner.start();

  ROS_INFO("Model switch topic ready at /switch_model (std_msgs/String)");
  ROS_INFO("Model switch service retained at /switch_model_srv");
  ROS_INFO("Available models: light_det2, aug_enhanced_v5s_opt2_v3");

  std::string initial_model_request = "aug_enhanced_v5s_opt2_v3";
  private_nh.param<std::string>("initial_model", initial_model_request,
                                initial_model_request);
  std::string active_model_name;
  std::string active_model_path;
  std::string model_error;
  if (!resolve_model(initial_model_request, active_model_name,
                     active_model_path, model_error))
  {
    ROS_FATAL("%s", model_error.c_str());
    return -1;
  }
  // std::string vedio_name = "/home/duzhong/Desktop/8.mp4";

  int threadNum = 3;  // RK3588 三个 NPU 核各使用一个上下文，避免第四线程争抢 core 0
  private_nh.param("thread_num", threadNum, threadNum);

  const char *display_env = std::getenv("DISPLAY");
  const bool has_display = display_env != nullptr && display_env[0] != '\0';
  bool draw = has_display;
  private_nh.param("draw", draw, draw);
  if (draw && !has_display)
  {
    ROS_WARN("~draw is true but DISPLAY is empty; disabling OpenCV window");
    draw = false;
  }

  rknnPool<RkPt, cv::Mat, DetectResultsGroup> detectPool(active_model_path, threadNum);
  if (detectPool.init() != 0)
  {
    ROS_FATAL("rknnPool init failed for %s", active_model_path.c_str());
    return -1;
  }

  std_msgs::String current_model_msg;
  current_model_msg.data = active_model_name;
  current_model_pub.publish(current_model_msg);
  if (active_model_name == "light_det2")
  {
    std_msgs::String empty_objects;
    objects_pub.publish(empty_objects);
  }
  else
  {
    det_pub.publish(make_offset_message());
  }
  ROS_INFO("Active model: %s (%s)", active_model_name.c_str(),
           active_model_path.c_str());

  cv::VideoCapture capture;
  // capture.open(vedio_name);

  // if (!capture.isOpened())
  // {
  //   printf("Error: Could not open video or camera\n");
  //   return -1;
  // }

  // int width = capture.get(cv::CAP_PROP_FRAME_WIDTH);
  // int height = capture.get(cv::CAP_PROP_FRAME_HEIGHT);

  // printf("capture width: %d height: %d\n", width, height);

  struct timeval time;
  gettimeofday(&time, nullptr);
  auto startTime = time.tv_sec * 1000 + time.tv_usec / 1000;

  int frames = 0;
  int last_published_frame_id = -1;
  uint64_t last_submitted_frame_sequence = 0;

  ros::Rate loop_rate(15);
  while (ros::ok())
  {
    // Copy the request while holding the mutex, then perform the blocking
    // model load without holding it.
    std::string next_model_name;
    std::string next_model_path;
    {
      std::lock_guard<std::mutex> lock(swap_mtx);
      if (need_swap && !pending_model_path.empty())
      {
        next_model_name.swap(pending_model_name);
        next_model_path.swap(pending_model_path);
        need_swap = false;
      }
    }

    if (!next_model_path.empty())
    {
      if (next_model_path == active_model_path)
      {
        ROS_INFO("Model %s is already active", active_model_name.c_str());
      }
      else
      {
        // Stop consumers of the old model before the blocking reinit. In
        // particular, offset_center valid=0 stops gimbal tracking/firing.
        if (active_model_name == "light_det2")
        {
          det_pub.publish(make_offset_message());
        }
        else
        {
          std_msgs::String empty_objects;
          objects_pub.publish(empty_objects);
        }

        ROS_INFO("Switching model: %s -> %s", active_model_name.c_str(),
                 next_model_name.c_str());
        int ret = detectPool.reinit(next_model_path);
        if (ret != 0)
        {
          ROS_ERROR("Model switch to %s failed; continuing with %s",
                    next_model_name.c_str(), active_model_name.c_str());
        }
        else
        {
          active_model_name = next_model_name;
          active_model_path = next_model_path;
          frames = 0;
          last_published_frame_id = -1;
          last_submitted_frame_sequence = 0;
          current_model_msg.data = active_model_name;
          current_model_pub.publish(current_model_msg);
          ROS_INFO("Model switch complete. Active model: %s",
                   active_model_name.c_str());
        }
      }
    }

    DetectResultsGroup results_group;
    const int result_status = detectPool.getLatestReady(results_group);
    if (result_status < 0)
    {
      ROS_ERROR_THROTTLE(1.0, "Inference task failed; continuing with latest frame");
    }

    // 推理队列最多保留 threadNum 个正在处理的任务。队列满时，相机回调只更新
    // latest_ros_frame，不再把过期帧继续塞入队列；一有空位便提交当时的最新帧。
    if (detectPool.pending() < static_cast<size_t>(threadNum))
    {
      cv::Mat frame_to_submit;
      uint64_t frame_sequence_to_submit = 0;
      {
        std::lock_guard<std::mutex> lock(latest_frame_mtx);
        if (!latest_ros_frame.empty() &&
            latest_frame_sequence != last_submitted_frame_sequence)
        {
          frame_to_submit = latest_ros_frame;
          frame_sequence_to_submit = latest_frame_sequence;
        }
      }

      if (!frame_to_submit.empty())
      {
        if (detectPool.put(frame_to_submit, frames) != 0)
        {
          ROS_ERROR("Failed to submit latest image for inference");
        }
        else
        {
          last_submitted_frame_sequence = frame_sequence_to_submit;
          ++frames;
        }
      }
    }

    const bool result_is_fresh =
        result_status == 0 &&
        results_group.cur_frame_id > last_published_frame_id;
    if (result_is_fresh && !results_group.cur_img.empty())
    {
      last_published_frame_id = results_group.cur_frame_id;
      if (active_model_name == "light_det2")
      {
        std_msgs::Int32MultiArray offset_msg = make_offset_message();
        if (!results_group.dets.empty())
        {
          // offset_center carries one target, so use the highest-confidence
          // light detection when multiple boxes are present.
          auto best = std::max_element(
              results_group.dets.begin(), results_group.dets.end(),
              [](const DetectionBox &lhs, const DetectionBox &rhs) {
                return lhs.score < rhs.score;
              });
          int center_x = best->box.x + best->box.width / 2;
          int center_y = best->box.y + best->box.height / 2;
          int offset_center_x = center_x - results_group.cur_img.cols / 2;
          int offset_center_y = center_y - results_group.cur_img.rows / 2;
          offset_msg = make_offset_message(offset_center_x, offset_center_y, 1);

          static auto last_light_print = std::chrono::steady_clock::now();
          auto now = std::chrono::steady_clock::now();
          if (std::chrono::duration_cast<std::chrono::seconds>(
                  now - last_light_print).count() >= 2)
          {
            printf("[LIGHT] offset=(%d,%d) score=%.2f\n",
                   offset_center_x, offset_center_y, best->score);
            last_light_print = now;
          }
        }
        det_pub.publish(offset_msg);
      }
      else
      {
        std_msgs::String objects_msg =
            make_detected_objects_message(results_group.dets);
        objects_pub.publish(objects_msg);

        std_msgs::String class_msg = make_visual_class_message(results_group.dets);
        if (!class_msg.data.empty())
        {
          visual_class_pub.publish(class_msg);
        }

        static auto last_objects_print = std::chrono::steady_clock::now();
        auto now = std::chrono::steady_clock::now();
        if (!objects_msg.data.empty() &&
            std::chrono::duration_cast<std::chrono::seconds>(
                now - last_objects_print).count() >= 2)
        {
          printf("[OBJECTS] %s\n", objects_msg.data.c_str());
          last_objects_print = now;
        }
      }

      if (draw)
      {
        show_draw_results(results_group);
        cv::imshow("src", results_group.cur_img);
        if (cv::waitKey(1) == 'q') // 延时1毫秒,按q键退出
          return 0;
      }
    }

    loop_rate.sleep();
  }

  while (true)
  {
    DetectResultsGroup results_group;
    if (detectPool.get(results_group) != 0)
      break;
  }

  capture.release();
  cv::destroyAllWindows();

  gettimeofday(&time, nullptr);
  auto endTime = time.tv_sec * 1000 + time.tv_usec / 1000;

  printf("Average:\t %f fps/s\n", float(frames) / float(endTime - startTime) * 1000.0);

  return 0;
}

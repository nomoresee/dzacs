#include <stdio.h>
#include <memory>
#include <sys/time.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
// #include <std_msgs/Bool.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "rkpt_25.hpp"
#include "rknnPool.hpp"

cv::Mat ros_frame;
// bool g_enable_detect = true;  // 全局变量，用于动态切换（暂时禁用）

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
  try
  {
    ros_frame = cv_bridge::toCvShare(msg, "bgr8")->image.clone();
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("No image convert to : %s", e.what());
    return;
  }
}

// 动态切换检测使能的回调函数（暂时禁用）
// void enableCallback(const std_msgs::Bool::ConstPtr &msg)
// {
//   g_enable_detect = msg->data;
//   ROS_INFO("det_node_25 detection %s", g_enable_detect ? "ENABLED" : "DISABLED");
// }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "det_node_25");
  ros::NodeHandle nh;

  ros::Publisher det_pub = nh.advertise<std_msgs::Int32MultiArray>("det_25_results", 1);

  // 订阅动态切换话题（暂时禁用）
  // ros::Subscriber enable_sub = nh.subscribe("detection_enable", 10, enableCallback);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

  std::string model_name = "/home/duzhong/dzacs/src/rknn_pt/model/bestjiance.rknn";

  int draw = 1;
  int threadNum = 4;

  // 从参数服务器读取是否启用检测（暂时禁用）
  // ros::NodeHandle private_nh("~");
  // private_nh.param<bool>("enable", g_enable_detect, true);

  rknnPool<RkPt25, cv::Mat, DetectResultsGroup> detectPool(model_name, threadNum);
  if (detectPool.init() != 0)
  {
    printf("rknnPool init fail!\n");
    return -1;
  }

  struct timeval time;
  gettimeofday(&time, nullptr);
  auto startTime = time.tv_sec * 1000 + time.tv_usec / 1000;

  int frames = 0;
  auto beforeTime = startTime;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    std_msgs::Int32MultiArray msg;

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 3;
    msg.layout.dim[0].stride = 3;
    msg.layout.dim[0].label = "detection results";

    msg.data = {0, 0, 0};

    DetectResultsGroup results_group;

    if (!ros_frame.empty())
    {
      int width = ros_frame.cols;
      int height = ros_frame.rows;

      if (detectPool.put(ros_frame, frames) != 0)
      {
        printf("put original images failed or work done!\n");
        break;
      }

      if (frames >= threadNum && detectPool.get(results_group) != 0)
      {
        printf("frames > 3 but get processed images failed! or work done\n");
        break;
      }

      if (!results_group.cur_img.empty())
      {
        // 用于存储 class 10-25 的检测结果
        std::vector<const DetectionBox*> high_class_dets;
        // 用于存储所有数字检测结果
        std::vector<const DetectionBox*> all_dets;
        
        for (const auto &res : results_group.dets)
        {
          int class_id = atoi(res.det_name.c_str());
          all_dets.push_back(&res);
          
          // 过滤 class 10-25
          if (class_id >= 10 && class_id <= 25)
          {
            high_class_dets.push_back(&res);
          }
        }
        
        int final_number = 0;
        int final_x = 0, final_y = 0;
        
        // 策略1: 如果有 class 10-25，选取置信度最高的
        if (!high_class_dets.empty())
        {
          const DetectionBox* best = high_class_dets[0];
          float best_score = best->score;
          for (const auto det : high_class_dets)
          {
            if (det->score > best_score)
            {
              best_score = det->score;
              best = det;
            }
          }
          final_number = atoi(best->det_name.c_str());
          final_x = best->box.x + best->box.width / 2 - width / 2;
          final_y = best->box.y + best->box.height / 2 - height / 2;
        }
        // 策略2: 如果没有 class 10-25，取置信度前两个数字拼接
        else if (all_dets.size() >= 2)
        {
          // 按置信度排序，取前两个
          std::vector<const DetectionBox*> top2;
          for (const auto det : all_dets)
          {
            if (top2.size() < 2)
            {
              top2.push_back(det);
              // 保持按置信度从高到低
              for (int i = top2.size() - 1; i > 0 && top2[i]->score > top2[i-1]->score; i--)
              {
                std::swap(top2[i], top2[i-1]);
              }
            }
            else if (det->score > top2.back()->score)
            {
              top2[1] = det;
              if (top2[1]->score > top2[0]->score)
              {
                std::swap(top2[0], top2[1]);
              }
            }
          }
          
          // 按x坐标排序确定十位和个位
          const DetectionBox* first = top2[0]->box.x < top2[1]->box.x ? top2[0] : top2[1];
          const DetectionBox* second = top2[0]->box.x < top2[1]->box.x ? top2[1] : top2[0];
          
          int first_num = atoi(first->det_name.c_str());
          int second_num = atoi(second->det_name.c_str());
          final_number = first_num * 10 + second_num;
          
          // 中心坐标取两个数字的中间点
          final_x = (first->box.x + first->box.width/2 + second->box.x + second->box.width/2) / 2 - width / 2;
          final_y = (first->box.y + first->box.height/2 + second->box.y + second->box.height/2) / 2 - height / 2;
        }
        // 策略3: 只有一个数字
        else if (all_dets.size() == 1)
        {
          final_number = atoi(all_dets[0]->det_name.c_str());
          final_x = all_dets[0]->box.x + all_dets[0]->box.width / 2 - width / 2;
          final_y = all_dets[0]->box.y + all_dets[0]->box.height / 2 - height / 2;
        }
        
        // 动态切换检测使能暂时禁用，始终发布结果
        msg.layout.dim[0].size = 3;
        msg.layout.dim[0].stride = 3;
        msg.layout.dim[0].label = "detection results";
        msg.data[0] = final_x;
        msg.data[1] = final_y;
        msg.data[2] = final_number;

        ROS_INFO("25-class result: %d, x: %d, y: %d", final_number, final_x, final_y);
        
        det_pub.publish(msg);

        if (draw)
        {
          show_draw_results_25(results_group);
          cv::imshow("det_25", results_group.cur_img);
          if (cv::waitKey(1) == 'q')
            return 0;
        }
      }

      frames++;
    }

    ros::spinOnce();
    loop_rate.sleep();
  }

  while (true)
  {
    DetectResultsGroup results_group;
    if (detectPool.get(results_group) != 0)
      break;
    frames++;
  }

  cv::destroyAllWindows();

  gettimeofday(&time, nullptr);
  auto endTime = time.tv_sec * 1000 + time.tv_usec / 1000;

  printf("25-class Average:\t %f fps/s\n", float(frames) / float(endTime - startTime) * 1000.0);

  return 0;
}

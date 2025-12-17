#include <stdio.h>
#include <memory>
#include <sys/time.h>
#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
// #include <sensor_msgs/

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include "rkpt.hpp"
#include "rknnPool.hpp"

cv::Mat ros_frame;
constexpr int CENTERING_THRESHOLD = 10;  // 允许的水平居中阈值(像素)

// 添加目标检测统计变量
static int total_detection_count = 0;      // 总检测次数
static int target_detected_count = 0;      // 检测到目标的次数
static int target_centered_count = 0;      // 目标居中的次数
static ros::Time last_detection_print_time;


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

int main(int argc, char **argv)
{
  ros::init(argc, argv, "det_node");
  ros::NodeHandle nh;
  last_detection_print_time = ros::Time::now();

  ros::Publisher det_pub = nh.advertise<std_msgs::Int32MultiArray>("offset_center", 1);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber image_sub = it.subscribe("/usb_cam/image_raw", 1, imageCallback);

  // 使用官方路径，但需要根据实际用户名修改
  std::string model_name = "/home/duzhong/dzacs/src/rknn_pt/model/light_det.rknn";
  
  // 检查模型文件是否存在
  FILE* file = fopen(model_name.c_str(), "rb");
  if (file == NULL) {
    printf("错误: 模型文件不存在: %s\n", model_name.c_str());
    printf("请检查以下路径是否存在:\n");
    printf("1. %s\n", model_name.c_str());
    printf("2. 请确认您的用户名是否为 'duzhong'\n");
    printf("3. 如果不是，请修改源代码中的路径\n");
    return -1;
  }
  fclose(file);
  printf("模型文件路径: %s\n", model_name.c_str());
  // std::string vedio_name = "/home/duzhong/Desktop/8.mp4";

  int draw = 1;
  int threadNum = 4;

  rknnPool<RkPt, cv::Mat, DetectResultsGroup> detectPool(model_name, threadNum);
  if (detectPool.init() != 0)
  {
    printf("rknnPool init fail!\n");
    return -1;
  }

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
  auto beforeTime = startTime;

  ros::Rate loop_rate(30);
  while (ros::ok())
  {
    std_msgs::Int32MultiArray msg;

    // 初始化 layout
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 2;       // size 为数据元素数量
    msg.layout.dim[0].stride = 2;     // stride 通常与 size 相同，除非数据结构很复杂
    msg.layout.dim[0].label = "light center coordinate";

    // 初始化 data
    msg.data = {0, 0, 0};  // 直接初始化为三个值
    // cv::Mat img;
    DetectResultsGroup results_group;
    // if (capture.read(img) == false)
    // {
    //   printf("read original images failed or work done!\n");
    //   break;
    // }
    if (!ros_frame.empty())
    {
      int width = ros_frame.cols;
      int height = ros_frame.rows;

      // printf("capture width: %d height: %d\n", width, height);

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
        for (const auto &res : results_group.dets)
        {

                  // 增加总检测次数
        total_detection_count++;

          int center_x = res.box.x + res.box.width / 2;
          int center_y = res.box.y + res.box.height / 2;

          int offset_center_x = center_x - width / 2;
          int offset_center_y = center_y - height / 2;

          // 增加检测到目标的次数
          target_detected_count++;
          
          // 检查目标是否居中
          if (abs(offset_center_x) <= CENTERING_THRESHOLD && abs(offset_center_y) <= 10) {
            target_centered_count++;
          }


          // printf("name: %s, x: %d, y: %d, width: %d, height: %d\n", res.det_name.c_str(), offset_center_x, offset_center_y, res.box.width, res.box.height);

          msg.layout.dim[0].size = 2;
          msg.layout.dim[0].stride = 2;
          msg.layout.dim[0].label = "light center coordinate";
          msg.data[0] = offset_center_x;
          msg.data[1] = offset_center_y;
          msg.data[2] = 1;

          // ROS_INFO("published: [%d %d]", msg.data[0], msg.data[1]);
        }
        det_pub.publish(msg);

        // 每5秒打印一次目标检测统计信息
        ros::Time current_time = ros::Time::now();
        if ((current_time - last_detection_print_time).toSec() >= 5.0) {
          ROS_INFO("目标检测统计 - 总检测次数: %d, 检测到目标次数: %d, 目标居中次数: %d, 居中率: %.2f%%", 
                   total_detection_count, 
                   target_detected_count, 
                   target_centered_count,
                   target_detected_count > 0 ? (double)target_centered_count / target_detected_count * 100.0 : 0.0);
          last_detection_print_time = current_time;
        }

        
        if (draw)
        {
          show_draw_results(results_group);
          cv::imshow("src", results_group.cur_img);
          if (cv::waitKey(1) == 'q') // 延时1毫秒,按q键退出
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

  capture.release();
  cv::destroyAllWindows();

  gettimeofday(&time, nullptr);
  auto endTime = time.tv_sec * 1000 + time.tv_usec / 1000;

  printf("Average:\t %f fps/s\n", float(frames) / float(endTime - startTime) * 1000.0);

  return 0;
}


#include "dzactuator.h"
#include "Quaternion_Solution.h"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <fstream>
#include <sstream>
#include <ros/package.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/convert.h>
#include <tf2/utils.h>

sensor_msgs::Imu Mpu6050;

namespace
{
// -------------------------- Yaw 运动靶调参区 --------------------------
// 视觉像素偏差到云台位置增量的换算系数。数值越小，Yaw 修正越激进；
// 应先调好下方的超前时间再调该值。正负方向必须与实际 Yaw 机械方向一致。
constexpr double kYawPixelsPerMotorPosition = 2.7;

// 降低速度估计的旧值权重，让云台更快跟上靶子速度变化；
// 预瞄位置仍保留独立滤波，防止速度噪声直接变成大幅指令。
constexpr double kYawVelocityFilterOldWeight = 0.72;

// 偏差滤波改为更接近当前帧，避免画面已明显离开中心后才修正。
constexpr double kYawOffsetFilterOldWeight = 0.52;

// Yaw 超前位置再做一层低通，避免速度估计的小幅波动直接变成云台
// 位置指令。低于速度死区时按静止目标处理。
constexpr double kYawLeadFilterOldWeight = 0.68;
constexpr double kYawLeadVelocityDeadband = 15.0;  // 云台位置单位/秒

// 补偿相机、推理、ROS 通信和云台执行总延迟的超前时间。此次只增加
// 0.01 秒，减少打在运动方向后方，同时避免过度预瞄。
constexpr double kYawLeadTimeSec = 0.21;

// 估计速度及其超前位置的安全上限，防止偶发错误检测框造成过大的 Yaw 指令。
constexpr double kYawMaxEstimatedVelocity = 700.0;  // 云台位置单位/秒
constexpr double kYawMaxLeadPosition = 147.0;       // 云台位置单位

// 提高单帧可撤销的位置误差，减少大偏差时连续多帧追赶。
constexpr int kYawTrackingCommandMaxStep = 130;
constexpr double kYawTrackingMaxMotorSpeed = 9000.0;

// 位置 PID 在持续跟踪时误差会变小，单靠误差给速度容易掉到靶子后面。
// 用已经低通滤波的目标速度做小幅前馈，不改变指令方向和位置。
constexpr double kYawVelocityFeedforwardGain = 2.3;

// 速度超过该阈值后发生正负号改变，视为导轨端点折返。
constexpr double kYawReversalVelocityThreshold = 25.0;  // 云台位置单位/秒

// 导轨端点会先减速/短暂停顿，此时清除旧方向预瞄并只按当前视觉
// 中心对准。连续两帧接近静止才进入端点保持，但第一帧已经停用旧预瞄。
// 卡帧后的长间隔样本不用于判断端点，避免把掉帧误判为停顿。
// 端点判断使用“云台反馈 + 当前原始视觉偏差”得到的目标位置，
// 不再等待慢速偏差滤波收敛。静止阈值留出少量检测框抖动余量。
constexpr double kYawEndpointStationaryVelocityThreshold = 120.0;
constexpr double kYawEndpointResumeVelocityThreshold = 180.0;
constexpr double kYawEndpointDecelerationMinVelocity = 150.0;
constexpr double kYawEndpointDecelerationRatio = 0.55;
constexpr double kYawEndpointMaxSampleIntervalSec = 0.12;
constexpr int kYawEndpointHoldConfirmFrames = 1;

// 端点清除预瞄后云台仍有机械惯性。先留出短暂刹停时间；如果靶子仍
// 停在端点，还必须连续多帧对准才允许发射，避免过冲途中穿过中心就开火。
constexpr double kYawEndpointFireSettleTimeSec = 0.05;
constexpr int kYawEndpointCenteredFramesBeforeFire = 1;
constexpr double kYawEndpointRecenterMinSpeed = 4500.0;
constexpr double kYawEndpointMinimumHoldTimeSec = 0.10;

// 开启后运动段绝不发射，每次只在确认靶子静止于端点并对准后发射一次。
constexpr bool kFireOnlyAtYawEndpoint = true;

// ---------------------- 定时单步双发模式 ----------------------
// 开启后完全绕过实时跟踪/端点开火：每 0.3 秒采样一次最新 offset，
// 按比例一步下发云台位置，短暂等待机械到位后非阻塞发射两次。
constexpr bool kUsePeriodicStepAndBurstMode = true;
constexpr double kPeriodicAimIntervalSec = 0.1;
constexpr double kPeriodicAimSettleTimeSec = 0.08;
constexpr double kPeriodicBurstShotIntervalSec = 0.08;
constexpr int kPeriodicBurstShotCount = 2;
constexpr int kPeriodicAimMotorSpeed = 8000;

// offset 像素数 / 该值 = 云台位置增量。数值越小，单次调整越大。
constexpr double kPeriodicYawPixelsPerMotorPosition = 1.35;
constexpr double kPeriodicPitchPixelsPerMotorPosition = 2.7;

// 保持原先“每 80 ms 最多一发”的频率，但不再阻塞视觉回调。只有确认下位机和
// 激光硬件能稳定接收更高频率指令后，才可以减小该值。
constexpr double kLaserShotMinIntervalSec = 0.08;

// 发射窗口。水平导轨靶以 Yaw 为主要精度指标；数值越小命中要求越严格，若仅因
// 视觉抖动导致无法发射才适当增大。
constexpr double kYawFireWindowPixels = 2.0;
constexpr double kPitchFireWindowPixels = 5.0;

// 第二发二次对准开关：true 时，只有最新视觉仍检测到靶子且偏差进入上述
// Yaw/Pitch 发射窗口才发第二发；false 时恢复原来的固定间隔无条件双发。
constexpr bool kPeriodicSecondShotRequireAlignedTarget = true;

// 常规扫描覆盖整个机械可用范围。命中后若视觉随即丢靶，则以最后一次发射时的
// 目标 Yaw 为中心，将总扫描宽度减半，并小幅提高步长和电机速度。
constexpr int kFullScanYawMin = 824;
constexpr int kFullScanYawMax = 3272;
constexpr int kNormalScanStep = 50;
constexpr int kFocusedScanStep = 65;
constexpr int kNormalScanSpeed = 1600;
constexpr int kFocusedScanSpeed = 1900;
constexpr int kScanCommandIntervalCycles = 3;  // 50 Hz 控制循环下约 16.7 Hz 更新
constexpr int kFocusedScanHalfWidth =
    (kFullScanYawMax - kFullScanYawMin) / 4;

// --------------------- 物资识别点位定姿模式 ---------------------
// 预留 ROS 接口：std_msgs/Int32，topic 为 material_recognition_point。
// data=1..16：选择对应点位；data=0：退出定姿模式；其他值：拒绝。
// 每行依次为 {Pitch(Position_0), Yaw(Position_1), 是否已标定}。
// 未标定点位即使收到 topic 也不会驱动云台，防止未填写坐标时误动作。
struct MaterialRecognitionPose
{
  int pitch;
  int yaw;
  bool calibrated;
};

constexpr int kMaterialRecognitionPointCount = 16;
MaterialRecognitionPose gMaterialRecognitionPoses[kMaterialRecognitionPointCount] = {
    {2047, 2047, false},  // 点位 1：待标定
    {2047, 2047, false},  // 点位 2：待标定
    {2047, 2047, false},  // 点位 3：待标定
    {2047, 2047, false},  // 点位 4：待标定
    {2047, 2047, false},  // 点位 5：待标定
    {2047, 2047, false},  // 点位 6：待标定
    {2047, 2047, false},  // 点位 7：待标定
    {2047, 2047, false},  // 点位 8：待标定
    {2047, 2047, false},  // 点位 9：待标定
    {2047, 2047, false},  // 点位10：待标定
    {2047, 2047, false},  // 点位11：待标定
    {2047, 2047, false},  // 点位12：待标定
    {2047, 2047, false},  // 点位13：待标定
    {2047, 2047, false},  // 点位14：待标定
    {2047, 2047, false},  // 点位15：待标定
    {2047, 2047, false},  // 点位16：待标定
};
constexpr int kMaterialRecognitionPositionSpeed = 2000;
// 比赛物资预扫描：以每个 YAML 标定点为中心，Yaw 两侧各 300。
constexpr int kMaterialLocalScanHalfWidth = 300;
constexpr int kMaterialLocalScanPitchSpeed = 2600;
constexpr int kMaterialLocalScanYawSpeed = 3600;

// material_pose_calibration 的 Int32MultiArray 接口：
// [0, point_id]：开始/切换到指定点位标定；[1, pitch, yaw]：设置当前点位姿态；
// [2]：保存当前点位并切换到下一点；[3]：退出标定且释放云台定姿锁定。
constexpr int kMaterialCalibrationCommandStart = 0;
constexpr int kMaterialCalibrationCommandSetPose = 1;
constexpr int kMaterialCalibrationCommandSaveAndNext = 2;
constexpr int kMaterialCalibrationCommandExit = 3;

bool parseYamlInteger(const std::string &line, const std::string &key, int &value)
{
  const std::size_t key_position = line.find(key);
  if (key_position == std::string::npos)
  {
    return false;
  }

  const std::size_t colon_position = line.find(':', key_position + key.size());
  if (colon_position == std::string::npos)
  {
    return false;
  }

  std::istringstream stream(line.substr(colon_position + 1));
  stream >> value;
  return !stream.fail();
}

bool parseYamlBoolean(const std::string &line, const std::string &key, bool &value)
{
  const std::size_t key_position = line.find(key);
  if (key_position == std::string::npos)
  {
    return false;
  }

  const std::size_t colon_position = line.find(':', key_position + key.size());
  if (colon_position == std::string::npos)
  {
    return false;
  }

  const std::string text = line.substr(colon_position + 1);
  if (text.find("true") != std::string::npos)
  {
    value = true;
    return true;
  }
  if (text.find("false") != std::string::npos)
  {
    value = false;
    return true;
  }
  return false;
}

double clampDouble(double value, double lower, double upper)
{
  return std::max(lower, std::min(value, upper));
}

void calculateFocusedScanBounds(int center_yaw, int &min_yaw, int &max_yaw)
{
  min_yaw = center_yaw - kFocusedScanHalfWidth;
  max_yaw = center_yaw + kFocusedScanHalfWidth;

  // 靶子靠近机械边界时整体平移局部窗口，保持“总范围减半”，同时不越界。
  if (min_yaw < kFullScanYawMin)
  {
    max_yaw += kFullScanYawMin - min_yaw;
    min_yaw = kFullScanYawMin;
  }
  if (max_yaw > kFullScanYawMax)
  {
    min_yaw -= max_yaw - kFullScanYawMax;
    max_yaw = kFullScanYawMax;
  }
}
} // namespace

turn_on_robot::turn_on_robot() : Power_voltage(0)
{

  // Clear the data
  // 清空数据
  linear_Speed = 0;
  ThetaSpeed = 0;
  ticksPerMeter = 0;
  ticksPer2PI = 0;
  leftDistance = 0;
  rightDistance = 0;
  calibrate_lineSpeed = 0;
  last_Battery_Percentage = 0;
  count_B = 0;
  count_A = 0;
  count_C = 0;

  Power_max = 12;
  Power_min = 10;

  stop_point_signal_msg = 0;
  find_center = true;   // 默认关闭云台自动扫描
  return_center = false;
  laser_shot_while_target_visible = false;
  post_shot_focused_scan_active = false;
  post_shot_scan_center_yaw = 2047;
  material_recognition_mode_active = false;
  material_local_scan_active = false;
  material_center_hold_active = false;
  material_scan_pitch = GIMBAL_MOTOR0_CENTER_POSITION;
  material_scan_center_yaw = 2047;
  material_scan_yaw_min = kFullScanYawMin;
  material_scan_yaw_max = kFullScanYawMax;
  material_scan_direction = -1;
  shooting_mode_active = false;
  material_recognition_point_id = 0;
  material_calibration_mode_active = false;
  material_calibration_pose_set = false;
  material_calibration_point_id = 0;
  material_calibration_pitch = GIMBAL_MOTOR0_CENTER_POSITION;
  material_calibration_yaw = 2047;
  memset(&Robot_Pos, 0, sizeof(Robot_Pos));
  memset(&Robot_Vel, 0, sizeof(Robot_Vel));
  memset(&Receive_Data, 0, sizeof(Receive_Data));
  memset(&Send_Data, 0, sizeof(Send_Data));
  memset(&Mpu6050_Data, 0, sizeof(Mpu6050_Data));
  memset(&moveBaseControl, 0, sizeof(sMartcarControl));
  moveBaseControl.Position_0 = 2047;
  moveBaseControl.Position_1 = 2047;
  moveBaseControl.Speed_0 = 200;
  moveBaseControl.Speed_1 = 200;
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;
  moveBaseControl.VoiceSwitch = 0;

  ros::NodeHandle private_nh("~"); // Create a node handle //创建节点句柄
  // The private_nh.param() entry parameter corresponds to the initial value of the name of the parameter variable on the parameter server
  // private_nh.param()入口参数分别对应：参数服务器上的名称  参数变量名  初始值
  private_nh.param<std::string>("serial_port_name", serial_port_name, "/dev/ttyACM0"); // Fixed serial port number //固定串口号
  private_nh.param<int>("serial_baud_rate", serial_baud_rate, 115200);                           // Communicate baud rate 115200 to the lower machine //和下位机通信波特率115200
  private_nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");                         // The odometer topic corresponds to the parent TF coordinate //里程计话题对应父TF坐标
  private_nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");                  // The odometer topic corresponds to sub-TF coordinates //里程计话题对应子TF坐标
  private_nh.param<std::string>("gyro_frame_id", gyro_frame_id, "imu_link");                     // IMU topics correspond to TF coordinates //IMU话题对应TF坐标
  private_nh.param("calibrate_lineSpeed", calibrate_lineSpeed, calibrate_lineSpeed);
  private_nh.param("ticksPerMeter", ticksPerMeter, ticksPerMeter);
  private_nh.param("ticksPer2PI", ticksPer2PI, ticksPer2PI);
  std::string default_pose_config_path = ros::package::getPath("dzactuator");
  if (default_pose_config_path.empty())
  {
    default_pose_config_path = "/home/duzhong/dzacs/src/dzactuator";
  }
  default_pose_config_path += "/config/material_recognition_points.yaml";
  private_nh.param<std::string>("material_recognition_pose_config_path",
                                material_recognition_pose_config_path,
                                default_pose_config_path);
  if (!loadMaterialRecognitionPoseTable())
  {
    ROS_WARN("物资识别点位表未加载，所有点位保持未标定状态: %s",
             material_recognition_pose_config_path.c_str());
  }

  voltage_publisher = n.advertise<std_msgs::Float32>("PowerVoltage", 10);            // Create a battery-voltage topic publisher //创建电池电压话题发布者
  Battery_Percentage_pub = n.advertise<std_msgs::Float32>("Battery_Percentage", 10); // Create a battery-voltage topic publisher //创建电池电量百分比话题发布者
  odom_publisher = n.advertise<nav_msgs::Odometry>("odom", 50);                      // Create the odometer topic publisher //创建里程计话题发布者
  imu_publisher = n.advertise<sensor_msgs::Imu>("raw", 20);                          // Create an IMU topic publisher //创建IMU话题发布者
  pub_diff = n.advertise<sensor_msgs::Imu>("imu_data", 20);
  pub_imu_msg_valid = n.advertise<std_msgs::UInt8>("imu_msg_valid", 10);            //Create an IMU online flag  创建IMU在线标志位
  pub_odom_msg_valid = n.advertise<std_msgs::UInt8>("odom_msg_valid", 10);          //Create an ODOM online flag 创建ODOM在线标志位
  pub_LaserShot_Command = n.advertise<std_msgs::UInt8>("LaserShot_Command", 10);    //Create an Laser online flag 创建Lidar在线标志位



  sub_voice_feedback = n.subscribe("/voice_feedback", 10, &turn_on_robot::callback_voice_feedback, this);
  sub_voice_status = n.subscribe("/voice_status", 10, &turn_on_robot::callback_voice_status, this);
  pub_robot_status = n.advertise<std_msgs::UInt8>("robot_status", 10);



  // Set the velocity control command callback function
  // 速度控制命令订阅回调函数设置
  sub_movebase_angle = n.subscribe("pursuitAngle", 1, &turn_on_robot::callback_movebase_angle, this);                           //速度控制回调
  sub_cmd_vel = n.subscribe("cmd_vel", 1, &turn_on_robot::callback_cmd_vel_angle, this);                                        //速度控制回调
  sub_monter_control = n.subscribe("/carema_monter_node/monter_control", 1, &turn_on_robot::callback_monter_control, this);     //云台控制回调
  sub_offset_center = n.subscribe("offset_center", 1, &turn_on_robot::callback_offset_center, this);                            //识别信息回调
  sub_stop_point_singal = n.subscribe("/move_base/stop_signal",1,&turn_on_robot::callback_stop_point_signal,this);              //停车状态回调
  sub_voice_switch = n.subscribe("/voice_switch",1,&turn_on_robot::callback_voice_switch,this);                                 //语音播报回调
  sub_material_scan_mode = n.subscribe("material_scan_mode", 1, &turn_on_robot::callback_material_scan_mode, this);               //物资扫描模式回调
  sub_material_scan_target = n.subscribe("/material_scan_target", 1, &turn_on_robot::callback_material_scan_target, this);        //物资点局部预扫描/回中回调
  sub_shooting_mode = n.subscribe("/shooting_mode", 1, &turn_on_robot::callback_shooting_mode, this);                              //打靶模式回调
  sub_material_recognition_point = n.subscribe("material_recognition_point", 1, &turn_on_robot::callback_material_recognition_point, this); //物资识别定姿点位回调
  sub_arrived_material_number = n.subscribe("/arrived_material_number", 1, &turn_on_robot::callback_arrived_material_number, this); //导航到达物资点回调
  sub_material_pose_calibration = n.subscribe("material_pose_calibration", 1, &turn_on_robot::callback_material_pose_calibration, this); //物资点位坐标标定回调
  sub_scan_gimbal_position = n.subscribe("scan_gimbal_position", 1, &turn_on_robot::callback_scan_gimbal_position, this);       //扫描云台位置回调
  // ros::Publisher det_pub = nh.advertise<std_msgs::Int32MultiArray>("/pt_det_topic", 1);

  imu_buff.clear();
  imu_state.clear();

  imu_flag = 0;
  g0 = 9.8;
  num_imu = 40;

  ROS_INFO_STREAM("Data ready"); // Prompt message //提示信息

  std::cout << FRED("Copyright©2016-2020 dzactuator. All rights reserved ") << std::endl;
  std::cout << FYEL("*****dzactuator:parameters*******************") << std::endl;
  std::cout << FGRN("serial_port_name:") << serial_port_name << std::endl;
  std::cout << FGRN("serial_baud_rate:") << serial_baud_rate << std::endl;
  std::cout << FGRN("calibrate_lineSpeed:") << calibrate_lineSpeed << std::endl;
  std::cout << FGRN("ticksPerMeter:") << ticksPerMeter << std::endl;
  std::cout << FGRN("ticksPer2PI:") << ticksPer2PI << std::endl;
  std::cout << FGRN("Power_max:") << Power_max << std::endl;
  std::cout << FGRN("Power_min:") << Power_min << std::endl;
  std::cout << FGRN("gyro_frame_id:") << gyro_frame_id << std::endl;
  std::cout << FYEL("*****dzactuator:parameters end***************") << std::endl;

  // RCLCPP_ERROR_STREAM(this->get_logger(), "dzactuator can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  try
  {
    // Attempts to initialize and open the serial port //尝试初始化与开启串口
    Stm32_Serial.setPort(serial_port_name);                      // Select the serial port number to enable //选择要开启的串口号
    Stm32_Serial.setBaudrate(serial_baud_rate);                 // Set the baud rate //设置波特率
    serial::Timeout _time = serial::Timeout::simpleTimeout(30); // Timeout //超时等待
    Stm32_Serial.setTimeout(_time);
    Stm32_Serial.open(); // Open the serial port //开启串口
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("dzactuator can not open serial port,Please check the serial port cable! "); // If opening the serial port fails, an error message is printed //如果开启串口失败，打印错误信息
  }
  if (Stm32_Serial.isOpen())
  {
    ROS_INFO_STREAM("dzactuator serial port opened"); // Serial port opened successfully //串口开启成功提示
  }
}

turn_on_robot::~turn_on_robot()
{
  // Sends the stop motion command to the lower machine before the turn_on_robot object ends
  // 对象turn_on_robot结束前向下位机发送停止运动命令
  Send_Data.tx[0] = FRAME_HEADER;
  Send_Data.tx[1] = 0;
  Send_Data.tx[2] = 0;

  // The target velocity of the X-axis of the robot //机器人X轴的目标线速度
  Send_Data.tx[4] = 0;
  Send_Data.tx[3] = 0;

  // The target velocity of the Y-axis of the robot //机器人Y轴的目标线速度
  Send_Data.tx[6] = 0;
  Send_Data.tx[5] = 0;

  // The target velocity of the Z-axis of the robot //机器人Z轴的目标角速度
  Send_Data.tx[8] = 0;
  Send_Data.tx[7] = 0;
  Send_Data.tx[9] = Check_Sum(9, SEND_DATA_CHECK); // Check the bits for the Check_Sum function //校验位，规则参见Check_Sum函数
  Send_Data.tx[10] = FRAME_TAIL;
  try
  {
    Stm32_Serial.write(Send_Data.tx, sizeof(Send_Data.tx)); // Send data to the serial port //向串口发数据
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); // If sending data fails, an error message is printed //如果发送数据失败，打印错误信息
  }
  Stm32_Serial.close(); // Close the serial port //关闭串口
  // RCLCPP_INFO_STREAM(this->get_logger(), "Shutting down"); // Prompt message //提示信息
}

//循环RUN()函数
void turn_on_robot::Control()
{



  std_msgs::UInt8 robot_status_msg;
  robot_status_msg.data = (find_center ? 1 : 0) | (montion_flag ? 2 : 0);
  pub_robot_status.publish(robot_status_msg);



  Robot_Pos.X = 0;
  Robot_Pos.Y = 0;
  Robot_Pos.Z = 0;
  ros::Time current_time, last_time;
  last_time = ros::Time::now();
  ros::Rate rate(50);
  while (ros::ok())
  {
    ros::spinOnce(); // The loop waits for the callback function //循环等待回调函数
    if (true == Get_Sensor_Data_New()) // The serial port reads and verifies the data sent by the lower computer, and then the data is converted to international units
    // 通过串口读取并校验下位机发送过来的数据，然后数据转换为国际单位
    {
      leftDistance += Robot_Vel.Left;
      rightDistance += Robot_Vel.Right;
      // 计算左右轮的速度
      double leftSpeed = Robot_Vel.Left / ticksPerMeter;
      double rightSpeed = Robot_Vel.Right / ticksPerMeter;

      linear_Speed = (leftSpeed + rightSpeed) / 2.0;
 
      ThetaSpeed = (Robot_Vel.Right - Robot_Vel.Left) * (2 * M_PI) / ticksPer2PI; // 使用打死情况下的编码器值计算角度变化

      if (linear_Speed != 0)
      {
        montion_flag = true;

        Robot_Pos.Z += ThetaSpeed; // The angular displacement about the Z axis, in rad //绕Z轴的角位移，单位：rad
        Robot_Pos.Z = normalizeAngle(Robot_Pos.Z);
        Robot_Pos.X += linear_Speed * cos(Robot_Pos.Z); // Calculate the displacement in the X direction, unit: m //计算X方向的位移，单位：m
        Robot_Pos.Y += linear_Speed * sin(Robot_Pos.Z);
      }
      Publish_Odom(); // Pub the speedometer topic //发布里程计话题

      if (calibrate_lineSpeed == 1)
      {
        printf("left=%.2f,right = %.2f,x=%.2f,y=%.2f,th=%.2f,linear_Speed=%f,leftDistance = %.2f,rightDistance = %.2f,Power_voltage = %.2f\n",
               Robot_Vel.Left, Robot_Vel.Right, Robot_Pos.X, Robot_Pos.Y, Robot_Pos.Z,linear_Speed,leftDistance, rightDistance, Power_voltage);
      }
      // 通过IMU绕三轴角速度与三轴加速度计算三轴姿态
      // ============================================================================
      Quaternion_Solution(Mpu6050.angular_velocity.x, Mpu6050.angular_velocity.y, Mpu6050.angular_velocity.z,
                          Mpu6050.linear_acceleration.x, Mpu6050.linear_acceleration.y, Mpu6050.linear_acceleration.z);

      Publish_ImuSensor();          // Pub the IMU topic //发布IMU话题
      Publish_Voltage();            // Pub the topic of power supply voltage //发布电源电压话题
      Publish_Battery_Percentage(); // Pub the topic of power supply voltage //发布电源电量百分比话题
      CaremaMontorControl();        // 云台相机控制策略
      sendCarInfoKernel();          // 向STM32发送控制指令
      rate.sleep();
    }
  }
}

void turn_on_robot::Cal_state_error()
{
  int size = imu_state.size();

  if (size < num_imu)
  {
    printf("wait imu data !\n");

    return;
  }
  else
  {
    printf("start caculate state error!");

    double ax, ay, az, gx, gy, gz;

    ax = 0;
    ay = 0;
    az = 0;
    gx = 0;
    gy = 0;
    gz = 0;

    for (int i = 5; i < size; i++)
    {
      ax += imu_state[i].linear_acceleration.x;
      ay += imu_state[i].linear_acceleration.y;
      az += imu_state[i].linear_acceleration.z;

      gx += imu_state[i].angular_velocity.x;
      gy += imu_state[i].angular_velocity.y;
      gz += imu_state[i].angular_velocity.z;
    }

    err_ax = ax / (size - 5);
    err_ay = ay / (size - 5);
    err_az = az / (size - 5) - g0;

    err_gx = gx / (size - 5);
    err_gy = gy / (size - 5);
    err_gz = gz / (size - 5);

    //  std::cout << "err_gz: " << err_gz << std::endl;

    imu_flag = 1;
    imu_state.clear();
    std::cout << " imu init sucess ! " << std::endl;
  }
}

void turn_on_robot::Average_filtering()
{
  sensor_msgs::Imu average_imu;
  average_imu.header.stamp = ros::Time::now();
  average_imu.header.frame_id = gyro_frame_id;
  int size = imu_buff.size();

  double ax, ay, az, gx, gy, gz;
  ax = 0;
  ay = 0;
  az = 0;
  gx = 0;
  gy = 0;
  gz = 0;

  while (imu_buff.size() > 3)
  {
    imu_buff.pop_front();
  }

  for (int i = 0; i < size; i++)
  {
    ax += imu_buff[i].linear_acceleration.x;
    ay += imu_buff[i].linear_acceleration.y;
    az += imu_buff[i].linear_acceleration.z;

    gx += imu_buff[i].angular_velocity.x;
    gy += imu_buff[i].angular_velocity.y;
    gz += imu_buff[i].angular_velocity.z;
  }

  ax = ax / size;
  ay = ay / size;
  az = az / size;

  gx = gx / size;
  gy = gy / size;
  gz = gz / size;

  average_imu.angular_velocity.x = gx - err_gx;
  average_imu.angular_velocity.y = gy - err_gy;
  average_imu.angular_velocity.z = gz - err_gz;

  // printf("1-  ax=%.2f,ay=%.2f,az=%.2f\n",ax,ay,az);
  average_imu.linear_acceleration.x = ax - err_ax;
  average_imu.linear_acceleration.y = ay - err_ay;
  average_imu.linear_acceleration.z = az - err_az;
  // printf("2-  ax=%.2f,ay=%.2f,az=%.2f\n",average_imu.linear_acceleration.x,average_imu.linear_acceleration.y,average_imu.linear_acceleration.z);

  if (montion_flag)
  {
    average_imu.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    average_imu.orientation_covariance[4] = 1e6;
    average_imu.orientation_covariance[8] = 1e-6;

    average_imu.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    average_imu.angular_velocity_covariance[4] = 1e6;
    average_imu.angular_velocity_covariance[8] = 1e-6;

    Quaternion_Solution(average_imu.angular_velocity.x, average_imu.angular_velocity.y, average_imu.angular_velocity.z,
                        average_imu.linear_acceleration.x, average_imu.linear_acceleration.y, average_imu.linear_acceleration.z);
  }
  else
  {

    average_imu.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    average_imu.orientation_covariance[4] = 1e6;
    average_imu.orientation_covariance[8] = 1e6;

    average_imu.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    average_imu.angular_velocity_covariance[4] = 1e6;
    average_imu.angular_velocity_covariance[8] = 1e6;
  }
  montion_flag = false;

  average_imu.orientation = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

  average_imu.linear_acceleration.z = az - err_az;

  imu_correct = average_imu;

  pub_diff.publish(average_imu);
}

/**
 * @description: The function normalizes angle values.
 * @param {double} angle
 * @return {double}  angle
 * @author: Senerity
 */
double turn_on_robot::normalizeAngle(double angle)
{
  while (angle > M_PI)
  {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI)
  {
    angle += 2.0 * M_PI;
  }
  return angle;
}

/**
 * @description: 云台电机0位置限幅函数
 * @param {int} position - 输入位置值
 * @return {int} 限幅后的位置值
 * @author: Assistant
 */
int turn_on_robot::limitGimbalMotor0Position(int position)
{
  // 应用限幅
  if (position < GIMBAL_MOTOR0_MIN_POSITION) {
    position = GIMBAL_MOTOR0_MIN_POSITION;
    ROS_WARN("云台电机0位置超出下限，已限制到最小值: %d", GIMBAL_MOTOR0_MIN_POSITION);
  } else if (position > GIMBAL_MOTOR0_MAX_POSITION) {
    position = GIMBAL_MOTOR0_MAX_POSITION;
    ROS_WARN("云台电机0位置超出上限，已限制到最大值: %d", GIMBAL_MOTOR0_MAX_POSITION);
  }
  
  return position;
}

bool turn_on_robot::loadMaterialRecognitionPoseTable()
{
  std::ifstream input(material_recognition_pose_config_path.c_str());
  if (!input.is_open())
  {
    return false;
  }

  MaterialRecognitionPose loaded[kMaterialRecognitionPointCount];
  for (int index = 0; index < kMaterialRecognitionPointCount; ++index)
  {
    loaded[index].pitch = GIMBAL_MOTOR0_CENTER_POSITION;
    loaded[index].yaw = 2047;
    loaded[index].calibrated = false;
  }

  int current_index = -1;
  bool found_point = false;
  std::string line;
  while (std::getline(input, line))
  {
    int point_id = 0;
    if (std::sscanf(line.c_str(), " point_%d:", &point_id) == 1)
    {
      current_index = point_id - 1;
      if (current_index >= 0 && current_index < kMaterialRecognitionPointCount)
      {
        found_point = true;
      }
      else
      {
        current_index = -1;
      }
      continue;
    }

    if (current_index < 0)
    {
      continue;
    }

    int value = 0;
    if (parseYamlInteger(line, "pitch", value))
    {
      loaded[current_index].pitch = value;
      continue;
    }
    if (parseYamlInteger(line, "yaw", value))
    {
      loaded[current_index].yaw = value;
      continue;
    }

    bool calibrated = false;
    if (parseYamlBoolean(line, "calibrated", calibrated))
    {
      loaded[current_index].calibrated = calibrated;
    }
  }

  if (!found_point)
  {
    return false;
  }

  for (int index = 0; index < kMaterialRecognitionPointCount; ++index)
  {
    gMaterialRecognitionPoses[index] = loaded[index];
  }
  ROS_INFO("物资识别点位表已加载: %s",
           material_recognition_pose_config_path.c_str());
  return true;
}

bool turn_on_robot::saveMaterialRecognitionPoseTable() const
{
  const std::string temporary_path = material_recognition_pose_config_path + ".tmp";
  std::ofstream output(temporary_path.c_str());
  if (!output.is_open())
  {
    ROS_ERROR("无法写入物资识别点位临时表: %s", temporary_path.c_str());
    return false;
  }

  output << "# 物资识别点位表。Pitch 对应 Position_0，Yaw 对应 Position_1。\n";
  output << "# calibrated 为 true 的点位才允许 material_recognition_point 调用。\n";
  output << "material_recognition_points:\n";
  for (int index = 0; index < kMaterialRecognitionPointCount; ++index)
  {
    const MaterialRecognitionPose &pose = gMaterialRecognitionPoses[index];
    output << "  point_" << (index + 1) << ":\n";
    output << "    pitch: " << pose.pitch << "\n";
    output << "    yaw: " << pose.yaw << "\n";
    output << "    calibrated: " << (pose.calibrated ? "true" : "false") << "\n";
  }
  output.close();

  if (std::rename(temporary_path.c_str(), material_recognition_pose_config_path.c_str()) != 0)
  {
    ROS_ERROR("无法保存物资识别点位表: %s", material_recognition_pose_config_path.c_str());
    return false;
  }
  return true;
}

/**************************************
Date: January 28, 2021
Function: Data conversion function
功能: 数据转换函数
***************************************/
short turn_on_robot::IMU_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8;
  transition_16 |= Data_Low;
  return transition_16;
}

float turn_on_robot::Odom_Trans(uint8_t Data_High, uint8_t Data_Low)
{
  float data_return;
  short transition_16;
  transition_16 = 0;
  transition_16 |= Data_High << 8; // Get the high 8 bits of data   //获取数据的高8位
  transition_16 |= Data_Low;       // Get the lowest 8 bits of data //获取数据的低8位
  data_return = transition_16;     //(transition_16 / 1000)+(transition_16 % 1000)*0.001; // The speed unit is changed from mm/s to m/s //速度单位从mm/s转换为m/s
  return data_return;
}

/**************************************
Date: January 28, 2021
Function: The speed topic subscription Callback function, according to the subscribed instructions through the serial port command control of the lower computer
功能: 速度话题订阅回调函数Callback，根据订阅的指令通过串口发指令控制下位机
***************************************/
void turn_on_robot::callback_movebase_angle(const geometry_msgs::Twist::ConstPtr &msg)
{
  // printf("callback_movebase_angle\n");
  float v = msg->linear.x;
  float w = msg->angular.z;

  moveBaseControl.TargetSpeed = abs(clamp(v * 45 / 0.25, -255.0, 255.0));
  moveBaseControl.TargetAngle = w;
  moveBaseControl.TargetAngle += 60;

  // printf("linear X = %.2d\n",moveBaseControl.TargetSpeed);
  if (moveBaseControl.TargetAngle < 20)
  {
    moveBaseControl.TargetAngle = 20;
  }
  if (moveBaseControl.TargetAngle > 100)
  {
    moveBaseControl.TargetAngle = 100;
  }
  // linear speed
  if (msg->linear.x > 0)
    moveBaseControl.SpeedDirection = 0x00;
  else if (msg->linear.x < 0)
    moveBaseControl.SpeedDirection = 0x01;
  else if (msg->linear.x == 0)
    moveBaseControl.SpeedDirection = 0x00;
}

void turn_on_robot::callback_cmd_vel_angle(const geometry_msgs::Twist::ConstPtr &msg)
{
  // printf("callback_cmd_vel_angle\n");
  float v = msg->linear.x;
  float w = msg->angular.z;

  moveBaseControl.TargetSpeed = abs(clamp(v * 45 / 0.25, -255.0, 255.0));
  if(!v == 0)
  {
    moveBaseControl.TargetAngle = round(atan(CARL * w / v) * 57.3);
  }
  else{
    moveBaseControl.TargetAngle = 0;
  }
  moveBaseControl.TargetAngle += 60;

  // std::cout << " msg->linear.x = " << moveBaseControl.TargetSpeed << " msg->angular.z =  " <<  moveBaseControl.TargetAngle <<std::endl;

  // printf("linear X = %.2f,gngularZ = %.2f,%d,angle = %d\n",msg->linear.x,msg->angular.z,
  // abs(moveBaseControl.TargetSpeed),abs(moveBaseControl.TargetAngle));
  if (moveBaseControl.TargetAngle < 20)
  {
    moveBaseControl.TargetAngle = 20;
  }
  if (moveBaseControl.TargetAngle > 100)
  {
    moveBaseControl.TargetAngle = 100;
  }
  // linear speed
  if (msg->linear.x > 0)
    moveBaseControl.SpeedDirection = 0x00;
  else if (msg->linear.x < 0)
    moveBaseControl.SpeedDirection = 0x01;
  else if (msg->linear.x == 0)
    moveBaseControl.SpeedDirection = 0x00;
  // // angle
  // if (moveBaseControl.TargetAngle > 0)
  //   moveBaseControl.TargetShiftPosition = 0x20; // turn left
  // else if (moveBaseControl.TargetAngle < 0)
  //   moveBaseControl.TargetShiftPosition = 0x10; // turn right
  // else if (moveBaseControl.TargetAngle == 0)
  //   moveBaseControl.TargetShiftPosition = 0x00; // turn left
}

void turn_on_robot::callback_voice_switch(const std_msgs::UInt8::ConstPtr &msg){
    moveBaseControl.VoiceSwitch= static_cast<int>(msg->data);
    std::cout << "moveBaseControl.VoiceSwitch = " << moveBaseControl.VoiceSwitch << std::endl;
}

void turn_on_robot::callback_stop_point_signal(const std_msgs::UInt8::ConstPtr &msg){
  stop_point_signal_msg= static_cast<int>(msg->data);
  std::cout << "stop_msgs = " << stop_point_signal_msg << std::endl;
}

void turn_on_robot::callback_monter_control(const geometry_msgs::Twist::ConstPtr &msg)
{
  if (material_recognition_mode_active || material_center_hold_active)
  {
    ROS_WARN_THROTTLE(1.0, "物资识别定姿模式中，忽略 monter_control 云台位置命令");
    return;
  }

  moveBaseControl.Position_0 = msg->linear.x;
  moveBaseControl.Position_1 = msg->linear.y;
  moveBaseControl.Speed_0 = 1000;
  moveBaseControl.Speed_1 = 1000;
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;
  moveBaseControl.Fun = msg->linear.z;
  moveBaseControl.Orin_ID = 1;
  moveBaseControl.Set_ID_Num = 1;
  // printf("callback_monter_control\n");
}
//用于云台舵机的速度调控
void turn_on_robot::CaremaMontorControl(){  
    if(find_center == false)
    {
      // if(stop_point_signal_msg == 1 ){
        // 静态变量用于保持状态
      static int direction_0 = -1;  // 控制扫描方向：1表示正向，-1表示反向
      static int direction_1 = -1;  // 控制扫描方向：1表示正向，-1表示反向

      static int scan_interval = 0; // 控制扫描速度的计数器

      int scan_yaw_min = material_local_scan_active
          ? material_scan_yaw_min : kFullScanYawMin;
      int scan_yaw_max = material_local_scan_active
          ? material_scan_yaw_max : kFullScanYawMax;
      const int scan_step = material_local_scan_active
          ? kNormalScanStep
          : (post_shot_focused_scan_active ? kFocusedScanStep : kNormalScanStep);
      const int scan_speed = material_local_scan_active
          ? kMaterialLocalScanYawSpeed
          : (post_shot_focused_scan_active ? kFocusedScanSpeed : kNormalScanSpeed);

      if (post_shot_focused_scan_active)
      {
        calculateFocusedScanBounds(
            post_shot_scan_center_yaw, scan_yaw_min, scan_yaw_max);
      }

      moveBaseControl.Position_0 = material_local_scan_active
          ? material_scan_pitch : GIMBAL_MOTOR0_CENTER_POSITION;

      // 降低扫描位置更新频率，给视觉留出完成检测和回传结果的时间。
      if (++scan_interval >= kScanCommandIntervalCycles)
      {
        scan_interval = 0;

        // 更新电机1的位置
        const int direction = material_local_scan_active
            ? material_scan_direction : direction_0;
        moveBaseControl.Position_1 += direction * scan_step;
        // moveBaseControl.Position_0 +=direction_0 * 50;
        // 到达当前搜索窗口边界后切换方向。
        if (moveBaseControl.Position_1 >= scan_yaw_max)
        {
          moveBaseControl.Position_1 = scan_yaw_max;
          if (material_local_scan_active)
          {
            material_scan_direction = -1;
          }
          else
          {
            direction_0 = -1; // 反向扫描
          }
        }
        else if (moveBaseControl.Position_1 <= scan_yaw_min)
        {
          moveBaseControl.Position_1 = scan_yaw_min;
          if (material_local_scan_active)
          {
            material_scan_direction = 1;
          }
          else
          {
            direction_0 = 1; // 正向扫描
          }
        }
        // if(moveBaseControl.Position_0 >=2047)
        // {
        //   moveBaseControl.Position_0=2047;
        //   direction_1= -1;
        // }
        // else if(moveBaseControl.Position_0 <=1847)
        // {
        //   moveBaseControl.Position_0=1847;
        //   direction_1= 1; 
        // }
      }

      // 设置电机速度
      moveBaseControl.Speed_0 = material_local_scan_active
          ? kMaterialLocalScanPitchSpeed : kNormalScanSpeed;//Pitch_Speed
      moveBaseControl.Speed_1 = scan_speed;//Yaw_Speed
    //}
    // else if(stop_point_signal_msg == 0 ){
    //     moveBaseControl.Position_0 = 2047;
    //     moveBaseControl.Position_1 = 2047;
    //     moveBaseControl.Speed_0 = 2000;
    //     moveBaseControl.Speed_1 = 2000;
    // }
  }
}




void turn_on_robot::callback_voice_feedback(const std_msgs::String::ConstPtr &msg){
  ROS_INFO("收到语音反馈: %s", msg->data.c_str());
}

void turn_on_robot::callback_voice_status(const std_msgs::UInt8::ConstPtr &msg){
  ROS_INFO("语音模块状态: %d", msg->data);
}

// 物资扫描模式回调
void turn_on_robot::callback_material_scan_mode(const std_msgs::UInt8::ConstPtr &msg)
{
  if (msg->data == 1)
  {
    material_local_scan_active = false;
    material_center_hold_active = false;
    material_recognition_mode_active = false;
    material_recognition_point_id = 0;
    material_calibration_mode_active = false;
    material_calibration_pose_set = false;
    material_calibration_point_id = 0;
    find_center = false;  // 收到明确命令后才开始扫描
    ROS_INFO("[dzactuator] Material scan mode activated");
  }
  else
  {
    material_local_scan_active = false;
    find_center = true;   // 停止扫描
    post_shot_focused_scan_active = false;
    ROS_INFO("[dzactuator] Material scan mode deactivated");
  }
}

// 比赛导航的预扫描接口：data=1..16 进入该标定点的局部扫描；data=0
// 立即回全局中心并锁住，确保最后一个物资点到打靶点的行驶期间不再被
// 视觉或迟到的物资到达消息覆盖。
void turn_on_robot::callback_material_scan_target(const std_msgs::UInt8::ConstPtr &msg)
{
  const int requested_point = static_cast<int>(msg->data);
  if (shooting_mode_active)
  {
    ROS_WARN("打靶模式中，忽略物资预扫描请求: %d", requested_point);
    return;
  }

  if (requested_point == 0)
  {
    material_local_scan_active = false;
    material_center_hold_active = true;
    material_recognition_mode_active = false;
    material_recognition_point_id = 0;
    material_calibration_mode_active = false;
    material_calibration_pose_set = false;
    material_calibration_point_id = 0;
    find_center = true;
    post_shot_focused_scan_active = false;
    moveBaseControl.Position_0 = GIMBAL_MOTOR0_CENTER_POSITION;
    moveBaseControl.Position_1 = 2047;
    moveBaseControl.Speed_0 = kMaterialLocalScanPitchSpeed;
    moveBaseControl.Speed_1 = kMaterialLocalScanPitchSpeed;
    moveBaseControl.Time_0 = 0;
    moveBaseControl.Time_1 = 0;
    moveBaseControl.Fun = 0;
    ROS_INFO("物资预扫描结束：云台回中心并保持，等待打靶点");
    return;
  }

  if (requested_point < 1 || requested_point > kMaterialRecognitionPointCount)
  {
    ROS_WARN("物资预扫描点位无效: %d，允许范围为 1-%d（0 为回中心）",
             requested_point, kMaterialRecognitionPointCount);
    return;
  }

  const MaterialRecognitionPose &pose =
      gMaterialRecognitionPoses[requested_point - 1];
  if (!pose.calibrated)
  {
    ROS_WARN("物资预扫描点位 %d 尚未标定，已忽略", requested_point);
    return;
  }

  material_scan_pitch = limitGimbalMotor0Position(pose.pitch);
  material_scan_center_yaw = clamp(pose.yaw, kFullScanYawMin, kFullScanYawMax);
  // 两侧分别截断，保留标定中心点；不会为了凑满宽度而把窗口整体平移。
  material_scan_yaw_min = std::max(
      kFullScanYawMin, material_scan_center_yaw - kMaterialLocalScanHalfWidth);
  material_scan_yaw_max = std::min(
      kFullScanYawMax, material_scan_center_yaw + kMaterialLocalScanHalfWidth);
  material_scan_direction = -1;
  material_local_scan_active = true;
  material_center_hold_active = false;
  // 复用定姿锁来屏蔽视觉跟踪和外部扫描话题；CaremaMontorControl 对局部
  // 扫描状态有专门分支，仍会持续发送本扫描命令。
  material_recognition_mode_active = true;
  material_recognition_point_id = requested_point;
  material_calibration_mode_active = false;
  material_calibration_pose_set = false;
  material_calibration_point_id = 0;
  post_shot_focused_scan_active = false;
  find_center = false;
  moveBaseControl.Position_0 = material_scan_pitch;
  moveBaseControl.Position_1 = material_scan_center_yaw;
  moveBaseControl.Speed_0 = kMaterialLocalScanPitchSpeed;
  moveBaseControl.Speed_1 = kMaterialLocalScanYawSpeed;
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;
  moveBaseControl.Fun = 0;
  ROS_INFO("物资预扫描：点位=%d, Pitch=%d, Yaw中心=%d, 扫描=[%d,%d], 速度=%d",
           requested_point, material_scan_pitch, material_scan_center_yaw,
           material_scan_yaw_min, material_scan_yaw_max,
           kMaterialLocalScanYawSpeed);
}

// This command is sent by auto_navigation only after the shooting model
// reports itself active on /current_model.
void turn_on_robot::callback_shooting_mode(const std_msgs::Bool::ConstPtr &msg)
{
  if (!msg->data)
  {
    shooting_mode_active = false;
    find_center = true;
    post_shot_focused_scan_active = false;
    ROS_INFO("[dzactuator] Shooting mode deactivated; gimbal scan stopped");
    return;
  }

  // Shooting must not inherit a material pose lock. A later delayed material
  // message is ignored while this mode is active.
  shooting_mode_active = true;
  material_local_scan_active = false;
  material_center_hold_active = false;
  material_recognition_mode_active = false;
  material_recognition_point_id = 0;
  material_calibration_mode_active = false;
  material_calibration_pose_set = false;
  material_calibration_point_id = 0;
  laser_shot_while_target_visible = false;
  post_shot_focused_scan_active = false;
  find_center = false;
  ROS_INFO("[dzactuator] Shooting mode active; gimbal scan started");
}

// 物资识别点位定姿回调。
// 接口预留：std_msgs/Int32.data 为 1..16 的到达点位编号，0 用于退出定姿模式。
void turn_on_robot::callback_material_recognition_point(const std_msgs::Int32::ConstPtr &msg)
{
  set_material_recognition_point(msg->data);
}

// 导航节点发布 /arrived_material_number（std_msgs/UInt8，1..16）后，
// 仍供识别/语音模块使用。若云台已经在对应物资局部扫描，则不要把它
// 改回固定定姿，保证到达后的三秒窗口内扫描不断。
void turn_on_robot::callback_arrived_material_number(const std_msgs::UInt8::ConstPtr &msg)
{
  ROS_INFO("收到导航到达物资点: %u", static_cast<unsigned int>(msg->data));
  if (material_center_hold_active)
  {
    ROS_INFO("云台已在打靶前中心保持状态，忽略迟到的物资定姿请求");
    return;
  }
  if (material_local_scan_active)
  {
    ROS_INFO("物资局部扫描持续中，到达通知不覆盖当前扫描姿态");
    return;
  }
  set_material_recognition_point(static_cast<int>(msg->data));
}

void turn_on_robot::set_material_recognition_point(int requested_point)
{
  if (requested_point != 0 && shooting_mode_active)
  {
    ROS_WARN("打靶模式中，忽略物资识别定姿请求: %d", requested_point);
    return;
  }

  if (requested_point == 0)
  {
    material_local_scan_active = false;
    material_center_hold_active = false;
    material_recognition_mode_active = false;
    material_recognition_point_id = 0;
    material_calibration_mode_active = false;
    material_calibration_pose_set = false;
    material_calibration_point_id = 0;
    find_center = true;
    post_shot_focused_scan_active = false;
    ROS_INFO("物资识别定姿模式已退出");
    return;
  }

  if (requested_point < 1 || requested_point > kMaterialRecognitionPointCount)
  {
    ROS_WARN("物资识别点位无效: %d，允许范围为 1-%d（0 为退出）",
             requested_point, kMaterialRecognitionPointCount);
    return;
  }

  const MaterialRecognitionPose &pose =
      gMaterialRecognitionPoses[requested_point - 1];
  if (!pose.calibrated)
  {
    ROS_WARN("物资识别点位 %d 尚未标定，已忽略本次定姿请求",
             requested_point);
    return;
  }

  const int pitch = limitGimbalMotor0Position(pose.pitch);
  const int yaw = clamp(pose.yaw, kFullScanYawMin, kFullScanYawMax);
  material_local_scan_active = false;
  material_center_hold_active = false;
  material_recognition_mode_active = true;
  material_recognition_point_id = requested_point;
  material_calibration_mode_active = false;
  material_calibration_pose_set = false;
  material_calibration_point_id = 0;
  find_center = true;
  post_shot_focused_scan_active = false;
  moveBaseControl.Position_0 = pitch;
  moveBaseControl.Position_1 = yaw;
  moveBaseControl.Speed_0 = kMaterialRecognitionPositionSpeed;
  moveBaseControl.Speed_1 = kMaterialRecognitionPositionSpeed;
  moveBaseControl.Time_0 = 0;
  moveBaseControl.Time_1 = 0;
  moveBaseControl.Fun = 0;

  ROS_INFO("物资识别定姿：点位=%d, Pitch=%d, Yaw=%d",
           requested_point, pitch, yaw);
}

void turn_on_robot::callback_material_pose_calibration(
    const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if (msg->data.empty())
  {
    ROS_WARN("物资点位标定消息为空");
    return;
  }

  const int command = msg->data[0];
  if (command == kMaterialCalibrationCommandStart)
  {
    if (msg->data.size() < 2 || msg->data[1] < 1 ||
        msg->data[1] > kMaterialRecognitionPointCount)
    {
      ROS_WARN("开始标定格式应为 [0, point_id]，point_id 范围为 1-%d",
               kMaterialRecognitionPointCount);
      return;
    }

    material_calibration_mode_active = true;
    material_local_scan_active = false;
    material_center_hold_active = false;
    material_calibration_pose_set = false;
    material_calibration_point_id = msg->data[1];
    material_recognition_mode_active = true;
    material_recognition_point_id = 0;
    find_center = true;
    post_shot_focused_scan_active = false;
    ROS_INFO("开始标定物资点位 %d；请发送 [1, pitch, yaw] 调整姿态",
             material_calibration_point_id);
    return;
  }

  if (command == kMaterialCalibrationCommandSetPose)
  {
    if (!material_calibration_mode_active || msg->data.size() < 3)
    {
      ROS_WARN("设置标定姿态前请先发送 [0, point_id]；设置格式为 [1, pitch, yaw]");
      return;
    }

    material_calibration_pitch = limitGimbalMotor0Position(msg->data[1]);
    material_calibration_yaw = clamp(msg->data[2], kFullScanYawMin, kFullScanYawMax);
    material_calibration_pose_set = true;
    moveBaseControl.Position_0 = material_calibration_pitch;
    moveBaseControl.Position_1 = material_calibration_yaw;
    moveBaseControl.Speed_0 = kMaterialRecognitionPositionSpeed;
    moveBaseControl.Speed_1 = kMaterialRecognitionPositionSpeed;
    moveBaseControl.Time_0 = 0;
    moveBaseControl.Time_1 = 0;
    moveBaseControl.Fun = 0;
    ROS_INFO("标定点位 %d 调整到 Pitch=%d, Yaw=%d",
             material_calibration_point_id, material_calibration_pitch,
             material_calibration_yaw);
    return;
  }

  if (command == kMaterialCalibrationCommandSaveAndNext)
  {
    if (!material_calibration_mode_active || !material_calibration_pose_set)
    {
      ROS_WARN("当前点位尚未设置 Pitch/Yaw，不能保存");
      return;
    }

    MaterialRecognitionPose &pose =
        gMaterialRecognitionPoses[material_calibration_point_id - 1];
    pose.pitch = material_calibration_pitch;
    pose.yaw = material_calibration_yaw;
    pose.calibrated = true;
    if (!saveMaterialRecognitionPoseTable())
    {
      ROS_ERROR("点位 %d 保存失败，仍停留在当前点位",
                material_calibration_point_id);
      return;
    }

    if (material_calibration_point_id == kMaterialRecognitionPointCount)
    {
      material_calibration_mode_active = false;
      material_calibration_pose_set = false;
      material_recognition_mode_active = true;
      material_recognition_point_id = kMaterialRecognitionPointCount;
      ROS_INFO("16 个物资点位均已完成保存，云台保持在点位 16 姿态");
      return;
    }

    ++material_calibration_point_id;
    material_calibration_pose_set = false;
    material_recognition_point_id = 0;
    ROS_INFO("点位已保存；已切换到点位 %d。云台保持当前位置，等待下一条 [1, pitch, yaw]",
             material_calibration_point_id);
    return;
  }

  if (command == kMaterialCalibrationCommandExit)
  {
    material_calibration_mode_active = false;
    material_local_scan_active = false;
    material_center_hold_active = false;
    material_calibration_pose_set = false;
    material_calibration_point_id = 0;
    material_recognition_mode_active = false;
    material_recognition_point_id = 0;
    find_center = true;
    post_shot_focused_scan_active = false;
    ROS_INFO("物资点位标定已退出，未保存的当前调整已丢弃");
    return;
  }

  ROS_WARN("未知物资点位标定命令: %d", command);
}

// 扫描云台位置回调
void turn_on_robot::callback_scan_gimbal_position(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if (material_recognition_mode_active || material_center_hold_active)
  {
    ROS_WARN_THROTTLE(1.0, "物资识别定姿模式中，忽略 scan_gimbal_position 命令");
    return;
  }

  if (msg->data.size() >= 2)
  {
    // 设置云台位置进行扫描
    moveBaseControl.Position_0 = msg->data[0];  // pitch 抬高
    moveBaseControl.Position_1 = msg->data[1];  // yaw 扫描位置
    moveBaseControl.Speed_0 = 2000;
    moveBaseControl.Speed_1 = 2000;
    moveBaseControl.Fun = 0;
  }
}




void turn_on_robot::callback_offset_center(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
  if (material_recognition_mode_active || material_center_hold_active)
  {
    ROS_DEBUG_THROTTLE(1.0, "物资识别定姿模式中，暂停视觉云台跟踪");
    return;
  }

  if (msg->data.size() < 3)
  {
    ROS_WARN_THROTTLE(1.0, "offset_center 消息格式应为 [Yaw偏差, Pitch偏差, 是否检测到]");
    return;
  }

  std_msgs::Int32MultiArray temp_msg = *msg;
  static double prev_pitch_offset = 0.0;
  static double prev_yaw_offset = 0.0;
  static bool has_prev_pitch_offset = false;
  static bool has_prev_yaw_offset = false;
  static int last_pitch_command = GIMBAL_MOTOR0_CENTER_POSITION;
  static int last_yaw_command = 2047;
  static double previous_target_yaw = 0.0;
  static double previous_raw_target_yaw = 0.0;
  static double filtered_target_yaw_velocity = 0.0;
  static double filtered_yaw_lead_position = 0.0;
  static ros::Time previous_target_yaw_time;
  static bool has_target_yaw_sample = false;
  static bool yaw_endpoint_hold_active = false;
  static int yaw_endpoint_stationary_frames = 0;
  static int last_yaw_motion_direction = 0;
  static ros::Time last_yaw_endpoint_transition_time;
  static ros::Time yaw_endpoint_hold_start_time;
  static int yaw_endpoint_centered_frames = 0;
  static bool laser_shot_at_current_endpoint = false;
  static ros::Time last_laser_shot_time;
  static ros::Time last_periodic_aim_time;
  static ros::Time next_periodic_burst_shot_time;
  static int periodic_burst_shots_remaining = 0;
  static int periodic_aim_yaw = 2047;
  double filtered_pitch_offset = temp_msg.data[1];
  double filtered_yaw_offset = temp_msg.data[0];

  if (has_prev_pitch_offset)
  {
    filtered_pitch_offset = 0.45
     * prev_pitch_offset + 0.55 * static_cast<double>(temp_msg.data[1]);
  }
  if (has_prev_yaw_offset)
  {
    filtered_yaw_offset = kYawOffsetFilterOldWeight * prev_yaw_offset +
        (1.0 - kYawOffsetFilterOldWeight) * static_cast<double>(temp_msg.data[0]);
  }
  has_prev_pitch_offset = true;
  has_prev_yaw_offset = true;
  prev_pitch_offset = filtered_pitch_offset;
  prev_yaw_offset = filtered_yaw_offset;

  const double pitch_deadband = 2.0;//死区
  const double yaw_deadband = 2.0;//死区
  if (std::abs(filtered_pitch_offset) < pitch_deadband)
  {
    filtered_pitch_offset = 0.0;
  }
  if (std::abs(filtered_yaw_offset) < yaw_deadband)
  {
    filtered_yaw_offset = 0.0;
  }

  ROS_INFO_THROTTLE(0.5,
                    "offsets: filtered_pitch=%.2f filtered_yaw=%.2f int_pitch=%d int_yaw=%d",
                    filtered_pitch_offset, filtered_yaw_offset,
                    static_cast<int>(filtered_pitch_offset), static_cast<int>(filtered_yaw_offset));

  auto applySlewLimit = [](int target, int current, int max_step)
  {
    const int delta = target - current;
    if (delta > max_step)
    {
      return current + max_step;
    }
    if (delta < -max_step)
    {
      return current - max_step;
    }
    return target;
  };

  if (kUsePeriodicStepAndBurstMode)
  {
    const ros::Time periodic_now = ros::Time::now();
    const bool burst_was_active = periodic_burst_shots_remaining > 0;

    // 双发使用视觉回调的后续帧定时触发，不 sleep，不阻塞图像和串口。
    if (burst_was_active &&
        periodic_now >= next_periodic_burst_shot_time)
    {
      const bool is_second_periodic_shot =
          periodic_burst_shots_remaining == 1;
      const bool second_shot_is_aligned =
          !kPeriodicSecondShotRequireAlignedTarget ||
          (temp_msg.data[2] == 1 &&
           std::abs(static_cast<double>(temp_msg.data[0])) <
               kYawFireWindowPixels &&
           std::abs(static_cast<double>(temp_msg.data[1])) <
               kPitchFireWindowPixels);

      // 第一发保持原来的固定等待后发射。第二发未重新检测到靶子或未居中
      // 时直接取消，让下一帧视觉重新瞄准，避免在目标已经离开时补发。
      if (is_second_periodic_shot && !second_shot_is_aligned)
      {
        periodic_burst_shots_remaining = 0;
        ROS_INFO("定时单步双发：第二发取消，目标未重新对准 (yaw=%d, pitch=%d, detected=%d)",
                 temp_msg.data[0], temp_msg.data[1], temp_msg.data[2]);
        return;
      }

      std_msgs::UInt8 shotdata;
      shotdata.data = 1;
      pub_LaserShot_Command.publish(shotdata);
      shotdata.data = 0;
      pub_LaserShot_Command.publish(shotdata);

      --periodic_burst_shots_remaining;
      last_laser_shot_time = periodic_now;
      laser_shot_while_target_visible = true;
      post_shot_scan_center_yaw = periodic_aim_yaw;

      if (periodic_burst_shots_remaining > 0)
      {
        next_periodic_burst_shot_time = periodic_now +
            ros::Duration(kPeriodicBurstShotIntervalSec);
      }

      ROS_INFO("定时单步双发：已发射，剩余=%d",
               periodic_burst_shots_remaining);
    }

    // 双发完成前锁住上一次单步位置；即使第一发后视觉丢靶，
    // 也不让无目标扫描立即改变云台位置。
    if (burst_was_active)
    {
      find_center = true;
      return;
    }

    if (temp_msg.data[2] == 1)
    {
      find_center = true;
      post_shot_focused_scan_active = false;

      const bool aim_interval_elapsed = last_periodic_aim_time.isZero() ||
          (periodic_now - last_periodic_aim_time).toSec() >=
              kPeriodicAimIntervalSec;
      if (aim_interval_elapsed)
      {
        const int periodic_pitch = limitGimbalMotor0Position(
            curYuntai_feedback_data.Position_0 + static_cast<int>(std::lround(
                static_cast<double>(temp_msg.data[1]) /
                kPeriodicPitchPixelsPerMotorPosition)));
        periodic_aim_yaw = clamp(
            curYuntai_feedback_data.Position_1 +
                static_cast<int>(std::lround(
                    static_cast<double>(temp_msg.data[0]) /
                    kPeriodicYawPixelsPerMotorPosition)),
            kFullScanYawMin, kFullScanYawMax);

        // 一次直接下发计算位置，不经过实时 PID、预瞄或步长限制。
        moveBaseControl.Position_0 = periodic_pitch;
        moveBaseControl.Position_1 = periodic_aim_yaw;
        moveBaseControl.Speed_0 = kPeriodicAimMotorSpeed;
        moveBaseControl.Speed_1 = kPeriodicAimMotorSpeed;
        moveBaseControl.Fun = 0;
        last_pitch_command = periodic_pitch;
        last_yaw_command = periodic_aim_yaw;

        last_periodic_aim_time = periodic_now;
        periodic_burst_shots_remaining = kPeriodicBurstShotCount;
        next_periodic_burst_shot_time = periodic_now +
            ros::Duration(kPeriodicAimSettleTimeSec);

        ROS_INFO("定时单步瞄准: offset_yaw=%d offset_pitch=%d target_yaw=%d target_pitch=%d",
                 temp_msg.data[0], temp_msg.data[1],
                 periodic_aim_yaw, periodic_pitch);
      }
      return;
    }

    // 没有目标且双发已结束时，下次重新识别到靶子立即采样，
    // 不继承上一个目标的 0.3 秒计时。
    last_periodic_aim_time = ros::Time();
  }

  // if (temp_msg.data[2] == 1 && stop_point_signal_msg == 1)
  if (temp_msg.data[2] == 1)
  { // 检测到目标
    find_center = true;

    const bool target_acquired_this_frame = !has_target_yaw_sample;
    if (target_acquired_this_frame)
    {
      // 一旦重新识别到目标便结束上一轮局部搜索。只有本轮再次实际发射后，
      // 随后的丢靶才允许重新进入局部快扫。
      post_shot_focused_scan_active = false;
      laser_shot_while_target_visible = false;

      // 扫描期间云台已经离开上一次跟踪指令。重新锁定时以实际反馈位置
      // 为新起点，防止 last_*_command 把云台瞬间拉回旧位置。
      last_pitch_command = curYuntai_feedback_data.Position_0;
      last_yaw_command = curYuntai_feedback_data.Position_1;
      filtered_target_yaw_velocity = 0.0;
      filtered_yaw_lead_position = 0.0;
      yaw_endpoint_hold_active = false;
      yaw_endpoint_stationary_frames = 0;
      last_yaw_motion_direction = 0;
      last_yaw_endpoint_transition_time = ros::Time();
      yaw_endpoint_hold_start_time = ros::Time();
      yaw_endpoint_centered_frames = 0;
      laser_shot_at_current_endpoint = false;

      // 丢失目标后的历史偏差不参与新目标第一帧滤波。
      filtered_pitch_offset = static_cast<double>(temp_msg.data[1]);
      filtered_yaw_offset = static_cast<double>(temp_msg.data[0]);
      prev_pitch_offset = filtered_pitch_offset;
      prev_yaw_offset = filtered_yaw_offset;
    }

    // 目标水平位置估计值 = 云台实际反馈位置 + 当前视觉残余偏差。
    // 不再使用上次指令位置，避免云台未到位时把指令变化误判为靶子速度。
    const ros::Time now = ros::Time::now();
    const double raw_target_yaw =
        static_cast<double>(curYuntai_feedback_data.Position_1) +
        static_cast<double>(temp_msg.data[0]) /
            kYawPixelsPerMotorPosition;
    const double estimated_target_yaw =
        static_cast<double>(curYuntai_feedback_data.Position_1) +
        filtered_yaw_offset / kYawPixelsPerMotorPosition;

    double yaw_lead_position = 0.0;
    double predicted_yaw_offset = filtered_yaw_offset;
    bool yaw_reversed_this_frame = false;
    bool yaw_endpoint_transition_this_frame = false;
    if (has_target_yaw_sample)
    {
      const double dt = (now - previous_target_yaw_time).toSec();
      // 相机暂停或时钟重置后，旧帧不能被错误地解释为很高的目标速度。
      if (dt > 0.001 && dt < 1.0)
      {
        const double raw_velocity = clampDouble(
            (estimated_target_yaw - previous_target_yaw) / dt,
            -kYawMaxEstimatedVelocity, kYawMaxEstimatedVelocity);
        const double endpoint_observed_velocity = clampDouble(
            (raw_target_yaw - previous_raw_target_yaw) / dt,
            -kYawMaxEstimatedVelocity, kYawMaxEstimatedVelocity);
        const double previous_velocity = filtered_target_yaw_velocity;
        const double candidate_filtered_velocity =
            kYawVelocityFilterOldWeight * filtered_target_yaw_velocity +
            (1.0 - kYawVelocityFilterOldWeight) * raw_velocity;

        const bool endpoint_sample_is_fresh =
            dt <= kYawEndpointMaxSampleIntervalSec;
        const int raw_motion_direction =
            endpoint_observed_velocity > kYawEndpointResumeVelocityThreshold
                ? 1
                : (endpoint_observed_velocity <
                           -kYawEndpointResumeVelocityThreshold
                       ? -1
                       : 0);

        // 强反向样本可以直接证明折返；若端点先停顿，则在第一个接近
        // 静止的样本就清除旧预瞄，连续两帧后进入端点保持状态。
        const bool raw_direction_reversed = endpoint_sample_is_fresh &&
            raw_motion_direction != 0 && last_yaw_motion_direction != 0 &&
            raw_motion_direction != last_yaw_motion_direction;
        const bool filtered_direction_reversed =
            previous_velocity * candidate_filtered_velocity < 0.0 &&
            std::abs(previous_velocity) > kYawReversalVelocityThreshold &&
            std::abs(candidate_filtered_velocity) > kYawReversalVelocityThreshold;
        const bool endpoint_deceleration_candidate =
            endpoint_sample_is_fresh &&
            std::abs(previous_velocity) >=
                kYawEndpointDecelerationMinVelocity &&
            endpoint_observed_velocity * previous_velocity >= 0.0 &&
            std::abs(endpoint_observed_velocity) <=
                std::abs(previous_velocity) * kYawEndpointDecelerationRatio;

        if (endpoint_sample_is_fresh &&
            std::abs(endpoint_observed_velocity) <=
                kYawEndpointStationaryVelocityThreshold)
        {
          ++yaw_endpoint_stationary_frames;
        }
        else
        {
          yaw_endpoint_stationary_frames = 0;
        }

        const bool endpoint_stop_started =
            yaw_endpoint_stationary_frames == 1;
        const bool endpoint_stop_confirmed =
            yaw_endpoint_stationary_frames >= kYawEndpointHoldConfirmFrames;
        const bool endpoint_stop_confirmed_this_frame =
            yaw_endpoint_stationary_frames ==
                kYawEndpointHoldConfirmFrames;
        const bool endpoint_reverse_motion_resumed =
            yaw_endpoint_hold_active && raw_motion_direction != 0;

        if (endpoint_stop_confirmed)
        {
          yaw_endpoint_hold_active = true;
        }
        if (endpoint_reverse_motion_resumed)
        {
          yaw_endpoint_hold_active = false;
          yaw_endpoint_stationary_frames = 0;
          laser_shot_at_current_endpoint = false;
        }

        yaw_reversed_this_frame =
            raw_direction_reversed || filtered_direction_reversed;
        yaw_endpoint_transition_this_frame =
            endpoint_deceleration_candidate || endpoint_stop_started ||
            endpoint_stop_confirmed_this_frame ||
            endpoint_reverse_motion_resumed || yaw_reversed_this_frame;

        // 端点保持期间不带任何旧预瞄。反向起步的第一帧同样归零，
        // 从下一帧开始按新方向平滑恢复预瞄，避免在端点猛甩。
        if (yaw_endpoint_hold_active || endpoint_deceleration_candidate ||
            raw_direction_reversed || filtered_direction_reversed ||
            endpoint_reverse_motion_resumed)
        {
          filtered_target_yaw_velocity = 0.0;
        }
        else
        {
          filtered_target_yaw_velocity = candidate_filtered_velocity;
        }

        if (!yaw_endpoint_transition_this_frame &&
            !yaw_endpoint_hold_active)
        {
          const double desired_lead_position = clampDouble(
              std::abs(filtered_target_yaw_velocity) >= kYawLeadVelocityDeadband
                  ? filtered_target_yaw_velocity * kYawLeadTimeSec
                  : 0.0,
              -kYawMaxLeadPosition, kYawMaxLeadPosition);
          filtered_yaw_lead_position =
              kYawLeadFilterOldWeight * filtered_yaw_lead_position +
              (1.0 - kYawLeadFilterOldWeight) * desired_lead_position;
          yaw_lead_position = filtered_yaw_lead_position;
          predicted_yaw_offset = filtered_yaw_offset +
              yaw_lead_position * kYawPixelsPerMotorPosition;
        }
        else
        {
          filtered_yaw_lead_position = 0.0;
        }

        if (raw_motion_direction != 0 &&
            (raw_direction_reversed || endpoint_reverse_motion_resumed))
        {
          last_yaw_motion_direction = raw_motion_direction;
        }
        else if (!yaw_endpoint_hold_active &&
                 std::abs(filtered_target_yaw_velocity) >
                     kYawReversalVelocityThreshold)
        {
          last_yaw_motion_direction =
              filtered_target_yaw_velocity > 0.0 ? 1 : -1;
        }
      }
    }

    previous_target_yaw = estimated_target_yaw;
    previous_raw_target_yaw = raw_target_yaw;
    previous_target_yaw_time = now;
    has_target_yaw_sample = true;

    if (yaw_endpoint_transition_this_frame)
    {
      last_yaw_endpoint_transition_time = now;
      yaw_endpoint_centered_frames = 0;
    }

    // Pitch 保持原有控制方式；Yaw 叠加超前位置，使恒速靶瞄准其当前位置而非
    // 视觉延迟对应的过去位置。
    const int desired_pitch = limitGimbalMotor0Position(curYuntai_feedback_data.Position_0 + static_cast<int>(filtered_pitch_offset/2.7));
    const bool endpoint_fast_recenter =
        yaw_endpoint_transition_this_frame || yaw_endpoint_hold_active;
    const double yaw_control_offset = endpoint_fast_recenter
        ? static_cast<double>(temp_msg.data[0])
        : filtered_yaw_offset;
    const int desired_yaw = curYuntai_feedback_data.Position_1 +
        static_cast<int>(yaw_control_offset / kYawPixelsPerMotorPosition +
                         yaw_lead_position);
    last_pitch_command = applySlewLimit(desired_pitch, last_pitch_command, 85);//位置变化最大限制幅度
    if (endpoint_fast_recenter)
    {
      // 端点时每帧都根据当前原始中心偏差一步到位，不使用跟踪步长限制。
      last_yaw_command = desired_yaw;
    }
    else
    {
      last_yaw_command = applySlewLimit(
          desired_yaw, last_yaw_command, kYawTrackingCommandMaxStep);
    }

    moveBaseControl.Position_0 = last_pitch_command;
    moveBaseControl.Position_1 = last_yaw_command;

    moveBaseControl.Speed_0 = CaremaSpeedControl(moveBaseControl.Position_0, curYuntai_feedback_data.Position_0, GimbalAxis::Pitch);
    const double yaw_pid_speed = CaremaSpeedControl(
        moveBaseControl.Position_1, curYuntai_feedback_data.Position_1,
        GimbalAxis::Yaw);
    const double yaw_velocity_feedforward =
        !yaw_endpoint_transition_this_frame && !yaw_endpoint_hold_active &&
        std::abs(filtered_target_yaw_velocity) >= kYawLeadVelocityDeadband
            ? std::abs(filtered_target_yaw_velocity) *
                  kYawVelocityFeedforwardGain
            : 0.0;
    double yaw_command_speed = yaw_pid_speed + yaw_velocity_feedforward;
    if (endpoint_fast_recenter &&
        std::abs(static_cast<double>(temp_msg.data[0])) >=
            kYawFireWindowPixels)
    {
      yaw_command_speed = std::max(
          yaw_command_speed, kYawEndpointRecenterMinSpeed);
    }
    moveBaseControl.Speed_1 = static_cast<int>(clampDouble(
        yaw_command_speed, 0.0, kYawTrackingMaxMotorSpeed));

    ROS_INFO_THROTTLE(0.5,
                      "Yaw跟踪: 当前偏差=%.2f 预测偏差=%.2f 估计速度=%.2f 超前位置=%.2f 速度前馈=%.2f 本帧折返=%d 端点保持=%d",
                      filtered_yaw_offset, predicted_yaw_offset,
                      filtered_target_yaw_velocity, yaw_lead_position,
                      yaw_velocity_feedforward,
                      yaw_reversed_this_frame, yaw_endpoint_hold_active);

    // 端点模式只用当前原始视觉中心判断开火。运动段继续跟踪和预瞄，
    // 但被 kFireOnlyAtYawEndpoint 硬性禁止发射。
    const double yaw_fire_error =
        (yaw_endpoint_transition_this_frame || yaw_endpoint_hold_active)
            ? static_cast<double>(temp_msg.data[0])
            : predicted_yaw_offset;

    const bool endpoint_settle_elapsed =
        last_yaw_endpoint_transition_time.isZero() ||
        (now - last_yaw_endpoint_transition_time).toSec() >=
            kYawEndpointFireSettleTimeSec;
    if (yaw_endpoint_hold_active && endpoint_settle_elapsed &&
        std::abs(static_cast<double>(temp_msg.data[0])) <
            kYawFireWindowPixels)
    {
      ++yaw_endpoint_centered_frames;
    }
    else if (yaw_endpoint_hold_active)
    {
      yaw_endpoint_centered_frames = 0;
    }

    const bool endpoint_fire_ready = yaw_endpoint_hold_active &&
        endpoint_settle_elapsed &&
        yaw_endpoint_centered_frames >=
            kYawEndpointCenteredFramesBeforeFire;
    const bool endpoint_only_fire_gate = !kFireOnlyAtYawEndpoint ||
        (endpoint_fire_ready && !laser_shot_at_current_endpoint);
    const bool within_fire_window =
        endpoint_only_fire_gate &&
        std::abs(filtered_pitch_offset) < kPitchFireWindowPixels &&
        std::abs(yaw_fire_error) < kYawFireWindowPixels;
    const bool shot_interval_elapsed = last_laser_shot_time.isZero() ||
        (now - last_laser_shot_time).toSec() >= kLaserShotMinIntervalSec;
    if (within_fire_window && shot_interval_elapsed)
    {
      std_msgs::UInt8 shotdata;
      shotdata.data = 1;
      pub_LaserShot_Command.publish(shotdata);
      last_laser_shot_time = now;
      laser_shot_at_current_endpoint = true;

      laser_shot_while_target_visible = true;
      post_shot_scan_center_yaw = clamp(
          desired_yaw, kFullScanYawMin, kFullScanYawMax);

      // dzjudgment 收到值 1 才真正击发。保留原来的尾随 0 语义，但不再让视觉
      // 追踪因 80 ms sleep 而阻塞。
      shotdata.data = 0;
      pub_LaserShot_Command.publish(shotdata);
    }
  }
  else if (temp_msg.data[2] == 0)
  { // 未检测到目标
    if (laser_shot_while_target_visible)
    {
      post_shot_focused_scan_active = true;
      laser_shot_while_target_visible = false;
      int focused_scan_yaw_min = kFullScanYawMin;
      int focused_scan_yaw_max = kFullScanYawMax;
      calculateFocusedScanBounds(post_shot_scan_center_yaw,
                                 focused_scan_yaw_min,
                                 focused_scan_yaw_max);
      ROS_INFO("激光发射后目标丢失：启用局部快扫，中心=%d，Yaw范围=[%d, %d]",
               post_shot_scan_center_yaw,
               focused_scan_yaw_min, focused_scan_yaw_max);
    }

    find_center = false;
    has_target_yaw_sample = false;
    filtered_target_yaw_velocity = 0.0;
    filtered_yaw_lead_position = 0.0;
    yaw_endpoint_hold_active = false;
    yaw_endpoint_stationary_frames = 0;
    last_yaw_motion_direction = 0;
    last_yaw_endpoint_transition_time = ros::Time();
    yaw_endpoint_centered_frames = 0;
    laser_shot_at_current_endpoint = false;
    has_prev_pitch_offset = false;
    has_prev_yaw_offset = false;
    
    std_msgs::UInt8 shotdata;
    shotdata.data =0;
    pub_LaserShot_Command.publish(shotdata);
  }
}

double turn_on_robot::CaremaSpeedControl(int target_pose,int current_pose,GimbalAxis axis){
    struct PidState
    {
        double prev_error = 0.0;
        double integral = 0.0;
        double filtered_derivative = 0.0;
        ros::Time last_update;
        bool has_update = false;
    };

    static PidState pitch_state;
    static PidState yaw_state;

    const bool is_pitch = axis == GimbalAxis::Pitch;
    PidState &state = is_pitch ? pitch_state : yaw_state;

    struct Gains
    {
        double kp;
        double ki;
        double kd;
        double max_speed;
        double offset;
        double deadband;
        double derivative_limit;
    };

    const Gains gains = is_pitch ? Gains{200,11.2,11.8,9000.0,4.5,1.0,600.0}
                                 : Gains{255,11.2,9.5,kYawTrackingMaxMotorSpeed,8,1.0,500.0};//前面pitch 后面yaw 分别对应kp,ki,kd,max_speed,offset,deadband,derivative_limit

    // 计算误差
    double error = static_cast<double>(target_pose - current_pose);

    const ros::Time now = ros::Time::now();
    double sampling_time = 0.01;
    const bool reset_derivative = !state.has_update ||
        (now - state.last_update).toSec() <= 0.0 ||
        (now - state.last_update).toSec() > 0.2;
    if (!reset_derivative)
    {
        sampling_time = clamp((now - state.last_update).toSec(), 0.005, 0.1);
    }
    state.last_update = now;
    state.has_update = true;

    if(std::abs(error) <= gains.deadband)
    {
        state.integral = 0.0;
        state.filtered_derivative = 0.0;
        state.prev_error = error;
        return 0.0;
    }

    // 更新积分项（带限幅）
    if(std::abs(error)>25)
    {state.integral=0.0;}
    else
    {state.integral += error * sampling_time;}
    if (gains.ki != 0.0) {
        state.integral = clamp(state.integral, -gains.max_speed / (2.0 * gains.ki), gains.max_speed / (2.0 * gains.ki));
    } else {
        state.integral = 0.0;  // 如果ki=0，清零积分
    }

    //微分项更新并滤波
    double derivative = reset_derivative ? 0.0 :
        (error - state.prev_error) / sampling_time;
    derivative = clamp(derivative, -gains.derivative_limit, gains.derivative_limit);
    state.filtered_derivative = 0.6 * state.filtered_derivative +0.4  * derivative;

    // PID 控制计算
    double speed = gains.kp * error + gains.ki * state.integral   + gains.kd * state.filtered_derivative;

    //补偿
    if(error>0)
    {speed+=gains.offset;}
    else if(error<0)
    {speed-=gains.offset;}
    // 对速度取绝对值
    speed = std::abs(speed);

    // 使用 std::clamp 限幅
    speed = clamp(speed, 0.0, gains.max_speed);

    // 更新前一次误差
    state.prev_error = error;
    ROS_DEBUG_THROTTLE(0.5,
                       "axis:%s error:%.2f speed:%.2f integral:%.2f derivative:%.2f",
                       is_pitch ? "pitch" : "yaw", error, speed,
                       state.integral, state.filtered_derivative);
    return speed;
}

void turn_on_robot::sendCarInfoKernel()
{
  unsigned char buf[23] = {0};
  buf[0] = 0xa5; // hdr1
  buf[1] = 0x5a; // hdr2
  buf[2] = 20;   // len - 数据长度 22 - 3 = 19字节
  // 角度 TargetAngle 转换成整形后放入 buf[3] 和 buf[4]
  int16_t angle = static_cast<int16_t>(moveBaseControl.TargetAngle * 10); // 假设角度以0.1度为单位
  buf[3] = (angle >> 8) & 0xFF;                                           // 高字节
  buf[4] = angle & 0xFF;                                                  // 低字节
  // 速度符号 SpeedDirection 放入 buf[5]
  buf[5] = static_cast<unsigned char>(moveBaseControl.SpeedDirection);
  // printf("moveBaseControl.TargetSpeed = %d\n",moveBaseControl.TargetSpeed);
  // 目标速度 TargetSpeed 放入 buf[6]（占1字节）
  buf[6] = static_cast<unsigned char>(moveBaseControl.TargetSpeed);

  //备用
  buf[7] = 0x00;
  // 云台电机0位置 Position_O 放入 buf[8] 和 buf[9]
  buf[8] = (moveBaseControl.Position_0 >> 8) & 0xFF; // 高字节
  buf[9] = moveBaseControl.Position_0 & 0xFF;        // 低字节
  // 云台电机1位置 Position_1 放入 buf[10] 和 buf[11]
  buf[10] = (moveBaseControl.Position_1 >> 8) & 0xFF; // 高字节
  buf[11] = moveBaseControl.Position_1 & 0xFF;        // 低字节
  // 云台电机0速度 Speed_0 放入 buf[12] 和 buf[13]
  buf[12] = (moveBaseControl.Speed_0 >> 8) & 0xFF; // 高字节
  buf[13] = moveBaseControl.Speed_0 & 0xFF;        // 低字节
  // 云台电机1速度 Speed_1 放入 buf[14] 和 buf[15]
  buf[14] = (moveBaseControl.Speed_1 >> 8) & 0xFF; // 高字节
  buf[15] = moveBaseControl.Speed_1 & 0xFF;        // 低字节
  // 云台电机0时间 Time_0 放入 buf[16]
  buf[16] = static_cast<unsigned char>(moveBaseControl.Time_0);
  // 云台电机1时间 Time_1 放入 buf[17]
  buf[17] = static_cast<unsigned char>(moveBaseControl.Time_1);
  // 功能模式
  buf[18] = static_cast<unsigned char>(moveBaseControl.Fun);
  // 使用ID
  buf[19] = static_cast<unsigned char>(moveBaseControl.Orin_ID);
  // 重置ID
  buf[20] = static_cast<unsigned char>(moveBaseControl.Set_ID_Num);
  // std::cout << " moveBaseControl.Position_0  =" << moveBaseControl.Position_0 << " moveBaseControl.Position_1 = " << moveBaseControl.Position_1 << std::endl;

  // 预留位 Reservel
  buf[21] = static_cast<unsigned char>(moveBaseControl.VoiceSwitch);
  // 校验和
  unsigned char sum = 0;
  for (int i = 2; i < 22; ++i) // 从 buf[2] 到 buf[18] 计算校验和
    sum += buf[i];
  buf[22] = sum;

  // for (int j = 0; j < 24; j++)
  // {
  //   printf("buf[%d] = %d ,", j, buf[j]);
  // }
  // printf("\n");
  // 通过串口发送数据
  try
  {
    Stm32_Serial.write(buf, 23); // 发送22字节的数据
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR_STREAM("Unable to send data through serial port"); // 如果发送数据失败，打印错误信息
  }
}

/**************************************
Date: January 28, 2021
Function: Publish the IMU data topic
功能: 发布IMU数据话题
***************************************/
void turn_on_robot::Publish_ImuSensor()
{
  // auto Imu_Data_Pub = std::make_unique<sensor_msgs::Imu>();

  sensor_msgs::Imu Imu_Data_Pub; // Instantiate IMU topic data //实例化IMU话题数据
  Imu_Data_Pub.header.stamp = ros::Time::now();
  Imu_Data_Pub.header.frame_id = gyro_frame_id;       // IMU corresponds to TF coordinates, which is required to use the robot_pose_ekf feature pack                                                // IMU对应TF坐标，使用robot_pose_ekf功能包需要设置此项
  Imu_Data_Pub.orientation.x = Mpu6050.orientation.x; // A quaternion represents a three-axis attitude //四元数表达三轴姿态
  Imu_Data_Pub.orientation.y = Mpu6050.orientation.y;
  Imu_Data_Pub.orientation.z = Mpu6050.orientation.z;
  Imu_Data_Pub.orientation.w = Mpu6050.orientation.w;

  Imu_Data_Pub.angular_velocity.x = Mpu6050.angular_velocity.x; // Triaxial angular velocity //三轴角速度
  Imu_Data_Pub.angular_velocity.y = Mpu6050.angular_velocity.y;
  Imu_Data_Pub.angular_velocity.z = Mpu6050.angular_velocity.z;

  Imu_Data_Pub.linear_acceleration.x = Mpu6050.linear_acceleration.x; // Triaxial acceleration //三轴线性加速度
  Imu_Data_Pub.linear_acceleration.y = Mpu6050.linear_acceleration.y;
  Imu_Data_Pub.linear_acceleration.z = Mpu6050.linear_acceleration.z;
  // printf("1---- x=%.2f,y=%.2f,z=%.2f,\n",Imu_Data_Pub.linear_acceleration.x,Imu_Data_Pub.linear_acceleration.y,Imu_Data_Pub.linear_acceleration.z);

  if (montion_flag)
  {
    Imu_Data_Pub.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e-6;

    Imu_Data_Pub.linear_acceleration_covariance[0] = 1e-6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.linear_acceleration_covariance[4] = 1e-6;
    Imu_Data_Pub.linear_acceleration_covariance[8] = 1e6;

    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e-6;
  }
  else
  {
    Imu_Data_Pub.orientation_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.orientation_covariance[4] = 1e6;
    Imu_Data_Pub.orientation_covariance[8] = 1e6;

    Imu_Data_Pub.angular_velocity_covariance[0] = 1e6; // Triaxial angular velocity covariance matrix //三轴角速度协方差矩阵
    Imu_Data_Pub.angular_velocity_covariance[4] = 1e6;
    Imu_Data_Pub.angular_velocity_covariance[8] = 1e6;
    Imu_Data_Pub.linear_acceleration_covariance[0] = 1e6; // Three-axis attitude covariance matrix //三轴姿态协方差矩阵
    Imu_Data_Pub.linear_acceleration_covariance[4] = 1e6;
    Imu_Data_Pub.linear_acceleration_covariance[8] = 1e6;
  }

  imu_publisher.publish(Imu_Data_Pub); // Pub IMU topic //发布IMU话题
  std_msgs::UInt8 imu_msg_valid;
  imu_msg_valid.data = 1;
  pub_imu_msg_valid.publish(imu_msg_valid);

  if (imu_flag == 1)
  {

    imu_buff.push_back(Imu_Data_Pub);
    imu_correct = Imu_Data_Pub;
    Average_filtering();
  }
  else
  {

    imu_state.push_back(Imu_Data_Pub);
    Cal_state_error();
  }
}

double getYaw(geometry_msgs::PoseStamped pose)
{
  return tf2::getYaw(pose.pose.orientation);
}

/**************************************
Date: January 28, 2021
Function: Publish the odometer topic, Contains position, attitude, triaxial velocity, angular velocity about triaxial, TF parent-child coordinates, and covariance matrix
功能: 发布里程计话题，包含位置、姿态、三轴速度、绕三轴角速度、TF父子坐标、协方差矩阵
***************************************/
void turn_on_robot::Publish_Odom()
{
  // Convert the Z-axis rotation Angle into a quaternion for expression
  // 把Z轴转角转换为四元数进行表达
  // geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

  if ((Robot_Pos.X == NAN) || (Robot_Pos.Y == NAN) || (Robot_Pos.Z == NAN))
  {
    printf("data no effect!\n");
    return;
  }

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Robot_Pos.Z);

  nav_msgs::Odometry odom; // Instance the odometer topic data //实例化里程计话题数据
  odom.header.stamp = ros::Time::now();
  odom.header.frame_id = odom_frame_id;    // Odometer TF parent coordinates //里程计TF父坐标
  odom.pose.pose.position.x = Robot_Pos.X; // Position //位置
  odom.pose.pose.position.y = Robot_Pos.Y;
  odom.pose.pose.position.z = 0.2;
  odom.pose.pose.orientation = odom_quat; // Posture, Quaternion converted by Z-axis rotation //姿态，通过Z轴转角转换的四元数
  // odom.pose.pose.orientation.x = Mpu6050.orientation.x;
  // odom.pose.pose.orientation.y = Mpu6050.orientation.y;
  // odom.pose.pose.orientation.z = Mpu6050.orientation.z;
  // odom.pose.pose.orientation.w = Mpu6050.orientation.w;

  odom.child_frame_id = robot_frame_id;                        // Odometer TF subcoordinates //里程计TF子坐标
  odom.twist.twist.linear.x = linear_Speed;                     // Speed in the X direction //X方向速度
  odom.twist.twist.linear.y = 0.0;                             // Speed in the Y direction //Y方向速度
  odom.twist.twist.angular.z = ThetaSpeed; // Angular velocity around the Z axis //绕Z轴角速度

  // There are two types of this matrix, which are used when the robot is at rest and when it is moving.Extended Kalman Filtering officially provides 2 matrices for the robot_pose_ekf feature pack
  // 这个矩阵有两种，分别在机器人静止和运动的时候使用。扩展卡尔曼滤波官方提供的2个矩阵，用于robot_pose_ekf功能包
  if (Robot_Vel.Left == 0 && Robot_Vel.Right == 0)
    // If the velocity is zero, it means that the error of the encoder will be relatively small, and the data of the encoder will be considered more reliable
    // 如果velocity是零，说明编码器的误差会比较小，认为编码器数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance2, sizeof(odom_pose_covariance2)),
        memcpy(&odom.twist.covariance, odom_twist_covariance2, sizeof(odom_twist_covariance2));
  else
    // If the velocity of the trolley is non-zero, considering the sliding error that may be brought by the encoder in motion, the data of IMU is considered to be more reliable
    // 如果小车velocity非零，考虑到运动中编码器可能带来的滑动误差，认为imu的数据更可靠
    memcpy(&odom.pose.covariance, odom_pose_covariance, sizeof(odom_pose_covariance)),
        memcpy(&odom.twist.covariance, odom_twist_covariance, sizeof(odom_twist_covariance));
  // printf("odom_publisher--> x=%.2f,y=%.2f,z=%.2f\n",odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.angular.z );
  odom_publisher.publish(std::move(odom)); // Pub odometer topic //发布里程计话题
  std_msgs::UInt8 odom_msg_valid;
  odom_msg_valid.data = 1;
  pub_odom_msg_valid.publish(odom_msg_valid);

  // publish the transform over tf add by jiaoyang 2022-12-27 start
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.header.frame_id = odom_frame_id;
  odom_trans.child_frame_id = robot_frame_id;

  odom_trans.transform.translation.x = Robot_Pos.X;
  odom_trans.transform.translation.y = Robot_Pos.Y;
  odom_trans.transform.translation.z = 0;
  odom_trans.transform.rotation = odom_quat;
  // odom_trans.transform.rotation.x = Mpu6050.orientation.x;
  // odom_trans.transform.rotation.y = Mpu6050.orientation.y;
  // odom_trans.transform.rotation.z = Mpu6050.orientation.z;
  // odom_trans.transform.rotation.w = Mpu6050.orientation.w;

  // add by jiaoyang 2022-12-27 end
}

/**************************************
Date: January 28, 2021
Function: Publish voltage-related information
功能: 发布电压相关信息
***************************************/
void turn_on_robot::Publish_Voltage()
{
  std_msgs::Float32 voltage_msgs;
  voltage_msgs.data = Power_voltage;       // The power supply voltage is obtained //电源供电的电压获取
  voltage_publisher.publish(voltage_msgs); // Post the power supply voltage topic unit: V, volt //发布电源电压话题单位：V、伏特
  // printf("voltage_publisher->publish=%.5f\n",Power_voltage);
}

/**************************************
Date: February 16, 2021
Function: Publish Battery_Percentage information
功能: 发布电量百分比信息
***************************************/
double v;
bool flag = false;
int N = 10;
int Index = 0;
std::vector<double> xx(N, 0);
void turn_on_robot::Publish_Battery_Percentage()
{
  v = Power_voltage;
  // v = 13;

  float x;
  // float a, b, c, d, x, p, q, s1, s2, s3;
  // a = 0.33301984;
  // b = -1.31139425;
  // c = 1.87580217;
  // // d = 10.85200718;
  // d = 11.1;
  // d -= v;
  // // printf("Power_voltage =%.2f,d=%.2f\n",Power_voltage,d);
  // p = (3 * a * c - b * b) / (3 * a * a);
  // q = (27 * a * a * d - 9 * a * b * c + 2 * b * b * b) / (27 * a * a * a);
  // s1 = -b / (3 * a);
  // if (-q / 2 + sqrt(q * q / 4 + p * p * p / 27) > 0)
  // {
  //   s2 = pow((-q / 2 + sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // else
  // {
  //   s2 = -pow(-(-q / 2 + sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // if (-q / 2 - sqrt(q * q / 4 + p * p * p / 27) > 0)
  // {
  //   s3 = pow((-q / 2 - sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // else
  // {
  //   s3 = -pow(-(-q / 2 - sqrt(q * q / 4 + p * p * p / 27)), 1.0 / 3);
  // }
  // x = s1 + s2 + s3;
  // x = (x + 0.6468011925188) / 2.592234808167281;
  // if (x > 1.0 && x < 0.0)
  // {
  //   if (v > 11)
  //     x = 0.10;
  //   else
  //     x = 0.01;
  // }

  if (v >= Power_max)
  {
    x = 1;
  }
  else if (v > Power_min)
  {
    x = (v - Power_min) / (Power_max - Power_min);
  }
  else if (v <= Power_min)
  {
    x = 0;
  }

  xx[Index] = x;
  ++Index;
  Index %= N;

  // std::time_t nowtime = std::time(NULL) - starttime; // 获取时间戳
  // out_txt_file.open("power_voltage.txt", ios::out | ios::trunc);
  // out_txt_file << fixed;
  // out_txt_file << nowtime;
  // out_txt_file << " ";
  // out_txt_file << setprecision(4) << v;
  // out_txt_file << endl;
  // std::cout << "nowtime= " << nowtime << std::endl;
  if (count_A < 100)
    count_A++;

  float temp = 0;
  for (int i = 0; i < N; ++i)
    temp += xx[i];

  double cur_Battery_Percentage;
  cur_Battery_Percentage = temp / N; // 设置数据字段
  if (cur_Battery_Percentage > 1)
    cur_Battery_Percentage = 1;
  // printf("cur_Battery_Percentage=%.2f,--count_A=%d\n",cur_Battery_Percentage,count_A);
  // 保证电量百分比稳定下降，上升时只有连续比历史值高100次，才上升
  if (count_A > 30)
  {
    if (cur_Battery_Percentage > last_Battery_Percentage)
    {
      count_B++;
      if (count_B > 100)
      {
        last_Battery_Percentage = cur_Battery_Percentage;
        count_B = 0;
        count_C = 0;
      }
    }
    else
      count_B = 0;

    if (count_B > 200)
      count_B = 0;
    // 下降时低10次就下降
    if (cur_Battery_Percentage < last_Battery_Percentage)
    {
      count_C++;
      if (count_C > 10)
      {
        last_Battery_Percentage = cur_Battery_Percentage;
        count_B = 0;
        count_C = 0;
      }
    }
    else
      count_C = 0;
  }
  else if (last_Battery_Percentage < cur_Battery_Percentage)
  {
    last_Battery_Percentage = cur_Battery_Percentage;
  }

  Battery_Percentage_msgs.data = last_Battery_Percentage;

  Battery_Percentage_pub.publish(Battery_Percentage_msgs); // 发布电池电量百分比
  // printf("last_Battery_Percentage=%f\n",last_Battery_Percentage);

  // std::cout << "电压" << v << "==>"
  //           << "电量百分比： " << Battery_Percentage_msgs.data << std::endl;
}

/**************************************
Date: January 28, 2021
Function: Serial port communication check function, packet n has a byte, the NTH -1 byte is the check bit, the NTH byte bit frame end.Bit XOR results from byte 1 to byte n-2 are compared with byte n-1, which is a BCC check
Input parameter: Count_Number: Check the first few bytes of the packet
功能: 串口通讯校验函数，数据包n有个字节，第n-1个字节为校验位，第n个字节位帧尾。第1个字节到第n-2个字节数据按位异或的结果与第n-1个字节对比，即为BCC校验
输入参数： Count_Number：数据包前几个字节加入校验   mode：对发送数据还是接收数据进行校验
***************************************/

unsigned char turn_on_robot::Check_Sum(unsigned char Count_Number, unsigned char mode)
{
  unsigned char check_sum = 0, k;

  if (mode == 0) // Receive data mode //接收数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.rx[k]; // By bit or by bit //按位异或
    }
  }
  if (mode == 1) // Send data mode //发送数据模式
  {
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.tx[k]; // By bit or by bit //按位异或
    }
  }
  return check_sum; // Returns the bitwise XOR result //返回按位异或结果
}

/**************************************
Date: November 18, 2021
Function: Read and verify the data sent by the lower computer frame by frame through the serial port, and then convert the data into international units
功能: 通过串口读取并逐帧校验下位机发送过来的数据，然后数据转换为国际单位
***************************************/
bool turn_on_robot::Get_Sensor_Data_New()
{
  // printf("Get_Sensor_Data_New success 1 \n");

  uint8_t head_Receive_Data[1] = {0};      // 临时变量，保存下位机数据
  Stm32_Serial.read(head_Receive_Data, 1); // 通过串口读取下位机发送的数据

  if (head_Receive_Data[0] != 0x7b)
    return false;

  uint8_t len_Receive_Data[1] = {0};      // 临时变量，保存下位机数据
  Stm32_Serial.read(len_Receive_Data, 1); // 通过串口读取下位机发送的数据

  if (len_Receive_Data[0] != 0x1f)
    return false;

  short transition_16 = 0, j = 0;
  uint8_t Receive_Data_Pr[RECEIVE_DATA_SIZE - 2] = {0};        // 临时变量，保存下位机数据
  Stm32_Serial.read(Receive_Data_Pr, sizeof(Receive_Data_Pr)); // 通过串口读取下位机发送的数据

  Receive_Data.rx[0] = head_Receive_Data[0];
  Receive_Data.rx[1] = len_Receive_Data[0];
  for (j = 0; j < sizeof(Receive_Data_Pr); j++)
  {
    Receive_Data.rx[j + 2] = Receive_Data_Pr[j];
  }

  // for (int kk = 0; kk < sizeof(Receive_Data_Pr); kk++)
  // {
  //   printf("[%d]=%02x,", kk, Receive_Data_Pr[kk]);
  // }
  // printf(" \n");
  // for (int kk = 0; kk < sizeof(Receive_Data.rx); kk++)
  // {
  //   printf("[%d]=%02x,", kk, Receive_Data.rx[kk]);
  // }
  // printf(" -----------------------------\n");

  // 校验和验证
  // if (Receive_Data.rx[33] != Check_Sum(33, READ_DATA_CHECK))
  // {
  //   printf("Check_Sum false  ----------\n");
  //   return false; // 校验和错误
  // }

  // 开始解析各个数据字段

  // 解析左轮和右轮速度
  Robot_Vel.Left = Odom_Trans(Receive_Data.rx[2], Receive_Data.rx[3]);  // 左轮速度
  Robot_Vel.Right = Odom_Trans(Receive_Data.rx[4], Receive_Data.rx[5]); // 右轮速度

  // 解析电池电压
  transition_16 = 0;
  transition_16 |= Receive_Data.rx[6] << 8;
  transition_16 |= Receive_Data.rx[7];
  Power_voltage = transition_16 / 1000.0; // 单位转换为伏特

  // 解析IMU加速度数据
  Mpu6050_Data.accele_x_data = IMU_Trans(Receive_Data.rx[8], Receive_Data.rx[9]);   // X轴加速度
  Mpu6050_Data.accele_y_data = IMU_Trans(Receive_Data.rx[10], Receive_Data.rx[11]); // Y轴加速度
  Mpu6050_Data.accele_z_data = IMU_Trans(Receive_Data.rx[12], Receive_Data.rx[13]); // Z轴加速度

  // 解析陀螺仪数据
  Mpu6050_Data.gyros_x_data = IMU_Trans(Receive_Data.rx[14], Receive_Data.rx[15]); // X轴角速度
  Mpu6050_Data.gyros_y_data = IMU_Trans(Receive_Data.rx[16], Receive_Data.rx[17]); // Y轴角速度
  Mpu6050_Data.gyros_z_data = IMU_Trans(Receive_Data.rx[18], Receive_Data.rx[19]); // Z轴角速度

  // 加速度和角速度的单位转换
  Mpu6050.linear_acceleration.x = Mpu6050_Data.accele_x_data * 24 / 32768.0f * 9.8;
  Mpu6050.linear_acceleration.y = Mpu6050_Data.accele_y_data * 24 / 32768.0f* 9.8;
  Mpu6050.linear_acceleration.z = Mpu6050_Data.accele_z_data * 24 / 32768.0f* 9.8;

  Mpu6050.angular_velocity.x = Mpu6050_Data.gyros_x_data * 1000 / 32768.0f / 57.3;
  Mpu6050.angular_velocity.y = Mpu6050_Data.gyros_y_data * 1000 / 32768.0f / 57.3;
  Mpu6050.angular_velocity.z = Mpu6050_Data.gyros_z_data * 1000 / 32768.0f / 57.3;

  // 解析位置、速度和负载数据
  // 位置 1
  curYuntai_feedback_data.Position_0 = Odom_Trans(Receive_Data.rx[20], Receive_Data.rx[21]);
  // 速度 1
  curYuntai_feedback_data.Speed_0 = Odom_Trans(Receive_Data.rx[22], Receive_Data.rx[23]);
  // 负载 1
  curYuntai_feedback_data.Load_0 = Odom_Trans(Receive_Data.rx[24], Receive_Data.rx[25]);
  // printf("Position 1: %d, Speed 1: %d, Load 1: %d\n", curYuntai_feedback_data.Position_0, curYuntai_feedback_data.Speed_0, curYuntai_feedback_data.Load_0);

  // 位置 2
  curYuntai_feedback_data.Position_1 = Odom_Trans(Receive_Data.rx[26], Receive_Data.rx[27]);
  // 速度 2
  curYuntai_feedback_data.Speed_1 = Odom_Trans(Receive_Data.rx[28], Receive_Data.rx[29]);
  // 负载 2
  curYuntai_feedback_data.Load_1 = Odom_Trans(Receive_Data.rx[30], Receive_Data.rx[31]);
  // printf("Position 2: %d, Speed 2: %d, Load 2: %d\n", curYuntai_feedback_data.Position_1, curYuntai_feedback_data.Speed_1, curYuntai_feedback_data.Load_1);

  // 处理预留位
  // Receive_Data.Flag_Reserved = Receive_Data.rx[32]; // 预留位

  // 最后返回true，表示数据解析成功
  return true;
}

 
int main(int argc, char **argv)
{
  ros::init(argc, argv, "dzactuator");
  turn_on_robot Robot_Control;
  Robot_Control.Control();
  return 0;
}

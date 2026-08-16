#include "judgment.h"

#include <algorithm>
#include <array>

dzjudgment::dzjudgment(ros::NodeHandle handle)
{
  m_baudrate = 115200;
  m_serialport = "/dev/ttyUSB1";

  handle.param("mcubaudrate", m_baudrate, m_baudrate);
  handle.param("mcuserialport", m_serialport, std::string("/dev/ttyUSB1"));

  // paramters
  std::cout << FRED("Copyright©2016-2020 duzhong robot. All rights reserved ") << std::endl;
  std::cout << FYEL("*****dzjudgment:parameters*******************") << std::endl;
  std::cout << FGRN("judgment_baudrate: ") << m_baudrate << std::endl;
  std::cout << FGRN("judgment_serialport: ") << m_serialport << std::endl;
  std::cout << FYEL("*****dzjudgment:parameters end***************") << std::endl;

  int retryCount = 3;
  while (retryCount > 0)
  {
    try
    {
      std::cout << "[dzjudgment-->]" << "Serial initialize start!" << std::endl;
      ser.setPort(m_serialport.c_str());
      ser.setBaudrate(m_baudrate);
      serial::Timeout to = serial::Timeout::simpleTimeout(30);
      ser.setTimeout(to);
      ser.open();
      break;
    }
    catch (serial::IOException &e)
    {
      std::cout << "[dzjudgment-->]" << "Unable to open port! Retrying..." << std::endl;
      retryCount--;
      ros::Duration(1).sleep();
    }
  }

  if (ser.isOpen())
  {
    std::cout << "[dzjudgment-->]" << "Serial initialize successfully!" << std::endl;
  }
  else
  {
    std::cout << "[dzjudgment-->]" << "Serial port failed after multiple attempts!" << std::endl;
    ros::shutdown();
  }

  sub_LaserShot_Command = handle.subscribe("LaserShot_Command", 2, &dzjudgment::callback_LaserShot_Command, this);
  sub_scan_command = handle.subscribe("scan_command", 2, &dzjudgment::callback_scan_command, this);

  pub_hpandhismsg = handle.advertise<std_msgs::UInt8MultiArray>("HpAndHitmsg", 10);
  pub_all_Material_Number = handle.advertise<std_msgs::UInt8MultiArray>("all_Material_Number", 10);
  pub_enemy_Material_Number = handle.advertise<std_msgs::UInt8MultiArray>("enemy_Material_Number", 10);
  pub_self_Material_Number = handle.advertise<std_msgs::UInt8MultiArray>("self_Material_Number", 10);
  pub_scan_mode = handle.advertise<std_msgs::UInt8>("material_scan_mode", 10);
  pub_scan_position = handle.advertise<std_msgs::Int32MultiArray>("scan_gimbal_position", 10);
}

dzjudgment::~dzjudgment()
{
  if (ser.isOpen())
  {
    ser.close();
    ROS_INFO("[dzjudgment-->] Serial port closed.");
  }
}

void dzjudgment::callback_LaserShot_Command(const std_msgs::UInt8::ConstPtr &msg)
{
  if (msg->data == 1)
  {
    sendLaserShot();
  }
}

void dzjudgment::run()
{
  int run_rate = 50; // HZ
  ros::Rate rate(run_rate);
  while (ros::ok())
  {
    ros::spinOnce();
    try
    {
      recvESP32Info();
    }
    catch (serial::IOException &e)
    {
      ROS_ERROR("[dzjudgment-->] Serial communication error: %s. Trying to reconnect...", e.what());
      if (!reconnectSerial())
      {
        ROS_ERROR("[dzjudgment-->] Failed to reconnect serial port.");
      }
    }

    // 执行云台扫描控制
    control_gimbal_scan();

    rate.sleep();
  }
}

bool dzjudgment::reconnectSerial()
{
  if (ser.isOpen())
  {
    ser.close();
  }

  int retryCount = 3;
  while (ros::ok())
  {
    try
    {
      ROS_INFO("[dzjudgment-->] Reconnecting serial port...");
      ser.setPort(m_serialport.c_str());
      ser.setBaudrate(m_baudrate);
      serial::Timeout to = serial::Timeout::simpleTimeout(30);
      ser.setTimeout(to);
      ser.open();
      ROS_INFO("[dzjudgment-->] Serial port reconnected successfully!");
      return true;
    }
    catch (serial::IOException &e)
    {
      ROS_WARN("[dzjudgment-->] Unable to reconnect port! Retrying...");
      retryCount--;
      ros::Duration(1).sleep();
    }
  }
  return false;
}

void dzjudgment::recvESP32Info()
{
  const size_t available_bytes = ser.available();
  if (available_bytes == 0)
    return;

  std::vector<uint8_t> incoming(available_bytes);
  const size_t bytes_read = ser.read(incoming.data(), incoming.size());
  serial_rx_buffer.insert(serial_rx_buffer.end(), incoming.begin(),
                          incoming.begin() + bytes_read);

  // 防止线路噪声长期无法形成有效帧时缓存无限增长，只保留可能包含帧头的尾部。
  constexpr size_t kMaxSerialBufferSize = 1024;
  if (serial_rx_buffer.size() > kMaxSerialBufferSize)
  {
    ROS_WARN_THROTTLE(1.0, "[dzjudgment-->] Serial receive buffer overflow, resynchronizing");
    serial_rx_buffer.erase(
        serial_rx_buffer.begin(),
        serial_rx_buffer.end() - kMaxSerialBufferSize / 2);
  }

  while (serial_rx_buffer.size() >= 2)
  {
    // 丢弃帧头前的噪声字节，但保留末尾单独出现的 0xB5，等待下一轮补齐。
    const std::array<uint8_t, 2> frame_header{{0xB5, 0x5B}};
    auto header = std::search(serial_rx_buffer.begin(), serial_rx_buffer.end(),
                              frame_header.begin(), frame_header.end());
    if (header == serial_rx_buffer.end())
    {
      const bool keep_possible_header = serial_rx_buffer.back() == 0xB5;
      serial_rx_buffer.clear();
      if (keep_possible_header)
        serial_rx_buffer.push_back(0xB5);
      return;
    }

    if (header != serial_rx_buffer.begin())
      serial_rx_buffer.erase(serial_rx_buffer.begin(), header);

    if (serial_rx_buffer.size() < 3)
      return;

    const uint8_t data_length = serial_rx_buffer[2];
    if (data_length < 2 || data_length > MAX_VALID_DATA_LENGTH + 2)
    {
      ROS_WARN_THROTTLE(1.0,
                        "[dzjudgment-->] Invalid serial frame length: %u",
                        static_cast<unsigned int>(data_length));
      serial_rx_buffer.erase(serial_rx_buffer.begin());
      continue;
    }

    const size_t complete_frame_size = 3 + static_cast<size_t>(data_length);
    if (serial_rx_buffer.size() < complete_frame_size)
      return;

    processData(serial_rx_buffer.data(), data_length);
    serial_rx_buffer.erase(serial_rx_buffer.begin(),
                           serial_rx_buffer.begin() + complete_frame_size);
  }
}

void dzjudgment::processData(uint8_t *charArray, uint8_t dataLength)
{
  if (charArray[0] == 0xB5 && charArray[1] == 0x5B)
  {
    validDataLength = dataLength - 2;
    if (validDataLength > MAX_VALID_DATA_LENGTH)
    {
      ROS_ERROR("[dzjudgment-->] Data length exceeds maximum limit!");
      return;
    }

    std_msgs::UInt8MultiArray msg;
    msg.data.clear();

    switch (charArray[3])
    {
    case 0x02:
      memset(all_Material_Number, 0, sizeof(all_Material_Number));
      for (int i = 0; i < validDataLength; i++)
      {
        all_Material_Number[i] = charArray[i + 4];
        msg.data.push_back(all_Material_Number[i]);
      }
      pub_all_Material_Number.publish(msg);
      break;
    case 0x03:
      memset(enemy_Material_Number, 0, sizeof(enemy_Material_Number));
      for (int i = 0; i < validDataLength; i++)
      {
        enemy_Material_Number[i] = charArray[i + 4];
        msg.data.push_back(enemy_Material_Number[i]);
      }
      pub_enemy_Material_Number.publish(msg);
      break;
    case 0x04:
      memset(self_Material_Number, 0, sizeof(self_Material_Number));
      for (int i = 0; i < validDataLength; i++)
      {
        self_Material_Number[i] = charArray[i + 4];
        msg.data.push_back(self_Material_Number[i]);
      }
      pub_self_Material_Number.publish(msg);
      break;
    case 0x07:
      targethp = charArray[4];
      self_hp = charArray[5];
      Bing_hit = charArray[6];
      Shooting_Count = charArray[7];
      ammo = charArray[8];
      msg.data.push_back(charArray[4]);
      msg.data.push_back(charArray[5]);
      msg.data.push_back(charArray[6]);
      msg.data.push_back(charArray[7]);
      pub_hpandhismsg.publish(msg);
      break;
    default:
      break;
    }
  }
}

void dzjudgment::sendLaserShot()
{
  unsigned char buf[6] = {0};
  buf[0] = 0xB5; // hdr1
  buf[1] = 0x5B; // hdr2
  buf[2] = 0x03; // len
  buf[3] = 0x21; //
  buf[4] = 0x01; //
  buf[5] = 0x25; // 校验位
  #if 0
    printf("write to 32: ");
    for(int i = 2; i < writesize;i++){
    printf("buf[%d] = %02x ,",i,buf[i]);
    }
    printf("\n");
#endif

  try
  {
    size_t writesize = ser.write(buf, 6);
    if (writesize != 6)
    {
      ROS_ERROR("[dzjudgment-->] Failed to send laser shot command!");
    }
  }
  catch (serial::IOException &e)
  {
    ROS_ERROR("[dzjudgment-->] Serial communication error while sending laser shot command: %s. Trying to reconnect...", e.what());
    if (!reconnectSerial())
    {
      ROS_ERROR("[dzjudgment-->] Failed to reconnect serial port.");
    }
  }
}

// 物资扫描指令回调函数
void dzjudgment::callback_scan_command(const std_msgs::String::ConstPtr &msg)
{
  std::string command = msg->data;
  ROS_INFO("[dzjudgment-->] Received scan command: %s", command.c_str());

  if (command == "rs" || command == "scan_start")
  {
    start_material_scan();
  }
  else if (command == "stop" || command == "scan_stop")
  {
    stop_material_scan();
  }
  else
  {
    ROS_WARN("[dzjudgment-->] Unknown scan command: %s", command.c_str());
  }
}

// 启动物资扫描
void dzjudgment::start_material_scan()
{
  scan_mode_active = true;
  scan_yaw_position = 2050;  // 从中间位置开始
  scan_direction = 1;       // 先向右扫
  ROS_INFO("[dzjudgment-->] Material scan started - gimbal lifting and sweeping");

  // 发布扫描模式状态
  std_msgs::UInt8 scan_mode_msg;
  scan_mode_msg.data = 1;
  pub_scan_mode.publish(scan_mode_msg);
}

// 停止物资扫描
void dzjudgment::stop_material_scan()
{
  scan_mode_active = false;
  scan_yaw_position = 2050;  // 回到中间位置

  // 发布停止扫描模式状态
  std_msgs::UInt8 scan_mode_msg;
  scan_mode_msg.data = 0;
  pub_scan_mode.publish(scan_mode_msg);

  ROS_INFO("[dzjudgment-->] Material scan stopped - gimbal centered");
}

// 云台扫描控制逻辑
void dzjudgment::control_gimbal_scan()
{
  if (!scan_mode_active)
    return;

  // 控制扫描速度，每隔一定周期移动一次
  if (scan_interval_counter++ >= 2)  // 控制扫描速度
  {
    scan_interval_counter = 0;

    // 更新 yaw 位置实现左右扫描
    scan_yaw_position += scan_direction * scan_step;

    // 到达边界时反向
    if (scan_yaw_position >= scan_gimbal_yaw_max)
    {
      scan_yaw_position = scan_gimbal_yaw_max;
      scan_direction = -1;
      ROS_INFO("[dzjudgment-->] Scan reached right limit, turning left");
    }
    else if (scan_yaw_position <= scan_gimbal_yaw_min)
    {
      scan_yaw_position = scan_gimbal_yaw_min;
      scan_direction = 1;
      ROS_INFO("[dzjudgment-->] Scan reached left limit, turning right");
    }

    // 发布扫描位置信息（dzactuator 订阅此话题来控制云台）
    std_msgs::Int32MultiArray scan_pos_msg;
    scan_pos_msg.data.clear();
    scan_pos_msg.data.push_back(scan_gimbal_pitch);  // pitch 抬高
    scan_pos_msg.data.push_back(scan_yaw_position);   // yaw 扫描位置
    pub_scan_position.publish(scan_pos_msg);

    ROS_DEBUG("[dzjudgment-->] Scan: pitch=%d, yaw=%d", scan_gimbal_pitch, scan_yaw_position);
  }
}

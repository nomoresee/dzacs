#include "voice_module.h"
#include <map>
#include <algorithm>

VoiceModule::VoiceModule(ros::NodeHandle nh) : nh_(nh) {
    // 初始化状态
    voice_recognition_ready_ = false;
    voice_synthesis_ready_ = false;
    is_listening_ = false;

    // 初始化命令映射
    initializeCommandMapping();

    // 初始化语音识别和合成
    if (initializeVoiceRecognition()) {
        ROS_INFO("语音识别模块初始化成功");
        voice_recognition_ready_ = true;
    } else {
        ROS_ERROR("语音识别模块初始化失败");
    }

    if (initializeVoiceSynthesis()) {
        ROS_INFO("语音合成模块初始化成功");
        voice_synthesis_ready_ = true;
    } else {
        ROS_ERROR("语音合成模块初始化失败");
    }

    // 订阅话题
    sub_voice_command_ = nh_.subscribe("/voice_command", 10, &VoiceModule::voiceCommandCallback, this);
    sub_visual_target_ = nh_.subscribe("/offset_center", 10, &VoiceModule::visualTargetCallback, this);
    sub_robot_status_ = nh_.subscribe("/robot_status", 10, &VoiceModule::robotStatusCallback, this);

    // 发布话题
    pub_voice_feedback_ = nh_.advertise<std_msgs::String>("/voice_feedback", 10);
    pub_control_command_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_voice_status_ = nh_.advertise<std_msgs::UInt8>("/voice_status", 10);

    ROS_INFO("语音模块初始化完成");
}

VoiceModule::~VoiceModule() {
    // 关闭串口
    if(serial_port_.isOpen()){
        serial_port_.close();
    }
    ROS_INFO("语音模块关闭");
}

void VoiceModule::initializeCommandMapping() {
    command_mapping_["前进"] = "forward";
    command_mapping_["后退"] = "backward";
    command_mapping_["左转"] = "turn_left";
    command_mapping_["右转"] = "turn_right";
    command_mapping_["停止"] = "stop";
    command_mapping_["开始"] = "start";
    command_mapping_["瞄准"] = "aim";
    command_mapping_["射击"] = "shoot";
    command_mapping_["状态"] = "status";
    command_mapping_["电量"] = "battery";
    command_mapping_["位置"] = "position";
}

bool VoiceModule::initializeVoiceRecognition() {
    try {
        ROS_INFO("正在初始化天问block语音识别...");
        ros::Duration(1.0).sleep();
        return true;
    } catch (const std::exception& e) {
        ROS_ERROR("语音识别初始化失败: %s", e.what());
        return false;
    }
}

// 核心修改点：初始化串口连接天问Block
bool VoiceModule::initializeVoiceSynthesis() {
    try {
        ROS_INFO("正在配置天问block串口通信...");
        // 注意：这里的端口号请根据实际情况修改，例如 /dev/ttyUSB0 或 /dev/ttyACM0
        serial_port_.setPort("/dev/ttyUSB0"); 
        serial_port_.setBaudrate(115200); // 必须与天问Block程序中的波特率保持一致
        serial::Timeout to = serial::Timeout::simpleTimeout(1000);
        serial_port_.setTimeout(to);
        
        serial_port_.open();

        if (serial_port_.isOpen()) {
            ROS_INFO("成功连接到天问Block串口，语音合成就绪");
            return true;
        } else {
            ROS_ERROR("无法打开串口，请检查连线或权限(sudo chmod 777 /dev/ttyUSB0)");
            return false;
        }
    } catch (serial::IOException& e) {
        ROS_ERROR("串口初始化异常: %s", e.what());
        return false;
    }
}

std::string VoiceModule::recognizeSpeech() {
    if (!voice_recognition_ready_) {
        ROS_WARN("语音识别模块未就绪");
        return "";
    }
    try {
        ROS_INFO("正在识别语音...");
        std::string recognized_text = "前进"; // 模拟识别结果
        ROS_INFO("识别结果: %s", recognized_text.c_str());
        return recognized_text;
    } catch (const std::exception& e) {
        ROS_ERROR("语音识别失败: %s", e.what());
        return "";
    }
}

// 核心修改点：通过串口发送文字指令给天问Block发声
void VoiceModule::synthesizeSpeech(const std::string& text) {
    if (!voice_synthesis_ready_ || !serial_port_.isOpen()) {
        ROS_WARN("语音合成模块（串口）未就绪，无法播报");
        return;
    }
    try {
        ROS_INFO("正在向天问Block发送语音合成指令: %s", text.c_str());
        
        // 在字符串末尾加上换行符 '\n'，方便天问Block做字符串截断接收
        std::string payload = text + "\n"; 
        serial_port_.write(payload);
        
        ROS_INFO("语音指令发送完成");
    } catch (const std::exception& e) {
        ROS_ERROR("串口发送失败: %s", e.what());
    }
}

void VoiceModule::voiceCommandCallback(const std_msgs::String::ConstPtr& msg) {
    ROS_INFO("收到语音命令: %s", msg->data.c_str());
    processVoiceCommand(msg->data);
}

void VoiceModule::visualTargetCallback(const std_msgs::Int32MultiArray::ConstPtr& msg) {
    // 假设msg->data[0]为物资编号，msg->data[1]为区域标志，msg->data[2]为类别编号
    if (msg->data.size() >= 3) {
        int goods_id = msg->data[0];
        bool in_area = (msg->data[1] == 1); // 1表示在区域内
        int class_id = msg->data[2];
        handleGoodsRecognition(goods_id, class_id, in_area);
    }
}

void VoiceModule::robotStatusCallback(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_INFO("收到机器人状态: %d", msg->data);
}

void VoiceModule::processVoiceCommand(const std::string& command) {
    last_command_ = command;
    auto it = command_mapping_.find(command);
    if (it != command_mapping_.end()) {
        std::string action = it->second;
        ROS_INFO("执行命令: %s -> %s", command.c_str(), action.c_str());
        if (action == "forward") {
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.5;
            cmd.angular.z = 0.0;
            pub_control_command_.publish(cmd);
            generateVoiceFeedback("正在前进");
        } else if (action == "backward") {
            geometry_msgs::Twist cmd;
            cmd.linear.x = -0.5;
            cmd.angular.z = 0.0;
            pub_control_command_.publish(cmd);
            generateVoiceFeedback("正在后退");
        } else if (action == "turn_left") {
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.5;
            pub_control_command_.publish(cmd);
            generateVoiceFeedback("正在左转");
        } else if (action == "turn_right") {
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = -0.5;
            pub_control_command_.publish(cmd);
            generateVoiceFeedback("正在右转");
        } else if (action == "stop") {
            geometry_msgs::Twist cmd;
            cmd.linear.x = 0.0;
            cmd.angular.z = 0.0;
            pub_control_command_.publish(cmd);
            generateVoiceFeedback("已停止");
        } else if (action == "aim") {
            generateVoiceFeedback("正在瞄准目标");
        } else if (action == "shoot") {
            generateVoiceFeedback("正在射击");
        } else if (action == "status") {
            generateVoiceFeedback("系统运行正常");
        } else if (action == "battery") {
            generateVoiceFeedback("电量充足");
        } else if (action == "position") {
            generateVoiceFeedback("当前位置已记录");
        }
    } else {
        generateVoiceFeedback("命令未识别，请重试");
    }
}

void VoiceModule::generateVoiceFeedback(const std::string& message) {
    std_msgs::String feedback_msg;
    feedback_msg.data = message;
    pub_voice_feedback_.publish(feedback_msg);
    // 触发串口发送语音播报
    synthesizeSpeech(message);
}

// 物资播报核心逻辑
void VoiceModule::handleGoodsRecognition(int goods_id, int class_id, bool in_area) {
    // 只处理1-20和101-120，101-120映射为1-20
    int mapped_id = goods_id;
    if (goods_id >= 101 && goods_id <= 120) mapped_id = goods_id - 100;
    if (mapped_id < 1 || mapped_id > 20) return;

    // 区域内且未被抢占
    if (in_area && occupied_goods_.count(mapped_id) == 0) {
        std::string type = (class_id >= 0 && class_id < class_names_.size()) ? class_names_[class_id] : "未知物资";
        std::string msg = std::to_string(mapped_id) + "号位置识别到" + type;
        generateVoiceFeedback(msg);
        occupied_goods_.insert(mapped_id);
        last_broadcast_time_ = ros::Time::now();
        last_goods_id_ = mapped_id;
    }
}

void VoiceModule::resetLap() {
    occupied_goods_.clear();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voice_module");
    ros::NodeHandle nh;

    VoiceModule voice_module(nh);

    ros::Rate rate(10); // 10Hz

    while (ros::ok()) {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
#ifndef __VOICE_MODULE_H_
#define __VOICE_MODULE_H_

#include "ros/ros.h"
#include <iostream>
#include <string>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include <set>
#include <vector>
#include <map>
#include <string>
#include <ros/time.h>

class VoiceModule {
public:
    VoiceModule(ros::NodeHandle nh);
    ~VoiceModule();

private:
    ros::NodeHandle nh_;

    // 订阅者
    ros::Subscriber sub_voice_command_;      // 订阅语音命令
    ros::Subscriber sub_visual_target_;      // 订阅视觉目标信息
    ros::Subscriber sub_robot_status_;       // 订阅机器人状态

    // 发布者
    ros::Publisher pub_voice_feedback_;      // 发布语音反馈
    ros::Publisher pub_control_command_;     // 发布控制命令
    ros::Publisher pub_voice_status_;        // 发布语音状态

    // 回调函数
    void voiceCommandCallback(const std_msgs::String::ConstPtr& msg);
    void visualTargetCallback(const std_msgs::Int32MultiArray::ConstPtr& msg);
    void robotStatusCallback(const std_msgs::UInt8::ConstPtr& msg);

    // 语音处理函数
    void processVoiceCommand(const std::string& command);
    void generateVoiceFeedback(const std::string& message);
    void handleGoodsRecognition(int goods_id, int class_id, bool in_area);
    void resetLap(); // 每圈调用，清空occupied_goods_

    // 语音识别和合成相关
    bool initializeVoiceRecognition();
    bool initializeVoiceSynthesis();
    std::string recognizeSpeech();
    void synthesizeSpeech(const std::string& text);

    // 状态变量
    bool voice_recognition_ready_;
    bool voice_synthesis_ready_;
    bool is_listening_;
    std::string last_command_;

    // 语音命令映射
    std::map<std::string, std::string> command_mapping_;
    void initializeCommandMapping();

    // 物资播报相关
    std::set<int> occupied_goods_; // 已抢占物资编号
    int current_lap_; // 当前圈数
    bool in_zebra_area_; // 是否在斑马线区域
    int last_goods_id_; // 上一次识别的物资编号
    ros::Time last_broadcast_time_;
    std::map<int, std::string> goods_type_map_; // 物资类别编号到名称
    std::vector<std::string> class_names_ = {
        "数字0", "数字1", "数字2", "数字3", "数字4", "数字5", "数字6", "数字7", "数字8", "数字9",
        "dianzuan", "erji", "jianpan", "shouji", "xianshiqi", "shubiao", "wanyongbiao", "shiboqi", 
        "qianzi", "dayinji", "luosidao", "diannaotie", "yinxiang", "juanchi", "banshou",
        "红色小车", "红色移动靶", "蓝色小车", "蓝色移动靶"
    };
};

#endif
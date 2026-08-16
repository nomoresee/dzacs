#ifndef VOICE_MODULE_H_
#define VOICE_MODULE_H_

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/time.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt8.h>

class VoiceModule {
public:
    explicit VoiceModule(ros::NodeHandle nh);
    ~VoiceModule();

private:
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    ros::Subscriber sub_voice_command_;
    ros::Subscriber sub_visual_class_;
    ros::Subscriber sub_material_position_;
    ros::Subscriber sub_robot_status_;
    ros::Timer visual_vote_timer_;

    ros::Publisher pub_voice_feedback_;
    ros::Publisher pub_control_command_;
    ros::Publisher pub_voice_status_;
    ros::Publisher pub_voice_switch_;
    ros::Publisher pub_voice_broadcast_started_;

    std::string visual_class_topic_;
    std::string material_position_topic_;
    double visual_vote_window_;
    double voice_sequence_interval_;

    bool voice_recognition_ready_;
    bool voice_synthesis_ready_;
    bool is_listening_;
    std::string last_command_;

    int current_material_position_;
    bool has_material_position_;
    bool material_position_broadcasted_;
    bool visual_vote_active_;
    std::map<std::string, int> visual_class_votes_;
    std::vector<std::string> visual_vote_order_;

    std::map<std::string, std::string> command_mapping_;
    std::vector<std::string> class_names_;

    void initializeCommandMapping();
    void initializeClassNames();

    bool initializeVoiceRecognition();
    bool initializeVoiceSynthesis();
    std::string recognizeSpeech();
    void synthesizeSpeech(const std::string& text);

    void voiceCommandCallback(const std_msgs::String::ConstPtr& msg);
    void visualClassCallback(const std_msgs::String::ConstPtr& msg);
    void materialPositionCallback(const std_msgs::UInt8::ConstPtr& msg);
    void robotStatusCallback(const std_msgs::UInt8::ConstPtr& msg);
    void visualVoteTimerCallback(const ros::TimerEvent& event);

    void processVoiceCommand(const std::string& command);
    void generateVoiceFeedback(const std::string& message);
    void collectVisualVote(const std::string& token);
    void broadcastVisualToken(const std::string& token);
    bool parseVisualToken(const std::string& token, std::vector<uint8_t>* voice_ids, std::string* broadcast_text) const;
    bool numberToVoiceId(int number, uint8_t* voice_id) const;
    std::string numberToChineseText(int number) const;
    bool classIndexToVoiceId(int class_id, uint8_t* voice_id) const;
    void publishVoiceSwitch(uint8_t voice_id);
    void publishVoiceSwitchSequence(const std::vector<uint8_t>& voice_ids);
    void publishVoiceStatus(uint8_t status);
};

#endif  // VOICE_MODULE_H_

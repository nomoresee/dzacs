#include "voice_module.h"

#include <algorithm>
#include <cctype>
#include <exception>
#include <sstream>

VoiceModule::VoiceModule(ros::NodeHandle nh)
    : nh_(nh),
      private_nh_("~"),
      visual_vote_window_(1.0),
      voice_sequence_interval_(2.0),
      voice_recognition_ready_(false),
      voice_synthesis_ready_(false),
      is_listening_(false),
      current_material_position_(0),
      has_material_position_(false),
      material_position_broadcasted_(false) {
    private_nh_.param<std::string>("visual_class_topic", visual_class_topic_, "/dzatornode2");
    private_nh_.param<std::string>("material_position_topic", material_position_topic_, "/voice_material_position");
    private_nh_.param<double>("visual_vote_window", visual_vote_window_, 1.0);
    private_nh_.param<double>("voice_sequence_interval", voice_sequence_interval_, 2.0);

    initializeCommandMapping();
    initializeClassNames();

    voice_recognition_ready_ = initializeVoiceRecognition();
    voice_synthesis_ready_ = initializeVoiceSynthesis();

    sub_voice_command_ = nh_.subscribe("/voice_command", 10, &VoiceModule::voiceCommandCallback, this);
    sub_visual_class_ = nh_.subscribe(visual_class_topic_, 10, &VoiceModule::visualClassCallback, this);
    sub_material_position_ = nh_.subscribe(material_position_topic_, 10, &VoiceModule::materialPositionCallback, this);
    sub_robot_status_ = nh_.subscribe("/robot_status", 10, &VoiceModule::robotStatusCallback, this);
    visual_vote_timer_ = nh_.createTimer(ros::Duration(visual_vote_window_),
                                         &VoiceModule::visualVoteTimerCallback, this,
                                         true, false);

    pub_voice_feedback_ = nh_.advertise<std_msgs::String>("/voice_feedback", 10);
    pub_control_command_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    pub_voice_status_ = nh_.advertise<std_msgs::UInt8>("/voice_status", 10);
    pub_voice_switch_ = nh_.advertise<std_msgs::UInt8>("/voice_switch", 10);
    pub_voice_broadcast_started_ = nh_.advertise<std_msgs::UInt8>("/voice_broadcast_started", 10);

    publishVoiceStatus(voice_synthesis_ready_ ? 1 : 0);

    ROS_INFO("voice_module started. visual_class_topic=%s material_position_topic=%s visual_vote_window=%.1f",
             visual_class_topic_.c_str(), material_position_topic_.c_str(), visual_vote_window_);
}

VoiceModule::~VoiceModule() = default;

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

void VoiceModule::initializeClassNames() {
    class_names_ = {
        "数字0", "数字1", "数字2", "数字3", "数字4",
        "数字5", "数字6", "数字7", "数字8", "数字9",
        "电钻", "耳机", "键盘", "手机", "显示器",
        "鼠标", "万用表", "示波器", "钳子", "打印机",
        "螺丝刀", "电烙铁", "音响", "卷尺", "扳手",
    };
}

bool VoiceModule::initializeVoiceRecognition() {
    ROS_INFO("voice recognition input uses /voice_command text topic.");
    return true;
}

bool VoiceModule::initializeVoiceSynthesis() {
    ROS_INFO("voice synthesis uses /voice_switch preset ids through dzactuator.");
    return true;
}

std::string VoiceModule::recognizeSpeech() {
    if (!voice_recognition_ready_) {
        return "";
    }
    return "";
}

void VoiceModule::synthesizeSpeech(const std::string& text) {
    ROS_INFO("voice feedback: %s", text.c_str());
}

void VoiceModule::voiceCommandCallback(const std_msgs::String::ConstPtr& msg) {
    processVoiceCommand(msg->data);
}

void VoiceModule::visualClassCallback(const std_msgs::String::ConstPtr& msg) {
    collectVisualVote(msg->data);
}

void VoiceModule::materialPositionCallback(const std_msgs::UInt8::ConstPtr& msg) {
    const int position = static_cast<int>(msg->data);
    if (position < 1 || position > 16) {
        visual_vote_timer_.stop();
        visual_class_votes_.clear();
        visual_vote_order_.clear();
        has_material_position_ = false;
        current_material_position_ = 0;
        material_position_broadcasted_ = false;
        ROS_WARN("ignored invalid material position: %d (expected 1-16).", position);
        return;
    }

    current_material_position_ = position;
    has_material_position_ = true;
    visual_vote_timer_.stop();
    visual_class_votes_.clear();
    visual_vote_order_.clear();
    material_position_broadcasted_ = false;

    visual_vote_timer_.setPeriod(ros::Duration(visual_vote_window_), true);
    visual_vote_timer_.start();

    ROS_INFO("material position updated: %d; collecting visual votes for %.1f seconds.",
             current_material_position_, visual_vote_window_);
}

void VoiceModule::robotStatusCallback(const std_msgs::UInt8::ConstPtr& msg) {
    ROS_DEBUG("robot_status=%u", msg->data);
}

void VoiceModule::processVoiceCommand(const std::string& command) {
    last_command_ = command;

    const auto it = command_mapping_.find(command);
    if (it == command_mapping_.end()) {
        generateVoiceFeedback("命令未识别");
        return;
    }

    const std::string& action = it->second;
    geometry_msgs::Twist cmd;
    bool should_publish_cmd = true;

    if (action == "forward") {
        cmd.linear.x = 0.5;
        generateVoiceFeedback("正在前进");
    } else if (action == "backward") {
        cmd.linear.x = -0.5;
        generateVoiceFeedback("正在后退");
    } else if (action == "turn_left") {
        cmd.angular.z = 0.5;
        generateVoiceFeedback("正在左转");
    } else if (action == "turn_right") {
        cmd.angular.z = -0.5;
        generateVoiceFeedback("正在右转");
    } else if (action == "stop") {
        generateVoiceFeedback("已停止");
    } else if (action == "aim") {
        should_publish_cmd = false;
        generateVoiceFeedback("正在瞄准目标");
    } else if (action == "shoot") {
        should_publish_cmd = false;
        generateVoiceFeedback("正在射击");
    } else if (action == "status") {
        should_publish_cmd = false;
        generateVoiceFeedback("系统运行正常");
    } else if (action == "battery") {
        should_publish_cmd = false;
        generateVoiceFeedback("电量充足");
    } else if (action == "position") {
        should_publish_cmd = false;
        generateVoiceFeedback("当前位置已记录");
    } else {
        should_publish_cmd = false;
    }

    if (should_publish_cmd) {
        pub_control_command_.publish(cmd);
    }
}

void VoiceModule::generateVoiceFeedback(const std::string& message) {
    std_msgs::String feedback_msg;
    feedback_msg.data = message;
    pub_voice_feedback_.publish(feedback_msg);
    synthesizeSpeech(message);
}

void VoiceModule::collectVisualVote(const std::string& token) {
    if (!has_material_position_ || material_position_broadcasted_) {
        return;
    }

    std::vector<uint8_t> voice_ids;
    std::string broadcast_text;
    if (!parseVisualToken(token, &voice_ids, &broadcast_text)) {
        ROS_WARN("invalid visual token: %s", token.c_str());
        return;
    }

    const auto vote_it = visual_class_votes_.find(token);
    if (vote_it == visual_class_votes_.end()) {
        visual_class_votes_[token] = 1;
        visual_vote_order_.push_back(token);
    } else {
        ++vote_it->second;
    }

}

void VoiceModule::visualVoteTimerCallback(const ros::TimerEvent&) {
    if (!has_material_position_ || material_position_broadcasted_) {
        return;
    }

    if (visual_vote_order_.empty()) {
        const int expired_position = current_material_position_;
        visual_class_votes_.clear();
        visual_vote_order_.clear();
        has_material_position_ = false;
        current_material_position_ = 0;
        material_position_broadcasted_ = false;
        ROS_WARN("no visual votes received within %.1f seconds for material position %d; cleared.",
                 visual_vote_window_, expired_position);
        return;
    }

    std::string selected_token;
    int highest_votes = -1;
    for (const std::string& token : visual_vote_order_) {
        const int votes = visual_class_votes_[token];
        if (votes > highest_votes) {
            highest_votes = votes;
            selected_token = token;
        }
    }

    ROS_INFO("visual vote selected token=%s votes=%d for material position %d.",
             selected_token.c_str(), highest_votes, current_material_position_);
    broadcastVisualToken(selected_token);
}

void VoiceModule::broadcastVisualToken(const std::string& token) {
    std::vector<uint8_t> voice_ids;
    std::string broadcast_text;
    if (!parseVisualToken(token, &voice_ids, &broadcast_text)) {
        ROS_WARN("cannot broadcast invalid visual token: %s", token.c_str());
        return;
    }

    publishVoiceSwitchSequence(voice_ids);
    generateVoiceFeedback(broadcast_text);
    material_position_broadcasted_ = true;
    const int announced_position = current_material_position_;
    visual_vote_timer_.stop();
    visual_class_votes_.clear();
    visual_vote_order_.clear();
    has_material_position_ = false;
    current_material_position_ = 0;
    material_position_broadcasted_ = false;
    ROS_INFO("material position %d broadcast completed and cleared.", announced_position);
}

bool VoiceModule::parseVisualToken(const std::string& token, std::vector<uint8_t>* voice_ids, std::string* broadcast_text) const {
    if (token.empty() || voice_ids == nullptr || broadcast_text == nullptr) {
        return false;
    }

    voice_ids->clear();

    const bool is_index = std::all_of(token.begin(), token.end(), [](unsigned char c) {
        return std::isdigit(c) != 0;
    });
    if (is_index) {
        uint8_t voice_id = 0;
        const int class_id = std::stoi(token);
        if (class_id < 0 || class_id >= static_cast<int>(class_names_.size()) ||
            !classIndexToVoiceId(class_id, &voice_id)) {
            return false;
        }
        if (has_material_position_ && class_id >= 10) {
            // The ASR board reserves 0x81-0x90 for position-prefix clips and
            // 0xA0-0xAE for the 15 item-name-only clips. Keeping these
            // separate from the legacy preset IDs avoids repeated prefixes.
            if (current_material_position_ < 1 || current_material_position_ > 16) {
                return false;
            }
            const uint8_t position_voice_id = static_cast<uint8_t>(0x80 + current_material_position_);
            const uint8_t item_voice_id = static_cast<uint8_t>(0xA0 + class_id - 10);
            voice_ids->push_back(position_voice_id);
            voice_ids->push_back(item_voice_id);
            *broadcast_text = std::to_string(current_material_position_) + "号位置识别到" + class_names_[class_id];
        } else {
            voice_ids->push_back(voice_id);
            *broadcast_text = "识别到" + class_names_[class_id];
        }
        return true;
    }

    std::string number_text;
    std::stringstream ss(token);
    std::string part;
    while (std::getline(ss, part, '_')) {
        if (part.empty()) {
            return false;
        }
        const bool part_is_digit = std::all_of(part.begin(), part.end(), [](unsigned char c) {
            return std::isdigit(c) != 0;
        });
        if (!part_is_digit) {
            return false;
        }
        number_text += part;
    }

    if (number_text.empty()) {
        return false;
    }

    const int number = std::stoi(number_text);
    uint8_t voice_id = 0;
    if (!numberToVoiceId(number, &voice_id)) {
        return false;
    }
    voice_ids->push_back(voice_id);

    *broadcast_text = "识别到" + numberToChineseText(number);
    return true;
}

bool VoiceModule::numberToVoiceId(int number, uint8_t* voice_id) const {
    if (voice_id == nullptr || number < 0 || number > 99) {
        return false;
    }

    if (number <= 9) {
        *voice_id = static_cast<uint8_t>(4 + number);
        return true;
    }

    // Measured preset table: 29=数字10, 30=数字11, so N>=10 follows N+19.
    *voice_id = static_cast<uint8_t>(number + 19);
    return true;
}

std::string VoiceModule::numberToChineseText(int number) const {
    static const char* digits[] = {
        "零", "一", "二", "三", "四", "五", "六", "七", "八", "九",
    };

    if (number < 0 || number > 99) {
        return "";
    }

    if (number < 10) {
        return digits[number];
    }

    const int tens = number / 10;
    const int ones = number % 10;

    if (tens == 1) {
        return ones == 0 ? std::string("十") : std::string("十") + digits[ones];
    }

    return ones == 0 ? std::string(digits[tens]) + "十"
                     : std::string(digits[tens]) + "十" + digits[ones];
}

bool VoiceModule::classIndexToVoiceId(int class_id, uint8_t* voice_id) const {
    if (voice_id == nullptr || class_id < 0 || class_id >= static_cast<int>(class_names_.size())) {
        return false;
    }

    if (class_id <= 9) {
        return numberToVoiceId(class_id, voice_id);
    }

    // Measured preset table: 14=电钻 maps to class index 10, continuing to 28=扳手.
    *voice_id = static_cast<uint8_t>(class_id + 4);
    return true;
}

void VoiceModule::publishVoiceSwitch(uint8_t voice_id) {
    std_msgs::UInt8 msg;
    msg.data = voice_id;
    pub_voice_switch_.publish(msg);
    ROS_INFO("voice_switch id=%u", voice_id);
}

void VoiceModule::publishVoiceSwitchSequence(const std::vector<uint8_t>& voice_ids) {
    for (size_t i = 0; i < voice_ids.size(); ++i) {
        publishVoiceSwitch(voice_ids[i]);
        if (i == 0 && has_material_position_) {
            std_msgs::UInt8 started_msg;
            started_msg.data = static_cast<uint8_t>(current_material_position_);
            pub_voice_broadcast_started_.publish(started_msg);
            ROS_INFO("voice broadcast started for material position %d.", current_material_position_);
        }
        if (i + 1 < voice_ids.size()) {
            ros::Duration(voice_sequence_interval_).sleep();
        }
    }
}

void VoiceModule::publishVoiceStatus(uint8_t status) {
    std_msgs::UInt8 status_msg;
    status_msg.data = status;
    pub_voice_status_.publish(status_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "voice_module");
    ros::NodeHandle nh;

    VoiceModule voice_module(nh);
    ros::spin();

    return 0;
}

// Auto-generated. Do not edit!

// (in-package auto_navigation.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;
let std_msgs = _finder('std_msgs');

//-----------------------------------------------------------

class NavigationStatus {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.status = null;
      this.current_goal = null;
      this.progress = null;
      this.distance_to_goal = null;
      this.estimated_time = null;
      this.error_message = null;
      this.is_executing = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('status')) {
        this.status = initObj.status
      }
      else {
        this.status = '';
      }
      if (initObj.hasOwnProperty('current_goal')) {
        this.current_goal = initObj.current_goal
      }
      else {
        this.current_goal = '';
      }
      if (initObj.hasOwnProperty('progress')) {
        this.progress = initObj.progress
      }
      else {
        this.progress = 0.0;
      }
      if (initObj.hasOwnProperty('distance_to_goal')) {
        this.distance_to_goal = initObj.distance_to_goal
      }
      else {
        this.distance_to_goal = 0.0;
      }
      if (initObj.hasOwnProperty('estimated_time')) {
        this.estimated_time = initObj.estimated_time
      }
      else {
        this.estimated_time = 0.0;
      }
      if (initObj.hasOwnProperty('error_message')) {
        this.error_message = initObj.error_message
      }
      else {
        this.error_message = '';
      }
      if (initObj.hasOwnProperty('is_executing')) {
        this.is_executing = initObj.is_executing
      }
      else {
        this.is_executing = false;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NavigationStatus
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [status]
    bufferOffset = _serializer.string(obj.status, buffer, bufferOffset);
    // Serialize message field [current_goal]
    bufferOffset = _serializer.string(obj.current_goal, buffer, bufferOffset);
    // Serialize message field [progress]
    bufferOffset = _serializer.float64(obj.progress, buffer, bufferOffset);
    // Serialize message field [distance_to_goal]
    bufferOffset = _serializer.float64(obj.distance_to_goal, buffer, bufferOffset);
    // Serialize message field [estimated_time]
    bufferOffset = _serializer.float64(obj.estimated_time, buffer, bufferOffset);
    // Serialize message field [error_message]
    bufferOffset = _serializer.string(obj.error_message, buffer, bufferOffset);
    // Serialize message field [is_executing]
    bufferOffset = _serializer.bool(obj.is_executing, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NavigationStatus
    let len;
    let data = new NavigationStatus(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [status]
    data.status = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [current_goal]
    data.current_goal = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [progress]
    data.progress = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [distance_to_goal]
    data.distance_to_goal = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [estimated_time]
    data.estimated_time = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [error_message]
    data.error_message = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [is_executing]
    data.is_executing = _deserializer.bool(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += _getByteLength(object.status);
    length += _getByteLength(object.current_goal);
    length += _getByteLength(object.error_message);
    return length + 37;
  }

  static datatype() {
    // Returns string type for a message object
    return 'auto_navigation/NavigationStatus';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '762d278883490d8229d0a8c5e42445ba';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # NavigationStatus.msg
    # 导航状态消息
    
    Header header
    
    # 导航状态
    string status  # "idle", "navigating", "reached", "failed", "cancelled"
    
    # 当前目标
    string current_goal
    
    # 进度百分比
    float64 progress  # 0.0-100.0
    
    # 距离目标剩余距离
    float64 distance_to_goal
    
    # 预计到达时间（秒）
    float64 estimated_time
    
    # 错误信息
    string error_message
    
    # 是否正在执行
    bool is_executing 
    ================================================================================
    MSG: std_msgs/Header
    # Standard metadata for higher-level stamped data types.
    # This is generally used to communicate timestamped data 
    # in a particular coordinate frame.
    # 
    # sequence ID: consecutively increasing ID 
    uint32 seq
    #Two-integer timestamp that is expressed as:
    # * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
    # * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
    # time-handling sugar is provided by the client library
    time stamp
    #Frame this data is associated with
    string frame_id
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NavigationStatus(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.status !== undefined) {
      resolved.status = msg.status;
    }
    else {
      resolved.status = ''
    }

    if (msg.current_goal !== undefined) {
      resolved.current_goal = msg.current_goal;
    }
    else {
      resolved.current_goal = ''
    }

    if (msg.progress !== undefined) {
      resolved.progress = msg.progress;
    }
    else {
      resolved.progress = 0.0
    }

    if (msg.distance_to_goal !== undefined) {
      resolved.distance_to_goal = msg.distance_to_goal;
    }
    else {
      resolved.distance_to_goal = 0.0
    }

    if (msg.estimated_time !== undefined) {
      resolved.estimated_time = msg.estimated_time;
    }
    else {
      resolved.estimated_time = 0.0
    }

    if (msg.error_message !== undefined) {
      resolved.error_message = msg.error_message;
    }
    else {
      resolved.error_message = ''
    }

    if (msg.is_executing !== undefined) {
      resolved.is_executing = msg.is_executing;
    }
    else {
      resolved.is_executing = false
    }

    return resolved;
    }
};

module.exports = NavigationStatus;

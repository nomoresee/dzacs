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
let geometry_msgs = _finder('geometry_msgs');

//-----------------------------------------------------------

class NavigationGoal {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.header = null;
      this.goal_pose = null;
      this.goal_type = null;
      this.priority = null;
      this.auto_execute = null;
      this.timeout = null;
      this.description = null;
    }
    else {
      if (initObj.hasOwnProperty('header')) {
        this.header = initObj.header
      }
      else {
        this.header = new std_msgs.msg.Header();
      }
      if (initObj.hasOwnProperty('goal_pose')) {
        this.goal_pose = initObj.goal_pose
      }
      else {
        this.goal_pose = new geometry_msgs.msg.PoseStamped();
      }
      if (initObj.hasOwnProperty('goal_type')) {
        this.goal_type = initObj.goal_type
      }
      else {
        this.goal_type = '';
      }
      if (initObj.hasOwnProperty('priority')) {
        this.priority = initObj.priority
      }
      else {
        this.priority = 0;
      }
      if (initObj.hasOwnProperty('auto_execute')) {
        this.auto_execute = initObj.auto_execute
      }
      else {
        this.auto_execute = false;
      }
      if (initObj.hasOwnProperty('timeout')) {
        this.timeout = initObj.timeout
      }
      else {
        this.timeout = 0.0;
      }
      if (initObj.hasOwnProperty('description')) {
        this.description = initObj.description
      }
      else {
        this.description = '';
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type NavigationGoal
    // Serialize message field [header]
    bufferOffset = std_msgs.msg.Header.serialize(obj.header, buffer, bufferOffset);
    // Serialize message field [goal_pose]
    bufferOffset = geometry_msgs.msg.PoseStamped.serialize(obj.goal_pose, buffer, bufferOffset);
    // Serialize message field [goal_type]
    bufferOffset = _serializer.string(obj.goal_type, buffer, bufferOffset);
    // Serialize message field [priority]
    bufferOffset = _serializer.int32(obj.priority, buffer, bufferOffset);
    // Serialize message field [auto_execute]
    bufferOffset = _serializer.bool(obj.auto_execute, buffer, bufferOffset);
    // Serialize message field [timeout]
    bufferOffset = _serializer.float64(obj.timeout, buffer, bufferOffset);
    // Serialize message field [description]
    bufferOffset = _serializer.string(obj.description, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type NavigationGoal
    let len;
    let data = new NavigationGoal(null);
    // Deserialize message field [header]
    data.header = std_msgs.msg.Header.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_pose]
    data.goal_pose = geometry_msgs.msg.PoseStamped.deserialize(buffer, bufferOffset);
    // Deserialize message field [goal_type]
    data.goal_type = _deserializer.string(buffer, bufferOffset);
    // Deserialize message field [priority]
    data.priority = _deserializer.int32(buffer, bufferOffset);
    // Deserialize message field [auto_execute]
    data.auto_execute = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [timeout]
    data.timeout = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [description]
    data.description = _deserializer.string(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    let length = 0;
    length += std_msgs.msg.Header.getMessageSize(object.header);
    length += geometry_msgs.msg.PoseStamped.getMessageSize(object.goal_pose);
    length += _getByteLength(object.goal_type);
    length += _getByteLength(object.description);
    return length + 21;
  }

  static datatype() {
    // Returns string type for a message object
    return 'auto_navigation/NavigationGoal';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return 'fc397e7956bc13f038b8233aec3f06a3';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    # NavigationGoal.msg
    # 导航目标消息
    
    Header header
    
    # 目标位置
    geometry_msgs/PoseStamped goal_pose
    
    # 目标类型
    string goal_type  # "point", "path", "sequence"
    
    # 目标优先级
    int32 priority  # 1-10, 数字越大优先级越高
    
    # 是否自动执行
    bool auto_execute
    
    # 超时时间（秒）
    float64 timeout
    
    # 目标描述
    string description 
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
    
    ================================================================================
    MSG: geometry_msgs/PoseStamped
    # A Pose with reference coordinate frame and timestamp
    Header header
    Pose pose
    
    ================================================================================
    MSG: geometry_msgs/Pose
    # A representation of pose in free space, composed of position and orientation. 
    Point position
    Quaternion orientation
    
    ================================================================================
    MSG: geometry_msgs/Point
    # This contains the position of a point in free space
    float64 x
    float64 y
    float64 z
    
    ================================================================================
    MSG: geometry_msgs/Quaternion
    # This represents an orientation in free space in quaternion form.
    
    float64 x
    float64 y
    float64 z
    float64 w
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new NavigationGoal(null);
    if (msg.header !== undefined) {
      resolved.header = std_msgs.msg.Header.Resolve(msg.header)
    }
    else {
      resolved.header = new std_msgs.msg.Header()
    }

    if (msg.goal_pose !== undefined) {
      resolved.goal_pose = geometry_msgs.msg.PoseStamped.Resolve(msg.goal_pose)
    }
    else {
      resolved.goal_pose = new geometry_msgs.msg.PoseStamped()
    }

    if (msg.goal_type !== undefined) {
      resolved.goal_type = msg.goal_type;
    }
    else {
      resolved.goal_type = ''
    }

    if (msg.priority !== undefined) {
      resolved.priority = msg.priority;
    }
    else {
      resolved.priority = 0
    }

    if (msg.auto_execute !== undefined) {
      resolved.auto_execute = msg.auto_execute;
    }
    else {
      resolved.auto_execute = false
    }

    if (msg.timeout !== undefined) {
      resolved.timeout = msg.timeout;
    }
    else {
      resolved.timeout = 0.0
    }

    if (msg.description !== undefined) {
      resolved.description = msg.description;
    }
    else {
      resolved.description = ''
    }

    return resolved;
    }
};

module.exports = NavigationGoal;

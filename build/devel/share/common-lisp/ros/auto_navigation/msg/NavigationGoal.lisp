; Auto-generated. Do not edit!


(cl:in-package auto_navigation-msg)


;//! \htmlinclude NavigationGoal.msg.html

(cl:defclass <NavigationGoal> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (goal_pose
    :reader goal_pose
    :initarg :goal_pose
    :type geometry_msgs-msg:PoseStamped
    :initform (cl:make-instance 'geometry_msgs-msg:PoseStamped))
   (goal_type
    :reader goal_type
    :initarg :goal_type
    :type cl:string
    :initform "")
   (priority
    :reader priority
    :initarg :priority
    :type cl:integer
    :initform 0)
   (auto_execute
    :reader auto_execute
    :initarg :auto_execute
    :type cl:boolean
    :initform cl:nil)
   (timeout
    :reader timeout
    :initarg :timeout
    :type cl:float
    :initform 0.0)
   (description
    :reader description
    :initarg :description
    :type cl:string
    :initform ""))
)

(cl:defclass NavigationGoal (<NavigationGoal>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <NavigationGoal>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'NavigationGoal)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name auto_navigation-msg:<NavigationGoal> is deprecated: use auto_navigation-msg:NavigationGoal instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_navigation-msg:header-val is deprecated.  Use auto_navigation-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'goal_pose-val :lambda-list '(m))
(cl:defmethod goal_pose-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_navigation-msg:goal_pose-val is deprecated.  Use auto_navigation-msg:goal_pose instead.")
  (goal_pose m))

(cl:ensure-generic-function 'goal_type-val :lambda-list '(m))
(cl:defmethod goal_type-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_navigation-msg:goal_type-val is deprecated.  Use auto_navigation-msg:goal_type instead.")
  (goal_type m))

(cl:ensure-generic-function 'priority-val :lambda-list '(m))
(cl:defmethod priority-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_navigation-msg:priority-val is deprecated.  Use auto_navigation-msg:priority instead.")
  (priority m))

(cl:ensure-generic-function 'auto_execute-val :lambda-list '(m))
(cl:defmethod auto_execute-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_navigation-msg:auto_execute-val is deprecated.  Use auto_navigation-msg:auto_execute instead.")
  (auto_execute m))

(cl:ensure-generic-function 'timeout-val :lambda-list '(m))
(cl:defmethod timeout-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_navigation-msg:timeout-val is deprecated.  Use auto_navigation-msg:timeout instead.")
  (timeout m))

(cl:ensure-generic-function 'description-val :lambda-list '(m))
(cl:defmethod description-val ((m <NavigationGoal>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader auto_navigation-msg:description-val is deprecated.  Use auto_navigation-msg:description instead.")
  (description m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <NavigationGoal>) ostream)
  "Serializes a message object of type '<NavigationGoal>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'goal_pose) ostream)
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'goal_type))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'goal_type))
  (cl:let* ((signed (cl:slot-value msg 'priority)) (unsigned (cl:if (cl:< signed 0) (cl:+ signed 4294967296) signed)))
    (cl:write-byte (cl:ldb (cl:byte 8 0) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) unsigned) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) unsigned) ostream)
    )
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'auto_execute) 1 0)) ostream)
  (cl:let ((bits (roslisp-utils:encode-double-float-bits (cl:slot-value msg 'timeout))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 32) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 40) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 48) bits) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 56) bits) ostream))
  (cl:let ((__ros_str_len (cl:length (cl:slot-value msg 'description))))
    (cl:write-byte (cl:ldb (cl:byte 8 0) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 8) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 16) __ros_str_len) ostream)
    (cl:write-byte (cl:ldb (cl:byte 8 24) __ros_str_len) ostream))
  (cl:map cl:nil #'(cl:lambda (c) (cl:write-byte (cl:char-code c) ostream)) (cl:slot-value msg 'description))
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <NavigationGoal>) istream)
  "Deserializes a message object of type '<NavigationGoal>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'goal_pose) istream)
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'goal_type) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'goal_type) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
    (cl:let ((unsigned 0))
      (cl:setf (cl:ldb (cl:byte 8 0) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) unsigned) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) unsigned) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'priority) (cl:if (cl:< unsigned 2147483648) unsigned (cl:- unsigned 4294967296))))
    (cl:setf (cl:slot-value msg 'auto_execute) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:let ((bits 0))
      (cl:setf (cl:ldb (cl:byte 8 0) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 32) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 40) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 48) bits) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 56) bits) (cl:read-byte istream))
    (cl:setf (cl:slot-value msg 'timeout) (roslisp-utils:decode-double-float-bits bits)))
    (cl:let ((__ros_str_len 0))
      (cl:setf (cl:ldb (cl:byte 8 0) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 8) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 16) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:ldb (cl:byte 8 24) __ros_str_len) (cl:read-byte istream))
      (cl:setf (cl:slot-value msg 'description) (cl:make-string __ros_str_len))
      (cl:dotimes (__ros_str_idx __ros_str_len msg)
        (cl:setf (cl:char (cl:slot-value msg 'description) __ros_str_idx) (cl:code-char (cl:read-byte istream)))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<NavigationGoal>)))
  "Returns string type for a message object of type '<NavigationGoal>"
  "auto_navigation/NavigationGoal")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'NavigationGoal)))
  "Returns string type for a message object of type 'NavigationGoal"
  "auto_navigation/NavigationGoal")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<NavigationGoal>)))
  "Returns md5sum for a message object of type '<NavigationGoal>"
  "fc397e7956bc13f038b8233aec3f06a3")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'NavigationGoal)))
  "Returns md5sum for a message object of type 'NavigationGoal"
  "fc397e7956bc13f038b8233aec3f06a3")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<NavigationGoal>)))
  "Returns full string definition for message of type '<NavigationGoal>"
  (cl:format cl:nil "# NavigationGoal.msg~%# 导航目标消息~%~%Header header~%~%# 目标位置~%geometry_msgs/PoseStamped goal_pose~%~%# 目标类型~%string goal_type  # \"point\", \"path\", \"sequence\"~%~%# 目标优先级~%int32 priority  # 1-10, 数字越大优先级越高~%~%# 是否自动执行~%bool auto_execute~%~%# 超时时间（秒）~%float64 timeout~%~%# 目标描述~%string description ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'NavigationGoal)))
  "Returns full string definition for message of type 'NavigationGoal"
  (cl:format cl:nil "# NavigationGoal.msg~%# 导航目标消息~%~%Header header~%~%# 目标位置~%geometry_msgs/PoseStamped goal_pose~%~%# 目标类型~%string goal_type  # \"point\", \"path\", \"sequence\"~%~%# 目标优先级~%int32 priority  # 1-10, 数字越大优先级越高~%~%# 是否自动执行~%bool auto_execute~%~%# 超时时间（秒）~%float64 timeout~%~%# 目标描述~%string description ~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')~%# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%string frame_id~%~%================================================================================~%MSG: geometry_msgs/PoseStamped~%# A Pose with reference coordinate frame and timestamp~%Header header~%Pose pose~%~%================================================================================~%MSG: geometry_msgs/Pose~%# A representation of pose in free space, composed of position and orientation. ~%Point position~%Quaternion orientation~%~%================================================================================~%MSG: geometry_msgs/Point~%# This contains the position of a point in free space~%float64 x~%float64 y~%float64 z~%~%================================================================================~%MSG: geometry_msgs/Quaternion~%# This represents an orientation in free space in quaternion form.~%~%float64 x~%float64 y~%float64 z~%float64 w~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <NavigationGoal>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'goal_pose))
     4 (cl:length (cl:slot-value msg 'goal_type))
     4
     1
     8
     4 (cl:length (cl:slot-value msg 'description))
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <NavigationGoal>))
  "Converts a ROS message object to a list"
  (cl:list 'NavigationGoal
    (cl:cons ':header (header msg))
    (cl:cons ':goal_pose (goal_pose msg))
    (cl:cons ':goal_type (goal_type msg))
    (cl:cons ':priority (priority msg))
    (cl:cons ':auto_execute (auto_execute msg))
    (cl:cons ':timeout (timeout msg))
    (cl:cons ':description (description msg))
))

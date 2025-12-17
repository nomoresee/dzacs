;; Auto-generated. Do not edit!


(when (boundp 'auto_navigation::NavigationGoal)
  (if (not (find-package "AUTO_NAVIGATION"))
    (make-package "AUTO_NAVIGATION"))
  (shadow 'NavigationGoal (find-package "AUTO_NAVIGATION")))
(unless (find-package "AUTO_NAVIGATION::NAVIGATIONGOAL")
  (make-package "AUTO_NAVIGATION::NAVIGATIONGOAL"))

(in-package "ROS")
;;//! \htmlinclude NavigationGoal.msg.html
(if (not (find-package "GEOMETRY_MSGS"))
  (ros::roseus-add-msgs "geometry_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass auto_navigation::NavigationGoal
  :super ros::object
  :slots (_header _goal_pose _goal_type _priority _auto_execute _timeout _description ))

(defmethod auto_navigation::NavigationGoal
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:goal_pose __goal_pose) (instance geometry_msgs::PoseStamped :init))
    ((:goal_type __goal_type) "")
    ((:priority __priority) 0)
    ((:auto_execute __auto_execute) nil)
    ((:timeout __timeout) 0.0)
    ((:description __description) "")
    )
   (send-super :init)
   (setq _header __header)
   (setq _goal_pose __goal_pose)
   (setq _goal_type (string __goal_type))
   (setq _priority (round __priority))
   (setq _auto_execute __auto_execute)
   (setq _timeout (float __timeout))
   (setq _description (string __description))
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:goal_pose
   (&rest __goal_pose)
   (if (keywordp (car __goal_pose))
       (send* _goal_pose __goal_pose)
     (progn
       (if __goal_pose (setq _goal_pose (car __goal_pose)))
       _goal_pose)))
  (:goal_type
   (&optional __goal_type)
   (if __goal_type (setq _goal_type __goal_type)) _goal_type)
  (:priority
   (&optional __priority)
   (if __priority (setq _priority __priority)) _priority)
  (:auto_execute
   (&optional (__auto_execute :null))
   (if (not (eq __auto_execute :null)) (setq _auto_execute __auto_execute)) _auto_execute)
  (:timeout
   (&optional __timeout)
   (if __timeout (setq _timeout __timeout)) _timeout)
  (:description
   (&optional __description)
   (if __description (setq _description __description)) _description)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; geometry_msgs/PoseStamped _goal_pose
    (send _goal_pose :serialization-length)
    ;; string _goal_type
    4 (length _goal_type)
    ;; int32 _priority
    4
    ;; bool _auto_execute
    1
    ;; float64 _timeout
    8
    ;; string _description
    4 (length _description)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; geometry_msgs/PoseStamped _goal_pose
       (send _goal_pose :serialize s)
     ;; string _goal_type
       (write-long (length _goal_type) s) (princ _goal_type s)
     ;; int32 _priority
       (write-long _priority s)
     ;; bool _auto_execute
       (if _auto_execute (write-byte -1 s) (write-byte 0 s))
     ;; float64 _timeout
       (sys::poke _timeout (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _description
       (write-long (length _description) s) (princ _description s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; geometry_msgs/PoseStamped _goal_pose
     (send _goal_pose :deserialize buf ptr-) (incf ptr- (send _goal_pose :serialization-length))
   ;; string _goal_type
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _goal_type (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; int32 _priority
     (setq _priority (sys::peek buf ptr- :integer)) (incf ptr- 4)
   ;; bool _auto_execute
     (setq _auto_execute (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; float64 _timeout
     (setq _timeout (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _description
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _description (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get auto_navigation::NavigationGoal :md5sum-) "fc397e7956bc13f038b8233aec3f06a3")
(setf (get auto_navigation::NavigationGoal :datatype-) "auto_navigation/NavigationGoal")
(setf (get auto_navigation::NavigationGoal :definition-)
      "# NavigationGoal.msg
# 导航目标消息

Header header

# 目标位置
geometry_msgs/PoseStamped goal_pose

# 目标类型
string goal_type  # \"point\", \"path\", \"sequence\"

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

")



(provide :auto_navigation/NavigationGoal "fc397e7956bc13f038b8233aec3f06a3")



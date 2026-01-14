;; Auto-generated. Do not edit!


(when (boundp 'auto_navigation::NavigationStatus)
  (if (not (find-package "AUTO_NAVIGATION"))
    (make-package "AUTO_NAVIGATION"))
  (shadow 'NavigationStatus (find-package "AUTO_NAVIGATION")))
(unless (find-package "AUTO_NAVIGATION::NAVIGATIONSTATUS")
  (make-package "AUTO_NAVIGATION::NAVIGATIONSTATUS"))

(in-package "ROS")
;;//! \htmlinclude NavigationStatus.msg.html
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass auto_navigation::NavigationStatus
  :super ros::object
  :slots (_header _status _current_goal _progress _distance_to_goal _estimated_time _error_message _is_executing ))

(defmethod auto_navigation::NavigationStatus
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:status __status) "")
    ((:current_goal __current_goal) "")
    ((:progress __progress) 0.0)
    ((:distance_to_goal __distance_to_goal) 0.0)
    ((:estimated_time __estimated_time) 0.0)
    ((:error_message __error_message) "")
    ((:is_executing __is_executing) nil)
    )
   (send-super :init)
   (setq _header __header)
   (setq _status (string __status))
   (setq _current_goal (string __current_goal))
   (setq _progress (float __progress))
   (setq _distance_to_goal (float __distance_to_goal))
   (setq _estimated_time (float __estimated_time))
   (setq _error_message (string __error_message))
   (setq _is_executing __is_executing)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:status
   (&optional __status)
   (if __status (setq _status __status)) _status)
  (:current_goal
   (&optional __current_goal)
   (if __current_goal (setq _current_goal __current_goal)) _current_goal)
  (:progress
   (&optional __progress)
   (if __progress (setq _progress __progress)) _progress)
  (:distance_to_goal
   (&optional __distance_to_goal)
   (if __distance_to_goal (setq _distance_to_goal __distance_to_goal)) _distance_to_goal)
  (:estimated_time
   (&optional __estimated_time)
   (if __estimated_time (setq _estimated_time __estimated_time)) _estimated_time)
  (:error_message
   (&optional __error_message)
   (if __error_message (setq _error_message __error_message)) _error_message)
  (:is_executing
   (&optional (__is_executing :null))
   (if (not (eq __is_executing :null)) (setq _is_executing __is_executing)) _is_executing)
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; string _status
    4 (length _status)
    ;; string _current_goal
    4 (length _current_goal)
    ;; float64 _progress
    8
    ;; float64 _distance_to_goal
    8
    ;; float64 _estimated_time
    8
    ;; string _error_message
    4 (length _error_message)
    ;; bool _is_executing
    1
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; string _status
       (write-long (length _status) s) (princ _status s)
     ;; string _current_goal
       (write-long (length _current_goal) s) (princ _current_goal s)
     ;; float64 _progress
       (sys::poke _progress (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _distance_to_goal
       (sys::poke _distance_to_goal (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; float64 _estimated_time
       (sys::poke _estimated_time (send s :buffer) (send s :count) :double) (incf (stream-count s) 8)
     ;; string _error_message
       (write-long (length _error_message) s) (princ _error_message s)
     ;; bool _is_executing
       (if _is_executing (write-byte -1 s) (write-byte 0 s))
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; string _status
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _status (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; string _current_goal
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _current_goal (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; float64 _progress
     (setq _progress (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _distance_to_goal
     (setq _distance_to_goal (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; float64 _estimated_time
     (setq _estimated_time (sys::peek buf ptr- :double)) (incf ptr- 8)
   ;; string _error_message
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _error_message (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _is_executing
     (setq _is_executing (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;;
   self)
  )

(setf (get auto_navigation::NavigationStatus :md5sum-) "762d278883490d8229d0a8c5e42445ba")
(setf (get auto_navigation::NavigationStatus :datatype-) "auto_navigation/NavigationStatus")
(setf (get auto_navigation::NavigationStatus :definition-)
      "# NavigationStatus.msg
# 导航状态消息

Header header

# 导航状态
string status  # \"idle\", \"navigating\", \"reached\", \"failed\", \"cancelled\"

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

")



(provide :auto_navigation/NavigationStatus "762d278883490d8229d0a8c5e42445ba")



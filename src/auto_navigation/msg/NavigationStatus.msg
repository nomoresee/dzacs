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
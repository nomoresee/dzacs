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
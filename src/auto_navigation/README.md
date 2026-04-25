# 自动导航节点包

## 功能概述

自动导航节点包提供了一套完整的自动导航解决方案，包括：

- **自动导航节点**：接收导航目标并自动执行
- **目标发布器**：发布导航目标
- **状态监控器**：监控导航状态
- **优先级队列**：支持多目标优先级排序
- **失败重试机制**：自动处理导航失败

## 文件结构

```
auto_navigation/
├── package.xml              # 包配置文件
├── CMakeLists.txt           # 编译配置文件
├── README.md               # 说明文档
├── msg/                    # 消息定义
│   ├── NavigationGoal.msg  # 导航目标消息
│   └── NavigationStatus.msg # 导航状态消息
├── scripts/                # Python脚本
│   ├── auto_navigation_node.py  # 主导航节点
│   ├── goal_publisher.py        # 目标发布器
│   └── status_monitor.py        # 状态监控器
├── launch/                 # 启动文件
│   └── auto_navigation.launch  # 主启动文件
└── config/                 # 配置文件
    └── navigation_config.yaml   # 导航配置
```

## 使用方法

### 1. 编译包

```bash
cd ~/dzacs
catkin build auto_navigation
source devel/setup.bash
```

### 2. 启动自动导航

```bash
# 启动自动导航系统
roslaunch auto_navigation auto_navigation.launch

# 或者手动启动各个组件
rosrun auto_navigation auto_navigation_node.py
rosrun auto_navigation goal_publisher.py
rosrun auto_navigation status_monitor.py
```

### 3. 发布导航目标

#### 方法1：使用目标发布器
```bash
# 启动目标发布器
rosrun auto_navigation goal_publisher.py

# 在交互界面中输入：
demo                    # 运行演示
goal 1.0 2.0          # 发布单个目标
goal 2.0 1.0 1.57 "目标点"  # 带方向和描述的目标
```

#### 方法2：使用rostopic
```bash
# 发布导航目标
rostopic pub /auto_navigation/goal auto_navigation/NavigationGoal "
header:
  stamp: now
  frame_id: map
goal_pose:
  header:
    stamp: now
    frame_id: map
  pose:
    position:
      x: 1.0
      y: 2.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0
goal_type: point
priority: 5
auto_execute: true
timeout: 60.0
description: '测试目标'
"
```

### 4. 监控导航状态

```bash
# 启动状态监控器
rosrun auto_navigation status_monitor.py

# 或者直接查看话题
rostopic echo /auto_navigation/status
```

### 5. 控制导航

```bash
# 取消导航
rostopic pub /auto_navigation/cancel std_msgs/Bool "data: true"

# 暂停导航
rostopic pub /auto_navigation/pause std_msgs/Bool "data: true"

# 恢复导航
rostopic pub /auto_navigation/pause std_msgs/Bool "data: false"
```

## 话题说明

### 订阅话题
- `/auto_navigation/goal` (NavigationGoal): 导航目标
- `/auto_navigation/cancel` (Bool): 取消导航
- `/auto_navigation/pause` (Bool): 暂停/恢复导航

### 发布话题
- `/auto_navigation/status` (NavigationStatus): 导航状态
- `/move_base_simple/goal` (PoseStamped): 发送给move_base的目标

## 参数配置

### 基本参数
- `auto_execute`: 是否自动执行目标 (默认: true)
- `goal_timeout`: 目标超时时间 (默认: 60.0秒)
- `max_retries`: 最大重试次数 (默认: 3)
- `goal_tolerance`: 目标容差 (默认: 0.1米)

### 启动参数
```bash
roslaunch auto_navigation auto_navigation.launch auto_execute:=true goal_timeout:=120.0
```

## 功能特性

### 1. 优先级队列
- 支持多目标优先级排序
- 高优先级目标优先执行
- 动态调整队列顺序

### 2. 失败重试机制
- 自动检测导航失败
- 支持多次重试
- 可配置重试次数和间隔

### 3. 状态监控
- 实时状态反馈
- 进度百分比显示
- 错误信息报告

### 4. 灵活控制
- 支持暂停/恢复
- 支持取消导航
- 支持手动控制

## 示例用法

### 1. 简单导航
```bash
# 启动系统
roslaunch auto_navigation auto_navigation.launch

# 发布目标
rostopic pub /auto_navigation/goal auto_navigation/NavigationGoal "
goal_pose:
  pose:
    position: {x: 1.0, y: 2.0, z: 0.0}
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
description: '测试目标'
priority: 5
auto_execute: true
"
```

### 2. 多目标导航
```bash
# 发布多个目标
for i in {1..5}; do
  rostopic pub /auto_navigation/goal auto_navigation/NavigationGoal "
  goal_pose:
    pose:
      position: {x: $i, y: $i, z: 0.0}
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  description: '目标$i'
  priority: $((6-$i))
  auto_execute: true
  "
  sleep 1
done
```

### 3. 监控和控制
```bash
# 启动监控器
rosrun auto_navigation status_monitor.py

# 在监控器中输入命令：
cancel    # 取消导航
pause     # 暂停导航
resume    # 恢复导航
status    # 查看状态
```

## 故障排除

### 1. 常见问题
- **move_base未启动**: 确保move_base节点正在运行
- **目标无法到达**: 检查目标点是否在地图范围内
- **导航超时**: 调整goal_timeout参数

### 2. 调试方法
```bash
# 查看节点状态
rosnode list
rosnode info /auto_navigation_node

# 查看话题数据
rostopic list | grep auto_navigation
rostopic echo /auto_navigation/status

# 查看日志
rosrun rqt_console rqt_console
```

## 扩展功能

### 1. 自定义消息
可以扩展NavigationGoal消息添加更多字段：
- 速度限制
- 路径类型
- 特殊行为

### 2. 集成其他系统
- 与视觉检测系统集成
- 与语音控制系统集成
- 与远程控制系统集成

### 3. 高级功能
- 路径记录和回放
- 动态障碍物避让
- 多机器人协调 
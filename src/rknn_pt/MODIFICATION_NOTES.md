# RKNN 检测节点逻辑修改说明

## 修改文件列表

| 文件 | 功能 | 修改位置 |
|------|------|----------|
| `rknn_pt/src/det_node.cc` | 靶子检测节点 | 第 117-153 行 |
| `rknn_pt/src/det_node_25.cc` | 数字识别节点 | 第 93-187 行 |
| `dzactuator/launch/dzrknnpt.launch` | 启动文件 | 全文 |

---

## 一、Launch 文件修改

### 1.1 修改位置
- **文件**: `dzactuator/launch/dzrknnpt.launch`

### 1.2 修改原因
原 launch 文件只启动了 `det_node`（靶子检测），需要同时启动 `det_node_25`（数字识别）

### 1.3 修改内容

**修改前**:
```xml
<launch>
    <node name="rknn_pt" pkg="rknn_pt" type="det_node"  />
</launch>
```

**修改后**:
```xml
<launch>
    <node name="rknn_pt_target" pkg="rknn_pt" type="det_node"  />
    <node name="rknn_pt_digit" pkg="rknn_pt" type="det_node_25"  />
</launch>
```

### 1.4 节点说明

| 节点名 | 可执行文件 | 功能 | 发布话题 |
|--------|-----------|------|----------|
| `rknn_pt_target` | det_node | 靶子检测（选置信度最高） | `/offset_center` |
| `rknn_pt_digit` | det_node_25 | 数字识别（拼接两位数） | `/det_25_results` |

---

## 二、det_node.cc 靶子检测节点

### 1.1 修改位置
- **文件**: `rknn_pt/src/det_node.cc`
- **原代码行**: 第 117-138 行
- **修改后行**: 第 117-153 行

### 1.2 修改原因
原代码遍历所有检测结果并发布多个目标，导致：
- 同一帧发布多条消息
- 置信度低的目标也可能被发布
- 后一个目标可能覆盖前一个

### 1.3 修改逻辑
```
每帧检测结果
    ↓
遍历所有检测框，找到置信度(box.prob)最高的一个
    ↓
发布该目标的坐标偏移量
    ↓
data[0] = offset_center_x  (目标中心x - 图像中心x)
data[1] = offset_center_y  (目标中心y - 图像中心y)
data[2] = 1                (检测到目标标志)
```

### 1.4 修改代码片段

```cpp
// 找到置信度最高的目标
int best_idx = -1;
float best_score = 0.0f;
int i = 0;
for (const auto &res : results_group.dets)
{
  if (res.box.prob > best_score)
  {
    best_score = res.box.prob;
    best_idx = i;
  }
  i++;
}

if (best_idx >= 0)
{
  const auto &best = results_group.dets[best_idx];
  int center_x = best.box.x + best.box.width / 2;
  int center_y = best.box.y + best.box.height / 2;

  int offset_center_x = center_x - width / 2;
  int offset_center_y = center_y - height / 2;

  msg.layout.dim[0].size = 2;
  msg.layout.dim[0].stride = 2;
  msg.layout.dim[0].label = "light center coordinate";
  msg.data[0] = offset_center_x;
  msg.data[1] = offset_center_y;
  msg.data[2] = 1;
}
det_pub.publish(msg);
```

---

## 二、det_node_25.cc 数字识别节点

### 2.1 修改位置
- **文件**: `rknn_pt/src/det_node_25.cc`
- **原代码行**: 第 93-113 行
- **修改后行**: 第 93-187 行

### 2.2 修改原因
原代码遍历所有检测结果并为每个目标发布一条消息，无法满足数字识别的以下需求：
- 两位数可能由两个独立的数字组成（如 "1" + "5" = "15"）
- 数字 10-25 可能直接作为一个类被检测到
- 需要智能选择和拼接数字

### 2.3 修改逻辑

```
检测结果分类
    ↓
┌─────────────────────────────────────────────────────────┐
│ 策略1: 存在 class 10-25 的检测框                        │
│   └─→ 选择置信度最高的那个，发布该数字                    │
├─────────────────────────────────────────────────────────┤
│ 策略2: 不存在 class 10-25，但有 2 个以上检测框           │
│   └─→ 取置信度最高的 2 个数字                            │
│   └─→ 按 x 坐标排序（左边是十位，右边是个位）             │
│   └─→ 拼接成两位数发布                                   │
├─────────────────────────────────────────────────────────┤
│ 策略3: 只有 1 个检测框                                   │
│   └─→ 发布该数字                                         │
├─────────────────────────────────────────────────────────┤
│ 策略4: 没有检测框                                        │
│   └─→ data[2] = 0（默认值）                              │
└─────────────────────────────────────────────────────────┘
```

### 2.4 修改代码片段

```cpp
// 用于存储 class 10-25 的检测结果
std::vector<const DetectionResult*> high_class_dets;
// 用于存储所有数字检测结果
std::vector<const DetectionResult*> all_dets;

for (const auto &res : results_group.dets)
{
  int class_id = atoi(res.det_name.c_str());
  all_dets.push_back(&res);
  
  // 过滤 class 10-25
  if (class_id >= 10 && class_id <= 25)
  {
    high_class_dets.push_back(&res);
  }
}

int final_number = 0;
int final_x = 0, final_y = 0;

// 策略1: 如果有 class 10-25，选取置信度最高的
if (!high_class_dets.empty())
{
  const DetectionResult* best = high_class_dets[0];
  float best_score = best->score;
  for (const auto det : high_class_dets)
  {
    if (det->score > best_score)
    {
      best_score = det->score;
      best = det;
    }
  }
  final_number = atoi(best->det_name.c_str());
  final_x = best->box.x + best->box.width / 2 - width / 2;
  final_y = best->box.y + best->box.height / 2 - height / 2;
}
// 策略2: 如果没有 class 10-25，取置信度前两个数字拼接
else if (all_dets.size() >= 2)
{
  // ... 取置信度最高的2个 ...
  // 按x坐标排序确定十位和个位
  // 拼接成两位数
}
// 策略3: 只有一个数字
else if (all_dets.size() == 1)
{
  final_number = atoi(all_dets[0]->det_name.c_str());
  final_x = all_dets[0]->box.x + all_dets[0]->box.width / 2 - width / 2;
  final_y = all_dets[0]->box.y + all_dets[0]->box.height / 2 - height / 2;
}
```

### 2.5 数字拼接示例

| 检测结果 | x坐标 | 置信度 | 输出数字 | 说明 |
|----------|-------|--------|----------|------|
| class 23 | 320 | 0.85 | **23** | 直接识别为23 |
| "1" | 200 | 0.9 | 15 | 两个数字拼接 |
| "5" | 350 | 0.8 | | |
| "3" | 300 | 0.95 | **3** | 只有1个数字 |
| "1" | 250 | 0.7 | | |
| "5" | 350 | 0.6 | | |
| "2" | 280 | 0.5 | 15 | 取最高2个拼接 |

---

## 三、发布消息格式

### det_node.cc (靶子检测)
```
话题名: /offset_center
类型: std_msgs/Int32MultiArray
长度: 3
data[0]: offset_center_x (目标x - 图像中心x)
data[1]: offset_center_y (目标y - 图像中心y)
data[2]: 1 (检测到目标) / 0 (未检测到)
```

### det_node_25.cc (数字识别)
```
话题名: /det_25_results
类型: std_msgs/Int32MultiArray
长度: 3
data[0]: offset_center_x (目标x - 图像中心x)
data[1]: offset_center_y (目标y - 图像中心y)
data[2]: final_number (识别的数字，如 15, 23 等)
```

---

## 四、编译

修改代码后需要重新编译：

```bash
cd ~/dzacs
catkin_make
# 或
catkin build rknn_pt
```

---

## 五、运行

```bash
# 终端1: 启动ROS
roscore

# 终端2: 启动相机
rosrun usb_cam usb_cam_node

# 终端3: 运行靶子检测
rosrun rknn_pt det_node

# 终端4: 运行数字识别
rosrun rknn_pt det_node_25
```

---

## 六、后续优化建议

### 6.1 添加置信度阈值过滤噪声
```cpp
float min_confidence = 0.5;  // 可调整
if (res.box.prob < min_confidence)
    continue;
```

### 6.2 添加目标类别过滤
如果只需要检测特定类别（如红靶/蓝靶）：
```cpp
if (res.det_name == "red_target")
{
    // 只处理红靶
}
```

### 6.3 添加多目标追踪
使用 Sort 或 DeepSort 算法实现目标稳定跟踪，避免置信度波动时目标跳动。

---

## 七、动态模式切换功能（预留）

### 7.1 功能说明
为了支持"导航到一点启动物资识别，到另一点启动打靶识别"的需求，预留了动态切换功能。

> **注意**: 当前功能已注释禁用，待导航系统完成后再启用。

### 7.2 预留的切换方式

#### 方式1：话题切换
发布 `std_msgs/Bool` 到对应话题：
```bash
# 切换到打靶模式
rostopic pub /det_node/detection_enable std_msgs/Bool "data: true"
rostopic pub /det_node_25/detection_enable std_msgs/Bool "data: false"

# 切换到物资识别模式
rostopic pub /det_node_25/detection_enable std_msgs/Bool "data: true"
rostopic pub /det_node/detection_enable std_msgs/Bool "data: false"
```

#### 方式2：参数切换
```bash
rosrun dynamic_reconfigure dynparam set /rknn_pt_target enable false
rosrun dynamic_reconfigure dynparam set /rknn_pt_digit enable true
```

#### 方式3：命令行启动时指定
```bash
# 打靶模式
roslaunch dzactuator dzrknnpt.launch mode:=target

# 物资识别模式
roslaunch dzactuator dzrknnpt.launch mode:=digit
```

### 7.3 导航集成代码示例
在导航控制节点中添加：
```cpp
// 订阅停止信号
ros::Subscriber stop_sub = nh.subscribe("/move_base/stop_signal", 1, callback_stop_signal);

// 切换检测模式
void callback_stop_signal(const std_msgs::UInt8::ConstPtr &msg)
{
    std_msgs::Bool enable_msg;
    std_msgs::UInt8 mode_msg;

    if (msg->data == TARGET_POINT)  // 打靶点
    {
        enable_msg.data = true;
        // 关闭数字识别
        mode_msg.data = 0;
    }
    else if (msg->data == MATERIAL_POINT)  // 物资点
    {
        enable_msg.data = true;
        // 关闭靶子检测
        mode_msg.data = 1;
    }
}
```

### 7.4 启用动态切换功能
当导航系统完成后，取消注释以下位置即可：

**det_node.cc:**
- 第 21 行：`bool g_enable_detect = true;`
- 第 39-41 行：`enableCallback` 函数
- 第 51 行：`ros::Subscriber enable_sub = ...`
- 第 60-61 行：`private_nh.param<bool>("enable", ...)`
- 第 149 行：`if (best_idx >= 0 && g_enable_detect)`

**det_node_25.cc:**
- 第 20 行：`bool g_enable_detect = true;`
- 第 36-40 行：`enableCallback` 函数
- 第 50 行：`ros::Subscriber enable_sub = ...`
- 第 62 行：`private_nh.param<bool>("enable", ...)`
- 第 194 行：`if (g_enable_detect)`

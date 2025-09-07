PX4飞控通过MAVROS连接到ROS系统，是实现无人机高阶控制（如Offboard模式）的核心技术。以下是具体步骤和关键要点：

---

### 🔧 **一、硬件连接方式**
1. **USB直连**  
   - 用USB线连接PX4飞控（如Pixhawk）与机载计算机（如Jetson Nano）。  
   - 终端检测设备：`ls /dev/ttyACM*`，确认识别到飞控（如`/dev/ttyACM0`）。  
   - **权限配置**：  
     ```bash
     sudo chmod 777 /dev/ttyACM0  # 赋予串口访问权限
     ```  
     避免权限冲突（否则MAVROS无法访问串口）。

2. **数传电台/串口连接**  
   - 飞控端：配置`TELEM2`端口为MAVLINK输出（参数设置：`MAV_1_CONFIG = TELEM2`，`SER_TEL2_BAUD = 921600`）。  
   - 机载计算机端：通过串口（如`/dev/ttyTHS1`）连接，启动命令：  
     ```bash
     roslaunch mavros px4.launch fcu_url:="serial:///dev/ttyTHS1:921600"
     ```  
     波特率需与飞控参数一致。

3. **网络连接（仿真或无线）**  
   - **仿真环境**（如Gazebo+SITL）：  
     ```bash
     roslaunch mavros px4.launch fcu_url:="udp://:14540@127.0.0.1:14557"
     ```  
     使用UDP协议模拟飞控通信。  
   - **WiFi中继**：  
     机载计算机通过WiFi连接地面站（QGC），飞控通过USB/串口连机载计算机：  
     ```bash
     roslaunch mavros px4.launch fcu_url:="/dev/ttyACM0:921600" gcs_url:="udp://@QGC_IP"
     ```  
     `QGC_IP`为地面站IP（需同局域网）。

---

### ⚙️ **二、软件配置与安装**
1. **安装MAVROS**  
   ```bash
   sudo apt-get install ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras
   ./install_geographiclib_datasets.sh  # 安装地理坐标系数据集
   ```  
   需匹配ROS版本（如`noetic`）。

2. **环境变量配置**  
   - **关键步骤**：在`~/.bashrc`中按顺序添加：  
     ```bash
     source /opt/ros/noetic/setup.bash         # 1. 初始化ROS
     source ~/catkin_ws/devel/setup.bash       # 2. 加载工作空间
     source ~/PX4-Autopilot/Tools/simulation/gazebo/setup_gazebo.bash  # 3. 添加PX4路径
     export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:~/PX4-Autopilot
     ```  
     顺序错误会导致路径覆盖（常见错误：`Resource not found: px4`）。

---

### 🚀 **三、启动与验证连接**
1. **启动MAVROS节点**  
   ```bash
   roslaunch mavros px4.launch _fcu_url:="/dev/ttyACM0:57600"
   ```  
   成功时终端显示飞控固件版本及心跳包。

2. **验证通信状态**  
   - 监听状态话题：  
     ```bash
     rostopic echo /mavros/state
     ```  
     检查`connected: True`（若为`False`则连接失败）。  
   - 查看高度信息：  
     ```bash
     rostopic echo /mavros/altitude
     ```  
     获取无人机实时高度（如`amsl`为海拔高度）。

---

### 💻 **四、高级应用：Offboard控制示例**
通过ROS节点控制无人机飞行（需先解锁并切换至Offboard模式）：
```cpp
// 初始化服务客户端（解锁&切模式）
ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

// 发布目标位置（ENU坐标系）
geometry_msgs::PoseStamped target_pose;
target_pose.pose.position.x = 2.0;  // 东向2米
target_pose.pose.position.y = 0.0;  // 北向0米
target_pose.pose.position.z = 2.0;  // 天向2米（离地高度）
local_pos_pub.publish(target_pose);
```
**坐标转换**：MAVROS自动将ROS的**ENU**（东-北-天）转为PX4的**NED**（北-东-地）坐标系，用户无需手动处理。

---

### ⚠️ **五、常见问题与解决**
| **问题**                | **原因**                          | **解决方案**                              |
|-------------------------|----------------------------------|------------------------------------------|
| **串口权限拒绝**         | 未正确配置`/dev/tty*`权限         | `sudo chmod 777 /dev/ttyACM0` |
| **连接QGC冲突**         | 串口被QGC占用                    | 关闭QGC后再启动MAVROS       |
| **环境变量覆盖**         | `source`顺序错误                 | 按ROS→工作空间→PX4顺序加载   |
| **Offboard模式拒绝切换** | 未持续发送目标指令（>2Hz）       | 循环发布指令（频率≥20Hz）   |

---

### 💎 **总结**
- **核心价值**：MAVROS作为协议转换层，打通了ROS与PX4的通信链路，支持**位置控制**、**状态监控**及**自主任务**开发。  
- **典型场景**：  
  - 仿真开发：UDP连接Gazebo+SITL  
  - 实机控制：串口/USB直连飞控  
  - 多机协同：WiFi中继连接地面站  
- **进阶提示**：开发Offboard应用时，务必注意**坐标系转换**和**指令频率**，避免飞控触发安全保护。
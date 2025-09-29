基于提供的代码，使用ego-planner和px4ctrl实现无人机操控的全流程如下：

## 🚀 系统启动与初始化流程

### 1. 系统组件启动顺序
```
PX4飞控 → MAVROS → px4ctrl → ego-planner → traj_server
```

### 2. 初始化阶段
**px4ctrl初始化：**
- 读取控制参数（PID增益、质量、最大速度等）
- 订阅MAVROS话题（状态、IMU、里程计、电池等）
- 等待RC遥控器信号（除非配置为无RC模式）
- 等待与PX4飞控连接

**ego-planner初始化：**
- 读取规划参数（最大速度、加速度、规划视野等）
- 初始化地图和优化器
- 根据配置选择目标输入方式（手动点击或预设航点）
- 等待触发信号和里程计数据

## 🎯 飞行模式切换流程

### 状态机转换流程：
```
MANUAL_CTRL → AUTO_HOVER → CMD_CTRL
```

### 详细步骤：

#### 步骤1：手动控制 → 自动悬停
```bash
# 通过RC遥控器切换
RC模式开关 → HOVER模式 → px4ctrl进入AUTO_HOVER状态
```

**条件检查：**
- ✓ 收到里程计数据
- ✓ 无人机速度 < 3.0m/s
- ✗ 没有正在接收控制指令

#### 步骤2：自动悬停 → 指令控制
```bash
# 通过RC遥控器切换
RC档位开关 → COMMAND模式 → px4ctrl进入CMD_CTRL状态
```

## 📍 任务规划与执行流程

### 方式1：手动目标模式
```
RVIZ点击目标点 → ego-planner接收目标 → 轨迹规划 → 执行
```

### 方式2：预设航点模式  
```
px4ctrl发送触发信号 → ego-planner读取预设航点 → 分段规划 → 顺序执行
```

### 详细规划执行流程：

#### 1. 目标接收阶段
```cpp
// ego-planner接收目标
waypointCallback() 或 triggerCallback()
↓
planNextWaypoint()  // 规划下一个航点
↓
changeFSMExecState(GEN_NEW_TRAJ)  // 状态切换
```

#### 2. 轨迹规划阶段
```cpp
// ego-planner进行轨迹规划
planFromGlobalTraj()  // 从全局轨迹规划
↓
callReboundReplan()   // 反弹重规划算法
↓
BsplineOptimizeTrajRebound()  // B样条优化
```

#### 3. 轨迹发布阶段
```cpp
// 发布规划结果
bspline_pub_.publish(bspline)        // → traj_server
swarm_trajs_pub_.publish(trajs)      // → 其他无人机（集群）
broadcast_bspline_pub_.publish(bspline) // → 广播
```

#### 4. 轨迹执行阶段
```cpp
// traj_server处理轨迹
bsplineCallback()  // 接收B样条轨迹
↓
cmdCallback()      // 定时发布位置指令
↓
pos_cmd_pub.publish(cmd)  // → px4ctrl
```

#### 5. 控制执行阶段
```cpp
// px4ctrl执行控制
process()  // 主处理循环
↓
calculateControl()  // 计算控制输出
↓
publish_attitude_ctrl()  // 发布姿态控制指令 → PX4
```

## 🔄 实时重规划流程

### 触发重规划的条件：
1. **位置偏差**：`(end_pt_ - pos).norm() > no_replan_thresh_`
2. **时间触发**：`t_cur > replan_thresh_`
3. **碰撞检测**：检测到障碍物或其他无人机
4. **航点切换**：到达当前航点，规划下一个航点

### 重规划过程：
```
检测到需要重规划 → changeFSMExecState(REPLAN_TRAJ)
↓
planFromCurrentTraj()  // 从当前轨迹重新规划
↓
callReboundReplan()    // 执行重规划
↓
发布新轨迹 → 继续执行
```

## 🛡️ 安全保护机制

### 1. 紧急停止
```cpp
// 检测到紧急情况
changeFSMExecState(EMERGENCY_STOP)
↓
callEmergencyStop()  // 紧急停止
↓
发布悬停轨迹
```

### 2. 碰撞检测
- **静态障碍物**：通过栅格地图检测
- **动态障碍物**：检测其他无人机轨迹
- **安全距离**：使用`getSwarmClearance()`确保安全间距

### 3. 超时保护
```cpp
// 各类消息超时检查
rc_is_received()    // RC信号
odom_is_received()  // 里程计
cmd_is_received()   // 控制指令
imu_is_received()   // IMU数据
```

## 🌐 多机协同流程

### 集群通信机制：
```
无人机N-1 → /drone_{N-1}_planning/swarm_trajs → 无人机N
↓
无人机N规划时考虑无人机N-1的轨迹
↓
无人机N发布自己的轨迹给无人机N+1
```

### 协同避障：
- 每架无人机广播自己的B样条轨迹
- 接收并解析其他无人机的轨迹
- 在规划时考虑集群轨迹避障
- 使用时间同步确保轨迹一致性

## 📊 系统特点总结

### 🎯 核心优势：
1. **分层架构**：规划、轨迹处理、控制分离，职责清晰
2. **状态机管理**：明确的状态转换，保证系统稳定性
3. **实时重规划**：应对环境变化和突发状况
4. **集群支持**：天然支持多机协同作业
5. **安全完备**：多重保护机制确保飞行安全

### 🔧 关键技术：
- **B样条轨迹**：平滑可导，适合无人机动力学
- **反弹重规划**：高效的局部轨迹优化
- **模型预测控制**：px4ctrl基于模型的控制算法
- **分布式通信**：基于ROS的集群通信机制

这个系统提供了一个完整、安全、高效的无人机自主导航解决方案，从单机到集群都有良好的扩展性。
# Ubuntu 20.04 ROS Noetic 安装指南

本指南详细说明在 Ubuntu 20.04 上安装 ROS Noetic 的完整步骤。

## 安装前准备

### 系统要求
- Ubuntu 20.04 (别名codename:Focal Fossa)

  <img src="images\ROS1与Ubuntu版本对应关系.png" alt="版本对应关系" />

- 确保系统已更新：
  ```bash
  sudo apt update
  ```

### 配置ROS软件源
使用国内清华源镜像加速下载：
```bash
sudo echo "deb http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ focal main" > /etc/apt/sources.list.d/ros-latest.list

# deb: 表示标准软件仓库。

# http://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/: 清华大学提供的 ROS 软件源镜像地址。

# focal: Ubuntu 20.04 的代号（Focal Fossa），此处固定写死，因此该命令仅适用于 20.04。

# main: 软件仓库的分支类型。

# /etc/apt/sources.list.d/ 用于存放第三方源的独立配置文件，避免修改主文件 /etc/apt/sources.list
```

## 核心安装步骤

### 1. 添加ROS密钥
```bash
# 直接下载密钥
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# 可能报错，可能是GitHub 的 raw 服务暂时不可用
gpg: no valid OpenPGP data found

# 更换为官方推荐的密钥服务器导入方式，绕过 GitHub 直接通过 Ubuntu 密钥服务器添加：
sudo apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
```

### 2. 更新软件包列表
```bash
sudo apt update
```

### 3. 安装ROS Noetic
```bash
# 安装完整桌面版
sudo apt install ros-noetic-desktop-full
```

### 4. 初始化rosdep

rosdep命令可以用来安装功能包的依赖项

rosdep install --from-paths your_package_directory --ignore-src

```bash
# 安装依赖
sudo apt install python3-rosdep2

# 初始化
sudo rosdep init

# 可能报错
ERROR: default sources list file already exists:
	/etc/ros/rosdep/sources.list.d/20-default.list
Please delete if you wish to re-initialize

# 删除对应文件后再初始化
sudo rm /etc/ros/rosdep/sources.list.d/20-default.list
sudo rosdep init

# 更新rosdep
rosdep update

# 可能报错
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml]:
	<urlopen error _ssl.c:1128: The handshake operation timed out> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/base.yaml)
ERROR: unable to process source [https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml]:
	<urlopen error _ssl.c:1128: The handshake operation timed out> (https://raw.githubusercontent.com/ros/rosdistro/master/rosdep/python.yaml)

# 是由于国内网络无法访问一些网站，可以设置国内镜像源
# 步骤1：修改20-default.list源文件，内容如下
# os-specific listings first
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/osx-homebrew.yaml osx

# generic
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/base.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/python.yaml
yaml https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/rosdep/ruby.yaml
gbpdistro https://mirrors.tuna.tsinghua.edu.cn/github-raw/ros/rosdistro/master/releases/fuerte.yaml fuerte

# newer distributions (Groovy, Hydro, ...) must not be listed anymore, they are being fetched from the rosdistro index.yaml instead

# 步骤2：设置rosdistro索引镜像
echo "export ROSDISTRO_INDEX_URL=https://mirrors.tuna.tsinghua.edu.cn/rosdistro/index-v4.yaml" >> ~/.bashrc
source ~/.bashrc

```


### 5. 设置环境变量
```bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# source命令通常用于重新执行刚修改的初始化文件，使之立即生效，而不必注销并重新登录
source ~/.bashrc
```

### 6. 安装构建工具rosinstall
```bash
sudo apt install python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
```

build-essential：提供编译ROS包的能力（gcc, g++, make）。

python3-rosinstall, python3-wstool, python3-rosinstall-generator：提供管理和获取ROS包源代码的能力,rosinstall 是一个用于管理 ROS 工作空间中多个软件包源代码的关键命令。它通过读取 .rosinstall 文件（包含源代码仓库信息,由rosinstall-generator生成）来自动检出、更新或管理多个包的源代码

### 7. 安装roslaunch
```bash
sudo apt install ros-noetic-roslaunch
```

## 安装验证

### 测试ROS核心服务
```bash
# 启动roscore
roscore
```
正常启动后应显示：
```bash
... logging to /home/user/.ros/log/xxxxx.log
Checking log directory for disk usage. This may take a while.
Press Ctrl-C to interrupt
Done checking log file disk usage. Usage is <1GB.

started core service [/rosout]
```
```bash
# 安装rosrun命令
sudo apt install ros-noetic-rosbash
```

### 测试小海龟仿真器
1. 启动仿真器节点（新终端）：
```bash
sudo apt install ros-noetic-turtlesim

rosrun turtlesim turtlesim_node
 ```
2. 启动控制节点（新终端）：
```bash
rosrun turtlesim turtle_teleop_key
```
3. 使用键盘方向键（↑↓←→）控制海龟移动

# MARVROS

MAVROS（MAVLink extendable communication node for ROS）是连接ROS（Robot Operating System）与无人机飞控系统（如PX4、ArduPilot）的开源通信中间件，通过MAVLink协议实现ROS与无人机硬件的高效数据交互。

MAVROS作为ROS与MAVLink协议的翻译层，将ROS话题（Topics）和服务（Services）转换为无人机飞控可识别的MAVLink消息，实现双向通信：

控制指令下发：将ROS发布的姿态、位置指令转为MAVLink消息发送至飞控13。

状态数据上报：接收飞控的传感器数据（IMU、GPS、电池状态等），以ROS话题形式发布供上层算法使用。

```bash
sudo apt-get install ros-noetic-mavros
#执行地理库安装脚本
cd /opt/ros/noetic/lib/mavros
sudo ./install_geographiclib_datasets.sh
```
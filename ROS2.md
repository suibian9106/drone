# ROS2
## ROS2 与 ROS1
ROS2和ROS1架构
<img src="images/ROS2和ROS1架构.png">

**Nodelet API** 和 **Intra-process API** 是进程内以共享内存方式提供的通讯方法

**DDS（Data Distribution Service）**，即**数据分发服务**，是一个由**对象管理组织（OMG）** 发布的**开放式国际标准**。它是一种专为**高性能、实时、分布式**系统设计的**以数据为中心**的通信中间件协议和API标准。

可以把它想象成一个**全球性的、分布式的“数据总线”**。应用程序不需要知道其他应用程序在哪里（IP地址、端口等），它们只需要**关心自己需要什么数据（订阅）或产生什么数据（发布）**。DDS负责高效、可靠、及时地将数据从生产者传递到所有需要它的消费者。

ROS1命令行
<img src="images/ROS1命令行.png">

ROS2命令行
<img src="images/ROS2命令行.png">

## ROS2 安装
* **设置编码**
```bash
$ sudo apt update && sudo apt install locales
$ sudo locale-gen en_US en_US.UTF-8
$ sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 
$ export LANG=en_US.UTF-8
```
* **添加源**
```bash
$ sudo apt update && sudo apt install curl gnupg lsb-release 
$ sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg 
$ echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

* **安装ROS2**
```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install ros-foxy-desktop
```
* **设置环境变量**
```bash
sudo vim ~/.bashrc 
##在文件最后添加以下内容，使ROS1和ROS2共存
unset ROS_DISTRO
unset ROS_PACKAGE_PATH
unset ROS_VERSION
echo "ros noetic(1) or ros2 foxy(2)?"
read edition
if [ "$edition" -eq "1" ];then
  source /opt/ros/noetic/setup.bash
else
  source /opt/ros/foxy/setup.bash
fi
```

## ROS2 构建工具——colcon
* **创建工作空间**
```bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src
```

* **功能包**
  ```bash
  #创建功能包，功能包包含源代码
  cd ~/dev_ws/src
  ros2 pkg create --build-type ament_cmake learning_pkg_c #C++
  ros2 pkg create --build-type ament_python learning_pkg_python #Python
  ```

  * C++功能包必然存在两个文件：`package.xml`和`CMakerLists.txt`
  * `package.xml`文件的主要内容如下，包含功能包的版权描述，和各种依赖的声明
  * `CMakerLists.txt`文件是编译规则

* **编译**
```bash
cd ~/dev_ws/
colcon build
```

* **设置环境变量**
```bash
source install/local_setup.sh # 仅在当前终端生效
echo " source ~/dev_ws/install/local_setup.sh" >> ~/.bashrc # 所有终端均生效
```

## 通信方式——话题、服务、动作、参数


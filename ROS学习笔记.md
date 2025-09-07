# ROS基础知识学习笔记

## ROS基本概念

ROS（Robot Operating System）的核心概念构建了一个强大且灵活的框架，用于开发复杂的机器人软件。理解这些核心概念是有效使用ROS的关键。以下是ROS核心概念的详细说明：

1.  **节点（Nodes）**
    *   **概念：** 节点是ROS中**执行计算的基本单元**。每个节点通常负责机器人系统中的一项特定、细粒度的功能（例如：读取传感器数据、控制电机、执行路径规划、处理图像等），节点在系统中的名称必须唯一。
    *   **特点：**
        *   **轻量级：** 节点设计得尽可能轻量和小巧。
        *   **独立进程：** 每个节点在操作系统中通常作为一个独立的进程运行。
        *   **可执行文件：** 节点是编译后的可执行文件（通常是C++或Python程序）。
        *   **分布式：** 节点可以运行在同一台机器上，也可以分布在网络中的不同机器上，共同组成一个机器人系统。
        *   **可组合性：** 通过添加或删除节点，可以轻松地修改或扩展机器人系统的功能。
    *   **作用：** 节点是**功能的载体**，它们通过ROS的通信机制进行交互，共同完成机器人的任务。

2.  **主节点、节点管理器（ROS Master）**
    *   **概念：** 主节点是ROS系统的**控制中心**。它像一个服务目录或名称服务器。
    *   **功能：**
        *   **注册：** 节点在启动时向主节点注册自己的信息（如节点名称、发布/订阅的话题、提供的服务等）。
        *   **查找：** 节点通过主节点查找其他节点及其提供的通信渠道（话题、服务、动作）。
        *   **连接中介：** 当节点A想与节点B通信时（例如订阅B发布的话题），主节点帮助A和B直接建立TCP（或UDP，较少用）连接（peer-to-peer）。一旦连接建立，数据流通常不再经过主节点。主节点没有机制去监听、记录、统计或干预A和B之间传输的消息内容、频率、大小或状态（如是否成功送达）。它不知道A是否真的收到了B的消息，也不知道消息传输是否失败。主节点只知道最初的注册信息：B在发布/sensor_data，A订阅了它。它知道这个连接曾经被请求建立过。
    *   **重要性：** 主节点是ROS网络正常工作的**前提**。在启动任何节点之前，必须先启动`roscore`命令（它启动了主节点以及一些其他基础服务）。

3.  **话题（Topics）**
    *   **概念：** 话题是ROS中实现**异步、单向、多对多**通信的主要机制。它遵循**发布/订阅（Publish/Subscribe）模型**。
    *   **机制：**
        *   **发布者（Publisher）：** 一个节点可以将消息（Message）**发布**到一个命名的话题上。
        *   **订阅者（Subscriber）：** 一个或多个节点可以**订阅**同一个话题。当有新消息发布到该话题时，所有订阅该话题的节点都会**异步地**接收到该消息的副本。
        *   **消息（Message）：** 在话题上传输的数据结构，有预定义的类型（如`sensor_msgs/Image`表示图像，`geometry_msgs/Twist`表示速度指令）和用户自定义类型。
    *   **特点：**
        *   **异步：** 发布者和订阅者彼此不知道对方的存在，也不需要同时运行。发布者只管发布消息，订阅者只在有新消息到达时被回调。
        *   **单向：** 数据流是单向的，从发布者到订阅者。
        *   **多对多：** 一个话题可以有多个发布者，也可以有多个订阅者。
    *   **典型用途：** 流式数据的传输，如传感器数据（激光扫描、摄像头图像、IMU数据）、连续的状态信息（机器人位姿）、控制指令流（速度命令）。

4.  **消息（Messages）**
    *   **概念：** 消息是节点之间通过话题（或服务、动作）进行**通信时所交换的数据**。
    *   **结构：** 消息是**强类型**的数据结构，由字段组成。字段可以是基础类型（整数、浮点数、布尔值、字符串等）或其他消息类型（嵌套）。
    *   **定义：** 消息类型在`.msg`文件中定义（位于包的`msg`目录下）。例如：
        ```
        # sensor_msgs/Image.msg
        Header header          # 时间戳和坐标系信息
        uint32 height          # 图像高度（像素）
        uint32 width           # 图像宽度（像素）
        string encoding        # 像素编码格式 (e.g., "rgb8", "mono16")
        uint8 is_big_endian    # 字节序
        uint32 step            # 一行数据的字节长度
        uint8[] data           # 实际的图像数据（一维数组）
        ```
    *   **作用：** 消息定义了通信数据的**格式和语义**，是节点间相互理解的“语言”。

5.  **服务（Services）**
    *   **概念：** 服务是ROS中实现**同步、双向、请求/响应**式通信的机制。它遵循**客户端/服务器（Client/Server）模型**。
    *   **机制：**
        *   **服务端（Service Server）：** 一个节点提供一个命名服务，并指定一个**回调函数**来处理传入的请求。
        *   **客户端（Service Client）：** 一个节点向服务端发送一个**请求消息**并**阻塞等待**（或异步回调）服务端返回的**响应消息**。
    *   **特点：**
        *   **同步（通常）：** 客户端发出请求后会等待响应，在此期间通常是阻塞的（也可以异步回调）。
        *   **双向：** 通信包含一次请求和一次响应的往返。
        *   **一对一：** 一个服务通常由一个服务端提供，但可以有多个客户端调用它。服务端每次处理一个请求。
    *   **典型用途：** 执行需要明确结果的一次性操作或计算，例如：请求当前位置、调用一次路径规划计算、开关某个硬件设备、查询状态。适用于不频繁的、需要即时响应的任务。

6.  **参数服务器（Parameter Server）**
    *   **概念：** 参数服务器是一个**共享的、全局的键值存储**，通常由ROS主节点托管。
    *   **功能：**
        *   节点可以在参数服务器上**设置（set）** 和**获取（get）** 参数。
        *   参数可以是基本数据类型（整型、浮点型、布尔型、字符串）或列表、字典。
    *   **用途：**
        *   存储配置参数（如机器人尺寸、控制器增益、传感器校准参数）。
        *   在运行时动态调整节点行为（通过`rosparam`命令行工具或节点API）。
        *   在启动文件（`.launch`）中设置参数，供多个节点使用。
    *   **注意：** 虽然方便，但参数服务器不适合存储大量或需要频繁更新的数据（使用话题更好）。

7.  **功能包（Packages）**
    *   **概念：** 功能包是ROS代码的**基本组织和分发单位**。它是ROS软件的主要构建块。
    *   **内容：** 一个功能包通常包含：
        *   实现特定功能的代码（C++/Python节点）。
        *   编译配置文件（`CMakeLists.txt`、`package.xml`）。
        *   消息定义（`.msg`）、服务定义（`.srv`）。
        *   配置文件（`.yaml`）。
        *   启动文件（`.launch`或`.launch.py`）。
        *   文档、资源（图像、模型）等。
    *   **作用：**
        *   **模块化：** 将相关功能组织在一起。
        *   **复用：** 方便代码的共享和复用。
        *   **依赖管理：** `package.xml`文件明确声明包对其他ROS包（或系统库）的依赖。
        *   **构建：** ROS构建系统知道如何编译和安装功能包。
        *   ROS中的package是catkin编译的基本单元，我们调用 catkin_make 编译的对象就是一个个ROS的package，也就是说任何ROS程序只有组织成package才能编译 。

8.  **元功能包（Metapackages）**
    *   **概念：** 元功能包是一种特殊的功能包，它本身**不包含任何代码或可执行文件**。
    *   **目的：** 主要用于**逻辑上分组相关功能包**，方便用户安装或引用一组包。
    *   **结构：** 它只包含一个`package.xml`文件（其中包含特殊的`<export><metapackage/>...</export>`标签）和一个`CMakeLists.txt`文件（通常只调用`find_package(catkin REQUIRED)`并声明为`metapackage`）。

9. **计算图（Computation Graph）**
    *   **概念：** 计算图是一个**动态的、运行时**的概念，它表示当前正在运行的ROS节点、话题和服务之间的**连接关系**。
    *   **可视化：** 可以使用`rqt_graph`工具可视化当前运行的ROS系统的计算图。图中节点代表ROS节点，边代表节点之间通过话题、服务或动作建立的通信连接。
    *   **作用：** 计算图是理解和调试ROS应用程序**数据流和节点交互**的极其重要的工具。它能清晰地展示哪些节点在发布什么话题，哪些节点订阅了哪些话题，服务调用关系等。

## ROS常用命令

1.  **核心与信息 (`roscore` / `rosnode`)**
    *   `roscore`: **启动ROS Master节点管理器**。任何ROS系统的起点。
    *   `rosnode list`: 列出当前所有正在运行的节点。
    *   `rosnode info <node_name>`: 查看指定节点的详细信息（发布/订阅的话题、提供的服务、连接的其他节点等）。**非常常用！**
    *   `rosnode ping <node_name>`: 测试与指定节点的连通性。
    *   `rosnode kill <node_name>`: 终止指定节点。

2.  **话题操作 (`rostopic`)**
    *   `rostopic list`: 列出当前所有活跃的话题。**非常常用！**
    *   `rostopic echo <topic_name>`: **实时显示**指定话题上发布的消息内容。**调试利器！**
    *   `rostopic info <topic_name>`: 查看指定话题的信息（类型、发布者、订阅者）。
    *   `rostopic type <topic_name>`: 查看指定话题的消息类型。
    *   `rostopic pub <topic_name> <msg_type> <args>`: **向指定话题发布一条消息**。用于手动测试订阅节点。例如：`rostopic pub /cmd_vel geometry_msgs/Twist "linear: {x: 0.1, y: 0.0, z: 0.0} angular: {x: 0.0, y: 0.0, z: 0.0}"`
    *   `rostopic hz <topic_name>`: 报告指定话题的发布频率（Hz）。
    *   `rostopic bw <topic_name>`: 报告指定话题的带宽使用量（Bytes/s）。

3.  **服务操作 (`rosservice`)**
    *   `rosservice list`: 列出当前所有可用的服务。
    *   `rosservice info <service_name>`: 查看指定服务的信息（类型、提供者）。
    *   `rosservice type <service_name>`: 查看指定服务的请求/响应消息类型。
    *   `rosservice call <service_name> <args>`: **调用指定的服务**。例如：`rosservice call /gazebo/reset_simulation` 或 `rosservice call /spawn "x: 1.0 y: 2.0"` (需要根据服务类型填入正确参数)。

4.  **参数服务器操作 (`rosparam`)**
    *   `rosparam list`: 列出参数服务器上的所有参数。
    *   `rosparam get <param_name>`: 获取指定参数的值。
    *   `rosparam set <param_name> <value>`: **设置指定参数的值**。常用于运行时调整配置。
    *   `rosparam dump <file.yaml>`: 将所有参数**保存**到YAML文件。
    *   `rosparam load <file.yaml>`: 从YAML文件**加载**参数到参数服务器。

5.  **包管理和构建 (`rospack` / `catkin`)**
    *   `rospack find <package_name>`: 查找指定包的路径。
    *   `rospack list`: 列出所有的ROS功能包（包括下载的包和自己创建的包）。
    *   `rospack depends <package_name>`: 查看指定包的依赖关系。
    *   `catkin_create_pkg <pkg_name> [dependencies]`: 在`src`目录下创建一个新的ROS包。
    *   `catkin_make`: 在`catkin`工作空间的根目录下**编译**所有包（传统方式）。
    *   `catkin build` (推荐): 在`catkin`工作空间的根目录下**编译**所有包（`catkin_tools`提供，更模块化）。

6.  **运行节点 (`rosrun` / `roslaunch`)**
    *   `rosrun <package_name> <executable_name>`: **运行**指定包中的一个可执行节点。例如：`rosrun turtlesim turtlesim_node`
    *   `roslaunch <package_name> <launch_file.launch>`: **运行**一个launch文件。launch文件用于一次性启动多个节点、设置参数等，是ROS1中组织复杂系统的主要方式。**极其常用！** 例如：`roslaunch turtlebot3_bringup turtlebot3_robot.launch`

7.  **其他实用工具**
    *   `rqt`: 启动**强大的可视化工具套件**，包含各种插件（节点图、话题监视器、参数编辑器、数据绘图等）。`rqt_graph` (显示节点和话题连接关系) 尤其常用。
    *   `rviz`: **3D可视化工具**，用于显示传感器数据（点云、图像、激光雷达）、机器人模型、TF变换等。**核心调试和展示工具！**
    *   `rosmsg` / `rossrv`: 查看消息(`.msg`)和服务(`.srv`)的定义（`show`, `list`, `md5`, `package`等子命令）。
    *   `rosbag record <topic1> [topic2 ...]`: **录制**指定话题的数据到bag文件。
    *   `rosbag play <bagfile.bag>`: **回放**bag文件中的数据。


## 工作空间（Workspace）与 Catkin 使用流程详解

### 一、工作空间（Workspace）
**工作空间是ROS开发的核心环境**，是一个包含源码、编译输出和开发环境的目录结构。它是管理、构建和测试ROS功能包的专用场所。

#### 核心目录结构：
```
catkin_workspace/      ← 工作空间根目录（自定义名称）
├── build/             ← **编译中间文件**（CMake缓存、日志等）
├── devel/             ← **开发空间**（编译结果：可执行文件/库/环境脚本）
└── src/               ← **源码空间**（存放所有功能包源码）
    ├── CMakeLists.txt ← Catkin的入口文件（由`catkin_init_workspace`生成）
    └── my_package/    ← 功能包，可多个，但不能重名
        ├── CMakeLists.txt  ← 已配置了多数编译选项，且包含详细的注释，只需稍作修改便可编译自己的文件。
        └── package.xml     ← 描述功能包清单的文件，包括功能包的名称、版本号、作者信息、许可信息、编译依赖和运行依赖等
```

---

### 二、Catkin 使用流程

#### 1：创建工作空间
```bash
mkdir ~/catkin_ws       # 创建工作空间目录
cd ~/catkin_ws
mkdir /src              # 创建src目录
cd src
catkin_init_workspace   # 初始化工作空间（在src文件夹生成顶层CMakeLists.txt）
```

#### 2：创建/添加功能包
```bash
# 在src目录下创建新包（依赖roscpp和std_msgs）
$catkin_create_pkg <package_name> [depend1][depend2][depend3]...
catkin_create_pkg my_package roscpp std_msgs

# 或添加现有包：
git clone https://github.com/user/package.git ~/catkin_ws/src/
```

* 在功能包的src目录下编写自己的代码文件,若include ros相关包时提示找不到源文件，可以先进行下面步骤4、5后，再在includePath中添加`/opt/ros/noetic/include`,最后重启窗口

#### 3：安装依赖和修改Cmake文件
* 在功能包的package.xml文件中添加未添加的功能包依赖
* 在功能包的CMakeLists.txt文件中补全find_package语句,即使用的额外的功能包;并在对应位置添加`add_executable()`,`target_link_libraries()`两个语句
    * `catkin_package()`语句为其他人要使用你功能包时需要完善的,`include_directories()`语句为该功能包需要的头文件路径

通常节点,代码文件,生成的可执行文件这三者命名相同,因为rosrun使用的是可执行文件的名字,而节点的名字是在代码中节点初始化命名的

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -y  # 自动安装package.xml中声明的所有依赖
```

#### 4：构建工作空间
```bash
cd ~/catkin_ws
catkin_make                # 构建、编译
```

#### 5：激活环境
```bash
source devel/setup.bash    # 使当前终端识别工作空间内的包
# 永久生效：将命令添加到 ~/.bashrc
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
```

#### 6：验证与使用
```bash
# 查看工作空间包
echo $ROS_PACKAGE_PATH     # 检查是否包含你的工作空间路径

#/home/suibian9106/catkin_ws/src 是自己工作空间功能包的路径
#/opt/ros/noetic/share 是安装的ROS功能包路径
/home/suibian9106/catkin_ws/src:/opt/ros/noetic/share


# 运行节点（假设包内有名为talker的节点）
rosrun my_package talker

# 或通过launch文件启动
roslaunch my_package demo.launch
```

#### 7. 生成安装包
```bash
catkin_make install        # 生成install目录（类似系统ROS安装结构）,用于部署
```

---

## launch文件知识
# ROS Launch 文件详解

## 一、Launch 文件的核心作用

Launch 文件是 ROS 中用于**启动多个节点和配置复杂系统**的关键工具，主要解决以下问题：
1. **批量启动**：同时启动多个相关节点（如传感器、控制、算法节点）
2. **参数管理**：统一设置和加载参数到参数服务器
3. **命名空间**：管理节点/话题的命名空间，避免冲突
4. **环境配置**：设置环境变量、重映射等配置
5. **条件控制**：实现条件启动和参数化启动

## 二、Launch 文件基本结构

### XML 格式基础
```xml
<launch>
    <!-- 配置内容 -->
    <node ... />
    <param ... />
    <include ... />
</launch>
```

### 执行方式
```bash
roslaunch <package_name> <launch_file_name>
```

## 三、核心元素详解

### 1. 节点启动（`<node>`）
```xml
<node 
    pkg="package_name"       <!-- 功能包名 -->
    type="executable_name"   <!-- 可执行文件名 -->
    name="node_name"         <!-- 运行时节点名（可覆盖代码中名称） -->
    output="screen"          <!-- 输出到屏幕而非日志文件 -->
    respawn="true"           <!-- 节点退出时自动重启 -->
    required="true"          <!-- 此节点失败则终止整个launch -->
    ns="namespace"           <!-- 命名空间 -->
    args="arg1 arg2"         <!-- 传递给节点的参数 -->
>
    <!-- 内部可包含子元素 -->
</node>
```

### 2. 参数设置（`<param>` & `<rosparam>`）
```xml
<!-- 设置单个参数 -->
<param name="param_name" type="str|int|double|bool" value="value" />

<!-- 从YAML文件加载参数 -->
<rosparam command="load" file="$(find pkg)/config/params.yaml" />

<!-- 直接在launch中定义YAML内容 -->
<rosparam command="load">
    camera:
        resolution: 1920x1080
        fps: 30
    motor:
        max_speed: 1.5
</rosparam>
```

### 3. 包含其他Launch文件（`<include>`）
```xml
<include file="$(find another_pkg)/launch/other.launch">
    <!-- 传递参数到被包含的launch -->
    <arg name="arg_in_other" value="$(arg local_arg)" />
</include>
```

### 4. 参数传递（`<arg>`）
```xml
<!-- 声明参数（可设默认值） -->
<arg name="simulation" default="true" />

<!-- 使用参数 -->
<node pkg="control" type="controller" if="$(arg simulation)">
    <!-- ... -->
</node>

<!-- 命令行覆盖参数 -->
<!-- roslaunch my_pkg launch_file.launch simulation:=false -->
```

## 四、高级功能

### 1. 重映射（Remapping）
```xml
<node pkg="image_processing" type="camera_node" name="camera">
    <!-- 将节点内的 /image_raw 重映射为 /camera/image_raw -->
    <remap from="image_raw" to="camera/image_raw" />
</node>
```

### 2. 条件启动
```xml
<!-- if/unless 条件控制 -->
<node pkg="sensors" type="lidar" if="$(arg use_lidar)" />
<node pkg="sensors" type="radar" unless="$(arg use_lidar)" />

<!-- 基于环境变量 -->
<node pkg="nav" type="planner" if="$(optenv USE_ADVANCED_NAV false)" />
```

### 3. 分组命名空间（`<group>`）
```xml
<!-- 为多个节点创建共同命名空间 -->
<group ns="robot1">
    <node pkg="control" type="driver" name="motor_driver" />
    <node pkg="sensors" type="encoder" name="wheel_encoder" />
</group>
```

### 4. 环境变量设置
```xml
<node pkg="gpu_processing" type="vision_node">
    <env name="CUDA_VISIBLE_DEVICES" value="0" />
</node>
```

教程完毕,现在你可以去编写ego-planner了


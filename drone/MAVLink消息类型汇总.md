# MAVLink消息统计与分类整理

## 一、MAVLink消息统计表格（按ID递增排序）

| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 0 | HEARTBEAT | 显示系统或组件存在并响应，通过类型和自动驾驶仪字段帮助接收系统正确处理后续消息。 |
| 1 | SYS_STATUS | 提供系统一般状态，包括系统模式（LOCKED、MANUAL等）、导航状态（LIFTOFF、LANDING等）和系统状态（活跃、紧急等），以及传感器状态、电池信息等。 |
| 2 | SYSTEM_TIME | 提供主时钟时间（UNIX时间和系统启动时间），用于日志时间戳同步，支持低可靠性时间源匹配高可靠性时间源。 |
| 4 | PING [DEP] | 用于测量系统延迟（包括串口、无线电调制解调器等），已被TIMESYNC替代。 |
| 5 | CHANGE_OPERATOR_CONTROL | 请求控制MAV，包含目标系统、控制请求类型、版本和密钥信息。 |
| 6 | CHANGE_OPERATOR_CONTROL_ACK | 接受或拒绝控制请求，包含GCS系统ID、控制请求类型和确认结果（ACK/NACK）。 |
| 7 | AUTH_KEY | 发送加密签名/密钥以标识系统，需通过加密通道传输以确保安全。 |
| 8 | LINK_NODE_STATUS [WIP] | 通信链中每个节点的状态，包括发送/接收缓冲区剩余空间、传输速率、解析错误和溢出计数等。 |
| 11 | SET_MODE [DEP] | 设置系统模式，已被MAV_CMD_DO_SET_MODE替代，无目标组件ID（模式针对整个飞行器）。 |
| 20 | PARAM_REQUEST_READ | 请求读取板载参数，通过参数ID或索引指定，支持跨组件参数查询。 |
| 21 | PARAM_REQUEST_LIST | 请求组件的所有参数列表，触发组件发送所有参数的PARAM_VALUE消息。 |
| 22 | PARAM_VALUE | 发送板载参数值，包含参数ID、值、类型、总参数数和索引，帮助接收方跟踪参数接收状态。 |
| 23 | PARAM_SET | 设置参数值并写入永久存储，接收方需广播PARAM_VALUE确认，发送方超时未收到需重发。 |
| 24 | GPS_RAW_INT | 提供GPS原始位置数据（非系统估计值），包括经纬度、海拔、精度因子、速度和可见卫星数等。 |
| 25 | GPS_STATUS | 报告GPS定位状态，包含可见卫星数量及每个卫星的PRN、使用状态、仰角、方位角和信噪比。 |
| 26 | SCALED_IMU | 提供9DOF传感器的缩放IMU读数（加速度、角速度、磁场强度），单位分别为mG、mrad/s、mgauss。 |
| 27 | RAW_IMU | 提供9DOF传感器的原始IMU读数（未缩放），用于数据捕获和系统调试。 |
| 28 | RAW_PRESSURE | 提供原始压力传感器读数（未缩放），包括绝对压力、差分压力和温度的ADC原始值。 |
| 29 | SCALED_PRESSURE | 提供压力传感器的缩放读数，包括绝对压力（hPa）、差分压力（hPa）和温度（cdegC）。 |
| 30 | ATTITUDE | 报告飞行器姿态，包括滚转、俯仰、偏航角（rad）及对应的角速度（rad/s）。 |
| 31 | ATTITUDE_QUATERNION | 用四元数（w,x,y,z）表示飞行器姿态，及滚转、俯仰、偏航角速度（rad/s），支持姿态偏移修正。 |
| 32 | LOCAL_POSITION_NED | 报告滤波后的本地位置（NED坐标系），包括x、y、z位置（m）和对应的速度（m/s）。 |
| 33 | GLOBAL_POSITION_INT | 报告滤波后的全球位置（GPS坐标系），经纬度以degE7、海拔以mm表示，包含地速和航向。 |
| 34 | RC_CHANNELS_SCALED | 报告缩放后的RC通道值，范围为-10000（-100%）至10000（100%）， inactive通道设为INT16_MAX。 |
| 35 | RC_CHANNELS_RAW | 报告原始RC通道值（微秒），标准PPM调制下1000μs为0%、2000μs为100%，未使用通道设为UINT16_MAX。 |
| 36 | SERVO_OUTPUT_RAW | 报告舵机输出原始值（微秒），已被ACTUATOR_OUTPUT_STATUS替代，支持16个舵机通道。 |
| 37 | MISSION_REQUEST_PARTIAL_LIST | 请求部分任务项列表，通过起始和结束索引指定，支持单一航点请求（起始=结束）。 |
| 38 | MISSION_WRITE_PARTIAL_LIST | 向MAV写入部分任务项列表，起始索引需小于等于当前列表最大索引，否则请求被拒绝。 |
| 39 | MISSION_ITEM [DEP] | 编码任务项信息（位置、命令等），已被MISSION_ITEM_INT替代，支持本地/全球坐标系。 |
| 40 | MISSION_REQUEST [DEP] | 请求指定序列号的任务项信息，已被MISSION_REQUEST_INT替代，响应为MISSION_ITEM。 |
| 41 | MISSION_SET_CURRENT [DEP] | 设置指定序列号的任务项为当前项，已被MAV_CMD_DO_SET_MISSION_CURRENT替代，可能触发任务状态机变化。 |
| 42 | MISSION_CURRENT | 广播当前目标任务项的序列号，包含总任务项数、任务状态和模式等扩展信息。 |
| 43 | MISSION_REQUEST_LIST | 请求组件的所有任务项列表，用于获取完整任务规划。 |
| 44 | MISSION_COUNT | 响应任务列表请求，提供任务项总数和任务ID，用于GCS请求单个任务项。 |
| 45 | MISSION_CLEAR_ALL | 删除所有任务项，清空任务规划。 |
| 46 | MISSION_ITEM_REACHED | 报告已到达指定序列号的任务项，系统可能悬停或继续至下一航点（取决于autocontinue设置）。 |
| 47 | MISSION_ACK | 任务处理的确认消息，包含任务结果（成功/错误类型）和任务ID，用于确认任务上传/下载状态。 |
| 48 | SET_GPS_GLOBAL_ORIGIN [DEP] | 设置本地原点（0,0,0）的GPS坐标，已被MAV_CMD_SET_GLOBAL_ORIGIN替代，支持室内外坐标转换。 |
| 49 | GPS_GLOBAL_ORIGIN | 发布本地原点的GPS坐标，在设置新原点后发送，用于本地与全球坐标系转换。 |
| 50 | PARAM_MAP_RC | 将RC通道绑定到参数，使参数值随RC通道值变化，支持参数范围限制和初始值设置。 |
| 51 | MISSION_REQUEST_INT | 请求指定序列号的任务项信息，响应为MISSION_ITEM_INT，支持更高精度的位置描述。 |
| 54 | SAFETY_SET_ALLOWED_AREA | 设置安全区域（立方体），定义MAV可接受的航点范围，用于符合法规或竞赛要求。 |
| 55 | SAFETY_ALLOWED_AREA | 读取MAV当前的安全区域设置，包含坐标系和立方体两角坐标。 |
| 61 | ATTITUDE_QUATERNION_COV | 用四元数表示姿态并包含3x3姿态协方差矩阵，提供姿态估计的不确定性信息。 |
| 62 | NAV_CONTROLLER_OUTPUT | 报告导航和位置控制器状态，包括期望滚转、俯仰、航向，目标方位角，距离航点距离和各种误差。 |
| 63 | GLOBAL_POSITION_INT_COV | 带6x6位置和速度协方差矩阵的全球位置估计，优化用于高带宽链路，提供更完整的不确定性信息。 |
| 64 | LOCAL_POSITION_NED_COV | 带9x9位置、速度和加速度协方差矩阵的本地位置估计，提供更全面的状态不确定性信息。 |
| 65 | RC_CHANNELS | 报告RC通道的PPM值（微秒），包含通道数量和信号强度，支持18个通道。 |
| 66 | REQUEST_DATA_STREAM [DEP] | 请求数据流，已被MAV_CMD_SET_MESSAGE_INTERVAL替代，指定数据流ID、速率和启停状态。 |
| 67 | DATA_STREAM [DEP] | 报告数据流状态，已被MESSAGE_INTERVAL替代，包含数据流ID、速率和启用状态。 |
| 69 | MANUAL_CONTROL | 通过标准摇杆轴手动控制飞行器，包含x（俯仰）、y（滚转）、z（推力）、r（偏航）轴和按钮状态。 |
| 70 | RC_CHANNELS_OVERRIDE | 覆盖RC通道值，发送到MAV以替代无线电接收的RC信号，支持18个通道，特定值表示忽略或释放通道。 |
| 73 | MISSION_ITEM_INT | 编码任务项信息，支持更高精度的位置（经纬度以degE7、本地坐标以m*1e4表示），替代MISSION_ITEM。 |
| 74 | VFR_HUD | 报告固定翼飞行器HUD(仪表盘)常用 metrics，包括空速、地速、航向、油门、海拔和爬升率。 |
| 75 | COMMAND_INT | 发送带整数参数的命令，适合包含位置信息的命令，参数5和6为缩放整数，支持更高精度。 |
| 76 | COMMAND_LONG | 发送带浮点参数的命令，适用于无需高精度位置的场景，COMMAND_INT更适合位置相关命令。 |
| 77 | COMMAND_ACK | 报告命令执行状态，包含命令ID、结果、进度和额外结果信息，用于确认命令处理情况。 |
| 80 | COMMAND_CANCEL [WIP] | 取消长时间运行的命令，目标系统需响应COMMAND_ACK报告取消结果，支持重试直至确认。 |
| 81 | MANUAL_SETPOINT | 操作员设置的滚转、俯仰、偏航速率和推力设定点，包含飞行模式和手动覆盖开关位置。 |
| 82 | SET_ATTITUDE_TARGET | 设置期望的飞行器姿态，用于外部控制器（如手动控制器），包含四元数、角速度和推力。 |
| 83 | ATTITUDE_TARGET | 报告当前命令的姿态目标，与SET_ATTITUDE_TARGET命令对应，反映实际执行的姿态指令。 |
| 84 | SET_POSITION_TARGET_LOCAL_NED | 设置本地NED坐标系的位置、速度和加速度目标，用于外部控制，支持多种坐标系和掩码过滤。 |
| 85 | POSITION_TARGET_LOCAL_NED | 报告当前命令的本地位置、速度和加速度目标，与SET_POSITION_TARGET_LOCAL_NED对应。 |
| 86 | SET_POSITION_TARGET_GLOBAL_INT | 设置全球坐标系的位置、速度和加速度目标，经纬度以degE7表示，支持多种坐标系。 |
| 87 | POSITION_TARGET_GLOBAL_INT | 报告当前命令的全球位置、速度和加速度目标，与SET_POSITION_TARGET_GLOBAL_INT对应。 |
| 89 | LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET | 报告本地位置（LOCAL_POSITION_NED）与全球坐标系的X、Y、Z偏移和姿态偏移（滚转、俯仰、偏航）。 |
| 90 | HIL_STATE [DEP] | 从仿真发送到自动驾驶仪的状态消息，已被HIL_STATE_QUATERNION替代，存在空速缺失和欧拉角奇点问题。 |
| 91 | HIL_CONTROLS | 自动驾驶仪发送到仿真的硬件在环控制输出，包含副翼、升降舵、方向舵、油门和辅助通道控制。 |
| 92 | HIL_RC_INPUTS_RAW | 仿真发送到自动驾驶仪的原始RC输入，包含12个通道的原始值和信号强度。 |
| 93 | HIL_ACTUATOR_CONTROLS | 自动驾驶仪发送到仿真的执行器控制输出，包含16个通道的控制值（-1至1）和系统模式标志。 |
| 100 | OPTICAL_FLOW | 光流传感器数据，包括X/Y方向光流、补偿后光流、质量和地面距离。 |
| 101 | GLOBAL_VISION_POSITION_ESTIMATE | 视觉源的全球位置/姿态估计，包含位置、姿态和6x6位姿协方差矩阵。 |
| 102 | VISION_POSITION_ESTIMATE | 视觉源的本地位置/姿态估计，包含位置、姿态和6x6位姿协方差矩阵。 |
| 103 | VISION_SPEED_ESTIMATE | 视觉源的速度估计，包含全球X/Y/Z速度和3x3速度协方差矩阵。 |
| 104 | VICON_POSITION_ESTIMATE | Vicon运动系统的全球位置估计，包含位置、姿态和6x6位姿协方差矩阵。 |
| 105 | HIGHRES_IMU | NED机体坐标系下的高精度IMU读数（SI单位），包括加速度、角速度、磁场、压力和温度。 |
| 106 | OPTICAL_FLOW_RAD | 角速率光流传感器数据，包括积分光流、角速度、温度、质量和地面距离。 |
| 107 | HIL_SENSOR | 仿真的IMU读数（SI单位，NED机体坐标系），包含加速度、角速度、磁场、压力和温度。 |
| 108 | SIM_STATE | 仿真环境状态，包含姿态（四元数和欧拉角）、加速度、角速度、位置、速度和不确定性。 |
| 109 | RADIO_STATUS | 无线电状态报告，包含本地/远程信号强度、剩余发送缓冲区、噪声水平、接收错误和纠错计数。 |
| 110 | FILE_TRANSFER_PROTOCOL | 文件传输协议消息，用于MAVLink FTP服务，包含目标地址和变长有效载荷。 |
| 111 | TIMESYNC | 时间同步消息，支持请求和响应，通过时间戳计算偏移，用于网络时间同步。 |
| 112 | CAMERA_TRIGGER | 相机-IMU触发和同步消息，包含图像帧时间戳和序列号。 |
| 113 | HIL_GPS | 仿真的GPS原始位置数据，包括经纬度、海拔、精度因子、速度、可见卫星数和航向。 |
| 114 | HIL_OPTICAL_FLOW | 仿真的光流数据，包括积分光流、角速度、温度、质量和地面距离。 |
| 115 | HIL_STATE_QUATERNION | 从仿真发送到自动驾驶仪的四元数状态消息，避免欧拉角奇点，包含空速信息。 |
| 116 | SCALED_IMU2 | 第二个9DOF传感器的缩放IMU读数，格式同SCALED_IMU。 |
| 117 | LOG_REQUEST_LIST | 请求可用日志列表，部分系统可能在请求期间停止日志记录，直至LOG_REQUEST_END。 |
| 118 | LOG_ENTRY | 响应日志列表请求，提供日志ID、总数、最后日志号、时间戳和大小。 |
| 119 | LOG_REQUEST_DATA | 请求日志的一部分数据，指定日志ID、偏移和字节数。 |
| 120 | LOG_DATA | 响应日志数据请求，发送日志片段，包含日志ID、偏移、数据长度和数据。 |
| 121 | LOG_ERASE | 删除所有日志。 |
| 122 | LOG_REQUEST_END | 停止日志传输并恢复正常日志记录。 |
| 123 | GPS_INJECT_DATA [DEP] | 注入数据到板载GPS（用于DGPS），已被GPS_RTCM_DATA替代。 |
| 124 | GPS2_RAW | 第二个GPS的原始数据，格式同GPS_RAW_INT，包含额外的DGPS信息和精度参数。 |
| 125 | POWER_STATUS | 电源状态报告，包括5V rail电压、伺服rail电压和电源状态标志。 |
| 126 | SERIAL_CONTROL | 控制串行端口，用于访问板载串行外设（如GPS、无线电），支持固件更新和设置修改。 |
| 127 | GPS_RTK | RTK GPS数据，提供基线计算信息，包括基线坐标、精度、卫星数量和健康状态。 |
| 128 | GPS2_RTK | 第二个RTK GPS的数据，格式同GPS_RTK。 |
| 129 | SCALED_IMU3 | 第三个9DOF传感器的缩放IMU读数，格式同SCALED_IMU。 |
| 130 | DATA_TRANSMISSION_HANDSHAKE | 图像传输协议的握手消息，用于初始化、控制和停止图像流传输。 |
| 131 | ENCAPSULATED_DATA | 图像传输协议的数据分组，包含序列号和图像数据字节。 |
| 132 | DISTANCE_SENSOR | 板载测距仪的距离信息，包括最小/最大/当前距离、传感器类型、朝向和测量方差。 |
| 133 | TERRAIN_REQUEST | 请求地形数据和状态，指定西南角坐标、网格间距和请求网格的位掩码。 |
| 134 | TERRAIN_DATA | 从GCS发送的地形数据，对应TERRAIN_REQUEST的请求，包含网格西南角坐标、间距和地形高度数据。 |
| 135 | TERRAIN_CHECK | 请求飞行器报告特定位置的地形高度，用于GCS检查飞行器是否拥有任务所需地形数据。 |
| 136 | TERRAIN_REPORT | 无人机发送的地形地图下载进度或地形检查响应，包含地形高度、当前高度和待加载/已加载网格数。 |
| 137 | SCALED_PRESSURE2 | 第二个气压计的读数，格式同SCALED_PRESSURE。 |
| 138 | ATT_POS_MOCAP | 运动捕捉的姿态和位置，包含四元数姿态、NED坐标和6x6位姿协方差矩阵。 |
| 139 | SET_ACTUATOR_CONTROL_TARGET | 设置飞行器姿态和机体角速率的执行器控制目标，包含8个执行器通道的控制值。 |
| 140 | ACTUATOR_CONTROL_TARGET | 报告当前命令的执行器控制目标，包含8个执行器通道的控制值。 |
| 141 | ALTITUDE | 系统当前的多种海拔测量值，包括单调海拔、MSL海拔、本地海拔、相对home海拔、地形海拔和下方净空高度。 |
| 142 | RESOURCE_REQUEST | 自动驾驶仪请求资源（文件、二进制等），指定资源URI、传输类型和存储路径。 |
| 143 | SCALED_PRESSURE3 | 第三个气压计的读数，格式同SCALED_PRESSURE。 |
| 144 | FOLLOW_TARGET | 指定系统的当前运动信息，包含位置、速度、加速度、姿态、速率和位置协方差。 |
| 146 | CONTROL_SYSTEM_STATE | 用于控制系统的平滑、单调系统状态，包括加速度、速度、位置、空速、姿态和角速度。 |
| 147 | BATTERY_STATUS | 电池信息，更新GCS的电池状态，包括温度、电池电压、电流、剩余电量和故障标志。 |
| 148 | AUTOPILOT_VERSION | 自动驾驶仪软件的版本和能力，包含能力标志、固件/中间件/OS版本、硬件版本和厂商/产品ID。 |
| 149 | LANDING_TARGET | 着陆目标的位置信息，包括角度偏移、距离、大小和三维位置，用于精确着陆。 |
| 162 | FENCE_STATUS | 地理围栏状态，在围栏启用时发送，包含 breach状态、次数、类型、时间和缓解措施。 |
| 192 | MAG_CAL_REPORT | 报告指南针校准结果，包含校准状态、自动保存标志、残差、偏移、矩阵参数和姿态信息。 |
| 225 | EFI_STATUS | EFI（电子燃油喷射）状态输出，包含健康状态、转速、燃油消耗、发动机负载、温度和点火正时等。 |
| 230 | ESTIMATOR_STATUS | 估计器状态消息，包括标志（有效输出）、创新测试比（传感器一致性）和估计精度。 |
| 231 | WIND_COV | 飞行器的风速估计，包含北/东/下方向风速、水平/垂直风速变化率和风速精度。 |
| 232 | GPS_INPUT | GPS传感器输入消息（原始传感器值），包含时间、周数、fix类型、位置、速度和精度。 |
| 233 | GPS_RTCM_DATA | 注入到板载GPS的RTCM消息，支持分片传输，包含标志、数据长度和RTCM数据。 |
| 234 | HIGH_LATENCY [DEP] | 高延迟连接（如铱星）的消息，已被HIGH_LATENCY2替代，包含系统模式、姿态、位置和电池等信息。 |
| 235 | HIGH_LATENCY2 | 高延迟连接的消息（版本2），包含时间戳、飞行器类型、位置、目标高度、速度和电池等信息。 |
| 241 | VIBRATION | 振动水平和加速度计削波计数，包含X/Y/Z轴振动值和三个加速度计的削波计数。 |
| 242 | HOME_POSITION | 包含home位置（系统返回和着陆的默认位置），包含GPS坐标、本地坐标、姿态和进近向量。 |
| 243 | SET_HOME_POSITION [DEP] | 设置home位置，已被MAV_CMD_DO_SET_HOME替代，格式同HOME_POSITION。 |
| 244 | MESSAGE_INTERVAL | 特定MAVLink消息ID的发送间隔，响应消息请求，包含消息ID和间隔（微秒）。 |
| 245 | EXTENDED_SYS_STATE | 提供额外功能的状态，包括VTOL状态（如悬停/固定翼模式）和着陆状态（如地面/空中）。 |
| 246 | ADSB_VEHICLE | ADS-B飞行器的位置和信息，包含ICAO地址、位置、高度、航向、速度、呼号和发射类型等。 |
| 247 | COLLISION | 潜在碰撞的信息，包含碰撞源、ID、采取的规避动作、威胁等级和最小距离/时间。 |
| 248 | V2_EXTENSION | 在V1帧中实现V2有效载荷规范的消息，用于过渡支持，包含目标地址、消息类型和变长载荷。 |
| 249 | MEMORY_VECT | 发送原始控制器内存数据，用于测试和调试，包含起始地址、版本、类型和内存内容。 |
| 250 | DEBUG_VECT | 用命名的3D向量调试，包含名称、时间戳和X/Y/Z向量值。 |
| 251 | NAMED_VALUE_FLOAT | 发送浮点键值对，用于测试和调试，包含启动时间、名称和浮点值。 |
| 252 | NAMED_VALUE_INT | 发送整数键值对，用于测试和调试，包含启动时间、名称和整数值。 |
| 253 | STATUSTEXT | 状态文本消息，在QGroundControl的COMM控制台显示（黄色），包含严重程度和文本内容。 |
| 254 | DEBUG | 发送调试值，索引用于区分不同值，在QGroundControl中以“DEBUG N”形式显示。 |
| 256 | SETUP_SIGNING | 设置MAVLink2签名密钥，全零密钥和初始时间戳禁用签名功能。 |
| 257 | BUTTON_CHANGE | 报告按钮状态变化，包含启动时间、最后变化时间和按钮状态位掩码。 |
| 258 | PLAY_TUNE [DEP] | 控制飞行器音调生成（蜂鸣器），已被PLAY_TUNE_V2替代，包含目标系统、组件和音调字符串。 |
| 259 | CAMERA_INFORMATION | 相机信息，可通过MAV_CMD_REQUEST_MESSAGE请求，包含厂商、型号、固件版本、焦距和定义URI。 |
| 260 | CAMERA_SETTINGS | 相机设置，可通过MAV_CMD_REQUEST_MESSAGE请求，包含模式、缩放级别和聚焦级别。 |
| 261 | STORAGE_INFORMATION | 存储介质信息，响应请求或状态变化时发送，包含容量、使用量、读写速度和类型。 |
| 262 | CAMERA_CAPTURE_STATUS | 相机捕捉状态信息，包含图像/视频捕捉状态、间隔、录制时间和可用存储。 |
| 263 | CAMERA_IMAGE_CAPTURED | 已捕捉图像的信息，每次捕捉时发送，包含时间戳、位置、姿态、图像索引和文件URL。 |
| 264 | FLIGHT_INFORMATION | 飞行信息，包括武装、起飞、着陆时间和飞行编号，部分字段名称存在误导（实为启动时间）。 |
| 265 | MOUNT_ORIENTATION [DEP] | 挂载的姿态，已被MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW替代，包含滚转、俯仰、偏航角。 |
| 266 | LOGGING_DATA | 包含日志数据的消息，用于日志传输，包含目标系统、组件、序列号、数据长度和日志数据。 |
| 267 | LOGGING_DATA_ACKED | 包含需要确认的日志数据，接收方需返回LOGGING_ACK，格式同LOGGING_DATA。 |
| 268 | LOGGING_ACK | 对LOGGING_DATA_ACKED消息的确认，包含目标系统、组件和对应的序列号。 |
| 269 | VIDEO_STREAM_INFORMATION | 视频流信息，可通过请求获取，包含流ID、类型、帧率、分辨率、比特率和URI。 |
| 270 | VIDEO_STREAM_STATUS | 视频流状态信息，包含流ID、状态标志、帧率、分辨率、比特率和旋转角度。 |
| 271 | CAMERA_FOV_STATUS | 相机视场信息，包含相机和图像中心位置、姿态、水平/垂直视场角。 |
| 275 | CAMERA_TRACKING_IMAGE_STATUS | 相机跟踪状态（主动跟踪时发送），包含跟踪状态、模式、目标位置（点或矩形）。 |
| 276 | CAMERA_TRACKING_GEO_STATUS | 相机跟踪状态（含地理信息），包含跟踪状态、目标位置、速度、距离和航向。 |
| 277 | CAMERA_THERMAL_RANGE | 相机绝对热范围，包含最高/最低温度及对应位置，随热成像视频流发送。 |
| 280 | GIMBAL_MANAGER_INFORMATION | 高级云台管理器信息，包含能力标志、负责的云台设备ID和云台角度范围。 |
| 281 | GIMBAL_MANAGER_STATUS | 高级云台管理器当前状态，包含标志、负责的云台设备ID和主/次要控制组件。 |
| 282 | GIMBAL_MANAGER_SET_ATTITUDE | 控制云台姿态的高级消息，包含标志、云台设备ID、四元数姿态和角速度。 |
| 283 | GIMBAL_DEVICE_INFORMATION | 低级云台信息，包含厂商、型号、固件/硬件版本、UID、能力标志和角度范围。 |
| 284 | GIMBAL_DEVICE_SET_ATTITUDE | 控制云台设备姿态的低级消息，包含标志、四元数姿态和角速度。 |
| 285 | GIMBAL_DEVICE_ATTITUDE_STATUS | 云台设备的姿态状态报告，包含标志、四元数姿态、角速度、故障标志和姿态偏移。 |
| 286 | AUTOPILOT_STATE_FOR_GIMBAL_DEVICE | 自动驾驶仪状态（用于云台设备估计校正），包含姿态、速度、前馈角速度和估计器状态。 |
| 287 | GIMBAL_MANAGER_SET_PITCHYAW | 设置云台管理器的俯仰和偏航角（高速消息），包含标志、云台设备ID、角度和速率。 |
| 288 | GIMBAL_MANAGER_SET_MANUAL_CONTROL | 手动控制云台的高级消息，包含标志、云台设备ID、无量纲角度和速率。 |
| 290 | ESC_INFO [WIP] | ESC信息（低速率传输），包含索引、时间戳、总数、连接类型、故障标志和温度。 |
| 291 | ESC_STATUS [WIP] | ESC信息（高速率传输），包含索引、时间戳和4个ESC的转速、电压、电流。 |
| 299 | WIFI_CONFIG_AP | 配置WiFi AP的SSID、密码和模式，消息会被AP重新发送作为确认。 |
| 300 | PROTOCOL_VERSION [WIP] | 协议版本和能力，用于握手确定网络使用的MAVLink版本，包含版本号和哈希值。 |
| 301 | AIS_VESSEL | AIS船舶的位置和信息，包含MMSI、位置、航向、速度、船舶类型、尺寸和呼号。 |
| 310 | UAVCAN_NODE_STATUS | UAVCAN节点的一般状态信息，包含启动时间、健康状态、模式和厂商特定状态码。 |
| 311 | UAVCAN_NODE_INFO | 特定UAVCAN节点的一般信息，包含启动时间、节点名称、硬件/软件版本和唯一ID。 |
| 320 | PARAM_EXT_REQUEST_READ | 请求读取扩展参数值，通过参数ID或索引指定，响应为PARAM_EXT_VALUE。 |
| 321 | PARAM_EXT_REQUEST_LIST | 请求所有扩展参数列表，触发组件发送所有扩展参数的PARAM_EXT_VALUE消息。 |
| 322 | PARAM_EXT_VALUE | 发送扩展参数值，包含参数ID、值（字符串）、类型、总参数数和索引。 |
| 323 | PARAM_EXT_SET | 设置扩展参数值，接收方需返回PARAM_EXT_ACK确认，支持值相同的立即确认。 |
| 324 | PARAM_EXT_ACK | 对PARAM_EXT_SET消息的响应，包含参数ID、值、类型和结果码。 |
| 330 | OBSTACLE_DISTANCE | 传感器前方的障碍物距离，从左到右按角度递增排列，包含距离、角度增量和传感器信息。 |
| 331 | ODOMETRY | 里程计消息，用于与外部接口通信里程计信息，符合ROS REP 147标准，包含位置、速度和协方差。 |
| 332 | TRAJECTORY_REPRESENTATION_WAYPOINTS [DEP] | 用航点描述轨迹（最多5个），已废弃，适用于PX4 v1.11至v1.14。 |
| 333 | TRAJECTORY_REPRESENTATION_BEZIER [DEP] | 用贝塞尔控制点描述轨迹（最多5个），已废弃，适用于PX4 v1.11至v1.14。 |
| 334 | CELLULAR_STATUS | 报告当前使用的蜂窝网络状态，包含状态、故障原因、无线电类型、信号质量和网络代码。 |
| 335 | ISBD_LINK_STATUS | Iridium SBD链路状态，包含时间戳、最后心跳时间、失败/成功会话数、信号质量和 pending状态。 |
| 336 | CELLULAR_CONFIG | 配置蜂窝调制解调器，包含LTE使能、PIN设置、APN和漫游设置，消息会被调制解调器确认。 |
| 339 | RAW_RPM | RPM传感器数据消息，包含传感器索引和指示速率（rpm）。 |
| 340 | UTM_GLOBAL_POSITION | GPS和传感器融合的全球位置，包含时间、UAS ID、位置、速度、不确定性和下一航点。 |
| 350 | DEBUG_FLOAT_ARRAY | 大型调试/原型数组，包含时间戳、名称、数组ID和数据，不建议用于生产代码。 |
| 360 | ORBIT_EXECUTION_STATUS | 轨道执行期间的飞行器状态报告，包含轨道半径、中心坐标和坐标系。 |
| 370 | SMART_BATTERY_INFO [DEP] | 智能电池信息（静态/低频更新），已被BATTERY_INFO替代，包含容量、循环计数和参数。 |
| 371 | FUEL_STATUS | 燃料状态信息，包含最大/消耗/剩余燃料、百分比、流量和温度，支持多种燃料类型。 |
| 372 | BATTERY_INFO [WIP] | 电池静态信息或低频更新信息，包含健康状态、串联电池数、循环计数和电压参数。 |
| 373 | GENERATOR_STATUS | 发电系统遥测信息，包含状态标志、转速、电池电流、负载电流、功率和温度。 |
| 375 | ACTUATOR_OUTPUT_STATUS | 电机输出的原始值（例如，在Pixhawk上，从MAIN和AUX端口）。 |
| 380 | TIME_ESTIMATE_TO_TARGET [WIP] | 各种事件和动作的时间估计，包含安全返回、着陆、任务项完成和命令动作的估计时间。 |
| 385 | TUNNEL | 在组件间传输变长数据的消息，包含目标地址、载荷类型、长度和数据，支持自定义编码。 |
| 386 | CAN_FRAME | 转发的CAN帧，应MAV_CMD_CAN_FORWARD请求发送，包含总线号、长度、ID和数据。 |
| 387 | CANFD_FRAME | 转发的CANFD帧，应MAV_CMD_CAN_FORWARD请求发送，包含总线号、长度、ID和数据。 |
| 388 | CAN_FILTER_MODIFY | 修改CAN消息转发的过滤器，指定总线、操作、ID数量和过滤ID，优化低带宽链路传输。 |
| 390 | ONBOARD_COMPUTER_STATUS [WIP] | 机载计算机的硬件状态，包含CPU/GPU使用率、温度、风扇速度、内存/存储使用和网络流量。 |
| 395 | COMPONENT_INFORMATION [DEP] | 组件信息消息，已被COMPONENT_METADATA替代，包含元数据文件CRC和URI。 |
| 396 | COMPONENT_INFORMATION_BASIC | 基本组件信息数据，包含能力标志、制造时间、厂商/型号名称和软/硬件版本。 |
| 397 | COMPONENT_METADATA [WIP] | 组件元数据消息，包含通用元数据文件的CRC和URI，用于获取组件元数据。 |
| 400 | PLAY_TUNE_V2 | 播放飞行器音调（蜂鸣器），替代PLAY_TUNE，包含目标系统、组件、格式和音调定义。 |
| 401 | SUPPORTED_TUNES | 飞行器支持的音调格式，响应MAV_CMD_REQUEST_MESSAGE请求，包含支持的格式位掩码。 |
| 410 | EVENT [WIP] | 事件消息，每个新事件有唯一序列号，包含目标组件、事件ID、时间戳、日志级别和参数。 |
| 411 | CURRENT_EVENT_SEQUENCE [WIP] | 定期广播组件的最新事件序列号，用于检查丢失事件，序列号变化表示事件集更新。 |
| 412 | REQUEST_EVENT [WIP] | 请求重发一个或多个事件，指定起始和结束序列号，触发EVENT或RESPONSE_EVENT_ERROR响应。 |
| 413 | RESPONSE_EVENT_ERROR [WIP] | 对REQUEST_EVENT的错误响应，包含目标组件、序列号、最早可用序列号和错误原因。 |
| 435 | AVAILABLE_MODES | 飞行模式信息，可枚举或请求特定模式，包含总模式数、索引、标准/自定义模式和名称。 |
| 436 | CURRENT_MODE | 报告当前飞行模式，包含标准模式、自定义模式和用户期望的自定义模式，模式变化时发送。 |
| 437 | AVAILABLE_MODES_MONITOR | 指示可用模式集变化的序列号，序列号变化时需重新请求所有可用模式。 |
| 440 | ILLUMINATOR_STATUS | 照明器状态，包含运行时间、启用状态、模式、亮度、频闪周期、温度和范围。 |
| 9000 | WHEEL_DISTANCE | 每个报告车轮的累计行驶距离，包含时间戳、车轮数量和每个车轮的距离（正向增加，反向减少）。 |
| 9005 | WINCH_STATUS | 绞盘状态，包含时间戳、释放长度、速度、张力、电压、电流、温度和状态标志。 |
| 12900 | OPEN_DRONE_ID_BASIC_ID | OpenDroneID基本ID消息数据，包含UAS ID、类型和UA类型，符合ASTM F3411和ASD-STAN标准。 |
| 12901 | OPEN_DRONE_ID_LOCATION | OpenDroneID位置消息数据，包含位置、高度、方向、速度和时间戳，用于远程ID。 |
| 12902 | OPEN_DRONE_ID_AUTHENTICATION | OpenDroneID认证消息数据，包含认证类型、数据页和认证数据，用于身份验证。 |
| 12903 | OPEN_DRONE_ID_SELF_ID | OpenDroneID自ID消息数据，包含描述类型和文本描述，用于操作员身份声明。 |
| 12904 | OPEN_DRONE_ID_SYSTEM | OpenDroneID系统消息数据，包含操作员位置、飞行器分类和区域信息，用于系统状态报告。 |
| 12905 | OPEN_DRONE_ID_OPERATOR_ID | OpenDroneID操作员ID消息数据，包含操作员ID类型和ID，用于CAA颁发的操作员标识。 |
| 12915 | OPEN_DRONE_ID_MESSAGE_PACK | OpenDroneID消息包，包含多个编码的OpenDroneID消息，用于低带宽无线传输（如蓝牙、WiFi）。 |
| 12918 | OPEN_DRONE_ID_ARM_STATUS | 远程ID系统的启用和就绪状态，用于飞行控制器作为arming条件，包含状态和错误消息。 |
| 12919 | OPEN_DRONE_ID_SYSTEM_UPDATE | 更新OPEN_DRONE_ID_SYSTEM消息中的位置信息，优化低带宽链路的操作员位置更新频率。 |
| 12920 | HYGROMETER_SENSOR | 湿度计的温度和湿度数据，包含传感器ID、温度（cdegC）和湿度（c%）。 |


## 二、MAVLink消息分类整理
### 一、系统状态与控制类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 0 | HEARTBEAT | 显示系统或组件存在并响应，帮助接收系统正确处理后续消息。 |
| 1 | SYS_STATUS | 提供系统一般状态，包括模式、导航状态、传感器状态、电池信息等。 |
| 2 | SYSTEM_TIME | 提供主时钟时间，用于日志时间戳同步。 |
| 5 | CHANGE_OPERATOR_CONTROL | 请求控制MAV，包含目标系统、控制请求类型等信息。 |
| 6 | CHANGE_OPERATOR_CONTROL_ACK | 接受或拒绝控制请求，包含确认结果。 |
| 11 | SET_MODE [DEP] | 设置系统模式，已被MAV_CMD_DO_SET_MODE替代。 |
| 245 | EXTENDED_SYS_STATE | 提供额外功能状态，如VTOL状态、着陆状态等。 |
| 435 | AVAILABLE_MODES | 飞行模式信息，可枚举或请求特定模式。 |
| 436 | CURRENT_MODE | 报告当前飞行模式，模式变化时发送。 |
| 437 | AVAILABLE_MODES_MONITOR | 指示可用模式集变化的序列号，序列号变化时需重新请求所有可用模式。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **0** | `HEARTBEAT` | **核心生存信号**，确认设备在线和组件类型（飞控、GCS等）。 |
| **1** | `SYS_STATUS` | **系统健康报告**，含电池电压、传感器状态、错误标志，实时监控必备。 |
| **245** | `EXTENDED_SYS_STATE` | **关键状态扩展**，报告VTOL模式切换、着陆状态等高级行为。 |
| **436** | `CURRENT_MODE` | **当前飞行模式**，模式变化时主动推送（如自稳、定高、返航）。 |

### 二、参数相关类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 20 | PARAM_REQUEST_READ | 请求读取板载参数，通过参数ID或索引指定。 |
| 21 | PARAM_REQUEST_LIST | 请求组件的所有参数列表，触发发送PARAM_VALUE消息。 |
| 22 | PARAM_VALUE | 发送板载参数值，包含参数ID、值、类型等。 |
| 23 | PARAM_SET | 设置参数值并写入永久存储，接收方需广播PARAM_VALUE确认。 |
| 50 | PARAM_MAP_RC | 将RC通道绑定到参数，使参数值随RC通道值变化。 |
| 320 | PARAM_EXT_REQUEST_READ | 请求读取扩展参数值，响应为PARAM_EXT_VALUE。 |
| 321 | PARAM_EXT_REQUEST_LIST | 请求所有扩展参数列表，触发发送PARAM_EXT_VALUE消息。 |
| 322 | PARAM_EXT_VALUE | 发送扩展参数值，包含参数ID、值（字符串）、类型等。 |
| 323 | PARAM_EXT_SET | 设置扩展参数值，接收方需返回PARAM_EXT_ACK确认。 |
| 324 | PARAM_EXT_ACK | 对PARAM_EXT_SET消息的响应，包含结果码。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **21** | `PARAM_REQUEST_LIST` | **请求所有参数列表**，初始化时加载参数集。 |
| **22** | `PARAM_VALUE` | **参数值传输**，读取或设置后返回确认。 |
| **23** | `PARAM_SET` | **设置参数值**，用于动态配置飞控（如PID调节）。 |

### 三、位置与姿态类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 30 | ATTITUDE | 报告飞行器姿态，包括滚转、俯仰、偏航角及角速度。 |
| 31 | ATTITUDE_QUATERNION | 用四元数表示飞行器姿态及角速度，支持姿态偏移修正。 |
| 32 | LOCAL_POSITION_NED | 报告滤波后的本地位置（NED坐标系）及速度。 |
| 33 | GLOBAL_POSITION_INT | 报告滤波后的全球位置（GPS坐标系）、地速和航向。 |
| 61 | ATTITUDE_QUATERNION_COV | 用四元数表示姿态并包含3x3姿态协方差矩阵。 |
| 63 | GLOBAL_POSITION_INT_COV | 带6x6位置和速度协方差矩阵的全球位置估计。 |
| 64 | LOCAL_POSITION_NED_COV | 带9x9位置、速度和加速度协方差矩阵的本地位置估计。 |
| 89 | LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET | 报告本地位置与全球坐标系的偏移和姿态偏移。 |
| 141 | ALTITUDE | 系统当前的多种海拔测量值，如单调海拔、MSL海拔等。 |
| 242 | HOME_POSITION | 包含home位置（系统返回和着陆的默认位置）。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **30** | `ATTITUDE` | **姿态角（滚转/俯仰/偏航）**，基础控制反馈。 |
| **31** | `ATTITUDE_QUATERNION` | **四元数姿态**，避免万向节锁，适合融合算法。 |
| **33** | `GLOBAL_POSITION_INT` | **全球位置（经纬高）**，GPS定位数据核心。 |
| **32** | `LOCAL_POSITION_NED` | **本地位置（NED坐标系）**，室内/视觉定位依赖。 |
| **242** | `HOME_POSITION` | **返航点位置**，安全机制（如失控返航）基础。 |

### 四、传感器数据类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 24 | GPS_RAW_INT | 提供GPS原始位置数据，包括经纬度、海拔、速度等。 |
| 25 | GPS_STATUS | 报告GPS定位状态，包含可见卫星数量及相关信息。 |
| 26 | SCALED_IMU | 提供9DOF传感器的缩放IMU读数（加速度、角速度、磁场强度）。 |
| 27 | RAW_IMU | 提供9DOF传感器的原始IMU读数，用于数据捕获和调试。 |
| 28 | RAW_PRESSURE | 提供原始压力传感器读数（未缩放）。 |
| 29 | SCALED_PRESSURE | 提供压力传感器的缩放读数，包括绝对压力、温度等。 |
| 100 | OPTICAL_FLOW | 光流传感器数据，包括X/Y方向光流、质量和地面距离。 |
| 105 | HIGHRES_IMU | NED机体坐标系下的高精度IMU读数（SI单位）。 |
| 106 | OPTICAL_FLOW_RAD | 角速率光流传感器数据，包括积分光流、角速度等。 |
| 124 | GPS2_RAW | 第二个GPS的原始数据，格式同GPS_RAW_INT。 |
| 127 | GPS_RTK | RTK GPS数据，提供基线计算信息。 |
| 128 | GPS2_RTK | 第二个RTK GPS的数据，格式同GPS_RTK。 |
| 116 | SCALED_IMU2 | 第二个9DOF传感器的缩放IMU读数，格式同SCALED_IMU。 |
| 129 | SCALED_IMU3 | 第三个9DOF传感器的缩放IMU读数，格式同SCALED_IMU。 |
| 132 | DISTANCE_SENSOR | 板载测距仪的距离信息，包括最小/最大/当前距离等。 |
| 137 | SCALED_PRESSURE2 | 第二个气压计的读数，格式同SCALED_PRESSURE。 |
| 143 | SCALED_PRESSURE3 | 第三个气压计的读数，格式同SCALED_PRESSURE。 |
| 232 | GPS_INPUT | GPS传感器输入消息（原始传感器值）。 |
| 123 | GPS_INJECT_DATA [DEP] | 注入数据到板载GPS（用于DGPS），已被GPS_RTCM_DATA替代。 |
| 233 | GPS_RTCM_DATA | 注入到板载GPS的RTCM消息，支持分片传输。 |
| 339 | RAW_RPM | RPM传感器数据消息，包含传感器索引和指示速率。 |
| 12920 | HYGROMETER_SENSOR | 湿度计的温度和湿度数据。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **24** | `GPS_RAW_INT` | **原始GPS数据**，包含卫星数、定位精度、HDOP。 |
| **105** | `HIGHRES_IMU` | **高精度IMU数据**（加速度/角速度），状态估计输入。 |
| **132** | `DISTANCE_SENSOR` | **测距传感器数据**，避障、定高（如超声波、激光）。 |
| **147** | `BATTERY_STATUS` | **电池详细信息**，电压、电流、剩余电量，续航管理核心。 |

### 五、任务与航点类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 37 | MISSION_REQUEST_PARTIAL_LIST | 请求部分任务项列表，通过起始和结束索引指定。 |
| 38 | MISSION_WRITE_PARTIAL_LIST | 向MAV写入部分任务项列表，起始索引需符合要求。 |
| 39 | MISSION_ITEM [DEP] | 编码任务项信息，已被MISSION_ITEM_INT替代。 |
| 40 | MISSION_REQUEST [DEP] | 请求指定序列号的任务项信息，已被MISSION_REQUEST_INT替代。 |
| 41 | MISSION_SET_CURRENT [DEP] | 设置指定序列号的任务项为当前项，已被替代。 |
| 42 | MISSION_CURRENT | 广播当前目标任务项的序列号及相关信息。 |
| 43 | MISSION_REQUEST_LIST | 请求组件的所有任务项列表，用于获取完整任务规划。 |
| 44 | MISSION_COUNT | 响应任务列表请求，提供任务项总数和任务ID。 |
| 45 | MISSION_CLEAR_ALL | 删除所有任务项，清空任务规划。 |
| 46 | MISSION_ITEM_REACHED | 报告已到达指定序列号的任务项。 |
| 47 | MISSION_ACK | 任务处理的确认消息，包含任务结果。 |
| 51 | MISSION_REQUEST_INT | 请求指定序列号的任务项信息，响应为MISSION_ITEM_INT。 |
| 73 | MISSION_ITEM_INT | 编码任务项信息，支持更高精度的位置描述，替代MISSION_ITEM。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **73** | `MISSION_ITEM_INT` | **高精度航点任务**（替代旧版），支持厘米级定位。 |
| **43** | `MISSION_REQUEST_LIST` | **请求任务列表**，规划航线前必备。 |
| **44** | `MISSION_COUNT` | **任务总数确认**，同步航点数量。 |
| **42** | `MISSION_CURRENT` | **当前执行航点**，实时进度反馈。 |

### 六、命令与控制类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 75 | COMMAND_INT | 发送带整数参数的命令，适合包含位置信息的命令。 |
| 76 | COMMAND_LONG | 发送带浮点参数的命令，适用于无需高精度位置的场景。 |
| 77 | COMMAND_ACK | 报告命令执行状态，包含命令ID、结果等。 |
| 80 | COMMAND_CANCEL [WIP] | 取消长时间运行的命令，目标系统需响应COMMAND_ACK。 |
| 69 | MANUAL_CONTROL | 通过标准摇杆轴手动控制飞行器，包含各轴和按钮状态。 |
| 81 | MANUAL_SETPOINT | 操作员设置的滚转、俯仰等设定点，包含飞行模式等。 |
| 82 | SET_ATTITUDE_TARGET | 设置期望的飞行器姿态，用于外部控制器。 |
| 83 | ATTITUDE_TARGET | 报告当前命令的姿态目标，与SET_ATTITUDE_TARGET对应。 |
| 84 | SET_POSITION_TARGET_LOCAL_NED | 设置本地NED坐标系的位置、速度和加速度目标。 |
| 85 | POSITION_TARGET_LOCAL_NED | 报告当前命令的本地位置等目标，与SET_POSITION_TARGET_LOCAL_NED对应。 |
| 86 | SET_POSITION_TARGET_GLOBAL_INT | 设置全球坐标系的位置、速度和加速度目标。 |
| 87 | POSITION_TARGET_GLOBAL_INT | 报告当前命令的全球位置等目标，与SET_POSITION_TARGET_GLOBAL_INT对应。 |
| 139 | SET_ACTUATOR_CONTROL_TARGET | 设置飞行器姿态和机体角速率的执行器控制目标。 |
| 140 | ACTUATOR_CONTROL_TARGET | 报告当前命令的执行器控制目标。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **76** | `COMMAND_LONG` | **通用命令发送**（无位置参数），如ARM/DISARM。 |
| **75** | `COMMAND_INT` | **含位置参数命令**（如飞到指定点）。 |
| **77** | `COMMAND_ACK` | **命令执行结果**（成功/失败原因），指令可靠性保障。 |
| **84** | `SET_POSITION_TARGET_LOCAL_NED` | **设定本地位置目标**，Offboard模式控制核心。 |

### 七、通信与链路类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 4 | PING [DEP] | 用于测量系统延迟，已被TIMESYNC替代。 |
| 8 | LINK_NODE_STATUS [WIP] | 通信链中每个节点的状态，包括缓冲区、传输速率等。 |
| 109 | RADIO_STATUS | 无线电状态报告，包含信号强度、噪声水平等。 |
| 111 | TIMESYNC | 时间同步消息，支持请求和响应，用于网络时间同步。 |
| 244 | MESSAGE_INTERVAL | 特定MAVLink消息ID的发送间隔，响应消息请求。 |
| 300 | PROTOCOL_VERSION [WIP] | 协议版本和能力，用于握手确定网络使用的MAVLink版本。 |
| 334 | CELLULAR_STATUS | 报告当前使用的蜂窝网络状态。 |
| 335 | ISBD_LINK_STATUS | Iridium SBD链路状态，包含信号质量等信息。 |
| 336 | CELLULAR_CONFIG | 配置蜂窝调制解调器，包含LTE使能、APN等设置。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **109** | `RADIO_STATUS` | **无线电链路质量**（RSSI/噪声），诊断通信可靠性。 |
| **111** | `TIMESYNC` | **系统时间同步**，日志和传感器数据对齐关键。 |

### 八、日志与数据传输类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 110 | FILE_TRANSFER_PROTOCOL | 文件传输协议消息，用于MAVLink FTP服务。 |
| 117 | LOG_REQUEST_LIST | 请求可用日志列表。 |
| 118 | LOG_ENTRY | 响应日志列表请求，提供日志相关信息。 |
| 119 | LOG_REQUEST_DATA | 请求日志的一部分数据，指定日志ID、偏移和字节数。 |
| 120 | LOG_DATA | 响应日志数据请求，发送日志片段。 |
| 121 | LOG_ERASE | 删除所有日志。 |
| 122 | LOG_REQUEST_END | 停止日志传输并恢复正常日志记录。 |
| 130 | DATA_TRANSMISSION_HANDSHAKE | 图像传输协议的握手消息，用于初始化等。 |
| 131 | ENCAPSULATED_DATA | 图像传输协议的数据分组，包含序列号和图像数据。 |
| 266 | LOGGING_DATA | 包含日志数据的消息，用于日志传输。 |
| 267 | LOGGING_DATA_ACKED | 包含需要确认的日志数据，接收方需返回LOGGING_ACK。 |
| 268 | LOGGING_ACK | 对LOGGING_DATA_ACKED消息的确认。 |

### 九、硬件与设备类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 34 | RC_CHANNELS_SCALED | 报告缩放后的RC通道值，范围为-10000至10000。 |
| 35 | RC_CHANNELS_RAW | 报告原始RC通道值（微秒）。 |
| 36 | SERVO_OUTPUT_RAW [DEP] | 报告舵机输出原始值，已被ACTUATOR_OUTPUT_STATUS替代。 |
| 70 | RC_CHANNELS_OVERRIDE | 覆盖RC通道值，替代无线电接收的RC信号。 |
| 65 | RC_CHANNELS | 报告RC通道的PPM值（微秒），包含通道数量和信号强度。 |
| 125 | POWER_STATUS | 电源状态报告，包括5V rail电压、伺服rail电压等。 |
| 126 | SERIAL_CONTROL | 控制串行端口，用于访问板载串行外设。 |
| 147 | BATTERY_STATUS | 电池信息，包括温度、电压、剩余电量等。 |
| 148 | AUTOPILOT_VERSION | 自动驾驶仪软件的版本和能力。 |
| 256 | SETUP_SIGNING | 设置MAVLink2签名密钥，全零密钥禁用签名功能。 |
| 257 | BUTTON_CHANGE | 报告按钮状态变化，包含按钮状态位掩码。 |
| 290 | ESC_INFO [WIP] | ESC信息（低速率传输），包含索引、温度等。 |
| 291 | ESC_STATUS [WIP] | ESC信息（高速率传输），包含转速、电压等。 |
| 299 | WIFI_CONFIG_AP | 配置WiFi AP的SSID、密码和模式。 |
| 371 | FUEL_STATUS | 燃料状态信息，包含最大/消耗/剩余燃料等。 |
| 372 | BATTERY_INFO [WIP] | 电池静态信息或低频更新信息。 |
| 373 | GENERATOR_STATUS | 发电系统遥测信息，包含转速、功率等。 |
| 375 | ACTUATOR_OUTPUT_STATUS | 电机输出的原始值（如Pixhawk的MAIN和AUX端口）。 |
| 390 | ONBOARD_COMPUTER_STATUS [WIP] | 机载计算机的硬件状态，如CPU使用率、温度等。 |
| 74 | VFR_HUD | 报告固定翼飞行器HUD(仪表盘)常用 metrics，包括空速、地速、航向、油门、海拔和爬升率。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **65** | `RC_CHANNELS` | **遥控器通道原始值**，手动控制信号来源。 |
| **70** | `RC_CHANNELS_OVERRIDE` | **覆盖遥控器信号**，紧急接管或自动化测试必备。 |
| **74** | `VFR_HUD` | **飞行仪表数据**（空速/高度/爬升率），驾驶员界面核心。 |
| **148** | `AUTOPILOT_VERSION` | **飞控固件版本和能力**，兼容性检查依据。 |

### 十、相机与云台类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 112 | CAMERA_TRIGGER | 相机-IMU触发和同步消息，包含图像帧时间戳和序列号。 |
| 259 | CAMERA_INFORMATION | 相机信息，包含厂商、型号、焦距等。 |
| 260 | CAMERA_SETTINGS | 相机设置，包含模式、缩放级别等。 |
| 262 | CAMERA_CAPTURE_STATUS | 相机捕捉状态信息，包含图像/视频捕捉状态等。 |
| 263 | CAMERA_IMAGE_CAPTURED | 已捕捉图像的信息，包含时间戳、位置等。 |
| 265 | MOUNT_ORIENTATION [DEP] | 挂载的姿态，已被MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW替代。 |
| 271 | CAMERA_FOV_STATUS | 相机视场信息，包含水平/垂直视场角等。 |
| 275 | CAMERA_TRACKING_IMAGE_STATUS | 相机跟踪状态（主动跟踪时发送）。 |
| 276 | CAMERA_TRACKING_GEO_STATUS | 相机跟踪状态（含地理信息）。 |
| 277 | CAMERA_THERMAL_RANGE | 相机绝对热范围，包含最高/最低温度及对应位置。 |
| 280 | GIMBAL_MANAGER_INFORMATION | 高级云台管理器信息，包含能力标志等。 |
| 281 | GIMBAL_MANAGER_STATUS | 高级云台管理器当前状态。 |
| 282 | GIMBAL_MANAGER_SET_ATTITUDE | 控制云台姿态的高级消息。 |
| 283 | GIMBAL_DEVICE_INFORMATION | 低级云台信息，包含厂商、型号等。 |
| 284 | GIMBAL_DEVICE_SET_ATTITUDE | 控制云台设备姿态的低级消息。 |
| 285 | GIMBAL_DEVICE_ATTITUDE_STATUS | 云台设备的姿态状态报告。 |
| 286 | AUTOPILOT_STATE_FOR_GIMBAL_DEVICE | 自动驾驶仪状态（用于云台设备估计校正）。 |
| 287 | GIMBAL_MANAGER_SET_PITCHYAW | 设置云台管理器的俯仰和偏航角（高速消息）。 |
| 288 | GIMBAL_MANAGER_SET_MANUAL_CONTROL | 手动控制云台的高级消息。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **259** | `CAMERA_INFORMATION` | **相机设备描述**（厂商/焦距），初始化配置。 |
| **260** | `CAMERA_SETTINGS` | **相机参数实时状态**（模式/缩放）。 |

### 十一、仿真与测试类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 90 | HIL_STATE [DEP] | 从仿真发送到自动驾驶仪的状态消息，已被HIL_STATE_QUATERNION替代。 |
| 91 | HIL_CONTROLS | 自动驾驶仪发送到仿真的硬件在环控制输出。 |
| 92 | HIL_RC_INPUTS_RAW | 仿真发送到自动驾驶仪的原始RC输入。 |
| 93 | HIL_ACTUATOR_CONTROLS | 自动驾驶仪发送到仿真的执行器控制输出。 |
| 107 | HIL_SENSOR | 仿真的IMU读数（SI单位，NED机体坐标系）。 |
| 108 | SIM_STATE | 仿真环境状态，包含姿态、位置、速度等。 |
| 113 | HIL_GPS | 仿真的GPS原始位置数据。 |
| 114 | HIL_OPTICAL_FLOW | 仿真的光流数据。 |
| 115 | HIL_STATE_QUATERNION | 从仿真发送到自动驾驶仪的四元数状态消息，避免欧拉角奇点。 |
| 249 | MEMORY_VECT | 发送原始控制器内存数据，用于测试和调试。 |
| 250 | DEBUG_VECT | 用命名的3D向量调试，包含名称和向量值。 |
| 251 | NAMED_VALUE_FLOAT | 发送浮点键值对，用于测试和调试。 |
| 252 | NAMED_VALUE_INT | 发送整数键值对，用于测试和调试。 |
| 253 | STATUSTEXT | 状态文本消息，在QGroundControl的COMM控制台显示。 |
| 254 | DEBUG | 发送调试值，索引用于区分不同值。 |
| 350 | DEBUG_FLOAT_ARRAY | 大型调试/原型数组，不建议用于生产代码。 |

### 十二、地形与环境类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 133 | TERRAIN_REQUEST | 请求地形数据和状态，指定西南角坐标等。 |
| 134 | TERRAIN_DATA | 从GCS发送的地形数据，对应TERRAIN_REQUEST的请求。 |
| 135 | TERRAIN_CHECK | 请求飞行器报告特定位置的地形高度。 |
| 136 | TERRAIN_REPORT | 无人机发送的地形地图下载进度或地形检查响应。 |
| 231 | WIND_COV | 飞行器的风速估计，包含北/东/下方向风速等。 |
| 247 | COLLISION | 潜在碰撞的信息，包含威胁等级和最小距离/时间。 |
| 330 | OBSTACLE_DISTANCE | 传感器前方的障碍物距离，按角度递增排列。 |

### 十三、事件与信息类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 192 | MAG_CAL_REPORT | 报告指南针校准结果，包含校准状态、残差等。 |
| 225 | EFI_STATUS | EFI（电子燃油喷射）状态输出，包含转速、温度等。 |
| 230 | ESTIMATOR_STATUS | 估计器状态消息，包括有效输出标志、估计精度等。 |
| 234 | HIGH_LATENCY [DEP] | 高延迟连接的消息，已被HIGH_LATENCY2替代。 |
| 235 | HIGH_LATENCY2 | 高延迟连接的消息（版本2），包含位置、速度等信息。 |
| 241 | VIBRATION | 振动水平和加速度计削波计数。 |
| 246 | ADSB_VEHICLE | ADS-B飞行器的位置和信息，包含ICAO地址、速度等。 |
| 264 | FLIGHT_INFORMATION | 飞行信息，包括武装、起飞、着陆时间等。 |
| 261 | STORAGE_INFORMATION | 存储介质信息，包含容量、使用量等。 |
| 269 | VIDEO_STREAM_INFORMATION | 视频流信息，包含帧率、分辨率等。 |
| 270 | VIDEO_STREAM_STATUS | 视频流状态信息，包含状态标志、比特率等。 |
| 380 | TIME_ESTIMATE_TO_TARGET [WIP] | 各种事件和动作的时间估计，如安全返回、着陆等。 |
| 410 | EVENT [WIP] | 事件消息，每个新事件有唯一序列号，包含事件ID等。 |
| 411 | CURRENT_EVENT_SEQUENCE [WIP] | 定期广播组件的最新事件序列号，用于检查丢失事件。 |
| 412 | REQUEST_EVENT [WIP] | 请求重发一个或多个事件，指定起始和结束序列号。 |
| 413 | RESPONSE_EVENT_ERROR [WIP] | 对REQUEST_EVENT的错误响应，包含错误原因。 |
| 440 | ILLUMINATOR_STATUS | 照明器状态，包含亮度、温度等信息。 |

**重要消息**
| 消息ID | 消息名称 | 重要性说明 |
|--------|----------|------------|
| **253** | `STATUSTEXT` | **状态文本信息**（警告/错误），飞控日志输出核心。 |
| **230** | `ESTIMATOR_STATUS` | **导航滤波器状态**，定位健康诊断（如GPS失效标志）。 |
| **241** | `VIBRATION` | **振动水平报告**，硬件异常预警。 |

### 十四、无人机ID与识别类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 12900 | OPEN_DRONE_ID_BASIC_ID | OpenDroneID基本ID消息数据，包含UAS ID等。 |
| 12901 | OPEN_DRONE_ID_LOCATION | OpenDroneID位置消息数据，包含位置、速度等。 |
| 12902 | OPEN_DRONE_ID_AUTHENTICATION | OpenDroneID认证消息数据，用于身份验证。 |
| 12903 | OPEN_DRONE_ID_SELF_ID | OpenDroneID自ID消息数据，用于操作员身份声明。 |
| 12904 | OPEN_DRONE_ID_SYSTEM | OpenDroneID系统消息数据，用于系统状态报告。 |
| 12905 | OPEN_DRONE_ID_OPERATOR_ID | OpenDroneID操作员ID消息数据，用于操作员标识。 |
| 12915 | OPEN_DRONE_ID_MESSAGE_PACK | OpenDroneID消息包，用于低带宽无线传输。 |
| 12918 | OPEN_DRONE_ID_ARM_STATUS | 远程ID系统的启用和就绪状态，用于飞行控制器arming条件。 |
| 12919 | OPEN_DRONE_ID_SYSTEM_UPDATE | 更新OPEN_DRONE_ID_SYSTEM消息中的位置信息。 |

### 十五、其他特殊功能类
| 消息ID | 消息名称 | 作用描述 |
|--------|----------|----------|
| 7 | AUTH_KEY | 发送加密签名/密钥以标识系统，需通过加密通道传输。 |
| 48 | SET_GPS_GLOBAL_ORIGIN [DEP] | 设置本地原点的GPS坐标，已被MAV_CMD_SET_GLOBAL_ORIGIN替代。 |
| 49 | GPS_GLOBAL_ORIGIN | 发布本地原点的GPS坐标，用于坐标转换。 |
| 54 | SAFETY_SET_ALLOWED_AREA | 设置安全区域（立方体），定义MAV可接受的航点范围。 |
| 55 | SAFETY_ALLOWED_AREA | 读取MAV当前的安全区域设置。 |
| 62 | NAV_CONTROLLER_OUTPUT | 报告导航和位置控制器状态，包括各种误差。 |
| 66 | REQUEST_DATA_STREAM [DEP] | 请求数据流，已被MAV_CMD_SET_MESSAGE_INTERVAL替代。 |
| 67 | DATA_STREAM [DEP] | 报告数据流状态，已被MESSAGE_INTERVAL替代。 |
| 101 | GLOBAL_VISION_POSITION_ESTIMATE | 视觉源的全球位置/姿态估计。 |
| 102 | VISION_POSITION_ESTIMATE | 视觉源的本地位置/姿态估计。 |
| 103 | VISION_SPEED_ESTIMATE | 视觉源的速度估计。 |
| 104 | VICON_POSITION_ESTIMATE | Vicon运动系统的全球位置估计。 |
| 138 | ATT_POS_MOCAP | 运动捕捉的姿态和位置。 |
| 142 | RESOURCE_REQUEST | 自动驾驶仪请求资源（文件、二进制等）。 |
| 144 | FOLLOW_TARGET | 指定系统的当前运动信息，包含位置、速度等。 |
| 146 | CONTROL_SYSTEM_STATE | 用于控制系统的平滑、单调系统状态。 |
| 149 | LANDING_TARGET | 着陆目标的位置信息，用于精确着陆。 |
| 162 | FENCE_STATUS | 地理围栏状态，在围栏启用时发送。 |
| 243 | SET_HOME_POSITION [DEP] | 设置home位置，已被MAV_CMD_DO_SET_HOME替代。 |
| 248 | V2_EXTENSION | 在V1帧中实现V2有效载荷规范的消息，用于过渡支持。 |
| 258 | PLAY_TUNE [DEP] | 控制飞行器音调生成，已被PLAY_TUNE_V2替代。 |
| 301 | AIS_VESSEL | AIS船舶的位置和信息，包含MMSI、速度等。 |
| 310 | UAVCAN_NODE_STATUS | UAVCAN节点的一般状态信息。 |
| 311 | UAVCAN_NODE_INFO | 特定UAVCAN节点的一般信息。 |
| 331 | ODOMETRY | 里程计消息，符合ROS REP 147标准。 |
| 332 | TRAJECTORY_REPRESENTATION_WAYPOINTS [DEP] | 用航点描述轨迹，已废弃。 |
| 333 | TRAJECTORY_REPRESENTATION_BEZIER [DEP] | 用贝塞尔控制点描述轨迹，已废弃。 |
| 340 | UTM_GLOBAL_POSITION | GPS和传感器融合的全球位置，包含速度、不确定性等。 |
| 360 | ORBIT_EXECUTION_STATUS | 轨道执行期间的飞行器状态报告。 |
| 370 | SMART_BATTERY_INFO [DEP] | 智能电池信息，已被BATTERY_INFO替代。 |
| 385 | TUNNEL | 在组件间传输变长数据的消息，支持自定义编码。 |
| 386 | CAN_FRAME | 转发的CAN帧，应MAV_CMD_CAN_FORWARD请求发送。 |
| 387 | CANFD_FRAME | 转发的CANFD帧，应MAV_CMD_CAN_FORWARD请求发送。 |
| 388 | CAN_FILTER_MODIFY | 修改CAN消息转发的过滤器，优化低带宽链路传输。 |
| 395 | COMPONENT_INFORMATION [DEP] | 组件信息消息，已被COMPONENT_METADATA替代。 |
| 396 | COMPONENT_INFORMATION_BASIC | 基本组件信息数据，包含能力标志等。 |
| 397 | COMPONENT_METADATA [WIP] | 组件元数据消息，用于获取组件元数据。 |
| 400 | PLAY_TUNE_V2 | 播放飞行器音调，替代PLAY_TUNE。 |
| 401 | SUPPORTED_TUNES | 飞行器支持的音调格式，响应请求消息。 |
| 9000 | WHEEL_DISTANCE | 每个报告车轮的累计行驶距离。 |
| 9005 | WINCH_STATUS | 绞盘状态，包含释放长度、速度等信息。 |

**重要消息**
| 分类 | 消息ID | 消息名称 | 作用 |
|------|--------|----------|------|
| 安全 | **54** | `SAFETY_SET_ALLOWED_AREA` | 设置电子围栏区域 |
| 避障 | **330** | `OBSTACLE_DISTANCE` | 障碍物距离扫描数据 |
| 着陆 | **149** | `LANDING_TARGET` | 精准着陆目标信息 |
| 识别 | **12900-12905** | `OPEN_DRONE_ID_*` | 无人机远程ID（法规要求） |

---

### **总结：最高优先级消息**
以下消息在**几乎所有无人机系统**中不可或缺：
1. **`HEARTBEAT (0)`**  
2. **`SYS_STATUS (1)`**  
3. **`GLOBAL_POSITION_INT (33)`**  
4. **`ATTITUDE (30)`** 或 **`ATTITUDE_QUATERNION (31)`**  
5. **`RC_CHANNELS (65)`**  
6. **`BATTERY_STATUS (147)`**  
7. **`COMMAND_LONG (76)`** + **`COMMAND_ACK (77)`**  
8. **`PARAM_SET (23)`** + **`PARAM_VALUE (22)`**  
9. **`STATUSTEXT (253)`**  

# Commands概念
**核心概念：命令是动作的抽象**

在 MAVLink 中，命令 (`MAVLink Command` 或 `MAV_CMD`) 本质上代表一个**希望接收系统执行的具体动作或操作**。它不是一个原始的数据传输，而是一个“指令”。

**关键特性详解：**

1.  **标准化枚举 (`MAV_CMD`)：**
    *   所有可能的命令都定义在一个名为 `MAV_CMD` 的大型枚举中。每个枚举项都有一个唯一的整数值和一个描述性的名称（例如：`MAV_CMD_NAV_WAYPOINT`， `MAV_CMD_COMPONENT_ARM_DISARM`， `MAV_CMD_REQUEST_MESSAGE`）。
    *   这个枚举是 MAVLink 规范的一部分，确保了不同厂商的飞控、地面站、配套硬件等对同一个命令值有**一致的理解**。这是命令与自由格式消息的关键区别之一。

2.  **结构化参数：**
    *   每个命令都携带最多 **7 个参数**，用于指定命令执行的具体细节。参数类型严格定义为：
        *   **模式 A：** 7 个 `float` 类型的参数。
        *   **模式 B：** 4 个 `float` 类型的参数 + 1 个 `int32_t` 类型的参数 + 2 个 `float` 类型的参数。
    *   **参数含义固定：** 对于每个特定的 `MAV_CMD_XXX` 命令，其 7 个参数中每个位置代表什么含义是**预先定义好**的（在 MAVLink XML 定义和文档中指定）。例如：
        *   `MAV_CMD_NAV_WAYPOINT`：参数1 可能是纬度，参数2 可能是经度，参数3 可能是高度，等等。
        *   `MAV_CMD_COMPONENT_ARM_DISARM`：参数1 通常用于表示 “1 = 上锁， 0 = 解锁”。
    *   这种严格的结构化使得命令解析和分发更简单、更高效，但也限制了它能携带的信息类型（只能是数字）。

3.  **封装在协议消息中传输：**
    *   命令本身 **不是** 一个可以直接在链路上发送的独立 MAVLink 消息帧 (`frame`)。
    *   命令必须被“打包”进特定的 **任务协议 (Mission Protocol)** 或 **命令协议 (Command Protocol)** 消息中才能发送：
        *   **任务协议 (`Mission Protocol`)：** 主要用于上传、下载、管理**任务序列**。命令被打包进 `MISSION_ITEM` 或 `MISSION_ITEM_INT` 消息中发送。这些消息包含额外的字段，如目标系统/组件、任务序列号、坐标系、容差值等。任务协议适用于**预先规划好的一系列命令**（航点、动作等）。
        *   **命令协议 (`Command Protocol`)：** 主要用于发送**即时执行的单个命令**。命令被打包进 `COMMAND_LONG` 或 `COMMAND_INT` 消息中发送。
            *   `COMMAND_LONG`： 使用 7 个 `float` 参数。
            *   `COMMAND_INT`： 使用 4 个 `float` + 1 个 `int32_t` + 2 个 `float` 参数，并且经纬度等位置信息使用整数缩放格式（`1e7` 度），精度更高且避免浮点舍入误差，是推荐的新方式。
        *   这些协议消息 (`MISSION_ITEM[_INT]`, `COMMAND_LONG`, `COMMAND_INT`) 才是真正在 MAVLink 链路上传输的载体。

4.  **确认机制 (ACK/NACK)：**
    *   命令协议 (`COMMAND_LONG/INT`) 和任务协议执行时，都**内置了强制的确认机制**。
    *   接收系统**必须**回复一个 `COMMAND_ACK` 消息给发送方，告知命令的处理结果：
        *   `MAV_RESULT_ACCEPTED`： 命令有效，已开始执行或已入队。
        *   `MAV_RESULT_TEMPORARILY_REJECTED`： 当前无法执行（如正忙），稍后可能可以。
        *   `MAV_RESULT_DENIED`： 命令被拒绝（如权限不足、当前模式不支持）。
        *   `MAV_RESULT_UNSUPPORTED`： 命令不被识别或不支持。
        *   `MAV_RESULT_FAILED`： 命令执行过程中出错。
        *   `MAV_RESULT_IN_PROGRESS`： 命令已接受，正在执行中（对于耗时命令）。
        *   等等...
    *   这种明确的 ACK/NACK 机制是命令的核心优势之一，确保了指令的可靠传递和执行反馈。发送方知道命令是否被收到、是否被接受、执行是否成功。这对于飞行安全和控制至关重要。

5.  **主要使用场景：**
    *   **飞行任务 (Mission)：** 航点 (`WAYPOINT`)、起降 (`TAKEOFF`, `LAND`)、触发动作 (`DO_SET_SERVO`, `DO_DIGICAM_CONTROL`) 等构成任务序列的核心元素。
    *   **即时控制：** 发送单个立即执行的指令，如：
        *   解锁/上锁 (`COMPONENT_ARM_DISARM`)
        *   切换飞行模式 (`SET_MODE`)
        *   请求数据流 (`REQUEST_DATA_STREAM`, 虽然现在更推荐 `REQUEST_MESSAGE`)
        *   设置参数 (`PARAM_SET`)
        *   请求发送特定消息 (`REQUEST_MESSAGE`)
        *   触发相机拍照 (`IMAGE_START_CAPTURE`)
        *   发送校准指令 (`DO_TRIGGER_CALIBRATION`)
    *   **需要可靠确认的操作：** 任何不能丢失、且必须知道执行结果的操作。

6.  **工作流程示例 (以 `COMMAND_LONG` 解锁为例)：**
    1.  地面站 (`GCS`) 构造一个 `COMMAND_LONG` 消息：
        *   `command` 字段 = `MAV_CMD_COMPONENT_ARM_DISARM`
        *   `param1` = `1.0` (表示解锁)
        *   `param2` ... `param7` = 0 或按需设置 (如强制解锁标志)。
        *   设置目标飞控 (`target_system`, `target_component`)。
        *   设置 `confirmation` = 0 (首次发送)。
    2.  地面站通过串口/网络发送该 `COMMAND_LONG` 消息。
    3.  飞控接收到 `COMMAND_LONG` 消息。
    4.  飞控解析 `command` 字段，知道是解锁命令。
    5.  飞控检查 `param1` 为 1，执行解锁逻辑（检查状态、解锁电机等）。
    6.  飞控构造 `COMMAND_ACK` 消息：
        *   `command` 字段 = `MAV_CMD_COMPONENT_ARM_DISARM` (与请求一致)
        *   `result` 字段 = `MAV_RESULT_ACCEPTED` (成功) 或 `MAV_RESULT_DENIED` (失败原因如未校准) 等。
    7.  飞控将 `COMMAND_ACK` 发送回地面站。
    8.  地面站收到 `COMMAND_ACK`，根据 `result` 更新 UI 状态（如显示解锁成功或失败原因）。

**命令 (Commands) vs 消息 (Messages) 的再强调：**

*   **命令是“做什么”的指令**： 它代表一个希望被执行的动作，有严格的结构 (`MAV_CMD` + 7参数) 和内置的确认机制 (`COMMAND_ACK`)。它必须通过特定协议消息 (`COMMAND_LONG/INT`, `MISSION_ITEM_INT`) 传输。
*   **消息是“数据是什么”的载体**： 它用于传输各种状态信息（位置 `GLOBAL_POSITION_INT`、姿态 `ATTITUDE`、电池状态 `BATTERY_STATUS`）、配置信息（参数 `PARAM_VALUE`）、日志、自定义数据等。消息结构灵活多样（由 `message id` 定义），可以是广播、流式传输或点对点，通常没有内置的强制 ACK（虽然某些消息可能有请求-响应模式，如 `PARAM_REQUEST_READ` -> `PARAM_VALUE`）。


## 最常用的核心命令

### 一、导航与航点控制
| 命令ID | 名称 | 核心作用 | 适用场景 |
| ---- | ---- | ---- | ---- |
| 16 | MAV_CMD_NAV_WAYPOINT | 导航至指定航点，支持停留时间、接受半径等参数配置 | 任务中的航点规划、路径导航 |
| 17 | MAV_CMD_NAV_LOITER_UNLIM | 在指定位置无限期盘旋（固定翼绕圈，多旋翼悬停） | 等待指令、区域监控、目标跟踪 |
| 19 | MAV_CMD_NAV_LOITER_TIME | 在指定位置盘旋指定时长后继续任务 | 定点观测、等待条件满足 |
| 20 | MAV_CMD_NAV_RETURN_TO_LAUNCH | 自动返回起飞点并准备着陆 | 紧急返航、任务结束返航 |
| 82 | MAV_CMD_NAV_SPLINE_WAYPOINT | 通过平滑的样条曲线导航至航点 | 要求路径平滑的任务（如测绘） |


### 二、起降控制
| 命令ID | 名称 | 核心作用 | 适用场景 |
| ---- | ---- | ---- | ---- |
| 21 | MAV_CMD_NAV_LAND | 在指定经纬度和高度着陆，支持精确着陆模式 | 常规着陆、定点着陆任务 |
| 22 | MAV_CMD_NAV_TAKEOFF | 从地面/手持起飞，支持俯仰角和偏航角配置 | 常规起飞、任务初始阶段 |
| 84 | MAV_CMD_NAV_VTOL_TAKEOFF | VTOL飞行器（如复合翼）从垂直起降模式切换至前向飞行 | VTOL飞行器起飞与模式转换 |
| 85 | MAV_CMD_NAV_VTOL_LAND | VTOL飞行器从飞行模式切换至垂直着陆模式 | VTOL飞行器着陆与模式转换 |
| 191 | MAV_CMD_DO_GO_AROUND | 中止当前着陆并复飞至指定高度 | 着陆异常时的安全复飞 |


### 三、任务流程控制
| 命令ID | 名称 | 核心作用 | 适用场景 |
| ---- | ---- | ---- | ---- |
| 112 | MAV_CMD_CONDITION_DELAY | 延迟执行下一条任务命令（按时间或特定时刻） | 任务步骤间等待、时序控制 |
| 176 | MAV_CMD_DO_SET_MODE | 设置飞行器系统模式（如自动、手动、制导模式） | 模式切换、任务阶段调整 |
| 177 | MAV_CMD_DO_JUMP | 在任务列表中跳至指定命令，支持重复次数配置 | 任务循环、分支逻辑执行 |
| 224 | MAV_CMD_DO_SET_MISSION_CURRENT | 设置当前任务项，跳过中间项直接执行目标项 | 任务重置、紧急任务切换 |
| 300 | MAV_CMD_MISSION_START | 开始执行任务，指定任务的起始和结束项 | 任务启动、任务分段执行 |


### 四、设备与动作控制
| 命令ID | 名称 | 核心作用 | 适用场景 |
| ---- | ---- | ---- | ---- |
| 178 | MAV_CMD_DO_CHANGE_SPEED | 调整飞行器速度或油门设定值（持续生效至下次修改） | 任务中速度调整、节能飞行 |
| 179 | MAV_CMD_DO_SET_HOME | 设置home位置（当前位置或指定经纬度） | 自定义返航点、更新基准位置 |
| 181 | MAV_CMD_DO_SET_RELAY | 控制继电器开关（如挂载设备供电、投放机构） | 有效载荷投放、设备供电控制 |
| 183 | MAV_CMD_DO_SET_SERVO | 控制伺服电机的PWM值（如相机云台、舵面） | 云台角度调整、机械结构控制 |
| 206 | MAV_CMD_DO_SET_CAM_TRIGG_DIST | 按距离触发相机拍摄（每次超过指定距离时拍照） | 测绘、巡检等按路径间隔拍照任务 |


### 五、安全与应急控制
| 命令ID | 名称 | 核心作用 | 适用场景 |
| ---- | ---- | ---- | ---- |
| 185 | MAV_CMD_DO_FLIGHTTERMINATION | 立即终止飞行（触发安全措施如降落伞、电机停转） | 紧急故障时的安全终止 |
| 207 | MAV_CMD_DO_FENCE_ENABLE | 启用/禁用地理围栏（限制飞行器活动区域） | 区域限制、安全边界设置 |
| 400 | MAV_CMD_COMPONENT_ARM_DISARM | 武装/解除武装飞行器（电机启动/关闭） | 起飞前准备、任务结束安全措施 |
| 401 | MAV_CMD_RUN_PREARM_CHECKS | 执行预武装检查（传感器、设备状态验证） | 起飞前自检、故障排查 |


### 六、相机与载荷控制
| 命令ID | 名称 | 核心作用 | 适用场景 |
| ---- | ---- | ---- | ---- |
| 200 | MAV_CMD_DO_CONTROL_VIDEO | 控制相机视频传输（启用/禁用、压缩模式） | 实时图传开关、传输质量调整 |
| 2000 | MAV_CMD_IMAGE_START_CAPTURE | 开始图像捕获序列（按间隔拍摄指定数量照片） | 航拍、测绘任务中的图像采集 |
| 2001 | MAV_CMD_IMAGE_STOP_CAPTURE | 停止当前图像捕获序列 | 结束拍照任务、节省存储空间 |
| 530 | MAV_CMD_SET_CAMERA_MODE | 设置相机运行模式（拍照、录像、回放等） | 相机工作模式切换 |
| 531 | MAV_CMD_SET_CAMERA_ZOOM | 控制相机变焦（绝对/相对变焦值） | 远距离目标拍摄、细节放大 |
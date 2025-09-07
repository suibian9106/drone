## MAVLink协议

MAVLink（Micro Air Vehicle Link）是一个轻量级、高效、开源的消息传递协议和编组库，专为资源受限的系统（如无人机、机器人）与地面站或其他系统之间的通信而设计。它实现通信的核心机制可以分解为以下几个关键方面：

1.  **基于消息的架构 (Message-Based Architecture)**
    *   **核心概念：** MAVLink通信的基本单位是**消息**。每条消息代表一个特定类型的数据或指令（例如：`HEARTBEAT`， `GPS_RAW_INT`， `COMMAND_LONG`）。
    *   **消息定义：** 消息在 XML 文件中定义，指定其唯一的 `message ID`、名称、字段（名称、数据类型、描述）。这些定义文件是生成不同编程语言（C, C++, Python, Java 等）编组库的基础。
    *   **结构化数据：** 每条消息包含一组结构化的字段，承载特定信息（如经纬度、高度、姿态角、指令参数等）。

2.  **帧结构 (Frame Structure)**
    MAVLink消息在传输时被封装成一个标准的二进制帧（`小端序`）。这是协议的核心结构，确保数据的可靠传输和解析。一个完整的MAVLink v1.0/v2.0帧包含以下部分（括号中为字节数），详细内容参考[MAVLink官网说明](https://mavlink.io/en/guide/serialization.html)：

    <img src="images\MAVLink 2 Packet Format.png" alt="MAVLink V2 帧结构">

    *   **起始位 (1 byte):** (`0xFE` for v1.0, `0xFD` for v2.0) - 标志着一个新帧的开始，接收方据此同步。
    *   **载荷长度 (1 byte):** 指示紧随其后的`Payload`部分的字节数。v2.0支持更长的消息。
    *   **不兼容标志 (1 byte, v2.0 only):** 指示接收方如果不理解该标志位必须丢弃该帧（用于协议未来扩展）。
    *   **兼容标志 (1 byte, v2.0 only):** 接收方即使不理解该标志位也可以安全地忽略它并解析消息。
    *   **序列号 (1 byte):** 发送方为每个发送的帧分配的递增序列号。**关键作用**：帮助接收方检测**丢包**（序列号不连续）和**乱序**（序列号大于预期）。
    *   **系统 ID (1 byte):** 标识消息来源的系统（例如：一架无人机、一个地面站）。范围 1-255。
    *   **组件 ID (1 byte):** 标识消息来源系统内的特定组件（例如：飞控主处理器（ID 1）、相机（ID 100）、避障传感器（ID 101））。范围 1-255。
    *   **消息 ID (1 byte for v1.0, 3 bytes for v2.0):** 唯一标识消息类型（如 `HEARTBEAT` 是 0, `ATTITUDE` 是 30）。告诉接收方如何解析后续的`Payload`。
    *   **载荷 (Payload Length 字节):** 包含消息的实际数据内容，按照消息定义文件中的字段顺序和类型进行打包。
    *   **校验码 (2 bytes):** 这是帧完整性的核心保障。使用 CRC-16/MCRF4XX 算法对整个帧（从载荷长度到载荷结束，加上 CRC_EXTRA，CRC_EXTRA 是根据消息ID对应的消息类型决定的，每个 MAVLink 消息都有一个固定的 CRC_EXTRA 值，它来源于 .xml 描述文件自动生成的 C 语言头文件中 ）计算出的校验和。**关键作用**：
        *   **检错：** 接收方重新计算校验码并与收到的校验码比较。如果不匹配，表明帧在传输过程中发生了比特错误（噪声、干扰），该帧会被丢弃。
        *   **防篡改：** 虽然MAVLink本身不加密，但校验码能检测到对帧内容的任何意外或恶意修改（确保消息来自可信源并完整接收需要更高层的安全机制）。
    *   **签名 (13 bytes, v2.0 optional):** 提供基于共享密钥的身份验证和防重放攻击保护（可选）。
    ---

    **备注：**
    * [系统ID和组件ID分配](https://mavlink.io/zh/services/mavlink_id_assignment.html)

    **步骤 1：设置系统 ID (sysid)**
    1. **修改飞控的 `sysid`**：
    * 通过飞控参数设置（如 PX4 的 `MAV_SYS_ID`，ArduPilot 的 `SYSID_THISMAV`）。
    * （可选）未来可能支持命令 `MAV_CMD_DO_SET_SYS_CMP_ID`（开发中）。
    2. **重启设备**：
    * 断开网络连接（如拔掉数传），确保组件只能检测到本机飞控。
    3. **组件自动同步 `sysid`**：
    * 组件启动后监听飞控的 `HEARTBEAT` 消息：
        * **3 秒内仅检测到 1 个飞控** → 组件将自身 `sysid` 设为该飞控的 ID。
        * **检测到多个飞控或无飞控** → 组件保持原 `sysid`。

    **步骤 2：分配组件 ID (compid)**
    * **手动分配**：由系统集成商完成。
    * **默认值**：组件出厂时通常使用 `MAV_COMPONENT` 中的首个默认值（如摄像头为 `100`）。
    * **同类型多组件处理**：
    * 手动为每个组件**按顺序分配唯一 ID**（如 `100`, `101`, `102`）。
    * 修改方式因组件而异（部分组件可能不支持修改）。

    **最佳实践**
    1. **简单系统**：使用默认 ID 可降低冲突风险（如单飞控 + 单摄像头）。
    2. **复杂系统**：
    * **多机网络**：务必为每台设备分配唯一 `sysid`。
    * **多同类型组件**：手动分配连续且唯一的 `compid`。
    3. **类型识别**：始终通过 `HEARTBEAT.type` 确认组件类型，而非依赖 ID 值[MAV_TYPE](https://mavlink.io/en/messages/common.html#MAV_TYPE)。

    **核心原则**：`sysid` 全局唯一，`compid` 系统内唯一，组件类型通过心跳消息确认。


    * [Message ID对应关系](https://mavlink.io/en/messages/common.html)
    **消息ID由XML文件进行分配，大部分消息都已经有通用的ID**
    ---

    抓包说明：

    <img src="images\MAVLink 1 Packet Format.png" alt="MAVLink V1 帧结构">

    <img src="images\MAVLink V1 HEX Packet.png" alt="MAVLink V1 十六进制包">

    *   **0xFE (1 byte):** MAVLink v1 协议
    *   **0x3E (1 byte):** 载荷`Payload`字节数为62
    *   **0x57 (1 byte):** 发送的该帧序列号为87。**关键作用**：帮助接收方检测**丢包**（序列号不连续）和**乱序**（序列号大于预期）
    *   **0x01 (1 byte):** 系统ID为1，通常为一架无人机
    *   **0x01 (1 byte):** 组件ID为1，通常为飞控
    *   **0x69 (1 byte for v1.0):** 消息ID为105，对应类型为[HIGHRES_IMU](https://mavlink.io/en/messages/common.html#HIGHRES_IMU)
    *   **载荷 (Payload Length 字节):** 包含消息的实际数据内容，按照消息定义文件中的字段顺序和类型进行打包。[Message ID对应关系](https://mavlink.io/en/messages/common.html)
    *   **0x23 0xD8 (2 bytes):** 使用 CRC-16/MCRF4XX 算法对整个帧的计算出的校验和，从载荷长度到载荷结束，加上 CRC_EXTRA，message ID 105 对应的 CRC_EXTRA为93(0x5D)

3.  **路由 (Routing)**    
    [官方描述](https://mavlink.io/en/guide/routing.html)

    The protocol defines two 8-bit fields that can (optionally) be specified in the `message payload` to indicate where the message should be sent/routed.可以在消息内容中定义两个八位的fields为target_system和target_component

    **系统/组件应在满足以下任一条件时本地处理消息：**

    * 该消息为广播消息（target_system 字段缺失或值为零）
    * target_system 匹配本系统ID，且目标组件为广播（target_component 缺失或值为零）
    * target_system 匹配本系统ID，且 target_component 与组件ID相符
    * target_system 匹配本系统ID，且目标组件未知（即本组件从未在任何链路上收到过含该 target_system/target_component 组合的消息）

    **系统应在满足以下任一条件时将消息转发至其他链路：**

    * 该消息为广播消息（target_system 字段缺失或值为零）
    * target_system 与系统ID不匹配，且系统知晓目标系统的链路（即此前曾在链路上收到过源出 target_system 的消息）
    * target_system 匹配本系统ID且消息含 target_component 字段，同时系统曾在链路上收到过该 target_system/target_component 组合的消息
    
4.  **序列号机制 (Sequence Number Mechanism)**
    *   每个发送方（由`System ID` + `Component ID`唯一标识）维护一个独立的、从0开始递增的序列号计数器。
    *   每发送一个帧，该计数器的当前值被填入`序列号`字段，然后计数器递增（模256）。
    *   **接收方作用：** 接收方为每个已知的发送方（`System ID` + `Component ID`）维护一个“期望的下一个序列号”状态。
        *   如果收到的序列号等于期望值，处理帧，期望值加1。
        *   如果收到的序列号大于期望值，说明有**丢包**（跳过了中间的序列号）。
        *   如果收到的序列号小于期望值，说明是**乱序**到达的旧包或重复包（通常丢弃或特殊处理）。
    *   **关键作用：** 在不可靠的物理链路（如串口、无线链路）上，提供了一种轻量级的、基于数据链路层的**丢包检测**能力。**注意：** MAVLink本身不提供自动重传，丢包处理通常由应用层逻辑决定。

5.  **通信协议层 (Transport Layer)**
    MAVLink定义了帧结构、寻址、校验和序列号机制。它**不指定**底层的物理传输方式。因此，MAVLink消息可以通过多种物理链路传输：
    *   **串口 (UART/USB):** 最常见的方式，用于无人机机载组件之间（飞控->数传电台）以及数传电台与地面站（USB串口）或USB直连的连接。简单、可靠、低延迟。
    *   **UDP (User Datagram Protocol):** 常用于地面站软件与模拟器之间、或者通过IP网络（WiFi, 以太网, 蜂窝网络）连接的地面站与无人机之间的通信。支持广播，效率高，但不可靠（可能丢包、乱序）。
    *   **TCP (Transmission Control Protocol):** 较少用于核心飞行控制，因为其可靠性和拥塞控制机制可能引入不可预测的延迟。有时用于文件传输（如MavFTP）或需要可靠流的辅助通道。
    *   **其他：** 理论上，任何能传输字节流的物理媒介都可以承载MAVLink（如蓝牙、共享内存、自定义无线链路）。关键在于实现对应传输方式的适配器。

6.  **编组与解组 (Marshalling & Unmarshalling)**
    *   **发送端 (Marshalling):** 应用程序（如飞控软件）创建特定消息类型的数据结构（如填充`ATTITUDE`消息的roll, pitch, yaw字段）。MAVLink库将此数据结构按照协议规范**打包**成二进制的帧（包括添加起始位、长度、序列号、系统/组件ID、消息ID、计算校验码等）。
    *   **传输：** 打包好的二进制帧通过选定的物理链路（串口、UDP socket等）发送出去。
    *   **接收端 (Unmarshalling):** 接收方（如地面站）从物理链路读取字节流。MAVLink库负责：
        1.  **帧同步：** 寻找起始位 (`0xFE` 或 `0xFD`)。
        2.  **提取长度：** 读取载荷长度，确定帧总长。
        3.  **接收完整帧：** 读取指定长度的后续字节。
        4.  **校验：** 计算接收帧的CRC校验码，与帧尾的校验码比较。**失败则丢弃。**
        5.  **解析：** 如果校验通过，根据`消息ID`查找对应的消息定义。
        6.  **解包：** 将`Payload`部分的二进制数据按照消息定义**解包**成对应编程语言的数据结构（如C结构体、Python对象）。
        7.  **序列号检查：** 检查序列号连续性（可选但推荐，用于统计丢包）。
        8.  **交付应用：** 将解包后的消息数据传递给应用程序处理（如在地面站地图上显示位置）。

**总结 MAVLink 通信流程：**

1.  **发送方应用层：** 产生需要发送的数据（如当前姿态）。
2.  **发送方 MAVLink 库 (Marshalling):**
    *   选择对应消息类型（如`ATTITUDE`）。
    *   填充消息字段。
    *   构建帧：添加起始位、长度、序列号（递增）、系统ID、组件ID、消息ID。
    *   计算整个帧（起始位到载荷）的CRC校验码。
    *   将完整的二进制帧推送给传输层（如串口驱动、UDP socket）。
3.  **物理传输：** 帧通过串口线缆、无线电波、IP网络等传输，可能受到干扰或丢失。
4.  **接收方传输层：** 接收到原始字节流。
5.  **接收方 MAVLink 库 (Unmarshalling):**
    *   在字节流中搜索有效的起始位。
    *   根据长度读取完整帧。
    *   **计算接收帧的CRC，与帧尾CRC比较。不匹配则丢弃！**
    *   校验通过后，提取系统ID、组件ID、消息ID、序列号。
    *   根据消息ID找到对应定义。
    *   将Payload二进制数据解包成结构化数据。
    *   （可选）检查序列号连续性。
    *   将解析好的结构化消息（包含系统ID、组件ID、数据）传递给接收方应用程序。
6.  **接收方应用层：** 处理消息（如更新UI显示姿态、执行指令、记录日志）。

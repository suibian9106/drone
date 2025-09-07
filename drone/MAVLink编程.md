## MAVLink编程

### 一、XML 文件的核心作用
1. **定义消息结构**  
   描述每条消息（Message）的字段、数据类型、ID 及用途，例如：
   - `HEARTBEAT` 消息（ID=0）包含系统类型、状态等字段。
   - 自定义消息（如 `CA_TRAJECTORY`）可扩展飞控特有数据（如轨迹系数）。

2. **管理枚举类型**  
   定义状态码、错误类型等枚举值（如 `MAV_BATTERY_TYPE`），用于消息字段的语义化描述。

3. **支持协议方言（Dialect）**  
   不同设备（如 PX4 飞控、ArduPilot）可基于公共协议（`common.xml`）扩展私有消息，形成独立方言：
   - PX4 的 `development.xml` 包含特有消息（如 `AIRSPEED`）。
   - Paparazzi 的 `paparazzi.xml` 定义脚本控制消息。

4. **协议版本控制**  
   通过 `<version>` 标签指定协议版本（如 MAVLink v2.0），启用新特性：
   - 24 位消息 ID（支持 1600 万条消息）。
   - 报文签名（防篡改）和载荷截断（优化带宽）。

### 二、XML 文件的关键结构
XML 文件采用层级标签定义，主要结构如下:
```xml
<?xml version="1.0"?>
<mavlink>
    <!-- 1. 包含基础协议 -->
    <include>common.xml</include>
    
    <!-- 2. MAVLink协议版本 (可选) -->
    <version>3</version>
    
    <!-- 3. 方言标识 (可选) -->
    <dialect>8</dialect>
    
    <!-- 4. 枚举定义 -->
    <enums>
        <enum name="CTRL_CMD_TYPE">
            <description>控制命令类型</description>
            <entry value="0" name="CMD_TAKEOFF">
                <description>起飞命令</description>
            </entry>
            <entry value="1" name="CMD_LAND">
                <description>降落命令</description>
            </entry>
            <entry value="2" name="CMD_SET_ALT">
                <description>设置高度</description>
            </entry>
        </enum>
    </enums>
    
    <!-- 5. 消息定义 -->
    <messages>
        <!-- 温度传感器消息 -->
        <message id="180" name="SENSOR_TEMP">
            <description>温度传感器数据</description>
            <field type="float" name="temperature" units="degC">
                <description>当前温度值</description>
            </field>
            <field type="uint64_t" name="timestamp" units="ms">
                <description>时间戳(毫秒)</description>
            </field>
        </message>
        
        <!-- 控制命令消息 -->
        <message id="181" name="CTRL_CMD">
            <description>飞控控制命令</description>
            <field type="uint8_t" name="command" enum="CTRL_CMD_TYPE">
                <description>命令类型</description>
            </field>
            <field type="float" name="param1">
                <description>参数1</description>
            </field>
            <field type="float" name="param2">
                <description>参数2</description>
            </field>
        </message>

    </messages>
</mavlink>
```

#### 关键标签说明：
| **标签**         | **作用**                                                                 | **示例**                                                     |
|------------------|--------------------------------------------------------------------------|-------------------------------------------------------------|
| `<include>`      | 引用基础协议（如 `common.xml`），复用标准消息                             | `<include>common.xml</include>`    |
| `<version>`      | 指定协议次要版本号，影响兼容性                                            | `<version>6</version>`（v2.0 对应版本 3） |
| `<dialect>`      | 方言唯一标识符，避免不同方言冲突                                          | `<dialect>8</dialect>`（PX4 方言）              |
| `<enum>`         | 定义枚举类型，用于消息字段的语义化                                        | 电池类型 `MAV_BATTERY_TYPE`                    |
| `<message>`      | 定义消息结构，含 `id`（唯一标识）、`name` 和字段列表                      | `id="180"`（自定义消息建议 180-229）           |
| `<field>`        | 消息字段，指定数据类型（如 `float`、`uint8_t[10]`）和名称                 | `<field type="double" name="pressure"/>`       |


使用xml文件生产对应的库文件
```bash
# Dependencies
sudo apt install python3-pip

# Clone mavlink into the directory of your choice
git clone https://github.com/mavlink/mavlink.git --recursive
cd mavlink

python3 -m pip install -r pymavlink/requirements.txt
```

使用pymavlink工具生成头文件目录

```bash
python3 -m pymavlink.tools.mavgen --lang=C --wire-protocol=2.0 --output=generated/include/mavlink/v2.0 message_definitions/v1.0/common.xml
```

在项目中包含头文件即可使用定义的消息类型相关函数

```c
#include <generated/include/mavlink/v2.0/your_dialect/mavlink.h>

// mavlink_msg_heartbeat_pack()，打包为mavlink_message_t类型
void send_some(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    // Whenever a second has passed, we send a heartbeat.
    static time_t last_time = 0;
    time_t current_time = time(NULL);
    if (current_time - last_time >= 1) {
        send_heartbeat(socket_fd, src_addr, src_addr_len);
        last_time = current_time;
    }
}

void send_heartbeat(int socket_fd, const struct sockaddr_in* src_addr, socklen_t src_addr_len)
{
    mavlink_message_t message;

    const uint8_t system_id = 42;
    const uint8_t base_mode = 0;
    const uint8_t custom_mode = 0;
    mavlink_msg_heartbeat_pack_chan(
        system_id,
        MAV_COMP_ID_PERIPHERAL,
        MAVLINK_COMM_0,
        &message,
        MAV_TYPE_GENERIC,
        MAV_AUTOPILOT_GENERIC,
        base_mode,
        custom_mode,
        MAV_STATE_STANDBY);

    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    const int len = mavlink_msg_to_send_buffer(buffer, &message);

    int ret = sendto(socket_fd, buffer, len, 0, (const struct sockaddr*)src_addr, src_addr_len);
    if (ret != len) {
        printf("sendto error: %s\n", strerror(errno));
    } else {
        printf("Sent heartbeat\n");
    }
}

// mavlink_msg_heartbeat_decode()，将mavlink_message_t类型提取出消息对应的mavlink_heartbeat_t结构类型
void receive_some(int socket_fd, struct sockaddr_in* src_addr, socklen_t* src_addr_len, bool* src_addr_set)
{
    // We just receive one UDP datagram and then return again.
    char buffer[2048]; // enough for MTU 1500 bytes

    const int ret = recvfrom(
            socket_fd, buffer, sizeof(buffer), 0, (struct sockaddr*)(src_addr), src_addr_len);

    if (ret < 0) {
        printf("recvfrom error: %s\n", strerror(errno));
    } else if (ret == 0) {
        // peer has done an orderly shutdown
        return;
    }

    *src_addr_set = true;

    mavlink_message_t message;
    mavlink_status_t status;
    for (int i = 0; i < ret; ++i) {
        if (mavlink_parse_char(MAVLINK_COMM_0, buffer[i], &message, &status) == 1) {

            // printf(
            //     "Received message %d from %d/%d\n",
            //     message.msgid, message.sysid, message.compid);

            switch (message.msgid) {
            case MAVLINK_MSG_ID_HEARTBEAT:
                handle_heartbeat(&message);
                break;
            }
        }
    }
}

void handle_heartbeat(const mavlink_message_t* message)
{
    mavlink_heartbeat_t heartbeat;
    mavlink_msg_heartbeat_decode(message, &heartbeat);

    printf("Got heartbeat from ");
    switch (heartbeat.autopilot) {
        case MAV_AUTOPILOT_GENERIC:
            printf("generic");
            break;
        case MAV_AUTOPILOT_ARDUPILOTMEGA:
            printf("ArduPilot");
            break;
        case MAV_AUTOPILOT_PX4:
            printf("PX4");
            break;
        default:
            printf("other");
            break;
    }
    printf(" autopilot\n");
}

// mavlink_message_t mavlink_types.h文件中
MAVPACKED(
typedef struct __mavlink_message {
	uint16_t checksum;      ///< sent at end of packet
	uint8_t magic;          ///< protocol magic marker
	uint8_t len;            ///< Length of payload
	uint8_t incompat_flags; ///< flags that must be understood
	uint8_t compat_flags;   ///< flags that can be ignored if not understood
	uint8_t seq;            ///< Sequence of packet
	uint8_t sysid;          ///< ID of message sender system/aircraft
	uint8_t compid;         ///< ID of the message sender component
	uint32_t msgid:24;      ///< ID of message in payload
	uint64_t payload64[(MAVLINK_MAX_PAYLOAD_LEN+MAVLINK_NUM_CHECKSUM_BYTES+7)/8];
	uint8_t ck[2];          ///< incoming checksum bytes
	uint8_t signature[MAVLINK_SIGNATURE_BLOCK_LEN];
}) mavlink_message_t;
```
# 校内赛小车 ESP32 Wi-Fi 通信

## 1. 模块用途与边界

本工程运行在 ESP32-S3，负责把本车机器人状态和异步比赛事件通过 Wi-Fi/UDP 上报到比赛服务器。

目前规划的网络路径：

```text
L431 / 车载业务逻辑 -> ESP32-S3 -> 2.4 GHz AP -> 比赛服务器
```

当前版本已实现小车到服务器的上报；服务器到小车的 UDP 通知接收链路仅作预留，收到后暂无具体业务。服务器收到上行数据后如何展示、裁判逻辑如何判定、是否向小车发送通知，不属于本工程。

当前服务器配置：`10.123.59.216:5005`（这是我的手机热点WIFI，到时候肯定需要改）；ESP32 使用 STA 模式连接 Wi-Fi，关闭 modem sleep，保证实时性优先。

## 2. 已实现功能

1. ESP32-S3 自动连接指定 Wi-Fi；掉线后自动重连。
2. 每 100 ms（10 Hz）向服务器发送一次机器人状态 UDP 帧。
3. 状态帧包含机器人 ID、队伍、血量、存活/死亡状态、是否允许射击。
4. 支持异步上报死亡、复活、受击、攻击、恢复射击、禁止射击；每种业务都有独立 UDP 帧。
5. 事件通过 FreeRTOS 队列交给网络发送任务，避免业务任务直接操作 socket。

## 3. UDP 协议

服务器地址、端口由 `sdkconfig.defaults` / `menuconfig` 中的 `Robot Wi-Fi` 配置项确定。

所有字段为 **小端序**；帧使用 `__attribute__((packed))`，不可按编译器默认对齐解析。

### 3.1 状态帧：10 Hz，10 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;         // 固定 0x5254
    uint8_t  version;       // 当前 1
    uint8_t  frame_type;    // 1 = 状态帧
    uint8_t  robot_id;      // 本车编号
    uint8_t  team;          // 本车队伍编号
    uint16_t hp;            // 当前血量
    uint8_t  alive;         // 0 = 死亡，1 = 存活
    uint8_t  shoot_enabled; // 0 = 禁止射击，1 = 允许射击
} robot_status_frame_t;
```

服务器以最后一次状态帧为准；连续超过预期时间未收到状态帧时，应由服务器判定该车离线。

### 3.2 异步事件帧：每种业务独立定义

事件发生时发送一次。每个业务是一种单独的 C 结构体和固定帧长；服务器先检查 `magic`、`version`，再根据 `frame_type` 按对应结构体解析即可。

#### 死亡帧：发生死亡时发送，6 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;      // 固定 0x5254
    uint8_t  version;    // 当前 1
    uint8_t  frame_type; // 固定 2：死亡
    uint8_t  robot_id;   // 死亡的小车编号
    uint8_t  team;       // 死亡小车的队伍编号
} robot_death_frame_t;
```

#### 复活帧：发生复活时发送，6 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;      // 固定 0x5254
    uint8_t  version;    // 当前 1
    uint8_t  frame_type; // 固定 3：复活
    uint8_t  robot_id;   // 复活的小车编号
    uint8_t  team;       // 复活小车的队伍编号
} robot_revive_frame_t;
```

#### 受击帧：本车受击且血量已更新时发送，8 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;      // 固定 0x5254
    uint8_t  version;    // 当前 1
    uint8_t  frame_type; // 固定 4：受击
    uint8_t  robot_id;   // 受击的小车编号
    uint8_t  team;       // 受击小车的队伍编号
    uint16_t hp;         // 受击后的本车血量，小端序
} robot_hit_frame_t;
```

#### 攻击帧：本车进入攻击/战斗状态时发送，6 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;      // 固定 0x5254
    uint8_t  version;    // 当前 1
    uint8_t  frame_type; // 固定 5：攻击
    uint8_t  robot_id;   // 发起攻击的小车编号
    uint8_t  team;       // 发起攻击小车的队伍编号
} robot_attack_frame_t;
```

攻击帧表示“进入攻击/战斗状态”，不是每发子弹发送一次。

#### 恢复射击帧：本车恢复射击权限时发送，6 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;      // 固定 0x5254
    uint8_t  version;    // 当前 1
    uint8_t  frame_type; // 固定 6：允许射击
    uint8_t  robot_id;   // 恢复射击的小车编号
    uint8_t  team;       // 该小车的队伍编号
} robot_shoot_enabled_frame_t;
```

#### 禁止射击帧：本车被禁止射击时发送，6 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;      // 固定 0x5254
    uint8_t  version;    // 当前 1
    uint8_t  frame_type; // 固定 7：禁止射击
    uint8_t  robot_id;   // 被禁止射击的小车编号
    uint8_t  team;       // 该小车的队伍编号
} robot_shoot_disabled_frame_t;
```

独立帧的意义：每个 `frame_type` 的业务含义、字段和长度唯一，不用再在一个通用事件包里猜 `event_type`、`value` 分别代表什么。持续状态仍以 10 Hz 状态帧为准。

### 3.3 服务器下行：仅保留 UDP 接收入口

ESP32 监听 UDP `5006`；服务器未来可单播到小车 IP，也可发送 AP 网段广播。当前未定义任何下行报文格式、目标字段或业务语义。收到任意 UDP 数据后，ESP32 仅交给空弱回调 `robot_network_on_server_datagram(data, length)`，不会执行任何小车动作。

后续真正增加禁射、复活等下行功能时，必须为每个业务分别定义自己的帧和处理函数；不要在当前工程中添加通用通知帧。

## 4. 业务层调用方式

协议定义和接口在 `main/robot_protocol.h`。

L431 数据接入完成后，由接收/业务任务在普通 FreeRTOS 任务上下文中调用：

```c
/* 每次本车状态更新时调用；网络任务会以 10 Hz 发送最新快照。 */
robot_network_set_status(hp, alive, shoot_enabled);

/* 发生一次性事件时调用。返回 false 表示 16 深度事件队列已满。 */
robot_network_publish_death();
robot_network_publish_revive();
robot_network_publish_hit(hp_after_hit);
robot_network_publish_attack();
robot_network_publish_shoot_enabled();
robot_network_publish_shoot_disabled();
```

当前初始状态为：HP=200、存活、允许射击。当前本车 ID 和队伍均为 `1`，在 `main/main.c` 顶部修改：

```c
#define LOCAL_ROBOT_ID 1U
#define LOCAL_ROBOT_TEAM 1U
```

每辆车必须使用不同 `LOCAL_ROBOT_ID`；队伍编号必须与服务器约定一致。

## 5. 编译、烧录与配置

使用 ESP-IDF v6.0.2，目标为 `esp32s3`。当前工程已关闭 PSRAM，以兼容板卡实际 PSRAM 型号不确定的情况。

需要修改网络时，改 `sdkconfig.defaults` 中：

```text
CONFIG_ROBOT_WIFI_SSID="..."
CONFIG_ROBOT_WIFI_PASSWORD="..."
CONFIG_ROBOT_SERVER_IP="..."
CONFIG_ROBOT_SERVER_PORT=5005
CONFIG_ROBOT_LISTEN_PORT=5006
CONFIG_ROBOT_REFEREE_UART_BAUD=115200
CONFIG_ROBOT_RESERVED_UART_BAUD=115200
```

配置变更后需要重新配置并编译；仅修改 `main/main.c` 或 `main/robot_protocol.h` 时应为增量编译。

固件产物：

```text
build/esp32_wifi_quality.bin
```

VS Code ESP-IDF 扩展中使用 **UART** 烧录，端口当前配置为 `COM24`。不要选择 JTAG/OpenOCD；该开发板使用 USB-UART 下载。

应用日志已切到 ESP32-S3 原生 USB Serial/JTAG；UART0 不输出应用日志，避免污染裁判通信。烧录仍使用 USB-UART。

### 5.1 串口预留与接线

UART0 用于裁判系统，已初始化为 `115200, 8N1, 无硬件流控`；波特率可通过 `CONFIG_ROBOT_REFEREE_UART_BAUD` 修改。UART1 同样已初始化为 `115200, 8N1, 无硬件流控`，但只作备用，当前没有业务。

| ESP32 串口 | ESP32 GPIO / 板上排针 | 接到外部设备时的连接方式 | 当前用途 |
|---|---|---|---|
| UART0 | GPIO43 / `TX` | ESP32 `TX` -> 裁判系统 `RX` | 裁判系统通信 |
| UART0 | GPIO44 / `RX` | ESP32 `RX` <- 裁判系统 `TX` | 裁判系统通信 |
| UART1 | GPIO17 | ESP32 `TX` -> 备用设备 `RX` | 预留 |
| UART1 | GPIO18 | ESP32 `RX` <- 备用设备 `TX` | 预留 |

UART0、UART1 当前均运行最小 `ping/pong` 接收任务，用于验证串口接线和收发方向：发送 ASCII `ping` 后，ESP32 会从**同一串口**回复：

| 测试串口 | 回复内容 |
|---|---|
| UART0 | `u0:pong\r\n` |
| UART1 | `u1:pong\r\n` |

`ping` 后带不带回车均可；每次连续匹配到 `ping` 都会回复一次。

这只是接线自检，不是裁判协议。裁判系统协议接入时，需要移除/替换 UART0 的 `ping/pong` 任务并新增独立解析任务；不要把裁判数据直接混入 Wi-Fi UDP 协议。

## 6. 当前未实现项

1. **尚未接入裁判系统 UART 协议。** UART0（GPIO43/44）已初始化并运行 `ping/pong` 接线自检，UART1（GPIO17/18）同样运行该自检；当前状态值仍为 ESP32 内部初始值。下一位开发者需要以裁判协议任务替换 UART0 自检任务，并在收到数据后调用 `robot_network_set_status()` 及对应的独立事件 API。
2. **没有 UDP ACK、重传或持久化。** 当前事件是单次 UDP 上报，事件队列满或 Wi-Fi 断开时可能丢失。若比赛规则要求死亡/复活等事件必须可靠送达，需要按具体事件分别补 ACK、超时重传和去重逻辑。
3. **没有服务器端程序。** 服务器需要按本 README 的小端二进制结构解析 UDP `5005` 端口。
4. **下行控制尚未实现。** ESP32 已预留 UDP `5006` 的原始接收入口，但没有定义任何下行帧，没有接入 L431 或业务执行。
5. **下行单播/广播仅预留，服务器端尚未实现。** 服务器未来可单播到单车 IP，也可向 AP 网段广播地址的 UDP `5006` 发送数据；ESP32 已能收到并交给空回调，但当前不会解析、过滤、转发或执行。

## 7. 文件职责

| 文件 | 职责 |
|---|---|
| `main/main.c` | UART0/1 初始化和 `ping/pong` 自检、Wi-Fi 连接、状态快照、事件队列、UDP 上行发送与下行原始数据接收任务 |
| `main/robot_protocol.h` | 正式上行协议结构、独立事件帧与供 UART/业务层调用的 API |
| `main/Kconfig.projbuild` | Wi-Fi、服务器地址/端口、下行监听端口和 UART 波特率配置 |
| `sdkconfig.defaults` | 当前默认网络配置、关闭 PSRAM |
| `host/wifi_monitor.py` | 旧 Wi-Fi 测试工具；正式比赛不运行 |

## 8. 上车前检查清单

- 每辆车的 `LOCAL_ROBOT_ID` 是否唯一；
- `LOCAL_ROBOT_TEAM` 是否正确；
- 服务器 IP、UDP 端口是否与服务器一致；
- 服务器是否按小端序、10 字节状态帧及各独立事件帧解析；
- 如启用下行功能，是否先为该具体业务定义独立帧，再向 UDP `5006` 发送；
- L431 接入后是否在状态变化时更新 `hp/alive/shoot_enabled`；
- 死亡、复活、受击、攻击、恢复/禁止射击是否都调用了事件 API；
- Wi-Fi AP 是否固定 2.4 GHz 信道，且所有小车能稳定连接。

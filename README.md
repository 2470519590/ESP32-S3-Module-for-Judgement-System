# 校内赛小车 ESP32 Wi-Fi 通信模块

## 1. 模块用途与边界

本工程运行在 ESP32-S3，负责把本车机器人状态和异步比赛事件通过 Wi-Fi/UDP 上报到比赛服务器。

网络路径：

```text
L431 / 车载业务逻辑 -> ESP32-S3 -> 2.4 GHz AP -> 比赛服务器
```

当前版本只做 **小车到服务器的上报**。服务器收到后如何展示、裁判逻辑如何判定、是否转发给其他客户端，不属于本工程。

当前服务器配置：`10.123.59.216:5005`（这是我的手机热点WIFI，到时候肯定需要改）；ESP32 使用 STA 模式连接 Wi-Fi，关闭 modem sleep，保证实时性优先。

## 2. 已实现功能

1. ESP32-S3 自动连接指定 Wi-Fi；掉线后自动重连。
2. 每 100 ms（10 Hz）向服务器发送一次机器人状态 UDP 帧。
3. 状态帧包含机器人 ID、队伍、血量、存活/死亡状态、是否允许射击。
4. 支持异步上报死亡、复活、受击、攻击、恢复射击、禁止射击事件。
5. 事件通过 FreeRTOS 队列交给网络发送任务，避免业务任务直接操作 socket。
6. 事件带递增 `event_id`，服务器可据此发现重复或缺失。

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

### 3.2 事件帧：事件发生时发送一次，12 字节

```c
typedef struct __attribute__((packed)) {
    uint16_t magic;            // 固定 0x5254
    uint8_t  version;          // 当前 1
    uint8_t  frame_type;       // 2 = 事件帧
    uint8_t  robot_id;         // 事件发起小车
    uint8_t  team;             // 事件发起小车队伍
    uint8_t  event_type;       // 见下表
    uint8_t  subject_robot_id; // 0 = 本车；受击时可填被击中车号
    uint16_t value;            // 受击后血量；其他事件填 0
    uint16_t event_id;         // 本车事件递增号，启动后从 0 开始
} robot_event_frame_t;
```

| `event_type` | 含义 | `subject_robot_id` | `value` |
|---:|---|---:|---:|
| 1 | 死亡 | 0 | 0 |
| 2 | 复活 | 0 | 0 |
| 3 | 受击 | 被击中机器人 ID；未知填 0 | 本车受击后的 HP |
| 4 | 攻击 | 目标机器人 ID；未知填 0 | 0 |
| 5 | 恢复射击 | 0 | 0 |
| 6 | 禁止射击 | 0 | 0 |

`攻击` 表示一次攻击/进入攻击状态，不应按每一颗弹丸重复发送。需要持续状态时，以 10 Hz 状态帧为准。

## 4. 业务层调用方式

协议定义和接口在 `main/robot_protocol.h`。

L431 数据接入完成后，由接收/业务任务在普通 FreeRTOS 任务上下文中调用：

```c
/* 每次本车状态更新时调用；网络任务会以 10 Hz 发送最新快照。 */
robot_network_set_status(hp, alive, shoot_enabled);

/* 发生一次性事件时调用。返回 false 表示 16 深度事件队列已满。 */
robot_network_publish_event(ROBOT_EVENT_DEATH, 0, 0);
robot_network_publish_event(ROBOT_EVENT_REVIVE, 0, 0);
robot_network_publish_event(ROBOT_EVENT_HIT, target_id, hp_after_hit);
robot_network_publish_event(ROBOT_EVENT_ATTACK, target_id, 0);
robot_network_publish_event(ROBOT_EVENT_SHOOT_ENABLED, 0, 0);
robot_network_publish_event(ROBOT_EVENT_SHOOT_DISABLED, 0, 0);
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
```

配置变更后需要重新配置并编译；仅修改 `main/main.c` 或 `main/robot_protocol.h` 时应为增量编译。

固件产物：

```text
build/esp32_wifi_quality.bin
```

VS Code ESP-IDF 扩展中使用 **UART** 烧录，端口当前配置为 `COM24`。不要选择 JTAG/OpenOCD；该开发板使用 USB-UART 下载。

烧录后串口只应看到 Wi-Fi 连接/断线等必要系统日志，不会再输出 Ping、RSSI、UDP 测试统计。

## 6. 当前未实现项（交接重点）

1. **尚未接入 L431 UART。** 当前状态值为 ESP32 内部初始值。下一位开发者需要实现 L431 -> ESP32 的串口协议与接收任务，并在收到数据后调用 `robot_network_set_status()` 和 `robot_network_publish_event()`。
2. **没有 UDP ACK、重传或持久化。** 当前事件是单次 UDP 上报，事件队列满或 Wi-Fi 断开时可能丢失。若比赛规则要求死亡/复活等事件必须可靠送达，需要补事件 ACK、超时重传和去重逻辑；服务器须按 `robot_id + event_id` 去重。
3. **没有服务器端程序。** 服务器需要按本 README 的小端二进制结构解析 UDP `5005` 端口。
4. **没有下行控制。** 服务器目前不能向小车下发禁射、复活等命令；若要实现，需要新增 ESP32 UDP 接收任务以及 L431 下行接口。
5. **没有广播 socket。** 当前是小车单播到服务器；“广播给其他客户端”应由服务器完成。如需 ESP32 直接 UDP 广播，需明确 AP 网段、端口和接收端规则后再实现。
6. **没有安全认证/加密。** 校内赛封闭网络可暂用；若网络不可信，需要至少加入消息认证或使用受控网络。
7. **未做整车电机干扰复测。** 之前 Wi-Fi 测试代码和 `host/wifi_monitor.py` 仍保留在仓库中，仅供诊断，不属于正式通信链路。

## 7. 文件职责

| 文件 | 职责 |
|---|---|
| `main/main.c` | Wi-Fi 连接、状态快照、事件队列、UDP 发送任务 |
| `main/robot_protocol.h` | 正式协议结构、事件枚举、供 UART/业务层调用的 API |
| `main/Kconfig.projbuild` | Wi-Fi 和服务器地址/端口配置 |
| `sdkconfig.defaults` | 当前默认网络配置、关闭 PSRAM |
| `host/wifi_monitor.py` | 旧 Wi-Fi 测试工具；正式比赛不运行 |

## 8. 上车前检查清单

- 每辆车的 `LOCAL_ROBOT_ID` 是否唯一；
- `LOCAL_ROBOT_TEAM` 是否正确；
- 服务器 IP、UDP 端口是否与服务器一致；
- 服务器是否按小端序、10 字节状态帧和 12 字节事件帧解析；
- L431 接入后是否在状态变化时更新 `hp/alive/shoot_enabled`；
- 死亡、复活、受击、攻击、恢复/禁止射击是否都调用了事件 API；
- 服务器是否对事件按 `robot_id + event_id` 去重；
- Wi-Fi AP 是否固定 2.4 GHz 信道，且所有小车能稳定连接。

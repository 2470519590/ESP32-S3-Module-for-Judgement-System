# ESP32-S3 小车无线通信模块交接说明

本文档的协议部分面向比赛服务器开发者。服务器只需要按照 UDP 帧、机器人字段和事件语义进行实现，不需要了解设备内部的数据来源或车内硬件结构；文末另列出固件联调信息和未完成事项。

## 1. 当前固件负责什么

```text
Xbox 手柄 --BLE--> ESP32-S3 --UART0--> 底盘主控
                         |
                         +--UART1 <--> L431PM USART2
                         |
                         +--Wi-Fi UDP--> 比赛服务器
```

- ESP32 解析 Xbox 手柄 HID，并通过 UART0 以约 100 Hz 输出 14 字节底盘控制帧。
- ESP32 通过 UART1 接收上游提供的真实 HP、存活、射击许可、热量、功率、供电状态和事件，然后转换为 V2 UDP 上行帧。
- ESP32 接收服务器下行命令，并将比赛开始、比赛结束、设置 HP、黄牌处罚、强制断电和请求通电转发给 L431PM；执行结果再通过 UDP ACK 返回服务器。
- ESP32 不重新计算或维护 HP、死活、比赛状态、热量、功率和射击许可，只转发上游接口提供的当前值。
- 装甲板只向 L431PM 上报受击，ESP32 不直接接收装甲板 CAN。

当前 UART 引脚：

| 接口 | ESP32 引脚 | 对端 | 用途 |
|---|---|---|---|
| UART0 | GPIO43 TX、GPIO44 RX | 底盘主控 RX/TX | 手柄控制帧；正式运行不输出文本 |
| UART1 | GPIO17 TX、GPIO18 RX | L431 PA3 RX、PA2 TX | L431PM 状态、事件和命令回执 |

两条 UART 均为 `115200 8N1`，交叉连接并共地。完整 UART0 控制帧见 `docs/手柄接收机通信协议.md`；L431PM 对接帧见 `docs/L431PM_对接任务提示词.md`。

## 2. V2 UDP 总规则

- ESP32 → 服务器：目标服务器 UDP `5005`。
- 服务器 → ESP32：目标 ESP32 UDP `5006`。
- 服务器程序应使用一个绑定 `5005` 的 UDP socket 同时接收上行和发送下行。这样 ESP32 的 ACK 会返回到服务器 `5005`。
- ESP32 只接受配置服务器 IP 发来的下行包；其他来源会被丢弃。
- 所有 V2 帧均为固定长度二进制、无额外转义；多字节整数为 little-endian。
- 公共头部为 `magic=0x5254`，在线路上为 `54 52`，`version=2`。
- 必须按 `version + frame_type + 精确帧长` 解码，不能按 V1 长度猜测。

公共头部不是独立帧，所有帧均直接包含：

```text
magic[2] | version[1] | frame_type[1] | payload...
```

## 3. ESP32 → 服务器上行帧

### 3.1 状态帧：14 字节，约 10 Hz

```text
offset  size  field
0       2     magic = 0x5254
2       1     version = 2
3       1     frame_type = 0x01
4       1     robot_id
5       2     hp (uint16, little-endian)
7       2     heat (uint16, little-endian)
9       2     power (uint16, little-endian, W)
11      1     alive (0/1)
12      1     shoot_enabled (0/1)
13      1     power_on (0/1，底盘供电输出状态)
```

状态帧是发送端提供的当前机器人状态快照。服务器应直接读取其中的 `robot_id、hp、heat、power、alive、shoot_enabled、power_on`。`shoot_enabled` 是比赛射击许可；`power_on=1` 表示小车底盘供电输出已打开，`0` 表示已关闭。这两个字段互不推导。当前状态帧只定义这些业务字段，不包含额外的网络诊断字段、时间戳或 CRC；服务器可使用本地 UDP 接收时间统计报文间隔和丢包情况。

### 3.2 设备健康帧：6 字节，约 10 Hz

```text
54 52 02 0C robot_id component_online
```

`component_online` 位图：bit0=L431PM，bit1=枪管，bit2~5=装甲板 NodeID 1~4。L431 每个 CAN 外设连续 1000 ms 未返回有效轮询回复即离线；四块装甲板尚未全部完成 NodeID 分配时，bit2~5 全部为 0。即使 CAN 回复仍在到达，装甲板仅在 `NORMAL` 或 `HIT` 状态时在线，其余状态不能正常上报受击。ESP32 连续 300 ms 未收到 L431 状态帧时清除全部位，因此服务器可区分“ESP32 仍在线、L431 已掉线”。该帧是状态快照，不需要 ACK。

### 3.3 独立事件帧

普通事件固定 5 字节：`54 52 02 type robot_id`。

| `type` | 事件 | 是否 ACK | 说明 |
|---:|---|---|---|
| `0x04` | 受击 | 否 | 只表示发生受击，不携带伤害和 HP；HP 以随后状态帧为准 |
| `0x05` | 攻击 | 否 | 发送端确认一次有效射击后产生 |
| `0x06` | 允许射击 | 否 | L431PM 比赛射击许可变为允许后产生；与 `power_on` 无关 |
| `0x07` | 禁止射击 | 否 | L431PM 比赛射击许可变为禁止后产生；与 `power_on` 无关 |
| `0x0B` | 脱离战斗 | 否 | 机器人连续 800 ms 没有新的有效射击后产生；与射击许可无关 |
| `0x08` | 上游业务链路断开 | 否 | 发送端未收到上游状态后产生 |
| `0x09` | 上游业务链路恢复 | 否 | 发送端重新收到有效状态后产生 |

死亡、复活为可靠事件，固定 9 字节：

```text
54 52 02 type robot_id transaction_id[4]
```

其中 `type=0x02` 为死亡，`type=0x03` 为复活；`transaction_id` 为 little-endian `uint32`。服务器收到后必须回一个 ACK，且同一 `(robot_id, type, transaction_id)` 只能执行业务一次，但重复收到时仍要回 ACK。

### 3.4 设备注册帧：25 字节，每 2 秒

```text
54 52 02 0A robot_id esp32_wifi_mac[6] controller_ble_mac[6] event_queue_drops[4] udp_send_failures[4]
```

`robot_id=0` 表示该 ESP32 尚未分配比赛身份。`esp32_wifi_mac` 是本板不可重复的 Wi-Fi STA MAC；`controller_ble_mac` 在尚未分配手柄时为全 `00`。两个计数均为 little-endian `uint32`，分别表示普通事件队列因满而丢弃的累计次数、以及本地 `sendto()` 失败累计次数。服务器应以 ESP32 Wi-Fi MAC 为设备主键，并将本帧 UDP 源 IP 记为该设备在当前 Wi-Fi 下的最新地址。

## 4. 服务器 → ESP32 下行帧

下行帧的目标地址是 ESP32 IP 的 UDP `5006`。所有比赛控制帧必须带非零 `target_robot_id`，且 ESP32 只处理与本车 ID 一致的帧；不支持比赛控制广播。

| `type` | 命令 | 长度 | 字节布局 | ACK |
|---:|---|---:|---|---|
| `0x81` | 比赛开始 | 9 B | `54 52 02 81 target_id tx[4]` | 是 |
| `0x82` | 比赛结束 | 9 B | `54 52 02 82 target_id tx[4]` | 是 |
| `0x83` | 分配机器人 ID 和手柄 MAC | 21 B | `54 52 02 83 robot_id target_esp32_mac[6] controller_mac[6] tx[4]` | 是 |
| `0x84` | 请求立即状态 | 5 B | `54 52 02 84 target_id` | 否 |
| `0x85` | 设置 HP | 11 B | `54 52 02 85 target_id hp[2] tx[4]` | 是 |
| `0x86` | 黄牌处罚 | 9 B | `54 52 02 86 target_id tx[4]` | 是 |
| `0x87` | 强制断电 | 9 B | `54 52 02 87 target_id tx[4]` | 是 |
| `0x88` | 请求通电 | 9 B | `54 52 02 88 target_id tx[4]` | 是 |

`0x83` 必须按 `DEVICE_ANNOUNCE` 的源 IP 单播；帧内 `target_esp32_mac` 必须等于目标板已上报的 Wi-Fi MAC，否则 ESP32 不执行也不回 ACK。`controller_mac` 为正常打印顺序的 6 个原始字节，例如 `8d:23:ab:a5:3c:c9` 对应 `8D 23 AB A5 3C C9`。

`SET_HP` 是直接设置 HP，不是“扣 20”命令；合法范围是 `0..300`。正常受击事件由设备业务自行产生，服务器不应通过 `SET_HP=当前 HP-20` 模拟一次受击。

`YELLOW_CARD` 表示服务器确认了一次恶意高速撞击处罚，只能针对一个明确的 `target_robot_id`，不能广播。服务器每判定一次就发送一次，设备内部累计本场黄牌：第 1、2 次分别扣 50 HP；第 3 次直接判负。该命令不携带累计次数，服务器必须为每次处罚使用新的事务号。黄牌计数、扣血、判负和是否允许复活均由设备业务端执行。

`FORCE_POWER_OFF` 和 `FORCE_POWER_ON` 只控制 L431PM 的底盘/弹道供电输出，不会关闭 L431PM 或 ESP32。两条命令均只能单播给明确的 `target_robot_id`，并且必须等待 ACK；通电是否允许由 L431PM 的电源保护条件决定。状态帧中的 `power_on` 是 L431PM 当前底盘供电输出状态。

## 5. ACK、事务号和重传

ACK 固定 10 字节：

```text
54 52 02 F0 acked_frame_type transaction_id[4] result
```

`result` 定义：`0=成功`，`1=不允许`，`2=失败`。

服务器必须实现：

1. `0x81/0x82/0x83/0x85/0x86/0x87/0x88` 使用非 0 的 `uint32 transaction_id`。
2. 等待 ACK；无 ACK 时用相同帧、相同事务号重发，不要生成新事务号。
3. 对 ESP32 上报的死亡/复活回 ACK，并按事务号去重。
4. 只有收到 `result=0` 才认为下行命令执行成功；`1/2` 应记录失败或进入人工处理。
5. `0x84`、状态帧、受击、攻击、射击许可和上游链路事件不需要 ACK；`0x86` 黄牌处罚、`0x87` 强制断电和 `0x88` 请求通电需要 ACK。

当前设备行为：关键下行命令收到重复事务时不会重复执行，而是返回已保存结果；死亡/复活待 ACK 记录保存在设备非易失存储器中，断电重启后仍会继续上报。比赛开始由设备本地设置 `HP=300、alive=1` 并清除本场黄牌和判负状态；只有比赛进行中且未判负时，HP 归零后的 5 秒复活才会执行。

当前协议没有定义服务器身份认证、时间戳、CRC 或会话层。设备发现固定使用 `0x0A DEVICE_ANNOUNCE`；服务器暂时不要自行添加会改变固定帧的字段。

## 6. 服务器应先实现的内容

服务器可以按下面顺序直接开始：

1. 绑定 `0.0.0.0:5005`，严格解析 25 字节设备注册帧、14 字节业务状态帧、6 字节设备健康帧、5/9 字节事件帧和 10 字节 ACK。
2. 用 ESP32 Wi-Fi MAC 建立设备表，保存当前源 IP、手柄 MAC 与最后在线时间；再以已分配的 `robot_id` 建立比赛状态表。
3. 对死亡/复活按事务号去重并回 ACK。
4. 实现下行帧构造、ACK 等待和相同事务号重传。
5. 开局时对每车发送 `GAME_START`；设备返回成功 ACK 后，服务器再认为该车已进入本场比赛。设备会自行设置 `HP=300`。
6. 将受击、进入战斗、脱离战斗、死亡、复活和射击许可作为独立事件处理，不从状态帧推断事件发生次数。

服务器应保存的最小数据：`robot_id`、最近状态、最近 UDP 源 IP、最后接收时间、事件去重键、下行事务状态。状态帧中的业务字段是发送端提供的当前快照，服务器不应要求 ESP32 重新维护一套 HP。

## 7. 当前测试 CLI

文件：`host/test_server_cli.py`。它是当前联调使用的服务器替身，不是最终比赛服务器。

启动：

```powershell
E:\miniconda\envs\py310\python.exe host\test_server_cli.py --bind 0.0.0.0 --port 5005 --robot-port 5006
```

CLI 界面上方显示实时状态，下方是可输入命令的输入框。首次配置先用 `devices` 查看 ESP32 注册表；之后比赛命令只输入机器人 ID，不输入 ESP32 IP：

```text
help
stats
devices
assign 1 8D:23:AB:A5:3C:C9
hp 1 300
yellow 1
power_on 1
power_off 1
status 1
start 1
end 1
quit
```

每台 ESP32 上电并连接 Wi-Fi 后，每 2 秒发送一帧 `DEVICE_ANNOUNCE`。其中含有不可重复的 ESP32 Wi-Fi MAC、当前机器人 ID（未分配为 `0`）、已保存的手柄 MAC、事件队列丢弃累计数和本地 UDP 发送失败累计数。CLI 用它维护 `robot_id ↔ ESP32 MAC ↔ 当前 IP ↔ 手柄 MAC` 注册表。因此全新设备的正常顺序是：

```text
仅上电目标 ESP32 -> devices 确认它已注册 -> assign ROBOT_ID CONTROLLER_MAC -> 再执行 hp/start/end
```

CLI 不要求输入 ESP32 MAC：它自动选择唯一在线的未分配设备，并将该设备注册帧中的 Wi-Fi MAC 同时写进 `assign` 帧作为防串台校验。若同时存在多台未分配 ESP32，CLI 会拒绝猜测；应只上电目标板后再执行 `assign`。配置成功后机器人 ID 与手柄 MAC 写入 NVS。若同一个机器人 ID 出现在多台在线 ESP32 上，CLI 会拒绝发送比赛命令，避免误控。

CLI 不保存本机设备登记文件，也不把电脑上的历史记录当作配置真源。CLI 重启后，等待已上电 ESP32 每 2 秒发送一次的 `DEVICE_ANNOUNCE` 自动重建当前在线表；该帧由 ESP32 从 NVS 读取自身保存的机器人 ID 与手柄 MAC。未重新上报的设备不会出现在表中，也不能接收下行命令。

未分配手柄 MAC 的新 ESP32 不会主动连接任何 Xbox 手柄；收到并保存 `assign` 后才扫描该指定地址。重新分配会断开旧手柄并开始扫描新地址。

### Windows 蓝牙扫描工具

联调前可用 `host/bluetooth_scan.py` 扫描附近的 BLE 设备，以便为 `assign` 选择手柄 MAC 地址。它优先列出游戏手柄，再按 RSSI 信号强度排序；仅扫描 BLE 广播设备，不会发现不发送 BLE 广播的纯经典蓝牙设备。

首次使用安装依赖：

```powershell
conda run -n py310 python -m pip install bleak
```

默认扫描 8 秒：

```powershell
conda run -n py310 python host\bluetooth_scan.py
```

也可指定时长，例如：

```powershell
conda run -n py310 python host\bluetooth_scan.py --seconds 10
```

脚本综合名称关键词（Xbox、Controller、Gamepad、DualSense、DualShock、8BitDo 等）、HID 服务 UUID 和可用的 Microsoft 厂商广播数据识别手柄；名称缺失时会显示 `<unknown>`。扫描结束时会输出手柄 MAC；将它作为 `assign ROBOT_ID CONTROLLER_MAC` 的最后一个参数。

CLI 当前会自动回复死亡/复活事件 ACK，显示 ESP32 返回的 UDP ACK，并打印发送帧十六进制。网络间隔按各机器人分别统计，不会把多车 10 Hz 数据混成一个虚假的高频流。ESP32 的 L431 命令等待在独立工作任务中完成，不会阻塞 UDP 接收任务。它不等同于正式服务器，尚未实现完整的服务器数据库、可靠命令自动重试策略和比赛业务状态机。

## 8. 多 ESP32 接入、测试配置和正式网络

### 8.1 多车支持

V2 UDP 协议和 ESP32 上行 socket 支持多台 ESP32 同时连接同一个服务器：每台 ESP32 有独立的 Wi-Fi IP 和 UDP 源端口；所有车辆可以共同向服务器的 `5005` 发送状态和事件。所有 ESP32 都监听 `5006` 不会冲突，因为端口属于各自 IP。

多车正常工作的必要条件：

- 每台车必须拥有唯一的 `robot_id`，不能多台都使用 `1`。
- 每台车连接同一个比赛 Wi-Fi，并把服务器地址配置为同一个服务器 IP。
- 服务器按 `robot_id` 保存最近状态、源 IP 和事务记录；不能用一个全局状态覆盖所有车辆。
- 下行命令按目标车的 IP 单播到 UDP `5006`；`target_robot_id=0` 不再被接受，避免任何比赛控制命令误广播到多车。
- 全新设备使用 `DEVICE_ANNOUNCE` 发现；首次分配必须按 ESP32 Wi-Fi MAC 精确寻址。

当前 CLI 注册表显示每台 ESP32 的设备 MAC、当前 IP、机器人 ID 和已保存手柄 MAC。新烧录的多台设备均为未分配 ID，不会错误地共同上报为 R1。

### 8.2 当前联调配置

当前固件配置为“真实设备数据 + 测试 Wi-Fi”，不是 ESP32 假比赛数据：

```text
Wi-Fi SSID: evil rats crazily squeak
Wi-Fi 密码: 66666666
UDP 服务器目标: 当前源码 TEST_SERVER_IP:5005
ESP32 下行监听: 5006
```

当前源码中的 `TEST_SERVER_IP` 需要以 `main/main.c` 为准；本次文档核对时仍为 `10.25.81.216`。如果测试电脑当前地址是 `10.25.81.187`，烧录前必须同步修改并重新构建，否则 ESP32 会继续向旧地址发送，也只接受旧地址的下行包。`CONFIG_ROBOT_TEST_MODE=n` 时，状态和事件仍必须来自真实 UART1/L431PM。

### 8.3 正式比赛网络配置

交接和服务器开发按下面的正式网络配置：

```text
Wi-Fi SSID: RM_GAME
Wi-Fi 密码: 12345678
服务器 IP: 192.168.1.3
ESP32 -> 服务器 UDP: 5005
服务器 -> ESP32 UDP: 5006
```

注意：当前源码仍有编译期开关 `USE_INTEGRATION_TEST_NETWORK`；发布正式固件前必须将其关闭并重新构建，否则仅修改 Kconfig 不会切换到下面的正式网络。服务器应监听 `192.168.1.3:5005`。

正式模式下 `CONFIG_ROBOT_TEST_MODE=n`，不会生成假比赛数据；状态和事件必须来自设备的真实业务输入。

## 8.3 当前固件已具备

当前已具备：

- Xbox BLE 指定地址连接、16 字节 HID 接收和 UART0 14 字节控制帧输出。
- 上游串口状态/事件/ACK 解析；死亡/复活仅在写入 NVS 待确认队列后回复 UART ACK，L431 会对未确认事件重传。
- V2 UDP 状态、事件、下行命令和 ACK。
- 机器人 ID、手柄 MAC、未确认死亡/复活事务、已执行下行事务结果的 NVS 保存。
- 上游业务链路断开/恢复事件。

## 9. 暂未实现或需要服务器确认的事项

- 正式网络发布构建：当前源码的 `USE_INTEGRATION_TEST_NETWORK` 仍需关闭后重新构建，Kconfig 中的正式网络参数才会生效。
- 正式服务器仍需实现自己的持久设备表、可靠命令自动重试、事务持久化、权限和比赛状态机。ESP32 的 `DEVICE_ANNOUNCE` 是机器人 ID 和手柄地址配置的真源；服务器重启后应等待各 ESP32 重新上报以恢复当前在线表。
- 设备发现和首次 ID/手柄地址分配已实现：ESP32 用 `DEVICE_ANNOUNCE` 主动上报，CLI 按唯一在线未分配设备自动选择目标，不需要人工输入 ESP32 IP 或 MAC。正式服务器应采用相同流程并保存设备表。
- 当前 V2 状态帧不包含额外网络诊断字段；若正式比赛需要新增诊断信息，应新增版本或独立诊断帧，不能修改当前 V2 状态帧长度。
- ESP32 不向服务器上报手柄原始 HID，也不向服务器提供电机控制量；底盘控制留在 UART0 和主控。

更细的结构定义和历史变更见：

- `docs/v2更新说明.md`
- `docs/开发计划书.md`
- `docs/手柄接收机通信协议.md`
- `docs/L431PM_对接任务提示词.md`

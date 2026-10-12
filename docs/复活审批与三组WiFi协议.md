# 复活审批与三组 Wi-Fi

## 正式比赛复活

L431 上电仍是 200 HP 准备模式，死亡 5 秒后自动复活。收到 GAME_START
后进入 300 HP 正式模式：死亡 5 秒后请求复活，保持 HP=0，服务器批准后
才恢复 300 HP。第三张黄牌判负及比赛结束后不允许复活。
正式模式死亡时 SET_HP 非零会被拒绝，避免绕过审批。

UDP V2 新增两种 10 字节帧，多字节整数小端：

| 方向 | type | 布局 |
|---|---|---|
| ESP32 -> 服务器 | 0x0D 请求 | 54 52 02 0D robot_id request_sequence request_tx[4] |
| 服务器 -> ESP32 | 0x89 批准 | 54 52 02 89 robot_id request_sequence request_tx[4] |

批准必须回显请求序号和事务号。普通 F0 ACK 不代表批准，也不会清除请求。
请求持久化至 ESP32 NVS 并持续重传，成功执行批准或观察到存活后清除。
不批准时不发送 0x89，小车保持死亡。批准回执使用现有 F0 格式，
acked_type=0x89，事务号=request_tx，result=0 成功、1 拒绝、2 失败。
批准重传使用相同请求事务号，设备复用原回执，不重复复活。

测试 CLI 收到请求后显示 REVIVE_REQUEST，人工输入 `revive ROBOT_ID` 批准；
CLI 不会自动批准。实际服务器需要实现上述请求和批准协议。

UART 新增：

| 帧 | 长度 | 说明 |
|---|---|---|
| B8 8B sequence CRC | 4 | L431 请求复活，可靠重传 |
| E8 8E sequence CRC | 4 | ESP32 持久化请求后的传输确认 |
| C7 7C request_tx[4] sequence[2] CRC | 9 | 复活批准；sequence 高字节=0 |
| D7 7D request_tx[4] result CRC | 8 | 批准结果 |
| C8 8C transaction[4] profile[2] CRC | 9 | Wi-Fi 档位同步；profile 高字节=0 |
| D8 8D transaction[4] result CRC | 8 | 档位同步结果 |

CRC 与现有 UART 协议相同。C8 成功表示 L431 接收目标档位，
枪管执行由独立 CAN 任务等待 ACK 并重试。

## Wi-Fi 手势

每次独立按下上箭头记一次，连续三次上箭头后进入选择。然后独立按下
N 次下箭头；上下箭头释放后 1 秒没有继续按动，就选择 N % 3。
长按只计一次。进入选择前的按键间隔超过 1 秒会重置计数，手柄断连或
报告超过 100 ms 则取消本轮选择。手势即使小车死亡也可识别。

| N % 3 | SSID | 枪管显示 |
|---|---|---|
| 0 | RM_GAME（正式 Kconfig） | 自动比赛灯效，CAN 0x233 命令 0x00 |
| 1 | evil rats crazily squeak，密码 66666666 | 全灭，命令 0x1F |
| 2 | 预留 | 全绿，命令 0x10，区别于红/蓝队色 |

例如上、上、上、下，释放等待 1 秒后选择 evil；三次下选择 RM_GAME。
索引为 0/1/2，零次下也选择 RM_GAME。
第三组通过 CONFIG_ROBOT_THIRD_WIFI_SSID / PASSWORD / SERVER_IP 配置。
SSID 为空时拒绝切换，保留当前网络和灯效。

Wi-Fi 选择只保存在 RAM，每次 ESP32 重启默认 RM_GAME。
切换网络后 UDP 目标与下行 IP 校验一起更新：RM 使用 CONFIG_ROBOT_SERVER_IP，
evil 使用 main.c 的 TEST_SERVER_IP，第三组使用 CONFIG_ROBOT_THIRD_SERVER_IP。
切换只改变网络和灯效，不改变准备/正式比赛状态。
L431 无连接时网络切换照常执行，链路恢复后补发灯控档位。
枪管无需升级，使用已有 0x233/0x234 协议。

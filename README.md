# ESP32-S3 SuperMini × 3DMIC-291 × 60GHz 毫米波雷达：声源定位 + 运动感知融合

在声源定位跟踪系统（继承 [ESP32_S3_CAM_Mic3_PMW1](https://github.com/art-jin/ESP32_S3_CAM_Mic3_PMW1) v2.6）基础上接入 **MS60-1211S80M 60GHz 毫米波雷达**（MoreSense，AT6010 SoC），实现：

- **声源-目标关联**：说话时自动判断声音是否来自雷达检测到的人/物，关联结果（方位差、距离）作为元数据输出
- **三种跟踪子模式**（REST 可切换，NVS 持久化）：`audio_only`（默认，纯声音）/ `fusion`（声音优先，静默 3 秒后雷达跟随）/ `radar_follow`（仅雷达驱动）
- **超范围策略**：目标超出舵机 9点~3点（±90°）时 `hold`（默认）/ `clamp` / `home` / `scan` 四选一
- **场景事件流**：目标进入/离开、声源关联、雷达离线/恢复、超范围、长静止看护告警——`/api/events` 增量轮询
- **老人居家监控场景**：麦克风（360° 听觉）+ 雷达（±60° 运动/呼吸感知）互补，舵机指向跟上场景事件

## 硬件

| 部件 | 说明 |
|---|---|
| 主控 | ESP32-S3 SuperMini（`BOARD_ESP32_S3_SUPERMINI`，注意丝印 +2 偏移）|
| 麦克风 | 3DMIC-291 三麦 MEMS 阵列，固定安装（CLK0=3/DAT0=4/CLK1=5/DAT1=6）|
| 雷达 | MS60-1211S80M，UART **115200** 8N1（TX→GPIO9，RX→GPIO8，OUT1→GPIO11）|
| 舵机 | MG90S 直驱 1:1（GPIO7），软件限位 ±90°，方向反转 |

### 感知覆盖（雷达正对被监测区，即 6 点方向）

```
 9点      10点           12点           2点      3点   ← 12点方向（背面）
  ├─────────└──── 雷达视野 ±60° ────┘─────────┤
  └──────────── 舵机可达 ±90° ────────────────┘
  └──────────────── 麦克风 DOA 360° ──────────┴── 3~9点后半圈仅事件上报
             ↑ 6点方向（雷达法线 / 舵机 home / 被监控人）
```

## 快速开始

```bash
# ESP-IDF v6.0.1
cp main/wifi_creds.h.example main/wifi_creds.h   # 填 WiFi SSID/密码
. $IDF_PATH/export.sh
idf.py build
idf.py -p /dev/cu.usbmodemXXXX flash monitor     # Ctrl-] 退出
```

上电后舵机自检扫动（±80°），随后进入声源跟踪。WiFi 连接后通过 mDNS 访问 `http://esp32-mic-<MAC4>.local`；DeviceID 从串口日志读取（6 位，REST 鉴权用）。

## REST API 摘要（完整规格见 [API.md](API.md)）

| 方法 | 路径 | 说明 |
|---|---|---|
| GET | /api/ping | 心跳（免鉴权） |
| GET | /api/status | 完整状态：DOA + 舵机 + 雷达目标 + 关联结果 + 子模式 |
| GET | /api/events?since=SEQ | 场景事件队列（32 槽，序号增量拉取） |
| GET | /api/logs | NVS 事件日志（重启留存，取证用） |
| POST | /api/mode | 切换 track/command；track 子模式 submode/oor/still_min |
| POST | /api/point | 指令转向（command 模式） |
| POST | /api/shake | 摇动舵机（command 模式） |

```bash
# 例：切到融合模式 + 开启 30 分钟长静止告警
curl -X POST "http://esp32-mic-XXXX.local/api/mode?device_id=XXXXXX" \
     -d '{"submode":"fusion","still_min":30}'
# 拉取新事件
curl "http://esp32-mic-XXXX.local/api/events?device_id=XXXXXX&since=42"
```

## 雷达协议实测要点（详见 `tasks/radar-protocol-notes.md`）

- 模组固件默认 **115200**（AT6010 文档写的 921600 不适用）；命令帧 0x58/回复 0x59，校验和为 u16 小端求和
- **主动上报（0x5A）不可用**——唯一数据源是 0x30 轮询（5Hz），返回**单个聚合目标**（fmcw_det_info_t：det_result/range/angle/conf/frame_idx）
- 无目标签名：det_result=0, range=0, conf=0；`velo` 恒 0（固件预留）
- 角度→钟点方位：`az = 187° + 2.1 × angle`（三点标定，残差 ±4°；斜率 ≈1/sin30°，固件输出正弦压缩角）
- 静态桌面幻影（呼吸@~64cm 假目标）：驱动在 init 时 save+System Reset 清除；桌面多反射体场景约 5 分钟后可能复发

## 实测状态（2026-09-02）

| 项 | 结果 |
|---|---|
| 声源-目标关联（T3） | 6点说话 diff 11°/4° 关联 ✓；视野外 diff 77° 正确不关联 ✓ |
| 偏航校准（T2） | 三点拟合残差 ±4° ✓ |
| 目标出现/消失/离线/恢复（T1/T8） | 全部正确，离线 600ms 内检出 ✓ |
| 雷达静默跟随 + 声音优先（T4） | 舵机随人移动，说话即接管 ✓ |
| 事件队列（T7） | seq/next/since 增量正确，关联事件去抖限频 ✓ |
| 长静止告警 | ALARM 触发 + 说话恢复 ✓；反抖动修复待长跑复测 |
| 超范围策略（T5） | REST/NVS/逻辑已验证；物理站位不可达，实体行为待补测 |
| 回归/稳定性（T9/T10） | 待 24h 长跑 |

## 代码结构

```
main/
├── main.c           应用装配：mic 任务 → DOA → tracker → radar/fusion
├── mic_capture / doa     I²S PDM 采集 + GCC-PHAT 三麦定位（继承）
├── servo / tracker       舵机驱动 + 跟踪决策（子模式/OOR 策略/command_target 共享路径）
├── radar.{c,h}      雷达驱动：5Hz 轮询、单目标状态、中值滤波、方位映射、离线检测、静止告警
├── fusion.{c,h}     声源-目标关联（20° 门限 + 翻转去抖 + 限频）
├── events.{c,h}     场景事件环形队列（RAM 32 槽 + evlog 镜像）
├── mode_manager     track/command + 子模式/超范围策略/告警阈值（NVS）
├── rest_api / status / wifi / evlog    REST 服务（继承 + 新端点）
└── radar_probe.{c,h}    协议探针固件（RADAR_PROBE_MODE=1 启用，调试用）
```

## 文档索引

- `ArthurReadMe.md` — 原始需求（雷达接线、场景目标；本地文件）
- `tasks/prd-radar-audio-fusion.md` — PRD：用户故事 / 功能需求 / 测试矩阵 / 开发计划
- `tasks/radar-protocol-notes.md` — 雷达协议实测笔记（波特率、帧格式、幻影、标定、告警教训）
- `CLAUDE.md` — 工程指南（构建/烧录/架构约定）
- 基础项目完整文档：上游仓库 [ESP32_S3_CAM_Mic3_PMW1](https://github.com/art-jin/ESP32_S3_CAM_Mic3_PMW1)

## 已知限制

- 麦克风 DOA 在 >1.5m 语音 3-mic 产出率骤降（10mm 阵列间距的物理限制）——远场以雷达角度为主、DOA 事件为触发
- 雷达固件仅暴露单聚合目标，多目标场景报告"最显著"目标
- 桌面/多反射体环境下静止告警事实上关闭（持续手部/幻影运动）；目标部署场景为椅子/床边单人
- T9 回归与 T10 24h 稳定性长跑待执行

## License

MIT（见 LICENSE）

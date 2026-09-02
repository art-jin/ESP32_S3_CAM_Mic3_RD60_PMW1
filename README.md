# ESP32-S3-CAM × 3DMIC-291 六向声源定位 + REST API

360° 六方向声源定位 + WiFi REST API 双模式控制，跑在 GOOUUU ESP32-S3-CAM + 3DMIC-291 三麦 MEMS 阵列上。算法用 1024 点 FFT + GCC-PHAT + 三麦几何求解。支持两种工作模式：**声源跟踪**（自动）和**指令转向**（REST API 远程控制）。

## 项目状态

**v2.0 — REST API 双模式完成**。

| 用户位置 | 期望 α | 实测 α | 偏差 | 模式 |
|---|---|---|---|---|
| 6 点钟（M3 后方）| 180° | 184.1° | +4.1° | 3-mic |
| 12 点钟（正北）| 0° | 6.6° | +6.6° | 3-mic |
| 3 点钟（正东）| 90° | 83.9° | -6.1° | 3-mic |
| 10 点钟（M2 后方）| 300° | 291.3° | -8.7° | 3-mic + 2-mic 混合 |
| 9 点钟（M1 对侧盲区）| 270° | 306° | +36° | 2-mic fallback |

**9 点钟是 3 麦阵列的几何盲区**：用户正好在 M1（2oc）的对侧，M1 信号弱导致 L/R 折叠，算法降级到 2-mic 半平面报告（"M2 side"——9oc 确实在 M2-M3 基线的 M2 半平面）。这是任何 3 麦平面阵列的固有限制，不是 bug。

单帧方位噪声约 ±15°，`stable_sextant`（3 帧滞后）锁定后不抖。

舵机驱动（GPIO38 + JS6620 + 15T/20T 外齿减速）**v1.6 完成**——`stable_sextant` 驱动舵机让 M3 跟踪声源，**feed-forward 补偿**消除闭环振荡，**前置预加重**让语音也能驱动跟踪，**PWM soft-start bug 修复**确保大范围运动不跑 clamp。

**性能**（v1.6，20T 外齿 + bug fix）：

| 测试位置 | 预期 target | 实测 servo | 状态 |
|---|---|---|---|
| 3 点钟 | -90° | **-86°** | ✓ 精准跟踪 |
| 5 点钟 | -30° | **-28°** | ✓ 完美稳定 |
| 7 点钟 | +30° | **+30°** | ✓ 完美稳定 |
| 10 点钟 | +120°→clamp | **+100°** | ✓ 极限稳定 |

**可跟踪弧**：M3 在 **~2:40 → 9:20 点钟**之间（±100°，约 5 小时 20 分弧，55% 圆周）。

**已知限制**：
- ±100° 是软件软限位（机械极限 ±101.25°，270° 舵机 / 1.333 减速比）
- 2-10 点钟全覆盖（±120°）需 320°+ 舵机或 1:1 直连，当前 270° 舵机差 50°
- 没有 IMU 时，feed-forward 是最佳算法（不需要硬件改动）
- Phase 4 UART 调试因硬件限制（CH343 单向）未完成

开发历史见 `SERVO_PLAN.md`。

**关键算法**：`α_room = α_array + β_servo` ——把声源方位从阵列坐标系换算到房间坐标系，让 target 不随舵机转动变化。详见 `CLAUDE.md` Pitfalls §6。

**已知限制**：
- ±20° 是软件软限位（机械极限 ±27°，但止位震动会触发 L/R 折叠反馈）
- 没有 IMU 时，feed-forward 是最佳算法（不需要硬件改动）
- Phase 4 UART 调试因硬件限制（CH343 单向）未完成

开发历史见 `SERVO_PLAN.md`。

## REST API（v2.0）

开机后默认进入**声源跟踪模式**。WiFi 连接后可通过 REST API 切换到**指令转向模式**，远程控制舵机指向。

**三种技能（Skills）**：

| 技能 | 模式 | 说明 |
|---|---|---|
| **声源跟踪** | `track`（默认）| DOA 自动检测声源方位，驱动舵机让 M3 指向声源。30-50cm 内 ±5° 精度 |
| **指令转向** | `command` | 声源跟踪停止，舵机只响应 REST API 指令（指向 2oc-10oc 任意整点或自定义角度）|
| **摇动** | `command` 子功能 | 在当前指向 ±10° 内左右摇动 3+2 次（~7s），用于吸引注意或确认指向 |

`track` 和 `command` 可通过 `/api/mode` 随时切换。指令模式 5 分钟无操作自动切回声源跟踪。摇动是 `command` 模式下的 `/api/shake` 接口，阻塞 ~7s 后自动结束并回到原位。

**完整 API 规格、技能详细说明、集成示例（Python/JS/curl）、典型工作流**：详见 [`API.md`](API.md)。

### 访问方式

- **mDNS**: `http://esp32-mic-<MAC4>.local`（自动发现）
- **Device ID**: 启动时看 UART 日志（仅此途径，不在网络层暴露）

### API 列表

| 方法 | 路径 | 鉴权 | 说明 |
|---|---|---|---|
| GET | `/api/ping` | ❌ | 心跳检查 |
| GET | `/api/status?device_id=XXXX` | ✅ | 查询状态（模式/舵机/方位/WiFi）|
| POST | `/api/mode?device_id=XXXX` | ✅ | 切换模式 |
| POST | `/api/point?device_id=XXXX` | ✅ | 指令转向（仅 command 模式）|

### 使用示例

```bash
# 查状态
curl "http://192.168.1.105/api/status?device_id=XXXXXX"

# 切到指令模式
curl -X POST "http://192.168.1.105/api/mode?device_id=XXXXXX" -d '{"mode":"command"}'

# 指向 7 点钟
curl -X POST "http://192.168.1.105/api/point?device_id=XXXXXX" -d '{"dir":"7oc"}'

# 切回声源跟踪
curl -X POST "http://192.168.1.105/api/mode?device_id=XXXXXX" -d '{"mode":"track"}'
```

### 安全特性

- 所有 API（除 ping）需 `?device_id=XXXX` 鉴权
- `/api/point` 速率限制 500ms（防止高频命令导致舵机抖动）
- 指令模式 5 分钟无操作自动切回声源跟踪
- Device ID 仅通过 UART 日志展示，不在 mDNS 或 API 响应中暴露

详见 `CLAUDE.md` "REST API" 章节。

## 环境限制：有效跟踪距离

3DMIC-291 麦间距仅 **10mm**，最大 TDOA = 1.4 个采样点。要在这么小的时差中提取可靠方位，需要高 SNR，导致语音有效距离受限：

| 声源类型 | 可靠跟踪（>10% 3-mic）| 偶尔触发 | 完全无效 |
|---|---|---|---|
| **吹哨** | ~2-3 m | ~4-5 m | >5 m |
| **大声说话** | ~1-1.5 m | ~2 m | >2 m |
| **正常说话** | **~30-50 cm** | ~80 cm-1 m | >1 m |

**根因**：10mm 间距为近距离交互设计（AR1105 DSP 配套）。1m 以上语音能量低于 GCC-PHAT 可靠提取 1.4 样本 TDOA 的阈值。

**适用场景**：桌面交互设备（30-50cm）。不适合房间级远场拾音。要扩大距离需改硬件（更大间距 / 更多麦 / 专用 DSP）。

详见 `CLAUDE.md` "Environmental limitations"。

## 关键发现

C3 和 S3_CAM_MIC3 两个同类项目都把 DAT0 上 L/R 时隙的"折叠"当成了硬件限制，放弃了 3-mic 定位。**这个判断是错的**——L/R 折叠是 SNR 条件性的，用户真实语音下 PDM2PCM 会自动分离 L/R 槽，3-mic 360° 定位可行。

详细对比和历史见 `CLAUDE.md` 的 "Pitfalls encountered" 一节，以及 `TECH_NOTE_for_S3_CAM_MIC3.md`。

## 硬件接线

| 3DMIC-291 | ESP32-S3-CAM GPIO | 角色 |
|---|---|---|
| 3.3V | 3v3 | 供电 |
| GND | GND | 地 |
| CLK0 | GPIO 1 | I²S PDM RX CLK |
| DAT0 | GPIO 2 | I²S PDM RX DIN[0] — M2 (L slot) + M1 (R slot) |
| CLK1 | GPIO 14 | GPIO 矩阵扇出的同一 I²S CLK 副本 |
| DAT1 | GPIO 42 | I²S PDM RX DIN[1] — M3 |
| JS6620 舵机 PWM | GPIO 38 | LEDC 50 Hz PWM（15T/20T 外齿减速，±100° 跟踪）|

> ⚠️ S3 的 I²S PDM RX 外设只暴露一个 CLK 输出。CLK1 必须通过 GPIO 矩阵把 I²S0 RX WS 信号（`I2S0I_WS_OUT_IDX`）复制到 GPIO14，否则两颗时钟各自漂移，DOA 失效。实现见 `main/mic_capture.c`。

麦克风几何（等边三角形，边长 10 mm，以板心为原点）：

```
    M2 (c0) @ 10oc        12oc (north, α=0°)
       ●
      /   \                ↗ 0°
     /     \             ↑
    /       \            |
   ●─────────●           +────→ east
 M3 (c2) @ 6oc    M1 (c1) @ 2oc
```

> **板子方向**：3DMIC-291 PCB **翻面安装**（芯片朝下，集音孔朝上）。这把布局沿 12oc-6oc 轴镜像：丝印上的 M1（10oc）现在物理位置在 2oc，丝印上的 M2（2oc）现在在 10oc，M3 不动。如果你把板子装成芯片朝上，需要把 `doa.c` 里 `sin_a = sm_01 / DOA_K` 改回负号。

> 通道到物理麦的映射是 tap test 实测得出的（2026-06-21），不是按 README 猜的。如果你想移植到别的板子，**务必重做 tap test**——参见 `CLAUDE.md` 的校准章节。

## 工具链

- ESP-IDF v6.0.1（`~/.espressif/v6.0.1/esp-idf`）
- 目标：`esp32s3`
- CMake + Ninja
- 编译器：`xtensa-esp32-elf-gcc`

## 构建 & 烧录

```bash
# 1. source IDF 环境
. ~/.espressif/v6.0.1/esp-idf/export.sh

# 2. 编译
idf.py build

# 3. 进 bootloader（GOOUUU 板的自动复位电路在 CH343 上不工作）
#    - 按住 BOOT
#    - 按一下 RST 并松开
#    - 松开 BOOT

# 4. 烧录 + 监视器
idf.py -p /dev/cu.usbmodem21201 flash monitor
# 退出 monitor: Ctrl-]
```

烧录后 USB-CDC 设备名通常是 `/dev/cu.usbmodem21201`（CH343 USB-UART）。

## 输出格式

板子启动后每 500 ms 打印一行 DOA 结果：

```
>>> 3-MIC  az=181.8°  sect=3 ( 6 o'clock)  stable=3 ( 6 o'clock)  conf=0.47
          lag01=+0.05 lag02=+1.01 lag12=+0.85  | ac 70/73/61  ρ01=0.83 peak01=0.59
```

每 5 秒打印一次模式直方图：

```
[5s] 46 frames: 3-mic=36 (78%)  2-mic=10 (22%)  bad=0
```

字段含义：
- **`az`** — 方位角 0..360°，0° = 12 点钟（正北），顺时针增加
- **`sect`** — 6 个扇区索引：0=12oc, 1=2oc, 2=4oc, 3=6oc, 4=8oc, 5=10oc
- **`stable_sextant`** — 3 帧一致才更新的滞后版本，**实际使用时看这个**
- **`conf`** — GCC-PHAT 峰值平均，0..1
- **`lag01/02/12`** — 三对麦的 TDOA（采样点，K=1.4 为物理上限）
- **`ac`** — 三通道 AC RMS（LSB），反映信号强度
- **`ρ01`** — c0 和 c1 的 Pearson 互相关，>0.95 表示 L/R 折叠
- **`peak01`** — c0 vs c1 的 GCC-PHAT 峰值

## 代码结构

```
main/
├── main.c           FreeRTOS mic 任务：capture → DOA → tracker → mode_tick → status_update
├── mic_capture.{c,h}  I²S PDM RX 多-DIN 初始化 + 阻塞读取 + GPIO-matrix CLK 扇出
├── doa.{c,h}        纯算法：FFT、GCC-PHAT、3-mic / 2-mic 几何求解、输出滞后
├── servo.{c,h}      LEDC PWM 舵机驱动 + mutex + PWM soft-start + boot sweep
├── tracker.{c,h}    DOA → servo 跟踪逻辑 + feed-forward + idle return
├── wifi.{c,h}       WiFi STA + mDNS + 事件处理 + LED 指示
├── rest_api.{c,h}   HTTP server + REST handlers + 鉴权 + CORS + 速率限制
├── mode_manager.{c,h} 双模式状态机 + 5 分钟超时
├── status.{c,h}     全局状态共享（mic_task ↔ httpd）
├── wifi_creds.h     WiFi SSID/密码（gitignored）
└── CMakeLists.txt   idf_component_register(...)
```

- 音频任务 `configMAX_PRIORITIES - 2`，8 KB 栈
- HTTP 任务 `configMAX_PRIORITIES - 5`，6 KB 栈
- 窗口：50 ms @ 48 kHz × 3 ch × int16 = 14 400 bytes / DMA read
- FFT 1024 点（~21 ms 窗口）
- K = d·fs/c ≈ 1.40 samples（边长 10 mm，48 kHz，声速 343 m/s）

## 算法概览

**Pairwise GCC-PHAT**（符号约定见 CLAUDE.md 的 pitfalls，坑过一次）：

```
lag_01 = gcc_phat(c0,c1) = arrival(M2) − arrival(M1) = +K·sin(α)
lag_02 = gcc_phat(c0,c2) = arrival(M2) − arrival(M3) = +K·sin(α−60°)
lag_12 = gcc_phat(c1,c2) = arrival(M1) − arrival(M3) = −K·sin(α+60°)
```

**3-mic 几何求解**：

```
sin α = +lag_01 / K              # 正号——因为板子翻面安装，M1/M2 位置镜像
cos α = (lag_01 − 2·lag_02) / (K·√3)
α = atan2(sin α, cos α)  →  0..360°
```

**2-mic fallback**（DAT0 L/R 折叠时）：

只用 M2 和 M3，输出半平面方位 [0°, 180°]，前后方向有歧义，分 3 个 bin。

**质量门**：
- ρ01 < 0.95（L/R 未折叠）才考虑 3-mic 模式
- peak_01 ≥ 0.40 且 peak_02 ≥ 0.40 才接受 3-mic 结果
- peak_02 ≥ 0.30 才接受 2-mic 结果
- 3 帧连续相同 raw sextant 才更新 `stable_sextant`

## 进一步阅读

- **`CLAUDE.md`** — 给 AI 协作者的完整项目文档。包括所有踩过的坑（4 个）、几何方程推导、标定步骤、舵机硬件规格、参考项目对比。
- **`TECH_NOTE_for_S3_CAM_MIC3.md`** — 写给同类项目 S3_CAM_MIC3 的移植建议（GPIO 接线、算法补丁、校准步骤）。
- **`SERVO_PLAN.md`** — 舵机声源跟踪的开发计划（硬件约束、软件架构、噪音抑制策略、分阶段实施）。

## 相关项目

- [`ESP32_C3_Mic3`](../ESP32_C3_Mic3) — 同一目标在 ESP32-C3 上的尝试。C3 的 I²S PDM RX 只支持 1 个数据线，L/R 折叠是真正的硬件天花板。
- [`ESP32_S3_CAM_MIC3`](../ESP32_S3_CAM_MIC3) — S3 + 3DMIC-291 + 摄像头 + 舵机。同样误判 L/R 折叠为硬件限制，只实现了 2-mic 1D DOA。本项目推翻了这个结论并跑通了 6 向 360° 定位。

## 许可

本项目采用 [Apache License 2.0](LICENSE)。

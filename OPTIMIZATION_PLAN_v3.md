# 优化计划 v3（整合版）

> **状态**：定稿，待实施
> **日期**：2026-06-25
> **基于**：v1.3（feed-forward + 前置预加重 + 270° 舵机修正 + ±33° clamp）
> **输入**：
> - `OPTIMIZATION_DRAFT.md`（智谱草案，量化优先）
> - `TECH_OPTIMIZATION_KIMI_PROPOSAL v2.md`（Kimi 综合版，控制理论优先）
> **目的**：形成单一可执行路线，避免两份文档之间的分歧导致实施摇摆。

---

## 0. 核心结论（先看这段）

**语音响应的真正瓶颈不是 DMA 窗，是 3-mic 帧稀缺 + 2-frame agreement。** 因此最优先的三项改动是：

1. **Per-sextant 校准补偿表**（精度，零运行时开销）
2. **Agreement 改为 2-of-3**（响应时间，对语音收益最大）
3. **滑动窗 50% 重叠 → 20 Hz 帧率**（响应时间，风险低于直接 25ms DMA）

这三项一起做，可在不动硬件、不动 FFT 大小的前提下，把语音典型响应从 0.5–3.9 s 降到 1–2 s，绝对精度从 ±5–16° 降到 ±2–3°，振荡保持为零。

---

## 1. 两份输入文档的关键差异与裁定

| 议题 | 智谱草案 | Kimi v2 | **裁定** |
|---|---|---|---|
| DMA 缩窗策略 | 直接 100→25ms（40Hz） | 50% 滑动重叠（20Hz） | **采纳 Kimi**：先 20Hz 验证 CPU 余量，再考虑 40Hz。25ms 窗与 FFT 2048 不兼容，且当前无 CPU 实测数据。 |
| 语音响应数据 | "10–15s → 3–5s" | "0.5–3.9s 典型，10–15s 极端" | **采纳 Kimi**：CLAUDE.md 记录的 v1.3 实测是 0.5s 和 3.9s，10–15s 是低 3-mic 帧率极端场景。优先级排序应基于典型值。 |
| 2-of-3 agreement 优先级 | ⭐⭐⭐（普通优化） | ⭐⭐⭐⭐⭐（核心优化） | **采纳 Kimi**：3-mic 帧率 16% 时，"连续 2 帧"等待 ~1.5s，"3 中 2"等待 ~0.8s。这是语音响应的最大单点收益。 |
| 400Hz 陷波 | 直接实施 | 先实测频率 | **采纳 Kimi**：JS6620 噪音频率会随负载漂移，需 UART 录音 + PC FFT 确认。 |
| FFT 2048 | ⭐⭐⭐，建议 50ms 窗配套 | 暂缓，Phase C | **采纳 Kimi**：内存/CPU 翻倍，收益仅 ±10°→±7°；先做更便宜的优化。 |
| 3-TDOA 残差门控 | 未提及 | ⭐⭐⭐⭐ | **采纳 Kimi**：作为 outlier rejection，剔除自相矛盾帧。 |
| PWM 软启动 | 未提及 | ⭐⭐⭐⭐ | **采纳 Kimi**：降低机械冲击和运动期噪音耦合。 |
| 两层跟踪状态机 | 未提及 | ⭐⭐⭐⭐ | **采纳 Kimi**：Acquisition vs Tracking，远近场景分开调参。 |
| 机械隔振 | 未提及 | ⭐⭐⭐⭐（非代码） | **采纳 Kimi**：硅胶垫圈，用户无需焊接。 |

---

## 2. 实施路线（三阶段）

### Phase A：低风险高收益（0.5–1 天）

零或极低风险，不动 DMA/FFT 架构，只改参数和小逻辑。

| # | 优化项 | 维度 | 文件 | 收益 |
|---|---|---|---|---|
| A1 | Per-sextant 校准补偿表 | 精度 | `main/doa.c` `doa_process()` | 系统性偏差 ±5–16° → ±2–3° |
| A2 | Agreement 2-of-3 | 响应 | `main/tracker.c` | 语音等待 1.5s → 0.8s |
| A3 | Lag 中值 5 → 3 帧 | 响应 | `main/doa.h` `DOA_HIST_N` | 平滑延迟 500ms → 300ms |
| A4 | 空闲回归 home | UX/振荡 | `main/tracker.c` 加 `idle_timer` | 防舵机长期卡极限 |

**Phase A 验收标准**：
- 6oc/12oc/3oc 三方位系统性偏差 ≤ ±3°
- 语音典型响应（正常说话音量，3-mic 帧率 ~16%）≤ 2.0s
- 吹哨响应 ≤ 0.5s（不退化）
- 振荡次数 = 0（6oc → 7oc → 6oc → 5oc 循环测试）
- 用户离开 10s 后舵机缓慢回 home

### Phase B：核心架构优化（1–2 天）

涉及帧率和控制结构改动，需逐项测试。

| # | 优化项 | 维度 | 文件 | 收益 | 风险 |
|---|---|---|---|---|---|
| B1 | 滑动窗 50% 重叠（20 Hz） | 响应 | `main/main.c`, `main/mic_capture.h` | 控制延迟 100ms → 50ms | CPU ~15% → ~30%，需 `esp_timer_get_time()` 监控 |
| B2 | 自适应 motion-pause | 响应+噪音 | `main/servo.c`, `main/servo.h` | 小角度修正快 300ms | 余震 > 200ms 时会污染 DOA，需实测 |
| B3 | PWM 软启动 / 速度规划 | 噪音+振荡 | `main/servo.c`，可能加 servo task | 降低运动期振动耦合 | 渐变延长到位时间，需与 B2 配合 |
| B4 | 两层跟踪状态机 | 响应+振荡 | `main/tracker.c` | 远距离切换快，近距离稳态精 | 状态切换边界需 hysteresis |
| B5 | 死区 3° → 1.5–2° + 软死区补偿 | 精度 | `main/tracker.h`, `main/tracker.c` | 稳态指向精度提升 | 死区过小可能微抖 |

**Phase B 验收标准**：
- 每帧 DOA 处理时间 < 25ms（用 `esp_timer_get_time()` 实测，留 50% 余量到 50ms 周期）
- 语音典型响应 ≤ 1.5s
- 舵机运动期间麦克风拾取的 300–800Hz 能量下降（主观或频谱对比）
- 振荡次数仍 = 0

### Phase C：进阶 / 实测后启用（1–3 天，可选）

需要先有实测数据支撑，或边际收益较低。

| # | 优化项 | 前置条件 |
|---|---|---|
| C1 | 400Hz 陷波器 | 先 UART 录音 + PC FFT 确认舵机噪音主频 |
| C2 | 三 TDOA 残差一致性门控 | 需标定残差阈值，注意 9oc 盲区天然不稳定 |
| C3 | GCC 峰值尖锐度门控 (`peak_ratio`) | 需与现有 `DOA_PEAK_THRESH_*` 联合标定 |
| C4 | 运动期间动态频谱掩蔽 | 收益主要在 motion-pause 刚结束时 |
| C5 | FFT 1024 → 2048 | 需 Phase B 后 CPU 实测有余量；与 25ms DMA 不兼容 |
| C6 | 几何标定（Levenberg-Marquardt）| 离线工具，需多方位采样数据 |
| C7 | 机械隔振（硅胶垫圈）| 物理改动，无需焊接 |

### 不建议实施

- **参考麦 LMS 自适应降噪**：需加第 4 颗麦 + 多一个 I²S DIN，硬件改动过大
- **β-PHAT**：调参成本高，收益不确定
- **Pre-emphasis 系数扫描**：边际收益，需多次烧录
- **舵机位置反馈（读电位器）**：需拆线引出 ADC，优先级低

---

## 3. 关键改动细节

### A1. Per-sextant 校准补偿表

在 `main/doa.c` 的 `doa_process()` 中，3-mic 分支计算 `alpha_deg` 后：

```c
/* sextant 顺序: 0=12oc, 1=2oc, 2=4oc, 3=6oc, 4=8oc, 5=10oc */
static const float sextant_offset[6] = {
    +7.0f,  /* 12oc */
     0.0f,  /* 2oc  */
     0.0f,  /* 4oc  */
    +4.0f,  /* 6oc  */
     0.0f,  /* 8oc  */
     0.0f,  /* 10oc */
};
if (out->mode == DOA_MODE_3MIC) {
    out->azimuth_deg += sextant_offset[out->sextant];
    if (out->azimuth_deg >= 360.0f) out->azimuth_deg -= 360.0f;
    if (out->azimuth_deg <    0.0f) out->azimuth_deg += 360.0f;
}
```

**标定方法**：用户在 6 个已知方位（12/2/4/6/8/10oc）说话 10s，记录平均 `alpha_deg`，填入差值。3oc 的 +16° 偏差需先确认是否含几何盲区效应（M1 在对面，L/R 弱），若是盲区效应则不应补偿（补偿会引入其他方位误差）。

**进阶**：把表存 NVS，加 console 命令现场标定（但当前 UART RX 不工作，需先解决 console 通道，见 CLAUDE.md Phase 4 限制）。

### A2. Agreement 2-of-3

替换 `main/tracker.c` 中 `s_pending_target` / `s_have_pending` 状态机为 3-slot ring buffer：

```c
typedef struct {
    float angles[3];       /* 最近 3 个 3-mic 帧的目标 */
    int   n;               /* 已填入数（< 3 时递增）*/
    int   idx;             /* 下一个写入位置 */
} agreement_buf_t;

/* 每帧调用 */
bool tracker_check_agreement(agreement_buf_t *b, float target, float *confirmed) {
    b->angles[b->idx] = target;
    b->idx = (b->idx + 1) % 3;
    if (b->n < 3) b->n++;
    if (b->n < 2) return false;
    /* 检查任意两帧差值 <= target_agreement_deg */
    for (int i = 0; i < b->n; i++) {
        for (int j = i + 1; j < b->n; j++) {
            float d = fabsf(b->angles[i] - b->angles[j]);
            if (d > 180.0f) d = 360.0f - d;  /* 圆周差 */
            if (d <= s_cfg.target_agreement_deg) {
                *confirmed = (b->angles[i] + b->angles[j]) * 0.5f;
                return true;
            }
        }
    }
    return false;
}
```

**关键**：bad/2-mic 帧**不清空 buffer，也不计入**（保持原 streak 不打断语义）。

### B1. 滑动窗 50% 重叠

保持 100ms 分析窗（保 FFT 分辨率），但每 50ms 输出一次 DOA 结果。

**实现方式**（在 `main/main.c` 的 mic_task 中）：

```c
/* 双缓冲：前 50ms + 当前 50ms */
static int16_t prev_buf[3 * MIC_WINDOW_SAMPLES / 2];  /* 前 50ms */
static int16_t curr_buf[3 * MIC_WINDOW_SAMPLES / 2];  /* 当前 50ms */
static int16_t analysis_buf[3 * MIC_WINDOW_SAMPLES];  /* 拼接 100ms */

while (1) {
    mic_read_50ms(curr_buf);            /* 读 50ms */
    memcpy(analysis_buf, prev_buf, sizeof(prev_buf));
    memcpy(analysis_buf + MIC_WINDOW_SAMPLES/2 * 3, curr_buf, sizeof(curr_buf));
    deinterleave_and_process(analysis_buf);  /* 100ms 数据 */
    memcpy(prev_buf, curr_buf, sizeof(prev_buf));
}
```

**注意**：需修改 `mic_capture.c` 让 DMA 窗口设为 50ms，但 `doa_process()` 仍按 100ms（即 `DOA_FFT_N` 不变）处理拼接后的数据。

**CPU 监控**：每次 `doa_process()` 前后用 `esp_timer_get_time()` 测耗时，超过 25ms 则告警。

### B3. PWM 软启动

把 `servo_set_angle_deg(target)` 拆成 20ms 一步的渐变：

```c
void servo_set_angle_deg_smooth(float target) {
    float current = servo_get_angle_deg();
    float diff = target - current;
    if (fabsf(diff) < 5.0f) {
        servo_set_angle_deg(target);  /* 小角度跳过软启动 */
        return;
    }
    /* 启动渐变 task 或在 servo task 中分步 */
    int steps = (int)(fabsf(diff) / 3.0f) + 1;  /* 每步 3° */
    float step = diff / steps;
    for (int i = 1; i <= steps; i++) {
        servo_set_angle_deg_raw(current + step * i);
        vTaskDelay(pdMS_TO_TICKS(20));
        s_servo.moving = true;  /* 持续标记运动中 */
    }
}
```

**风险**：渐变期间阻塞调用方，应放到独立 servo task 或用 software timer。最简单实现是 servo task 内部维护目标队列，主线程只投递目标。

### C1. 400Hz 陷波器（需先实测）

**第一步：实测舵机噪音频率**。临时加 UART 录音模式，让舵机运动期间持续输出原始 PCM，PC 端用 Python FFT 分析频谱，找到主峰。

```python
# PC 端分析脚本（示意）
import numpy as np, serial
s = serial.Serial('/dev/cu.usbmodem21201', 115200)
data = s.read(48000 * 2)  /* 1s 数据 */
pcm = np.frombuffer(data, dtype=np.int16).astype(float)
spec = np.fft.rfft(pcm)
freqs = np.fft.rfftfreq(len(pcm), 1/48000)
peak_freq = freqs[np.argmax(np.abs(spec[1:])) + 1]
print(f"Servo noise peak: {peak_freq} Hz")
```

确认频率后再设计 IIR 系数（用 Python `scipy.signal.iirnotch`），嵌入 `main/main.c` 的 `deinterleave()`：

```c
/* 二阶 IIR notch, fc=F, Q=5, fs=48kHz */
/* 系数由 Python 预计算后写死 */
static float notch_x1[3] = {0}, notch_x2[3] = {0};
static float notch_y1[3] = {0}, notch_y2[3] = {0};
/* y[n] = b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2] */
```

---

## 4. CPU / 内存预算

| 配置 | 帧率 | FFT/帧 | 预估 CPU | 内存增量 | 评估 |
|---|---|---|---|---|---|
| 当前 v1.3 | 10 Hz | 3×1024 | ~15% | — | 基准 |
| Phase A 完成 | 10 Hz | 3×1024 | ~15% | +24 B（校准表）+ ring buffer | 可忽略 |
| + B1 滑动窗 | 20 Hz | 3×1024 | ~30% | +14 KB（前 50ms 缓冲） | **推荐** |
| + B3 软启动 | 20 Hz | 3×1024 | ~30% | 微增 | servo task 开销小 |
| 激进：25ms DMA | 40 Hz | 3×1024 | ~60% | +14 KB | 仅在不跑 WiFi/摄像头时考虑 |
| C5: FFT 2048 | 20 Hz | 3×2048 | ~60% | +24 KB | 与 50ms 窗兼容，需实测 |
| 25ms + FFT 2048 | 40 Hz | 3×2048 | ~120% | +38 KB | **不可行**（除非隔帧） |

**实测义务**：Phase B 开始前，在 `doa_process()` 入口出口加 `esp_timer_get_time()` 计时，运行 30s 取最大值。若最大耗时 > 25ms（即超过 50ms 周期的 50%），不能上 20Hz。

---

## 5. 风险与缓解

| 风险 | 缓解措施 |
|---|---|
| 滑动窗导致 CPU 过载 | 加 `esp_timer` 监控；预留 `#define MIC_WINDOW_MS 100` 回退开关 |
| 改前端滤波后 peak threshold 失效 | 每次改滤波都重新标定 `DOA_PEAK_THRESH_3MIC/2MIC` 和 `lr_independent` |
| 自适应 motion-pause 过短 | 从 500ms 起步逐步降低；保留固定 500ms 作为 fallback |
| 校准表过拟合特定房间 | 表存 NVS；考虑温度补偿扩展（Phase C 之后） |
| PWM 软启动延长到位时间 | 小角度（< 5°）跳过；参数可调 |
| 陷波器频率不准 | **必须先实测**，否则可能误伤语音 F1 |
| 2-of-3 agreement 误触发 | 配合 `min_confidence` 和 out-of-range 门限；保留 streak 清零逻辑给 bad 帧 |

---

## 6. 预期综合性能

| 指标 | v1.3 | Phase A 后 | Phase A+B 后 | Phase C 后 |
|---|---|---|---|---|
| 吹哨响应 | 0.5 s | 0.3–0.5 s | 0.2–0.3 s | 0.2 s |
| 语音典型响应 | 0.5–3.9 s | 1.5–2.0 s | 1.0–1.5 s | 0.8–1.5 s |
| 语音极端响应（盲区/低帧率）| 10–15 s | 5–8 s | 3–5 s | 2–4 s |
| 系统性偏差 | ±5–16° | ±2–3° | ±2–3° | ±1–2°（含几何标定）|
| 稳态抖动 | ±3° | ±3° | ±1.5–2° | ±1–1.5° |
| 振荡 | 0 | 0 | 0 | 0 |
| 舵机极限嗡鸣 | 偶发 | 基本消除（A4 回 home）| 进一步降低（B3 软启动）| 基本消除 |

---

## 7. 落地建议

**立即开工**（Phase A，半天到一天）：
1. A1 校准表（30 分钟编码 + 一次标定实验）
2. A2 2-of-3 agreement（1 小时编码 + 测试）
3. A3 lag 5→3（改一个宏 + 测试）
4. A4 空闲回 home（1 小时编码 + 测试）

**Phase A 完成后**，根据实测决定 Phase B 节奏：
- 如果 Phase A 后语音响应已满足需求（≤ 2s），Phase B 可缓做
- 如果想进一步降到 1.5s 以内，按 B1→B2→B4→B3→B5 顺序实施

**Phase C 仅在以下情况启动**：
- Phase B 后仍有特定场景痛点（如运动期噪音、特定方位偏差）
- 有时间做实测标定（陷波频率、残差阈值）

---

## 8. 与项目现有 v1.3 标签的关系

- 本计划不破坏 v1.3 的任何工作（feed-forward、预加重、270° 舵机修正、±33° clamp 全部保留）
- Phase A 完成后建议打 tag `v1.4`（精度+响应优化）
- Phase B 完成后打 tag `v1.5`（20Hz 控制环）
- Phase C 完成后打 tag `v1.6`（鲁棒性增强）

每个 Phase 完成后更新 `CLAUDE.md` 的 Project status 段落和性能表。

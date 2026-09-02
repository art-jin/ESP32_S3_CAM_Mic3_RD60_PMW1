# 三项优化详细计划

> **状态**：计划完成，待确认
> **日期**：2026-06-30
> **基于**：v1.6（20T 外齿 + feed-forward + 预加重 + soft-start bug fix）
> **原则**：零硬件改动，不破坏现有功能，可独立回滚

---

## 优化 1：α_room EMA 平滑 + 合理性检查

### 目标

减少单帧 DOA 噪声（±15°）对 target 的影响，让舵机运动更平滑，减少无效的小幅运动。

### 当前问题

```
帧 N:   α_room = 210° → target = +30° → servo 命令 +30°
帧 N+1: α_room = 195° → target = +15° → servo 命令 +15°（噪声，不是用户移动）
帧 N+2: α_room = 213° → target = +33° → servo 命令 +33°（又跳回去）
```

每帧 ±15° 的 DOA 噪声让 target 在 deadband 边缘反复跳，舵机不停微调。

### 改动方案

**文件**：`main/tracker.c`

**新增状态变量**：
```c
static float s_alpha_room_ema = -1.0f;  // -1 = 未初始化
static float s_prev_alpha_room = -1.0f;
```

**逻辑**（插入在 feed-forward 计算 α_room 之后、out-of-range 检查之前）：

```c
float alpha_room_raw = doa->azimuth_deg + servo_get_angle_deg();

/* ---- 合理性检查：拒绝物理不可能的跳变 ----
 * 人不会瞬移。如果 α_room 单帧跳 > 60°，大概率是 GCC-PHAT
 * 噪声峰或舵机运动余震造成的幻像方位，丢弃这一帧。
 * 60° = 2 个 sextant，正常走路速度不可能达到。 */
if (s_prev_alpha_room >= 0.0f) {
    float delta = alpha_room_raw - s_prev_alpha_room;
    // 处理 0/360° 边界
    if (delta > 180.0f) delta -= 360.0f;
    if (delta < -180.0f) delta += 360.0f;
    if (fabsf(delta) > 60.0f) {
        s_mode = TRACKER_MODE_SUPPRESSED;
        return;  // 丢弃，不更新 s_prev_alpha_room
    }
}
s_prev_alpha_room = alpha_room_raw;

/* ---- EMA 平滑 ----
 * α=0.3 → 时间常数 ~100ms（2 帧 @ 20Hz）
 * 对真实位置变化（用户走动，~0.5-2s 完成）影响极小
 * 对单帧噪声（±15°）衰减 70% */
if (s_alpha_room_ema < 0.0f) {
    s_alpha_room_ema = alpha_room_raw;  // 首帧初始化
} else {
    s_alpha_room_ema = 0.7f * s_alpha_room_ema + 0.3f * alpha_room_raw;
}

float alpha_room = s_alpha_room_ema;
```

后续代码（target 计算、out-of-range、clamp 等）用 `alpha_room` 替代原来的 `alpha_room_raw`。

**tracker_init 中重置**：
```c
s_alpha_room_ema = -1.0f;
s_prev_alpha_room = -1.0f;
```

### 风险分析

| 风险 | 可能性 | 影响 | 缓解 |
|---|---|---|---|
| EMA 引入额外延迟 | 低 | α=0.3 → 100ms 延迟，可忽略（tracker 本身已有 300ms lag 中位数） | 如果实测延迟明显，调 α=0.5（50ms） |
| 合理性检查误杀真实跳变 | 极低 | 用户不可能在 50ms 内移动 60° | 60° 阈值已经非常宽松 |
| EMA 初值导致启动收敛慢 | 低 | 首帧直接初始化为 raw 值，第二帧开始平滑 | 已处理 |
| α_room 跨 0/360° 时 EMA 计算错误 | 中 | 例如 350° 和 10° 平均应该得到 0°，但直接算得 180° | **需要处理**：见下文 |

### 环形角度 EMA 的边界处理

角度是环形的（0° = 360°），直接做算术平均会出错。解决方案：转为单位向量再平均。

```c
// 将 alpha_room_raw 转为 sin/cos 分量
float sin_raw, cos_raw;
sin_raw = sinf(alpha_room_raw * M_PI / 180.0f);
cos_raw = cosf(alpha_room_raw * M_PI / 180.0f);

// EMA 在 sin/cos 空间做
static float s_sin_ema = 0.0f, s_cos_ema = 0.0f;
if (s_alpha_room_ema < 0.0f) {
    s_sin_ema = sin_raw;
    s_cos_ema = cos_raw;
} else {
    s_sin_ema = 0.7f * s_sin_ema + 0.3f * sin_raw;
    s_cos_ema = 0.7f * s_cos_ema + 0.3f * cos_raw;
}

// 转回角度
float alpha_room = atan2f(s_sin_ema, s_cos_ema) * 180.0f / (float)M_PI;
if (alpha_room < 0.0f) alpha_room += 360.0f;
```

这样在 0/360° 边界附近也正确。代价：每帧 2 次 sinf + 2 次 cosf + 1 次 atan2f ≈ ~1µs，可忽略。

### 预期效果

- target 抖动 ±15° → **±5°**（EMA 衰减 70%）
- deadband 可以从 3° 降到 **2°**（因为噪声更小）
- 舵机无效微调频率降低 50%+
- 不影响真实位置跟踪（用户走动响应 < 200ms 延迟增加）

### 回滚方案

如果导致问题，删除新增的 EMA/合理性检查代码块，恢复直接 `alpha_room = doa->azimuth_deg + servo_get_angle_deg()` 即可。改动集中在 tracker.c 的 ~15 行，不影响其他文件。

---

## 优化 2：VAD 静音门控

### 目标

静音时跳过 FFT + GCC-PHAT 计算，省 CPU + 减少噪声驱动的假阳性。

### 当前问题

即使 AC RMS 只有 8-15 LSB（纯环境噪声），doa_process 仍然跑完整 3 对 GCC-PHAT（~15ms CPU）。偶尔噪声产生的"峰"恰好通过所有阈值检查，驱动舵机乱转。

### 改动方案

**文件**：`main/doa.c`，`doa_process()` 函数开头

在 L/R 折叠检测之前（已经在计算 AC RMS 的循环里），加入能量门控：

```c
/* VAD gate: 如果三路 AC RMS 都低于噪声地板，跳过所有 DOA 计算。
 * 噪声地板 ~25 LSB（实测环境噪声 AC RMS 8-20 LSB）。
 * 留余量到 25 LSB，避免边界抖动。 */
if (c0_ac_rms < 25.0f && c1_ac_rms < 25.0f && c2_ac_rms < 25.0f) {
    out->mode = DOA_MODE_INVALID;
    out->sextant = -1;
    update_stable_sextant(out);
    return;
}
```

**插入位置**：在 `c0_ac_rms` / `c1_ac_rms` / `c2_ac_rms` 计算完成后、L/R 检测之前。

当前代码已经在 L/R 检测循环里计算了这些值（`c0_var`, `c1_var`, `c0_ac_rms` 等），所以变量已经有了。只需在循环后加一个 if 判断。

### 风险分析

| 风险 | 可能性 | 影响 | 缓解 |
|---|---|---|---|
| 阈值 25 太高，丢弃有效语音帧 | 低 | 正常说话 AC RMS 30-70 LSB，25 阈值不会误杀 | 如果实测发现误杀，降到 20 |
| 阈值 25 太低，噪声帧仍然通过 | 低 | 环境噪声 8-20 LSB，25 留了余量 | 可以用自适应阈值（跟踪最近 N 帧的最小 RMS） |
| 吹哨在远距离（3m+）可能 AC RMS < 25 | 中 | 远距离吹哨可能被门控掉 | 这本来就无法可靠跟踪，不算损失 |
| idle return 期间 DOA 被门控 → 无法检测用户回来 | 低 | idle return 期间 tracker 不依赖 DOA（它直接驱动 servo 回 home） | 用户重新说话后 AC RMS > 25，DOA 恢复 |

### 预期效果

- 静音时 CPU 30% → **~5%**（省 15ms × 20Hz = 300ms/s）
- 噪声驱动的假阳性减少 **90%+**（噪声帧直接 INVALID，不进入 tracker）
- 不影响正常语音/吹哨的检测（它们的 AC RMS 远 > 25）

### 回滚方案

删除 if 块（3 行），恢复原来"每帧都跑完整 DOA"的行为。

---

## 优化 3：舵机速度限制

### 目标

防止 tracker 命令的 target 在单帧内跳变过大（如从 -86° 到 +30° = 116° 跳变），让大角度切换更平滑，减少齿轮冲击。

### 当前问题

当前 tracker 直接命令 `servo_set_angle_deg(target)`。如果用户快速从 3oc 走到 7oc，target 从 -90° 跳到 +30°，舵机要瞬间转 120°。PWM soft-start 虽然缓了 PWM 变化（3°/20ms），但 target 本身的突变会让 soft-start timer 跑很久（120° / 3° per 20ms = 800ms），期间 tracker 被持续 motion-pause 抑制。

### 改动方案

**文件**：`main/tracker.c`，在 clamp 之后、`servo_set_angle_deg()` 之前

```c
/* 速度限制：每帧 target 变化不超过 max_delta_deg。
 * 20Hz 帧率 × 15°/帧 = 300°/s 最大角速度。
 * JS6620 物理极限约 300°/s（0.2s/60°），所以 15°/帧
 * 刚好匹配舵机机械能力。超出部分留到下一帧。 */
float max_delta = 15.0f;
float diff = target - s_last_target_deg;
if (diff > max_delta) target = s_last_target_deg + max_delta;
if (diff < -max_delta) target = s_last_target_deg - max_delta;
```

**插入位置**：在 deadband 检查之后（deadband 内不动的不受影响）、`servo_set_angle_deg()` 之前。

注意：deadband 检查的是"raw target vs last commanded"，速度限制检查的是"clamped target vs last commanded"。如果 deadband 通过（需要动），速度限制再约束单帧最大动多少。

### 风险分析

| 风险 | 可能性 | 影响 | 缓解 |
|---|---|---|---|
| 大角度切换响应变慢 | 中 | 120° 切换需要 8 帧 = 400ms 才能到位 | 这是预期行为——平滑比速度重要 |
| 与 soft-start 冲突 | 低 | soft-start 在 PWM 层缓，速度限制在 target 层缓，两者不冲突 | soft-start 每步 3°，速度限制每帧 15°，soft-start 更细 |
| 用户从 3oc 走到 9oc（180°），需要 12 帧 = 600ms | 可接受 | 用户走这段路本身也要 2-3 秒 | 600ms < 走路时间，不影响体验 |
| 速度限制与 idle return 叠加 | 低 | idle return 已经有自己的步进逻辑（2.5°/s），不经过这个速度限制代码 | idle return 在 tracker_update 早期 return，不会走到这里 |

### 预期效果

- 大角度切换从"突然甩头"变成"平滑转过"
- 齿轮冲击减少，舵机寿命延长
- motion-pause 窗口更短（每次实际运动的 Δangle 更小 → holdoff 更短）
- 对正常跟踪（±30° 以内）几乎无影响（2 帧就到位）

### 回滚方案

删除 max_delta 限制块（3 行），恢复直接命令 target 的行为。

---

## 三项改动的交互分析

| 组合 | 效果 |
|---|---|
| 1 + 2 | VAD 先过滤静音帧 → 剩余帧再做 EMA 平滑。**互补**：VAD 省计算，EMA 提精度 |
| 1 + 3 | EMA 让 target 更稳定 → 速度限制很少触发。**互补**：EMA 减少噪声跳变，速度限制处理真实大角度切换 |
| 2 + 3 | VAD 不影响速度限制。**独立** |
| 1 + 2 + 3 | 三层串联：VAD 门 → 合理性检查 → EMA 平滑 → deadband → 速度限制 → clamp → servo。每层只处理自己擅长的问题 |

### 执行顺序建议

**一次性全部实施**（三项改动互不冲突，总计 ~25 行代码，都在 tracker.c 和 doa.c）。烧一次固件，测试一轮即可判断效果。

如果担心风险，可以分两次：
1. 先做 2（VAD，最安全，只省 CPU 不改变行为）
2. 再做 1+3（EMA + 速度限制，改变了 tracker 行为，需要实测验证）

### 测试验证方法

| 测试 | 预期 |
|---|---|
| 静默 30 秒 | CPU 明显降低（VAD 跳过 DOA），舵机不动 |
| 6oc 说话 30 秒 | servo 在 0° ± 2° 稳定（EMA 平滑 + deadband） |
| 7oc 说话 15 秒 | servo 平滑到 +30°，无甩头（速度限制 + EMA） |
| 从 3oc 快速走到 7oc | servo 在 ~600ms 内平滑转过 120°，不跳变 |
| 吹哨 5 秒然后停 | 舵机跟踪到位，停吹后 10s 开始 idle return |

---

## 不改动的文件

以下文件完全不受影响：
- `main/mic_capture.{c,h}` — DMA 采集不变
- `main/servo.{c,h}` — 舵机驱动不变（soft-start bug 已在 v1.6 修复）
- `main/console.{c,h}` — 调试接口不变
- `main/main.c` — 任务调度不变（VAD 在 doa.c 内部，不改 main.c 的调用方式）

改动集中在：
- `main/doa.c`：+3 行（VAD 门控）
- `main/tracker.c`：+15 行（EMA + 合理性检查 + 速度限制）
- `main/tracker.h`：可选，新增 `max_target_delta_deg` 配置项（或直接硬编码 15.0f）

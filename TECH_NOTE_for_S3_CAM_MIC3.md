# 技术建议：在 S3_CAM_MIC3 上实现 3DMIC-291 六向声源定位

> **来自**：`ESP32_S3_CAM_Mic3_PMW1` 项目（同款硬件，无摄像头版本）
> **致**：S3_CAM_MIC3 项目维护者
> **核心结论**：你们 `hw_validate.c` 把 L/R 折叠当成硬件限制放弃了 3-mic 定位，**这个判断是错的**。L/R 折叠是 SNR 条件性的，用户真实语音下 3-mic 360° 定位是可行的。下面是我们踩过的坑、解法、以及 GPIO 改为 39/37/36/35 的具体建议。

## TL;DR

我们 PMW1 项目用同一块 3DMIC-291 + 同一颗 ESP32-S3，跑通了 README 要求的 **360° 六向声源定位**：

| 用户位置 | 期望 α | 实测 α | 偏差 |
|---|---|---|---|
| 6 点钟 | 180° | 181.8° | **+1.8°** |
| 12 点钟 | 0° | 6.6° | +6.6° |
| 3 点钟 | 90° | ~106° | +16° |

所有方向都正确归到 60° 扇区。`stable_sextant` 锁定后不抖。

你们 hw_validate.c 里的 GCC-PHAT + 1024-pt FFT 实现是正确的，但缺少三个关键检测，导致把可解的 3-mic 模式误判为不可解。把 PMW1 的 `main/doa.{c,h}` 整套搬过去（你们 GCC-PHAT 数学已经写对，只需要补 ρ01/peak 双阈值 + stable_sextant 滞后），3-mic 360° 就能跑通。

---

## 你们 hw_validate.c 的核心误区

`hw_validate.c` 在 line 578-583 写道：

> ch[0] and ch[1] are the same data (PDM port-0 L/R slots are not independent mics), so only the M3↔M2 pair carries physical TDOA. M1 (4-o'clock) shares DAT0 with M3 electrically and is invisible.

**这个结论在环境噪声下成立，但在真实语音下不成立。**

### L/R 折叠的真相

S3 片上 PDM2PCM 在低 SNR 时会把 DAT0 上的两个麦（用 PDM 时钟相位区分）折叠成同一个数据流。这就是你们看到 `ch[0] ≈ ch[1]` 的原因——**不是硬件电路限制了，而是 SNR 太低 PDM2PCM 无法区分**。

我们在 PMW1 上实测：

| 条件 | AC RMS | ρ01（c0,c1 Pearson 互相关） | L/R 状态 |
|---|---|---|---|
| 静默/环境噪声 | 10-50 LSB | 0.97-0.99 | 折叠（你们的结论） |
| 用户 50 cm 说话 | 200-1500 LSB | 0.6-0.9 | **独立** |
| 用户敲麦 | 飙到几千 LSB | 0.4-0.7 | 独立 |

SNR 一高，两个麦的 PDM 比特流就有显著差异，PDM2PCM 自动分离它们。3-mic 占比从 8% 升到 50%+。

### 关键诊断指标：ρ01（Pearson 互相关系数）

`max |c0 − c1|` 这种绝对值阈值（你们 hw_validate.c 用的诊断是看 `max_diff_01` 和 `same_01`）在低 SNR 时失效——信号本身就小，差值自然也小。改用 Pearson 互相关系数：

```c
/* ρ01 ∈ [-1, +1]。> 0.95 = 折叠；< 0.9 = 独立。 */
float cov    = (float)cross_sum / n - c0_mean * c1_mean;
float c0_var = (float)c0_sq_sum / n - c0_mean * c0_mean;
float c1_var = (float)c1_sq_sum / n - c1_mean * c1_mean;
float rho01  = (c0_var > 1.0f && c1_var > 1.0f)
             ? cov / sqrtf(c0_var * c1_var)
             : 0.0f;
```

这个量是**尺度无关的**，无论 AC RMS 是 30 还是 3000 都能正确区分折叠/独立。

---

## 算法移植建议

你们 hw_validate.c 的 GCC-PHAT + 1024-pt FFT + 抛物线峰值插值**全部正确**（我们也借鉴了）。但需要在 GCC-PHAT 之后、几何求解之前补三道质量门，缺一不可：

### 门 1：ρ01 L/R 折叠检测

如上。阈值 `ρ01 < 0.95` 才考虑 3-mic 模式。

### 门 2：GCC-PHAT peak 双阈值

```c
#define DOA_PEAK_THRESH_3MIC  0.40f
#define DOA_PEAK_THRESH_2MIC  0.30f

int three_mic_eligible = lr_independent
                       && (peak_01 >= DOA_PEAK_THRESH_3MIC)
                       && (peak_02 >= DOA_PEAK_THRESH_3MIC);
```

**为什么必要**：即使 ρ01 < 0.95（看起来独立），纯噪声帧的 GCC-PHAT 也会给出**随机但低于 0.4** 的峰。如果不加这个门，会报出 az=19° / az=1° 这种完全错误的方位（实测过）。

3-mic 模式用 `peak_01`（M1↔M2）和 `peak_02`（M2↔M3），因为几何方程只用到这两个 lag。`peak_12` 用于一致性检查但不门控。

### 门 3：stable_sextant 输出滞后

要求 3 帧连续相同 raw sextant 才更新 `stable_sextant`。但有个**非常隐蔽的坑**：

**2-mic 模式的 sect 是 0..2 尺度（"M2 side / broadside / M3 side"），3-mic 模式是 0..5 尺度（"12oc / 2oc / ... / 10oc"）。** 这两个尺度的数值 2 含义完全不同（2-mic 的 2 是 "M3 side"，3-mic 的 2 是 "4 o'clock"）。如果用同一个 hysteresis 函数处理两种模式，2-mic 帧的 sect=2 会污染 pending streak，导致 stable_sextant 卡在错的值。

我们的写法（关键部分）：

```c
/* 2-mic 分支：调用 hysteresis 时把 sextant 临时设为 -1，
 * 让它不推进 streak 但仍然 echo 上一个 stable 值。 */
int saved = out->sextant;
out->sextant = -1;
update_stable_sextant(out);
out->sextant = saved;
```

3-mic 分支和 invalid 分支正常调用即可。

---

## GPIO 引脚建议（35/36/37/39 这套）

你们 hw_validate.c 当前用的就是 35/36/37/39（user 提到改成"39/37/36/35"是同一组）。**这套不用改。** 我们 PMW1 用的是 1/2/42/14，纯粹的 GPIO 选择差异，代码逻辑完全一样。

| 信号 | 你们 hw_validate.c | PMW1（参考） | 角色 |
|---|---|---|---|
| I²S PDM RX CLK | **GPIO 36** | GPIO 1 | 主时钟，驱动 DAT0 上两颗麦 |
| DAT0 (DIN[0]) | **GPIO 35** | GPIO 2 | M2 + M1 共享（PDM 时钟相位区分） |
| DAT1 (DIN[1]) | **GPIO 37** | GPIO 42 | M3 独占 |
| CLK1 fanout | **GPIO 39** | GPIO 14 | GPIO 矩阵把 I²S CLK 复制给 3DMIC 的 CLK1 |
| 舵机 | GPIO 38 | GPIO 38 | 50 Hz LEDC PWM |

**只改 4 行 `#define` 就能切换 pinout。** 算法代码与 GPIO 完全解耦。

### 与摄像头共存

S3-CAM 板的 OV3660 摄像头占用 GPIO 19/20（USB D-/D+ 复用）。你们接了摄像头，所以麦不能用 19/20。35/36/37/39 这套避开冲突，**和 PMW1 的 1/2/42/14 在功能上等价**。

### Octal PSRAM caveat（必读）

ESP32-S3-WROOM-1 N16R8 模块带 **octal PSRAM**，理论上 GPIO 35/36/37 是 SPI flash 的 DQ4/DQ5/DQ6 数据线。**但你们 hw_validate.c 实测能跑**，说明：

1. 实际硬件可能是 N16R2（quad PSRAM）变种，或
2. SPI flash/PSRAM 在运行时不会持续占用这些 pin 做输入，或
3. 你们板子厂商定制了模块

**建议**：升级 IDF 或换板子前，先用 `idf.py monitor` 看启动日志里 `SPIRAM` 那行确认模块型号。如果之后换成 N16R2 / N8R2 / R8 这类 quad PSRAM 模块，GPIO 35/36/37 就完全可用；如果是真正的 N16R8 octal PSRAM，可能需要考虑迁移到 PMW1 用的 1/2/42/14 pinout（避开 SPI 数据线）。

---

## 校准步骤（必做！）

跑通算法后，**必须做两件事**才能信任输出。我们 PMW1 一开始跳过了这两步，结果浪费了好几轮 debug。

### 步骤 1：Tap test 确定 c→M 映射

你们的 README 写"DAT0 = M0+M2, DAT1 = M1"。**我们 PMW1 实测不是这样**：DAT0 实际是 M2+M1（c0=M2, c1=M1），DAT1 是 M3（c2=M3）。

S3_CAM_MIC3 板上的实际映射可能相同也可能不同（取决于 3DMIC-291 PCB 走线）。**唯一可靠的验证方法是 tap test**：

```python
# 烧固件后，让用户依次敲三颗麦，每颗间隔 2 秒
# 看每次哪个 c 通道飙到几千 LSB
import serial, re
s = serial.Serial('/dev/cu.usbmodem...', 115200, timeout=1)
peak_rms = [0, 0, 0]
while True:
    line = s.readline().decode('utf-8', errors='replace')
    m = re.search(r'ac (\d+)/(\d+)/(\d+)', line)
    if m:
        rms = [int(m.group(i)) for i in (1,2,3)]
        for i in range(3):
            if rms[i] > peak_rms[i]: peak_rms[i] = rms[i]
    # 用户敲完三颗麦后，peak_rms 最大值告诉你哪个 c 对应哪颗物理麦
```

按实测结果改 `doa.c` 顶部注释里的 c→M 映射，几何方程也要相应调整。

### 步骤 2：Sign convention 验证（站在已知麦正后方）

GCC-PHAT 通过 `IFFT(A·conj(B))` 实现时，返回的 peak lag 对应：

```
peak n  =  arrival(a) − arrival(b)
```

**不是**教科书上的 `arrival(b) − arrival(a)`。这是 FFT 共轭引入的符号翻转。

如果你按教科书约定写几何方程，所有方位会**偏 180°**，而且数据内部完全自洽——极难发现。

验证方法（30 秒就能确认）：

1. 烧固件
2. 让用户站在 c2 那颗麦（DAT1 接的）正后方 50 cm
3. 对着麦说话 10 秒
4. 看日志里 `3-MIC az=` 的平均值

**预期**：
- az ≈ 180°（如果 c2 那颗在 6 点钟）→ 符号对
- az ≈ 0° → 符号反了

PMW1 修正版的几何方程：

```c
float sin_a = -sm_01 / DOA_K;
float cos_a = (sm_01 - 2.0f * sm_02) / (DOA_K * sqrtf(3.0f));
```

注意 `sin_a` 前面的负号——这是修正 GCC-PHAT 符号的关键。

---

## 已知坑点总结

| 坑点 | 现象 | PMW1 的解法 | 涉及代码 |
|---|---|---|---|
| L/R 折叠被误判为硬件限制 | 3-mic 定位被认为不可行，全部走 2-mic | ρ01 + peak 双阈值分清"低 SNR"和"硬件折叠" | `doa.c::doa_process` |
| 教科书 GCC-PHAT 符号约定 | 方位偏 180°，数据内部自洽极难发现 | 几何方程前加负号；站在已知麦正后方验证 | `doa.c` 3-mic solve |
| DAT0/DAT1 ↔ M1/M2/M3 mapping 错 | 方位偏任意角度（我们偏 90°） | Tap test，分别敲三颗麦，看哪个 c 响应 | `doa.c` 顶部注释 + 几何方程 |
| 2-mic sect 和 3-mic sect 尺度冲突 | `stable_sextant` 卡在错的值（一直 2） | 2-mic 帧调用 hysteresis 时 sextant = -1 | `doa.c::update_stable_sextant` |
| 绝对阈值 `max_diff > 50` 在低 SNR 失效 | 真信号被误判为折叠 | 改为 `max(25, 0.5 * AC_RMS_c0)` 跟着信号缩放 | `doa.c` L/R 检测 |
| 噪声帧 GCC-PHAT 给随机峰 | 噪声帧报随机方位 | `peak_01/peak_02 ≥ 0.40`（3-mic）/`≥ 0.30`（2-mic）双阈值 | `doa.c::doa_process` |
| `idf.py monitor` 需要 TTY | 在某些 shell（非交互）跑不了 | 改用 `python3 -c "import serial..."` 抓 | 看 PMW1 的 CLAUDE.md |
| Clangd 报 `sys/features.h` not found | 编辑器假错 | 忽略，`idf.py build` 正常 | 编辑器配置问题 |

---

## 推荐移植流程

1. **复制文件**：把 PMW1 的 `main/doa.{c,h}` 整套复制到 S3_CAM_MIC3 的 `main/` 下，替换 hw_validate.c 里的 GCC-PHAT 实现
2. **改 GPIO**：把 `mic_capture.h` 里的 `MIC_CLK0_GPIO` 等常量改成你们的 36/35/37/39（或者你们 hw_validate.c 里现有的 I²S init 保留，只换算法部分）
3. **改 CMakeLists.txt**：把 `doa.c` 加进 `SRCS`
4. **做 tap test**：参考上面"校准步骤 1"，验证 c→M 映射并改 `doa.h` 几何注释
5. **做 sign convention 验证**：参考上面"校准步骤 2"
6. **跑 6/12/3 点钟实测**：和 PMW1 的标定结果对比，误差应该 < ±20°
7. **驱动舵机**：`stable_sextant` 直接喂给 `servo_set_bin()`（你们 hw_validate.c 已经有这套）

整个过程预计 1-2 小时（不含标定调试）。

---

## 参考文件位置

PMW1 项目（`~/PycharmProjects/ESP32_S3_CAM_Mic3_PMW1/`）：

- `main/mic_capture.{c,h}` — I²S PDM RX 多-DIN 驱动 + GPIO matrix CLK 扇出（120 行）
- `main/doa.{c,h}` — FFT + GCC-PHAT + 几何求解 + ρ01/peak 双阈值 + stable_sextant 滞后（350 行）
- `main/main.c` — FreeRTOS 单任务 + UART 日志（120 行）
- `CLAUDE.md` — 完整踩坑记录、几何推导、实测精度

最值得抄的：
- `doa.c::doa_process()` 端到端算法
- `doa.c::update_stable_sextant()` hysteresis 逻辑（注意 2-mic 的尺度冲突坑）
- `doa.h` 顶部的几何注释——完整的 c→M mapping 和 TDOA 方程

S3_CAM_MIC3 项目（`~/PycharmProjects/ESP32_S3_CAM_MIC3/`）：

- `main/hw_validate.c` — 你们现有的实现，GCC-PHAT 数学是对的，只需补三道质量门
- `main/camera_pins.h` — 摄像头引脚（与麦不冲突）
- `CLAUDE.md` — 项目背景

---

## 联系

如果对算法细节、几何推导、或者实测过程有疑问，看 PMW1 的 CLAUDE.md "Pitfalls encountered" 一节，里面有完整的 debug 过程记录。

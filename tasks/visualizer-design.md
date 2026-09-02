# 可视化页面设计（态势图 + 双控制栏）

状态: 设计定稿（2026-09-02），V1/V2 分期实施
决策记录: 设备内置（固件服务静态页）✓；雷达配置完整实现（US-010）✓

## 1. 布局

```
┌─────────────────────────────────┬──────────────────────┐
│         雷达态势图 (PPI)         │  麦克风阵列 / 跟踪    │
│         (Canvas, 正方形)         │  (上栏)              │
│                                 │  模式/子模式/超范围/  │
│   声音三角(12钟点位) 目标方框     │  长静止/DOA读数/     │
│   距离环 舵机指向针 雷达扇区      │  (指令模式:指向/摇动) │
│                                 ├──────────────────────┤
│                                 │  雷达 (下栏)          │
│                                 │  链路/目标卡(只读)     │
│                                 │  检测配置/保存/重置    │
└─────────────────────────────────┴──────────────────────┘
```

## 2. 左侧 PPI 元素

| 元素 | 数据源 | 表现 |
|---|---|---|
| 本机 | — | 圆点居中 |
| 舵机指向针 | status.servo | 从圆心伸出的针，实时角度（0°=6点方向，±90°=9/3点弧） |
| 距离环 | status.radar.target.range_mm | 0.5/1/2/5m 同心环 + 刻度标签；目标出现时自动缩放到包含目标 |
| 雷达扇区 | 常量 | 4点~8点（±60°）淡色底纹 + 边界虚线 |
| 舵机弧 | 常量 | ±90° 弧线标注"9点~3点可达" |
| 目标方框 | status.radar.target | 极坐标(azimuth, range)定位；固定尺寸；状态色：运动=橙/呼吸=绿/靠近=蓝/远离=紫；框边标签: 距离/方位/conf；无效(conf<12/8)半透明 |
| 声音三角 | events SOUND_ASSOC/UNASSOC + status.azimuth | 12 个钟点位外圈红色三角+光晕；关联=实心+连线到目标框，未关联=空心；3 秒渐隐 |
| 关联连线 | fusion.associated | 三角↔方框红色连线（关联时） |

诚实性约束（不画固件没有的数据）：单目标只有一个方框；无大小维度（方框固定）；无速度数值（用状态色代替）；velo 恒 0 不显示。

## 3. 右侧上栏（麦克风阵列 / 跟踪）→ 现有 API

| 控件 | API | 说明 |
|---|---|---|
| 模式切换 | POST /api/mode {"mode"} | track / command（command 下出现指向/摇动） |
| 子模式单选 | POST /api/mode {"submode"} | audio_only / fusion / radar_follow |
| 超范围策略单选 | POST /api/mode {"oor"} | hold / clamp / home / scan |
| 长静止阈值 | POST /api/mode {"still_min"} | 数字输入(0-1440)+开关 |
| 指向 | POST /api/point {"dir"/"angle"} | command 模式，12 钟点按钮 + 角度输入 |
| 摇动 | POST /api/shake | command 模式，提示阻塞 ~7s |
| DOA 读数 | GET /api/status | azimuth / sect / conf 实时数字+历史迷你条形图（可选） |

## 4. 右侧下栏（雷达）→ V2 新端点

只读区（现有数据）：链路状态灯、目标详情卡（状态/距离/方位/conf/frame_idx/角度原始值）。

配置区（US-010，新固件端点）：

| 控件 | UART 命令 | 约束 |
|---|---|---|
| 感应开关 | 0xD1 (on/off) | — |
| 运动检测距离 min/max | 0x34 / 0xD2 | 0-10m，单位 cm |
| 微动检测距离 min/max | 0x37 / 0x36 | 0-8m（固件版本无效项仍下发，读回验证） |
| 呼吸检测距离 min/max | 0x3A / 0x39 | 0-8m |
| 灵敏度（运动/微动/呼吸） | 0x35 / 0x38 / 0x3B? | 0-10 档 |
| 保存 | 0x08 save | 提示"写入雷达 flash" |
| 重置雷达 ⚠ | 0x13 system reset | 副作用=清除幻影；提示会中断 ~4s |

**新增 REST**：
```
GET  /api/radar?device_id=      → {online, bounds, cfg, target}（读回 0x32/0x33）
POST /api/radar?device_id=      → {"mot_min_cm":50,...,"save":1}（任选字段）
POST /api/radar/reset?device_id= → 发送 System Reset（清幻影运维按钮）
```

## 5. 数据流与实现

- 轮询：`/api/status` 2Hz；`/api/events?since=` 1Hz 增量；配置读回 `/api/radar` 5s 一次或按需
- 单文件 `main/web/index.html`（HTML+CSS+Canvas+原生 JS，无框架无依赖，中文 UI），CMake `EMBED_FILES` 编入 flash（估 25-45KB，分区余量 80%）
- 固件：`httpd_register_uri_handler` GET `/` 返回嵌入页（Content-Type text/html, gzip 可选）
- 鉴权：页面本地填 DeviceID 存 localStorage，所有请求自动附加 query 参数（/api/ping 免鉴权用于连通性测试）

## 6. 分期

| 期 | 内容 | 固件改动 |
|---|---|---|
| V1 | PPI 态势图 + 右上全部控制 + 右下只读目标卡 | 仅静态页嵌入与 `/` 路由 |
| V2 | 右下配置区：/api/radar 三端点 + radar.c 命令构造/回复分发 | US-010 |

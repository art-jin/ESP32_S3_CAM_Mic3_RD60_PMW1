# 开新脖子固件 SOP（固定步骤）

> 沉淀自 2026-11-04 anan（安安）第 4 台脖子开号全流程（前例: redwolf 09-08 / heizai 09-20 / greywolf 10-27）。
> 每次开新脖子照此执行；跨仓库单一事实源在 `../Hermes_ESP32_MQTT/docs/48-radar-presence-trigger.md`（§12/§15 照步登记）。
> 机器人命名/人设见 docs/75（安安）；broker 运维细节见 memory `aliyun-broker-ops`。

## 0. 前置输入（开号前必须拿到）

- [ ] 机器人名与 prefix（如 anan → `arthur139/anan`）
- [ ] 新板已接好硬件（雷达 TX/RX **先核对再上电**——TX-TX 互顶可烧模块，2026-11-04 anan 台架第一只雷达即此退役）
- [ ] 4.2 屏换其他面板时确认 VCI=3.3V（5V 过压死屏前科）

## 1. Broker 开户（SSH，照 aliyun-broker-ops 配方）

```bash
PW=$(openssl rand -hex 16) && echo "$PW" > /tmp/<robot>_neck_pw.txt   # 只进 gitignored creds
ssh -i ~/.ssh/KinCalKey.pem root@8.160.185.39
cd /opt/mqtt-broker/deploy/aliyun-broker
# 密码（docker mosquitto_passwd）
docker run --rm -v $PWD/passwords.txt:/passwd.txt eclipse-mosquitto:2 \
  mosquitto_passwd -b /passwd.txt arthur139_<robot>_neck $PW
# ACL：照 neck 五行块模板追加（保 inode！cat tmp > acl.conf，绝不能 mv/sed -i）
#   user arthur139_<robot>_neck
#   topic write arthur139/<robot>/neck/#
#   topic read  arthur139/<robot>/cmd/cfg_set
#   topic read  arthur139/<robot>/cmd/display
#   topic write arthur139/<robot>/status/cfg
docker compose exec -T mosquitto kill -HUP 1          # 无停机重载
```

验收（Mac 本机，**必须 -V 5**，mosquitto 2.1.2 的 3.1.1 PUBACK 无 reason code）：

```bash
mosquitto_pub -h mqtt.kincal.cn -p 8883 --cafile /etc/ssl/cert.pem \
  -u arthur139_<robot>_neck -P "$PW" -V 5 -q 1 -t arthur139/<robot>/neck/test -m x -d
#   own 树 PUBACK RC:16（无订阅者，正常）
#   跨树（他机）RC:135（拒）✓
```

## 2. creds 双块切换（gitignored，永不入 git）

- `main/mqtt_creds.h`：新块 ACTIVE（BROKER/USERNAME/PASSWORD/ROBOT_PREFIX/`MQTT_EXPECT_DEVICE_ID` 先占位 `TESTXX`），旧机器人块转 INACTIVE 注释
- `main/wifi_creds.h`：双槽 DHCP 不动；mDNS 主机名从 prefix 自动推导 `anban-neck-<robot>.local`（wifi.c）
- 面板非 4.2 时：`main/board_config.h` 面板开关（4.2 默认 / `EPD_PANEL_5IN83`），注释写明归属与日期

## 3. 首烧 → 读设备号 → 回填 → 重烧（两烧流程）

```bash
idf.py -p /dev/cu.usbmodemXXXX build flash
# 抓启动横幅读 NVS 设备号（identity guard 未匹配 → MQTT 拒连，属预期）：
#   grep "Device ID (from NVS)" /tmp/capture_last.log
# 回填 mqtt_creds.h MQTT_EXPECT_DEVICE_ID=<实测> → 重烧
```

注：esptool 与 capture.py 抢串口——复位+采集用一次性 pyserial 脚本（RTS 脉冲复位后同口采集）。

## 4. 台架验收清单（全部要看实测，不看想当然）

| 项 | 通过标准 | 手段 |
|---|---|---|
| WiFi/mDNS | DHCP 拿 IP、`anban-neck-<robot>.local` 可解析 | ping + curl（mDNS 抖就直连 IP） |
| REST 鉴权 | `/api/status?device_id=<ID>` 返回 ok | curl |
| MQTT | connected=true、seq 持续增长、dropped=0、own 树留痕、cfg_set 订阅成功 | `/api/status` + 串口 `mqtt:` 行 |
| 雷达 | online=1、miss=0、stray=0、conf 16/16、走动有目标 | 串口 `radar: [5s]` 行；可视化页 `/` |
| 3DMIC | 3-mic 帧率 >50%（接触病则重插再测）、DOA 流动 | 串口 `[5s] frames` + `>>> 3-MIC` 行 |
| 舵机 | 开机扫掠 0→±80→0 | 目视 + 串口 `servo:` 行 |
| EPD | init_ok=true + 三色卡/目标页上屏 | `/api/epd/status`（pattern_ms<300=没engage）+ 目视 |
| 内存 | heap 低水位充裕（多面板/大缓冲须 PSRAM） | 串口 `heap=A/B`（2026-11-04 起 PSRAM 已启用） |

排查工具：`/api/epd/{wiring,recover,status,sweep}`、`/api/radar(/reset)`、`tools/capture.py`。

## 5. 五处登记（漏一处下次必踩）

1. `../Hermes_ESP32_MQTT/fleet/config/devices.json` — 增 `<robot>` 条目（neck url=mdns + id；camera 待装机则不写）
2. `../Hermes_ESP32_MQTT/docs/48-radar-presence-trigger.md` — §15 进度日志新条目（开户/身份/验收结论/遗留/DHCP 重叠注记）+ §16 变更记录一行
3. `../Hermes_ESP32_MQTT/docs/74-epd-multi-mode-panel-plan.md` — 仅涉及面板/显示时（§7 实施进展注记）
4. 本仓库 `CLAUDE.md` — 状态块（ACTIVE 板归属、新事实、待办）
5. memory — `project_docs48_presence_trigger`（舰队现状）、`project_cam_epd_phase`（面板进展）

## 6. Commit 规范

- 本仓库：固件改动 + CLAUDE.md + SOP 一并 commit（creds 是 gitignored 不会带走）
- Hermes 仓库：devices.json + docs/48 + docs/74 逐文件 add（避开他人未提交文件），单独 commit
- 运维注记别忘：新机器人**机身板（对话板）上线时须发一次 `cmd/presence_on`**（存量机器惯例）；装机后 zone 参数按部署位定型

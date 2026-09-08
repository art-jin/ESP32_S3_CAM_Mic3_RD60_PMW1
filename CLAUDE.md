# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ESP32-S3-SuperMini + 3DMIC-291 three-mic array + **MS60-1211S80M 60GHz mmWave radar** + 1-channel PWM servo. Goal: combine the existing acoustic sound-source localization (DOA + servo tracking) with the radar's multi-target motion sensing — 声源定位与多目标运动状态探测.

**Current status: feature-complete through fall detection (2026-09-09); tags `v0.1.0-radar-fusion` / `v0.2.0-visualizer` on GitHub.** The mic-array + servo + REST-API stack was imported from the base project's working tree into `main/`; this project added: `radar.c` (5Hz 0x30 poll driver with offline auto-recovery + baud self-heal, stillness alarm, **fall-suspect detector** — bimodal range-oscillation signature, see protocol notes §11, and the config-request mailbox), `fusion.c` (sound↔target association + the optional association gate that blocks robot-speaker capture), `events.c` (scene-event ring), tracking sub-modes + OOR policy in `mode_manager`/`tracker`, the embedded visualization page (`main/index.html` served at `/`, EMBED_FILES — PPI scope, controls, radar config panel, experimental multi-target tracking, lying-posture badge), and REST endpoints (`/api/events`, `/api/radar(+/reset)`, extended `/api/status` + `/api/mode`). Pending: T9 regression, T10 24h soak, T5 physical OOR test, bed-zone suppression for bedside fall deployment (protocol notes §11 — lying-on-bed and lying-on-floor range clusters overlap). Read `ArthurReadMe.md` (requirements) first, then `tasks/prd-radar-audio-fusion.md` (PRD), `tasks/radar-protocol-notes.md` (measured facts and detector constants) and `tasks/visualizer-design.md` (page design).

**Bench workflow**: `tools/capture.py [秒] [正则]` — countdown banner + per-second timer + regex-filtered live tail over the USB serial; full log lands in `/tmp/capture_last.log`. Synchronized captures (user acts while the tool runs) are the standard way to validate detector changes.

**Cross-repo project**: 雷达人感触发免唤醒词对话（presence-triggered wake）→ single source of truth at `../Hermes_ESP32_MQTT/docs/48-radar-presence-trigger.md`. **This repo owns Tier 1** (MQTT publisher + Zone presence state machine on the radar neck; current deployment robot is **redwolf**, prefix `arthur139/redwolf`). Update that doc's §12/§15 after each task.

## Base project (source of existing code)

The inherited stack comes from `~/PycharmProjects/ESP32_S3_CAM_Mic3_PMW1` (GitHub: `art-jin/ESP32_S3_CAM_Mic3_PMW1`), copied here as a fresh repo (no shared git history). The import captured the base project's **uncommitted v2.6 working-tree state**: `BOARD_ESP32_S3_SUPERMINI` + `SERVO_MODEL_MG90S_DIRECT_DRIVE` already enabled in `main/board_config.h`, and the EPD scaffold files removed.

**`CLAUDE_BASE.md`** (frozen copy of the base project's CLAUDE.md, v2.6; local-only, gitignored — the equivalent lives in the upstream repo on GitHub) is the authoritative doc for the inherited stack: DOA pipeline (GCC-PHAT, sextant calibration, sign conventions), tracker (feed-forward, boot grace period), servo driver, REST API, evlog/coredump diagnostics, build/flash procedure, and the 9 documented pitfalls. Do not duplicate that content here; read it before touching inherited code. If the base project evolves further upstream, diff against the sibling directory.

Relevant base-project config for this build (already supported there via compile-time switches in `main/board_config.h`):
- `BOARD_ESP32_S3_SUPERMINI` — this project's board
- `SERVO_MODEL_*` — servo selection (board and servo are orthogonal switches)
- `MIC_ARRAY_MOUNTED_ON_SERVO` — array-rotating vs fixed-array tracker math

## Hardware wiring

Radar ↔ ESP32-S3-SuperMini (from `ArthurReadMe.md`):

| Radar pin | ESP32-S3 GPIO | Role |
|---|---|---|
| OUT1 | GPIO 11 | radar digital output |
| RX | GPIO 8 | ESP32 TX → radar RX (UART) |
| TX | GPIO 9 | radar TX → ESP32 RX (UART) |

Mic array / servo / LED GPIO map for SuperMini (from base project's CLAUDE.md; no conflicts with radar pins 8/9/11): CLK0=3, DAT0=4, CLK1=5, DAT1=6, servo=7, LED=48.

**SuperMini pitfalls** (details in base CLAUDE.md):
- Silkscreen numbers are offset +2 from actual GPIO numbers (silkscreen "N" = GPIO N+2)
- Servo GND must be common-grounded with 3DMIC GND and the board, or PWM has no return path

## Radar module (MS60-1211S80M)

Vendor: 深圳觅感科技 (MoreSense); SoC: AT6010 (AIR Touch 隔空科技). 1T2R FMCW at 60GHz, 12.0×11.0×2.0 mm, 3.0–5.5 V, ~80 mA average.

Capabilities (product manual §1.2): multi-target motion detection outputting per-target distance / velocity / angle; micro-motion (breathing) presence detection for stationary people; multi-target localization with target IDs.

### Reference documents (`reference/`)

- `产品手册.pdf` — product manual: specs, dimensions, pin definitions, output parameters, installation (horizontal / tilted / ceiling), app notes
- `雷达通信协议.pdf` — AT6010 HCI protocol: frame formats, all commands (basic, radar config, ULP, active reporting, debug), UART/IIC interfaces

Both PDFs are text-based Chinese; extract with `pdftotext` (available on this machine).

### Protocol essentials (from 雷达通信协议.pdf)

- UART: **115200 8N1 measured** (2026-09-02 Phase 0 — the AT6010 doc's 921600 default does NOT apply to this module's MoreSense firmware); IIC also available (§4.2)
- Host→radar frame: `Head 0x58` + payload (CMD Group 3 bits + CMD 5 bits, Parameter Length, params) + `Check Code` = **u16 little-endian sum** of all preceding bytes
- Key command groups: 3.1 basic (reset, version, save settings), 3.2 radar config (motion/micro-motion/breath detection distances & sensitivities), 3.4 active reporting — **NOT functional on this firmware** (never streams; poll command 3.2.6/0x30 instead)
- Active report TYPE=0 struct = same `fmcw_det_info_t` returned by the 0x30 poll: `is_detected`, `det_result` (0x04 运动 / 0x10 呼吸 / 0 = no target), `range_val` (u16, mm), `angle_val` (s16, 1° units), `velo_val` (s16, always 0), `rb_conf` (u8, 0–16; **range_val may be wrong when < 12**)
- Report types also exist for altimeter, occupancy, motion-presence, breath/heart-rate, and zone detection (3.4.1–3.4.6)

The radar's range+angle output is the intended fusion input for the mic-array DOA (radar gives coarse azimuth + range + motion state for ONE aggregated target via 5 Hz polling of command 0x30; the 3-mic array gives precise azimuth for the sounding source). Phase 0 measured findings live in `tasks/radar-protocol-notes.md`; the radar probe firmware (compile-time `RADAR_PROBE_MODE` in `main/radar_probe.c`) replaces normal startup for protocol sniffing.

## Toolchain & build (inherited from base project)

ESP-IDF v6.0.1 at `/Users/arthurjin/.espressif/v6.0.1/esp-idf`; target `esp32s3`; CMake + Ninja.

```bash
. /Users/arthurjin/.espressif/v6.0.1/esp-idf/export.sh   # before any idf.py command
idf.py build
idf.py -p <port> flash
idf.py -p <port> monitor    # Ctrl-] to exit
idf.py -p <port> coredump-info   # panic backtrace after reboot
```

Verify the serial port for this SuperMini board before flashing (the base project's S3-CAM used `/dev/cu.usbmodem21201`; SuperMini enumerates on native USB-CDC and the name will differ — check `ls /dev/cu.usbmodem*`).

## GitHub delivery

Published at `art-jin/ESP32_S3_CAM_Mic3_RD60_PMW1` (public, `main`). Pushes go through a local proxy at 127.0.0.1:7001 — when it is down, direct pushes fail; wait for the user to bring it up (the user manages it) rather than reconfiguring git.

# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project overview

ESP32-S3-SuperMini + 3DMIC-291 three-mic array + **MS60-1211S80M 60GHz mmWave radar** + 1-channel PWM servo. Goal: combine the existing acoustic sound-source localization (DOA + servo tracking) with the radar's multi-target motion sensing — 声源定位与多目标运动状态探测.

**Current status: base code imported (2026-09-02), radar development not yet started.** The mic-array + servo + REST-API stack was copied from the base project's working tree (see below) into `main/`. Read `ArthurReadMe.md` (the requirements doc, in Chinese) first, then `CLAUDE_BASE.md` for the inherited stack. Radar fusion is the new work. The reviewed requirements/test plan/dev plan for the radar work live in **`tasks/prd-radar-audio-fusion.md`** — that file is the source of truth for scope, coverage zones (radar ±60°, servo ±90°, mic 360°), tracking sub-modes, and out-of-range policy.

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

When the initial version works, publish to the user's GitHub as new repo **`ESP32_S3_CAM_Mic3_RD60_PMW1`** (confirm with the user before the first push).

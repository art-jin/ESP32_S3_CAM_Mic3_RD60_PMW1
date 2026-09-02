# HANDOFF: MG90S Servo Adaptation — Hardware Validation

**Status (2026-07-25)**: Code complete. Compiles cleanly under both JS6620 and MG90S configurations (verified at the object-file level). **Pending: hardware flash + 3-stage validation on the real robot.**

**Audience**: the agent who picks this up on the machine with ESP-IDF v6.0.1 + `wifi_creds.h` + physical access to the S3-Zero + MG90S robot. You have no prior context — read this whole doc before touching anything.

---

## 1. Background

The project originally targeted the **GOOUUU ESP32-S3-CAM + JS6620 (270° servo) + 15T/20T external spur gear** mechanical config. Every servo-related constant in the code was hardcoded for this: slope `2000 µs / 202.5°`, direction flag tied to "shaft-down + external mesh = two inversions cancel", boot sweep fixed at ±60°.

A fourth robot was built with a different mechanical config:
- **Waveshare ESP32-S3-Zero** (inside the rotating head)
- **MG90S 180° servo** (on the fixed base, shaft up)
- **Rigid direct drive** (servo horn → disc → robot head, no gears)
- **Battery powered**

When the existing JS6620 code was flashed to this robot, the boot sweep behavior was **reversed**: `0 → +60 → 0 → −60 → 0` produced head motion `6oc → 4oc → 6oc → 8oc → 6oc` (expected `6oc → 8oc → 6oc → 4oc → 6oc`). Cause is empirical — likely a clone MG90S with non-standard direction, or a disc-mount offset. Fix is software-only.

## 2. What was changed (code state)

A new `SERVO_MODEL_*` compile-time switch was added to `main/board_config.h`, decoupled from the existing `BOARD_*` switch. The servo mechanical/electrical params are now per-model.

**Files modified**:

| File | Change |
|---|---|
| `main/board_config.h` | +`SERVO_MODEL_*` section after `BOARD_*`. Two options: `JS6620_EXTERNAL_GEAR` (default) and `MG90S_DIRECT_DRIVE`. Mutually exclusive, both enforced via `#error`. |
| `main/servo.h` | Parameterized `SERVO_TRAVEL_DEG`, `SERVO_GEAR_RATIO`, `SERVO_DIRECTION_INVERTED`, `SERVO_ANGLE_MIN/MAX_DEG`, `SERVO_BOOT_SWEEP_DEG`. Renamed `SERVO_SHAFT_INSTALLED_DOWN` → `SERVO_DIRECTION_INVERTED` (old name implied mounting but really gated slope sign). |
| `main/servo.c` | `write_pwm_for_angle()` uses macro `2000 / (SERVO_TRAVEL_DEG / SERVO_GEAR_RATIO)` instead of hardcoded `2000/202.5`. `servo_boot_sweep()` uses `SERVO_BOOT_SWEEP_DEG` instead of hardcoded 60. |
| `main/tracker.c` | Both feed-forward branches (line ~226 non-conservative, line ~266 conservative) wrapped in `#if SERVO_DIRECTION_INVERTED` to flip sign for MG90S. |
| `CLAUDE.md` | New "Multi-servo support via board_config.h (v2.3)" section + version bump. |

**Per-model values**:

| Constant | JS6620 (default) | MG90S (new) |
|---|---|---|
| `SERVO_TRAVEL_DEG` | 270.0 | 180.0 |
| `SERVO_GEAR_RATIO` | 1.333 | 1.0 |
| `SERVO_DIRECTION_INVERTED` | 0 | 1 |
| `SERVO_ANGLE_MIN/MAX_DEG` | ±90° | ±90° |
| `SERVO_BOOT_SWEEP_DEG` | 60° | 80° |
| Slope (computed) | 9.877 µs/° at gear output | 11.11 µs/° at servo shaft |
| Feed-forward sign (non-conservative) | `α_array − β_servo` | `α_array + β_servo` |
| Feed-forward sign (conservative) | `sextant + β_servo − 180` | `sextant − β_servo − 180` |

**Backward compatibility**: JS6620 default config is **behaviorally identical to before the refactor** — the values didn't change, they just moved behind a macro. All existing JS6620/S3-CAM testing remains valid.

## 3. What was verified already (on the editing machine)

- ✅ JS6620 config: `servo.c.obj` and `tracker.c.obj` compile clean under ESP-IDF v5.5.4
- ✅ MG90S config: same object files compile clean after switching the `SERVO_MODEL` flag
- ❌ Full firmware build NOT completed — editing machine lacks v6.0.1 Python venv and `wifi_creds.h`. See §4.

## 4. Environment requirements (your machine)

The codebase requires:
- **ESP-IDF v6.0.1** (per `CLAUDE.md`). v5.5.4 fails on `mic_capture.c:39` — uses `I2S_PDM_RX_LINE0_SLOT_LEFT` which is v6.0.1-only.
- **`main/wifi_creds.h`** file (gitignored). Must define:
  ```c
  #pragma once
  static const char *WIFI_CREDS_SSID = "...";
  static const char *WIFI_CREDS_PASSWORD = "...";
  ```
- Physical access to the robot: S3-Zero + MG90S + 3DMIC-291 + battery.
- USB-CDC cable to `/dev/cu.usbmodem*` for flash + monitor.
- A phone/computer on the same WiFi for REST API testing.
- Optional: protractor + paper marker for V2 angle measurement. Or a camera + ImageJ.

## 5. Your mission

### Step 0: Pull and sanity-check

```bash
git pull                  # get the latest main with the v2.3 changes
git log --oneline -5      # confirm you see the v2.3 commit at HEAD
git diff HEAD~1 --stat    # should show 5 files: 4 main/* + CLAUDE.md + HANDOFF.md
```

Verify `main/board_config.h` has `SERVO_MODEL_JS6620_EXTERNAL_GEAR` enabled (default).

### Step 1: Build the default JS6620 config first

**Why**: confirm the refactor didn't break the existing config. If this fails, something in the refactor is wrong — report back, don't try to fix unilaterally.

```bash
idf.py build
```

Expect: success. If fails, capture the error log verbatim and report.

### Step 2: Switch to MG90S + S3-Zero and rebuild

Edit `main/board_config.h`:

```c
// #define BOARD_GOOUUU_S3_CAM    1
#define BOARD_WAVESHARE_S3_ZERO_M   1
// #define BOARD_ESP32_S3_SUPERMINI    1

// #define SERVO_MODEL_JS6620_EXTERNAL_GEAR   1
#define SERVO_MODEL_MG90S_DIRECT_DRIVE   1
```

```bash
idf.py build
```

Expect: success. If fails on `SERVO_MODEL_*` or `BOARD_*` `#error` directives, you got the mutually-exclusive selection wrong — re-read `board_config.h` comments.

### Step 3: Flash and capture boot log

```bash
idf.py -p /dev/cu.usbmodem21201 flash
idf.py -p /dev/cu.usbmodem21201 monitor
```

**Manual BOOT-button entry may be required** (per `CLAUDE.md` "Flashing" section): hold BOOT, press RST, release BOOT, then flash.

Capture the first ~30 seconds of UART log. Look for:
- Reset reason (no brownout)
- `evlog: === event log:` block (previous boot's events)
- `servo: boot sweep: 0 -> +80 -> 0 -> -80 -> 0` (note: **80, not 60**)
- WiFi connect + mDNS register
- `servo: init OK` line

### Step 4: V1 — Boot sweep visual check

During Step 3's boot, watch the head **from above**. Expected boot sweep behavior:

| Command | JS6620 (old, for reference) | MG90S (expected now) |
|---|---|---|
| `+80°` | n/a (was ±60°) | head moves to **8oc** (CW from home) |
| `0°` | head at 6oc home | head at 6oc home |
| `−80°` | n/a | head moves to **4oc** (CCW from home) |
| `0°` | head at 6oc home | head at 6oc home |

**Pass criteria**: `+80` takes head to 8oc (not 4oc), `−80` takes head to 4oc (not 8oc). I.e., direction is **reversed from what the robot was doing before this commit**.

**Fail modes**:
- If `+80` still takes head to 4oc (same as before): `SERVO_DIRECTION_INVERTED` not taking effect. Check `board_config.h` is saved, `idf.py build` actually rebuilt (`servo.c.obj` timestamp).
- If head moves much less than 80°: slope is off, proceed to V2 to measure.

### Step 5: V2 — REST angle measurement (open loop)

Get the device_id from the UART log (look for `device_id=XXXX` line, 6 chars `[A-Z0-9]`).

Switch to command mode:
```bash
DEV=XXXX   # your device id
curl -s -X POST "http://esp32-mic-<MAC4>.local/api/mode?device_id=$DEV" \
     -H 'Content-Type: application/json' \
     -d '{"mode":"command"}'
```

Run the angle sweep. For each command, before sending: mark the head's current position with a paper arrow on a piece of paper under the robot. After the head stops, mark the new position. Measure the angle between marks with a protractor. (Alternative: fix a phone directly above on a tripod, record video, measure frames in ImageJ.)

```bash
# Always return to home first to establish baseline
curl -s -X POST "http://esp32-mic-<MAC4>.local/api/point?device_id=$DEV" \
     -H 'Content-Type: application/json' -d '{"angle":0}'

# Test points
for a in 30 60 90 -30 -60 -90; do
  curl -s -X POST "http://esp32-mic-<MAC4>.local/api/point?device_id=$DEV" \
       -H 'Content-Type: application/json' -d "{\"angle\":$a}"
  sleep 2   # let it settle
  # MARK + MEASURE here
done
```

Record in a table:

| Commanded | Measured | Ratio (measured/commanded) | Buzzing? |
|---|---|---|---|
| +30° | ? | ? | y/n |
| +60° | ? | ? | y/n |
| +90° | ? | ? | y/n |
| −30° | ? | ? | y/n |
| −60° | ? | ? | y/n |
| −90° | ? | ? | y/n |

**Pass criteria**:
- All ratios in **[0.95, 1.05]** (±5% tolerance)
- Symmetry: `|+30|` and `|−30|` within 2° of each other; same for ±60, ±90
- No buzzing at any angle (buzzing = servo at end-of-travel straining)

**Action on failure** — **DO NOT TUNE UNILATERALLY**. Report the table. Tuning decisions (e.g., adjust `SERVO_TRAVEL_DEG` from 180 → 170) are made by the user after seeing the data.

### Step 6: V3 — Closed-loop DOA tracking

```bash
curl -s -X POST "http://esp32-mic-<MAC4>.local/api/mode?device_id=$DEV" \
     -H 'Content-Type: application/json' \
     -d '{"mode":"track"}'
```

Stand ~50cm from the mic array (close range — per CLAUDE.md "Environmental limitations", normal speech is reliable only at 30-50cm). Speak continuously for 10+ seconds at each position. Watch:
- The head's physical orientation (where M3 / the front points)
- The UART log's `servo=+XX°` and `az=XXX°` lines
- Or poll `/api/status?device_id=$DEV` and read `servo_angle` + `doa_azimuth`

**Test matrix**:

| User position | Expected servo | Expected DOA α (array frame, post feed-forward) |
|---|---|---|
| 6oc | ~0° | ~180° |
| 7oc | ~+30° | ~180° |
| 5oc | ~−30° | ~180° |
| 8oc | ~+60° | ~180° |
| 4oc | ~−60° | ~180° |

**Critical insight**: DOA α should return to **~180°** when the servo tracks correctly — feed-forward compensation makes the source "appear stationary in array frame" once the array has rotated to face the source. If α is far from 180° (e.g., 150° or 210°), feed-forward sign is wrong.

**Pass criteria**:
- Servo points within ±5° of expected at all 5 positions
- DOA α stays in [170°, 190°] at all 5 positions (feed-forward compensation working)
- No oscillation (servo doesn't dart back and forth)
- No brownout reset during 5+ minutes of continuous operation

**Action on failure** — **DO NOT FLIP SIGNS UNILATERALLY**. Report which positions failed and the actual servo + α values. The user will diagnose (could be feed-forward sign, could be calibration, could be Pitfall 6 oscillation).

### Step 7: evlog capture

After V3, reset the robot and capture the evlog block from UART (printed at every boot). Look for:
- `EV_BOOT` events with `value=9` (brownout) — should be 0 of these
- `EV_DOA_FIRST` with wild values (e.g., 254° toward blind spot) — grace period should prevent
- `EV_SERVO_CMD` sequence matching your test commands

Report the evlog block verbatim.

## 6. What to report back

Concise report with these sections:

```
## Environment
- ESP-IDF version: ...
- Board: ...
- Servo: ... (confirm MG90S, not SG90 or other)

## Build
- JS6620 config build: ✓/✗ (if ✗, error log)
- MG90S config build: ✓/✗ (if ✗, error log)

## V1 Boot sweep
- +80° → head at ___oc  (expected 8oc)
- −80° → head at ___oc  (expected 4oc)
- Pass/fail

## V2 Angle measurement
| Commanded | Measured | Ratio | Buzzing |
| +30° | ? | ? | ? |
... (all 6 rows)

## V3 DOA tracking
| User pos | Servo | DOA α | Expected servo | Expected α | OK? |
| 6oc | ? | ? | 0° | 180° | ? |
... (all 5 rows)
- Oscillation observed: y/n
- Brownout during 5min: y/n

## evlog
[paste evlog block]

## Anomalies / observations
[anything weird, free-form]
```

## 7. Hard rules — what NOT to do

- **Don't tune constants unilaterally.** If V2 ratio is 0.85, don't change `SERVO_TRAVEL_DEG` to fix it. Report the data; the user decides.
- **Don't flip `SERVO_DIRECTION_INVERTED` or feed-forward signs.** If direction is wrong, report it with observations. The user will diagnose root cause.
- **Don't commit or push changes from this machine.** Code changes go through the editing machine to keep the diff coherent.
- **Don't delete `HANDOFF.md`.** It's a temporary working doc — will be removed in a follow-up commit once validation is signed off.
- **Don't skip V1 to "save time".** V1 takes 10 seconds and rules out 50% of failure modes. V2/V3 results are misleading if V1 fails.

## 8. Tuning decision tree (informational — decisions made by user, not you)

For your reference, so you understand what the data will be used for:

| Observation | Likely cause | Likely fix (user decides) |
|---|---|---|
| V1 direction still wrong | `SERVO_DIRECTION_INVERTED` not applied | Verify build, check macro expansion |
| V2 ratio uniformly 0.9× | MG90S clone with shorter travel | `SERVO_TRAVEL_DEG` 180 → 170 |
| V2 ratio uniformly 1.1× | MG90S clone with longer travel | `SERVO_TRAVEL_DEG` 180 → 200 |
| V2 ±90° buzzing | Servo at mechanical hard limit | `SERVO_ANGLE_MAX_DEG` ±90 → ±85 (Pitfall 5) |
| V3 servo right direction, α wrong | Feed-forward sign mismatch | Flip `tracker.c:226/266` #if branches |
| V3 servo wrong direction | Direction flag still wrong | Flip `SERVO_DIRECTION_INVERTED` |
| V3 oscillation | Pitfall 6 closed-loop | Multiple causes — diagnose via evlog |
| Brownout | Battery can't supply MG90S inrush | `SERVO_BOOT_SWEEP_DEG` ±80 → ±60 |

## 9. References

- `CLAUDE.md` — project overview, especially "Multi-servo support via board_config.h (v2.3)" section added in this commit
- `main/board_config.h` — both `BOARD_*` and `SERVO_MODEL_*` switches
- `main/servo.h` — per-model constant definitions with full comments
- `main/servo.c:50-70` — `write_pwm_for_angle()` with slope computation
- `main/tracker.c:220-285` — feed-forward compensation, both branches
- CLAUDE.md "Pitfalls" section — especially Pitfall 5 (buzzing/L/R collapse) and Pitfall 6 (closed-loop oscillation)

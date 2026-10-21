#!/usr/bin/env python3
"""台架串口采集工具：3-2-1 倒计时 → "开始"横幅 → 秒级计时器 + 实时过滤行。
完整日志存 /tmp/capture_last.log（过滤词匹配的行同时实时打印）。

用法:
  python3 tools/capture.py [秒数=50] [过滤词=radar] [串口=/dev/cu.usbmodem21201]

床区标定模式 (docs/52 P0-2)：轮询 /api/status 收集可信距离样本，
每 10s 打印一次 100mm 直方图，结束给出建议床区窗（p2–p98）与
zone_enter 建议（窗上限+400 迟滞余量）。人保持目标姿势不动。

用法:
  python3 tools/capture.py --bed-cal [主机=192.168.1.113] [秒=120] [device_id]
"""
import json as _json
import re
import sys
import time
import urllib.request


def bed_cal(host, dur, did):
    url = f"http://{host}/api/status?device_id={did}"
    samples = []          # trusted range_mm only
    bins = {}             # 100mm bin → count
    t0 = time.time()
    print(f"\n━━━━━━━━ 床区标定 ━━━━━━━━  {time.strftime('%H:%M:%S')}  "
          f"{dur}s  {url}\n保持目标姿势不动…\n", flush=True)
    last_hist = 0
    while time.time() - t0 < dur:
        try:
            with urllib.request.urlopen(url, timeout=2) as r:
                d = _json.load(r)
            t = d.get("radar", {}).get("target", {})
            if t.get("valid") and t.get("rb_conf", 0) >= 12:
                mm = t["range_mm"]
                samples.append(mm)
                bins[(mm // 100) * 100] = bins.get((mm // 100) * 100, 0) + 1
        except Exception:
            pass
        el = time.time() - t0
        if el - last_hist >= 10:
            last_hist = el
            _hist(samples, bins, el)
        time.sleep(0.2)
    print(flush=True)
    _hist(samples, bins, time.time() - t0, final=True)
    if len(samples) < 50:
        print(f"⚠ 样本仅 {len(samples)} 条（可信帧不足）——延长时长或检查姿势",
              flush=True)
        return
    s = sorted(samples)
    lo, hi = s[max(0, len(s) * 2 // 100)], s[min(len(s) - 1, len(s) * 98 // 100)]
    lo100, hi100 = (lo // 100) * 100, ((hi + 99) // 100) * 100
    print(f"\n建议:  bed_min_mm={lo100}  bed_max_mm={hi100}  "
          f"zone_enter={min(hi100 + 400, 6000)}", flush=True)
    print(f"下发:  curl -X POST http://{host}/api/radar?device_id={did} "
          f"-H 'Content-Type: application/json' "
          f"-d '{{\"bed_min_mm\":{lo100},\"bed_max_mm\":{hi100}}}'", flush=True)


def _hist(samples, bins, el, final=False):
    if not bins:
        return
    lo, hi = min(bins), max(bins)
    peak = max(bins.values())
    print(f"--- {el:5.0f}s  n={len(samples)}  "
          f"{'FINAL' if final else ''}", flush=True)
    for b in range(lo, hi + 100, 100):
        c = bins.get(b, 0)
        bar = "#" * int(c * 40 / peak)
        print(f"  {b:>5}-{b + 99:<5} {c:>4} {bar}", flush=True)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "--bed-cal":
        host = sys.argv[2] if len(sys.argv) > 2 else "192.168.1.113"
        dur = int(sys.argv[3]) if len(sys.argv) > 3 else 120
        did = sys.argv[4] if len(sys.argv) > 4 else "QS0JZC"
        bed_cal(host, dur, did)
        sys.exit(0)

    import serial

    dur = int(sys.argv[1]) if len(sys.argv) > 1 else 50
    filt = sys.argv[2] if len(sys.argv) > 2 else "radar"
    port = sys.argv[3] if len(sys.argv) > 3 else "/dev/cu.usbmodem21201"
    pat = re.compile(filt)

    for i in (3, 2, 1):
        print(f"   {i}...", flush=True)
        time.sleep(1)

    t0 = time.time()
    print(f"\n━━━━━━━━━━ 开始 ━━━━━━━━━━  {time.strftime('%H:%M:%S')}  "
          f"采集 {dur}s  过滤: {filt}\n", flush=True)

    ser = serial.Serial(port, 115200, timeout=1)
    log = open("/tmp/capture_last.log", "w")
    last_tick = -1
    try:
        while True:
            el = time.time() - t0
            if el >= dur:
                break
            tick = int(el)
            if tick != last_tick:
                last_tick = tick
                print(f"[{tick:>3}s]", flush=True)
            line = ser.readline()
            if line:
                t = line.decode("utf-8", errors="replace")
                log.write(t)
                if pat.search(t):
                    print("   " + t.rstrip(), flush=True)
    finally:
        ser.close()
        log.close()
        print(f"\n━━━━━━━━━━ 结束 ━━━━━━━━━━  完整日志: /tmp/capture_last.log",
              flush=True)

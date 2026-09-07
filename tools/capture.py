#!/usr/bin/env python3
"""台架串口采集工具：3-2-1 倒计时 → "开始"横幅 → 秒级计时器 + 实时过滤行。
完整日志存 /tmp/capture_last.log（过滤词匹配的行同时实时打印）。

用法:
  python3 tools/capture.py [秒数=50] [过滤词=radar] [串口=/dev/cu.usbmodem21201]
"""
import re
import serial
import sys
import time

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

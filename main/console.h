#pragma once

/* Runtime UART console for servo/tracker tuning without reflashing.
 *
 * Console task reads lines from UART0 (the same USB-CDC channel ESP_LOG
 * uses) and dispatches commands. Output from ESP_LOG interleaves with
 * command echo — that's fine for debugging.
 *
 * Commands (case-insensitive):
 *   help                 Show command list
 *   status               Print servo / tracker / DOA snapshot
 *   servo <deg>          Manual: command servo to angle (clamped ±20°)
 *                        Also disables tracker (use 'tracker on' to resume)
 *   tracker <on|off>     Enable / disable auto-tracking
 *   cfg                  Print current tracker config
 *   cfg <key> <val>      Set a config value. Keys:
 *       home <deg>           Home offset added to target
 *       deadband <deg>       Min target change to trigger motion
 *       minconf <0..1>       Min 3-mic confidence to accept
 *       range <deg>          Out-of-range suppression threshold
 *       agree <deg>          Two-frame agreement tolerance
 *                        (config changes take effect immediately)
 *
 * Usage: drop the chip into bootloader mode normally, flash, run. Then
 * type commands in the same terminal you use for `idf.py monitor`. */
void console_init(void);

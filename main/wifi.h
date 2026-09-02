#pragma once
#include <stdbool.h>
#include <esp_err.h>

/* WiFi STA mode init. Reads credentials from wifi_creds.h (gitignored).
 * Non-blocking: returns immediately, connection happens in event callbacks.
 * On successful IP acquisition: starts mDNS + HTTP server + sets LED steady. */
esp_err_t wifi_init(void);

bool wifi_is_connected(void);
const char *wifi_get_ip_string(void);
const char *wifi_get_hostname(void);

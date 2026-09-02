#pragma once
#include <esp_err.h>

/* Start HTTP server with REST API handlers. Called once after WiFi connects. */
esp_err_t rest_api_start(void);

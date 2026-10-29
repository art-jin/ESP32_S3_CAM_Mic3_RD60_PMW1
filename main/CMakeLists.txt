idf_component_register(SRCS "main.c" "mic_capture.c" "doa.c" "servo.c" "tracker.c" "wifi.c" "rest_api.c" "mode_manager.c" "status.c" "evlog.c" "radar.c" "fusion.c" "events.c" "radar_probe.c" "mqtt_publisher.c" "zone_tracker.c" "cfg_channel.c" "display_mode.c" "epd.c" "epd_task.c" "epd_calendar.c" "epd_pages.c" "u8g2_font.c" "fonts.c" "lunar.c" "fonts_extra.c"
                    INCLUDE_DIRS "."
                    EMBED_FILES "index.html"
                    PRIV_REQUIRES cjson nvs_flash esp_timer esp_driver_gpio esp_driver_ledc esp_driver_i2s esp_driver_uart esp_driver_spi esp_driver_i2c esp_app_format esp-tls mqtt esp_wifi esp_netif esp_http_server mdns esp_event)

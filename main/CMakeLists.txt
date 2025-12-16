idf_component_register(
    SRCS
        "init.c"
        "bno055.c"
        "main.c"
        "as5600_lib.c"
        "bldc_pwm.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer esp_http_server esp_wifi esp_event esp_netif nvs_flash lwip
)

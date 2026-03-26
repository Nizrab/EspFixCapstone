idf_component_register(
    SRCS 
        "mqtt_client.c"
        "lib/mqtt_msg.c"
        "lib/mqtt_outbox.c"
        "lib/platform_esp32_idf.c"
    INCLUDE_DIRS "include" "lib/include"
    REQUIRES tcp_transport
    PRIV_REQUIRES esp-tls log esp_event esp_timer http_parser
)
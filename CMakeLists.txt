idf_component_register(
    SRCS "esp_wan_cli.cpp" "p1_reader.cpp"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio esp_netif nvs_flash
    REQUIRES mqtt
)
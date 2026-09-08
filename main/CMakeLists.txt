idf_component_register(SRCS "l431_link.c" "main.c" "xbox_ble.c" INCLUDE_DIRS "."
                       REQUIRES esp_wifi esp_event esp_netif nvs_flash lwip esp_driver_uart bt)

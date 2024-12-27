idf_component_register(SRCS "node_constants.c" "wifi_station_control.c" "svatky_api.c" "openweathermap_api.c" "sensor_control.c"
                        "sht4x.c" "sgp41.c" "sensirion_gas_index_algorithm.c" "bme280_i2c.c" "scd4x.c" "sps30_i2c.c"
                        "radio_control.c" "nrf24l01p.c"
                        "sntp_control.c" "display_control.cpp" "cjson_parsing.c" "main.cpp"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "svatkyapicz_cert.pem" "openweathermaporg_cert.pem")
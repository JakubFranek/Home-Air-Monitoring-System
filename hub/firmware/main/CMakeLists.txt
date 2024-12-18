idf_component_register(SRCS "svatky_api.c" "openweathermap_api.c" "wifi_sta.c" "sntp_api.c" "display_control.cpp" "cjson_parsing.c" "main.cpp"
                    INCLUDE_DIRS "."
                    EMBED_TXTFILES "svatkyapicz_cert.pem" "openweathermaporg_cert.pem")
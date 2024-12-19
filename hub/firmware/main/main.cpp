#include <stdio.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "display_control.h"
#include "wifi_station.h"
#include "sntp_api.h"
#include "svatky_api.h"
#include "openweathermap_api.h"
#include "sensor_control.h"

static const char *TAG = "main";

extern "C"
{
    void app_main(void);
}

static SvatkyApiData svatky_data;
static WeatherData weather_data;

uint8_t minute_counter = 0;

void app_main(void)
{
    setup_display();
    setup_wifi();
    initialize_sntp();
    request_svatkyapi_data(&svatky_data);
    request_weather_data(&weather_data);
    setup_i2c_bus();
    setup_sht4x();
    setup_sgp41();
    setup_bme280();
    setup_scd4x();

    while (true)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        measure_sgp41();
        measure_sht4x();
        measure_bme280();

        if (minute_counter % 5 == 0)
        {
            measure_scd4x(); // the SCD41 algorithm expects 5 minute sampling period
        }

        update_display();

        minute_counter++;

        xTaskDelayUntil(&xLastWakeTime, 60 * 1000 / portTICK_PERIOD_MS);
    }
}
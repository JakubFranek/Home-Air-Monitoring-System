#include <stdio.h>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "display_control.h"
#include "wifi_sta.h"
#include "sntp_api.h"
#include "svatky_api.h"
#include "openweathermap_api.h"

extern "C"
{
    void app_main(void);
}

static SvatkyApiData svatky_data;
static WeatherData weather_data;

void app_main(void)
{
    setup_display();
    setup_wifi();
    initialize_sntp();
    request_svatkyapi_data(&svatky_data);
    request_weather_data(&weather_data);

    while (true)
    {
        update_display();
        vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
    }
}
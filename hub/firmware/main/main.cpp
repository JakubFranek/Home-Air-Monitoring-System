#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "display_control.h"
#include "wifi_station.h"
#include "sntp_api.h"
#include "svatky_api.h"
#include "openweathermap_api.h"
#include "sensor_control.h"
#include "radio_control.h"

#define FAN_SWITCH_PIN GPIO_NUM_32
#define ESPINK_VSENSOR_ENA_PIN GPIO_NUM_2

static const char *TAG = "main";

extern "C"
{
    void app_main(void);
}

uint8_t minute_counter = 0;

struct timeval tv;
struct tm timeinfo;
char time_str[64];
uint64_t time_now_us;

static DisplayData display_data;

void app_main(void)
{
    gpio_reset_pin(FAN_SWITCH_PIN);
    gpio_set_direction(FAN_SWITCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(FAN_SWITCH_PIN, GPIO_FLOATING);

    gpio_set_direction(ESPINK_VSENSOR_ENA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(ESPINK_VSENSOR_ENA_PIN, GPIO_FLOATING);
    gpio_set_level(ESPINK_VSENSOR_ENA_PIN, 1);

    setup_display();

    setup_wifi(); // TODO: reconnect if connection lost
    initialize_sntp();

    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    request_svatkyapi_data(&display_data.svatky); // TODO: to be requested once per calendar day
    request_weather_data(&display_data.weather);  // TODO: to be requested once per hour

    setup_sensors();

    xTaskCreatePinnedToCore(task_nrf24_control, "nrf24_receive", 4096, NULL, 1, NULL, APP_CPU_NUM);

    while (true)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        gpio_set_level(FAN_SWITCH_PIN, 1);
        ESP_LOGI(TAG, "Fan ON");
        vTaskDelay(5000 / portTICK_PERIOD_MS);
        gpio_set_level(FAN_SWITCH_PIN, 0);
        ESP_LOGI(TAG, "Fan OFF");
        vTaskDelay(5000 / portTICK_PERIOD_MS);

        measure_sgp41();
        measure_sht4x();
        measure_bme280();
        measure_sps30();

        if (minute_counter % 5 == 0)
        {
            measure_scd4x(); // the SCD41 algorithm expects 5 minute sampling period
        }

        display_data.hub = sensor_hub_data;
        int8_t status = get_node_data(display_data.nodes);

        gettimeofday(&tv, NULL);                                              // Get the current time
        localtime_r(&tv.tv_sec, &timeinfo);                                   // Convert seconds to local time
        time_now_us = ((uint64_t)tv.tv_sec * 1000000) + (uint64_t)tv.tv_usec; // Get current time in microseconds
        strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", &timeinfo); // Format time
        time_str[strcspn(time_str, "\n")] = '\0';                             // Remove the newline (for ESP_LOGI call)
        ESP_LOGI(TAG, "Current time: %s.%06ld", time_str, tv.tv_usec);

        update_display(&display_data);

        minute_counter++;

        // TODO: consider some soft low power mode?

        xTaskDelayUntil(&xLastWakeTime, 60 * 1000 / portTICK_PERIOD_MS);
    }
}

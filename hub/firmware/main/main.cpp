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

int8_t status_code;

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
    gpio_set_level(FAN_SWITCH_PIN, 1);
    print_line("E-paper display initialized.\nFan turned on.\nInitializing Wi-Fi... ");
    status_code = setup_wifi();
    print_line("done (%d).\nInitializing SNTP... ", status_code);
    status_code = initialize_sntp();
    print_line("done (%d).\nInitializing timezone... ", status_code);
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    print_line("done.\n");

    print_line("Requesting data from svatkyapi.cz... ");
    status_code = request_svatkyapi_data(&display_data.svatky);
    print_line("done (%d).\nRequesting data from openweathermap.org... ", status_code);
    status_code = request_weather_data(&display_data.weather);
    print_line("done (%d).\n", status_code);

    print_line("Fan turned off.\n");
    gpio_set_level(FAN_SWITCH_PIN, 0);

    print_line("Initializing I2C bus... ");
    status_code = setup_i2c_bus();
    print_line("done (%d).\nInitializing SHT40... ", status_code);
    status_code = setup_sht4x();
    print_line("done (%d).\nInitializing SGP41... ", status_code);
    status_code = setup_sgp41();
    print_line("done (%d).\nInitializing BME280... ", status_code);
    status_code = setup_bme280();
    print_line("done (%d).\nInitializing SCD41 (cca 10 seconds)... ", status_code);
    status_code = setup_scd4x();
    print_line("done (%d).\nInitializing SPS30... ", status_code);
    status_code = setup_sps30();
    print_line("done (%d).\n", status_code);

    print_line("Starting nRF24L01+ receiver thread... ");
    xTaskCreatePinnedToCore(task_nrf24_control, "nrf24_receive", 4096, NULL, 1, NULL, APP_CPU_NUM);

    print_line("done.\nMaking SHT4x measurement... ");
    status_code = measure_sht4x();
    print_line("done (%d).\nMaking SGP41 measurement... ", status_code);
    status_code = measure_sgp41();
    print_line("done (%d).\nMaking BME280 measurement... ", status_code);
    status_code = measure_bme280();
    print_line("done (%d).\nMaking SPS30 measurement... ", status_code);
    status_code = measure_sps30();
    print_line("done (%d).\nMaking SCD41 measurement (cca 5 seconds)... ", status_code);
    status_code = measure_scd4x();
    print_line("done (%d).\nClearing screen and entering main loop...\n", status_code);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    clear_screen();
    display_data.hub = sensor_hub_data;
    int8_t status = get_node_data(display_data.nodes);
    update_display(&display_data);

    vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);

    while (true)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        gpio_set_level(FAN_SWITCH_PIN, 1);
        ESP_LOGI(TAG, "Fan ON");

        if (!is_wifi_connected())
        {
            ESP_LOGI(TAG, "Reconnecting to Wi-Fi...");
            setup_wifi();
        }

        gettimeofday(&tv, NULL);            // Get the current time
        localtime_r(&tv.tv_sec, &timeinfo); // Convert seconds to local time

        // If svatky data are from previous day, request them again
        struct tm svatky_timeinfo;
        localtime_r(&display_data.svatky.timestamp.tv_sec, &svatky_timeinfo);
        if (svatky_timeinfo.tm_mday != timeinfo.tm_mday)
        {
            ESP_LOGI(TAG, "Requesting new svatky data...");
            request_svatkyapi_data(&display_data.svatky);
        }

        if (minute_counter % 60 == 0)
        {
            request_weather_data(&display_data.weather); // update weather once per hour
        }

        xTaskDelayUntil(&xLastWakeTime, 5 * 1000 / portTICK_PERIOD_MS);
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

        update_display(&display_data);

        minute_counter++;

        xTaskDelayUntil(&xLastWakeTime, 60 * 1000 / portTICK_PERIOD_MS);
    }
}

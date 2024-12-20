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

#define LED_PIN GPIO_NUM_27
#define FAN_SWITCH_PIN GPIO_NUM_32
#define ESPINK_VSENSOR_ENA_PIN GPIO_NUM_2
#define BUTTON_DEBUG_PIN GPIO_NUM_36

static const char *TAG = "main";

extern "C"
{
    void app_main(void);
}

static uint8_t minute_counter = 0;

static struct timeval current_time;
static struct tm time_info;
static char time_str[64];

static int8_t status_code;

static DisplayData display_data;

static volatile bool debug_mode = false;

static TaskHandle_t task_handle_display_update;
static TaskHandle_t task_handle_nrf24_receive;
static TaskHandle_t task_handle_poll_button;

static void task_display_update(void *pvParameters);
static void task_poll_button(void *pvParameters);

void app_main(void)
{
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(LED_PIN, GPIO_FLOATING);
    gpio_set_level(LED_PIN, 0);

    gpio_reset_pin(FAN_SWITCH_PIN);
    gpio_set_direction(FAN_SWITCH_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(FAN_SWITCH_PIN, GPIO_FLOATING);

    gpio_set_direction(ESPINK_VSENSOR_ENA_PIN, GPIO_MODE_OUTPUT);
    gpio_set_pull_mode(ESPINK_VSENSOR_ENA_PIN, GPIO_FLOATING);
    gpio_set_level(ESPINK_VSENSOR_ENA_PIN, 1);

    ESP_ERROR_CHECK(gpio_set_direction(BUTTON_DEBUG_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(BUTTON_DEBUG_PIN, GPIO_FLOATING));

    setup_display();
    gpio_set_level(FAN_SWITCH_PIN, 1);
    print_line("E-paper display initialized.\nFan turned on.\nInitializing Wi-Fi... ");
    status_code = setup_wifi();
    print_line("done (%d).\nInitializing SNTP... ", status_code);
    status_code = initialize_sntp();
    print_line("done (%d).\nInitializing timezone... ", status_code);
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    gettimeofday(&display_data.start_time, NULL);
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

    print_line("Starting nRF24L01+ receiver task... ");
    xTaskCreatePinnedToCore(task_nrf24_control, "nrf24_receive", 4096, NULL, 1, &task_handle_nrf24_receive, APP_CPU_NUM);

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

    get_wifi_ap_record(&display_data);
    display_data.hub = sensor_hub_data;
    int8_t status = get_node_data(display_data.nodes);
    display_data.sntp_last_sync = sntp_last_sync;
    display_data.sntp_sync_count = sntp_sync_count;

    update_display(&display_data);

    xTaskCreatePinnedToCore(task_display_update, "display_update", 4096, NULL, 1, &task_handle_display_update, APP_CPU_NUM);
    xTaskCreatePinnedToCore(task_poll_button, "poll_button", 1024, NULL, 1, &task_handle_poll_button, APP_CPU_NUM);

    vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);

    while (true)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        gpio_set_level(FAN_SWITCH_PIN, 1);
        ESP_LOGI(TAG, "Fan ON"); // Turn on the fan for 5 seconds to forcefully refresh the air inside the enclosure

        if (!is_wifi_connected())
        {
            ESP_LOGI(TAG, "Reconnecting to Wi-Fi...");
            setup_wifi();
        }

        gettimeofday(&current_time, NULL);             // Get the current time
        localtime_r(&current_time.tv_sec, &time_info); // Convert seconds to local time

        // If svatky data or weather forecast are from previous day, request them again
        struct tm svatky_timeinfo;
        localtime_r(&display_data.svatky.timestamp.tv_sec, &svatky_timeinfo);
        if (svatky_timeinfo.tm_mday != time_info.tm_mday)
        {
            ESP_LOGI(TAG, "Requesting new svatky data...");
            request_svatkyapi_data(&display_data.svatky);
            request_weather_data(&display_data.weather);
        }

        if (display_data.weather.timestamp.tv_sec + 60 * 60 < current_time.tv_sec)
        {
            request_weather_data(&display_data.weather); // update weather at least once per hour
        }

        xTaskDelayUntil(&xLastWakeTime, 5 * 1000 / portTICK_PERIOD_MS);
        gpio_set_level(FAN_SWITCH_PIN, 0);
        ESP_LOGI(TAG, "Fan OFF");
        vTaskDelay(5000 / portTICK_PERIOD_MS); // Wait for 5 seconds after turning off the fan for sensors to stabilize

        measure_sht4x();
        measure_sgp41();
        measure_bme280();
        measure_sps30();
        if (display_data.hub.co2_timestamp.tv_sec + 5 * 60 < current_time.tv_sec)
        {
            measure_scd4x(); // the SCD41 algorithm expects 5 minute sampling period
        }

        // TODO: encapsulate the display data update code
        get_wifi_ap_record(&display_data);
        display_data.hub = sensor_hub_data;
        int8_t status = get_node_data(display_data.nodes);
        display_data.sntp_last_sync = sntp_last_sync;
        display_data.sntp_sync_count = sntp_sync_count;

        xTaskNotify(task_handle_display_update, 1, eSetValueWithOverwrite);

        xTaskDelayUntil(&xLastWakeTime, 60 * 1000 / portTICK_PERIOD_MS);
    }
}

static void task_display_update(void *pvParameters)
{
    uint32_t notification;
    while (true)
    {
        notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        if ((notification == 1) && !debug_mode)
        {
            update_display(&display_data);
        }
        else if (notification == 2 && debug_mode)
        {
            show_debug_info(&display_data);
        }
        else if (notification == 3 && !debug_mode)
        {
            clear_screen();
            update_display(&display_data);
        }
    }
}

static void task_poll_button(void *pvParameters)
{
    bool button_history[3] = {false, false, false};
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t notification;

    while (true)
    {
        // Poll button at 100 ms rate, if it is high for two cycles, toggle debug mode and send notification
        xLastWakeTime = xTaskGetTickCount();

        button_history[0] = button_history[1];
        button_history[1] = button_history[2];
        button_history[2] = gpio_get_level(BUTTON_DEBUG_PIN);
        if (button_history[0] == false && button_history[1] == true && button_history[2] == true)
        {
            debug_mode = !debug_mode;
            notification = (debug_mode) ? 2 : 3;
            xTaskNotify(task_handle_display_update, notification, eSetValueWithOverwrite);
        }
        xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    }
}
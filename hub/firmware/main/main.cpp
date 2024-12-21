#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

// Set log level to debug
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

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

#define NOTIFY_DISPLAY_UPDATE 1
#define NOTIFY_ENTER_DEBUG_MODE 2
#define NOTIFY_EXIT_DEBUG_MODE 3

#define WEATHER_UPDATE_PERIOD_S 60 * 60 // 1 hour
#define MAIN_LOOP_PERIOD_S 60           // 1 minute
#define CO2_MEASUREMENT_PERIOD_S 5 * 60 // 5 minutes
#define FAN_TOGGLE_PERIOD_S 5           // 5 seconds

static const char *TAG = "main";

extern "C"
{
    void app_main(void);
}

static struct timeval current_time;
static struct tm time_info;
static int8_t status_code;

SemaphoreHandle_t display_data_mutex;
static DisplayData display_data;

static bool debug_mode = false;

static TaskHandle_t task_handle_display_update;
static TaskHandle_t task_handle_nrf24_receive;
static TaskHandle_t task_handle_poll_button;

static void task_display_update(void *pvParameters);
static void task_poll_button(void *pvParameters);
static void update_display_data(void);
static void take_display_data_mutex(void);
static void give_display_data_mutex(void);

/**
 * @brief Main entry point of the application.
 *
 * Initializes the hardware, sets up the tasks and enters the main loop.
 */
void app_main(void)
{
    display_data_mutex = xSemaphoreCreateMutex();

    take_display_data_mutex();

    ESP_ERROR_CHECK(gpio_reset_pin(LED_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(LED_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));

    ESP_ERROR_CHECK(gpio_reset_pin(FAN_SWITCH_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(FAN_SWITCH_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(FAN_SWITCH_PIN, GPIO_FLOATING));

    ESP_ERROR_CHECK(gpio_set_direction(ESPINK_VSENSOR_ENA_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(ESPINK_VSENSOR_ENA_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(ESPINK_VSENSOR_ENA_PIN, 1));

    ESP_ERROR_CHECK(gpio_set_direction(BUTTON_DEBUG_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(BUTTON_DEBUG_PIN, GPIO_FLOATING));

    setup_display();
    ESP_ERROR_CHECK(gpio_set_level(FAN_SWITCH_PIN, 1));
    print_line("E-paper display initialized.\nFan turned on.\nInitializing Wi-Fi... ");
    status_code = setup_wifi();
    if (status_code == 0) // If connection is successful
    {
        display_data.wifi_connection_count++;
    }
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
    ESP_ERROR_CHECK(gpio_set_level(FAN_SWITCH_PIN, 0));

    print_line("Starting nRF24L01+ receiver task... ");
    xTaskCreatePinnedToCore(task_nrf24_control, "nrf24_receive", 4096, NULL, 1, &task_handle_nrf24_receive, APP_CPU_NUM);

    print_line("Initializing I2C bus... ");
    status_code = setup_i2c_bus();
    print_line("done (%d).\nInitializing SHT40... ", status_code);
    status_code = setup_sht4x();
    print_line("done (%d).\nInitializing SGP41 (cca 10 s)... ", status_code);
    status_code = setup_sgp41();
    print_line("done (%d).\nInitializing BME280... ", status_code);
    status_code = setup_bme280();
    print_line("done (%d).\nInitializing SCD41 (cca 10 s)... ", status_code);
    status_code = setup_scd4x();
    print_line("done (%d).\nInitializing SPS30... ", status_code);
    status_code = setup_sps30();
    print_line("done (%d).\nMaking SHT4x measurement... ", status_code);
    status_code = measure_sht4x();
    print_line("done (%d).\nMaking SGP41 measurement... ", status_code);
    status_code = measure_sgp41();
    print_line("done (%d).\nMaking BME280 measurement... ", status_code);
    status_code = measure_bme280();
    print_line("done (%d).\nMaking SPS30 measurement... ", status_code);
    status_code = measure_sps30();
    print_line("done (%d).\nMaking SCD41 measurement (cca 5 s)... ", status_code);
    status_code = measure_scd4x();
    print_line("done (%d).\nClearing screen and entering main loop...\n", status_code);

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second to allow user to read the last message

    clear_screen();
    update_display_data();
    update_display(&display_data);

    give_display_data_mutex();

    xTaskCreatePinnedToCore(task_display_update, "display_update", 4096, NULL, 1, &task_handle_display_update, APP_CPU_NUM);
    xTaskCreatePinnedToCore(task_poll_button, "poll_button", 1024, NULL, 1, &task_handle_poll_button, APP_CPU_NUM);

    vTaskDelay(60 * 1000 / portTICK_PERIOD_MS); // Wait for 60 seconds to maintain cca 1 minute interval between measurement/update cycles

    while (true)
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        take_display_data_mutex();

        ESP_ERROR_CHECK(gpio_set_level(FAN_SWITCH_PIN, 1));
        ESP_LOGI(TAG, "Fan ON"); // Turn on the fan for 5 seconds to forcefully refresh the air inside the enclosure

        if (!is_wifi_connected())
        {
            ESP_LOGI(TAG, "Attempting to connect to Wi-Fi...");
            if (connect_wifi() == 0) // If connection was successful
            {
                display_data.wifi_connection_count++;

                synchronize_time();

                // If svatky data has not been successfully initialized yet, request immediately
                if (display_data.svatky.timestamp.tv_sec == 0)
                {
                    request_svatkyapi_data(&display_data.svatky);
                }

                // If weather data has not been successfully initialized yet, request immediately
                if (display_data.weather.timestamp.tv_sec == 0)
                {
                    request_weather_data(&display_data.weather);
                }
            }
        }

        gettimeofday(&current_time, NULL);             // Get the current time
        localtime_r(&current_time.tv_sec, &time_info); // Convert seconds to local time

        // If svatky data or weather forecast are from previous day, request them again
        struct tm svatky_timeinfo;
        localtime_r(&display_data.svatky.timestamp.tv_sec, &svatky_timeinfo);
        if (svatky_timeinfo.tm_mday != time_info.tm_mday)
        {
            request_svatkyapi_data(&display_data.svatky);
            request_weather_data(&display_data.weather);
        }

        if (display_data.weather.timestamp.tv_sec + WEATHER_UPDATE_PERIOD_S < current_time.tv_sec)
        {
            request_weather_data(&display_data.weather); // update weather at least once per hour
        }
        give_display_data_mutex();

        xTaskDelayUntil(&xLastWakeTime, FAN_TOGGLE_PERIOD_S * 1000 / portTICK_PERIOD_MS);

        ESP_ERROR_CHECK(gpio_set_level(FAN_SWITCH_PIN, 0));
        ESP_LOGI(TAG, "Fan OFF");
        vTaskDelay(FAN_TOGGLE_PERIOD_S * 1000 / portTICK_PERIOD_MS); // Wait for 5 seconds after turning off the fan for sensors to stabilize

        measure_sht4x();
        measure_sgp41();
        measure_bme280();
        measure_sps30();

        take_display_data_mutex();

        if (display_data.hub.co2_timestamp.tv_sec + CO2_MEASUREMENT_PERIOD_S < current_time.tv_sec)
        {
            measure_scd4x(); // the SCD41 algorithm expects 5 minute sampling period
        }
        update_display_data();

        give_display_data_mutex();

        xTaskNotify(task_handle_display_update, NOTIFY_DISPLAY_UPDATE, eSetValueWithOverwrite);

        xTaskDelayUntil(&xLastWakeTime, MAIN_LOOP_PERIOD_S * 1000 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Update the display with current data or switch to debug mode.
 *
 * This task waits for notifications from other tasks to update the display
 * with the current data or switch to debug mode. The notifications are
 * `NOTIFY_DISPLAY_UPDATE`, `NOTIFY_ENTER_DEBUG_MODE` and `NOTIFY_EXIT_DEBUG_MODE`.
 *
 * @param pvParameters Unused
 */
static void task_display_update(void *pvParameters)
{
    uint32_t notification;

    while (true)
    {
        notification = ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a notification from other tasks

        if ((notification == NOTIFY_DISPLAY_UPDATE) && !debug_mode)
        {
            take_display_data_mutex();

            update_display(&display_data);

            give_display_data_mutex();
        }
        else if (notification == NOTIFY_ENTER_DEBUG_MODE && debug_mode)
        {
            take_display_data_mutex();

            update_display_data();
            show_debug_info(&display_data);

            give_display_data_mutex();
        }
        else if (notification == NOTIFY_EXIT_DEBUG_MODE && !debug_mode)
        {
            take_display_data_mutex();

            clear_screen();
            update_display_data();
            update_display(&display_data);

            give_display_data_mutex();
        }
    }
}

/**
 * @brief Poll the debug button and switch between display debug mode and normal mode.
 *
 * This task monitors the debug button and if it is pressed, it switches between
 * debug mode and normal mode of the display.
 */
static void task_poll_button(void *pvParameters)
{
    bool button_history[3] = {false, false, false};
    TickType_t xLastWakeTime;
    uint32_t notification;

    while (true)
    {
        xLastWakeTime = xTaskGetTickCount();

        button_history[0] = button_history[1];
        button_history[1] = button_history[2];
        button_history[2] = gpio_get_level(BUTTON_DEBUG_PIN);
        if (button_history[0] == false && button_history[1] == true && button_history[2] == true)
        {
            debug_mode = !debug_mode;
            notification = (debug_mode) ? NOTIFY_ENTER_DEBUG_MODE : NOTIFY_EXIT_DEBUG_MODE;
            xTaskNotify(task_handle_display_update, notification, eSetValueWithOverwrite);
        }

        xTaskDelayUntil(&xLastWakeTime, 100 / portTICK_PERIOD_MS);
    }
}

/**
 * @brief Updates the display data structure with the latest information.
 *
 * This function fetches the latest Wi-Fi access point record and sensor hub data,
 * updates the node data, and synchronizes the SNTP last sync time and sync count.
 */

static void update_display_data(void)
{
    get_wifi_ap_record(&display_data);
    display_data.hub = sensor_hub_data;
    display_data.app_status = get_node_data(display_data.nodes);
    display_data.sntp_last_sync = sntp_last_sync;
    display_data.sntp_sync_count = sntp_sync_count;
}

static void take_display_data_mutex(void)
{
    if (xSemaphoreTake(display_data_mutex, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to take display_data_mutex, task = %s", pcTaskGetName(xTaskGetCurrentTaskHandle()));
        abort();
    }
    else
    {
        ESP_LOGD(TAG, "Taken display_data_mutex, task = %s", pcTaskGetName(xTaskGetCurrentTaskHandle()));
    }
}

static void give_display_data_mutex(void)
{
    ESP_LOGD(TAG, "Given display_data_mutex, task = %s", pcTaskGetName(xTaskGetCurrentTaskHandle()));
    xSemaphoreGive(display_data_mutex);
}
#include <stdio.h>
#include <time.h>
#include <sys/time.h>
#include <string.h>

#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"

#include "hams_defines.h"
#include "display_control.h"
#include "wifi_station_control.h"
#include "sntp_control.h"
#include "sensor_control.h"
#include "radio_control.h"
#include "svatky_api.h"
#include "openweathermap_api.h"

#define LED_PIN GPIO_NUM_27
#define FAN_SWITCH_PIN GPIO_NUM_32
#define ESPINK_VSENSOR_ENA_PIN GPIO_NUM_2
#define BUTTON_DEBUG_PIN GPIO_NUM_36

#define NOTIFY_DISPLAY_UPDATE 1
#define NOTIFY_ENTER_DEBUG_MODE 2
#define NOTIFY_EXIT_DEBUG_MODE 3

extern "C"
{
    void app_main(void);
}

static const char *TAG = "main";

static int8_t status_code;

static SemaphoreHandle_t hams_data_mutex;
static HamsData hams_data;

static volatile bool debug_mode = false;
static volatile bool co2_schedule_calibration = false;

static TaskHandle_t task_handle_display_update;
static TaskHandle_t task_handle_nrf24_receive;
static TaskHandle_t task_handle_poll_button;

static void task_display_update(void *pvParameters);
static void task_poll_button(void *pvParameters);

static void update_display_data(void);
static void take_hams_data_mutex(void);
static void give_hams_data_mutex(void);

static void set_fan_state(bool on);
static void initialize_gpio(void);

/**
 * @brief Main entry point of the application.
 *
 * Initializes the hardware, sets up the tasks and enters the main loop.
 */
void app_main(void)
{
    hams_data_mutex = xSemaphoreCreateMutex();

    take_hams_data_mutex();

    initialize_gpio();
    setup_display();

    set_fan_state(true);

    print_line("GPIO & e-paper display initialized.\nFan turned on.\nInitializing Wi-Fi... ");
    status_code = setup_wifi();
    print_line("done (%d).\nInitializing SNTP... ", status_code);
    status_code = initialize_sntp();
    print_line("done (%d).\nInitializing timezone... ", status_code);
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
    gettimeofday(&hams_data.debug.start_time, NULL);
    print_line("done.\n");

    print_line("Requesting data from svatkyapi.cz... ");
    status_code = request_calendar_data(&hams_data.calendar);
    print_line("done (%d).\nRequesting data from openweathermap.org... ", status_code);
    status_code = request_weather_data(&hams_data.weather);
    print_line("done (%d).\n", status_code);

    set_fan_state(false);
    print_line("Fan turned off.\n");

    print_line("Starting nRF24L01+ receiver task...");
    status_code = xTaskCreatePinnedToCore(task_nrf24_control, "nrf24_receive", 4096, NULL, 1, &task_handle_nrf24_receive, PRO_CPU_NUM);
    print_line("done (%d).\n", !status_code); // success = pdTRUE = 1, but HAMS convention is success = 0

    setup_sensors(true, &print_line);
    measure_sensors(true, &print_line);

    print_line("Clearing screen and entering main loop...\n");

    vTaskDelay(1000 / portTICK_PERIOD_MS); // Wait for 1 second to allow user to read the last message

    clear_screen();
    update_display_data();
    update_display(&hams_data);

    give_hams_data_mutex();

    xTaskCreatePinnedToCore(task_display_update, "display_update", 4096, NULL, 1, &task_handle_display_update, APP_CPU_NUM);
    xTaskCreatePinnedToCore(task_poll_button, "poll_button", 4096, NULL, 1, &task_handle_poll_button, APP_CPU_NUM);

    vTaskDelay(MAIN_LOOP_PERIOD_S * 1000 / portTICK_PERIOD_MS); // Wait to maintain interval between measurement/update cycles

    while (true) // Main loop
    {
        TickType_t xLastWakeTime = xTaskGetTickCount();

        set_fan_state(true);

        take_hams_data_mutex();

        if (!is_wifi_connected() && connect_wifi() == 0) // If Wi-Fi is not connected and connection is successful
        {
            synchronize_time(); // Forcefully synchronize time
        }

        request_calendar_data(&hams_data.calendar);
        request_weather_data(&hams_data.weather);

        give_hams_data_mutex();

        xTaskDelayUntil(&xLastWakeTime, FAN_TOGGLE_PERIOD_S * 1000 / portTICK_PERIOD_MS); // Wait until the fan period elapses
        set_fan_state(false);

        vTaskDelay(FAN_TOGGLE_PERIOD_S * 1000 / portTICK_PERIOD_MS); // Wait for 5 seconds after turning off the fan for sensors to stabilize

        if (co2_schedule_calibration)
        {
            schedule_co2_correction();
            co2_schedule_calibration = false;
        }
        measure_sensors(false, NULL); // Measure sensors without printing to the display

        take_hams_data_mutex();
        update_display_data();
        give_hams_data_mutex();

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
            ESP_LOGI(TAG, "NOTIFY_DISPLAY_UPDATE");
            take_hams_data_mutex();

            update_display(&hams_data);

            give_hams_data_mutex();
        }
        else if (notification == NOTIFY_ENTER_DEBUG_MODE && debug_mode)
        {
            ESP_LOGI(TAG, "NOTIFY_ENTER_DEBUG_MODE");
            take_hams_data_mutex();

            update_display_data();
            show_debug_info(&hams_data);

            give_hams_data_mutex();
        }
        else if (notification == NOTIFY_EXIT_DEBUG_MODE && !debug_mode)
        {
            ESP_LOGI(TAG, "NOTIFY_EXIT_DEBUG_MODE");
            take_hams_data_mutex();

            clear_screen();
            update_display_data();
            update_display(&hams_data);

            give_hams_data_mutex();
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
    bool button_pressed = false;
    TickType_t t_loop = 0, t_button_press = 0, t_button_release = 0;
    uint32_t notification;

    while (true)
    {
        t_loop = xTaskGetTickCount();

        button_history[0] = button_history[1];
        button_history[1] = button_history[2];
        button_history[2] = !gpio_get_level(BUTTON_DEBUG_PIN);

        if (button_history[0] == false && button_history[1] == true && button_history[2] == true)
        {
            t_button_press = xTaskGetTickCount();
            button_pressed = true;
            ESP_LOGI(TAG, "Debug button pressed, ticks = %ld", t_button_press);
        }
        else if (button_pressed && button_history[0] == true && button_history[1] == false && button_history[2] == false)
        {
            t_button_release = xTaskGetTickCount();
            button_pressed = false;
            ESP_LOGI(TAG, "Debug button released, ticks = %ld", t_button_release);

            if ((t_button_release - t_button_press) < 2000 / portTICK_PERIOD_MS)
            {
                ESP_LOGI(TAG, "Debug button short press (%ld ticks), toggling debug mode", t_button_release - t_button_press);
                debug_mode = !debug_mode;
                notification = (debug_mode) ? NOTIFY_ENTER_DEBUG_MODE : NOTIFY_EXIT_DEBUG_MODE;
                xTaskNotify(task_handle_display_update, notification, eSetValueWithOverwrite);
            }
            else
            {
                ESP_LOGI(TAG, "Debug button long press (%ld ticks), scheduling SCD41 calibration", t_button_release - t_button_press);
                co2_schedule_calibration = true;
            }
        }

        xTaskDelayUntil(&t_loop, 100 / portTICK_PERIOD_MS); // Poll at 10 Hz
    }
}

/**
 * @brief Updates the display data structure with the latest information.
 *
 * This function fetches the latest Wi-Fi access point record and sensor hub data,
 * updates the node data, and synchronizes the SNTP last sync time and sync count.
 *
 * @warning This function is not thread-safe, taking the `hams_data_mutex` before
 * calling this is required.
 */
static void update_display_data(void)
{
    int8_t status = 0;

    // If any of the function calls returns any number other than zero, the status will be set to that number

    status = (get_wifi_ap_record(&hams_data.debug) || status);
    status = (get_sensor_data(&hams_data.hub_sensors) || status);
    status = (get_node_data(hams_data.nodes) || status);
    status = (get_sntp_stats(&hams_data.debug.sntp_last_sync, &hams_data.debug.sntp_sync_count) || status);

    hams_data.debug.app_status = status;
}

/**
 * @brief Tries to take the hams_data_mutex semaphore.
 */
static void take_hams_data_mutex(void)
{
    if (xSemaphoreTake(hams_data_mutex, portMAX_DELAY) != pdPASS)
    {
        ESP_LOGE(TAG, "Failed to take hams_data_mutex, task = %s", pcTaskGetName(xTaskGetCurrentTaskHandle()));
        abort();
    }
    else
    {
        ESP_LOGD(TAG, "Taken hams_data_mutex, task = %s", pcTaskGetName(xTaskGetCurrentTaskHandle()));
    }
}

/**
 * @brief Releases the hams_data_mutex semaphore
 */
static void give_hams_data_mutex(void)
{
    ESP_LOGD(TAG, "Given hams_data_mutex, task = %s", pcTaskGetName(xTaskGetCurrentTaskHandle()));
    xSemaphoreGive(hams_data_mutex);
}

/**
 * @brief Set the state of the fan.
 *
 * @param on `true` to turn the fan on, `false` to turn it off.
 */
static void set_fan_state(bool on)
{
    ESP_ERROR_CHECK(gpio_set_level(FAN_SWITCH_PIN, (uint32_t)on));
    ESP_LOGI(TAG, "Fan turned %s", on ? "on" : "off");
}

/**
 * @brief Initializes the GPIO pins.
 *
 * Sets the GPIO pins to the following modes:
 *
 *  - `LED_PIN`: Output, floating pull mode, initially set to 0
 *
 *  - `FAN_SWITCH_PIN`: Output, floating pull mode, initially set to 0
 *
 *  - `ESPINK_VSENSOR_ENA_PIN`: Output, floating pull mode, initially set to 1
 *
 *  - `BUTTON_DEBUG_PIN`: Input, floating pull mode
 */
static void initialize_gpio(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(LED_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(LED_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(LED_PIN, 0));

    ESP_ERROR_CHECK(gpio_reset_pin(FAN_SWITCH_PIN));
    ESP_ERROR_CHECK(gpio_set_direction(FAN_SWITCH_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(FAN_SWITCH_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(FAN_SWITCH_PIN, 0));

    ESP_ERROR_CHECK(gpio_set_direction(ESPINK_VSENSOR_ENA_PIN, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(ESPINK_VSENSOR_ENA_PIN, GPIO_FLOATING));
    ESP_ERROR_CHECK(gpio_set_level(ESPINK_VSENSOR_ENA_PIN, 1));

    ESP_ERROR_CHECK(gpio_set_direction(BUTTON_DEBUG_PIN, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(BUTTON_DEBUG_PIN, GPIO_FLOATING));
}
/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sntp_control.h"

#include <string.h>
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "esp_netif_sntp.h"
#include "lwip/ip_addr.h"
#include "esp_sntp.h"

#define CONFIG_SNTP_TIME_SERVER "0.cz.pool.ntp.org"
#define SNTP_RETRY_COUNT 10

static const char *TAG = "sntp_control";

static struct timeval sntp_last_sync;
static uint32_t sntp_sync_count = 0;
static struct tm timeinfo;

static int8_t wait_for_sync(void);

/**
 * @brief Time synchronization callback function.
 *
 * This function is called after a successful synchronization with an NTP server.
 * The current time is passed as a parameter.
 *
 * @param tv Pointer to the current time.
 */
void time_sync_notification_callback(struct timeval *tv)
{
    gettimeofday(&sntp_last_sync, NULL);
    localtime_r(&sntp_last_sync.tv_sec, &timeinfo);

    sntp_sync_count++;

    ESP_LOGI(TAG, "Time synchronized with NTP server at %02d:%02d:%02d (count = %ld)",
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, sntp_sync_count);
}

/**
 * @brief Initializes and starts the Simple Network Time Protocol (SNTP)
 *
 * This function will perform the following steps:
 *   1. Initialize the SNTP client
 *   2. Set the time synchronization callback function
 *   3. Start the SNTP client
 *   4. Wait for the system time to be set. If it does not receive a valid time
 *      within 15 attempts, it will return -1.
 *
 * @return 0 on success, -1 if the system time could not be set within 15 attempts.
 */
int8_t initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing and starting SNTP...");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);

    config.sync_cb = time_sync_notification_callback; // Note: this is an optional callback

    esp_netif_sntp_init(&config);

    return wait_for_sync();
}

/**
 * @brief Synchronizes the system time using SNTP
 *
 * This function "manually" forces the system time to be synchronized with the NTP server.
 *
 * @return 0 on success, -1 if the system time could not be set within 15 attempts
 */
int8_t synchronize_time(void)
{
    ESP_LOGI(TAG, "Attempting to synchronize time via SNTP...");
    esp_netif_sntp_start();
    return wait_for_sync();
}

/**
 * @brief Waits for the system time to be synchronized.
 *
 * This function repeatedly checks if the system time has been set by the SNTP client.
 * It logs the progress and continues to retry until the system time is set or the maximum
 * number of retries is reached.
 *
 * @return 0 on success when the system time is set, -1 if the system time could not be set
 * within the defined number of retry attempts.
 */
static int8_t wait_for_sync(void)
{
    int retry = 0;
    while (true)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, SNTP_RETRY_COUNT);
        esp_err_t err = esp_netif_sntp_sync_wait(1000 / portTICK_PERIOD_MS);

        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "System time is set");
            break;
        }
        else if (retry == SNTP_RETRY_COUNT)
        {
            ESP_LOGE(TAG, "System time could not be set");
            return -1;
        }
        retry++;
    }

    return 0;
}

/**
 * @brief Get the last SNTP synchronization time and the number of successful synchronizations
 *
 * @param[out] last_sync Pointer to a `struct timeval` to store the last synchronization time
 * @param[out] sync_count Pointer to an `uint32_t` to store the number of successful synchronizations
 *
 * @return 0 on success, -1 if either of the pointers is `NULL`
 */
int8_t get_sntp_stats(struct timeval *last_sync, uint32_t *sync_count)
{
    if (last_sync == NULL || sync_count == NULL)
    {
        return -1;
    }

    *last_sync = sntp_last_sync;
    *sync_count = sntp_sync_count;

    return 0;
}
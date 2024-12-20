/* LwIP SNTP example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "sntp_api.h"

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

static const char *TAG = "sntp_api";

/* --- Public variables --- */
struct timeval sntp_last_sync;
uint32_t sntp_sync_count = 0;

static struct tm timeinfo;

/**
 * @brief Time synchronization callback function.
 *
 * This function is called after a successful synchronization with an NTP server.
 * The current time is passed as a parameter.
 *
 * @param tv Pointer to the current time.
 */
void time_sync_notification_cb(struct timeval *tv)
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
    ESP_LOGI(TAG, "Initializing and starting SNTP");

    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG(CONFIG_SNTP_TIME_SERVER);

    config.sync_cb = time_sync_notification_cb; // Note: This is only needed if we want

    esp_netif_sntp_init(&config);

    int retry = 0;
    const int retry_count = 15;
    while (++retry < retry_count)
    {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        esp_err_t err = esp_netif_sntp_sync_wait(2000 / portTICK_PERIOD_MS);

        if (err == ESP_OK)
        {
            break;
        }
        else if (err == ESP_ERR_TIMEOUT && retry == retry_count)
        {
            return -1;
        }
    }

    return 0;
}

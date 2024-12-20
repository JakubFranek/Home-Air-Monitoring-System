#include "wifi_station.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "nvs_flash.h"

#include "secrets.h" // WiFi SSID and Password macros

#define CONFIG_ESP_MAXIMUM_RETRY 5

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT BIT1

static const char *TAG = "wifi station";

static int s_retry_num = 0;

/**
 * @brief WiFi event handler.
 *
 * Handles the following events:
 *  - `WIFI_EVENT_STA_START`: Triggered when the Wi-Fi station starts.
 *    In response, attempt to connect to the AP.
 *  - `WIFI_EVENT_STA_DISCONNECTED`: Triggered when the Wi-Fi station disconnects
 *    from the AP. Attempt to reconnect to the AP up to the maximum number of
 *    retries. If the maximum number of retries is exceeded, set the WIFI_FAIL_BIT
 *    in the event group.
 *  - `IP_EVENT_STA_GOT_IP`: Triggered when the Wi-Fi station gets an IP address.
 *    Reset the retry counter and set the WIFI_CONNECTED_BIT in the event group.
 */
static void event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        if (s_retry_num < CONFIG_ESP_MAXIMUM_RETRY)
        {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "Retrying connection to the AP");
        }
        else
        {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG, "Connection to the AP failed");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Received IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

/**
 * @brief Initialize Wi-Fi as a station and connect to the AP.
 *
 * @return void
 *
 * @details This function initializes the Wi-Fi as a station and
 * connects to the AP. The function will block until the station is
 * connected to the AP or the maximum number of retries is reached.
 *
 * If the connection is successful, the function will return and the
 * bits WIFI_CONNECTED_BIT will be set in the event group. If the
 * maximum number of retries is reached, the function will return and
 * the bits WIFI_FAIL_BIT will be set in the event group.
 */
void wifi_init_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = CONFIG_ESP_WIFI_SSID,
            .password = CONFIG_ESP_WIFI_PASSWORD,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
            .sae_h2e_identifier = "",
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
     * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
                                           WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
                                           pdFALSE,
                                           pdFALSE,
                                           portMAX_DELAY);

    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
     * happened. */
    if (bits & WIFI_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "Connected to AP SSID: %s ", CONFIG_ESP_WIFI_SSID);
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGI(TAG, "Failed to connect to SSID: %s", CONFIG_ESP_WIFI_SSID);
    }
    else
    {
        ESP_LOGE(TAG, "Unexpected event");
    }
}

/**
 * @brief Initializes the WiFi station mode by setting up non-volatile storage,
 * network interface and event loop, and attempts to connect to a WiFi network.
 *
 * @return `int8_t` Returns 0 on successful connection to the WiFi network,
 * otherwise returns -1 if the connection could not be established.
 */
int8_t setup_wifi(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_sta();

    if (!is_wifi_connected())
    {
        return -1;
    }
    return 0;
}

/**
 * @brief Check if WiFi is connected.
 *
 * @return `true` if WiFi is connected, `false` otherwise
 */
bool is_wifi_connected(void)
{
    return (xEventGroupGetBits(s_wifi_event_group) & WIFI_CONNECTED_BIT) & !(xEventGroupGetBits(s_wifi_event_group) & WIFI_FAIL_BIT);
}

void get_wifi_ap_record(DisplayData *data)
{
    wifi_ap_record_t ap_record;
    esp_err_t err = esp_wifi_sta_get_ap_info(&ap_record);

    if (err == ESP_OK)
    {
        strncpy(data->wifi_status, "connected", sizeof(data->wifi_status));
        strncpy(data->wifi_ssid, (char *)ap_record.ssid, sizeof(data->wifi_ssid));
        data->wifi_rssi = ap_record.rssi;
    }
    else if (err == ESP_ERR_WIFI_CONN)
    {
        strncpy(data->wifi_status, "uninitialized", sizeof(data->wifi_status));
        strncpy(data->wifi_ssid, "--", sizeof(data->wifi_ssid));
        data->wifi_rssi = 0;
    }
    else if (err == ESP_ERR_WIFI_NOT_CONNECT)
    {
        strncpy(data->wifi_status, "disconnected", sizeof(data->wifi_status));
        strncpy(data->wifi_ssid, "--", sizeof(data->wifi_ssid));
        data->wifi_rssi = 0;
    }
}
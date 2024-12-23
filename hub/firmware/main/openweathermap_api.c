#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/param.h> // includes MIN macro

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include "esp_event.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "cJSON.h"

#include "openweathermap_api.h"
#include "secrets.h"
#include "cjson_parsing.h"

#define REQUEST_TIMEOUT_MS 10000
#define MAX_HTTP_OUTPUT_BUFFER 8192 - 1

// Macro to log an error and exit a function with error code -1 if expr != 0
#define CJSON_CHECK_ERROR_RETURN(expr)     \
    if (expr != 0)                         \
    {                                      \
        ESP_LOGE(TAG, "cJSON hard error"); \
        return -1;                         \
    }

extern const char openweathermaporg_cert_pem_start[] asm("_binary_openweathermaporg_cert_pem_start");
extern const char openweathermaporg_cert_pem_end[] asm("_binary_openweathermaporg_cert_pem_end");

static const char *TAG = "openweathermap_api";

static char received_data[MAX_HTTP_OUTPUT_BUFFER + 1] = {0}; // Extra byte for the NULL character
static int received_data_len = 0;
static bool received_data_valid = false;

esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGD(TAG, "HTTP_EVENT_ERROR");
        break;

    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_CONNECTED");
        break;

    case HTTP_EVENT_HEADER_SENT:
        ESP_LOGD(TAG, "HTTP_EVENT_HEADER_SENT");
        break;

    case HTTP_EVENT_ON_HEADER:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_HEADER, key=%s, value=%s", evt->header_key, evt->header_value);
        break;

    case HTTP_EVENT_ON_DATA:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_DATA, len=%d", evt->data_len);

        // Clean the buffer in case of a new request
        if (received_data_len == 0 && evt->user_data)
        {
            // we are just starting to copy the output data into the use
            memset(evt->user_data, 0, MAX_HTTP_OUTPUT_BUFFER);
        }

        if (!esp_http_client_is_chunked_response(evt->client))
        {
            int copy_len = 0;

            if (evt->user_data) // If user_data buffer is configured, copy the response into the buffer
            {
                // The last byte in evt->user_data is kept for the NULL character in case of out-of-bound access.
                copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - received_data_len));
                if (copy_len)
                {
                    memcpy(evt->user_data + received_data_len, evt->data, copy_len);
                }
            }
            else // If user_data buffer is not configured, accumulate the response in output_buffer
            {
                int content_len = esp_http_client_get_content_length(evt->client);

                if (received_data_len == 0)
                {
                    memset(received_data, 0, MAX_HTTP_OUTPUT_BUFFER + 1);
                }

                copy_len = MIN(evt->data_len, (content_len - received_data_len));
                if (copy_len)
                {
                    memcpy(received_data + received_data_len, evt->data, copy_len);
                }
            }
            received_data_len += copy_len;
        }
        else // Chunked encoding
        {
            ESP_LOGD(TAG, "Received data chunk = %.*s", evt->data_len, (char *)evt->data);

            if (received_data_len == 0)
            {
                memset(received_data, 0, MAX_HTTP_OUTPUT_BUFFER + 1);
            }

            // Check if we have enough space in the buffer
            if (received_data_len + evt->data_len <= MAX_HTTP_OUTPUT_BUFFER)
            {
                // Append the received data to the global buffer
                memcpy(received_data + received_data_len, evt->data, evt->data_len);
                received_data_len += evt->data_len;
            }
            else
            {
                ESP_LOGW(TAG, "Not enough space to store all data in buffer");
                return ESP_FAIL;
            }
        }
        break;

    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD(TAG, "HTTP_EVENT_ON_FINISH");

        if (received_data_len > 0)
        {
            // Add null-termination character to the end of the string
            if (received_data_len <= MAX_HTTP_OUTPUT_BUFFER)
            {
                received_data[received_data_len] = '\0';
            }
            ESP_LOGD(TAG, "Accumulated Response = %.*s", received_data_len, received_data);
            received_data_len = 0;
            received_data_valid = true;
        }
        break;

    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD(TAG, "HTTP_EVENT_DISCONNECTED");
        int mbedtls_err = 0;
        esp_err_t err = esp_tls_get_and_clear_last_error((esp_tls_error_handle_t)evt->data, &mbedtls_err, NULL);
        if (err != 0)
        {
            ESP_LOGD(TAG, "Last esp error code = 0x%x", err);
            ESP_LOGD(TAG, "Last mbedtls failure = 0x%x", mbedtls_err);
        }
        break;

    case HTTP_EVENT_REDIRECT:
        ESP_LOGD(TAG, "HTTP_EVENT_REDIRECT");
        esp_http_client_set_header(evt->client, "From", "user@example.com");
        esp_http_client_set_header(evt->client, "Accept", "text/html");
        esp_http_client_set_redirection(evt->client);
        break;
    }
    return ESP_OK;
}

int8_t request_weather_data(WeatherData *data)
{
    ESP_LOGI(TAG, "Requesting weather data...");

    char url[200];
    snprintf(url, sizeof(url), "%s%s%s%s%s%s%s",
             "https://api.openweathermap.org/data/3.0/onecall?lat=",
             OPENWEATHERMAP_LATITUDE, "&lon=", OPENWEATHERMAP_LONGITUDE,
             "&appid=", OPENWEATHERMAP_API_KEY,
             "&units=metric&exclude=current,minutely,hourly");

    esp_http_client_config_t config = {
        .url = url,
        .event_handler = _http_event_handler,
        .cert_pem = openweathermaporg_cert_pem_start,
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_err_t err = esp_http_client_perform(client);

    if (err == ESP_OK)
    {
        ESP_LOGD(TAG, "HTTPS Status = %d, content_length = %" PRId64,
                 esp_http_client_get_status_code(client),
                 esp_http_client_get_content_length(client));
    }
    else
    {
        ESP_LOGE(TAG, "Error performing HTTP request %s", esp_err_to_name(err));
        return -1;
    }

    esp_http_client_cleanup(client);

    // Get current time (as ticks) to check for timeout
    TickType_t start_time = xTaskGetTickCount();
    TickType_t diff;
    while (!received_data_valid) // Wait for the data to be received.
    {
        vTaskDelay(100 / portTICK_PERIOD_MS);

        diff = xTaskGetTickCount() - start_time;
        if (diff > REQUEST_TIMEOUT_MS / portTICK_PERIOD_MS) // Check for timeout
        {
            ESP_LOGE(TAG, "Request timed out.");
            return -1;
        }
    }

    cJSON *json = cJSON_Parse(received_data); // Watch out, this contains dynamic memory allocation
    if (json != NULL)
    {
        cJSON *daily = cJSON_GetObjectItem(json, "daily");
        if (daily != NULL)
        {
            cJSON *daily_0 = cJSON_GetArrayItem(daily, 0);
            if (daily_0 != NULL)
            {
                cJSON *temp = cJSON_GetObjectItem(daily_0, "temp");
                if (temp != NULL)
                {
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(temp, "morn", &data->temperature.real_morning));
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(temp, "day", &data->temperature.real_day));
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(temp, "eve", &data->temperature.real_evening));
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(temp, "night", &data->temperature.real_night));
                }

                cJSON *feels_like = cJSON_GetObjectItem(daily_0, "feels_like");
                if (feels_like != NULL)
                {
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(feels_like, "morn", &data->temperature.feel_morning));
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(feels_like, "day", &data->temperature.feel_day));
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(feels_like, "eve", &data->temperature.feel_evening));
                    CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(feels_like, "night", &data->temperature.feel_night));
                }

                CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(daily_0, "humidity", &data->humidity));
                CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(daily_0, "clouds", &data->clouds));

                // rain and snow are not always present in the JSON, which is OK (but have to be reset if so)
                if (set_float_from_cjson(daily_0, "rain", &data->rain) != 0)
                    data->rain = 0.0;
                if (set_float_from_cjson(daily_0, "snow", &data->snow) != 0)
                    data->snow = 0.0;

                CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(daily_0, "wind_speed", &data->wind_avg));
                CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(daily_0, "wind_gust", &data->wind_gust));
                CJSON_CHECK_ERROR_RETURN(set_float_from_cjson(daily_0, "pop", &data->pop));
                CJSON_CHECK_ERROR_RETURN(set_string_from_cjson(daily_0, "summary", data->weather_summary, WEATHER_SUMMARY_STRING_LENGTH));

                cJSON *weather = cJSON_GetObjectItem(daily_0, "weather");
                if (weather != NULL)
                {
                    cJSON *weather_0 = cJSON_GetArrayItem(weather, 0);
                    if (weather_0 != NULL)
                    {
                        CJSON_CHECK_ERROR_RETURN(set_string_from_cjson(weather_0, "icon", data->weather_icon, WEATHER_ICON_STRING_LENGTH));
                    }
                }
            }
        }

        cJSON_Delete(json); // Deallocate the JSON object

        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        data->timestamp = current_time;
        data->update_count++;

        ESP_LOGI(TAG, "OpenWeatherMap data received, summary = %s", data->weather_summary);
        return 0;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return -1;
    }
}
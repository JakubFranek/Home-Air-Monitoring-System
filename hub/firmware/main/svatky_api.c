#include <string.h>
#include <time.h>
#include <sys/time.h>
#include <sys/param.h> // includes MIN macro

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG // must be defined before "esp_log.h"

#include "esp_event.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_tls.h"
#include "esp_http_client.h"
#include "cJSON.h"

#include "svatky_api.h"
#include "cjson_parsing.h"

#define REQUEST_TIMEOUT_MS 10000
#define MAX_HTTP_OUTPUT_BUFFER 2048 - 1

extern const char svatkyapicz_cert_pem_start[] asm("_binary_svatkyapicz_cert_pem_start");
extern const char svatkyapicz_cert_pem_end[] asm("_binary_svatkyapicz_cert_pem_end");

static const char *TAG = "svatky_api";

static char received_data[MAX_HTTP_OUTPUT_BUFFER + 1]; // Extra byte is for the NULL character.
static int received_data_len = 0;
static bool received_data_valid = false;

static esp_err_t _http_event_handler(esp_http_client_event_t *evt)
{
    static char *output_buffer; // Buffer to store response of http request from event handler.
    static int output_len;      // Stores number of bytes read.
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

        // Clean the buffer in case of a new request.
        if (output_len == 0 && evt->user_data)
        {
            memset(evt->user_data, 0, MAX_HTTP_OUTPUT_BUFFER);
        }

        if (!esp_http_client_is_chunked_response(evt->client))
        {
            int copy_len = 0;

            if (evt->user_data) // If user_data buffer is configured, copy the response into the buffer.
            {
                // The last byte in evt->user_data is kept for the NULL character in case of out-of-bound access.
                copy_len = MIN(evt->data_len, (MAX_HTTP_OUTPUT_BUFFER - output_len));
                if (copy_len)
                {
                    memcpy(evt->user_data + output_len, evt->data, copy_len);
                }
            }
            else // If user_data buffer is not configured, accumulate the response in output_buffer.
            {
                int content_len = esp_http_client_get_content_length(evt->client);

                if (output_buffer == NULL)
                {
                    // We initialize output_buffer with 0 because it is used by strlen() and similar functions therefore should be null terminated.
                    output_buffer = (char *)calloc(content_len + 1, sizeof(char));
                    output_len = 0;
                    if (output_buffer == NULL)
                    {
                        ESP_LOGE(TAG, "Failed to allocate memory for output buffer");
                        return ESP_FAIL;
                    }
                }

                copy_len = MIN(evt->data_len, (content_len - output_len));
                if (copy_len)
                {
                    memcpy(output_buffer + output_len, evt->data, copy_len);
                }
            }
            output_len += copy_len;
        }
        else // Chunked encoding
        {
            ESP_LOGD(TAG, "Received data chunk = %.*s", evt->data_len, (char *)evt->data);

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
        if (output_buffer != NULL)
        {
            ESP_LOGD(TAG, "Accumulated Response = %.*s", output_len, output_buffer);
            free(output_buffer);
            output_buffer = NULL;
        }
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
        output_len = 0;
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
        if (output_buffer != NULL)
        {
            free(output_buffer);
            output_buffer = NULL;
        }
        output_len = 0;
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

int8_t request_svatkyapi_data(SvatkyApiData *data)
{
    esp_http_client_config_t config = {
        .url = "https://svatkyapi.cz/api/day",
        .event_handler = _http_event_handler,
        .cert_pem = svatkyapicz_cert_pem_start,
        //.crt_bundle_attach = esp_crt_bundle_attach,
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
        ESP_LOGE(TAG, "Error perform HTTP request %s", esp_err_to_name(err));
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
            ESP_LOGE(TAG, "Request timed out");
            return -1;
        }
    }

    cJSON *json = cJSON_Parse(received_data); // Watch out, this contains dynamic memory allocation
    if (json != NULL)
    {
        set_string_from_cjson(json, "dayInWeek", data->day_in_week, SVATKY_MAX_STRING_LENGTH);
        set_string_from_cjson(json, "name", data->name, SVATKY_MAX_STRING_LENGTH);
        set_bool_from_cjson(json, "isHoliday", &data->is_holiday);
        if (data->is_holiday)
        {
            set_string_from_cjson(json, "holidayName", data->holiday_name, SVATKY_MAX_STRING_LENGTH);
        }

        cJSON_Delete(json); // Deallocate the JSON object

        struct timeval current_time;
        gettimeofday(&current_time, NULL);
        data->timestamp = current_time;

        ESP_LOGI(TAG, "SvatkyApiData received, day_in_week = %s, name = %s, is_holiday = %d, holiday_name = %s",
                 data->day_in_week, data->name, data->is_holiday, data->holiday_name);
        return 0;
    }
    else
    {
        ESP_LOGE(TAG, "Failed to parse JSON");
        return -1;
    }
}

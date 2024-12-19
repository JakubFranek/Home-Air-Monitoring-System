#include "cjson_parsing.h"
#include <string.h>
#include <esp_log.h>

static const char *TAG = "cjson_parsing";

/**
 * @brief Parse a float value from a `cJSON` object.
 *
 * @param[in] json `cJSON` object to parse.
 * @param[in] key Key to look for in the `cJSON` object.
 * @param[out] data Address to store the parsed float value.
 *
 * @return 0 on success, -1 on failure.
 */
int8_t set_float_from_cjson(cJSON *json, char *key, float *data)
{
    cJSON *value = cJSON_GetObjectItem(json, key);
    if (value != NULL && cJSON_IsNumber(value))
    {
        *data = (float)value->valuedouble;
        return 0;
    }
    ESP_LOGW(TAG, "Failed to parse float from cJSON, key = %s", key);
    return -1;
}

/**
 * @brief Parse a string value from a `cJSON` object.
 *
 * @param[in] json `cJSON` object to parse.
 * @param[in] key Key to look for in the `cJSON` object.
 * @param[out] data  `char` buffer to store the parsed string value.
 * @param[in] size Size of the `char` buffer
 *
 * @return 0 on success, -1 on failure.
 */
int8_t set_string_from_cjson(cJSON *json, char *key, char *data, size_t size)
{
    cJSON *value = cJSON_GetObjectItem(json, key);
    if (value != NULL && cJSON_IsString(value))
    {
        size_t length = strlen(value->valuestring) + 1;
        if (length > size)
        {
            return -1;
        }
        strncpy(data, value->valuestring, length);
        return 0;
    }
    ESP_LOGW(TAG, "Failed to parse float from cJSON, key = %s", key);
    return -1;
}

/**
 * @brief Parse an integer value from a `cJSON` object.
 *
 * @param[in] json `cJSON` object to parse.
 * @param[in] key Key to look for in the `cJSON` object.
 * @param[out] data Address to store the parsed int value.
 *
 * @return 0 on success, -1 on failure.
 */
int8_t set_int_from_cjson(cJSON *json, char *key, int *data)
{
    cJSON *value = cJSON_GetObjectItem(json, key);
    if (value != NULL)
    {
        *data = value->valueint;
        return 0;
    }
    ESP_LOGW(TAG, "Failed to parse float from cJSON, key = %s", key);
    return -1;
}

/**
 * @brief Parse a boolean value from a `cJSON` object.
 *
 * @param[in] json `cJSON` object to parse.
 * @param[in] key Key to look for in the `cJSON` object.
 * @param[out] data Address to store the parsed boolean value.
 *
 * @return 0 on success, -1 on failure.
 */

int8_t set_bool_from_cjson(cJSON *json, char *key, bool *data)
{
    cJSON *value = cJSON_GetObjectItem(json, key);
    if (value != NULL)
    {
        if (cJSON_IsTrue(value))
        {
            *data = true;
            return 0;
        }
        else if (cJSON_IsFalse(value))
        {
            *data = false;
            return 0;
        }
    }
    ESP_LOGW(TAG, "Failed to parse float from cJSON, key = %s", key);
    return -1;
}
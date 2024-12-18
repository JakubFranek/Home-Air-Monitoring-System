#ifdef __cplusplus
extern "C"
{
#endif

#ifndef __CJSON_PARSING_H__
#define __CJSON_PARSING_H__

#include <stdint.h>
#include <stdbool.h>
#include "cJSON.h"

    int8_t set_float_from_cjson(cJSON *json, char *key, float *data);
    int8_t set_string_from_cjson(cJSON *json, char *key, char *data, size_t size);
    int8_t set_int_from_cjson(cJSON *json, char *key, int *data);
    int8_t set_bool_from_cjson(cJSON *json, char *key, bool *data);

#endif // __CJSON_PARSING_H__

#ifdef __cplusplus
}
#endif
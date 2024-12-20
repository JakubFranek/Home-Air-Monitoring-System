#ifdef __cplusplus
extern "C"
{
#endif

#define SVATKY_MAX_STRING_LENGTH 64

#ifndef __SVATKY_API_H__
#define __SVATKY_API_H__

    typedef struct SvatkyApiData
    {
        char day_in_week[SVATKY_MAX_STRING_LENGTH];
        char name[SVATKY_MAX_STRING_LENGTH];
        bool is_holiday;
        char holiday_name[SVATKY_MAX_STRING_LENGTH];
        struct timeval timestamp;
        int32_t update_count;
    } SvatkyApiData;

    int8_t request_svatkyapi_data(SvatkyApiData *data);

#endif /* __SVATKY_API_H__ */

#ifdef __cplusplus
}
#endif
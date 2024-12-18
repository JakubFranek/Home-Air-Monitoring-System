#ifdef __cplusplus
extern "C"
{
#endif

#define SVATKY_MAX_STRING_LENGTH 64

    typedef struct SvatkyApiData
    {
        char day_in_week[SVATKY_MAX_STRING_LENGTH];
        char name[SVATKY_MAX_STRING_LENGTH];
        bool is_holiday;
        char holiday_name[SVATKY_MAX_STRING_LENGTH];
    } SvatkyApiData;

    void request_svatkyapi_data(SvatkyApiData *data);

#ifdef __cplusplus
}
#endif
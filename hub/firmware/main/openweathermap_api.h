#ifdef __cplusplus
extern "C"
{
#endif

#define OPENWEATHERMAP_SUMMARY_STRING_LENGTH 128
#define OPENWEATHERMAP_ICON_STRING_LENGTH 4

    typedef struct WeatherTemperatureData
    {
        float real_morning;
        float real_day;
        float real_evening;
        float real_night;
        float feel_morning;
        float feel_day;
        float feel_evening;
        float feel_night;
    } WeatherTemperatureData;

    typedef struct WeatherData
    {
        WeatherTemperatureData temperature;
        float humidity;
        float pop; // probability of percipitation
        float clouds;
        float rain;
        float snow;
        float wind_avg;
        float wind_gust;
        char weather_summary[OPENWEATHERMAP_SUMMARY_STRING_LENGTH];
        char weather_icon[OPENWEATHERMAP_ICON_STRING_LENGTH];
    } WeatherData;

    int8_t request_weather_data(WeatherData *data);

#ifdef __cplusplus
}
#endif
#ifndef __WEATHER_DATA_H__
#define __WEATHER_DATA_H__

#include <time.h>
#include <sys/time.h>

#define WEATHER_SUMMARY_STRING_LENGTH 128
#define WEATHER_ICON_STRING_LENGTH 4

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
    char weather_summary[WEATHER_SUMMARY_STRING_LENGTH];
    char weather_icon[WEATHER_ICON_STRING_LENGTH];

    struct timeval timestamp;
    int32_t update_count;
} WeatherData;

#endif
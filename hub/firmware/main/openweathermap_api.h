#ifdef __cplusplus
extern "C"
{
#endif

#ifndef __OPENWEATHERMAP_API_H__
#define __OPENWEATHERMAP_API_H__

#include <time.h>
#include <sys/time.h>

#include "weather_data.h"

    int8_t request_weather_data(WeatherData *data);

#endif /* __OPENWEATHERMAP_API_H__ */

#ifdef __cplusplus
}
#endif
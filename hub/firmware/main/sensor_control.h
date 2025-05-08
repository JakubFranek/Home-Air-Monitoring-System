#ifdef __cplusplus
extern "C"
{
#endif

#ifndef __SENSOR_CONTROL_H__
#define __SENSOR_CONTROL_H__

#include <time.h>
#include <sys/time.h>

#define TEMPERATURE_CORRECTION_FACTOR -0.6 // °C, this is approximately difference between temperature outside and inside the box

    typedef struct HubSensorData
    {
        float pressure_hPa;
        struct timeval pressure_timestamp;
        int32_t pressure_measurements;
        int8_t pressure_status;
        int32_t pressure_errors;

        uint16_t co2;
        struct timeval co2_timestamp;
        int32_t co2_measurements;
        int8_t co2_status;
        int32_t co2_errors;
        int16_t co2_frc_correction;
        bool co2_scheduled_correction;

        int32_t voc_index;
        int32_t nox_index;
        struct timeval gas_index_timestamp;
        int32_t gas_index_measurements;
        int8_t gas_index_status;
        int32_t gas_index_errors;

        float pm_1_0;
        float pm_2_5;
        float pm_4_0;
        float pm_10_0;
        float pm_typical_size; // in micrometers
        struct timeval pm_timestamp;
        int32_t pm_measurements;
        int8_t pm_status;
        int32_t pm_errors;

        float temperature;
        float humidity;
        struct timeval temperature_humidity_timestamp;
        int32_t temperature_humidity_measurements;
        int8_t temperature_humidity_status;
        int32_t temperature_humidity_errors;
    } HubSensorData;

    typedef void (*printf_like_t)(const char *format, ...);

    int8_t setup_sensors(bool debug_print, printf_like_t debug_print_fn);
    int8_t measure_sensors(bool debug_print, printf_like_t debug_print_fn);
    int8_t get_sensor_data(HubSensorData *data);
    int8_t schedule_co2_correction(void);

#endif /* __SENSOR_CONTROL_H__ */

#ifdef __cplusplus
}
#endif
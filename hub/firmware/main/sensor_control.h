#ifdef __cplusplus
extern "C"
{
#endif

#ifndef __SENSOR_CONTROL_H__
#define __SENSOR_CONTROL_H__

#include <time.h>
#include <sys/time.h>

    typedef struct SensorHubData
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
        float pm_typical_size;
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
    } SensorHubData;

    extern SensorHubData sensor_hub_data;

    int8_t setup_i2c_bus(void);
    int8_t setup_sht4x(void);
    int8_t setup_sgp41(void);
    int8_t setup_bme280(void);
    int8_t setup_scd4x(void);
    int8_t setup_sps30(void);

    int8_t measure_sht4x(void);
    int8_t measure_sgp41(void);
    int8_t measure_bme280(void);
    int8_t measure_scd4x(void);
    int8_t measure_sps30(void);

#endif /* __SENSOR_CONTROL_H__ */

#ifdef __cplusplus
}
#endif
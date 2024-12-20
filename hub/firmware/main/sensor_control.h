#ifdef __cplusplus
extern "C"
{
#endif

#ifndef __SENSOR_CONTROL_H__
#define __SENSOR_CONTROL_H__

    typedef struct SensorHubData
    {
        float pressure;
        timeval pressure_timestamp;
        int8_t pressure_status;

        float co2;
        timeval co2_timestamp;
        int8_t co2_status;

        int32_t voc_index;
        int32_t nox_index;
        timeval gas_index_timestamp;
        int8_t gas_index_status;

        float pm_1_0;
        float pm_2_5;
        float pm_4_0;
        float pm_10_0;
        float pm_typical_size;
        timeval pm_timestamp;
        int8_t pm_status;

        float temperature;
        float humidity;
        timeval temperature_humidity_timestamp;
        int8_t temperature_humidity_status;
    } SensorHubData;

    void setup_sensors(void);

    void measure_sht4x(void);
    void measure_sgp41(void);
    void measure_bme280(void);
    void measure_scd4x(void);
    void measure_sps30(void);

#endif /* __SENSOR_CONTROL_H__ */

#ifdef __cplusplus
}
#endif
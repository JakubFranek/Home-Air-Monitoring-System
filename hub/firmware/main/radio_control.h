#ifdef __cplusplus
extern "C"
{
#endif

#ifndef __RADIO_CONTROL_H__
#define __RADIO_CONTROL_H__

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "node_constants.h"

#define NODE_NAME_MAX_LENGTH 12

    typedef struct NodeData
    {
        char node_name[NODE_NAME_MAX_LENGTH];
        uint8_t node_id;
        int32_t rx_count; // Number of received packets

        int8_t app_status;
        int8_t sht4x_status;
        int8_t nrf24_status;
        float temperature_celsius;
        float humidity_pct;
        float vdda_v;
        struct timeval timestamp;

        float temperature_24h_min;
        struct timeval timestamp_temperature_24h_min;
    } NodeData;

    void task_nrf24_control(void *pvParameters);

    int8_t get_node_data(NodeData node_data_set[NODE_COUNT]);

#endif /* __RADIO_CONTROL_H__ */

#ifdef __cplusplus
}
#endif
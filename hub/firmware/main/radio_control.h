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
#define RX_PERIOD_S 60
#define RX_PERIOD_TOLERANCE_PCT 10

    typedef struct NodeData
    {
        char node_name[NODE_NAME_MAX_LENGTH];
        uint8_t node_id;
        int32_t rx_count; // Number of received packets
        int32_t rx_missed_count;
        int32_t rx_missed_windows;
        int32_t rx_max_missed_window;
        int32_t rx_current_missed_window;
        int8_t rx_missed_window_active; // Evaluated as boolean

        int8_t app_status;
        int8_t sht4x_status;
        int8_t nrf24_status;
        float temperature_celsius;
        float humidity_pct;
        float vdda_v;
        struct timeval timestamp;
    } NodeData;

    void task_nrf24_control(void *pvParameters);

    int8_t get_node_data(NodeData node_data_set[NODE_COUNT]);
    int8_t update_node_diagnostics(void);

#endif /* __RADIO_CONTROL_H__ */

#ifdef __cplusplus
}
#endif
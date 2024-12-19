#ifdef __cplusplus
extern "C"
{
#endif

#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"

    typedef struct NodeData
    {
        uint8_t node_id;
        int8_t app_error;
        int8_t sht4x_status;
        int8_t nrf24_status;
        float temperature_celsius;
        float humidity_pct;
        float vdda_v;
        struct timeval timestamp;
    } NodeData;

    typedef struct NodeDataSet
    {
        NodeData node_data[6];
        SemaphoreHandle_t mutex; // Mutex for accessing `node_data`
    } NodeDataSet;

    void task_nrf24_control(void *pvParameters);

    int8_t get_node_data(NodeDataSet *node_data_set);

#ifdef __cplusplus
}
#endif
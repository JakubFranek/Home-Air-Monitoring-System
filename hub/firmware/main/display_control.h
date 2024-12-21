#ifndef __DISPLAY_CONTROL_H__
#define __DISPLAY_CONTROL_H__

#include "openweathermap_api.h"
#include "svatky_api.h"
#include "radio_control.h"
#include "sensor_control.h"

#include "esp_wifi.h"

#define NODE_NAME_MAX_LENGTH 12
#define DATE_STRING_MAX_LENGTH 32

typedef struct DisplayData
{
    // Display data
    SvatkyApiData svatky;
    WeatherData weather;
    NodeData nodes[6];
    SensorHubData hub;

    // Debug data
    char wifi_status[32];
    char wifi_ssid[33];
    int8_t wifi_rssi;
    struct timeval start_time;
    struct timeval sntp_last_sync;
    int32_t sntp_sync_count;
    int8_t app_status;
} DisplayData;

void setup_display(void);
void clear_screen(void);
void print_line(const char *format, ...);
void update_display(DisplayData *data);
void show_debug_info(DisplayData *data);

#endif
#ifndef __HAMS_DATA_H__
#define __HAMS_DATA_H__

#include <time.h>
#include <sys/time.h>

#include "calendar_data.h"
#include "weather_data.h"
#include "radio_control.h"
#include "sensor_control.h"

typedef struct DebugData
{
    struct timeval start_time; // Timestamp of the start of the application
    int8_t app_status;         // Status of the application

    char wifi_status[32];           // "connected", "disconnected" or "unitialized"
    char wifi_ssid[33];             // Name of the Wi-Fi access point
    int8_t wifi_rssi;               // Wi-Fi signal strength as dBm (less negative = better)
    uint32_t wifi_connection_count; // Number of successful connection attempts

    struct timeval sntp_last_sync; // Last SNTP sync time
    uint32_t sntp_sync_count;      // Number of successful SNTP sync attempts
} DebugData;

typedef struct HamsData
{
    CalendarData calendar;
    WeatherData weather;
    NodeData nodes[6];
    HubSensorData hub_sensors;
    DebugData debug;
} HamsData;

#endif
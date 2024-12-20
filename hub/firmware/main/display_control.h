#include "openweathermap_api.h"
#include "svatky_api.h"
#include "radio_control.h"
#include "sensor_control.h"

#define NODE_NAME_MAX_LENGTH 12
#define DATE_STRING_MAX_LENGTH 32

typedef struct DisplayData
{
    SvatkyApiData svatky;
    WeatherData weather;
    NodeData nodes[6];
    SensorHubData hub;
    bool is_ok;
} DisplayData;

typedef struct DebugData
{
    bool wifi_is_connected;
} DebugData;

void setup_display(void);
void update_display(DisplayData *display_data);
void show_debug_info(DebugData *debug_data); // TODO: implement debug screen
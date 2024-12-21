#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>
#include "esp_wifi.h"
#include "display_control.h"

    int8_t setup_wifi(void);
    int8_t connect_wifi(void);
    bool is_wifi_connected(void);
    void get_wifi_ap_record(DisplayData *data);

#ifdef __cplusplus
}
#endif
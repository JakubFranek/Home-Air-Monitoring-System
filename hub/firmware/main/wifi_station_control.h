#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

#include "hams_data.h"

    int8_t setup_wifi(void);
    int8_t connect_wifi(void);
    bool is_wifi_connected(void);
    int8_t get_wifi_ap_record(DebugData *data);

#ifdef __cplusplus
}
#endif
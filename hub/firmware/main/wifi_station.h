#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdint.h>

    void wifi_init_sta(void);
    int8_t setup_wifi(void);
    bool is_wifi_connected(void);

#ifdef __cplusplus
}
#endif
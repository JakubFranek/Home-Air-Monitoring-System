#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <time.h>
#include <sys/time.h>

    extern struct timeval sntp_last_sync;
    extern uint32_t sntp_sync_count;

    int8_t initialize_sntp(void);

#ifdef __cplusplus
}
#endif
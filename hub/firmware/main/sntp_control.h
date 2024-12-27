#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include <time.h>
#include <sys/time.h>

    int8_t initialize_sntp(void);
    int8_t synchronize_time(void);
    int8_t get_sntp_stats(struct timeval *last_sync, uint32_t *sync_count);

#ifdef __cplusplus
}
#endif
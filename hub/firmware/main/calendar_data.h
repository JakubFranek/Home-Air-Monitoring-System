#ifndef __CALENDAR_DATA_H__
#define __CALENDAR_DATA_H__

#include <time.h>
#include <sys/time.h>

#define SVATKY_MAX_STRING_LENGTH 64

typedef struct CalendarData
{
    char day_in_week[SVATKY_MAX_STRING_LENGTH];
    char name[SVATKY_MAX_STRING_LENGTH];
    bool is_holiday;
    char holiday_name[SVATKY_MAX_STRING_LENGTH];

    struct timeval timestamp;
    int32_t update_count;
} CalendarData;

#endif
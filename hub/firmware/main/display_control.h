#ifndef __DISPLAY_CONTROL_H__
#define __DISPLAY_CONTROL_H__

#include "hams_data.h"

void setup_display(void);
void clear_screen(void);
void print_line(const char *format, ...);
void update_display(HamsData *data);
void show_debug_info(HamsData *data);

#endif
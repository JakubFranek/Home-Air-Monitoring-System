#include <stdio.h>
#include <string>
#include <iostream>
#include <unordered_map>
#include <cstring>
#include "sdkconfig.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "gfxfont.h"
#include "Fonts/FreeSansBold8pt7b.h"
#include "Fonts/FreeSansBold10pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSansBold14pt7b.h"
#include "Fonts/FreeSansBold16pt7b.h"
#include "Fonts/FreeSansBold18pt7b.h"
#include "Fonts/FreeSansBold72pt7b.h"
#include "Fonts/FreeSans8pt7b.h"
#include "Fonts/FreeSans9pt7b.h"
#include "Fonts/FreeSans10pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSans15pt7b.h"

#include "displays/goodisplay/gdey075T7.h"

#include "weather_icons.h"
#include "time_of_day_icons.h"
#include "nodes.h"

#include "display_control.h"

using namespace std;

#define SHOW_DEBUG_RECTS false

#define DISPLAY_REFRESH_EVERY_N_FRAMES 15
#define DISPLAY_TEXT_12PT_YOFFSET 30
#define DISPLAY_TEXT_10PT_YOFFSET 25
#define DISPLAY_VSEC0_HEIGHT 110
#define DISPLAY_TIME_ICON_YPOS 142
#define DISPLAY_VSEC2_TEXT_YPOS DISPLAY_TIME_ICON_YPOS + 4 * TIME_OF_DAY_ICON_SIZE + 20

static const char *TAG = "display_control";

// Define a mapping of Czech characters to ASCII equivalents
static unordered_map<string, char> cz_to_ascii = {
    {"á", 'a'}, {"č", 'c'}, {"ď", 'd'}, {"é", 'e'}, {"ě", 'e'}, {"í", 'i'}, {"ň", 'n'}, {"ó", 'o'}, {"ř", 'r'}, {"š", 's'}, {"ť", 't'}, {"ú", 'u'}, {"ů", 'u'}, {"ý", 'y'}, {"ž", 'z'}, {"Á", 'A'}, {"Č", 'C'}, {"Ď", 'D'}, {"É", 'E'}, {"Ě", 'E'}, {"Í", 'I'}, {"Ň", 'N'}, {"Ó", 'O'}, {"Ř", 'R'}, {"Š", 'S'}, {"Ť", 'T'}, {"Ú", 'U'}, {"Ů", 'U'}, {"Ý", 'Y'}, {"Ž", 'Z'}};

static EpdSpi epd_spi;
static Gdey075T7 display(epd_spi);

static unordered_map<string, const uint8_t *> weather_icons = {
    {"01d", weather_01d},
    {"02d", weather_02d},
    {"03d", weather_03d},
    {"04d", weather_04d},
    {"09d", weather_09d},
    {"10d", weather_10d},
    {"11d", weather_11d},
    {"13d", weather_13d},
    {"50d", weather_50d},
};

static const uint8_t *weather_icon = weather_01d;
static uint32_t display_counter = 0;
static uint32_t line_counter = 0;
static uint32_t full_update_count = 0, fast_update_count = 0;
static char string_buffer[512] = {0};

static void print_weather_summary(Gdey075T7 *display, string weather_summary);
static void replace_czech_characters(char *buffer);
static void get_timestamp_string(struct timeval *timestamp, char *buffer);
static void print_holiday(DisplayData *data, struct tm *time_info);

/**
 * @brief Initializes the display and clears it.
 *
 * This function calls the `initialize()` method of the display object to set
 * up the display controller. It then calls `clear_screen()` to fill the display
 * with white and reset the internal display pointer. Finally, it sets the display
 * rotation to horizontal mode and sets the text color to black.
 */
void setup_display(void)
{
    display.initialize();
    display.clear_screen();
    full_update_count++;
    display.setRotation(2);
    display.setTextColor(EPD_BLACK);
}

/**
 * @brief Clears the display and fills it with white.
 */
void clear_screen(void)
{
    display.fillScreen(EPD_WHITE);
    display.clear_screen();
    full_update_count++;
    line_counter = 0;
}

/**
 * @brief Prints a line of text to the display.
 *
 * This function prints a line of text to the display using a printf-style
 * format string. The function handles the display wake-up and update process
 * internally.
 *
 * @param format The printf-style format string.
 * @param ... Additional arguments for the format string.
 */
void print_line(const char *format, ...)
{
    va_list args;
    va_start(args, format);
    char text_buffer[1024];
    int size = vsnprintf(text_buffer, sizeof text_buffer, format, args);
    va_end(args);
    string text = "";
    if (size < sizeof(text_buffer))
        text = std::string(text_buffer);
    else
        ESP_LOGE(TAG, "print_line: text_buffer out of range");

    display.wake_up();
    display.setFont(&FreeSans10pt7b);
    if (line_counter == 0)
    {
        display.setCursor(0, 15);
    }
    display.print(text);
    line_counter++;

    display.update();
    fast_update_count++;
}

/**
 * @brief Updates the display with the current time, date, weather information, and sensor data.
 *
 * @param data Pointer to the DisplayData structure containing time, weather, and sensor information.
 */

void update_display(DisplayData *data)
{
    if (display_counter > 0)
    {
        display.wake_up(); // this re-initializes the "old" frame buffer after deep sleep period, which prevents pixel noise
    }

    /* --- Frame buffer CAN change now --- */

    display.fillScreen(EPD_WHITE); // Reset frame buffer to white

    struct timeval current_time;
    struct tm time_info;
    gettimeofday(&current_time, NULL);
    localtime_r(&current_time.tv_sec, &time_info);
    strftime(string_buffer, sizeof(string_buffer), "%H:%M", &time_info);
    display.draw_aligned_text(&FreeSansBold72pt7b, 0, 0, GDEY075T7_WIDTH / 2, 107, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, string_buffer);

    print_holiday(data, &time_info);

    display.drawLine(0, DISPLAY_VSEC0_HEIGHT, GDEY075T7_WIDTH, DISPLAY_VSEC0_HEIGHT, EPD_BLACK);
    display.drawLine(GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT, GDEY075T7_WIDTH / 2, GDEY075T7_HEIGHT, EPD_BLACK);

    display.draw_aligned_text(&FreeSansBold10pt7b, 0, DISPLAY_VSEC0_HEIGHT, 100, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "T [*C]");
    display.draw_aligned_text(&FreeSansBold10pt7b, 50, DISPLAY_VSEC0_HEIGHT, 85, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "Realna");
    display.draw_aligned_text(&FreeSansBold10pt7b, 140, DISPLAY_VSEC0_HEIGHT, 95, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "Pocitova");

    display.drawBitmap(0, DISPLAY_TIME_ICON_YPOS + 0 * TIME_OF_DAY_ICON_SIZE, morning, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
    display.drawBitmap(0, DISPLAY_TIME_ICON_YPOS + 1 * TIME_OF_DAY_ICON_SIZE, day, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
    display.drawBitmap(0, DISPLAY_TIME_ICON_YPOS + 2 * TIME_OF_DAY_ICON_SIZE, evening, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
    display.drawBitmap(0, DISPLAY_TIME_ICON_YPOS + 3 * TIME_OF_DAY_ICON_SIZE, night, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
    if (SHOW_DEBUG_RECTS)
    {
        display.drawRect(0, DISPLAY_TIME_ICON_YPOS + 0 * TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
        display.drawRect(0, DISPLAY_TIME_ICON_YPOS + 1 * TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
        display.drawRect(0, DISPLAY_TIME_ICON_YPOS + 2 * TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
        display.drawRect(0, DISPLAY_TIME_ICON_YPOS + 3 * TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, TIME_OF_DAY_ICON_SIZE, EPD_BLACK);
    }

    sprintf(string_buffer, "%.1f", data->weather.temperature.real_morning);
    display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 0 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.1f", data->weather.temperature.real_day);
    display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 1 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.1f", data->weather.temperature.real_evening);
    display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 2 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.1f", data->weather.temperature.real_evening);
    display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 3 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    sprintf(string_buffer, "%.1f", data->weather.temperature.feel_morning);
    display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 0 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.1f", data->weather.temperature.feel_day);
    display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 1 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.1f", data->weather.temperature.feel_evening);
    display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 2 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.1f", data->weather.temperature.feel_evening);
    display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 3 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    display.drawLine(0, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT + 8 * 30, EPD_BLACK); // horizontal line in the weather section separating temperatures and other weather data

    sprintf(string_buffer, "%.0f %%%%", data->weather.humidity);
    display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Vlhkost:");
    display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    sprintf(string_buffer, "%.1f mm", data->weather.rain);
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Dest:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    sprintf(string_buffer, "%.0f %%%%", 100.0 * data->weather.pop);
    display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "%% srazek:");
    display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    sprintf(string_buffer, "%.1f mm", data->weather.snow);
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Snih:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    sprintf(string_buffer, "%.0f %%%%", data->weather.clouds);
    display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Oblacnost:");
    display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    sprintf(string_buffer, "%.1f / %.1f m/s", data->weather.wind_avg, data->weather.wind_gust);
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Vitr:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    display.drawLine((GDEY075T7_WIDTH / 4), DISPLAY_VSEC0_HEIGHT + 8 * 30, (GDEY075T7_WIDTH / 4), 429, EPD_BLACK); // vertical line in the middle of weather section
    display.drawLine(0, 429, (GDEY075T7_WIDTH / 2), 429, EPD_BLACK);                                               // last horizontal line in the weather section

    print_weather_summary(&display, string(data->weather.weather_summary));

    display.draw_aligned_text(&FreeSansBold12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Misto");
    display.draw_aligned_text(&FreeSansBold12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "Teplota");
    display.draw_aligned_text(&FreeSansBold12pt7b, 670, DISPLAY_VSEC0_HEIGHT, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "Vlhkost");

    display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 1 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Obyvak");
    sprintf(string_buffer, "%.2f *C", data->hub.temperature);
    display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 1 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.2f *C", data->hub.humidity);
    display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 1 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    for (int i = 0; i < NODE_COUNT; i++)
    {
        bool is_ok = (data->nodes[i].app_status == 0 && data->nodes[i].sht4x_status == 0 && data->nodes[i].nrf24_status == 0);
        bool is_up_to_date = (data->nodes[i].timestamp.tv_sec + NODE_MAX_SECONDS_SINCE_LAST_UPDATE > current_time.tv_sec);

        if (is_ok)
        {
            sprintf(string_buffer, "%s", NODE_NAMES[i]);
        }
        else
        {
            sprintf(string_buffer, "%s*", NODE_NAMES[i]);
        }
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + (2 + i) * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, string_buffer);

        if (is_up_to_date)
        {
            sprintf(string_buffer, "%.2f *C", data->nodes[i].temperature_celsius);
        }
        else
        {
            sprintf(string_buffer, "-- *C");
        }
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + (2 + i) * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
        if (is_up_to_date)
        {
            sprintf(string_buffer, "%.2f %%%%", data->nodes[i].humidity_pct);
        }
        else
        {
            sprintf(string_buffer, "-- %%%%");
        }
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + (2 + i) * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    }

    display.drawLine(GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, GDEY075T7_WIDTH, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, EPD_BLACK);

    display.drawLine((GDEY075T7_WIDTH * 3) / 4, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, (GDEY075T7_WIDTH * 3) / 4, GDEY075T7_HEIGHT, EPD_BLACK);

    sprintf(string_buffer, "%.1f", data->hub.pressure_hPa);
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Tlak:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "hPa");
    sprintf(string_buffer, "%d", data->hub.co2);
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "CO2:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "ppm");
    sprintf(string_buffer, "%ld", data->hub.voc_index);
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "VOC index:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "/100");
    sprintf(string_buffer, "%ld", data->hub.nox_index);
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "NOx index:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "/1");

    if (data->nodes[NODE_TEMPERATURE_24H_MIN].timestamp_temperature_24h_min.tv_sec != 0)
    {
        sprintf(string_buffer, "%.2f", data->nodes[NODE_TEMPERATURE_24H_MIN].temperature_24h_min);
    }
    else
    {
        sprintf(string_buffer, "--");
    }
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "T 24h min:");
    display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "*C");

    sprintf(string_buffer, "%.2f ug/m^3", data->hub.pm_1_0);
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 1.0:");
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.2f ug/m^3", data->hub.pm_2_5);
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 2.5:");
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.2f ug/m^3", data->hub.pm_4_0);
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 4.0:");
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.2f ug/m^3", data->hub.pm_10_0);
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 10:");
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    sprintf(string_buffer, "%.2f nm", data->hub.pm_typical_size);
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM size:");
    display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    string weather_icon_key = string(data->weather.weather_icon);
    ESP_LOGI(TAG, "weather_icon_key: %s", weather_icon_key.c_str());

    if (weather_icons.find(weather_icon_key) == weather_icons.end()) // Test if key is in map
    {
        ESP_LOGE(TAG, "weather_icon_key not found in map");
    }
    else
    {
        weather_icon = weather_icons[weather_icon_key]; // Map weather icon string to bitmap
    }

    display.drawBitmap(400 - 155, 165, weather_icon, WEATHER_ICON_SIZE, WEATHER_ICON_SIZE, EPD_BLACK);

    if (SHOW_DEBUG_RECTS)
        display.drawRect(400 - 155, 165, WEATHER_ICON_SIZE, WEATHER_ICON_SIZE, EPD_BLACK);

    if (display_counter > 0 && display_counter % DISPLAY_REFRESH_EVERY_N_FRAMES == 0)
    {
        display.clear_screen();
        full_update_count++;
    }
    display.update();
    fast_update_count++;
    /* --- Frame buffer MUST NOT change now --- */

    display_counter++;

    ESP_LOGI(TAG, "update_display done");
}

/**
 * @brief Prints a weather summary string on the display, splitting it into two lines if it's too long.
 * @param display The display to print on.
 * @param weather_summary The weather summary string.
 */
void print_weather_summary(Gdey075T7 *display, string weather_summary)
{
    display->setFont(&FreeSans10pt7b);
    uint16_t w, h;
    int16_t x, y;
    display->getTextBounds(weather_summary.c_str(), 0, 0, &x, &y, &w, &h);

    if (w < GDEY075T7_WIDTH / 2)
    {
        display->draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, GDEY075T7_WIDTH / 2, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, weather_summary.c_str());
        return;
    }

    string line_1 = "";
    string line_2 = "";
    string line_test = "";

    int i = 0;
    int whitespace_pos = weather_summary.length();
    while (i < weather_summary.length())
    {
        if (weather_summary[i] == ' ')
        {
            line_test = weather_summary.substr(0, i);

            display->getTextBounds(line_test.c_str(), 0, 0, &x, &y, &w, &h);

            if (w < GDEY075T7_WIDTH / 2)
            {
                whitespace_pos = i; // save this position
            }
            else
            {
                line_1 = weather_summary.substr(0, whitespace_pos);
                line_2 = weather_summary.substr(whitespace_pos + 1, weather_summary.length() - whitespace_pos - 1);
                break;
            }
        }
        i++;
    }

    if (whitespace_pos == weather_summary.length()) // No suitable whitespace found
    {
        line_1 = weather_summary;
        line_2 = "";
        ESP_LOGE("main", "No suitable whitespace for weather summary string splitting");
    }

    display->draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, GDEY075T7_WIDTH / 2, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, line_1.c_str());
    display->draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, GDEY075T7_WIDTH / 2, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, line_2.c_str());
}

void print_holiday(DisplayData *data, struct tm *time_info)
{
    uint16_t y_height = (data->svatky.is_holiday) ? (DISPLAY_VSEC0_HEIGHT) / 3 : (DISPLAY_VSEC0_HEIGHT) / 2;

    char date_buffer[16];
    sprintf(string_buffer, "%s", data->svatky.day_in_week);
    replace_czech_characters(string_buffer);
    strftime(date_buffer, sizeof(date_buffer), " %d.%m.%Y", time_info);
    strcat(string_buffer, date_buffer);
    display.draw_aligned_text(&FreeSansBold18pt7b, GDEY075T7_WIDTH / 2, 0, GDEY075T7_WIDTH / 2, y_height, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    sprintf(string_buffer, "%s", data->svatky.name);
    replace_czech_characters(string_buffer);
    display.draw_aligned_text(&FreeSansBold16pt7b, GDEY075T7_WIDTH / 2, y_height, GDEY075T7_WIDTH / 2, y_height, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);

    if (data->svatky.is_holiday)
    {
        int16_t x, y;
        uint16_t w, h;
        sprintf(string_buffer, "%s", data->svatky.holiday_name);
        replace_czech_characters(string_buffer);

        display.setFont(&FreeSansBold16pt7b);
        display.getTextBounds(string_buffer, 0, 0, &x, &y, &w, &h);

        if (w < GDEY075T7_WIDTH / 2 - 5)
        {
            display.draw_aligned_text(&FreeSansBold16pt7b, GDEY075T7_WIDTH / 2, y_height * 2, GDEY075T7_WIDTH / 2, y_height, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
            return;
        }

        display.setFont(&FreeSansBold14pt7b);
        display.getTextBounds(string_buffer, 0, 0, &x, &y, &w, &h);
        if (w < GDEY075T7_WIDTH / 2 - 5)
        {
            display.draw_aligned_text(&FreeSansBold14pt7b, GDEY075T7_WIDTH / 2, y_height * 2, GDEY075T7_WIDTH / 2, y_height, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
            return;
        }

        display.setFont(&FreeSansBold12pt7b);
        display.getTextBounds(string_buffer, 0, 0, &x, &y, &w, &h);
        if (w < GDEY075T7_WIDTH / 2 - 5)
        {
            display.draw_aligned_text(&FreeSansBold12pt7b, GDEY075T7_WIDTH / 2, y_height * 2, GDEY075T7_WIDTH / 2, y_height, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
            return;
        }

        display.setFont(&FreeSansBold10pt7b);
        display.getTextBounds(string_buffer, 0, 0, &x, &y, &w, &h);
        if (w < GDEY075T7_WIDTH / 2 - 5)
        {
            display.draw_aligned_text(&FreeSansBold10pt7b, GDEY075T7_WIDTH / 2, y_height * 2, GDEY075T7_WIDTH / 2, y_height, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
            return;
        }

        display.draw_aligned_text(&FreeSansBold8pt7b, GDEY075T7_WIDTH / 2, y_height * 2, GDEY075T7_WIDTH / 2, y_height, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, string_buffer);
    }
}

/**
 * @brief Replace Czech characters in a string with their ASCII equivalents.
 *
 * This function iterates over the string and replaces any Czech characters
 * with their ASCII equivalents. It uses the cz_to_ascii mapping for this.
 *
 * @param buffer The string to replace characters in.
 */
static void replace_czech_characters(char *buffer)
{
    string input(buffer);
    string result;
    size_t i = 0;

    while (i < input.size())
    {
        bool replaced = false;

        // Try to match a multibyte UTF-8 character
        for (const auto &pair : cz_to_ascii)
        {
            const string &key = pair.first;

            if (input.compare(i, key.size(), key) == 0)
            {
                // Match found: replace with ASCII equivalent
                result += pair.second;
                i += key.size(); // Skip matched bytes
                replaced = true;
                break;
            }
        }

        if (!replaced)
        {
            // No match: copy the original character
            result += input[i];
            ++i;
        }
    }

    // Copy the result back to the input buffer
    std::strcpy(buffer, result.c_str());
}

/**
 * @brief Shows debug information about the system on the display.
 *
 * This includes the current and start time, Wi-Fi AP status and RSSI, display update counts, last
 * svatkyapi and openweathermap request timestamps and counts, timestamps of last measurements of all
 * hub sensors, timestamps and error codes of all nodes.
 *
 * @param data The structure containing all the debug information.
 */
void show_debug_info(DisplayData *data)
{
    clear_screen();

    display.setFont(&FreeSans8pt7b);
    display.setCursor(0, 15);

    char timestamp_buffer_1[64], timestamp_buffer_2[64];

    // Print start and current time
    get_timestamp_string(&data->start_time, timestamp_buffer_1);
    struct timeval current_time;
    gettimeofday(&current_time, NULL);
    get_timestamp_string(&current_time, timestamp_buffer_2);
    sprintf(string_buffer, "Start time = %s, current time = %s, app_status = %d\n", timestamp_buffer_1, timestamp_buffer_2, data->app_status);
    display.print(string_buffer);

    // Print display update counts
    sprintf(string_buffer, "Display: full updates = %ld, fast updates = %ld\n", full_update_count, fast_update_count);
    display.print(string_buffer);

    display.print("\n");

    // Print Wi-Fi AP record
    sprintf(string_buffer, "Wi-Fi AP: status = %s, SSID = %s, RSSI = %d, successful connections = %ld\n",
            data->wifi_status, data->wifi_ssid, data->wifi_rssi, data->wifi_connection_count);
    display.print(string_buffer);

    // Print SNTP last sync time and count
    get_timestamp_string(&data->sntp_last_sync, timestamp_buffer_1);
    sprintf(string_buffer, "SNTP: last sync = %s, count = %ld\n", timestamp_buffer_1, data->sntp_sync_count);
    display.print(string_buffer);

    // Print last svatkyapi request timestamp
    get_timestamp_string(&data->svatky.timestamp, timestamp_buffer_1);
    sprintf(string_buffer, "SvatkyAPI: last request = %s, count = %ld\n", timestamp_buffer_1, data->svatky.update_count);
    display.print(string_buffer);

    // Print last openweathermap request timestamp
    get_timestamp_string(&data->weather.timestamp, timestamp_buffer_1);
    sprintf(string_buffer, "OpenWeatherMap: last request = %s, count = %ld\n", timestamp_buffer_1, data->weather.update_count);
    display.print(string_buffer);

    display.print("\n");

    // Print timestamps of last measurements of all hub sensors
    get_timestamp_string(&data->hub.temperature_humidity_timestamp, timestamp_buffer_1);
    sprintf(string_buffer, "SHT40: last measurement = %s, measurements = %ld, errors = %ld\n",
            timestamp_buffer_1, data->hub.temperature_humidity_measurements, data->hub.temperature_humidity_errors);
    display.print(string_buffer);

    get_timestamp_string(&data->hub.gas_index_timestamp, timestamp_buffer_1);
    sprintf(string_buffer, "SGP41: last measurement = %s, measurements = %ld, errors = %ld\n",
            timestamp_buffer_1, data->hub.gas_index_measurements, data->hub.gas_index_errors);
    display.print(string_buffer);

    get_timestamp_string(&data->hub.pressure_timestamp, timestamp_buffer_1);
    sprintf(string_buffer, "BME280: last measurement = %s, measurements = %ld, errors = %ld\n",
            timestamp_buffer_1, data->hub.pressure_measurements, data->hub.pressure_errors);
    display.print(string_buffer);

    get_timestamp_string(&data->hub.pm_timestamp, timestamp_buffer_1);
    sprintf(string_buffer, "SPS30: last measurement = %s, measurements = %ld, errors = %ld\n",
            timestamp_buffer_1, data->hub.pm_measurements, data->hub.pm_errors);
    display.print(string_buffer);

    get_timestamp_string(&data->hub.co2_timestamp, timestamp_buffer_1);
    sprintf(string_buffer, "SCD41: last measurement = %s, measurements = %ld, errors = %ld\n",
            timestamp_buffer_1, data->hub.co2_measurements, data->hub.co2_errors);
    display.print(string_buffer);

    display.print("\n");

    // Print timestamps and error codes of all nodes
    for (int i = 0; i < NODE_COUNT; i++)
    {
        get_timestamp_string(&data->nodes[i].timestamp, timestamp_buffer_1);
        get_timestamp_string(&data->nodes[i].timestamp_temperature_24h_min, timestamp_buffer_2);
        sprintf(string_buffer, "Node %d [%s]: last RX = %s, RX count = %ld, vdda = %.3f V,\n"
                               "     status (app/sht4x/nrf24) = %d/%d/%d, T 24h min. timestamp = %s\n",
                i + 1, NODE_NAMES[i], timestamp_buffer_1, data->nodes[i].rx_count, data->nodes[i].vdda_v,
                data->nodes[i].app_status, data->nodes[i].sht4x_status, data->nodes[i].nrf24_status, timestamp_buffer_2);
        display.print(string_buffer);
    }

    display.update();
    fast_update_count++;
}

/**
 * @brief Convert a struct timeval to a string in the format "YYYY-MM-DD HH:MM:SS.SSSSSS"
 *
 * @param timestamp The struct timeval to convert
 * @param buffer The string buffer to write the result to
 */

static void get_timestamp_string(struct timeval *timestamp, char *buffer)
{
    struct tm time_info;
    char time_buffer[32];
    localtime_r(&timestamp->tv_sec, &time_info);
    strftime(time_buffer, sizeof(time_buffer), "%Y-%m-%d %H:%M:%S", &time_info);
    char usec_buffer[16];
    sprintf(usec_buffer, ".%06ld", timestamp->tv_usec);
    strcat(time_buffer, usec_buffer);
    strcpy(buffer, time_buffer);
}
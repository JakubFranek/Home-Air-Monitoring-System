/* Hello World Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "sdkconfig.h"

#include "gfxfont.h"
#include "Fonts/FreeSansBold10pt7b.h"
#include "Fonts/FreeSansBold11pt7b.h"
#include "Fonts/FreeSansBold12pt7b.h"
#include "Fonts/FreeSansBold14pt7b.h"
#include "Fonts/FreeSansBold16pt7b.h"
#include "Fonts/FreeSansBold18pt7b.h"
#include "Fonts/FreeSans10pt7b.h"
#include "Fonts/FreeSans11pt7b.h"
#include "Fonts/FreeSans12pt7b.h"
#include "Fonts/FreeSans14pt7b.h"
#include "Fonts/FreeSans15pt7b.h"
#include "Fonts/FreeSans16pt7b.h"
#include "Fonts/FreeSans18pt7b.h"
#include "Fonts/FreeSansBold72pt7b.h"

#include "displays/goodisplay/gdey075T7.h"

#include "weather_icons.h"
#include "time_of_day_icons.h"

#define SHOW_DEBUG_RECTS false

#define DISPLAY_TEXT_12PT_YOFFSET 30
#define DISPLAY_TEXT_10PT_YOFFSET 25
#define DISPLAY_VSEC0_HEIGHT 110
#define DISPLAY_TIME_ICON_YPOS 142
#define DISPLAY_VSEC2_TEXT_YPOS DISPLAY_TIME_ICON_YPOS + 4 * TIME_OF_DAY_ICON_SIZE + 20

EpdSpi epd_spi;
Gdey075T7 display(epd_spi);

void print_weather_summary(Gdey075T7 *display, string weather_summary);

extern "C"
{
    void app_main(void);
}

void app_main(void)
{
    // On laskakit ESPink board, the display is powered through a transistor controlled by GPIO_NUM_2
    // To power the display, GPIO_NUM_2 must be set to 1
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_2, 1);

    display.initialize();
    display.clear_screen();
    display.setRotation(2);

    display.setTextColor(EPD_BLACK);
    display.setFont(&FreeSansBold72pt7b);

    const uint8_t *weather_icon = weather_01d;

    uint32_t display_counter = 0;
    string counter_string;

    while (true)
    {
        if (display_counter > 0)
        {
            gpio_set_level(GPIO_NUM_2, 1);
            vTaskDelay(10 / portTICK_PERIOD_MS);

            display.wake_up(); // this re-initializes the "old" frame buffer after deep sleep period, which prevents pixel noise
        }

        /* --- Frame buffer CAN change now --- */

        display.fillScreen(EPD_WHITE);
        display.draw_aligned_text(&FreeSansBold72pt7b, 0, 0, GDEY075T7_WIDTH / 2, 107, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "23:45");

        display.draw_aligned_text(&FreeSansBold18pt7b, GDEY075T7_WIDTH / 2, 0, GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT / 2, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "utery 24.12.2024");
        display.draw_aligned_text(&FreeSansBold16pt7b, GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT / 2, GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT / 2, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "Stedry den / Adam a Eva");

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

        display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 0 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "9.7");
        display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 1 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "23.9");
        display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 2 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "33.2");
        display.draw_aligned_text(&FreeSans15pt7b, 50, DISPLAY_TIME_ICON_YPOS + 3 * TIME_OF_DAY_ICON_SIZE, 85, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "-15.8");

        display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 0 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "8.3");
        display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 1 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "18.9");
        display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 2 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "36.4");
        display.draw_aligned_text(&FreeSans15pt7b, 140, DISPLAY_TIME_ICON_YPOS + 3 * TIME_OF_DAY_ICON_SIZE, 95, TIME_OF_DAY_ICON_SIZE, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "-17.5");

        display.drawLine(0, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT + 8 * 30, EPD_BLACK); // horizontal line in the weather section separating temperatures and other weather data

        display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Vlhkost:");
        display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "84 %%");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Dest:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "2.7 mm");
        display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "%% srazek:");
        display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "63 %%");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Snih:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "0.0 mm");
        display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Oblacnost:");
        display.draw_aligned_text(&FreeSans10pt7b, 0, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "71 %%");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Vitr:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "6.1 / 11.9 m/s");

        display.drawLine((GDEY075T7_WIDTH / 4), DISPLAY_VSEC0_HEIGHT + 8 * 30, (GDEY075T7_WIDTH / 4), 429, EPD_BLACK); // vertical line in the middle of weather section

        display.drawLine(0, 429, (GDEY075T7_WIDTH / 2), 429, EPD_BLACK); // last horizontal line in the weather section

        print_weather_summary(&display, "You can expect light showers in the morning, with clearing in the afternoon.");
        // print_weather_summary(&display, "Expect a day of partly cloudy with rain.");

        display.draw_aligned_text(&FreeSansBold12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Misto");
        display.draw_aligned_text(&FreeSansBold12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "Teplota");
        display.draw_aligned_text(&FreeSansBold12pt7b, 670, DISPLAY_VSEC0_HEIGHT, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "Vlhkost");
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 1 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Obyvak");
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 1 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "23.2 *C");
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 1 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "57.8 %%");
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 2 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Pracovna");
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 2 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "22.1 *C");
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 2 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "54.6 %%");
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 3 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Loznice");
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 3 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "19.5 *C");
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 3 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "59.4 %%");
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 4 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Kuchyn");
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 4 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "24.2 *C");
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 4 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "49.8 %%");
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 5 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Koupelna");
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 5 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "26.6 *C");
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 5 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "79.8 %%");
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 6 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Lodzie");
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 6 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "14.3 *C");
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 6 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "37.9 %%");
        display.draw_aligned_text(&FreeSans12pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC0_HEIGHT + 7 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Sever");
        display.draw_aligned_text(&FreeSans12pt7b, ((GDEY075T7_WIDTH * 5) / 8) + 5, DISPLAY_VSEC0_HEIGHT + 7 * DISPLAY_TEXT_12PT_YOFFSET, 135, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "13.3 *C");
        display.draw_aligned_text(&FreeSans12pt7b, 670, DISPLAY_VSEC0_HEIGHT + 7 * DISPLAY_TEXT_12PT_YOFFSET, 130, DISPLAY_TEXT_12PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "47.1 %%");

        display.drawLine(GDEY075T7_WIDTH / 2, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, GDEY075T7_WIDTH, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, EPD_BLACK);

        display.drawLine((GDEY075T7_WIDTH * 3) / 4, DISPLAY_VSEC0_HEIGHT + 8 * DISPLAY_TEXT_12PT_YOFFSET, (GDEY075T7_WIDTH * 3) / 4, GDEY075T7_HEIGHT, EPD_BLACK);

        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "Tlak:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "1029.7");
        display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "hPa");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "CO2:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "890");
        display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "ppm");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "VOC index:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "102");
        display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "/100");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "NOx index:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "2");
        display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "/1");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 190, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "T 24h min:");
        display.draw_aligned_text(&FreeSans10pt7b, (GDEY075T7_WIDTH / 2) + 5, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 144, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "-14.7");
        display.draw_aligned_text(&FreeSans10pt7b, 559, DISPLAY_VSEC2_TEXT_YPOS + 4 * DISPLAY_TEXT_10PT_YOFFSET, 37, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "*C");

        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 1.0:");
        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 0 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "9.2 ug/m^3");
        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 2.5:");
        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 1 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "11.2 ug/m^3");
        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 4.0:");
        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 2 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "13.7 ug/m^3");
        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_LEFT, "PM 10:");
        display.draw_aligned_text(&FreeSans10pt7b, ((GDEY075T7_WIDTH * 3) / 4) + 5, DISPLAY_VSEC2_TEXT_YPOS + 3 * DISPLAY_TEXT_10PT_YOFFSET, 195, DISPLAY_TEXT_10PT_YOFFSET, SHOW_DEBUG_RECTS, SHOW_DEBUG_RECTS, TEXT_ALIGNMENT_RIGHT, "14.1 ug/m^3");

        switch (display_counter % 9)
        {
        case 0:
            weather_icon = weather_01d;
            break;
        case 1:
            weather_icon = weather_02d;
            break;
        case 2:
            weather_icon = weather_03d;
            break;
        case 3:
            weather_icon = weather_04d;
            break;
        case 4:
            weather_icon = weather_09d;
            break;
        case 5:
            weather_icon = weather_10d;
            break;
        case 6:
            weather_icon = weather_11d;
            break;
        case 7:
            weather_icon = weather_13d;
            break;
        case 8:
            weather_icon = weather_50d;
            break;
        default:
            ESP_LOGE("main", "Unknown weather icon");
        }

        display.drawBitmap(400 - 155, 165, weather_icon, WEATHER_ICON_SIZE, WEATHER_ICON_SIZE, EPD_BLACK);

        if (SHOW_DEBUG_RECTS)
            display.drawRect(400 - 155, 165, WEATHER_ICON_SIZE, WEATHER_ICON_SIZE, EPD_BLACK);

        if (display_counter > 0 && display_counter % 15 == 0)
        {
            display.clear_screen();
        }
        display.update();
        /* --- Frame buffer MUST NOT change now --- */

        display_counter++;
        gpio_set_level(GPIO_NUM_2, 0);

        vTaskDelay(60 * 1000 / portTICK_PERIOD_MS);
    }

    return;
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
/*
 * This file is based on source code originally from martinberlin/CalEPD GitHub repository,
 * available at https://github.com/martinberlin/CalEPD.
 *
 * Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek),
 * as permitted under the Apache License, Version 2.0.
 */

#ifndef epd_h
#define epd_h

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string>
#include <stdint.h>
#include <math.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "esp_system.h"
#include "sdkconfig.h"
#include "esp_log.h"

#include "Adafruit_GFX.h"
#include "epdspi.h"

using namespace std;

typedef enum TEXT_ALIGNMENT
{
    TEXT_ALIGNMENT_LEFT = 0,
    TEXT_ALIGNMENT_CENTER,
    TEXT_ALIGNMENT_RIGHT
} TEXT_ALIGNMENT;

class Epd : public virtual Adafruit_GFX
{
public:
    Epd(int16_t w, int16_t h) : Adafruit_GFX(w, h) {};

    virtual void drawPixel(int16_t x, int16_t y, uint16_t color) = 0; // Override GFX own drawPixel method
    virtual void update() = 0;

    size_t write(uint8_t);
    void print(const std::string &text);
    void print(const char c);
    void println(const std::string &text);
    void printerf(const char *format, ...);
    void newline();
    void draw_aligned_text(const GFXfont *font, int16_t x, int16_t y, uint16_t w, uint16_t h,
                           bool draw_box_outline, bool draw_text_outline, TEXT_ALIGNMENT alignment,
                           const char *format, ...);

protected:
    static constexpr const char *TAG = "Epd";

    static inline uint16_t gx_uint16_min(uint16_t a, uint16_t b) { return (a < b ? a : b); };
    static inline uint16_t gx_uint16_max(uint16_t a, uint16_t b) { return (a > b ? a : b); };

    template <typename T>
    static inline void
    swap(T &a, T &b)
    {
        T t = a;
        a = b;
        b = t;
    }

private:
    virtual void wait_while_busy_(const char *message) = 0;
};
#endif

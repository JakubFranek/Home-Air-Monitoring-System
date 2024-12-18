/*
 * This file is based on source code originally from martinberlin/CalEPD GitHub repository,
 * available at https://github.com/martinberlin/CalEPD.
 *
 * Modifications have been made to the original code by Jakub Franek (https://github.com/JakubFranek),
 * as permitted under the Apache License, Version 2.0.
 */

#include "epd.h"

#include <stdio.h>
#include <stdlib.h>

#include "freertos/task.h"
#include "esp_log.h"

/**
 * @brief Write a single byte (character) to the display
 *
 * This function writes a single byte (character) to the display and returns
 * the number of bytes written, i.e. 1.
 *
 * @param v The byte to write
 * @return The number of bytes written
 */
size_t Epd::write(uint8_t v)
{
  Adafruit_GFX::write(v);
  return 1;
}

void Epd::print(const std::string &text)
{
  for (auto c : text)
  {
    if (c == 195 || c == 194) // accents and special multi-byte characters start with these values
      continue;               // Skip to next letter

    write(uint8_t(c));
  }
}

/**
 * @brief Prints a single character to the display
 *
 * Convenience function to print a single character to the display. This is useful
 * for printing text that spans multiple lines.
 */
void Epd::print(const char c)
{
  write(uint8_t(c));
}

/**
 * @brief Prints a string to the display and inserts a newline at the end
 *
 * Convenience function to print a string to the display and then insert a newline
 * character at the current position. This is useful for printing text that spans
 * multiple lines.
 *
 * @param text The string to print
 */
void Epd::println(const std::string &text)
{
  print(text);
  newline();
}

/**
 * @brief Prints formatted text to the display.
 *
 * This function formats a string using printf-style arguments and prints
 * it to the display. If the formatted string exceeds the buffer size,
 * an error is logged.
 *
 * @param format The printf-style format string.
 * @param ... Additional arguments for the format string.
 */
void Epd::printerf(const char *format, ...)
{
  va_list args;
  va_start(args, format);
  char max_buffer[1024];
  int size = vsnprintf(max_buffer, sizeof max_buffer, format, args);
  va_end(args);

  if (size < sizeof(max_buffer))
    print(std::string(max_buffer));
  else
    ESP_LOGE("Epd::printerf", "max_buffer out of range. Increase max_buffer!");
}

/**
 * @brief Writes a newline character
 *
 * Convenience function to insert a newline character at the current position.
 */
void Epd::newline()
{
  write('\n');
}

/**
 * @brief Draws text vertically centered within a specified boundary.
 *
 * This function formats a string using printf-style arguments, calculates
 * the bounding box of the rendered text, and positions it centered within
 * the specified rectangle defined by (x, y, w, h). The function considers
 * text boundaries and logs errors if the text exceeds the given bounds.
 *
 * @param font The font to use for rendering the text.
 * @param x The x-coordinate of the upper-left corner of the bounding box.
 * @param y The y-coordinate of the upper-left corner of the bounding box.
 * @param w The width of the bounding box.
 * @param h The height of the bounding box.
 * @param draw_box_outline If true, draw an outline around the bounding box.
 * @param draw_text_outline If true, draw an outline around the text.
 * @param alignment The horizontal alignment of the text within the bounding box. Text does not wrap!
 * @param format The printf-style format string for the text.
 * @param ... Additional arguments for the format string.
 */
void Epd::draw_aligned_text(const GFXfont *font, int16_t x, int16_t y, uint16_t w, uint16_t h,
                            bool draw_box_outline, bool draw_text_outline, TEXT_ALIGNMENT alignment, const char *format, ...)
{
  // Handle printf arguments
  va_list args;
  va_start(args, format);
  char max_buffer[1024];
  int size = vsnprintf(max_buffer, sizeof max_buffer, format, args);
  va_end(args);
  string text = "";

  if (size < sizeof(max_buffer))
    text = std::string(max_buffer);
  else
    ESP_LOGE(TAG, "draw_centered_text: max_buffer out of range");

  // Draw external boundary where text needs to be centered in the middle
  setFont(font);
  int16_t text_x = 0, text_y = 0;
  uint16_t text_w = 0, text_h = 0;

  getTextBounds(text.c_str(), x, y, &text_x, &text_y, &text_w, &text_h);

  // Calculate the middle position
  uint16_t dty = (h / 2) + (y - text_y) / 2; // shift text cursor by this much
  uint16_t ty = y + dty;                     // new text origin (cursor y-position)
  uint16_t dtx;
  uint16_t text_x_offset = text_x - x;

  if (alignment == TEXT_ALIGNMENT_LEFT)
    dtx = -text_x_offset;
  else if (alignment == TEXT_ALIGNMENT_RIGHT)
    dtx = (w - text_w) - text_x_offset;
  else
    dtx = (w - text_w) / 2 - text_x_offset;

  uint16_t tx = x + dtx;

  if (draw_text_outline)
  {
    drawRect(tx + text_x_offset, text_y + dty, text_w, text_h, 0);
  }

  if (draw_box_outline)
  {
    drawRect(x, y, w, h, 0);
  }

  if (text_w > w)
    ESP_LOGE(TAG, "draw_centered_text [%s]: text width out of bounds", text.c_str());

  if (text_h > h)
    ESP_LOGE(TAG, "draw_centered_text [%s]: text height out of bounds", text.c_str());

  ESP_LOGI(TAG, "draw_centered_text: string = %s, x = %d, y = %d, \
w = %d, h = %d, text_x = %d, text_y = %d, text_w = %d, text_h = %d, tx = %d, ty = %d",
           text.c_str(), x, y, w, h, text_x, text_y, text_w, text_h, tx, ty);

  setCursor(tx, ty);
  print(text);
}

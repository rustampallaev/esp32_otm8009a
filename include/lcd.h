
#include "esp_system.h"
#include "stdbool.h"

typedef void (*touch_callback_t)(uint32_t);

void lcd_init(touch_callback_t ptr_callback);
uint16_t lcd_get_width(void);
uint16_t lcd_get_height(void);
void lcd_display_on(void);
void lcd_display_off(void);
void lcd_backlight(uint8_t on);
void lcd_clear(uint16_t color);
void lcd_fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void lcd_draw_point(uint16_t x,uint16_t y, uint16_t color);
void lcd_draw_line_h(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color);
void lcd_draw_line_v(uint16_t y0, uint16_t y1, uint16_t x, uint16_t color);
void lcd_draw_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color);
void lcd_draw_char(uint16_t x, uint16_t y, const char ch, const char *font, uint16_t backcolor, uint16_t forecolor);
void lcd_draw_text(uint16_t x, uint16_t y, const char *text, const char *font, uint16_t backcolor, uint16_t forecolor);
uint16_t lcd_get_text_size_w(const char *font, const char *text);
uint16_t lcd_get_text_size_h(const char *font, const char *text);

uint8_t lcd_ts_read(uint16_t *x, uint16_t *y);
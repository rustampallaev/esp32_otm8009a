
#define GFX_COLOR_CREAM         0xE677
#define GFX_COLOR_WHITE         0xFFFF
#define GFX_COLOR_BLACK         0x0000
#define GFX_COLOR_RED           0xF800
#define GFX_COLOR_GREEN         0x07E0
#define GFX_COLOR_BLUE          0x001F
#define GFX_COLOR_LIGHT_BLUE	0x14FB
#define GFX_COLOR_LIGHT_GREEN	0x16ED

#define GFX_COLOR_LIGHT_GRAY    0xE71C
#define GFX_COLOR_GRAY          0x9CB2
#define GFX_COLOR_DARK_GRAY     0x5AA9

#define GFX_COLOR_DARK_ORANGE   0xCA80

#define GFX_COLOR_BACKGROUND			GFX_COLOR_CREAM
#define GFX_COLOR_BUTTON_FORECOLOR		GFX_COLOR_BACKGROUND
#define GFX_COLOR_BUTTON_BACKCOLOR		GFX_COLOR_LIGHT_BLUE


#include "esp_system.h"
#include "stdint.h"
#include "stdbool.h"
#include "lcd.h"

typedef struct
{
  uint16_t x;
  uint16_t y;
} gfx_point_t;

typedef struct
{
  uint16_t width;
  uint16_t height;
} gfx_size_t;

typedef struct
{
	gfx_point_t point;
	gfx_size_t size;
	uint16_t backcolor;
	uint16_t forecolor;
	uint16_t bordercolor;
	uint8_t border_thickness;
	char *text;
} gfx_button_t;

typedef struct
{
	gfx_point_t point;
	gfx_size_t size;
	uint16_t backcolor;
	uint16_t forecolor;
	uint16_t bordercolor;
	uint8_t border_thickness;
	char *text;
} gfx_label_t;

typedef struct
{
	gfx_point_t point;
	gfx_size_t size;
	uint16_t backcolor;
	uint16_t forecolor;
	uint16_t bordercolor;
	uint8_t border_thickness;
	uint16_t maxValue;
	uint16_t minValue;
	uint16_t value;
} gfx_progressbar_t;

typedef struct
{
	gfx_point_t point;
	gfx_size_t size;
	uint16_t backcolor;
	uint16_t forecolor;
	uint16_t bordercolor;
	uint8_t border_thickness;
	uint16_t maxValue;
	uint16_t minValue;
	uint16_t value;
} gfx_keyboard_t;


void gfx_init(touch_callback_t ptr_callback);
uint8_t gfx_ts_read(uint16_t *x, uint16_t *y);
void gfx_backligh(uint8_t on);
uint16_t gfx_get_width(void);
uint16_t gfx_get_height(void);
void gfx_draw_button(gfx_button_t *button);
void gfx_draw_label(gfx_label_t *label);
void gfx_draw_progressbar(gfx_progressbar_t *pb);
void gfx_draw_keyboard(gfx_keyboard_t *kb);



//#include "font_8x12.h"
#include "font_16x32.h"
#include "lcd.h"
#include "gfx.h"

void gfx_init(touch_callback_t ptr_callback)
{
  lcd_init(ptr_callback);
  lcd_clear(GFX_COLOR_BACKGROUND);
  lcd_display_on();
}

void gfx_backligh(uint8_t on)
{
  lcd_backlight(on);
}

uint16_t gfx_get_width(void)
{
  return lcd_get_width();
}
uint16_t gfx_get_height(void)
{
  return lcd_get_height();
}

void gfx_draw_button(gfx_button_t *button)
{
	lcd_fill(button->point.x,
				 button->point.y,
				 button->point.x + button->size.width,
				 button->point.y + button->size.height,
				 button->backcolor);
  if (button->border_thickness > 0) {
	  lcd_draw_rect(button->point.x,
                  	  	button->point.y,
						button->point.x + button->size.width,
						button->point.y + button->size.height,
						button->bordercolor);
  }

  uint16_t x = button->point.x + (button->size.width - lcd_get_text_size_w(font_16x32, button->text)) / 2;
  uint16_t y = button->point.y + (button->size.height - lcd_get_text_size_h(font_16x32, button->text)) / 2;

  lcd_draw_text(x, y, button->text, font_16x32, button->backcolor, button->forecolor);
}

void gfx_draw_label(gfx_label_t *label)
{
	lcd_fill(label->point.x,
			label->point.y,
			label->point.x + label->size.width,
			label->point.y + label->size.height,
			label->backcolor);
  if (label->border_thickness > 0) {
	  lcd_draw_rect(label->point.x,
			  label->point.y,
			  label->point.x + label->size.width,
			  label->point.y + label->size.height,
			  label->bordercolor);
  }

  uint16_t x = label->point.x + (label->size.width - lcd_get_text_size_w(font_16x32, label->text)) / 2;
  uint16_t y = label->point.y + (label->size.height - lcd_get_text_size_h(font_16x32, label->text)) / 2;

  lcd_draw_text(x,
                	y,
					label->text,
	                font_16x32,
					label->backcolor,
					label->forecolor);
}

void gfx_draw_progressbar(gfx_progressbar_t *pb)
{
	uint16_t percent = (pb->size.width * pb->value) / 100;

	lcd_fill(pb->point.x,
			pb->point.y,
			pb->point.x + percent,
			pb->point.y + pb->size.height,
			pb->forecolor);

	if (pb->border_thickness > 0) {
	  lcd_draw_rect(pb->point.x,
			  pb->point.y,
			  pb->point.x + pb->size.width,
			  pb->point.y + pb->size.height,
			  pb->bordercolor);
	}

	char str[30];
	sprintf(str, "%d", pb->value);

	uint16_t x = pb->point.x + (pb->size.width - lcd_get_text_size_w(font_16x32, str)) / 2;
	uint16_t y = pb->point.y + (pb->size.height - lcd_get_text_size_h(font_16x32, str)) / 2;

	lcd_draw_text(x,
                      y,
                      str,
                      font_16x32,
                      pb->backcolor,
                      pb->forecolor);
}


void gfx_draw_keyboard(gfx_keyboard_t *kb)
{
  /*
	uint16_t percent = (pb->size.width * pb->value) / 100;

	lcd_fill(pb->point.x,
			pb->point.y,
			pb->point.x + percent,
			pb->point.y + pb->size.height,
			pb->forecolor);

	if (pb->border_thickness > 0) {
	  lcd_draw_rect(pb->point.x,
			  pb->point.y,
			  pb->point.x + pb->size.width,
			  pb->point.y + pb->size.height,
			  pb->bordercolor);
	}

	char str[30];
	sprintf(str, "%d", pb->value);

	uint16_t x = pb->point.x + (pb->size.width - lcd_get_text_size_w(font_16x32, str)) / 2;
	uint16_t y = pb->point.y + (pb->size.height - lcd_get_text_size_h(font_16x32, str)) / 2;

	lcd_draw_text(x,
					y,
					str,
					font_16x32,
					pb->backcolor,
					pb->forecolor);
  */
}

uint8_t gfx_ts_read(uint16_t *x, uint16_t *y)
{
  return lcd_ts_read(x, y);
}
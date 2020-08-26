
#include "gfx.h"
#include "gfx_menu.h"
#include "string.h"
#include "stdlib.h"

void gfx_menu_main(void)
{
/*
  gfx_label_t label = {
                  .size.width = 200,
                  .size.height = 60,
                  .point.x = (gfx_get_width() - 200) / 2,
                  .point.y = 100,
                  .backcolor = GFX_COLOR_BACKGROUND,
                  .forecolor = GFX_COLOR_BLACK,
                  .bordercolor = GFX_COLOR_BLACK,
                  .border_thickness = 0,
                  .text = "label"
  };
  gfx_draw_label(&label);


  gfx_progressbar_t pb = {
                  .size.width = gfx_get_width() - 10,
                  .size.height = 60,
                  .point.x = 5,
                  .point.y = 200,
                  .backcolor = GFX_COLOR_BACKGROUND,
                  .forecolor = GFX_COLOR_BUTTON_BACKCOLOR,
                  .bordercolor = GFX_COLOR_BUTTON_BACKCOLOR,
                  .border_thickness = 1,
                  .maxValue = 100,
                  .minValue = 0,
                  .value = 76
  };
  gfx_draw_progressbar(&pb);

  gfx_button_t bBack = {
                  .size.width = 200,
                  .size.height = 60,
                  .point.x = 0,
                  .point.y = gfx_get_height() - 60,
                  .backcolor = GFX_COLOR_BUTTON_BACKCOLOR,
                  .forecolor = GFX_COLOR_BUTTON_FORECOLOR,
                  .bordercolor = GFX_COLOR_BLACK,
                  .border_thickness = 0,
                  .text = "BACK"
  };
  gfx_draw_button(&bBack);


  gfx_button_t bNext = {
                  .size.width = 200,
                  .size.height = 60,
                  .point.x = gfx_get_width() - 200,
                  .point.y = gfx_get_height() - 60,
                  .backcolor = GFX_COLOR_BUTTON_BACKCOLOR,
                  .forecolor = GFX_COLOR_BUTTON_FORECOLOR,
                  .bordercolor = GFX_COLOR_BLACK,
                  .border_thickness = 0,
                  .text = "NEXT"
  };
  gfx_draw_button(&bNext);

*/  
  gfx_button_t bClear = {
                  .size.width = 200,
                  .size.height = 60,
                  .point.x = gfx_get_width() - 200,
                  .point.y = gfx_get_height() - 60,
                  .backcolor = GFX_COLOR_BUTTON_BACKCOLOR,
                  .forecolor = GFX_COLOR_BUTTON_FORECOLOR,
                  .bordercolor = GFX_COLOR_BLACK,
                  .border_thickness = 0,
                  .text = "CLEAR"
  };
  gfx_draw_button(&bClear);
  //printTouchXY(0,0);
}

void printTouchXY(uint8_t count, uint16_t x, uint16_t y)
{ 
  char *string = (char *) malloc(20 * sizeof(char));
  sprintf(string, "%d %d", x, y);
  gfx_label_t label = {
                  .size.width = 200,
                  .size.height = 60,
                  .point.x = 10,
                  .point.y = 10,
                  .backcolor = GFX_COLOR_BACKGROUND,
                  .forecolor = GFX_COLOR_BLACK,
                  .bordercolor = GFX_COLOR_BLACK,
                  .border_thickness = 0,
                  .text = "ready"
  };
  label.text = string;
  gfx_draw_label(&label);
  free(string);
}
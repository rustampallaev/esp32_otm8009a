
#include "esp_system.h"
#include "delay.h"
#include "lcd.h"
#include "font_8x12.h"
#include "string.h"
#include "stdlib.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "mcp23s17.h"

#define LCD_CS_PIN      0
#define LCD_RS_PIN      2
#define LCD_WR_PIN      16
#define LCD_RD_PIN      4
#define LCD_BL_PIN      19
#define LCD_RST_PIN     18//17

#define TS_I2C_SCL_PIN  22
#define TS_I2C_SDA_PIN  21
#define TS_I2C_NUMBER   I2C_NUM_0
#define I2C_MASTER_ACK 	0x00
#define I2C_MASTER_NACK 0x01
#define ACK_CHECK_EN    0x01
#define ACK_CHECK_DIS   0x00
#define ACK_VAL					0x00
#define NACK_VAL        0x01

#define GPIO_INPUT_IO_0       34
#define GPIO_INPUT_PIN_SEL    (1ULL<<GPIO_INPUT_IO_0)
#define ESP_INTR_FLAG_DEFAULT 0

#define LCD_CS_CLR      gpio_set_level(LCD_CS_PIN, 0);
#define LCD_CS_SET      gpio_set_level(LCD_CS_PIN, 1);

#define LCD_RS_CLR      gpio_set_level(LCD_RS_PIN, 0);
#define LCD_RS_SET      gpio_set_level(LCD_RS_PIN, 1);

#define LCD_WR_CLR      gpio_set_level(LCD_WR_PIN, 0);
#define LCD_WR_SET      gpio_set_level(LCD_WR_PIN, 1);

#define LCD_RD_CLR      gpio_set_level(LCD_RD_PIN, 0);
#define LCD_RD_SET      gpio_set_level(LCD_RD_PIN, 1);

#define LCD_BL_CLR      gpio_set_level(LCD_BL_PIN, 0);
#define LCD_BL_SET      gpio_set_level(LCD_BL_PIN, 1);

#define LCD_RST_CLR     gpio_set_level(LCD_RST_PIN, 0);
#define LCD_RST_SET     gpio_set_level(LCD_RST_PIN, 1);

#define LCD_WIDTH       (800) 
#define LCD_HEIGHT      (480)

touch_callback_t ts_callback = NULL;

static void lcd_gpio_init(touch_callback_t ptr_callback);
static void lcd_ts_gpio_init_isr(touch_callback_t ptr_callback);
static void IRAM_ATTR lcd_ts_gpio_isr_handler(void* arg);

void lcd_ts_i2c_init();
void lcd_ts_i2c_read_reg(uint8_t reg, uint8_t* data, size_t size);


void lcd_setup(void);
void lcd_dataout(uint16_t value);
void lcd_reset(void);
void lcd_write_cmd(uint16_t data);
void lcd_write_data(uint16_t data);
void lcd_write_reg(uint16_t reg, uint16_t value);
void lcd_draw_point(uint16_t x,uint16_t y, uint16_t color);
void lcd_draw_line_h(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color);
void lcd_set_window(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd);
void lcd_set_cursor(uint16_t xpos, uint16_t ypos);
void lcd_idle_mode_on(void);
void lcd_idle_mode_off(void);
void lcd_tearing_on(void);
void lcd_tearing_off(void);

#define FT5216_ADDR     0x70

#define TD_STATUS       0x02

#define TOUCH1_XH       0x03
#define TOUCH1_XL       0x04
#define TOUCH1_YH       0x05 
#define TOUCH1_YL       0x06

#define TOUCH2_XH       0x09
#define TOUCH2_XL       0x0A
#define TOUCH2_YH       0x0B
#define TOUCH2_YL       0x0C

#define TOUCH3_XH       0x0F
#define TOUCH3_XL       0x10
#define TOUCH3_YH       0x11
#define TOUCH3_YL       0x12

#define TOUCH4_XH       0x15
#define TOUCH4_XL       0x16
#define TOUCH4_YH       0x17
#define TOUCH4_YL       0x18

#define TOUCH5_XH       0x1B 
#define TOUCH5_XL       0x1C
#define TOUCH5_YH       0x1D
#define TOUCH5_YL       0x1E

static uint8_t ts_req_x[10] = {
  TOUCH1_XH, TOUCH1_XL,
  TOUCH2_XH, TOUCH2_XL,
  TOUCH3_XH, TOUCH3_XL,
  TOUCH4_XH, TOUCH4_XL,
  TOUCH5_XH, TOUCH5_XL
};
static uint8_t ts_req_y[10] = {
  TOUCH1_YH, TOUCH1_YL,
  TOUCH2_YH, TOUCH2_YL,
  TOUCH3_YH, TOUCH3_YL,
  TOUCH4_YH, TOUCH4_YL,
  TOUCH5_YH, TOUCH5_YL
};

static void lcd_gpio_init(touch_callback_t ptr_callback)
{
  gpio_pad_select_gpio(LCD_CS_PIN);
  gpio_set_direction(LCD_CS_PIN, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(LCD_RS_PIN);
  gpio_set_direction(LCD_RS_PIN, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(LCD_WR_PIN);
  gpio_set_direction(LCD_WR_PIN, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(LCD_RD_PIN);
  gpio_set_direction(LCD_RD_PIN, GPIO_MODE_OUTPUT);

  gpio_pad_select_gpio(LCD_BL_PIN);
  gpio_set_direction(LCD_BL_PIN, GPIO_MODE_OUTPUT);
  
  gpio_pad_select_gpio(LCD_RST_PIN);
  gpio_set_direction(LCD_RST_PIN, GPIO_MODE_OUTPUT);

  lcd_ts_gpio_init_isr(ptr_callback);
}

static void lcd_ts_gpio_init_isr(touch_callback_t ptr_callback)
{
  gpio_config_t io_conf;
  io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  io_conf.mode = GPIO_MODE_INPUT;
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
  gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);
  gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
  gpio_isr_handler_add(GPIO_INPUT_IO_0, lcd_ts_gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
  
  ts_callback = ptr_callback;
}

static void IRAM_ATTR lcd_ts_gpio_isr_handler(void* arg)
{
  uint32_t gpio_num = (uint32_t) arg;
  if(ts_callback != NULL) {
    (*ts_callback)(gpio_num);
  }
}

void lcd_ts_i2c_init(void)
{
  i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = TS_I2C_SDA_PIN,
		.scl_io_num = TS_I2C_SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_DISABLE,
		.scl_pullup_en = GPIO_PULLUP_DISABLE,
		.master.clk_speed = 400000
	};
	i2c_param_config(TS_I2C_NUMBER, &i2c_config);
	i2c_driver_install(TS_I2C_NUMBER, I2C_MODE_MASTER, 0, 0, 0);
}
/*
void lcd_ts_i2c_read_reg(uint8_t address, uint8_t reg, uint8_t* data, size_t size)
{
    if (size == 0) {
        return;
    }
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( address << 1 ), ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( address << 1 ) | I2C_MASTER_READ, ACK_CHECK_EN);
    if (size > 1) {
        i2c_master_read(cmd, data, size - 1, ACK_VAL);
    }
    i2c_master_read_byte(cmd, data + size - 1, NACK_VAL);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(TS_I2C_NUMBER, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
}
*/
void lcd_ts_i2c_read_reg(uint8_t reg, uint8_t* data, size_t size)
{
	if (size == 0) {
		return;
	}
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
	i2c_master_write_byte(cmd, FT5216_ADDR | I2C_MASTER_WRITE, ACK_CHECK_EN);
	i2c_master_write_byte(cmd, reg, ACK_CHECK_EN);
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(TS_I2C_NUMBER, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);

	cmd = i2c_cmd_link_create();
	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, FT5216_ADDR | I2C_MASTER_READ, ACK_CHECK_EN);
	if (size > 1) {
		i2c_master_read(cmd, data, size - 1, ACK_VAL);
	}
	i2c_master_read_byte(cmd, data + size - 1,   NACK_VAL);
	ESP_ERROR_CHECK(i2c_master_stop(cmd));
	i2c_master_stop(cmd);
	i2c_master_cmd_begin(TS_I2C_NUMBER, cmd, 1000/portTICK_PERIOD_MS);
	i2c_cmd_link_delete(cmd);
}


uint8_t lcd_ts_read(uint16_t *x, uint16_t *y)
{
  uint8_t count = 0, h = 0, l = 0;
  lcd_ts_i2c_read_reg(TD_STATUS, &count, 1);
  count = count & 0x0F;
  
  if (count == 0)
    return count;
  
  for(uint8_t i = 0; i < count; i ++) {
    lcd_ts_i2c_read_reg(ts_req_x[i*2], &h, 1);
    lcd_ts_i2c_read_reg(ts_req_x[i*2+1], &l, 1);
    x[i] = (uint16_t)(h & 0x0F) << 8 | l;
    
    lcd_ts_i2c_read_reg(ts_req_y[i*2], &h, 1);
    lcd_ts_i2c_read_reg(ts_req_y[i*2+1], &l, 1);
    y[i] = (uint16_t)(h & 0x0F) << 8 | l;
  }
  return count;
}

void lcd_dataout(uint16_t data)
{
    mcp23S17_WritePorts(0x00, data);
}

void lcd_reset(void)
{
  LCD_RST_SET;
  delay_hw(100);
  LCD_RST_CLR;
  delay_hw(500);	
  LCD_RST_SET;
  delay_hw(200);
}

void lcd_write_cmd(uint16_t data)
{ 
  LCD_CS_CLR; 
  LCD_RD_SET; 
  LCD_RS_CLR;

  LCD_WR_CLR;
  
  lcd_dataout(data); 

  //__NOP();__NOP();__NOP();__NOP();__NOP();
  
  LCD_WR_SET; 
  LCD_CS_SET; 
}

void lcd_write_data(uint16_t data)
{
  LCD_CS_CLR; 
  LCD_RD_SET; 
  LCD_RS_SET;

  LCD_WR_CLR;
  
  lcd_dataout(data);

  //__NOP();__NOP();__NOP();__NOP();__NOP();
  LCD_WR_SET;

  LCD_CS_SET;
}

void lcd_write_reg(uint16_t reg, uint16_t value)
{	
  lcd_write_cmd(reg);  
  lcd_write_data(value);	    		 
}	 

void lcd_idle_mode_on(void)
{
  lcd_write_cmd(0x39); 
}

void lcd_idle_mode_off(void)
{
  lcd_write_cmd(0x34); 
}

void lcd_tearing_on(void)
{
  lcd_write_cmd(0x35);
  lcd_write_data(0x01);  
}

void lcd_tearing_off(void)
{
  lcd_write_cmd(0x34);
}

void lcd_clear(uint16_t color)
{
  uint16_t i, j;

  lcd_set_window(0, 0, lcd_get_width(), lcd_get_height());
  
  LCD_CS_CLR; 
  LCD_RS_SET;
  LCD_RD_SET; 
  lcd_dataout(color);
  for(i = 0; i <= lcd_get_width(); i++)
  {
    for(j = 0; j <= lcd_get_height();j++)
    {
      LCD_WR_CLR;
      LCD_WR_SET;
    }
  }

  LCD_CS_SET;
}

void lcd_fill(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  uint16_t i, j;
  lcd_set_window(x0, y0, x1, y1);
  
  LCD_CS_CLR; 
  LCD_RS_SET;
  LCD_RD_SET; 
  lcd_dataout(color);
  for(i = 0; i <= (x1 - x0); i++)
  {
    for(j = 0; j <= (y1 - y0);j++)
    {
      LCD_WR_CLR;
      LCD_WR_SET;
    }
  }

  LCD_CS_SET;
}

void lcd_set_window(uint16_t xStar, uint16_t yStar, uint16_t xEnd, uint16_t yEnd)
{
  lcd_write_cmd(0x2a);   
  lcd_write_data(xStar>>8);
  lcd_write_data(xStar&0xff);
  lcd_write_data(xEnd>>8);
  lcd_write_data(xEnd&0xff);

  lcd_write_cmd(0x2b);   
  lcd_write_data(yStar>>8);
  lcd_write_data(yStar&0xff);
  lcd_write_data(yEnd>>8);
  lcd_write_data(yEnd&0xff);

  lcd_write_cmd(0x2c);
}

void lcd_set_cursor(uint16_t xpos, uint16_t ypos)
{	  	    			
  lcd_set_window(xpos, ypos, xpos, ypos);	
}

void lcd_setup(void)
{
  lcd_reset();
  /*
  for(uint8_t i = 0; i < 16; i++) {
    lcd_write_cmd(1<<i);
    __NOP();
  }
  */
  
  lcd_write_cmd(0x11);
  lcd_write_data(0x00);   
  delay_hw(40);
  lcd_write_cmd(0xB0);
  lcd_write_data(0x04);
  lcd_write_cmd(0xB3);
  lcd_write_data(0x02);
  lcd_write_data(0x00);
  lcd_write_cmd(0xB6);
  lcd_write_data(0x52);
  lcd_write_data(0x83);
  lcd_write_cmd(0xB7);
  lcd_write_data(0x80);
  lcd_write_data(0x72);
  lcd_write_data(0x11);
  lcd_write_data(0x25);
  lcd_write_cmd(0xB8);
  lcd_write_data(0x00);
  lcd_write_data(0x0F);
  lcd_write_data(0x0F);
  lcd_write_data(0xFF);
  lcd_write_data(0xFF);
  lcd_write_data(0xC8);
  lcd_write_data(0xC8);
  lcd_write_data(0x02);
  lcd_write_data(0x18);
  lcd_write_data(0x10);
  lcd_write_data(0x10);
  lcd_write_data(0x37);
  lcd_write_data(0x5A);
  lcd_write_data(0x87);
  lcd_write_data(0xBE);
  lcd_write_data(0xFF);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_cmd(0xB9);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_cmd(0xBD);
  lcd_write_data(0x00);
  lcd_write_cmd(0xC0);
  lcd_write_data(0x02);
  lcd_write_data(0x76);
  lcd_write_cmd(0xC1);
  lcd_write_data(0x63);
  lcd_write_data(0x31);
  lcd_write_data(0x00);
  lcd_write_data(0x27);
  lcd_write_data(0x27);
  lcd_write_data(0x32);
  lcd_write_data(0x12);
  lcd_write_data(0x28);
  lcd_write_data(0x4E);
  lcd_write_data(0x10);
  lcd_write_data(0xA5);
  lcd_write_data(0x0F);
  lcd_write_data(0x58);
  lcd_write_data(0x21);
  lcd_write_data(0x01);
  lcd_write_cmd(0xC2);
  lcd_write_data(0x28);
  lcd_write_data(0x06);
  lcd_write_data(0x06);
  lcd_write_data(0x01);
  lcd_write_data(0x03);
  lcd_write_data(0x00);
  lcd_write_cmd(0xC3);
  lcd_write_data(0x40);
  lcd_write_data(0x00);
  lcd_write_data(0x03);
  lcd_write_cmd(0xC4);
  lcd_write_data(0x00);
  lcd_write_data(0x01);
  lcd_write_cmd(0xC6);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_cmd(0xC7);
  lcd_write_data(0x11);
  lcd_write_data(0x8D);
  lcd_write_data(0xA0);
  lcd_write_data(0xF5);
  lcd_write_data(0x27);
  lcd_write_cmd(0xC8);
  lcd_write_data(0x02);
  lcd_write_data(0x13);
  lcd_write_data(0x18);
  lcd_write_data(0x25);
  lcd_write_data(0x34);
  lcd_write_data(0x4E);
  lcd_write_data(0x36);
  lcd_write_data(0x23);
  lcd_write_data(0x17);
  lcd_write_data(0x0E);
  lcd_write_data(0x0C);
  lcd_write_data(0x02);
  lcd_write_data(0x02);
  lcd_write_data(0x13);
  lcd_write_data(0x18);
  lcd_write_data(0x25);
  lcd_write_data(0x34);
  lcd_write_data(0x4E);
  lcd_write_data(0x36);
  lcd_write_data(0x23);
  lcd_write_data(0x17);
  lcd_write_data(0x0E);
  lcd_write_data(0x0C);
  lcd_write_data(0x02);
  lcd_write_cmd(0xC9);
  lcd_write_data(0x02);
  lcd_write_data(0x13);
  lcd_write_data(0x18);
  lcd_write_data(0x25);
  lcd_write_data(0x34);
  lcd_write_data(0x4E);
  lcd_write_data(0x36);
  lcd_write_data(0x23);
  lcd_write_data(0x17);
  lcd_write_data(0x0E);
  lcd_write_data(0x0C);
  lcd_write_data(0x02);
  lcd_write_data(0x02);
  lcd_write_data(0x13);
  lcd_write_data(0x18);
  lcd_write_data(0x25);
  lcd_write_data(0x34);
  lcd_write_data(0x4E);
  lcd_write_data(0x36);
  lcd_write_data(0x23);
  lcd_write_data(0x17);
  lcd_write_data(0x0E);
  lcd_write_data(0x0C);
  lcd_write_data(0x02);
  lcd_write_cmd(0xCA);
  lcd_write_data(0x02);
  lcd_write_data(0x13);
  lcd_write_data(0x18);
  lcd_write_data(0x25);
  lcd_write_data(0x34);
  lcd_write_data(0x4E);
  lcd_write_data(0x36);
  lcd_write_data(0x23);
  lcd_write_data(0x17);
  lcd_write_data(0x0E);
  lcd_write_data(0x0C);
  lcd_write_data(0x02);
  lcd_write_data(0x02);
  lcd_write_data(0x13);
  lcd_write_data(0x18);
  lcd_write_data(0x25);
  lcd_write_data(0x34);
  lcd_write_data(0x4E);
  lcd_write_data(0x36);
  lcd_write_data(0x23);
  lcd_write_data(0x17);
  lcd_write_data(0x0E);
  lcd_write_data(0x0C);
  lcd_write_data(0x02);
  lcd_write_cmd(0xD0);
  lcd_write_data(0xA9);
  lcd_write_data(0x03);
  lcd_write_data(0xCC);
  lcd_write_data(0xA5);
  lcd_write_data(0x00);
  lcd_write_data(0x53);
  lcd_write_data(0x20);
  lcd_write_data(0x10);
  lcd_write_data(0x01);
  lcd_write_data(0x00);
  lcd_write_data(0x01);
  lcd_write_data(0x01);
  lcd_write_data(0x00);
  lcd_write_data(0x03);
  lcd_write_data(0x01);
  lcd_write_data(0x00);
  lcd_write_cmd(0xD1);
  lcd_write_data(0x18);
  lcd_write_data(0x0C);
  lcd_write_data(0x23);
  lcd_write_data(0x03);
  lcd_write_data(0x75);
  lcd_write_data(0x02);
  lcd_write_data(0x50);
  lcd_write_cmd(0xD3);
  lcd_write_data(0x33);
  lcd_write_cmd(0xD5);
  lcd_write_data(0x2a);
  lcd_write_data(0x2a);
  lcd_write_cmd(0xD6);
  lcd_write_data(0x28);//a8
  lcd_write_cmd(0xD7);
  lcd_write_data(0x01);
  lcd_write_data(0x00);
  lcd_write_data(0xAA);
  lcd_write_data(0xC0);
  lcd_write_data(0x2A);
  lcd_write_data(0x2C);
  lcd_write_data(0x22);
  lcd_write_data(0x12);
  lcd_write_data(0x71);
  lcd_write_data(0x0A);
  lcd_write_data(0x12);
  lcd_write_data(0x00);
  lcd_write_data(0xA0);
  lcd_write_data(0x00);
  lcd_write_data(0x03);
  lcd_write_cmd(0xD8);
  lcd_write_data(0x44);
  lcd_write_data(0x44);
  lcd_write_data(0x22);
  lcd_write_data(0x44);
  lcd_write_data(0x21);
  lcd_write_data(0x46);
  lcd_write_data(0x42);
  lcd_write_data(0x40);
  lcd_write_cmd(0xD9);
  lcd_write_data(0xCF);
  lcd_write_data(0x2D);
  lcd_write_data(0x51);
  lcd_write_cmd(0xDA);
  lcd_write_data(0x01);
  lcd_write_cmd(0xDE);
  lcd_write_data(0x01);
  lcd_write_data(0x51);//58
  lcd_write_cmd(0xE1);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_cmd(0xE6);
  lcd_write_data(0x55);//58
  lcd_write_cmd(0xF3);
  lcd_write_data(0x06);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x24);
  lcd_write_data(0x00);
  lcd_write_cmd(0xF8);
  lcd_write_data(0x00);
  lcd_write_cmd(0xFA);
  lcd_write_data(0x01);
  lcd_write_cmd(0xFB);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_cmd(0xFC);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_cmd(0xFD);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x70);
  lcd_write_data(0x00);
  lcd_write_data(0x72);
  lcd_write_data(0x31);
  lcd_write_data(0x37);
  lcd_write_data(0x70);
  lcd_write_data(0x32);
  lcd_write_data(0x31);
  lcd_write_data(0x07);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_cmd(0xFE);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x20);
  lcd_write_cmd(0xB0);
  lcd_write_data(0x04); //04
  delay_hw(40);
  lcd_write_cmd(0x35);
  lcd_write_data(0x00);
  lcd_write_cmd(0x44);
  lcd_write_data(0x00);
  lcd_write_cmd(0x36);
  lcd_write_data(0x00);
  lcd_write_cmd(0x3A);
  lcd_write_data(0x55);
  lcd_write_cmd(0x2A);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x01);
  lcd_write_data(0xDF);
  lcd_write_cmd(0x2B);
  lcd_write_data(0x00);
  lcd_write_data(0x00);
  lcd_write_data(0x03);
  lcd_write_data(0x1F);
  lcd_write_cmd(0x29);
  lcd_write_data(0x00);     
  delay_hw(180);
  lcd_write_cmd(0x2C);
  lcd_write_data(0x00); 
  delay_hw(10); 
  lcd_display_off();

  lcd_write_cmd(0x36);
  lcd_write_data(0x60);
}


void lcd_init(touch_callback_t ptr_callback)
{
  lcd_gpio_init(ptr_callback);

  MCP23S17_Initalize(0x00);
  mcp23S17_GpioMode(0x00, 0x0000);  		// PORTB | PORTA      

  lcd_ts_i2c_init();

  lcd_setup();
}

void lcd_display_on(void)
{
  lcd_write_cmd(0x29);
}

void lcd_display_off(void)
{
  lcd_write_cmd(0x28);
}

void lcd_backlight(uint8_t on)
{
  if(on) { LCD_BL_SET; }
  else { LCD_BL_CLR; }
}

uint16_t lcd_get_width(void) 
{
  return LCD_WIDTH - 1;
}
uint16_t lcd_get_height(void)
{
  return LCD_HEIGHT - 1;
}

void lcd_draw_point(uint16_t x, uint16_t y, uint16_t color)
{
  lcd_set_cursor(x,y);
  lcd_write_data(color); 
}

void lcd_draw_line_h(uint16_t x0, uint16_t x1, uint16_t y, uint16_t color)
{
  uint16_t i;
  lcd_set_window(x0, y, x1, y);
  
  LCD_CS_CLR; 
  LCD_RS_SET;
  LCD_RD_SET; 
  lcd_dataout(color);
  for(i = 0; i < (x1 - x0); i++)
  {
    LCD_WR_CLR;
    LCD_WR_SET;
  }

  LCD_CS_SET;
}

void lcd_draw_line_v(uint16_t y0, uint16_t y1, uint16_t x, uint16_t color)
{
  uint16_t i;
  lcd_set_window(x, y0, x, y1);
  
  LCD_CS_CLR; 
  LCD_RS_SET;
  LCD_RD_SET; 
  lcd_dataout(color);
  for(i = 0; i < (y1 - y0); i++)
  {
    LCD_WR_CLR;
    LCD_WR_SET;
  }

  LCD_CS_SET;
}

void lcd_draw_line(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  uint16_t y = 0;
  float slope = (y1 - y0)/(x1 - x0);
  for(uint16_t x = x0; x < x1; x++){
    y = slope * x + y0;
    lcd_draw_point(x, y, color);
  }
}

void lcd_draw_rect(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1, uint16_t color)
{
  lcd_draw_line_h(x0, x1, y0, color);
  lcd_draw_line_h(x0, x1, y1, color);
  
  lcd_draw_line_v(y0, y1, x0, color);
  lcd_draw_line_v(y0, y1, x1, color);
}
/*
void lcd_draw_char(uint16_t x, uint16_t y, const char symbol, uint16_t backcolor, uint16_t forecolor)
{
  int slen = 1;
  uint16_t *array = (uint16_t *) malloc(slen * FONT_8x12_W * FONT_8x12_H * sizeof(uint16_t));
  
  uint16_t ai = 0;

  for(int j = 0; j < FONT_8x12_H; j ++)
  {
    for(int i = 0; i < FONT_8x12_W; i ++)
    {
      int offset = (symbol - 32) * FONT_8x12_H;
      ai++;
      if ( ( (font_8x12[offset + j] >> (FONT_8x12_W-i)) &1 ) != 0)
              array[ai] = forecolor;
      else array[ai] = backcolor;
    }
  }
  
  lcd_set_window(x, y, x + (slen * FONT_8x12_W) - 1, y + FONT_8x12_H - 1);
  
  LCD_CS_CLR; 
  LCD_RS_SET;
  LCD_RD_SET; 
  for(int i = 0; i < ai; i++)
  {
    lcd_dataout(array[i]);
    LCD_WR_CLR;
    LCD_WR_SET;
  }

  LCD_CS_SET;
}
*/
uint16_t lcd_get_text_size_w(const char *font, const char *text)
{
  return (strlen(text) * font[0]);
}

uint16_t lcd_get_text_size_h(const char *font, const char *text)
{
  return (font[1]);
}


void lcd_draw_char(uint16_t x, uint16_t y, const char ch, const char *font, uint16_t backcolor, uint16_t forecolor)
{
  uint8_t width = font[0];
  uint8_t height = font[1];
  uint16_t *array = (uint16_t *) malloc(width * height * sizeof(uint16_t));
  
  uint16_t ai = 0;
  uint16_t row = 0;
  uint16_t offset = (ch - 32) * height * 2 + 2;
  
  for(int j = 0; j < height * 2; j = j + 2)
  {
    row = (uint16_t)font[offset + j + 0] << 8 | (uint16_t)font[offset + j + 1];
    for(int i = 0; i < width; i ++)
    {
      if ( ( (row >> (width - i)) &1 ) != 0)
              array[ai] = forecolor;
      else array[ai] = backcolor;
      ai++;
    }
  }
  
  lcd_set_window(x, y, x + width - 1, y + height - 1);
  
  LCD_CS_CLR; 
  LCD_RS_SET;
  LCD_RD_SET;
  for(int i = 0; i < ai; i++)
  {
    lcd_dataout(array[i]);
    LCD_WR_CLR;
    LCD_WR_SET;
  }

  LCD_CS_SET;
  
  free(array);
}

void lcd_draw_text(uint16_t x, uint16_t y, const char *text, const char *font, uint16_t backcolor, uint16_t forecolor)
{
  for(int i = 0; i < strlen(text); i++){
    lcd_draw_char(x + i * font[0], y, text[i], font, backcolor, forecolor);
  }
}

void lcd_draw_text_2(uint16_t x, uint16_t y, const char *text, const char *font, uint16_t backcolor, uint16_t forecolor)
{
  uint8_t width = font[0];
  uint8_t height = font[1];
  int slen = strlen(text);
  uint16_t *array = (uint16_t *) malloc(slen * width * height * sizeof(uint16_t));
  //uint16_t array[500];
  
  uint16_t ai = 0;

  for(int j = 0; j < height; j ++)
  {
    for(int k = 0; k < slen; k++)
    {
      for(int i = 0; i < width; i ++)
      {
        uint8_t ch = text[k];
        int offset = (ch - 32) * height + 2;
        if ( ( (font[offset + j] >> (width-i)) &1 ) != 0)
                array[ai] = forecolor;
        else array[ai] = backcolor;
        ai++;
      }
    }
  }
  
  lcd_set_window(x, y, x + (slen * width) - 1, y + height - 1);
  
  LCD_CS_CLR; 
  LCD_RS_SET;
  LCD_RD_SET;
  for(int i = 0; i < ai; i++)
  {
    lcd_dataout(array[i]);
    LCD_WR_CLR;
    LCD_WR_SET;
  }

  LCD_CS_SET;
  
  free(array);
}
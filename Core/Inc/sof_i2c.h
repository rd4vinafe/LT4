#ifndef __SOF_I2C_H__
#define __SOF_I2C_H__
#include "stm32f1xx_hal.h"

#define PCF_ADDRES_WRITE 0x4E // (pcf8574A 7E) or 4e
#define PCF_ADDRES_READ  0x4D // Read address for PCF8574

#define LCD_BACKLIGHT    0x08
#define LCD_NO_BACKLIGHT 0x00

#define FUNCTION_SET    0X28
#define DISPLAY_CONTROL 0X0C
#define CLEAR_DISPLAY   0X01
#define ENTRY_MODE      0X06

#define LCD_RETURN_HOME       0x02
#define LCD_CURSOR_OFF        0x0C
#define LCD_UNDERLINE_ON      0x0E
#define LCD_BLINK_CURSOR_ON   0x0F
#define LCD_MOVE_CURSOR_LEFT  0x10
#define LCD_MOVE_CURSOR_RIGHT 0x14
#define LCD_TURN_ON           0x0C
#define LCD_TURN_OFF          0x08
#define LCD_SHIFT_LEFT        0x18
#define LCD_SHIFT_RIGHT       0x1E
#define LCD_LEFT_MODE         0x04
#define LCD_RIGHT_MODE        0x06

#define ADDR_LINE1 0X80
#define ADDR_LINE2 0XC0
#define ADDR_LINE3 0X94
#define ADDR_LINE4 0XD4

#define DEGREE_SYMBOL 0 // Custom character location for degree symbol


void lcd_soft_i2c_init(void);
void lcd_soft_i2c_start(void);
void lcd_soft_i2c_stop(void);
void lcd_soft_i2c_write(uint8_t data_send);
uint8_t lcd_soft_i2c_read(uint8_t addr);
void swap(uint8_t *da);
void lcd_soft_i2c_command(char mdk);
void lcd_soft_i2c_data(char mht);
void lcd_create_custom_char(uint8_t location, uint8_t charmap[]);
void lcd_init_degree_symbol(void);
void lcd_soft_i2c_setup(void);

#endif 












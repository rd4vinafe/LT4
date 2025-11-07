#include "sof_i2c.h" 
void delay_us(uint32_t us)
{
    for (volatile uint32_t i = 0; i < us * 10; i++);
}
void lcd_soft_i2c_init(void)
{
    RCC->APB2ENR |= (1<<3);

    // B6 = CRL[31:28], B7 = CRL[27:24]
    GPIOB->CRL &= ~((0xF << 24) | (0xF << 28));
    GPIOB->CRL |=  ((0x7 << 24) | (0x7 << 28)); // Output 50MHz, Open-drain

    GPIOB->ODR |= (1 << 6) | (1 << 7); // Ðua SDA/SCL v? HIGH
}

// ========== GPIO di?u khi?n SDA/SCL ==========
#define LCD_SCL_HIGH()   (GPIOB->ODR |=  (1 << 6))
#define LCD_SCL_LOW()    (GPIOB->ODR &= ~(1 << 6))
#define LCD_SDA_HIGH()   (GPIOB->ODR |=  (1 << 7))
#define LCD_SDA_LOW()    (GPIOB->ODR &= ~(1 << 7))
#define LCD_SDA_READ()   ((GPIOB->IDR >> 7) & 0x01)

// ========== Ð?t/Ð?c SDA/SCL ==========
#define LCD_SCL_PIN(x)   ((x) ? LCD_SCL_HIGH() : LCD_SCL_LOW())
#define LCD_SDA_PIN(x)   ((x) ? LCD_SDA_HIGH() : LCD_SDA_LOW())

// ========== START/STOP/WRITE/READ ==========
void lcd_soft_i2c_start(void) {
    LCD_SCL_PIN(1);
    LCD_SDA_PIN(1);
    delay_us(10);
    LCD_SDA_PIN(0);
    delay_us(10);
    LCD_SCL_PIN(0);
}

void lcd_soft_i2c_stop(void) {
    delay_us(10);
    LCD_SCL_PIN(1);
    delay_us(10);
    LCD_SDA_PIN(1);
}

void lcd_soft_i2c_write(uint8_t data_send) {
    for (int i = 0; i < 8; i++) {
        delay_us(10);
        LCD_SDA_PIN((data_send & 0x80) ? 1 : 0);
        data_send <<= 1;
        LCD_SCL_PIN(1);
        delay_us(10);
        LCD_SCL_PIN(0);
    }
    delay_us(20);
    LCD_SCL_PIN(1);
    delay_us(25);
    LCD_SCL_PIN(0);
    LCD_SDA_PIN(0);
    delay_us(25);
}

uint8_t lcd_soft_i2c_read(uint8_t addr) {
    uint8_t data_receive = 0;
    lcd_soft_i2c_start();
    lcd_soft_i2c_write(addr);
    LCD_SDA_PIN(1); // SDA input
    for (int i = 0; i < 8; i++) {
        LCD_SCL_PIN(1);
        delay_us(10);
        data_receive <<= 1;
        if (LCD_SDA_READ()) {
            data_receive |= 0x01;
        }
        LCD_SCL_PIN(0);
        delay_us(10);
    }
    lcd_soft_i2c_stop();
    return data_receive;
}

void swap(uint8_t *da) {
    *da = (*da << 4) | (*da >> 4);
}

void lcd_soft_i2c_command(char mdk) {
    uint8_t ms_bit_e, ls_bit_e, ms_data, ls_data;
    ms_data = mdk & 0xf0;
    ls_data = mdk & 0x0f;
    swap(&ls_data);
    ms_bit_e = ms_data | 0x04;
    ls_bit_e = ls_data | 0x04;

    lcd_soft_i2c_start();
    lcd_soft_i2c_write(PCF_ADDRES_WRITE);
    lcd_soft_i2c_write(ms_bit_e | LCD_BACKLIGHT);
    lcd_soft_i2c_write(ms_data | LCD_BACKLIGHT);
    lcd_soft_i2c_write(ls_bit_e | LCD_BACKLIGHT);
    lcd_soft_i2c_write(ls_data | LCD_BACKLIGHT);
    lcd_soft_i2c_stop();

    if (mdk == CLEAR_DISPLAY || mdk == LCD_RETURN_HOME) delay_us(2000);
    else delay_us(100);
}

void lcd_soft_i2c_data(char mht) {
    uint8_t ms_bit_e, ls_bit_e, ms_data, ls_data;
    ms_data = mht & 0xf0;
    ls_data = mht & 0x0f;
    swap(&ls_data);
    ms_data++;
    ls_data++;
    ms_bit_e = ms_data | 0x05;
    ls_bit_e = ls_data | 0x05;

    lcd_soft_i2c_start();
    lcd_soft_i2c_write(PCF_ADDRES_WRITE);
    lcd_soft_i2c_write(ms_bit_e | LCD_BACKLIGHT);
    lcd_soft_i2c_write(ms_data | LCD_BACKLIGHT);
    lcd_soft_i2c_write(ls_bit_e | LCD_BACKLIGHT);
    lcd_soft_i2c_write(ls_data | LCD_BACKLIGHT);
    lcd_soft_i2c_stop();
    delay_us(100);
}

void lcd_create_custom_char(uint8_t location, uint8_t charmap[]) {
    location &= 0x7;
    lcd_soft_i2c_command(0x40 | (location << 3));
    for (int i = 0; i < 8; i++) {
        lcd_soft_i2c_data(charmap[i]);
    }
}

void lcd_init_degree_symbol() {
    uint8_t degree_charmap[8] = { 0x07, 0x05, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00 };
    lcd_create_custom_char(DEGREE_SYMBOL, degree_charmap);
}

void lcd_soft_i2c_setup() {
    lcd_soft_i2c_init();
    lcd_soft_i2c_start();
    lcd_soft_i2c_write(0x4E);
    lcd_soft_i2c_write(0x00);
    lcd_soft_i2c_stop();
    delay_us(10);

    lcd_soft_i2c_command(3); delay_us(5000);
    lcd_soft_i2c_command(3); delay_us(5000);
    lcd_soft_i2c_command(3); delay_us(5000);
    lcd_soft_i2c_command(LCD_RETURN_HOME);
    lcd_soft_i2c_command(FUNCTION_SET);
    lcd_soft_i2c_command(ENTRY_MODE);
    lcd_soft_i2c_command(LCD_TURN_ON);
    lcd_soft_i2c_command(CLEAR_DISPLAY);

    lcd_soft_i2c_start();
    lcd_soft_i2c_write(0x4C);
    lcd_soft_i2c_write(0xFF);
    lcd_soft_i2c_stop();
    delay_us(10);

    lcd_soft_i2c_command(3); delay_us(5000);
    lcd_soft_i2c_command(3); delay_us(5000);
    lcd_soft_i2c_command(3); delay_us(5000);

    lcd_init_degree_symbol();
}


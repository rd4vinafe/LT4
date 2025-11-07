

/************************************************************************************
 * File Name     : ADS1220.c                                                                                                                                                                                                                                                                                                                                  *
 * Author        : SAAD IDRISSI                                                                                                                                                                                                                                                                                                                 *
 * Designation   : Electronic Engineer                                                                                                                                                                                                                                                                                                                      *
 * Version       : 1.0                                                                                                                                                                            *
 * Description   : Bare metal driver to implement for ADS1220 Using SPI Communication.                                                                                                                                        *
 * Target        : stm32f103 microcontroller
 * Processor     : ARM-Cortex M3
 * IDE           : Keil Uvision
 */

#include "main.h" // Device header
#include "ADS1220.h"
#include "stdint.h"
#include <math.h>

void GPIO_Init(void) // set gpios for ADS1220 and stm32 I/O
{
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPCEN;
}
void io_init(GPIO_TypeDef *GPIOx, GPIO_Pin_t pin, GPIO_Mode_t mode)
{
  // 1. Enable GPIO clock
  if (GPIOx == GPIOA)
    RCC->APB2ENR |= (1 << 2);
  else if (GPIOx == GPIOB)
    RCC->APB2ENR |= (1 << 3);
  else if (GPIOx == GPIOC)
    RCC->APB2ENR |= (1 << 4);
  else if (GPIOx == GPIOD)
    RCC->APB2ENR |= (1 << 5);
  else if (GPIOx == GPIOE)
    RCC->APB2ENR |= (1 << 6);

  // 2. Ch?n thanh ghi CRL/CRH v? t?nh v? tr?
  volatile uint32_t *config_reg;
  uint8_t shift = (pin % 8) * 4;

  if (pin <= 7)
    config_reg = &GPIOx->CRL;
  else
    config_reg = &GPIOx->CRH;

  // 3. X?a 4 bit cu c?a ch?n pin
  *config_reg &= ~(0xF << shift);

  // 4. Thi?t l?p mode
  switch (mode)
  {
  case OUTPUT:
    // MODE = 11 (50MHz), CNF = 00 => 0b0011
    *config_reg |= (0x3 << shift);
    GPIOx->ODR |= (1 << pin); // Set m?c d?nh LOW (an to?n)
    break;

  case INPUT:
    // MODE = 00, CNF = 10 => 0b1000
    *config_reg |= (0x8 << shift);
    GPIOx->ODR |= (1 << pin); // K?o l?n
    break;

  default:
    // N?u mode kh?ng h?p l?, kh?ng l?m g?
    break;
  }
}

void gpio_write(GPIO_TypeDef *GPIOx, GPIO_Pin_t pin, GPIO_PinState_t state)
{
  if (state != PIN_RESET)
  {
    GPIOx->BSRR |= (1 << pin);
  }
  else
  {
    GPIOx->BSRR |= (1 << (pin + 16u));
  }
}

uint8_t gpio_read(GPIO_TypeDef *GPIOx, GPIO_Pin_t pin)
{
  return (GPIOx->IDR & (1 << pin)) ? 1 : 0;
}

void USART2_Init(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
  GPIOA->CRH |= GPIO_CRH_CNF9_1;
  GPIOA->CRH |= (GPIO_CRH_MODE9_1 | GPIO_CRH_MODE9_0);

  USART1->BRR = 0XEA6;

  USART1->CR1 |= USART_CR1_UE;
  USART1->CR1 |= USART_CR1_TE;
  USART1->CR1 |= USART_CR1_RE;
}
void USART_write(int ch)
{

  while (!(USART1->SR & USART_SR_TXE))
  {
  } // we check if the transmit buffer is empty before sending the data

  USART1->DR = (ch); // contains the received or transmitted data
                     // we put the data which we will send in DR register of the microcontroller
}
void SPI1_init(void)
{

  GPIOA->CRL |= GPIO_CRL_MODE4_0;
  GPIOA->CRL |= GPIO_CRL_MODE4_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF4_0;
  GPIOA->CRL &= ~GPIO_CRL_CNF4_1;

  // GPIOA->BSRR = GPIO_BSRR_BS4; // Set idle state as high

  // In sofwtare NSS , PA4 could be used as GPIO

  // PA5-->SCK as Master AF push-pull

  GPIOA->CRL |= GPIO_CRL_MODE5_0;
  GPIOA->CRL |= GPIO_CRL_MODE5_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF5_0;
  GPIOA->CRL |= GPIO_CRL_CNF5_1;

  // PA6-->MISO as Full-Duplex MASTER -->Input pull up
  GPIOA->CRL &= ~GPIO_CRL_MODE6_0;
  GPIOA->CRL &= ~GPIO_CRL_MODE6_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF6_0;
  GPIOA->CRL |= GPIO_CRL_CNF6_1;

  // PA7-->MOSI as Full-Duplex MASTER -->AF push-pull
  GPIOA->CRL |= GPIO_CRL_MODE7_0;
  GPIOA->CRL |= GPIO_CRL_MODE7_1;
  GPIOA->CRL &= ~GPIO_CRL_CNF7_0;
  GPIOA->CRL |= GPIO_CRL_CNF7_1;

  // Enable SPI1 clock
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  // set RXONLY to 0 to select Full-Duplex Mode
  SPI1->CR1 &= ~SPI_CR1_RXONLY;

  // 2 : Configure SPI1  P707-743
  //---------------------------
  SPI1->CR1 &= ~SPI_CR1_BIDIMODE; // Use 2 lines Full duplex

  SPI1->CR1 &= ~SPI_CR1_DFF;      // 8 bits Data Frame Format p742
  SPI1->CR1 &= ~SPI_CR1_LSBFIRST; // MSB first

  // Clock Phase
  SPI1->CR1 &= ~SPI_CR1_CPOL; // Set clock Plority
  SPI1->CR1 |= SPI_CR1_CPHA;  // set Clock Phase

  // Baud rate control
  SPI1->CR1 |= SPI_CR1_BR_2; //    Fpclk=72Mhz/32 = 2.25 Mhz
  SPI1->CR1 &= ~SPI_CR1_BR_1 | SPI_CR1_BR_0;

  // Software slave select  --> In sofwtare NSS , PA4 could be used as GPIO
  //  Set as master (SSI must be high), with software managed NSS
  SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSI | SPI_CR1_SSM;

  SPI1->CR1 |= SPI_CR1_SPE; // Enable SPI1 peripheral
}

void DelayTimerUs(int n)
{
  // 1 Mhz ---> 1Us
  // ARR = n (n=1 --> 1us) (n=2 --> 2us) ....

  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
  TIM2->PSC = 36;
  TIM2->ARR = (n & 0xFF);
  TIM2->CNT = 0;
  TIM2->CR1 = TIM_CR1_CEN;

  for (int i = 0; i < n; i++)
  {

    while (!(TIM2->SR & (1 << 0)))
    {
    }
  }
  TIM2->SR &= ~(1 << 0);

  TIM2->CR1 &= ~TIM_CR1_CEN;
}

unsigned char SPI_Write(unsigned char data)
{

  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
  DelayTimerUs(5);
  while (!(SPI1->SR & SPI_SR_TXE))
  {
  }

  *(__IO uint8_t *)&SPI1->DR = data;

  return data;
}

void SPI_Send(unsigned char byte)
{

  SPI_Write(byte);
}

unsigned char SPI_Receive(void)
{

  unsigned char Result;
  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low

  Result = SPI_Write(0xFF);

  while ((SPI1->SR & SPI_SR_RXNE) != SPI_SR_RXNE)
  {
  }

  // Result = SPI_Write(0xFF);

  Result = *(__IO uint8_t *)&SPI1->DR;

  return Result;
}

void Write_Register(unsigned char addr, unsigned char data)
{

  // GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
  DelayTimerUs(5);
  SPI_Send(WREG | addr << 2);
  DelayTimerUs(5);
  SPI_Send(data);
  DelayTimerUs(5);
}

unsigned char Read_register(unsigned char addr)
{

  unsigned char Register_Value;

  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
  DelayTimerUs(5);

  SPI_Send(RREG | addr << 2);
  DelayTimerUs(5);
  // Register_Value = SPI_Send_Receive(0xFF);
  Register_Value = SPI_Receive();
  Register_Value = SPI_Receive();
  // DelayTimerUs(10);
  return Register_Value;
}

void ADS_Init()
{

  DelayTimerUs(50); // wait 50us
  SPI1_init();
  // pin_config(); // set drdy as falling edge triggered Interrupt
  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
  DelayTimerUs(1); // wait 60ns
  SPI_Send(RESET);
  DelayTimerUs(51); // wait 50us+(32*t(clk)); tclk = (1/2,25) Mhz
  // DelayTimerUs(60);
  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
  DelayTimerUs(2);
  Write_Register(config_REG0_ADDR, 0x5E);
  DelayTimerUs(2);
  Write_Register(config_REG1_ADDR, 0xC0);
  DelayTimerUs(2);
  Write_Register(config_REG2_ADDR, 0x54);
  DelayTimerUs(2);
  Write_Register(config_REG3_ADDR, 0x00);
  DelayTimerUs(2);
  // Read back all configurations registers
  DelayTimerUs(2);
  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
  DelayTimerUs(2);
  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
  SPI_Send(START_SYNC);         // Start conversion
  DelayTimerUs(1);              // wait 1us
  GPIOA->BSRR |= GPIO_BSRR_BS4; // clear SPI interface --> Set CS high
  GPIOA->BSRR |= GPIO_BSRR_BR4; // Set CS Low
}

int32_t Read_ADC(void)
{
  int32_t mResult32 = 0;
  long int bit24 = 0;
  SPI_Send(RDATA);

#ifdef ADS1220
  bit24 = SPI_Receive(); // retrieve the first 8 bits data
  bit24 = SPI_Receive();
  bit24 = (bit24 << 8);  // schift this 8 bit left 8 position
  bit24 = SPI_Receive(); // retrieve the second 8 bits data
  bit24 = SPI_Receive();

  if (bit24 & 0x8000)
    bit24 |= 0xffff0000;
#else
  bit24 = SPI_Receive();
  bit24 = SPI_Receive();

  bit24 = (bit24 << 8) | SPI_Receive();
  bit24 = (bit24 << 8) | SPI_Receive();

  /* sign extend data */
  if (bit24 & 0x800000)
    bit24 |= 0xff000000;

#endif

  mResult32 = bit24; //      mResult32 = bi24 - Offset

  return mResult32;

  // wait_us(0.02);
}

float CodeToRtd(int32_t code)
{

  volatile float RTD;

  RTD = ((code * 3240.0 * 2.0) / (16 * 8388607.0));

  return RTD;
}

float RtdToTemperature(float rtd)
{

  // float coeff =  0.00385;

  volatile float temperature;
  // temperature = (int)(((rtd/100)-1)/coeff);

  temperature = (-A + sqrt((A * A) - (4 * B * (1 - (rtd / R100))))) / (2 * B);

  return temperature;
}
double ADS1220_CodeToVoltage(int32_t rawCode, double vref, int gain)
{
  double codeMax = (double)(1 << 23); // 2^23 = 8,388,608
  return ((double)rawCode / codeMax) * (vref / gain);
}

// Hàm d?i raw code sang nhi?t d? °C (Type K + bù m?i n?i l?nh)
double ADS1220_CodeToTemp_TypeK(int32_t rawCode, double vref, int gain, double coldJunctionTemp)
{
  // B1: raw code -> di?n áp (V)
  double voltage = ADS1220_CodeToVoltage(rawCode, vref, gain);
  // B2: d?i sang µV
  double microVolt = voltage * 1e6;
  // B3: d?i sang °C theo h? s? nh?y trung bình
  double thermoTemp = microVolt / TYPEK_SENSITIVITY;
  // B4: c?ng thêm nhi?t d? m?i n?i l?nh
  return thermoTemp + coldJunctionTemp;
}
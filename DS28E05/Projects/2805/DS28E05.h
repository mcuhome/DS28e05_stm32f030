#ifndef __DS28E05_H
#define __DS28E05_H

#include "stm32f0xx_gpio.h"
#include "stm32f0xx.h"
#define DS28E05_GPIO     GPIOA
#define DS28E05_GPIO_Pin GPIO_Pin_7

#define DS28E05_Write_1()    GPIO_SetBits( DS28E05_GPIO, DS28E05_GPIO_Pin )        //写1
#define DS28E05_Write_0()    GPIO_ResetBits( DS28E05_GPIO, DS28E05_GPIO_Pin )        //写0
#define DS28E05_ReadBit()    GPIO_ReadInputDataBit( DS28E05_GPIO, DS28E05_GPIO_Pin )        //读DS28E05上的值


#define DS28e05_IO_OUT() {GPIOA->MODER&=~0x11;GPIOA->MODER|=0x01;GPIOA->OTYPER&=~0x01;GPIOA->OTYPER|=0x00;}//设置PA0输出 推挽
#define DS28e05_IO_IN()  {GPIOA->MODER&=~(0x03<<14);}//设置PA0输入


void delay_init(void);
void delay_us(int32_t nus);
void delay_ms(int16_t nms);
void GPIO_DS28E05_Out_Mode_init( void );
unsigned char DS28E05_ReadRom( unsigned char *id );
unsigned char DS28E05_ReadMemory( unsigned char tgaddr, unsigned char len, unsigned char * buffer );
unsigned char DS28E05_WriteBlock( unsigned char nblock, unsigned char *buffer );
unsigned char DS28E05_WriteMemory(unsigned char address, char *buffer, int length );
int ds28e05WriteMemory(uint8_t address, char *buffer, int length);


#endif


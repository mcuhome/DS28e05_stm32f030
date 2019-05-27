#ifndef __UART_H
#define __UART_H
#include "main.h"
 
#include "stm32f0xx.h"
#include <stdio.h>

#ifdef __GNUC__
  /* With GCC/RAISONANCE, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

void USART1_Init(void);
void uart_init(void);
uint8_t USART_Scanf(uint32_t value);
void process_rcv(uint16_t *p_buffer,uint8_t len);
  
#endif  
  

  


  

  
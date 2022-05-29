#ifndef __EC_UART_H
#define __EC_UART_H

#include "stm32f411xe.h"
#include <stdio.h>

/************************************************/
//  USART1  TX : PA9 , PB6    RX : PA10 , PB3
// 	USART2  TX : PA2  				RX : PA3
// 	USART6	TX : PA11, PC6  	RX : PA12, PC7
//*******************************************/

void UART2_init();
void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes);
void USART_delay(uint32_t us);

void USART_init(USART_TypeDef* USARTx, uint32_t baud);

void USART_begin(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX,GPIO_TypeDef* GPIO_RX, int pinRX, int baud);
uint8_t USART_getc(USART_TypeDef *USARTx);
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
//void USART_IRQHandler(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t * pRx_counter);

#endif
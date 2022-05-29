#include "stm32f4xx.h"

#ifndef __ECSYSTICK_H
#define __ECSYSTICK_H

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

static volatile uint32_t TimeDelay = 0;
#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 

void Systick_Init(uint32_t msec);
void SysTick_Handler(void);
void delay_ms(uint32_t msec);
void delay_us(uint32_t usec);
void SysTick_reset(void);

uint32_t SysTick_val(void);
void SysTick_enable(void);
void SysTick_disable (void);
 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
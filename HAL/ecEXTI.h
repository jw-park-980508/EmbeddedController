
#include "stm32f4xx.h"

#ifndef __ECEXTI_H
#define __ECEXTI_H

#define FALL_EXTI 0
#define RISE_EXTI 1
#define BOTH_EXTI 2
 
#define PA_pin 0x0
#define PB_pin 0x1
#define PC_pin 0x2
#define PD_pin 0x3
#define PE_pin 0x4

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void EXTI_init (GPIO_TypeDef *Port, uint32_t pin, int edge , int prior);
void EXTI_enable(uint32_t pin);
void EXTI_disnable(uint32_t pin);
uint32_t is_pending_EXTI(uint32_t pin);
void clear_pending_EXTI(uint32_t pin);
void EXTI15_10_IRQHandler(void);
void LED_toggle(GPIO_TypeDef *Port, uint32_t pin);
	 
	 
	 
	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
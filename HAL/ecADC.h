#ifndef __MY_ADC_H
#define __MY_ADC_H
#include "stm32f411xe.h"

// ADC trigmode
#define SW 0
#define TRGO 1

// ADC contmode
#define CONT 0
#define SINGLE 1

// Edge Type
#define RISE_ADC 1
#define FALL_ADC 2
#define BOTH_ADC 3

#define _DEFAULT 0

// ADC setting
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode);			// trigmode : SW , TRGO
void ADC_continue(int contmode); 													// contmode : CONT, SINGLE / Operate both ADC,JADC
void ADC_TRGO(TIM_TypeDef* TIMx, uint32_t msec, int edge);
void ADC_sequence(int length, int *seq); 
void TIM_TRGO_init(TIM_TypeDef* timx, uint32_t msec);

void ADC_inject_init(GPIO_TypeDef *port, int pin, int trigmode);
void ADC_inject_TRGO(TIM_TypeDef* TIMx, uint32_t msec, int edge);
void ADC_inject_sequence(uint16_t length, uint16_t *seq);
void ADC_inject_start(void);
void ADC_start(void);

uint32_t is_ADC_EOC(void);
uint32_t is_ADC_OVR(void);
void clear_ADC_OVR(void);

uint32_t ADC_read(void);
uint32_t ADC_read_JDR(uint32_t num);
uint32_t ADC_pinmap(GPIO_TypeDef *port, int pin);
#endif

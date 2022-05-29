#include "stm32f4xx.h"
#include "ecSystick.h"

#ifndef __ECTIM_H
#define __ECTIM_H

#define UP_COUNT 0
#define DOWN_COUNT 1

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
typedef struct {
	GPIO_TypeDef *port;
	int pin;
	TIM_TypeDef *timer;
	TIM_TypeDef *timx;
	int ch;
}TIM_t; 
	 
void TIM_init(TIM_TypeDef *timerx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* timx, uint32_t usec);
void TIM_period_ms(TIM_TypeDef* timx, uint32_t msec);
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec);
void TIM_INT_enable(TIM_TypeDef* timx);
void TIM_INT_disable(TIM_TypeDef* timx);

uint32_t is_UIF(TIM_TypeDef *TIMx);
void clear_UIF(TIM_TypeDef *TIMx);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ch);
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ch);


/* Input Capture*/

// Edge Type
#define RISE_TIM 0
#define FALL_TIM 1
#define BOTH_TIM 2

//Input Capture

typedef struct{
	GPIO_TypeDef *port;
	int pin;   
	TIM_TypeDef *timer;
	int ch;  		//int Timer Channel
	int ICnum;  //int IC number
} IC_t;



void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin);
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type);
void ICAP_counter_us(IC_t *ICx, int usec);

void ICAP_pinmap(IC_t *timer_pin);

uint32_t is_pending_TIM(TIM_TypeDef *TIMx);
void clear_pending_TIM(TIM_TypeDef *TIMx);

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);


#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
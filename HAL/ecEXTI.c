
#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"



void EXTI_enable(uint32_t pin)
	{
		   EXTI -> IMR |= 1<<pin;
		
		  if(pin>= 10 && pin <= 15) NVIC_EnableIRQ(EXTI15_10_IRQn);
			else if(pin>= 5 && pin <= 9) NVIC_EnableIRQ(EXTI9_5_IRQn); //
			else if(pin <=4)	NVIC_EnableIRQ(pin+6); //
			
	}
	
void EXTI_disnable(uint32_t pin)
	{
		   EXTI -> IMR |= 0<<pin;
		
		  if(pin>= 10 && pin <= 15) NVIC_DisableIRQ(EXTI15_10_IRQn);
			else if(pin>= 5 && pin <= 9) NVIC_DisableIRQ(EXTI9_5_IRQn); //
			else if(pin <=4)	NVIC_DisableIRQ(pin+6); //
			
	}


uint32_t is_pending_EXTI(uint32_t pin)
	{
		return  (EXTI->PR & (1<<pin)) == (1<<pin)  ;
	}
	
void clear_pending_EXTI(uint32_t pin)
  {
	   EXTI->PR |= (1<<pin);
  }
	
void EXTI_init (GPIO_TypeDef *Port, uint32_t pin, int edge , int prior)
	{
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN; // clock on
		
		
		if(Port == GPIOA)
		{
		SYSCFG -> EXTICR[pin/4] &= ~(0xF << 4*(pin%4)); // PC13   ~(1111<<4)
		SYSCFG -> EXTICR[pin/4] |= PA_pin <<4*(pin%4) ; // 0x 0020 (0000 0000 0010 0000) choose MUX
		}
		else if(Port == GPIOB)
		{
		SYSCFG -> EXTICR[pin/4] &= ~(0xF << 4*(pin%4)); // PC13   ~(1111<<4)
		SYSCFG -> EXTICR[pin/4] |= PB_pin <<4*(pin%4) ; // 0x 0020 (0000 0000 0010 0000) choose MUX
		}
		else if(Port == GPIOC)
		{
		SYSCFG -> EXTICR[pin/4] &= ~(0xF << 4*(pin%4)); // PC13   ~(1111<<4)
		SYSCFG -> EXTICR[pin/4] |= PC_pin <<4*(pin%4) ; // 0x 0020 (0000 0000 0010 0000) choose MUX
		}
		
		if(edge == FALL_EXTI)
		EXTI-> FTSR |= 1<<pin;  // (1<<13)
		else if(edge == RISE_EXTI)
		EXTI-> RTSR |= 1<<pin;
		else if(edge == BOTH_EXTI)
		{
		EXTI-> FTSR |= 1<<pin;  
		EXTI-> RTSR |= 1<<pin;
		}
		
		EXTI_enable(pin);
		
		
		if(pin>= 10 && pin <= 15)
		{
	    NVIC_SetPriority(EXTI15_10_IRQn,prior); // priority 0   very improtance
		}
		else if(pin>= 5 && pin <= 9)
		{
	    NVIC_SetPriority(EXTI9_5_IRQn,prior); 
		}
		else if(pin <=4)
		{
	    NVIC_SetPriority(pin+6,prior);
		}
	}
	
	
	
	
	
	
	
	
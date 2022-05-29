#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"

#include "ecSystick.h"


void Systick_Init(uint32_t ticks)
{
  	//  SysTick Control and Status Register
	SysTick->CTRL = 0;				// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = (MCU_CLK_PLL*ticks/1000) -1;				// 1ms
//	SysTick->LOAD = (MCU_CLK_PLL*ticks/1000000) -1;				// 1us

	// Clear SysTick Current Value 
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
		
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;
	
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 1
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}

void SysTick_reset ()
{
	SysTick->VAL = 0;
	
}
void SysTick_enable(void)
{
	SysTick->CTRL |=SysTick_CTRL_ENABLE_Msk;
}

void SysTick_disable (void)
{
	SysTick->CTRL &= 0;
}
uint32_t SysTick_val(void)
{
	return SysTick->VAL ;
}
void delay_ms(uint32_t nTime)
{
	TimeDelay =nTime;
	while(TimeDelay != 0);
}

void SysTick_Handler(void)
{
	if (TimeDelay > 0) TimeDelay -- ;
}

void delay_us(uint32_t nTime)
{
	TimeDelay =nTime;
	while(TimeDelay != 0);
}

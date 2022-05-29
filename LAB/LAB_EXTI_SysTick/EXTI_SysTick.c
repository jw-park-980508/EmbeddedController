/******************************************************************************
* @author   JeongWoo Park
* @Mod      2021-10-19 by JeongWoo Park     
* @brief   Embedded Controller:  LAB: SysTick and External Interrupt 
*                - 7-segment display to show a decimal number (0~9)
* 							 - when press the user button, reset
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSystick.h"

static volatile uint32_t TimeDelay = 0;
static volatile uint32_t Flag = 0;
static volatile int count = 0;


void setup(void);

int main(void) {
	
// System CLOCK, GPIO Initialiization,SysTick Initialiization
	setup();

while(1){
		sevensegment_decode(count); 
		delay_ms(1000);
  	count ++;  
	
	  if(Flag ==HIGH )
		{
			count = 0;
			Flag = LOW;
		}
		if (count>=10) count = 0; 
		SysTick_reset();
	}
}

void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) 
	{ 
		count ++;
		
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
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
 
void setup(void)
{
	RCC_PLL_init();                         // System Clock = 84MHz	
	SysRick_Initialize(1000);   
	sevensegment_init();
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
}


/******************************************************************************
* @author   JeongWoo Park
* @Mod      2021-10-07 by JeongWoo Park     
* @brief   Embedded Controller:  LAB Digital In/Out  - 7-segment Display
*                - 7-segment display to show a decimal number (0~9)
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecSystick.h"

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

static volatile uint32_t TimeDelay = 0;
static volatile uint32_t cur_time = 0;
static volatile uint32_t past_time = 0;
static volatile uint32_t del_time = 0;
static volatile uint32_t Flag = 0;
static volatile int count = 0;


void setup(void);

int main(void) {
	
// System CLOCK, GPIO Initialiization ----------------------------------------
	setup();

// SysTick Initialiization ------------------------------------------------------				
 EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
// While loop ------------------------------------------------------				

while (GPIO_read (GPIOC,BUTTON_PIN) != 0) {;}
while(1){
		cur_time = SysTick_val();
		sevensegment_decode(count); 
		delay_ms(1000);
  	count ++;  
	
	  if(Flag ==HIGH )
		{
			delay_ms(del_time);
			count = 0;
			Flag = LOW;
		}
		
		if (count>=10) count = 0; 
		SysTick_reset();
		
	}
}


void EXTI15_10_IRQHandler(void) {
	past_time = SysTick_val();
	if (is_pending_EXTI(BUTTON_PIN)) 
	{ 

		Flag = HIGH;
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
	
	SysRick_Initialize(84000);   
	sevensegment_init();
}


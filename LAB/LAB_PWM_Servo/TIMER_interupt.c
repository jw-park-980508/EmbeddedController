/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-12 by YKKIM  	
* @brief   Embedded Controller:  Tutorial ___
*					 - _________________________________
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecSystick.h"
#include "ecUART_student.h"
#include "ecEXTI.h"

static volatile uint32_t _count=0;
static volatile uint32_t  flag = 0;
static volatile uint32_t  first = 0;
static volatile uint32_t  second = 0;
	
void TIM2_IRQHandler(void);
void TIM3_IRQHandler(void);
void EXTI15_10_IRQHandler (void);
void TIM1_UP_TIM10_IRQHandler (void);
void SysTick_Handler(void);

#define LED_PIN 	5

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
	
		
		if(flag == 2)
		{
			second = _count;
			printf("first time: %d\r\n",first);
			printf("second time: %d\r\n",second);
			printf("%d\r\n",second-first);
			flag = 0;
		}
		
		}
}


// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                       // System Clock = 84MHz
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	TIM_INT_init(TIM2,1);
	TIM_INT_enable(TIM2);
	SysRick_Initialize(1000);
	UART2_init();
}

void TIM2_IRQHandler (void){
	if(is_UIF(TIM2)){ // update interrupt flag
		_count ++;
	}
	  clear_UIF(TIM2);                      // clear by writing 0
	}
	
	void EXTI15_10_IRQHandler(void) {
	if (is_pending_EXTI(BUTTON_PIN)) 
	{ 
		
		flag++;
		if(flag ==1)
		{
			first = _count;
		}
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


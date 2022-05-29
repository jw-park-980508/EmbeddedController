/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-10-27 by Park JeongWoo & Lee JunGi
* @brief   Embedded Controller:  LAB5: PWM_Servo Motor
*					 - Change the angle of the servo motor by changing the duty ratio
*
******************************************************************************
*/

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecPWM.h"
#include "ecTIM.h"
#include "ecSystick.h"
#include "ecUART_student.h" 

#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000


static volatile uint32_t count = 0;
/*
void setup(void);
PWM_t pwm;


void setup(void){
	RCC_PLL_init();
	SysTick_Init(1000);
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	GPIO_pudr(GPIOC,BUTTON_PIN,PULL_UP);
	
	GPIO_ospeed(GPIOA,1,FAST_SPEED);
	GPIO_pudr(GPIOA, 1, NO_PUPD);
	GPIO_otype(GPIOA,1,PUSH_PULL);
	
	TIM_init(TIM2,1);
	
	PWM_init(&pwm, GPIOA, 1);
	PWM_period_ms(&pwm,20);
}

int main(void) {
	setup();
	
	while (1){
		
		switch(count){
			
		case 0: PWM_duty(&pwm,0.5/20); {delay_ms(100); break;}
		case 1: PWM_duty(&pwm,0.7/20); {delay_ms(100); break;}
		case 2: PWM_duty(&pwm,0.9/20); {delay_ms(100); break;}
		case 3: PWM_duty(&pwm,1.1/20); {delay_ms(100); break;}
		case 4: PWM_duty(&pwm,1.3/20); {delay_ms(100); break;}
		case 5: PWM_duty(&pwm,1.5/20); {delay_ms(100); break;}
		case 6: PWM_duty(&pwm,1.7/20); {delay_ms(100); break;}
		case 7: PWM_duty(&pwm,1.9/20); {delay_ms(100); break;}
		case 8: PWM_duty(&pwm,2.1/20); {delay_ms(100); break;}
	  case 9: PWM_duty(&pwm,2.3/20); {delay_ms(100); break;}
   	case 10:PWM_duty(&pwm,2.5/20); {delay_ms(1000); count=0; break;}
		
			}
		}
	}
void EXTI15_10_IRQHandler(void)
{
		if (is_pending_EXTI(BUTTON_PIN)) 
	{ 
		count ++;
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}
*/
/*
//CCR_interrupt!!!!!!!!!!!
void TIM2_IRQHandler(void);

void setup(void){
	RCC_PLL_init();
	CCR_INT_init(TIM2, 2000,1,2,1);
	CCR_INT_init(TIM2, 2000,2,3,1);
	CCR_INT_init(TIM2, 2000,3,4,1);
	CCR_INT_enable(TIM2);
	
	GPIO_pudr(GPIOC,BUTTON_PIN,PULL_UP);
	
	GPIO_init(GPIOA,PA5, OUTPUT);
	GPIO_init(GPIOA,PA6, OUTPUT);
	GPIO_init(GPIOA,PA7, OUTPUT);

	
}

int main(void) {
	setup();
	
	while (1){
	
	if(count == 0)
		{
			GPIO_write(GPIOA,PA5,HIGH);
			GPIO_write(GPIOA,PA7,LOW);
		}
			else if (count == 1)
			{
				GPIO_write(GPIOA,PA6,HIGH);
				GPIO_write(GPIOA,PA5,LOW);
			}
				else if (count == 2)
				{
					GPIO_write(GPIOA,PA7,HIGH);
					GPIO_write(GPIOA,PA6,LOW);
				}
					else if (count == 3)
			count = 0;
	} 
	}

void TIM2_IRQHandler(void)
{ if(is_CCR(TIM2)){
			
		count ++;
		
		clear_CCR(TIM2);
   }
}
*/
/*
void TIM2_IRQHandler(void);
void setup(void);
PWM_t pwm;
PWM_t pwm3;
PWM_t pwm2;
PWM_t pwm1;

static volatile uint32_t k = 0;

void setup(void){
	RCC_PLL_init();
	SysTick_Init(1000);
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 0);
	UART2_init(); 
	CCR_INT_init(TIM2, 1000,1,2,1);
	CCR_INT_enable(TIM2);
	
	
	GPIO_pudr(GPIOC,BUTTON_PIN,PULL_UP);
	
	GPIO_ospeed(GPIOA,1,FAST_SPEED);
	GPIO_pudr(GPIOA, 1, NO_PUPD);
	GPIO_otype(GPIOA,1,PUSH_PULL);
	
	TIM_init(TIM2,1);
	
		PWM_init(&pwm1, GPIOA, 3);
		PWM_init(&pwm2, GPIOB, 3);
		PWM_init(&pwm3, GPIOB, 10);
		TIM_period_ms(TIM2,1000);
}

int main(void) {
	setup();
			
		
		  PWM_duty(&pwm1,0.5);
			PWM_duty(&pwm2,0.66);
			PWM_duty(&pwm3,0.1);
		}
*/

void TIM2_IRQHandler(void);
void setup(void);
PWM_t pwm;
PWM_t pwm3;
PWM_t pwm2;
PWM_t pwm1;

static volatile uint32_t k = 0;

void setup(void){
	RCC_PLL_init();
	SysTick_Init(1000);
	
	UART2_init(); 

	
	
	GPIO_pudr(GPIOC,BUTTON_PIN,PULL_UP);
	
	GPIO_ospeed(GPIOA,1,FAST_SPEED);
	GPIO_pudr(GPIOA, 1, NO_PUPD);
	GPIO_otype(GPIOA,1,PUSH_PULL);
	
	TIM_init(TIM1,1);
	
		PWM_init(&pwm1, GPIOA, 8);
		PWM_init(&pwm2, GPIOA, 7);
		//PWM_init(&pwm3, GPIOB, 10);
		TIM_period_ms(TIM1,1000);
	
}

int main(void) {
	setup();
			
	 	
		  PWM_duty(&pwm1,0.5);
			PWM_duty(&pwm2,0.5);
		//	PWM_duty(&pwm3,0.1);
		}



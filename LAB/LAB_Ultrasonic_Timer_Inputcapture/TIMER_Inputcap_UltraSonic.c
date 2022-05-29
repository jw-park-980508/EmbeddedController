/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-11-12 by ParkJeongWoo  	
* @brief   Embedded Controller:  Inputcap_UltraSonic
*					 - Detect the distance through the UltraSonic sensor 
* 
******************************************************************************
*/


#include "stm32f411xe.h"
#include "math.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecUART_student.h"
#include "ecSystick.h"

void TIM2_IRQHandler(void);

uint32_t ovf_cnt = 0;
double distance = 0;
double timeInterval = 0;
double timeSt = 0;
double timeEnd= 0;

static volatile int flag = 0;

void setup(void);

int main(void){
	
	setup();
	
	
	while(1){
	  //distance = (float) timeInterval/58*10; // Ultrasonic speed[m/s] * echo pulse duration[us]
		//printf("%f [mm]\r\n",distance);
		if(flag == 1) {
			
			distance = (float) (timeInterval*340)/(2*10000);
			printf("%f [cm]\r\n",distance);
		  delay_ms(500);		
			
		}
	}
}
void TIM2_IRQHandler(void){
	if(is_pending_TIM(TIM2)){                     // Update interrupt
		ovf_cnt++;									        // overflow count
		clear_pending_TIM(TIM2);  					    // clear update interrupt flag
	}
	if(is_CCIF(TIM2,3)){ 									// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		timeSt = TIM2->CCR3;								// Capture TimeStart from CC3u
		clear_CCIF(TIM2,3);                 // clear capture/compare interrupt flag 
	}								                    
	else if(is_CCIF(TIM2,4)){ 						// TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		timeEnd =	TIM2->CCR4;								// Capture TimeEnd from CC4
    timeInterval = 10*((timeEnd - timeSt) + (ovf_cnt * ((TIM2->ARR)+1)));		// Total time of echo pulse
		flag = 1;
		ovf_cnt = 0;                        // overflow reset
		clear_CCIF(TIM2,4);								  // clear capture/compare interrupt flag 
	}
}

void setup(){

	RCC_PLL_init(); 
	Systick_Init(1);
	UART2_init();
  
// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;											// PWM1 for trig
	
	PWM_init(&trig,GPIOA,6);			 	// PWM init as PA_6: Ultrasonic trig puls
	PWM_period_us(&trig,50000);    	// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig,10);   	// PWM pulse width of 10us

// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo;											// Input Capture for echo
	ICAP_init(&echo,GPIOB,10);    	// ICAP init as PB10 as input caputre
	ICAP_counter_us(&echo, 10);   	// ICAP counter step time as 10us
	ICAP_setup(&echo, 3, RISE);   	// TIM2_CH3 as IC3 , rising edge detect
	ICAP_setup(&echo, 4, FALL);			// TIM2_CH3 as IC4 , falling edge detect

// Enable TIMx interrupt			-----------------------------------------------------------------------	

	TIM_INT_enable(TIM2);  					// TIM2 Interrupt Enable

}
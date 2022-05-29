/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-11-18 by Park JeongWoo  	
* @brief   Embedded Controller:   ADC - IR Reflective Sensor
*					 - Using sensors to do line tracing.
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecSystick.h"
#include "ecUART_student.h"
#include "ecADC.h"

//IR parameter//
double result_v =0;
static volatile uint32_t flag = 0;
static volatile uint32_t IR1 = 0;
static volatile uint32_t IR2 = 0;


void setup(void);
static uint16_t seq[2] = {10 , 11};

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	ADC_start();
	printf("IR1 = %d\r\n",IR1);
	// Inifinite Loop ----------------------------------------------------------
	while(1)
	{		
			uint32_t thresh = 1500;
			
			
			printf("IR1 = %d\r\n",IR1);
			printf("IR2 = %d\r\n",IR2);
		  
		  if(IR1 < thresh && IR2 > thresh ) printf("GO RIGHT");
		  if(IR1 > thresh && IR2 < thresh ) printf("GO LEFT");
			
			printf("\r\n\r\n");
		  delay_ms(1000);
		
	}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                         // System Clock = 84MHz
	UART2_init();
	Systick_Init(1);
	ADC_inject_init(GPIOC,1,TRGO);
	ADC_inject_init(GPIOC,0,TRGO);
	ADC_continue(SINGLE);
	//ADC_sequence(2,seq);
	ADC_inject_sequence(2,seq); 
	ADC_inject_start();
}


void ADC_IRQHandler(void){
   if(is_ADC_OVR()){       //after finishing sequence
     clear_ADC_OVR();
   }
   if(is_ADC_JEOC()){

      IR1 = ADC_read_JDR(1);
      IR2 = ADC_read_JDR(2);
		clear_ADC_JEOC();
   }
}
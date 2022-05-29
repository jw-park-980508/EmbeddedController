
/**
******************************************************************************
* @author   JeongWoo Park
* @Mod      2021-10-07 by JeongWoo Park     
* @brief   Embedded Controller:  LAB Digital In/Out
*                - Toggle LED LD2 by Button B1  pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

void setup(void);
	
int main(void) {

   uint32_t flag =0;
	 
   // Initialiization --------------------------------------------------------
   setup();
   
   // Inifinite Loop ---------------------------------------------- ------------
   while(1){
     
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
      
      if(flag == 0)
			{
					flag =1;
				GPIO_write(GPIOA,LED_PIN,HIGH);
			
      }
			else 
			{		
				flag =0;
			  GPIO_write(GPIOA,LED_PIN,LOW);
				
			}
		 while(GPIO_read(GPIOC, BUTTON_PIN) == 0) {;}
			
		 }
  }
}


// Initialiization  
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()
	
		// Digital in --------------------------------------------------------------
	GPIO_pudr(GPIOC, BUTTON_PIN, PULL_UP);
	
	// Digital out -------------------------------------------------------------
	GPIO_pudr(GPIOA, LED_PIN , PULL_UP);
	GPIO_otype(GPIOA, LED_PIN , OPEN_DRAIN);
	GPIO_ospeed(GPIOA, LED_PIN , MEDIUM_SPEED);
	
}


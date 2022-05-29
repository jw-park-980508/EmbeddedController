/******************************************************************************
* @author   JeongWoo Park
* @Mod      2021-10-07 by JeongWoo Park     
* @brief   Embedded Controller:  LAB Digital In/Out  - 7-segment Display
*                - 7-segment display to show a decimal number (0~9)
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

void setup(void);

	
int main(void) {
   
	uint32_t count =1;
  
	// Initialiization --------------------------------------------------------
   setup();
	
  // Inifinite Loop ----------------------------------------------------------
   while(1){
   
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
		
      if(count > 9) count = 0;
	
    while(GPIO_read(GPIOC, BUTTON_PIN) == 0) {;}
			 
		  sevensegment_decode(count % 10);	
      count++;			
		
	
		}
  }
}


// Initialiization  
void setup(void)
{
	RCC_HSI_init();	
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable()
	sevensegment_init();
	
	sevensegment_decode(0);		
}






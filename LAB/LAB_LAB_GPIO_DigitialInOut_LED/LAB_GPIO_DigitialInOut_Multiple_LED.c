
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
   
   // Inifinite Loop ----------------------------------------------- -----------
   while(1){
     
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0){
      
      if(flag == 0)
			{
				flag =1;
				GPIO_write(GPIOB,PB6,LOW);
				GPIO_write(GPIOA,PA5,HIGH);
      }
			else if(flag ==	1)
			{		
				flag =2;
			  GPIO_write(GPIOA,PA5,LOW);
				GPIO_write(GPIOA,PA6,HIGH);
			}
			else if(flag ==	2)
			{		
				flag =3;
			  GPIO_write(GPIOA,PA6,LOW);
				GPIO_write(GPIOA,PA7,HIGH);
			}
			else if(flag ==	3)
			{		
				flag = 0;
			  GPIO_write(GPIOA,PA7,LOW);
				GPIO_write(GPIOB,PB6,HIGH);
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
	GPIO_init(GPIOA, PA5, OUTPUT);    // calls RCC_GPIOA_enable()
	GPIO_init(GPIOA, PA6, OUTPUT);
	GPIO_init(GPIOA, PA7, OUTPUT);
	GPIO_init(GPIOB, PB6, OUTPUT);
	
		// Digital in --------------------------------------------------------------
	GPIO_pupdr(GPIOC, BUTTON_PIN, PULL_UP);
	
	// Digital out -------------------------------------------------------------
	GPIO_pupdr(GPIOA, PA5 , PULL_UP);
	GPIO_otype(GPIOA, PA5, PUSH_PULL);
	GPIO_ospeed(GPIOA, PA5, MEDIUM_SPEED);
	
	GPIO_pupdr(GPIOA, PA6 , PULL_UP);
	GPIO_otype(GPIOA, PA6, PUSH_PULL);
	GPIO_ospeed(GPIOA, PA6, MEDIUM_SPEED);
	
	GPIO_pupdr(GPIOA, PA7, PULL_UP);
	GPIO_otype(GPIOA, PA7, PUSH_PULL);
	GPIO_ospeed(GPIOA, PA7, MEDIUM_SPEED);
	
	GPIO_pupdr(GPIOA, PB6, PULL_UP);
	GPIO_otype(GPIOA, PB6, PUSH_PULL);
	GPIO_ospeed(GPIOA, PB6, MEDIUM_SPEED);
	
}


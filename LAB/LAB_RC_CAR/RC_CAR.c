/**
******************************************************************************
* @author  SSSLAB
* @Mod       2021-8-12 by YKKIM
* @brief   Embedded Controller:  USART communication
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecSystick.h"
#include "ecPWM.h"
#include "ecTIM.h"
#include "String.h"

// USART2 : MCU to PC via usb 
// USART1 : MCU to MCU2

PWM_t pwm2;
PWM_t pwm1;

uint8_t mcu2Data = 0;
uint8_t pcData = 0;
int indx =0;
int maxBuf=10;
uint8_t buffer[100]={0,};
uint8_t buffer2 = '\r\n';
int endChar = 13;

int dir =0;
int DC_velocity =0;
int SV_angle = 3;
float duty = 0.f;

void RC_control(char cmd);
void setup(void);

int main(void) {
   // Initialiization --------------------------------------------------------
   setup();
   printf("Hello Nucleo\r\n");
	 delay_ms(500);

 
   // Inifinite Loop ----------------------------------------------------------
   while (1){
       
     if(dir ==0)
     printf("RC car: DIR:%d[deg] VEL:%d[%%] FWD\r\n\r\n",(SV_angle-1)*45,DC_velocity*25);
     else if(dir ==1)
     printf("RC car: DIR:%d[deg] VEL:%d[%%] BWD\r\n\r\n",(SV_angle-1)*45,DC_velocity*25);
		 delay_ms(1000);
	}
}

// Initialiization 
void setup(void)
{
  RCC_PLL_init();
  Systick_Init(1);
   
  // USART congfiguration
  USART_init(USART2, 38400);
  USART_begin(USART1, GPIOA,9,GPIOA,10, 9600);    // PA9 (D8) - RXD , PA10 (D2) - TXD
	 
	//PWM setting
	PWM_init(&pwm1, GPIOA, 1);  //RC servo angle //TIM2 - ch2
	PWM_init(&pwm2, GPIOB, 10); //DC Motor Speed //TIM2 - ch3
	
	PWM_period_ms(&pwm1, 20);	 //TIMER 2 period
	PWM_duty(&pwm2,0);					//DC default
	PWM_duty(&pwm1,0.075);			//servo default

	
	//GPIO output setting
	GPIO_init(GPIOA,4,OUTPUT);
	//GPIO_init(GPIOB,4,OUTPUT);	
	

}

void RC_control(char cmd)
{
	switch(cmd){
		case 'U' : DC_velocity++; break;
		case 'D' : DC_velocity--; break;
		case 'R' : SV_angle++;    break;
		case 'L' : SV_angle--;    break;
		case 'F' : dir =0;				break;
		case 'B' : dir =1;				break;
		case 'S' : DC_velocity = 0;	SV_angle = 3; break;
		}
	
	if		 (DC_velocity < 0) DC_velocity = 0;
	else if(DC_velocity > 4) DC_velocity = 4;		
		
	if		 (SV_angle < 1) SV_angle = 1;
  else if(SV_angle > 5) SV_angle = 5;
	
	GPIO_write(GPIOA,4,dir);
	//GPIO_write(GPIOB,4,dir);
		
	if	   (dir == 0) duty = 0.25 * (float)DC_velocity;
  else if(dir == 1) duty = 1.0 - 0.25 * (float)DC_velocity;
	
	PWM_duty(&pwm2,duty);			
	PWM_duty(&pwm1,0.025*SV_angle);	
	
}


//FIX this breciecve 
void USART1_IRQHandler(){      //USART1 INT 
   if(is_USART_RXNE(USART1)){
      mcu2Data = USART_getc(USART1);
			USART_write(USART1,&mcu2Data,1);
      if(mcu2Data==endChar) {
				 RC_control(buffer[0]);
				 USART_write(USART1,&buffer2,1);
				 buffer[0] = NULL;
         indx = 0;
				
      }
      else{
         if(indx>maxBuf){
            indx =0;
            memset(buffer, 0, sizeof(char) * maxBuf);
            printf("ERROR : Too long string\r\n");
         }
         buffer[indx] = mcu2Data;
         indx++;
				 
      }
   }
}

void USART2_IRQHandler(){      //USART2 INT 
   if(is_USART_RXNE(USART2)){
      pcData = USART_getc(USART2);
      USART_write(USART1,&pcData,1);   // transmit char to USART1
      printf("%c",pcData);             // echo to sender(pc)
      
      if(pcData==endChar){
         printf("\r\n");               // to change line on PC display
      }
   }
}
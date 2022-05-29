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
#include "ecUART_student.h"
#include "ecSystick.h"
#include "String.h"

// USART2 : MCU to PC via usb 
// USART1 : MCU to MCU2

uint8_t mcu2Data = 0;
uint8_t pcData = 0;
int indx =0;
int maxBuf=10;
uint8_t buffer[100]={0,};
int bReceive=0;
int ledOn = 0;
int endChar = 13;
int led2On = 0;

void setup(void);

int main(void) {
   // Initialiization --------------------------------------------------------
   setup();
   printf("Hello Nucleo\r\n");
   
   // Inifinite Loop ----------------------------------------------------------
   while (1){
       
      if(bReceive==1 && buffer[0]=='L'){
				 buffer[2] = NULL;
         printf("buffer : %s\r\n",buffer);
         if (buffer[1] >= '0' && buffer[1] <= '7') {
            led2On = buffer[1]-'0';
         } else {
            printf("ERROR : Wrong command\r\n");
         }      
         bReceive = 0;
      }
       
      GPIO_write(GPIOA, 6, (led2On&(1U<<2))>>2);      //3
      GPIO_write(GPIOA, 7, (led2On&(1U<<1))>>1);      //2
      GPIO_write(GPIOB, 6, (led2On&1U));      //1
      delay_ms(500);
      
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
   
   // GPIO configuration
   // Q1
   GPIO_init(GPIOA,5,OUTPUT);
   
   // Q2
   LED_init(GPIOA,6, OUTPUT);
   LED_init(GPIOA,7, OUTPUT);
   LED_init(GPIOB,6, OUTPUT);
}

void USART1_IRQHandler(){      //USART1 INT 
   if(is_USART_RXNE(USART1)){
      mcu2Data = USART_getc(USART1);                  
      if(mcu2Data==endChar) {
         bReceive=1;
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
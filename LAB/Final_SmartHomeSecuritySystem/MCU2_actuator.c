/**
******************************************************************************
* @author  SSSLAB
* @Mod       2021-11-27 by Park Jeong Woo
* @brief   Embedded Controller:  MCU2 
*
******************************************************************************
*/

#include "ecInclude.h"

PWM_t pwm2;
PWM_t pwm1;

uint8_t mcu2Data = 0;
uint8_t pcData = 0;
int idx =0;
uint8_t buf[4] = {0};
int maxBuf=10;
uint8_t buffer[100]={0,};
uint8_t buffer2 = '\r\n';
int endChar = 13;

void mode_scan(uint8_t *buf);

static volatile uint16_t stepper_flag = 0;
static volatile uint16_t siren_flag = 0;

static volatile uint32_t led_3sec_flag = 0;
static volatile uint32_t led_count = 0;

static volatile int count = 0;
static volatile uint32_t blink_flag = 0;
static volatile uint32_t segment_blink = 1;
static volatile uint32_t stop_flag = 0;
static volatile uint32_t curtain_state = 0;

static uint8_t state[2] = {'S','N'};
static uint8_t state_flag = 0;


static uint8_t N_stepper_flag = 0;
void setup(void);

int main(void) {
   // Initialiization --------------------------------------------------------
   setup();
   printf("Hello Nucleo\r\n");
	 
   // Inifinite Loop ----------------------------------------------------------
   while (1){

    if(state_flag){
			USART_write(USART6,&state[0], 1); 
			state_flag = 0;
		}
	if(stepper_flag == 1 && curtain_state == 0)
	{
   		Stepper_step(2048, 0,FULL);   //close 
			 curtain_state = 1;
			 stepper_flag = 0;
	}
	 else if(stepper_flag == 1 && curtain_state == 1)
	 {
		 	 Stepper_step(2048, 1,FULL);  //open
			 curtain_state = 0;
			 stepper_flag = 0;
	 }
		 else if(blink_flag == 1 &&  curtain_state == 1){
					
				Stepper_step(2048, 1, FULL);
				curtain_state = 0;
			  N_stepper_flag = 0;

	 }
	 else if(N_stepper_flag == 1 &&  curtain_state == 0){
					
				Stepper_step(2048, 0, FULL);
				curtain_state = 1;
		    N_stepper_flag = 0;
	 }
	 }
 }
// Initialiization 
void setup(void)
{
  RCC_PLL_init();
  Systick_Init(1);
      
  // USART congfiguration
  USART_init(USART2, 9600);
	USART_begin(USART6, GPIOA,11,GPIOA,12, 9600);   // PA11 (D12) - RXD , PA12 (D13) - TXD
	 
	//Stepper motor setting													// MCU2
	
	Stepper_init(GPIOA,0,GPIOA,1,GPIOA,4,GPIOB,0);  // (D6) (D5) (D4) (D3) Stepper GPIO pin initialization
  Stepper_setSpeed(12,FULL);                      //  set stepper motor speed

		
	TIM_INT_init(TIM3,1000);
	TIM_INT_enable(TIM3);
	
  TIM_INT_init(TIM2,1000);
	TIM_INT_enable(TIM2);

	
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL, 1);
	//Button
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);
	
	
	//LED + segment setting
	GPIO_init(GPIOB,3,OUTPUT); //IN_LED
	GPIO_init(GPIOA,10,OUTPUT); //OUT_LED	

	
	sevensegment_init();
  sevensegment_decode(1);


}

void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) 
   { 
  
		  siren_flag = 0;
		  sevensegment_decode(5);
		  GPIO_write(GPIOB,3,0);
		  GPIO_write(GPIOA,10,0);
		  count = 0;
      blink_flag = 0;		 
		  segment_blink = 1;
		  USART_write(USART6,&state[1], 1); 
      clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
   }
}

void USART6_IRQHandler(){	

	if(is_USART_RXNE(USART6))
	{
		
		mcu2Data = USART_getc(USART6);
		
		if(mcu2Data == '*') {
			USART_write(USART2, buf, 4);
			printf("\r\n");
			mode_scan(buf);
			idx = 0;
		}
		else{
			if(idx>maxBuf){
				idx =0;
				memset(buf, 0, sizeof(char) * maxBuf);
				printf("ERROR : Too long string\r\n");
			}
			buf[idx] = mcu2Data;
			idx++;
		}

	}
}


void TIM2_IRQHandler (void){
 if(is_UIF(TIM2))
 { 
	 
	 if(led_3sec_flag  ==1){
		 
				GPIO_write(GPIOB,3,1);
			  led_count++;
		 
		 if(led_count == 4) {
			
			 GPIO_write(GPIOB,3,0);
			 led_count =0;
			 led_3sec_flag  = 0;
		 }
	 }
   clear_UIF(TIM2);
 }
}
void TIM3_IRQHandler (void){
 if(is_UIF(TIM3))
 { 
	 
		if(siren_flag == 1){
			if(blink_flag == 0){
				
				count++;
				sevensegment_decode(5-count);
				if(count == 5) blink_flag = 1;
			}
		
		
		if(blink_flag == 1){

			state_flag = 1;
		  segment_blink =! segment_blink;
			sevensegment_decode(segment_blink*10);

			
		 }	
		}
   clear_UIF(TIM3);
 }
}

void mode_scan(uint8_t *buf)
{ 
	if(buf[0] == 'N'){

		sevensegment_decode(1);
		if(buf[1] == 'X') N_stepper_flag = 1;
		if(buf[1] == 'P'){
		
		   switch (buf[2]){
			 
				 case '1' : led_3sec_flag = 1; break;//IN_LED ON		
			 }
		}
		else if(buf[1] == 'O'){
		
		   switch (buf[2]){		 
				 case '0' : GPIO_write(GPIOA,10,0);  break;  //OUT_LED OFF
				 case '1' : GPIO_write(GPIOA,10,1);  break;  //OUT_LED ON	
			 }
		}
		else if(buf[1] == 'L'){
		   switch (buf[2]){
				 case '0' : stepper_flag = 1;   break; 
				 case '1' : stepper_flag = 1;   break;
			 }
		}
			
		
	}
	else if(buf[0] == 'S'){
	
		if(siren_flag == 0)sevensegment_decode(5);
		if(buf[1] == 'P'){
		   switch (buf[2]){
				 case '1' : GPIO_write(GPIOB,3,1);  siren_flag =1 ; break;
			 }
		}
		else if(buf[1] == 'R'){
		   switch (buf[2]){
				 case '0' : siren_flag =1 ; break;
			 }
		}
		else if(buf[1] == 'O'){
		
		   switch (buf[2]){
				 case '0' : GPIO_write(GPIOA,10,0);  break;
				 case '1' : GPIO_write(GPIOA,10,1);	 break;
			 }
		}
		else if(buf[1] == 'S'){
		   switch (buf[2]){
			 
				 case '1' :  siren_flag =1;  break;
			 }
		}
  }
}




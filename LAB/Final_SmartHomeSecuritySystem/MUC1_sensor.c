/**
******************************************************************************
* @author  SSSLAB
* @Mod       2021-11-27 by Park Jeong Woo
* @brief   Embedded Controller:  Smart House 
*
******************************************************************************
*/

#include "ecInclude.h"
/*****************************************************
**********************USART Variables*****************
******************************************************/
static uint8_t mcu2Data = 0;
static uint16_t pcData[4]={0};
static uint16_t indx     = 0;
static uint8_t modebuf[4] = {'N','X','X','*'};
static uint16_t modestate[5] = {'O','R','S','L','P'};

static uint8_t buf1[4];
static uint8_t buf2[4];
static uint8_t buf3[5];
static uint8_t buf4[4];
static uint8_t buf5 [4];
/*****************************************************
**********************sensor variable*****************
******************************************************/
struct Sensor{
	
	uint16_t ultra;
	uint16_t reflect;
	uint16_t sound;
	uint16_t light;
	uint16_t button;
	uint16_t pir;

};


static struct Sensor flag;
static struct Sensor prev_flag;
static struct Sensor value;

void flag_init(void);
void flag_init(void)
{
	flag.ultra = 0;
	flag.reflect = 0;
	flag.sound  = 0;
	flag.light  = 0;
	flag.button = 0;
	flag.pir    = 0;
	
	prev_flag.ultra = 0;
	prev_flag.reflect = 0;
	prev_flag.sound  = 0;
	prev_flag.light  = 0;
	prev_flag.button = 0;
	prev_flag.pir    = 0;
}
	

static uint16_t light_value_prev = 0;
static uint8_t outside_flag_count = 0;
static uint8_t reflect_flag_count = 0;
/*****************************************************
**********************Other variable******************
******************************************************/

static uint16_t seq[2] = {10, 11};
static uint16_t ovf_cnt = 0;
static float distance = 0.f;
static double timeInterval = 0;
static double timeSt = 0;
static double timeEnd= 0;
static uint8_t print_count= 0;
static uint8_t ADC_seq_flag= 0;

static uint8_t visitor_flag =0;
static uint8_t siren_flag =0;

void save_buf(uint8_t sensor, uint8_t state);
static uint16_t light_count = 0;
static uint32_t distance_count = 10;
static volatile uint16_t flag_ultra = 0;
static volatile uint16_t prev_flag_ultra = 0;
static uint16_t reflect_count = 0;

void setup(void);

int main (void){
	
  setup();																			// Initialization
  printf("Welcome to Smart House\r\n"); 				// Finish Init
	delay_ms(500);
	 
	USART_write(USART1,(unsigned char*) "CLEARSHEET\r\n",12);	
	USART_write(USART1,(unsigned char*) "LABEL,Date,Time,Timer,Reflect,Sound,Dist,VISIT_LOG,SINEN\r\n",59);	
	
 while(1){

	sprintf(buf1, "%d", value.reflect);
	sprintf(buf2, "%d", value.sound);
	sprintf(buf3, "%f", distance);
  sprintf(buf4, "%d", visitor_flag);
	sprintf(buf5, "%d", siren_flag);
	 
	 if(print_count == 20){
				
			  USART_write(USART1,(unsigned char*) "DATA,DATE,TIME,TIMER,",21);	// transmit char to USART6
				USART_write(USART1,buf1,4);
				USART_write(USART1,(unsigned char*) ",",1);	// transmit char to USART6
				USART_write(USART1,buf2,4);
				USART_write(USART1,(unsigned char*) ",",1);	// transmit char to USART6
			  USART_write(USART1,buf3,5);
		    USART_write(USART1,(unsigned char*) ",",1);	// transmit char to USART6
				USART_write(USART1,buf4,1);
				USART_write(USART1,(unsigned char*) ",",1);	// transmit char to USART6
			  USART_write(USART1,buf5,1);
				USART_write(USART1,(unsigned char*) ",AUTOSCROLL_20\r\n",16);	// transmit char to USART6
		
		    print_count =0;
		} 
	}
}

void setup(void)
{
	
	RCC_PLL_init();																//PLL Clock ON
	Systick_Init(1); 															//Systick ON
	
	USART_init(USART2, 9600);											//USART2 PA2 (D1) - RXD , PA3  (D0) - TXD
	USART_begin(USART1, GPIOA,9,GPIOA,10, 9600);  //USART1 PA9 (D8) - RXD , PA10 (D2) - TXD
  USART_begin(USART6, GPIOA,11,GPIOA,12, 9600); //USART6 PA11(D12) - RXD , PA12 (D13) - TXD
	
	GPIO_init(GPIOC, 13, INPUT);  								  //Button Input PC13 
	GPIO_init(GPIOA, 0, INPUT);  								  //Light Sensor Input PA0 (A0)
	GPIO_init(GPIOA, 1, INPUT);  								  //PIR Sensor Input PA1 (A1)
	
	EXTI_init(GPIOC, BUTTON_PIN, FALL_EXTI, 2);		//Button EXTI PC13
  EXTI_init(GPIOA, 0, BOTH_EXTI, 2);						//Light Sensor Input PA0 (A0)
  EXTI_init(GPIOA, 1, RISE_EXTI, 2);					  //PIR sensor EXTI PA1 (A1)
	
	ADC_init(GPIOC, 1, TRGO);											//Reflect Sensor PC1 (A4) 
	ADC_init(GPIOC, 0, TRGO);											//Sound Sensor PC0 (A5)
	ADC_continue(SINGLE);                         //timer 3 100msec
  ADC_sequence(2,seq);                          //when i use sequence 
  ADC_start();																	//Star ADC
	
	PWM_t trig;                                   //PWM1 for trig 
  PWM_init(&trig,GPIOA,5);                      //timer 2 100msec
  PWM_period_us(&trig,100000);                  //PWM of 100ms period.
  PWM_pulsewidth_us(&trig,10);									//Ultrasonic trig puls (D13)
	
	IC_t echo;                                    //Input Capture for echo //timer 4 1msec
  ICAP_init(&echo,GPIOB,6);                     //ICAP init as PB6 (D10)
  ICAP_counter_us(&echo, 10);                   //ICAP counter step time as 10us
  ICAP_setup(&echo, 1, RISE_TIM);               //TIM4_CH1 as IC1 , rising edge detect
  ICAP_setup(&echo, 2, FALL_TIM);               //TIM4_CH2 as IC2 , falling edge detect
	TIM_INT_enable(TIM4);													//TIM4 interrup init
	
	TIM_INT_init(TIM2,100);												//Light sensor interrup timer
  TIM_INT_enable(TIM2);													//TIM2 enable
	
	flag_init();
}

void save_buf(uint8_t sensor, uint8_t state)
{
  modebuf[1] = sensor;
  modebuf[2] = state + '0';
	modebuf[3] = '*';
	
	if(modebuf[0] == 'S' && sensor == 'O' && state == 1)  visitor_flag = 1;
	else if(modebuf[0] == 'S' && sensor == 'O' && state == 0)  visitor_flag = 0;
	USART_write(USART6,modebuf,4);	
}
//Button press
void EXTI15_10_IRQHandler(void) {
   if (is_pending_EXTI(BUTTON_PIN)) 
   { 
      flag.button =! flag.button;
      
      if (flag.button == 0) modebuf[0] = 'N';     
      else if (flag.button == 1) modebuf[0] = 'S';
      
		  modebuf[1] = 'X';
		  modebuf[2] = 'X';
			modebuf[3] = '*';
				
			USART_write(USART6,modebuf,4);
		
		  if(modebuf[0] != 'N' || modebuf[0] != 'S' || modebuf[1] != 'X' || modebuf[2] != 'X' || modebuf[3] != '*' )
			  	memset(modebuf,0,sizeof(uint8_t)*4);
			
      clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
   }
}
//PIR sensor
void EXTI1_IRQHandler(){
	if(is_pending_EXTI(1)){
		  flag.pir = 1;
		  save_buf(modestate[4],flag.pir);
      clear_pending_EXTI(1); // cleared by writing '1'
 }
}
//Light sensor
void EXTI0_IRQHandler(void) {
   if (is_pending_EXTI(0)){
		 
  	  delay_ms(10);
		  value.light = GPIO_read(GPIOA,0);
		 	clear_pending_EXTI(0);
 }	
}
//Light sensor

void TIM2_IRQHandler (void){
 if(is_UIF(TIM2))
 {
	 print_count ++;
	 
	 if(light_value_prev != value.light)  light_count = 0;
	 else if(light_value_prev == value.light) light_count ++;
	
		if(light_count == 20){
			flag.light = value.light;
			light_count = 0;
		}
		if(prev_flag.light != flag.light) save_buf(modestate[3],flag.light);
		
		prev_flag.light = flag.light;
		light_value_prev = value.light;
		
   clear_UIF(TIM2);
 }
}
//Ultra Sonic Senor 
void TIM4_IRQHandler(void){	
   if(is_UIF(TIM4)){                     // Update interrupt
      ovf_cnt++;    						         // overflow count
      clear_UIF(TIM4);                   // clear update interrupt flag
   }
   if(is_CCIF(TIM4,1)){                  // TIM4_Ch1 (IC1) Capture Flag. Rising Edge Detect
      timeSt = TIM4->CCR1;               // Capture TimeStart from CC1
      clear_CCIF(TIM4,1);                // clear capture/compare interrupt flag 
   }                                            
   else if(is_CCIF(TIM4,2)){              // TIM4_Ch2 (I2) Capture Flag. Falling Edge Detect
      timeEnd = TIM4->CCR2;               // Capture TimeEnd from CC4
      timeInterval = 10*((timeEnd - timeSt) + (ovf_cnt * ((TIM4->ARR)+1)));      // Total time of echo pulse
      distance = (float) (timeInterval*340)/(2*10000); // [cm] 
		  ovf_cnt = 0;											  // overflow reset
		 	if(distance < 30) distance_count--;
		 
			if(outside_flag_count == 10){
						
					if (distance_count < 5) flag_ultra = 1;
					else  flag_ultra = 0;
		
	   		 if(prev_flag_ultra != flag_ultra) save_buf(modestate[0], flag_ultra);
			
						prev_flag_ultra = flag_ultra;
						outside_flag_count =0;
						distance_count = 10;
			}
			outside_flag_count++; 
      clear_CCIF(TIM4,2);                          // clear capture/compare interrupt flag 
   }
}

//sound and reflect sensors

void ADC_IRQHandler(void){
	
   if(is_ADC_OVR()){       //after finishing sequence
     clear_ADC_OVR();
   }
   if(is_ADC_EOC()){
      if(ADC_seq_flag==0){
         value.sound = ADC_read();

				 flag.sound = 1;
				if(value.sound > 1000) save_buf(modestate[2],flag.sound);
				
      } else if(ADC_seq_flag==1){
        value.reflect = ADC_read();
				reflect_flag_count++;
			 if(value.reflect < 500) reflect_count++;
			
			 if(reflect_flag_count == 10){
					
					if (reflect_count >= 7) flag.reflect = 1;
					else if (reflect_count < 6)  flag.reflect= 0;
							
					if(prev_flag.reflect != flag.reflect) save_buf(modestate[1],flag.reflect);
					  prev_flag.reflect = flag.reflect;
					
					 reflect_flag_count =0;
					 reflect_count = 0;
					}
      }
      ADC_seq_flag =! ADC_seq_flag;
   }
}

void USART6_IRQHandler(){   

   if(is_USART_RXNE(USART6))
   {
      
      mcu2Data = USART_getc(USART6);
     
     if(mcu2Data == 'S') siren_flag = 1;
     else if(mcu2Data == 'N') siren_flag =0;
      
   }
}
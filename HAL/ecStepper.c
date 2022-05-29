#include "stm32f4xx.h"
#include "ecStepper.h"

//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_full_t;

State_full_t FSM_full[4] = {  
 {0xC,{S1,S3}},
 {0x6,{S2,S0}},
 {0x3,{S3,S1}},
 {0x9,{S0,S2}}
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_half_t;

State_half_t FSM_half[8] = {  
 {0x8, {S1, S7}},
 {0xC, {S2, S0}},
 {0x4, {S3, S1}},
 {0x6, {S4, S2}},
 {0x2, {S5, S3}},
 {0x3, {S6, S4}},
 {0x1, {S7, S5}},
 {0x9, {S0, S6}}    
};



void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
//  GPIO Digital Out Initiation
	 myStepper.port1 = port1;
   myStepper.pin1  = pin1;
	 // Repeat for port2,pin2~port4,pin4 
	 myStepper.port2 = port2;
   myStepper.pin2  = pin2;
		
	 myStepper.port3 = port3;
   myStepper.pin3  = pin3;
		
	 myStepper.port4 = port4;
   myStepper.pin4  = pin4;
	
	
	
//  GPIO Digital Out Initiation
		GPIO_init(myStepper.port1,myStepper.pin1,OUTPUT);
		GPIO_init(myStepper.port2,myStepper.pin2,OUTPUT);
		GPIO_init(myStepper.port3,myStepper.pin3,OUTPUT);
		GPIO_init(myStepper.port4,myStepper.pin4,OUTPUT);
		// No pull-up Pull-down , Push-Pull, Fast	
		// Port1,Pin1 ~ Port4,Pin4
	  GPIO_setting(myStepper.port1, myStepper.pin1, PUSH_PULL, NO_PUPD, FAST_SPEED);
	  GPIO_setting(myStepper.port2, myStepper.pin2, PUSH_PULL, NO_PUPD, FAST_SPEED);
		GPIO_setting(myStepper.port3, myStepper.pin3, PUSH_PULL, NO_PUPD, FAST_SPEED);
		GPIO_setting(myStepper.port4, myStepper.pin4, PUSH_PULL, NO_PUPD, FAST_SPEED);
	
}

void Stepper_pinOut (uint32_t state, int mode){
	
	   if (mode ==FULL){         // FULL mode
		
			
			
			GPIO_write(myStepper.port1, myStepper.pin1, (FSM_full[state].out & 0x8) >> 3);		// YOUR CODE_____);                                    
			GPIO_write(myStepper.port2, myStepper.pin2, (FSM_full[state].out & 0x4)  >> 2);
			GPIO_write(myStepper.port3, myStepper.pin3, (FSM_full[state].out & 0x2)  >> 1);
			GPIO_write(myStepper.port4, myStepper.pin4, (FSM_full[state].out & 0x1)  >> 0);
		 }
			 
	  else if (mode ==HALF){    // HALF mode
	 	  GPIO_write(myStepper.port1, myStepper.pin1, (FSM_half[state].out & 0x8) >> 3);		// YOUR CODE_____);                                    
			GPIO_write(myStepper.port2, myStepper.pin2, (FSM_half[state].out & 0x4)  >> 2);
			GPIO_write(myStepper.port3, myStepper.pin3, (FSM_half[state].out & 0x2)  >> 1);
			GPIO_write(myStepper.port4, myStepper.pin4, (FSM_half[state].out & 0x1)  >> 0);
	 	}
	

}


void Stepper_setSpeed (long whatSpeed, int mode){      // rppm
	if(mode == FULL)
		step_delay = (uint32_t) (60000*1000)/(64*32*whatSpeed);// Convert rpm to milli sec
	else if(mode == HALF)
		step_delay = (uint32_t) (60000*1000)/(64*32*whatSpeed*2);
}


void Stepper_step(int steps, int direction,int mode){
	 int step_number = 0;
	 myStepper._step_num = steps;
	 uint32_t state_number = 0;
	 int max_step = 3;
	 if (mode == HALF) max_step = 7;
	 
	
	 for(;myStepper._step_num>0;myStepper._step_num--){ // run for step size
				delay_us(step_delay);                                 // delay (step_delay); 
				
		    if(direction == 0) step_number++;                  // + direction step number++
				else if(direction ==1)   step_number--;                                 // - direction step number--
				
				// YOUR CODE                   								//  step_number must be 0 to max_step
		   // YOUR CODE 
			  if (step_number>max_step)
						step_number = 0;
				else if (step_number<0)
						step_number = max_step;
						
		 
	
				
		    //if (mode == FULL) 
				//{		state_number=FSM_full[state_number].next[direction];
				//		//state_number = step_number;						// YOUR CODE       // state_number = 0 to 3 for FULL step mode
				//		state_number++;
				//}
				//
				//		if(state_number == 4)
				//			state_number = 0;
			//	else if (mode == HALF) 
			//			state_number= step_number;// YOUR CODE       // state_number = 0 to 7 for HALF step mode					
				state_number = step_number;
				Stepper_pinOut(state_number, mode);
   }
}


void Stepper_stop (void){ 
     
    	myStepper._step_num = 0;    
			// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
			GPIO_write(myStepper.port1, myStepper.pin1, 0x0);	
			GPIO_write(myStepper.port2, myStepper.pin2, 0x0);
			GPIO_write(myStepper.port3, myStepper.pin3, 0x0);
			GPIO_write(myStepper.port4, myStepper.pin4, 0x0);			 

	
}


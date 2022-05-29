/**
******************************************************************************
* @author  SSSLAB
* @Mod     2021-11-03 by JeongWoo Park     
* @brief   Embedded Controller:  LAB6_StepperMotor
*                - Select the mode, direction of motor
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecEXTI.h"
#include "ecSystick.h"
#include "ecStepper.h"
void setup(void);

int main(void) { 
   // Initialiization --------------------------------------------------------
   setup();
   
    Stepper_step(10000, 0, HALF);   // (Step : 1024, Direction : 0 or 1, Mode : FULL or HALF)

   // Inifinite Loop ----------------------------------------------------------
   while(1){
		 
	 }
}
// Initialiization 
void setup(void)
{   
   
   RCC_PLL_init();                                 // System Clock = 84MHz
   SysRick_Initialize(1);                                 // Systick init
   
   EXTI_init(GPIOC,BUTTON_PIN,FALL,0);             // External Interrupt Setting
   GPIO_init(GPIOC, BUTTON_PIN, INPUT);           // GPIOC pin13 initialization
   Stepper_init(GPIOA,0,GPIOA,1,GPIOA,4,GPIOB,0);  // Stepper GPIO pin initialization
   Stepper_setSpeed(12,HALF);                          //  set stepper motor speed
}
void EXTI15_10_IRQHandler(void) {  
   if (is_pending_EXTI(BUTTON_PIN)) {
      Stepper_stop();
      clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
   }
}
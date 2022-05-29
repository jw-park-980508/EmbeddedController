#include "stm32f4xx.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

// pin
#define      PA5                5
#define      PA6                6
#define      PA7                7
#define      PB6                6
#define      PC7                7
#define      PA9                9
#define      PA8                8
#define      PB10               10
	 
#define      BUTTON_PIN        13


// Setting
#define      HIGH               1 
#define      LOW                0

// MODE Setting
#define      INPUT        			0
#define      OUTPUT       			1 
#define      ALTERNATE    			2
#define      ANALOG       			3 

// Output type Setting
#define      PUSH_PULL 					0
#define      OPEN_DRAIN 				1

// Output speed Setting
#define      LOW_SPEED          0
#define      MEDIUM_SPEED       1 
#define      FAST_SPEED         2 
#define      HIGH_SPEED         3 

// Output PUPD Setting
#define      NO_PUPD    	      0
#define      PULL_UP            1 
#define      PULL_DOWN          2 
#define      RESERVED_2         3 

void LED_toggle(GPIO_TypeDef *Port, uint32_t pin);

void GPIO_init(GPIO_TypeDef *Port, int pin, uint32_t mode);

void GPIO_write(GPIO_TypeDef *Port, int pin, uint32_t output);

uint32_t  GPIO_read(GPIO_TypeDef *Port, uint32_t pin);

void GPIO_mode(GPIO_TypeDef* Port, int pin, uint32_t mode);

void GPIO_ospeed(GPIO_TypeDef* Port, int pin, uint32_t speed);

void GPIO_otype(GPIO_TypeDef* Port, int pin, uint32_t type);

void GPIO_pudr(GPIO_TypeDef* Port, int pin, uint32_t pudr);

void sevensegment_init(void);

void sevensegment_decode(int number);

void GPIO_setting(GPIO_TypeDef* Port, int pin, uint32_t type, uint32_t pudr,uint32_t speed);

void LED_init(GPIO_TypeDef *Port, int pin, uint32_t mode);
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
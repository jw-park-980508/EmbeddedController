#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#define CLEAR1 1UL
#define CLEAR3 3UL



void LED_toggle(GPIO_TypeDef *Port, uint32_t pin)
{
		Port->ODR ^= 1<<pin;
}

void LED_init(GPIO_TypeDef *Port, int pin, uint32_t mode)
{
	if(Port == GPIOA)
		RCC_GPIOA_enable();
	
	if(Port == GPIOB)
		RCC_GPIOB_enable();
	
	if(Port == GPIOC)
		RCC_GPIOC_enable();
	
	GPIO_mode(Port, pin, mode);
}

void GPIO_init(GPIO_TypeDef *Port, int pin, uint32_t mode)
{
	if(Port == GPIOA)
		RCC_GPIOA_enable();
	
	if(Port == GPIOB)
		RCC_GPIOB_enable();
	
	if(Port == GPIOC)
		RCC_GPIOC_enable();
	
	GPIO_mode(Port, pin, mode);
		
}
void GPIO_mode(GPIO_TypeDef* Port, int pin, uint32_t mode)
{
	
	Port->MODER &= ~(CLEAR3<<(2*pin));
	Port->MODER |= mode <<(2*pin);
}
 
void GPIO_write(GPIO_TypeDef *Port, int pin, uint32_t output)
{ 
	Port->ODR  &= ~(CLEAR1 << pin) ; 
  Port->ODR  |= (output << pin) ; 
}

uint32_t  GPIO_read (GPIO_TypeDef *Port, uint32_t pin)
{
	return (Port->IDR) & (1 << pin);
}

void GPIO_ospeed(GPIO_TypeDef* Port, int pin, uint32_t speed)
{
	Port->OSPEEDR &= ~(CLEAR3<<(2*pin));
  Port->OSPEEDR |=   speed <<(2*pin);
}

void GPIO_otype(GPIO_TypeDef* Port, int pin, uint32_t type)
{
  Port->OTYPER  &= ~(CLEAR1 << pin) ; 
  Port->OTYPER  |= (type << pin) ; 
}

void GPIO_pudr(GPIO_TypeDef* Port, int pin, uint32_t pudr)
{
	  Port->PUPDR &= ~(CLEAR3<<(2*pin));
    Port->PUPDR  |= (pudr<<(2*pin));
}
void GPIO_setting(GPIO_TypeDef* Port, int pin, uint32_t type, uint32_t pudr,uint32_t speed)
{
	  Port->OTYPER  &= ~(CLEAR1 << pin) ; 
    Port->OTYPER  |= (type << pin) ;
	  Port->PUPDR &= ~(CLEAR3<<(2*pin));
    Port->PUPDR  |= (pudr<<(2*pin));
		Port->OSPEEDR &= ~(CLEAR3<<(2*pin));
    Port->OSPEEDR |=   speed <<(2*pin);
}
void sevensegment_init()
{
	GPIO_init(GPIOA, PA5, OUTPUT);   
	GPIO_init(GPIOA, PA6, OUTPUT); 
	GPIO_init(GPIOA, PA7, OUTPUT); 
	GPIO_init(GPIOB, PB6, OUTPUT); 
	GPIO_init(GPIOC, PC7, OUTPUT); 
	GPIO_init(GPIOA, PA9, OUTPUT); 
	GPIO_init(GPIOA, PA8, OUTPUT); 
	GPIO_init(GPIOB, PB10, OUTPUT); 
	
	
	
	// Digital in --------------------------------------------------------------
	GPIO_pudr(GPIOC, BUTTON_PIN, PULL_UP);
	
	// Digital out -------------------------------------------------------------
	//GPIO_setting(GPIOA, PA6, PUSH_PULL, NO_PUPD);
	//GPIO_setting(GPIOA, PA7, PUSH_PULL, NO_PUPD);
	//GPIO_setting(GPIOB, PB6, PUSH_PULL, NO_PUPD);
	//GPIO_setting(GPIOC, PC7, PUSH_PULL, NO_PUPD);
	//GPIO_setting(GPIOA, PA9, PUSH_PULL, NO_PUPD);	
  //GPIO_setting(GPIOA, PA8, PUSH_PULL, NO_PUPD);	
	//GPIO_setting(GPIOB, PB10,PUSH_PULL, NO_PUPD);

}

void sevensegment_decode(int number)
{
	 uint32_t segment_value [11][8]={
                    {0,0,0,0,0,0,1,1},          //zero
                    {1,0,0,1,1,1,1,1},          //one
                    {0,0,1,0,0,1,0,1},          //two
                    {0,0,0,0,1,1,0,1},          //three
                    {1,0,0,1,1,0,0,1},          //four
                    {0,1,0,0,1,0,0,1},          //five
                    {0,1,0,0,0,0,0,1},          //six
                    {0,0,0,1,1,0,1,1},          //seven
                    {0,0,0,0,0,0,0,1},          //eight
                    {0,0,0,0,1,0,0,1},          //nine
                    {1,1,1,1,1,1,1,0}           //dot
                    };
	 
	GPIO_write(GPIOA, PA5, segment_value  [number][0]);
  GPIO_write(GPIOA, PA6, segment_value  [number][1]);		
	GPIO_write(GPIOA, PA7, segment_value  [number][2]);		
	GPIO_write(GPIOB, PB6, segment_value  [number][3]);		
	GPIO_write(GPIOC, PC7, segment_value  [number][4]);		
	GPIO_write(GPIOA, PA9, segment_value  [number][5]);		
	GPIO_write(GPIOA, PA8, segment_value  [number][6]);		
	GPIO_write(GPIOB, PB10, segment_value [number][7]);		
									

}
 
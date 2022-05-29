#include "ecUART.h"
#include "ecGPIO.h"
#include <math.h>


// ********************** DO NOT MODIFY HERE ***************************
// 
// Implement a dummy __FILE struct, which is called with the FILE structure.
//#ifndef __stdio_h
struct __FILE {
    //int dummy;
		int handle;

};

FILE __stdout;
FILE __stdin;
//#endif

// Retarget printf() to USART2
int fputc(int ch, FILE *f) { 
  uint8_t c;
  c = ch & 0x00FF;
  USART_write(USART2, (uint8_t *)&c, 1);
  return(ch);
}

// Retarget getchar()/scanf() to USART2  
int fgetc(FILE *f) {  
  uint8_t rxByte;
  rxByte = USART_getc(USART2);
  return rxByte;
}


/************************************************/
//  USART1  TX : PA9 , PB6    RX : PA10 , PB3
// 	USART2  TX : PA2  				RX : PA3
// 	USART6	TX : PA11, PC6  	RX : PA12, PC7
//*******************************************/

void USART_init(USART_TypeDef* USARTx, uint32_t baud){
// **********************************************************
// Default Tx,Rx GPIO, pin configuration
// PA_2 = USART2_TX, PA_3 = USART2_RX
// PB_6 = USART1_TX (default), PB_3 = USART1_RX (default)	
// PA_11 = USART6_TX (default),	PA_12 = USART6_RX (default)	
// **********************************************************
	
// 1. GPIO Pin for TX and RX   
	GPIO_TypeDef* GPIO_TX; 
	GPIO_TypeDef* GPIO_RX;
	int pinTX = 0, pinRX =0;

	if (USARTx==USART1) {
		GPIO_TX = GPIOB;
		GPIO_RX = GPIOB;
		pinTX = 6;
		pinRX = 3;
	}
	if (USARTx==USART2) {
		GPIO_TX = GPIOA;
		GPIO_RX = GPIOA;
		pinTX = 2;
		pinRX = 3;
	}
	if (USARTx==USART6) {
		GPIO_TX = GPIOA;
		GPIO_RX = GPIOA;
		pinTX = 11;
		pinRX = 12;
	}
	// if for other USART input?
	
	// USART_begin() 
	USART_begin(USARTx, GPIO_TX, pinTX, GPIO_RX, pinRX, baud);
}

void USART_begin(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, int baud){
//1. GPIO Pin for TX and RX	
	// Enable GPIO peripheral clock 	 
	// Alternative Function mode selection for Pin_y in GPIOx
	GPIO_init(GPIO_TX,pinTX,ALTERNATE);											// GPIO mode setting : AF 
	GPIO_init(GPIO_RX,pinRX,ALTERNATE);											// GPIO mode setting : AF
	
	// Set Alternative Function Register for USARTx.	
	// AF7 - USART1,2 AF8 - USART6 
	if (USARTx==USART6){ 
		// USART_TX GPIO AFR
		if(pinTX<8)
			{
		     GPIO_TX->AFR[0]  &=   ~(15U << (4*pinTX));
         GPIO_TX->AFR[0]  |=   8U << (4*pinTX);
      }   else {
         GPIO_TX->AFR[1]  &=   ~(15U << (4*(pinTX%8)));
         GPIO_TX->AFR[1] |= 8U << (4*(pinTX%8));    
      }        
				// USART_RX GPIO AFR
	  if(pinRX<8) {
         GPIO_RX->AFR[0]  &=   ~(15U << (4*pinRX));
         GPIO_RX->AFR[0]  |=   8U << (4*pinRX);
      }   else {
         GPIO_RX->AFR[1]  &=   ~(15U << (4*(pinRX%8)));
         GPIO_RX->AFR[1] |= 8U << (4*(pinRX%8)); 
      }   			
	}
	else{	//USART1,USART2
		// USART_TX GPIO AFR
		if(pinTX<8) {
         GPIO_TX->AFR[0]  &=   ~(15U << (4*pinTX));
         GPIO_TX->AFR[0]  |=   7U << (4*pinTX); 
      }   else {
         GPIO_TX->AFR[1]  &=   ~(15U << (4*(pinTX%8)));
         GPIO_TX->AFR[1] |= 7U << (4*(pinTX%8)); 
      }         
      // USART_RX GPIO AFR
      if(pinRX<8) {
         GPIO_RX->AFR[0]  &=   ~(15U << (4*pinRX));
         GPIO_RX->AFR[0]  |=   7U << (4*pinRX);  
      }   else {
         GPIO_RX->AFR[1]  &=   ~(15U << (4*(pinRX%8)));
         GPIO_RX->AFR[1] |= 7U << (4*(pinRX%8));
      }       			
	}
	// No pull up, No pull down 
  GPIO_pudr(GPIO_TX, pinTX, NO_PUPD);
	GPIO_pudr(GPIO_RX, pinRX, NO_PUPD);
	
	
	
	
//2. USARTx (x=2,1,6) configuration	
	// Enable USART peripheral clock 
	if (USARTx==USART1)
		RCC->APB2ENR |= RCC_APB2ENR_USART1EN; 	// Enable USART 1 clock (APB2 clock: AHB clock = 84MHz)	
	else if(USARTx==USART2)
		RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  		// Enable USART 2 clock (APB1 clock: AHB clock/2 = 42MHz)
	else
		RCC->APB2ENR |= RCC_APB2ENR_USART6EN;  // Enable USART 6 clock (APB2 clock: AHB clock = 84MHz)
	
	// Disable USARTx. 
	USARTx->CR1  &= ~USART_CR1_UE; 							// USART disable
	 
	// No Parity / 8-bit word length / Oversampling x16 
	USARTx->CR1 &= ~USART_CR1_PCE;  	// No parrity bit
	USARTx->CR1 &= ~USART_CR1_M;    // M: 0 = 8 data bits, 1 start bit    
	USARTx->CR1 &= ~USART_CR1_OVER8;  	// 0 = oversampling by 16 (to reduce RF noise)	 
	// Configure Stop bit
	USARTx->CR2 &= ~USART_CR2_STOP;  	// 1 stop bit																 

	// CSet Baudrate to 9600 using APB frequency (42MHz)
	// If oversampling by 16, Tx/Rx baud = f_CK / (16*USARTDIV),  
	// If oversampling by 8,  Tx/Rx baud = f_CK / (8*USARTDIV)
	// USARTDIV = 42MHz/(16*9600) = 237.4375
		 
	// Configure Baud-rate 
	float Hz=84000000; 									// if(USARTx==USART1 || USARTx==USART6)
	if(USARTx==USART2) Hz=42000000;

					
	float USARTDIV = (float)Hz/16/baud;
	 uint32_t MNT = (uint32_t)USARTDIV;
   uint32_t FRC = round((USARTDIV - MNT) * 16);
   if (FRC > 15) {
      MNT += 1;
      FRC = 0;
   }
	USARTx->BRR  |= (MNT << 4) | FRC;
	
	
	// Enable TX, RX, and USARTx 
	USARTx->CR1 |= (USART_CR1_RE | USART_CR1_TE); // Transmitter and Receiver enable
	USARTx->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
	  
	
// 3. Read USARTx Data (Interrupt)	
	// Set the priority and enable interrupt
	USARTx->CR1 |= USART_CR1_RXNEIE;      // Received Data Ready to be Read Interrupt
	if (USARTx==USART1){
		NVIC_SetPriority(USART1_IRQn, 0);      		// Set Priority to 1
		NVIC_EnableIRQ(USART1_IRQn);   						// Enable interrupt of USART1 peripheral
	}
	else if (USARTx==USART2){
		NVIC_SetPriority(USART2_IRQn, 0);      		// Set Priority to 1
		NVIC_EnableIRQ(USART2_IRQn);             	// Enable interrupt of USART2 peripheral
	}
	else {																			// if(USARTx==USART6)
		NVIC_SetPriority(USART6_IRQn, 0);      		// Set Priority to 1
		NVIC_EnableIRQ(USART6_IRQn);            	// Enable interrupt of USART2 peripheral
	}
	USARTx->CR1 |=  USART_CR1_UE; 		// USART enable
} 

uint8_t USART_getc(USART_TypeDef * USARTx){
	// Wait until RXNE (RX not empty) bit is set by HW -->Read to read
	while ((USARTx->SR & USART_SR_RXNE) != USART_SR_RXNE);
	// Reading USART_DR automatically clears the RXNE flag 
	return ((uint8_t)(USARTx->DR & 0xFF)); 	
}

uint32_t is_USART_RXNE(USART_TypeDef * USARTx){
	return (USARTx->SR & USART_SR_RXNE);
}


void UART2_init(){
	// Enable the clock of USART2
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;  // Enable USART 2 clock (APB1 clock: AHB clock / 2 = 42MHz)
	
	// Enable the peripheral clock of GPIO Port
	RCC->AHB1ENR |=   RCC_AHB1ENR_GPIOAEN;	
	
	// ********************** USART 2 ***************************
	// PA2 = USART2_TX
	// PA3 = USART2_RX
	// Alternate function(AF7), High Speed, Push pull, Pull up
	// **********************************************************
	int TX_pin = 2;
	
	GPIOA->MODER   &= ~(0xF << (2*TX_pin));	// Clear bits
	GPIOA->MODER   |=   0xA << (2*TX_pin);  // Alternate Function(10)    		
	GPIOA->AFR[0]  |=   0x77<< (4*TX_pin);  // AF7 - USART2     	
	GPIOA->OSPEEDR |=   0xF<<(2*TX_pin); 		// High speed (11)			 		
	GPIOA->PUPDR   &= ~(0xF<<(2*TX_pin));
	GPIOA->PUPDR   |=   0x5<<(2*TX_pin);    // Pull-up (01)			
	GPIOA->OTYPER  &=  ~(0x3<<TX_pin) ; 		// push-pull (0, reset)
	
	
	USART_TypeDef *USARTx = USART2;
	// No hardware flow control, 8 data bits, no parity, 1 start bit and 1 stop bit      
	USARTx->CR1 &= ~USART_CR1_UE;  			// Disable USART

	// Configure word length to 8 bit
	USARTx->CR1 &= ~USART_CR1_M;       	// M: 0 = 8 data bits, 1 start bit   
	USARTx->CR1 &= ~USART_CR1_PCE;   		// No parrity bit 
	USARTx->CR2 &= ~USART_CR2_STOP;  		// 1 stop bit    

	// Configure oversampling mode (to reduce RF noise)
	USARTx->CR1 &= ~USART_CR1_OVER8;  	// 0 = oversampling by 16   
															 
	// CSet Baudrate to 9600 using APB frequency (42MHz)
	// If oversampling by 16, Tx/Rx baud = f_CK / (16*USARTDIV),  
	// If oversampling by 8,  Tx/Rx baud = f_CK / (8*USARTDIV)
	// USARTDIV = 42MHz/(16*9600) = 237.4375

	//USARTx->BRR  = 42000000/ baud_rate;
	float Hz = 42000000;

	float USARTDIV = (float)Hz/16/9600;
	uint32_t MNT = (uint32_t)USARTDIV;
	uint32_t FRC = round((USARTDIV - MNT) * 16);
	if (FRC > 15) {
		MNT += 1;
		FRC = 0;
	}
	USARTx->BRR  |= (MNT << 4) | FRC;

	USARTx->CR1  |= (USART_CR1_RE | USART_CR1_TE);   	// Transmitter and Receiver enable
	USARTx->CR3 |= USART_CR3_DMAT | USART_CR3_DMAR;
	USARTx->CR1  |= USART_CR1_UE; 										// USART enable

	USARTx->CR1 |= USART_CR1_RXNEIE;                 	// Enable Read Interrupt
	NVIC_SetPriority(USART2_IRQn, 1);		// Set Priority to 1
	NVIC_EnableIRQ(USART2_IRQn);			 	// Enable interrupt of USART2 peripheral

}

void USART_write(USART_TypeDef * USARTx, uint8_t *buffer, uint32_t nBytes) {
	// TXE is set by hardware when the content of the TDR 
	// register has been transferred into the shift register.
	int i;
	for (i = 0; i < nBytes; i++) {
		// wait until TXE (TX empty) bit is set
		while (!(USARTx->SR & USART_SR_TXE));  
		// Writing USART_DR automatically clears the TXE flag 	
		USARTx->DR = buffer[i] & 0xFF;
		USART_delay(300);
	}
	// wait until TC bit is set
	while (!(USARTx->SR & USART_SR_TC));		
	// TC bit clear
	USARTx->SR &= ~USART_SR_TC;	
	
}  

void USART_delay(uint32_t us) {
   uint32_t time = 100*us/7;    
   while(--time);   
}

// **********************************************************
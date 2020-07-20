/*************************************************Library***********************************/
#include "libUART.h"
/*************************************************Define***********************************/

/**********************************************Use functions***********************************/
void initUsart1(void)
{
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

	GPIOB->AFR[0] &= ~(GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7); //Alternate function for PB6 - USART1_TX and PB7 - USART1_RX

	//PB6 - USART1_TX
	GPIOB->MODER &= ~GPIO_MODER_MODE6_0; //Alternative function
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_6; //Output push-pull
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED6; //Very high speed

	//PB7 - USART1_RX
	GPIOB->MODER &= ~GPIO_MODER_MODE7_0; //Alternative function
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD7; //No pull-up, no pull down

	//Usart settings
	USART1->CR1 &= ~USART_CR1_OVER8; //Oversampling by 16
	USART1->CR2 &= ~USART_CR2_STOP; //1 stop bit

	USART1->BRR = 0xD05; //9.6 Kbit/s
//	USART1->BRR = 0x116; //115.2 Kbit/s
//	USART1->BRR = 0x10; //2Mbit/s

	USART1->CR1 |= USART_CR1_TE; //Transmit enable
	USART1->CR1 |= USART_CR1_RE; //Receive enable
	USART1->CR1 |= USART_CR1_UE; //Usart enable

	//USART1->CR1 |=USART_CR1_RXNEIE;
	//NVIC_EnableIRQ(USART1_IRQn);
}

void sendUsart1 (char *chr)
{
	while(!(USART1->ISR & USART_ISR_TC));
	USART1->TDR = *chr;
}

void sendStringUsart1 (char *str)
{
	uint8_t i=0;
	while(str[i])
		sendUsart1(&str[i++]);
	sendUsart1(0);
}

char readUsart1()
{
	while ((USART1->ISR & USART_ISR_RXNE) == 0);
	return USART1->RDR;
}

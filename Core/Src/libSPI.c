/*************************************************Library***********************************/
#include "libSPI.h"
/*************************************************Define***********************************/

/*************************************************Variables***********************************/
//char Buffer[512];
//int BufCounter;
//uint8_t RX_FINISHED=0;
/**********************************************Use functions***********************************/

void initSPI1(void)
{//page 769
	//SPI1
	//PA4 - NSS,  	PA5 - SCK
	//PA6 - MISO, 	PA7 - MOSI

	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;

	//PA5(SCK)  PA7(MOSI) ����� - �������������� �������  push pull, PA6(MISO) ����� - Input floating, PA4(CS) ����� - �����, push-pull
	//clear registers
	GPIOA->MODER &= ~(GPIO_MODER_MODE4 | GPIO_MODER_MODE5 | GPIO_MODER_MODE6 | GPIO_MODER_MODE7);
	GPIOA->OTYPER &= ~(GPIO_OTYPER_OT_4 | GPIO_OTYPER_OT_5 | GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7);
	GPIOA->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEED4 | GPIO_OSPEEDER_OSPEED5 | GPIO_OSPEEDER_OSPEED6 | GPIO_OSPEEDER_OSPEED7);
	GPIOA->PUPDR &= ~(GPIO_PUPDR_PUPD4 | GPIO_PUPDR_PUPD5 | GPIO_PUPDR_PUPD6 | GPIO_PUPDR_PUPD7);
	GPIOA->AFR[0] &= ~(GPIO_AFRL_AFSEL4 | GPIO_AFRL_AFSEL5 | GPIO_AFRL_AFSEL6 | GPIO_AFRL_AFSEL7);

	//NSS
	GPIOA->MODER |= GPIO_MODER_MODE4_0; //General purpose output mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_4; //Output push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED4; //Very high speed
//	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL4; //set AF0 for alternate like SPI1

	//SCK
	GPIOA->MODER |= GPIO_MODER_MODE5_1; //Alternative function
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_5; //Output push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED5; //Very high speed
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL5; //set AF0 for alternate like SPI1

	//MISO
	GPIOA->MODER |= GPIO_MODER_MODE6_1; //Alternative function
	GPIOA->PUPDR &= ~GPIO_PUPDR_PUPD6; //No pull-up, no pull down
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL6; //set AF0 for alternate like SPI1

	//MOSI
	GPIOA->MODER |= GPIO_MODER_MODE7_1; //Alternative function
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_7; //Output push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED7; //Very high speed
	GPIOA->AFR[0] &= ~GPIO_AFRL_AFSEL7; //set AF0 for alternate like SPI1

	//Setup SPI1
	RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
	SPI1->CR1 &= ~SPI_CR1_SPE;			//Disable SPI1
	SPI1->CR1 = 0;						//Reset CR1 registers

	SPI1->CR1 &= ~SPI_CR1_BR;			//Baud rate = Fpclk/2
	SPI1->CR1 &= ~SPI_CR1_BIDIMODE;
	SPI1->CR1 &= ~SPI_CR1_DFF;          //8 bit data
	SPI1->CR1 &= ~SPI_CR1_CPOL;         //Polarity cls signal CPOL = 0;
	SPI1->CR1 &= ~SPI_CR1_CPHA;         //Phase cls signal    CPHA = 0;
	SPI1->CR1 |= SPI_CR1_SSM;  			//Hardware slave management disabled
	SPI1->CR1 |= SPI_CR1_SSI;			//Master NSS is high
	SPI1->CR1 &= ~SPI_CR1_LSBFIRST;     //MSB will be first
	SPI1->CR1 &= ~SPI_CR1_CRCEN;		//CRC disabled
	SPI1->CR1 |= SPI_CR1_MSTR;          //Mode Master

//	SPI1->CR2 |= SPI_CR2_SSOE;
	SPI1->CR2 &= ~SPI_CR2_FRF;			//SPI mode: 0 - Motorola, 1 - TI

	SPI1->CR1 |= SPI_CR1_SPE;           //Enable SPI1
	NVIC_EnableIRQ(SPI1_IRQn);			//Interrupt enable

	NSS_SET;
}

//void SPI1_IRQHandler(void)
//{
//	if(SPI1->SR & SPI_SR_RXNE)
//	{
//		Buffer[BufCounter]=SPI1->DR;
//		BufCounter++;
//	}
//	else
//	{
//		BufCounter=0;
//		RX_FINISHED=1;
//		SPI1->CR2 &=~SPI_CR2_RXNEIE;
//	}
//}

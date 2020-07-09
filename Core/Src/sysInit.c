/*************************************************Library***********************************/
#include "sysInit.h"
/*************************************************Define***********************************/

/*************************************************Variables***********************************/

/**********************************************Use functions***********************************/
int clockInit(void)
{
	RCC->CIER |= RCC_CIER_HSERDYIE; // HSE ready interrupt enabled
	RCC->CIER |= RCC_CIER_PLLRDYIE; //PLL lock interrupt enabled
	RCC->CR |= RCC_CR_CSSHSEON | RCC_CR_HSEON; // set HSE calibration check and enable HSE clock
	while((RCC->CIER & RCC_CIFR_HSERDYF) == 0); //Wait until HSERDYF is set

	RCC->CICR |= RCC_CICR_HSERDYC; //Clear the flag HSE ready

	RCC->CR &= ~RCC_CR_PLLON; //Disable the PLL
	while((RCC->CR & RCC_CR_PLLRDY) != 0); //Wait until PLLRDY is cleared

	FLASH->ACR |= FLASH_ACR_LATENCY; //Set latency to 1 wait state

	//---------Set 32 MHz by 12 MHz HSE --------------//
	RCC->CFGR |= RCC_CFGR_PLLSRC_HSE; //HSE is input clock for PLL
	RCC->CFGR |= RCC_CFGR_PLLMUL8; //HSE * 8 = PLL
	RCC->CFGR |= RCC_CFGR_PLLDIV3; //PLL / 3
	//------------------------------------------------//

	RCC->CFGR |= RCC_CFGR_HPRE_DIV1; //AHB = PLL / 1
	RCC->CFGR |= RCC_CFGR_PPRE2_DIV1; //APB2 = AHB / 1
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV1; //APB1 = AHB / 1

	RCC->CR |= RCC_CR_PLLON; //Enable the PLL
	while ((RCC->CR & RCC_CR_PLLRDY) == 0); //Wait until PLLRDY is set

	RCC->CFGR |= RCC_CFGR_SW_PLL; // Select PLL as system clock
	while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); //Wait until the PLL is switched on

	RCC->CR &= ~RCC_CR_MSION; //Disable the PLL
	while((RCC->CR & RCC_CR_MSIRDY) != 0); //Wait until PLLRDY is cleared

	return 0;
}

void timerInit(void)
{
	RCC->APB1ENR|=RCC_APB1ENR_TIM2EN;
	TIM2->PSC=1999;//999;
	TIM2->ARR=36000;//7200;
	TIM2->CR1|=TIM_CR1_CEN;
	TIM2->DIER|=TIM_DIER_UIE;
	NVIC_EnableIRQ(TIM2_IRQn);
}

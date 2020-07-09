/*************************************************Library***********************************/
#include "string.h"
#include "stdint.h"
#include "stm32l051xx.h"

/*************************************************Define***********************************/
#define NSS_SET GPIOA->ODR |= GPIO_ODR_OD4;
#define NSS_RESET GPIOA->ODR &= ~GPIO_ODR_OD4;
/*************************************************Variables***********************************/

/**********************************************Use functions***********************************/
void initSPI1(void);
//void writeSPI1 (char data);
//uint8_t readSPI1 (void);
//void sendDataSPI1(char *TxBuffer,int len);
//void resiveDataSPI1(char* RxBuffer);

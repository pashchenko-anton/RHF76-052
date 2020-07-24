/*************************************************Library***********************************/
#include "SX1276.h"
/*************************************************Define***********************************/
uint8_t i,rx_bytes,counter;
uint8_t Flag=0;
int8_t rssi,fr_rssi,snr,act_RFM=6;
uint8_t flag_update_lcd_rfm=0;
char RX_BUF[16];
//----------------- for tx module--------------------------



const unsigned char LoRa_config[]={// 868MHz, SF12, 125kHz, 300bps, MaxPower, OcpOn, 9Byte info

		REG_LR_OPMODE,					0x80, 		//Lora mode
		REG_LR_FRFMSB, 					0x6c,		//Freq: 868 MHz = 0xd9; 433 MHz = 0x6c
		REG_LR_FRFMID, 					0x40,		//Freq: 868 MHz = 0x00; 433 MHz = 0x40
		REG_LR_FRFLSB, 					0x0,		//Freq: 868 MHz = 0x00;	433 MHz = 0x00
		REG_LR_PACONFIG, 				RFLR_PACONFIG_PASELECT_PABOOST | RFLR_PACONFIG_PASELECT_MASK, //0b11111111 - Max power, however, use RFO for best result on HF
		REG_LR_OCP,						0x1F,	//OCP-on, Current 130 mA
		REG_LR_MODEMCONFIG1,			RFLR_MODEMCONFIG1_BW_10_41_KHZ | RFLR_MODEMCONFIG1_CODINGRATE_4_8 | RFLR_MODEMCONFIG1_IMPLICITHEADER_OFF,		//125kHz,4/5 Coding Rate/ Explicit
		REG_LR_MODEMCONFIG2, 			RFLR_MODEMCONFIG2_SF_12 | RFLR_MODEMCONFIG2_RXPAYLOADCRC_ON, //0xC2,			//
		REG_LR_PAYLOADLENGTH,			255,			//16 bytes // Standart -1
		REG_LR_IRQFLAGSMASK, 			RFLR_IRQFLAGS_RXDONE_MASK | RFLR_IRQFLAGS_TXDONE | RFLR_IRQFLAGS_PAYLOADCRCERROR, //Tx_Complete IRQ, RX_Complete IRQ
//		REG_LR_FIFOTXBASEADDR, 			0x00,					//Standart
//		REG_LR_FIFORXBASEADDR, 			0x00,					//Standart

		// -------------------Standart parameters	-----------------------
		/*
	REG_LR_IRQFLAGSMASK, 		0x48,					//Tx_Complete IRQ, RX_Complete IRQ
	REG_LR_PARAMP,				0x09,					//Standart 40us
	REG_LR_FIFOADDRPTR,			0x00,					//Standart
	REG_LR_FIFOTXBASEADDR, 		0x80,					//Standart
	REG_LR_FIFORXBASEADDR, 		0x00,					//Standart
	REG_LR_LNA,					0b00100000,				//Standart
	REG_LR_SYMBTIMEOUTLSB,		0x64, 					//Standart
	REG_LR_PREAMBLEMSB,			0x00,					//Standart
	REG_LR_PREAMBLELSB,			0x08,					//Standart
	REG_LR_PAYLOADMAXLENGTH,	0xFF,					//Standart
	REG_LR_HOPPERIOD,			0x00,					//Standart
		 */
};

/**********************************************Use functions***********************************/
void Delay(uint32_t delay){
	for(; delay > 0; delay--);
}

void SX1276Init()
{

	//Setup RESET Pin SX1276
	RCC->IOPENR |= RCC_IOPENR_GPIOBEN;
	GPIOB->MODER |= GPIO_MODER_MODE11_0; //General purpose output mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_11; //Output push-pull
	GPIOB->OSPEEDR |= GPIO_OSPEEDER_OSPEED11; //Very high speed
	GPIOB->ODR |= GPIO_ODR_OD11;

	//-------------Setup RF Switch---------------//
	RCC->IOPENR |= RCC_IOPENR_GPIOAEN;
	//
	GPIOA->MODER &= ~GPIO_MODER_MODE1_1; //General purpose output mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1; //Output push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED1; //Very high speed
	GPIOA->ODR &= ~GPIO_ODR_OD1;
	//
	GPIOA->MODER &= ~GPIO_MODER_MODE2_1; //General purpose output mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_2; //Output push-pull
	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEED2; //Very high speed
	GPIOA->ODR &= ~GPIO_ODR_OD2;
	//-------------------------------------------//

	//Setup interrupt from PB10 - DIO0 from SX1276
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
	GPIOB->MODER &= ~GPIO_MODER_MODE10;
	SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PB;
	EXTI->IMR |= EXTI_IMR_IM10;
	EXTI->RTSR |= EXTI_RTSR_RT10;
	EXTI->PR |= EXTI_PR_PIF10;
	NVIC_EnableIRQ(EXTI4_15_IRQn);

	CS_HI();

}

uint8_t SX1276_WriteSingle(uint8_t command,uint8_t value)
{//WriteSingle
	uint8_t temp;
	CS_LO();
	//while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready

	while (!(SPI1->SR & SPI_SR_TXE)){};
	SPI1_DR_8bit = (WRITE_SINGLE | command);
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	temp=SPI1_DR_8bit;

	while (!(SPI1->SR & SPI_SR_TXE)){};
	SPI1_DR_8bit = value;
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	temp=SPI1_DR_8bit;

	CS_HI();
	return temp;
}

uint8_t SX1276_ReadSingle(uint8_t command)
{//ReadSingle

	uint8_t temp;
	CS_LO();
	//while (GPIOB->IDR & MISO){};							//waiting until CC1101 ready

	while (!(SPI1->SR & SPI_SR_TXE)){};
	SPI1_DR_8bit = (command | READ_SINGLE);
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	command = SPI1_DR_8bit;
	while (!(SPI1->SR & SPI_SR_TXE)){};
	SPI1_DR_8bit = 0x00;
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	temp = SPI1_DR_8bit;

	CS_HI();
	return temp;
}
void SX1276_WriteBurst( uint8_t addr, char *buff, uint8_t size )
{//WriteBurst

	uint8_t j_;
	CS_LO();
	//while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready
	while (!(SPI1->SR & SPI_SR_TXE)){};
	SPI1_DR_8bit = (addr | WRITE_SINGLE);
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	SPI1_DR_8bit;
	for( j_ = 0; j_ < size; j_ ++ )
	{
		while (!(SPI1->SR & SPI_SR_TXE)){};
		SPI1_DR_8bit = buff[j_];
		while (SPI1->SR & SPI_SR_BSY){};
		while (!(SPI1->SR & SPI_SR_RXNE)){};
		SPI1_DR_8bit;
	}
	CS_HI();
}
void SX1276_ReadBurst( uint8_t cmd, char *buff, uint8_t size )
{//ReadBurst

	uint8_t j_;
	CS_LO();
	//while (GPIOA->IDR & MISO){};							//waiting until CC1101 ready
	while (!(SPI1->SR & SPI_SR_TXE)){};
	SPI1_DR_8bit = (cmd | READ_SINGLE);
	while (SPI1->SR & SPI_SR_BSY){};
	while (!(SPI1->SR & SPI_SR_RXNE)){};
	SPI1_DR_8bit;
	for( j_ = 0; j_ < size; j_ ++ )
	{
		while (!(SPI1->SR & SPI_SR_TXE)){};
		SPI1_DR_8bit = 0x00;
		while (SPI1->SR & SPI_SR_BSY){};
		while (!(SPI1->SR & SPI_SR_RXNE)){};
		buff[j_] = SPI1_DR_8bit;
	}
	CS_HI();
}
void SX1276_Init(void)
{//CC1101_Init
	uint8_t qnt,i_temp=0;
	qnt=sizeof (LoRa_config);

	SX1276_RES();
	Delay(0xFFF);
	SX1276_REL();
	Delay(0xFFF);

//	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
	while (i_temp < qnt)
	{
		SX1276_WriteSingle(LoRa_config[i_temp],LoRa_config[i_temp+1]);
		i_temp+=2;
	}
	SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_00);

	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
	SX1276_WriteSingle(REG_LR_IRQFLAGS,0xFF);
	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER|0x80); // set scan receiver mod
//	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER_SINGLE|0x80);
//	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER);
}

uint8_t SX1276_ReadRXBUF (char RX_BUF[], uint8_t* RX_BUF_SIZE)
{

	uint8_t isReceived = 0;
	int flags = SX1276_ReadSingle(REG_LR_IRQFLAGS);
	if((flags & RFLR_IRQFLAGS_RXDONE) || (flags & RFLR_IRQFLAGS_VALIDHEADER))
	{
		rx_bytes= SX1276_ReadSingle(REG_LR_RXNBBYTES);
		if(rx_bytes > 0)
		{
			rssi=SX1276_ReadSingle(REG_LR_PKTRSSIVALUE);
			snr= SX1276_ReadSingle(REG_LR_PKTSNRVALUE);
			SX1276_ReadBurst( REG_LR_FIFO, RX_BUF, rx_bytes);
			*RX_BUF_SIZE = rx_bytes;
			isReceived = 1;
		}
	}
	SX1276_WriteSingle(REG_LR_IRQFLAGS,0xFF);

	return isReceived;
}

void SX1276_SendTXBUF (uint8_t TX_BUF [], uint8_t BUF_SIZE)
{
	int txflag=0;
	//	txflag = SX1276_ReadSingle(REG_LR_IRQFLAGS);// & RFLR_IRQFLAGS_TXDONE_MASK;
	SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_01);
	SX1276_WriteSingle(REG_LR_IRQFLAGS, 0xFF);
	SX1276_WriteSingle(REG_LR_IRQFLAGSMASK, ~RFLR_IRQFLAGS_TXDONE_MASK);
	SX1276_WriteSingle(REG_LR_FIFOTXBASEADDR, 0);
	SX1276_WriteSingle(REG_LR_FIFOADDRPTR, 0);

	SX1276_WriteBurst( REG_LR_FIFO, TX_BUF, BUF_SIZE);
	SX1276_WriteSingle(REG_LR_PAYLOADLENGTH,BUF_SIZE);

	SX1276_WriteSingle(REG_LR_SYNCWORD,0x12);
	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_STANDBY|0x80);
	SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x80);
//		Delay(72000);

	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_TRANSMITTER|0x80);

	while(!txflag)
	{
		txflag = SX1276_ReadSingle(REG_LR_IRQFLAGS) & RFLR_IRQFLAGS_TXDONE_MASK;
	}
	SX1276_WriteSingle(REG_LR_IRQFLAGSMASK, 0xFF);
	SX1276_WriteSingle(REG_LR_IRQFLAGS,0xFF);

	SX1276_WriteSingle(REG_LR_DIOMAPPING1,	RFLR_DIOMAPPING1_DIO0_00); //set IRQ for RX_Done
	SX1276_WriteSingle(REG_LR_IRQFLAGSMASK, ~RFLR_IRQFLAGS_RXDONE);
	//
	//		while (!LFlag) {};
	//		SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
	//		LFlag=0;


}

void SX1276_CleanBuffer(void){
	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_SLEEP|0x80);
	SX1276_WriteSingle(REG_LR_FIFOADDRPTR,0x00);
	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER|0x80);
//	SX1276_WriteSingle(REG_LR_OPMODE,RFLR_OPMODE_RECEIVER_SINGLE|0x80);
//	Delay(0xFFFFFF);
}

//void SX1276_BuildPacket (char* PACK_NUM,char* PAYLOAD,char* END,char* BuildArray)
//{
//	sprintf(BuildArray,"%s%s%s",PACK_NUM,PAYLOAD,END);
//}


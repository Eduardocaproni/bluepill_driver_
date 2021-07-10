/*
 * lora.c
 *
 *  Created on: 8 de nov de 2020
 *      Author: edusc
 */




#include "lora.h"



int Lora_init(SPI_HandleTypeDef *hspix){
	//Put to sleep mode
	Lora_OPModeConfig(hspix,LORA_SLEEP_MODE);

	//Change from FSK to Lora
	Lora_WriteRegister(hspix, LORA_REG_OP_MODE_ADDR, LORA_SLEEP_MODE);

	//putting to explicit header mode
	Lora_MessageHeadTypeConfig(hspix, LORA_EXPLI_HEADER_MODE);

	//setting RxPayloadCrcOn (?)
	//Lora_WriteRegister(hspix, LORA_REG_MODEM_CONFIG2_ADDR,(uint8_t)0b01110100);

	//Masking some interrupts   //CREATE A FUNCTION TO DO THIS
	Lora_WriteRegister(hspix, LORA_REG_IRQ_FLAGS_MASK_ADDR, (uint8_t)0b00110111);

	//Change to standby mode
	Lora_OPModeConfig(hspix, LORA_STDBY_MODE);


	return 1;
}

HAL_StatusTypeDef Lora_Transmit(SPI_HandleTypeDef *hspix, uint8_t PayloadLenght, uint8_t *Data){
	//check OP mode status
	 if(Lora_OPModeStatus(hspix, LORA_TX_MODE)){
		 return HAL_ERROR;
	 }
	 //clear TX done that may be set or not writing 1 ///think about it later
	 Lora_WriteRegister(hspix, LORA_REG_IRQ_FLAGS_ADDR, (uint8_t)LORA_TX_DONE_MASK);

	 //Put to standby mode
	 Lora_OPModeConfig(hspix, LORA_STDBY_MODE);

	 //set the Payload Lenght in the PayloadLength register
	 Lora_WriteRegister(hspix, LORA_REG_PAYLOAD_LENGTH_ADDR, PayloadLenght);

	 //set the reg Fifo Ptr Addr to FIFO TX BASE ADDR
	 uint8_t tx_base_addr;
	 Lora_ReadRegister(hspix, LORA_REG_FIFO_TX_BASEADDR, &tx_base_addr);
	 Lora_WriteRegister(hspix, LORA_REG_FIFO_PTR_ADDR, tx_base_addr);

	 //write the data to fifo
	 HAL_StatusTypeDef status = Lora_WriteToFIFO(hspix, Data, PayloadLenght);

	 //put in TX mode
	 Lora_OPModeConfig(hspix, LORA_TX_MODE);

	 return status;

}


HAL_StatusTypeDef Lora_PutToRXContinuous(SPI_HandleTypeDef *hspix){
	//check OP mode status
	 if(Lora_OPModeStatus(hspix, LORA_TX_MODE)){
		 return HAL_ERROR;
	 }
	 //put to standby mode
	 Lora_OPModeConfig(hspix, LORA_STDBY_MODE);
	 //put to RXCONT
	 Lora_OPModeConfig(hspix, LORA_RXCONTINUOS_MODE);


	 //you have to wait to interrupt RX DONE

	return HAL_OK;
}

uint8_t Lora_RXDone_Status(SPI_HandleTypeDef *hspix){
	uint8_t irq_flags;
	Lora_ReadRegister(hspix, LORA_REG_IRQ_FLAGS_ADDR, &irq_flags);
	uint8_t status = (irq_flags & LORA_RX_DONE_MASK);

	//return 1 if it is set or 0 if it is not set
	return status;
}
/*
 * here it puts to STANDBY MODE
 */
HAL_StatusTypeDef Lora_ReceivePacket(SPI_HandleTypeDef *hspix, uint8_t *Packet, uint8_t *LenghtReceived,uint8_t MaxLenght ){
	//reading the length of the packet
	Lora_ReadRegister(hspix, LORA_REG_RX_NBBYTES_ADDR, LenghtReceived);
	uint8_t length = *LenghtReceived;

	//put to standby mode
	Lora_OPModeConfig(hspix, LORA_STDBY_MODE);

	if(length > MaxLenght)
		return HAL_ERROR;

	 //set the reg Fifo Ptr Addr to REG FIFO RX CURRENT ADDR
	 uint8_t rx_curr_addr;
	 Lora_ReadRegister(hspix, LORA_REG_FIFO_RX_CURRENT_ADDR, &rx_curr_addr);
	 Lora_WriteRegister(hspix, LORA_REG_FIFO_PTR_ADDR, rx_curr_addr);

	 //read the data from FIFO
	 HAL_StatusTypeDef status = Lora_ReadFromFIFO(hspix, Packet, length);


	 return status;
}

HAL_StatusTypeDef Lora_ReadFromFIFO(SPI_HandleTypeDef *hspix, uint8_t *Packet, uint8_t Length){
	uint8_t address = LORA_REG_FIFO_ADDR & 0x7f;

	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspix, &address, 1,100);

	HAL_StatusTypeDef status;
	status = HAL_SPI_Receive(hspix, Packet, Length,HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_SET);
	return status;
}



HAL_StatusTypeDef Lora_WriteToFIFO(SPI_HandleTypeDef *hspix, uint8_t *Data, uint8_t PayloadLenght){
	uint8_t address = LORA_REG_FIFO_ADDR | 0x80;

	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_RESET);

	HAL_SPI_Transmit(hspix, &address, 1 ,100 );

	HAL_StatusTypeDef status;
	status = HAL_SPI_Transmit(hspix, Data, PayloadLenght, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_SET);
	return status;
}



void Lora_WriteRegister(SPI_HandleTypeDef *hspix,uint8_t address, uint8_t value){

  //singleTransfer(address | 0x80, value);
	uint8_t MOSIBuffer[2];
	MOSIBuffer[0] = address | 0x80;
	MOSIBuffer[1] = value;
	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspix, MOSIBuffer, 2, 100);
	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_SET);
}


void Lora_ReadRegister(SPI_HandleTypeDef *hspix,uint8_t address, uint8_t *value){

	address = address & 0x7f;
	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_RESET);
	HAL_SPI_Transmit(hspix, &address, 1, 100);
	HAL_SPI_Receive(hspix, value, 1 ,100 );
	HAL_GPIO_WritePin(SPI2_GNSS_PORT, SPI2_GNSS_PIN, GPIO_PIN_SET);
}


void Lora_OPModeConfig(SPI_HandleTypeDef *hspix, uint8_t mode){

	Lora_WriteRegister(hspix, LORA_REG_OP_MODE_ADDR, mode);
}

int Lora_OPModeStatus(SPI_HandleTypeDef *hspix, uint8_t mode){
	uint8_t temp;
	Lora_ReadRegister(hspix, LORA_REG_OP_MODE_ADDR, &temp);
	//isolating digits
	temp = temp & (0x7);
	mode = mode & (0x7);

	//check the OP mode
	if(mode == temp)
		return 1;

	return 0;
}

void Lora_MessageHeadTypeConfig(SPI_HandleTypeDef *hspix, uint8_t mode){

	Lora_WriteRegister(hspix, LORA_REG_MODEM_CONFIG1_ADDR, mode);
}



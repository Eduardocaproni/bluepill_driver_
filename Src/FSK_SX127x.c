/*
 * FSK_RFM96.c
 *
 *  Created on: Feb 9, 2021
 *      Author: edusc
 */


#include "FSK_SX127x.h"


void FKS_SX127x_SetPins(FSK_Radio_t *SX127x, SPI_HandleTypeDef *spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, GPIO_TypeDef* reset_gpio_port, uint16_t reset_pin){
	SX127x->hspix = spi_bus;
	SX127x->ss_gpio_port = ss_gpio_port;
	SX127x->ss_pin = ss_pin;
	SX127x->reset_gpio_port = reset_gpio_port;
	SX127x->reset_pin =reset_pin;

}
HAL_StatusTypeDef FSK_init(FSK_Radio_t *SX127x){

	// didnt configure anything, all DEFAULT.
	FSK_Reset(SX127x);
	return HAL_OK;
}


HAL_StatusTypeDef FSK_WriteRegister(FSK_Radio_t *SX127x, uint8_t address, uint8_t value){
	HAL_StatusTypeDef status = HAL_OK;

	//singleTransfer(address | 0x80, value);
	uint8_t MOSIBuffer[2];
	MOSIBuffer[0] = address | 0x80;
	MOSIBuffer[1] = value;
	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(SX127x->hspix, MOSIBuffer, 2, 100);
	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_SET);
	RETURN_ON_ERROR(status);
	return HAL_OK;
}


HAL_StatusTypeDef FSK_ReadRegister(FSK_Radio_t *SX127x,uint8_t address, uint8_t *value){
	HAL_StatusTypeDef status = HAL_OK;

	address = address & 0x7f;
	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_RESET);
	status = HAL_SPI_Transmit(SX127x->hspix, &address, 1, 100);
	status = HAL_SPI_Receive(SX127x->hspix, value, 1 ,100 );
	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_SET);
	RETURN_ON_ERROR(status);
	return HAL_OK;
}

HAL_StatusTypeDef FSK_WriteToFIFO(FSK_Radio_t *SX127x, uint8_t *Data, uint8_t PayloadLenght){
	uint8_t address = FSK_REG_FIFO_ADDR | 0x80;

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(SX127x->hspix, &address, 1 ,100 );

	HAL_StatusTypeDef status;
	status = HAL_SPI_Transmit(SX127x->hspix, &PayloadLenght, 1, HAL_MAX_DELAY);
	status = HAL_SPI_Transmit(SX127x->hspix, Data, PayloadLenght, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_SET);
	RETURN_ON_ERROR(status);
	return HAL_OK;
}
HAL_StatusTypeDef FSK_WriteToFIFO_Fixed_Packet_length(FSK_Radio_t *SX127x, uint8_t *Data, uint8_t PayloadLenght){
	uint8_t address = FSK_REG_FIFO_ADDR | 0x80;

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(SX127x->hspix, &address, 1 ,100 );

	HAL_StatusTypeDef status;
	status = HAL_SPI_Transmit(SX127x->hspix, Data, PayloadLenght, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_SET);
	RETURN_ON_ERROR(status);
	return HAL_OK;
}

HAL_StatusTypeDef FSK_ReadFromFIFO(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *Length){
	uint8_t address = FSK_REG_FIFO_ADDR & 0x7f;

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(SX127x->hspix, &address, 1,100);

	HAL_StatusTypeDef status;
	status = HAL_SPI_Receive(SX127x->hspix, Length, 1, HAL_MAX_DELAY);
	status = HAL_SPI_Receive(SX127x->hspix, Packet, *Length,HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_SET);
	return status;
}

int FSK_ReadFromFIFO2(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *Length){

	uint8_t temp = 0;
	int i = 0;

	//read the length received
	FSK_ReadRegister(SX127x, FSK_REG_FIFO_ADDR, Length);

	//read the packet checking the flag FIFO empty
	FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS2, &temp);
	while( !(temp & FSK_IRQ2_FIFO_EMPTY) ){

		FSK_ReadRegister(SX127x, FSK_REG_FIFO_ADDR, &Packet[i]);
		i++;
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS2, &temp);
	}
	//returns the number of bytes that were actually readed
	return i;
}

HAL_StatusTypeDef FSK_ReadFromFIFO_Fixed_Length(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *Length){
	uint8_t address = FSK_REG_FIFO_ADDR & 0x7f;

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_RESET);

	HAL_SPI_Transmit(SX127x->hspix, &address, 1,100);

	HAL_StatusTypeDef status;
	status = HAL_SPI_Receive(SX127x->hspix, Packet, *Length, HAL_MAX_DELAY);

	HAL_GPIO_WritePin(SX127x->ss_gpio_port, SX127x->ss_pin, GPIO_PIN_SET);
	return status;
}



HAL_StatusTypeDef FSK_Transmit(FSK_Radio_t *SX127x, uint8_t PayloadLenght, uint8_t *Data){
	HAL_StatusTypeDef status = HAL_OK;

	//add some checks before enter in the STDBY MODE
	 uint8_t temp = 0;


	 //Put to standby mode
	 FSK_OPModeConfig(SX127x, FSK_STDBY_MODE);

	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;
	 }


	 //setting TxStart condition
	 temp = 0;
	 FSK_ReadRegister(SX127x, FSK_REG_FIFO_THRESH, &temp );
	 temp |= FSK_TX_START_CONDITION;
	 FSK_WriteRegister(SX127x, FSK_REG_FIFO_THRESH, temp);

	 //setting the SyncValues
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE1,0xD3);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE2,0x91);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE3,0xDA);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE4,0x26);

	 //Putting to variable Packet Lenght (can change this depending on the application)
	 //MAXSIZE of the message is 255 bytes in this mode operation (FIFO FULL WITH 66 BYTES)
	 FSK_ReadRegister(SX127x, FSK_REG_PACKET_CONFIG1, &temp);
	 temp |= FSK_PACKET_FORMAT_VARIABLE_LENGTH;
	 FSK_WriteRegister(SX127x, FSK_REG_PACKET_CONFIG1, temp);

	 //remember to let the PAYLOADlenght different from 0
	 //Put the Payloadlenght in the beginning of the FIFO in the write to FIFO function
	 FSK_WriteToFIFO(SX127x, Data, PayloadLenght);

	 //put in the FSTX mode (pay attention to this)
	 FSK_OPModeConfig(SX127x, FSK_FSTX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;
	 }

	 //put to TX mode
	 FSK_OPModeConfig(SX127x, FSK_TX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_IRQ1_TX_READY;
	 }


	 return status;
	 //REMEMBER TO CHECK THE FLAG PACKETSENT before changing the mode
}

HAL_StatusTypeDef FSK_PutToRXMODE(FSK_Radio_t *SX127x ){

	uint8_t temp;


	 //Put to standby mode
	 FSK_OPModeConfig(SX127x, FSK_STDBY_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;
	 }


	 //setting the SyncValues
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE1,0xD3);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE2,0x91);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE3,0xDA);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE4,0x26);

	 //Putting to variable Packet Lenght (can change this depending on the application)
	 //MAXSIZE of the message is 255 bytes in this mode operation (FIFO FULL WITH 66 BYTES)
	 //The Payload register contain the MAXSIZE in this case it is 66 BYTES
	 //Also desactivate the CrcAutoClear in the CrcAutoClearOff
	 FSK_ReadRegister(SX127x, FSK_REG_PACKET_CONFIG1, &temp);
	 temp = temp|FSK_PACKET_FORMAT_VARIABLE_LENGTH|FSK_CRC_AUTO_CLEAR_OFF;
	 FSK_WriteRegister(SX127x, FSK_REG_PACKET_CONFIG1, temp);


	 //put in the FSRX mode (pay attention to this)
	 FSK_OPModeConfig(SX127x, FSK_FSRX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;
	 }

	 //put in the RX mode
	 FSK_OPModeConfig(SX127x, FSK_RX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_IRQ1_RX_READY;
	 }


	 return HAL_OK;
	 //Remember to check the flag PAYLOADREADY before calling the function read package
	 //pay attention if the flag PAYLOADREADY is being clear, because she is cleared when the FIFO is empty

}





HAL_StatusTypeDef FSK_ReceivePacket(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *LenghtReceived, uint8_t MaxLenght, int *RealLengthReceived ){
	HAL_StatusTypeDef status = HAL_OK;
	//put to STDBY MODE
	FSK_OPModeConfig(SX127x, FSK_STDBY_MODE);


	//read FIFO
	*RealLengthReceived = FSK_ReadFromFIFO2(SX127x, Packet, LenghtReceived);

	if(*LenghtReceived > MaxLenght)
		return HAL_ERROR;

	 return status;
}


HAL_StatusTypeDef FSK_Transmit_Fixed_Length(FSK_Radio_t *SX127x, uint8_t PayloadLenght, uint8_t *Data){
	HAL_StatusTypeDef status = HAL_OK;

	//remember to let it less than 66 bytes
	//add some checks before enter in the STDBY MODE
	 uint8_t temp = 0;


	 //Put to standby mode
	 FSK_OPModeConfig(SX127x, FSK_STDBY_MODE);

	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;
	 }


	 //setting TxStart condition
	 temp = 0;
	 FSK_ReadRegister(SX127x, FSK_REG_FIFO_THRESH, &temp );
	 temp |= FSK_TX_START_CONDITION;
	 FSK_WriteRegister(SX127x, FSK_REG_FIFO_THRESH, temp);


	 //setting the SyncValues
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE1,0xD3);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE2,0x91);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE3,0xDA);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE4,0x26);

	 //Putting to fixed Packet Lenght (can change this depending on the application)
	 FSK_ReadRegister(SX127x, FSK_REG_PACKET_CONFIG1, &temp);
	 temp |= FSK_PACKET_FORMAT_FIXED_LENGTH;
	 FSK_WriteRegister(SX127x, FSK_REG_PACKET_CONFIG1, temp);


	 //Put the Payloadlenght in the register(less than 66 bytes)
	 FSK_WriteRegister(SX127x, FSK_REG_PAYLOAD_LENGHT, PayloadLenght);

	 //write data to FIFO
	 FSK_WriteToFIFO_Fixed_Packet_length(SX127x, Data, PayloadLenght);

	 //put in the FSTX mode (pay attention to this)
	 FSK_OPModeConfig(SX127x, FSK_FSTX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;
	 }

	 //put to TX mode
	 FSK_OPModeConfig(SX127x, FSK_TX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_IRQ1_TX_READY;
	 }


	 return status;
	 //REMEMBER TO CHECK THE FLAG PACKETSENT before changing the mode
}

HAL_StatusTypeDef FSK_PutToRXMODE_Fixed_Length(FSK_Radio_t *SX127x, uint8_t PayloadLenght ){

	uint8_t temp;

	 //Put to standby mode
	 FSK_OPModeConfig(SX127x, FSK_STDBY_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;

	 }

	 //setting the SyncValues
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE1,0xD3);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE2,0x91);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE3,0xDA);
	 FSK_WriteRegister(SX127x, FSK_REG_SYNC_VALUE4,0x26);

	 //Putting to fixed Packet Lenght (can change this depending on the application)
	 //Also desactivate the CrcAutoClear in the CrcAutoClearOff
	 FSK_ReadRegister(SX127x, FSK_REG_PACKET_CONFIG1, &temp);
	 temp = temp|FSK_PACKET_FORMAT_FIXED_LENGTH|FSK_CRC_AUTO_CLEAR_OFF;
	 FSK_WriteRegister(SX127x, FSK_REG_PACKET_CONFIG1, temp);

	 //Put the Payloadlenght in the register(less than 66 bytes)
	 FSK_WriteRegister(SX127x, FSK_REG_PAYLOAD_LENGHT, PayloadLenght);



	 //put in the FSRX mode (pay attention to this)
	 FSK_OPModeConfig(SX127x, FSK_FSRX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_MODE_READY;
	 }

	 //put in the RX mode
	 FSK_OPModeConfig(SX127x, FSK_RX_MODE);
	 temp = 0;
	 while(!temp){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		temp &= FSK_IRQ1_RX_READY;
	 }


	 return HAL_OK;
	 //Remember to check the flag PAYLOADREADY before calling the function read package
	 //pay attention if the flag PAYLOADREADY is being clear, because she is cleared when the FIFO is empty

}

HAL_StatusTypeDef FSK_ReceivePacket_Fixed_Length(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t PayloadLenght ){
	 HAL_StatusTypeDef status = HAL_OK;
	 //put to STDBY MODE
	 FSK_OPModeConfig(SX127x, FSK_STDBY_MODE);


	 //read FIFO
	 FSK_ReadFromFIFO_Fixed_Length(SX127x, Packet, &PayloadLenght);


	 return status;
}


HAL_StatusTypeDef FSK_OPModeConfig(FSK_Radio_t *SX127x, uint8_t mode){
	HAL_StatusTypeDef status = HAL_OK;
	status = FSK_WriteRegister(SX127x, FSK_REG_OP_MODE_ADDR, mode);
	RETURN_ON_ERROR(status);
	return HAL_OK;
}

int FSK_OPModeStatus(FSK_Radio_t *SX127x, uint8_t mode){

	uint8_t temp;
	FSK_ReadRegister(SX127x, FSK_REG_OP_MODE_ADDR, &temp);

	//check the OP mode
	if(mode == temp)
		return 1;
	//if it is not returns
	return 0;

}
int FSK_IRQFlagStatus(FSK_Radio_t *SX127x, uint8_t Register1or2, uint8_t flag){
	uint8_t temp;
	if(Register1or2 == 1){
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS1, &temp);
		if(flag & temp)
			return 1;

	}
	else{
		FSK_ReadRegister(SX127x, FSK_REG_IRQ_FLAGS2, &temp);
		if(flag & temp)
			return 1;
	}
	return 0;
}
HAL_StatusTypeDef FSK_Reset(FSK_Radio_t *SX127x){
	HAL_StatusTypeDef status = HAL_OK;

		HAL_GPIO_WritePin(SX127x->reset_gpio_port, SX127x->reset_pin, GPIO_PIN_RESET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(SX127x->reset_gpio_port, SX127x->reset_pin, GPIO_PIN_SET);
		HAL_Delay(10);


	return status;
}






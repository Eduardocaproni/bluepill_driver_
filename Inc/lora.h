/*
 * lora.h
 *
 *  Created on: 8 de nov de 2020
 *      Author: edusc
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_



/*
 * Includes
 */
#include "main.h"

/*
 * Addresses of the LORA register
 */

#define LORA_REG_FIFO_ADDR                    0x00U
#define LORA_REG_OP_MODE_ADDR                 0x01U
#define LORA_REG_FIFO_PTR_ADDR                0x0DU     //pointer for both TX and RX operations in the memory- when you
//read or write to the RegFIFO it increments automatically
#define LORA_REG_FIFO_TX_BASEADDR             0x0EU
#define LORA_REG_FIFO_RX_BASEADDR             0x0FU
#define LORA_REG_FIFO_RX_CURRENT_ADDR         0x10U     //indicates the location of the last packet received in the FIFO
#define LORA_REG_IRQ_FLAGS_MASK_ADDR          0x11U
#define LORA_REG_IRQ_FLAGS_ADDR               0x12U
#define LORA_REG_RX_NBBYTES_ADDR              0x13U     // defines the size of the memory location to be readen in a receive ->
//is not used in Implicity mode because we know the size of the message
#define LORA_REG_MODEM_STAT_ADDR              0x18U
#define LORA_REG_MODEM_CONFIG1_ADDR           0x1DU
#define LORA_REG_MODEM_CONFIG2_ADDR           0x1EU
#define LORA_REG_PAYLOAD_LENGTH_ADDR          0x22U     //defines the size of the memory location to be transmitted
#define LORA_REG_FIFO_RX_BYTEADDR             0x25U


/*
 * Lora IRQ Flags mask
 */
#define LORA_TX_DONE_MASK               0b00001000U
#define LORA_RX_DONE_MASK               0b01000000U
#define LORA_RXTIMEOUT_MASK             0b10000000U
/*
 * Lora spi pins
 */
#define SPI2_GNSS_PORT               GPIOB
#define SPI2_GNSS_PIN                GPIO_PIN_12

/*
 * Lora OP modes
 */
#define LORA_STDBY_MODE                   (uint8_t)0b10001001
#define LORA_SLEEP_MODE                   (uint8_t)0b10001000
#define LORA_TX_MODE                      (uint8_t)0b10001011
#define LORA_RXCONTINUOS_MODE             (uint8_t)0b10001101
#define LORA_RXSINGLE_MODE                (uint8_t)0b10001110

/*
 * Lora header message mode
 */
#define LORA_EXPLI_HEADER_MODE            (uint8_t)0b01110010
#define LORA_IMPLI_HEADER_MODE            (uint8_t)0b01110011

/*
 * Lora init functions
 */
int Lora_init(SPI_HandleTypeDef *hspix);

/*
 * Lora register write and read functions
 */
HAL_StatusTypeDef Lora_WriteToFIFO(SPI_HandleTypeDef *hspix, uint8_t *Data, uint8_t PayloadLenght);
HAL_StatusTypeDef Lora_ReadFromFIFO(SPI_HandleTypeDef *hspix, uint8_t *Packet, uint8_t Length);
void Lora_WriteRegister(SPI_HandleTypeDef *hspix,uint8_t address, uint8_t value);
void Lora_ReadRegister(SPI_HandleTypeDef *hspix,uint8_t address, uint8_t *value);

/*
 * Lora config functions
 */
void Lora_OPModeConfig(SPI_HandleTypeDef *hspix, uint8_t mode);  //LORA_SLEEP_MODE, LORA_STDBY_MODE or other
void Lora_MessageHeadTypeConfig(SPI_HandleTypeDef *hspix, uint8_t mode);  //LORA_IMPLI_HEADER_MODE or other
/*
 * Status check functions
 */
int Lora_OPModeStatus(SPI_HandleTypeDef *hspix, uint8_t mode);

/*
 * IRQ Flags status check
 */
uint8_t Lora_RXDone_Status(SPI_HandleTypeDef *hspix);
/*
 *  Transmit and receive data functions
 */
HAL_StatusTypeDef Lora_Transmit(SPI_HandleTypeDef *hspix,uint8_t PayloadLenght, uint8_t *Data);
HAL_StatusTypeDef Lora_PutToRXContinuous(SPI_HandleTypeDef *hspix);
HAL_StatusTypeDef Lora_ReceivePacket(SPI_HandleTypeDef *hspix, uint8_t *Packet, uint8_t *LenghtReceived,uint8_t MaxLenght );



#endif /* INC_LORA_H_ */

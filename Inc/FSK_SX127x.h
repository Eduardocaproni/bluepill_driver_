/*
 * FSK_SX127x.h
 *
 *  Created on: Feb 9, 2021
 *      Author: edusc
 */

#ifndef INC_FSK_SX127X_H_
#define INC_FSK_SX127X_H_

/*
 * ERROR HANDLING
 */
#define RETURN_ON_ERROR(status);\
	if(status != HAL_OK){\
		return status;\
	}


/*
 * INCLUDES
 */
#include "main.h"
/*
 * register address
 */
#define FSK_REG_FIFO_ADDR                    0x00U
#define FSK_REG_OP_MODE_ADDR                 0x01U
#define FKS_REG_BITRATE_MSB_ADDR             0x02U
#define FKS_REG_BITRATE_LSB_ADDR             0x03U
#define FSK_REG_PAYLOAD_LENGHT               0x32U
#define FSK_REG_FIFO_THRESH                  0x35U
#define FSK_REG_IRQ_FLAGS1                   0x3EU
#define FSK_REG_IRQ_FLAGS2                   0x3FU
#define FSK_REG_SYNC_CONFIG                  0x27U
#define FSK_REG_SYNC_VALUE1                  0x28U
#define FSK_REG_SYNC_VALUE2                  0x29U
#define FSK_REG_SYNC_VALUE3                  0x2AU
#define FSK_REG_SYNC_VALUE4                  0x2BU
#define FSK_REG_PACKET_CONFIG1               0x30U
#define FSK_REG_PACKET_CONFIG2               0x31U



/*
 * OP MODES
 */
#define LORA_STDBY_MODE                   (uint8_t)0b10001001
#define LORA_SLEEP_MODE                   (uint8_t)0b10001000
#define LORA_TX_MODE                      (uint8_t)0b10001011
#define LORA_RXCONTINUOS_MODE             (uint8_t)0b10001101
#define LORA_RXSINGLE_MODE                (uint8_t)0b10001110
#define FSK_SLEEP_MODE                    (uint8_t)0b00001000
#define FSK_STDBY_MODE                    (uint8_t)0b00001001
#define FSK_FSTX_MODE                     (uint8_t)0b00001010
#define FSK_TX_MODE                       (uint8_t)0b00001011
#define FSK_FSRX_MODE                     (uint8_t)0b00001100
#define FSK_RX_MODE                       (uint8_t)0b00001101

/*
 * IRQ FLAGS MASKS
 */
#define FSK_IRQ1_MODE_READY                (uint8_t)0b10000000
#define FSK_IRQ1_RX_READY                  (uint8_t)0b01000000
#define FSK_IRQ1_TX_READY                  (uint8_t)0b00100000
#define FSK_IRQ1_PLL_LOCK                  (uint8_t)0b00010000
#define FSK_IRQ1_RSSI                      (uint8_t)0b00001000
#define FSK_IRQ1_TIME_OUT                  (uint8_t)0b00000100
#define FSK_IRQ1_PREAMBLE_DETECTD          (uint8_t)0b00000010
#define FSK_IRQ1_SYNC_ADDR_MATCH           (uint8_t)0b00000001
#define FSK_IRQ2_FIFO_FULL                 (uint8_t)0b10000000    //66 BYTES MAXIMUM
#define FSK_IRQ2_FIFO_EMPTY                (uint8_t)0b01000000
#define FSK_IRQ2_FIFO_LEVEL                (uint8_t)0b00100000
#define FSK_IRQ2_FIFO_OVERRUN              (uint8_t)0b00010000
#define FSK_IRQ2_PACK_SENT                 (uint8_t)0b00001000
#define FSK_IRQ2_PAYLOAD_READY             (uint8_t)0b00000100
#define FSK_IRQ2_CRC_OK                    (uint8_t)0b00000010
#define FSK_IRQ2_LOW_BAT                   (uint8_t)0b00000001


/*
 * Other config masks
 */
#define FSK_SYNC_ON                        (uint8_t)0b00010000
#define FSK_TX_START_CONDITION             (uint8_t)0b10000000
#define FSK_PACKET_FORMAT_VARIABLE_LENGTH  (uint8_t)0b10000000
#define FSK_PACKET_FORMAT_FIXED_LENGTH     (uint8_t)0b00000000
#define FSK_MODE_READY                     (uint8_t)0b10000000
#define FSK_CRC_AUTO_CLEAR_OFF             (uint8_t)0b00001000

/*
 * Structs
 */
typedef struct{
	SPI_HandleTypeDef *hspix;
	GPIO_TypeDef* ss_gpio_port;
	uint16_t ss_pin;
	GPIO_TypeDef* reset_gpio_port;
	uint16_t reset_pin;
}FSK_Radio_t;





/*
 * FSK init functions
 */
void FKS_SX127x_SetPins(FSK_Radio_t *SX127x, SPI_HandleTypeDef *spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t nss_pin, GPIO_TypeDef* reset_gpio_port, uint16_t reset_pin);
HAL_StatusTypeDef FSK_init(FSK_Radio_t *SX127_x);


/*
 * FSK register write and read functions
 */
HAL_StatusTypeDef FSK_WriteToFIFO(FSK_Radio_t *SX127x, uint8_t *Data, uint8_t PayloadLenght);
HAL_StatusTypeDef FSK_ReadFromFIFO(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *Length);
HAL_StatusTypeDef FSK_WriteRegister(FSK_Radio_t *SX127x,uint8_t address, uint8_t value);
HAL_StatusTypeDef FSK_ReadRegister(FSK_Radio_t *SX127x,uint8_t address, uint8_t *value);
int FSK_ReadFromFIFO2(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *Length);
HAL_StatusTypeDef FSK_WriteToFIFO_Fixed_Packet_length(FSK_Radio_t *SX127x, uint8_t *Data, uint8_t PayloadLenght);
HAL_StatusTypeDef FSK_ReadFromFIFO_Fixed_Length(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *Length);
/*
 * FSK config functions
 */
HAL_StatusTypeDef FSK_OPModeConfig(FSK_Radio_t *SX127x, uint8_t mode);  //FSK_SLEEP_MODE, FSK_STDBY_MODE or other
HAL_StatusTypeDef FSK_PutToRXMODE(FSK_Radio_t *SX127x );
HAL_StatusTypeDef FSK_PutToRXMODE_Fixed_Length(FSK_Radio_t *SX127x, uint8_t PayloadLenght );
/*
 * FSK status functions
 */
int FSK_OPModeStatus(FSK_Radio_t *SX127x, uint8_t mode);
int FSK_IRQFlagStatus(FSK_Radio_t *SX127x, uint8_t Register1or2, uint8_t flag);
/*
 * FSK Transmit and Receive function
 */
HAL_StatusTypeDef FSK_Transmit(FSK_Radio_t *SX127x, uint8_t PayloadLenght, uint8_t *Data);
HAL_StatusTypeDef FSK_ReceivePacket(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t *LenghtReceived, uint8_t MaxLenght,int *RealLengthReceived );
HAL_StatusTypeDef FSK_Transmit_Fixed_Length(FSK_Radio_t *SX127x, uint8_t PayloadLenght, uint8_t *Data);
HAL_StatusTypeDef FSK_ReceivePacket_Fixed_Length(FSK_Radio_t *SX127x, uint8_t *Packet, uint8_t PayloadLenght );

/*
 * Other functions
 */
HAL_StatusTypeDef FSK_Reset(FSK_Radio_t *SX127x);


#endif /* INC_FSK_SX127X_H_ */

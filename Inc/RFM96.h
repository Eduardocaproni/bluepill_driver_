#ifndef LIB_ZENITH_RFM96_H
#define LIB_ZENITH_RFM96_H
#ifdef __cplusplus
extern "C" {
#endif

#include "zSPI.h"

/* Common Registers */
#define RFM96_REG_FIFO					0x00	// FIFO data input/output
#define RFM96_REG_OP_MODE				0x01	// Operation Mode
#define RFM96_REG_FRF_MSB				0x06	// RF carrier Frequency MSB
#define RFM96_REG_FRF_MID				0x07	// RF carrier Frequency MID
#define RFM96_REG_FRF_LSB				0x08	// RF carrier Frequency LSB
#define RFM96_REG_PA_CONFIG				0x09	// Power Amplifier Configuration
#define RFM96_REG_PA_RAMP				0x0A	// Power Amplifier ramp up/down time
#define RFM96_REG_OCP					0x0B	// Over Current Protection Configuration
#define RFM96_REG_LNA					0x0C	// Low-Noise Amplifier Configuration
#define RFM96_REG_DIO_MAPPING_1			0x40	// Mapping for DIO0 and DIO3
#define RFM96_REG_DIO_MAPPING_2			0x41	// Mapping for DIO4 and DIO5, ClkOutfrequency
#define RFM96_REG_VERSION				0x42	// Semtech ID relating the silicon revision
#define RFM96_REG_TCXO					0x4B	// TCXO or XTAL input setting
#define RFM96_REG_PA_DAC				0x4D	// High power settings of the Power Amplifier
#define RFM96_REG_FORMER_TEMP			0x5B	// Stored temperature during the former IQ Calibration
#define RFM96_REG_AGC_REF				0x61	// Adjustment of the AGC thresholds
#define RFM96_REG_AGC_THRESH1			0x62	// Adjustment of the AGC thresholds
#define RFM96_REG_AGC_THRESH2			0x63	// Adjustment of the AGC thresholds
#define RFM96_REG_AGC_THRESH3			0x64	// Adjustment of the AGC thresholds
#define RFM96_REG_PLL					0x70	// Control of the PLL bandwidth

/* LoRa Registers */
#define RFM96_REG_FIFO_ADDR_PTR			0x0D	// SPI Interface FIFO Pointer
#define RFM96_REG_FIFO_TX_BASE_ADDR		0x0E	// Base address for TX writing on FIFO
#define RFM96_REG_FIFO_RX_BASE_ADDR		0x0F	// Base address for RX reading on FIFO
#define RFM96_REG_FIFO_RX_CURRENT_ADDR	0x10	// Address of last packet received
#define RFM96_REG_IRQ_FLAGS_MASK		0x11	// Interrupt Request Flags Mask
#define RFM96_REG_IRQ_FLAGS				0x12	// Interrupt Request Flags
#define RFM96_REG_RX_NB_BYTES			0x13	// Number of received bytes
#define RFM96_REG_RX_HEADER_CNT_MSB		0x14	// Number of valid headers received since last transition into Rx mode MSB
#define RFM96_REG_RX_HEADER_CNT_LSB		0x15	// Number of valid headers received since last transition into Rx mode LSB
#define RFM96_REG_RX_PACKET_CNT_MSB		0x16	// Number of valid packets received since last transition into Rx mode MSB
#define RFM96_REG_RX_PACKET_CNT_LSB		0x17	// Number of valid packets received since last transition into Rx mode LSB
#define RFM96_REG_MODEM_STAT			0x18	// Modem status
#define RFM96_REG_PKT_SNR_VALUE			0x19	// Last Packet Signal-to-Noise Ratio
#define RFM96_REG_PKT_RSSI_VALUE		0x1A	// Last Packet Received Signal Strength Indicator
#define RFM96_REG_RSSI_VALUE			0x1B	// Current Received Signal Strength Indicator
#define RFM96_REG_HOP_CHANNEL			0x1C	// FHSS start channel
#define RFM96_REG_MODEM_CONFIG_1		0x1D	// Modem Configuration 1
#define RFM96_REG_MODEM_CONFIG_2		0x1E	// Modem Configuration 2
#define RFM96_REG_SYMB_TIMEOUT_LSB		0x1F	// Receiver timeout LSB. MSBs are in Modem Config
#define RFM96_REG_PREAMBLE_MSB			0x20	// Preamble size MSB
#define RFM96_REG_PREAMBLE_LSB			0x21	// Preamble size MSB
#define RFM96_REG_PAYLOAD_LENGTH		0x22	// Payload Lenght
#define RFM96_REG_MAX_PAYLOAD_LENGTH	0x23	// Maximum Payload Lenght
#define RFM96_REG_HOP_PERIOD			0x24	// FHSS HOP Period
#define RFM96_REG_FIFO_RX_BYTE_ADDR		0x25	// Address of last byte written in FIFO
#define RFM96_REG_MODEM_CONFIG_3		0x26	// Modem Configuration 3
#define RFM96_REG_FREQ_ERROR_MSB		0x28	// Estimated frequency error MSB
#define RFM96_REG_FREQ_ERROR_MID		0x29	// Estimated frequency error MID
#define RFM96_REG_FREQ_ERROR_LSB		0x2A	// Estimated frequency error LSB
#define RFM96_REG_RSSI_WIDEBAND			0x2C	// Wideband RSSI measurement
#define RFM96_REG_LF_FREQ_1				0x2F	// Optimize receiver
#define RFM96_REG_LF_FREQ_2				0x30	// Optimize receiver
#define RFM96_REG_DETECTION_OPTIMIZE	0x31	// Detection Optimize for Spreading Factor 6
#define RFM96_REG_INVERTIQ				0x33	// Invert LoRa I and Q signals
#define RFM96_REG_HIGH_BW_OPTIMIZE1		0x36	// Sensitivity optimization for 500 kHz bandwidth
#define RFM96_REG_DETECTION_THRESHOLD	0x37	// Detection Threshold for Spreading Factor 6
#define RFM96_REG_SYNC_WORD				0x39	// Sync Word
#define RFM96_REG_HIGH_BW_OPTIMIZE2		0x3A	// Sensitivity optimization for 500 kHz bandwidth
#define RFM96_REG_INVERTIQ2				0x3B	// Optimize for inverted IQ

/* LoRa Operation Modes (section 4.1.3) */
#define RFM96_LOW_FREQ_MODE				0x08
#define RFM96_GET_LORA_MODE(reg_val)	(reg_val&0b10000111)
#define RFM96_LORA_MODE_SLEEP			0x80
#define RFM96_LORA_MODE_STANDBY			0x81
#define RFM96_LORA_MODE_FSTX			0x82
#define RFM96_LORA_MODE_TX				0x83
#define RFM96_LORA_MODE_FSRX			0x84
#define RFM96_LORA_MODE_RX_CONT			0x85
#define RFM96_LORA_MODE_RX_SINGLE		0x86
#define RFM96_LORA_MODE_CAD				0x87

/* LoRa Bandwidths (section 4.1.1.4) */
#define RFM96_LORA_BW_CLR(reg_val)		(reg_val&0x0F)
#define RFM96_LORA_BW_MASK(reg_val)		(reg_val&0xF0)
#define RFM96_LORA_BW_7_8				0x00	// 7.8 kHz
#define RFM96_LORA_BW_10_4				0x10	// 10.4 kHz
#define RFM96_LORA_BW_15_6				0x20	// 15.6 kHz
#define RFM96_LORA_BW_20_8				0x30	// 20.8 kHz
#define RFM96_LORA_BW_31_25				0x40	// 31.25 kHz
#define RFM96_LORA_BW_41_7				0x50	// 41.7 kHz
#define RFM96_LORA_BW_62_5				0x60	// 62.5 kHz
#define RFM96_LORA_BW_125				0x70	// 125 kHz
#define RFM96_LORA_BW_250				0x80	// 250 kHz
#define RFM96_LORA_BW_500				0x90	// 500 kHz

/* LoRa Cyclic Redundancy Check (CRC) */
#define RFM96_LORA_CRC_SET(reg_val)		(reg_val|0x04)
#define RFM96_LORA_CRC_RESET(reg_val)	(reg_val&0xFB)

/* LoRa Low Data Rate Optimize */
#define RFM96_LORA_LDO_SET(reg_val)		(reg_val|0b00001000)
#define RFM96_LORA_LDO_RESET(reg_val)	(reg_val&0b11110111)

/* LoRa Spreading Factor (section 4.1.1.2) */
#define RFM96_LORA_SET_SF(reg_val, sf)	((reg_val&0x0F)|((sf<<4)&0xF0))

/* LoRa Power Amplifier (section 5.4.1 and 5.4.2)*/
#define RFM96_LORA_PA_RFO_PIN			0x70
#define RFM96_LORA_PA_BOOST_PIN			0x80

/* LoRa Over Current Protection (section 5.4.4) */
#define RFM96_LORA_OCP_SET(ocp_trim)	(0x20|(0x1F & ocp_trim))
#define RFM96_LORA_OCP_RESET			0x00

/* LoRa Coding Rate (section 4.1.1.3) */
#define RFM96_LORA_SET_CR(reg_val, cr)	((reg_val&0xF1)|cr)
#define RFM96_LORA_CR_4_5				0x02
#define RFM96_LORA_CR_4_6				0x04
#define RFM96_LORA_CR_4_7				0x06
#define RFM96_LORA_CR_4_8				0x08

/* LoRa Header Modes (section 4.1.1.6) */
#define RFM96_LORA_EXPLICIT(reg_val)	(reg_val & 0b11111110)
#define RFM96_LORA_IMPLICIT(reg_val)	(reg_val | 0b00000001)

/* Macros */
#define RFM96_IGNORE_RESET_PIN			0xFFFF	// Use this in init to ignore reset pin
#define RFM96_TIMEOUT					100

typedef struct _rfm96{
	SPI_HandleTypeDef spi_bus;
	GPIO_TypeDef* ss_gpio_port;
	uint16_t ss_pin;
	GPIO_TypeDef* reset_gpio_port;
	uint16_t reset_pin;
	long frequency;
}RFM96;

/* Radio Handling */
HAL_StatusTypeDef RFM96_set_pins(RFM96* sensor, SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, GPIO_TypeDef* reset_gpio_port, uint16_t reset_pin);
HAL_StatusTypeDef RFM96_init(RFM96* sensor, long frequency, uint8_t sensor_version);
HAL_StatusTypeDef RFM96_reset(RFM96* sensor);

/* Radio Configuration */
HAL_StatusTypeDef RFM96_set_op_mode(RFM96* sensor, uint8_t mode);
HAL_StatusTypeDef RFM96_set_frequency(RFM96* sensor, long frequency);
HAL_StatusTypeDef RFM96_set_tx_power(RFM96* sensor, int power, uint8_t pa_out_pin);
HAL_StatusTypeDef RFM96_set_spreading_factor(RFM96* sensor, uint8_t sf);
HAL_StatusTypeDef RFM96_set_signal_bandwidth(RFM96* sensor, uint8_t bw);
HAL_StatusTypeDef RFM96_set_preamble_lenght(RFM96* sensor, uint16_t lenght);
HAL_StatusTypeDef RFM96_set_sync_word(RFM96* sensor, uint8_t sync_word);
HAL_StatusTypeDef RFM96_set_coding_rate(RFM96* sensor, uint8_t coding_rate);
HAL_StatusTypeDef RFM96_set_ocp(RFM96* sensor, uint16_t current_mA);
HAL_StatusTypeDef RFM96_disable_ocp(RFM96* sensor);
HAL_StatusTypeDef RFM96_enable_crc(RFM96* sensor);
HAL_StatusTypeDef RFM96_disable_crc(RFM96* sensor);
HAL_StatusTypeDef RFM96_explicit_header_mode(RFM96* sensor);
HAL_StatusTypeDef RFM96_implicit_header_mode(RFM96* sensor);
// enable/disable invert_iq
HAL_StatusTypeDef RFM96_set_ldo_flag(RFM96* sensor);

/* Radio Data */
HAL_StatusTypeDef RFM96_get_packet_rssi(RFM96* sensor, int* rssi);
HAL_StatusTypeDef RFM96_get_packet_snr(RFM96* sensor, float* snr);
HAL_StatusTypeDef RFM96_get_spreading_factor(RFM96* sensor, uint8_t* sf);
HAL_StatusTypeDef RFM96_get_raw_signal_bandwidth(RFM96* sensor, uint8_t* raw_sbw);
HAL_StatusTypeDef RFM96_get_signal_bandwidth(RFM96* sensor, long* sbw);
HAL_StatusTypeDef RFM96_raw_sbw_to_long(uint8_t raw_sbw, long* sbw);
// packet_frequency_error
HAL_StatusTypeDef RFM96_random(RFM96* sensor, uint8_t* random);

/* Packet Transmission */
HAL_StatusTypeDef RFM96_send_packet(RFM96* sensor, void* packet, uint8_t length);
HAL_StatusTypeDef RFM96_read_packet(RFM96* sensor, void* packet, uint8_t length);
// handle_dio0
// on_receive
// available
// is_transmiting

#ifdef __cplusplus
}
#endif
#endif//LIB_ZENITH_RFM96_H

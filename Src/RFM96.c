#include "RFM96.h"

#define READ_REG(reg_addr, pvalue)\
SPI_read_register(sensor->spi_bus, sensor->ss_gpio_port, sensor->ss_pin, reg_addr, pvalue, RFM96_TIMEOUT)

#define BURST_READ(spi_bus, start_addr, pvalue, size);\
HAL_StatusTypeDef I2C_burst_read(i2c_bus, BMP280_I2C_ADDR, start_addr, pvalue, size, BMP280_TIMEOUT);

#define WRITE_REG(reg_addr, value)\
SPI_write_register(sensor->spi_bus, sensor->ss_gpio_port, sensor->ss_pin, reg_addr, value, RFM96_TIMEOUT)

/*********************************
		Radio Handling
*********************************/
// DONE - Need to test
HAL_StatusTypeDef RFM96_set_pins(RFM96* sensor, SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, GPIO_TypeDef* reset_gpio_port, uint16_t reset_pin){
	sensor->spi_bus = spi_bus;
	sensor->ss_gpio_port = ss_gpio_port;
	sensor->ss_pin = ss_pin;
	sensor->reset_gpio_port = reset_gpio_port;
	sensor->reset_pin = reset_pin;

	return HAL_OK;
}

// DONE - Depending on set_tx_power and set_frequency/get_packet_rssi to complete
HAL_StatusTypeDef RFM96_init(RFM96* sensor, long frequency, uint8_t sensor_version){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_version = 0;
	uint8_t read_value = 0;

	/* Performs Reset */
	status = RFM96_reset(sensor);
	RETURN_ON_ERROR(status);

	/* Verifies Version */
	status = READ_REG(RFM96_REG_VERSION, &read_version);
	RETURN_ON_ERROR(status);
	if(read_version != sensor_version){
		return HAL_ERROR;
	}

	/* Set Sleep Mode */
	status = RFM96_set_op_mode(sensor, RFM96_LORA_MODE_SLEEP);
	RETURN_ON_ERROR(status);

	/*set lora mode*/
	status = RFM96_set_op_mode(sensor, RFM96_LORA_MODE_SLEEP);
	RETURN_ON_ERROR(status);

	/* Set Frequency */
	status = RFM96_set_frequency(sensor, frequency);
	RETURN_ON_ERROR(status);

	/* Set Base address */
	status = WRITE_REG(RFM96_REG_FIFO_RX_BASE_ADDR, 0x00);				//BASE ADDR TA ESTRANHA TBM
	RETURN_ON_ERROR(status);											//DEVIA SER DIFERENTE PRA RX E TX
	status = WRITE_REG(RFM96_REG_FIFO_TX_BASE_ADDR, 0x00);
	RETURN_ON_ERROR(status);

	/* Set LNA boost */
	status = READ_REG(RFM96_REG_LNA, &read_value);
	RETURN_ON_ERROR(status);
	status = WRITE_REG(RFM96_REG_LNA, read_value|0x03);
	RETURN_ON_ERROR(status);

	/* Set auto AGC */
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_3, 0x04);
	RETURN_ON_ERROR(status);

	/* Set Output Power */
	status = RFM96_set_tx_power(sensor, 17, RFM96_LORA_PA_BOOST_PIN);
	RETURN_ON_ERROR(status);

	/* Set Standby Mode */
	status = RFM96_set_op_mode(sensor, RFM96_LORA_MODE_STANDBY);
	return status;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_reset(RFM96* sensor){
	HAL_StatusTypeDef status = HAL_OK;

	if(sensor->ss_pin != RFM96_IGNORE_RESET_PIN){
		HAL_GPIO_WritePin(sensor->reset_gpio_port, sensor->reset_pin, GPIO_PIN_RESET);
		HAL_Delay(10);
		HAL_GPIO_WritePin(sensor->reset_gpio_port, sensor->reset_pin, GPIO_PIN_SET);
		HAL_Delay(10);
	}

	return status;
}


/*********************************
		Radio Configuration
*********************************/
// DONE - Need to test
HAL_StatusTypeDef RFM96_set_op_mode(RFM96* sensor, uint8_t mode){
	return WRITE_REG(RFM96_REG_OP_MODE, mode);
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_set_frequency(RFM96* sensor, long frequency){
	HAL_StatusTypeDef status = HAL_OK;
	uint64_t frf = ((uint64_t)frequency << 19)/ 32000000;

	sensor->frequency = frequency;

	status = WRITE_REG(RFM96_REG_FRF_MSB, (uint8_t)(frf >> 16));
	RETURN_ON_ERROR(status);
	status = WRITE_REG(RFM96_REG_FRF_MID, (uint8_t)(frf >> 8));
	RETURN_ON_ERROR(status);
	status = WRITE_REG(RFM96_REG_FRF_LSB, (uint8_t)(frf));
	return status;
}

// PARTIAL - Need to implement MaxPower and understand OCP values
HAL_StatusTypeDef RFM96_set_tx_power(RFM96* sensor, int power, uint8_t pa_out_pin){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t pa_config_val = 0;

	/* Protects against invalid values */
	if(power < 0){
		power = 0;
	}

	/* Selects Output Pin */
	if(pa_out_pin == RFM96_LORA_PA_RFO_PIN){
		/* Low Power */
		if(power > 14){
			power = 14;
		}

		/* Default Operation (section 5.4.3) */
		status = WRITE_REG(RFM96_REG_PA_DAC, 0x84);
		RETURN_ON_ERROR(status);
		status = RFM96_set_ocp(sensor, 100);				///?
		RETURN_ON_ERROR(status);
	}
	else if(pa_out_pin == RFM96_LORA_PA_BOOST_PIN){
		/* High Power */
		if(power > 17){
			/* Protects against invalid values */
			if(power > 20){
				power = 20;
			}

			/* +20 dBm Operation (section 5.4.3) */
			status = WRITE_REG(RFM96_REG_PA_DAC, 0x87);
			RETURN_ON_ERROR(status);
			status = RFM96_set_ocp(sensor, 140);			///?
			RETURN_ON_ERROR(status);
		}
		else{
			/* Default Operation (section 5.4.3) */
			status = WRITE_REG(RFM96_REG_PA_DAC, 0x84);
			RETURN_ON_ERROR(status);
			status = RFM96_set_ocp(sensor, 100);			///?
			RETURN_ON_ERROR(status);
		}
	}
	else{
		/* Invalid Output Pin */
		return HAL_ERROR;
	}

	/* Writes Final value in PaConfig */
	pa_config_val = ((uint8_t)power)|pa_out_pin;
	return WRITE_REG(RFM96_REG_PA_CONFIG, pa_config_val);
}

// PARTIAL - Need to implement Special Case SF6
HAL_StatusTypeDef RFM96_set_spreading_factor(RFM96* sensor, uint8_t sf){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t reg_val = 0;

	/* Protects against invalid spreading factor values */
	if(sf < 6){
		sf = 6;
	}
	else if(sf > 12){
		sf = 12;
	}

	/* Special Case: Spreading Factor = 6 (section 4.1.1.2) */
	if(sf == 6){
		/* Set header to Implicit Mode */
		/* Set bits 2-0 of register detection optimize (address 0x31) to value "0b101" */
		/* Set register detection threshold (address 0x37) to value 0x0C */
	}

	/* Set sf in Register ModemConfig2 */
	status = READ_REG(RFM96_REG_MODEM_CONFIG_2, &reg_val);
	RETURN_ON_ERROR(status);
	reg_val = RFM96_LORA_SET_SF(reg_val, sf);
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_2, reg_val);
	RETURN_ON_ERROR(status);

	return RFM96_set_ldo_flag(sensor);
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_set_signal_bandwidth(RFM96* sensor, uint8_t bw){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_1, &read_value);
	RETURN_ON_ERROR(status);
	read_value = (RFM96_LORA_BW_CLR(read_value))|bw;
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_1, read_value);
	RETURN_ON_ERROR(status);

	return RFM96_set_ldo_flag(sensor);
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_set_preamble_lenght(RFM96* sensor, uint16_t lenght){
	HAL_StatusTypeDef status = HAL_OK;

	status = WRITE_REG(RFM96_REG_PREAMBLE_LSB, (uint8_t)(lenght));
	RETURN_ON_ERROR(status);
	status = WRITE_REG(RFM96_REG_PREAMBLE_MSB, (uint8_t)(lenght>>8));
	return status;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_set_sync_word(RFM96* sensor, uint8_t sync_word){
	return WRITE_REG(RFM96_REG_SYNC_WORD, sync_word);
}

// DONE - Need to test & protect macros against wrong values
HAL_StatusTypeDef RFM96_set_coding_rate(RFM96* sensor, uint8_t coding_rate){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t reg_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_1, &reg_value);
	RETURN_ON_ERROR(status);
	reg_value = RFM96_LORA_SET_CR(reg_value, coding_rate);
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_1, reg_value);
	RETURN_ON_ERROR(status);

	return RFM96_set_ldo_flag(sensor);
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_set_ocp(RFM96* sensor, uint16_t current_mA){
	uint8_t ocp_trim = 0;
	uint8_t ocp_val = 0;

	/* calculates register value corresponding to current (section 5.4.4) */
	if(current_mA >= 240){
		ocp_trim = 28;
	}
	else if(current_mA > 120){
		ocp_trim = (current_mA + 30)/10;
	}
	else if(current_mA >= 45){
		ocp_trim = (current_mA - 45)/5;
	}
	else{
		return RFM96_disable_ocp(sensor);
	}

	ocp_val = RFM96_LORA_OCP_SET(ocp_trim);

	return WRITE_REG(RFM96_REG_OCP, ocp_val);
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_disable_ocp(RFM96* sensor){
	return WRITE_REG(RFM96_REG_OCP, RFM96_LORA_OCP_RESET);
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_enable_crc(RFM96* sensor){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_2, &read_value);
	RETURN_ON_ERROR(status);
	read_value = RFM96_LORA_CRC_SET(read_value);
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_2, read_value);
	return status;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_disable_crc(RFM96* sensor){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_2, &read_value);
	RETURN_ON_ERROR(status);
	read_value = RFM96_LORA_CRC_RESET(read_value);
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_2, read_value);
	return status;
}


HAL_StatusTypeDef RFM96_explicit_header_mode(RFM96* sensor){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t reg_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_1, &reg_value);
	RETURN_ON_ERROR(status);
	reg_value = RFM96_LORA_EXPLICIT(reg_value);
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_1, reg_value);
	return status;
}

HAL_StatusTypeDef RFM96_implicit_header_mode(RFM96* sensor){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t reg_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_1, &reg_value);
	RETURN_ON_ERROR(status);
	reg_value = RFM96_LORA_IMPLICIT(reg_value);
	status = WRITE_REG(RFM96_REG_MODEM_CONFIG_1, reg_value);
	return status;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_set_ldo_flag(RFM96* sensor){
	HAL_StatusTypeDef status = HAL_OK;
	long sbw = 0;
	uint8_t sf = 0;
	long symbol_duration;
	uint8_t reg_config3_val = 0;

	/* Read arguments from registers*/
	status = RFM96_get_signal_bandwidth(sensor, &sbw);
	RETURN_ON_ERROR(status);
	status = RFM96_get_spreading_factor(sensor, &sf);
	RETURN_ON_ERROR(status);
	status = READ_REG(RFM96_REG_MODEM_CONFIG_3, &reg_config3_val);
	RETURN_ON_ERROR(status);

	/* Calculates and sets ldo value */
	symbol_duration = 1000/(sbw/(1L << sf)) ; // Section 4.1.1.5 and 4.1.1.6
	if(symbol_duration > 16){
		reg_config3_val = RFM96_LORA_LDO_SET(reg_config3_val);
	}
	else{
		reg_config3_val = RFM96_LORA_LDO_RESET(reg_config3_val);
	}

	return WRITE_REG(RFM96_REG_MODEM_CONFIG_3, reg_config3_val);
}


/*********************************
			Radio Data
*********************************/
// DONE - Need to test and verify formula
HAL_StatusTypeDef RFM96_get_packet_rssi(RFM96* sensor, int* rssi){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = READ_REG(RFM96_REG_PKT_RSSI_VALUE, &read_value);
	RETURN_ON_ERROR(status);
	*rssi = read_value - (sensor->frequency < 868E6 ? 164 : 157);
	return HAL_OK;
}

// DONE - Need to test and verify formula
HAL_StatusTypeDef RFM96_get_packet_snr(RFM96* sensor, float* snr){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = READ_REG(RFM96_REG_PKT_SNR_VALUE, &read_value);
	RETURN_ON_ERROR(status);
	*snr = read_value*0.25;
	return HAL_OK;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_get_spreading_factor(RFM96* sensor, uint8_t* sf){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_2, &read_value);
	RETURN_ON_ERROR(status);
	*sf = read_value >> 4;
	return HAL_OK;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_get_raw_signal_bandwidth(RFM96* sensor, uint8_t* raw_sbw){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t read_value = 0;

	status = READ_REG(RFM96_REG_MODEM_CONFIG_1, &read_value);
	RETURN_ON_ERROR(status);
	*raw_sbw = RFM96_LORA_BW_MASK(read_value);
	return HAL_OK;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_get_signal_bandwidth(RFM96* sensor, long* sbw){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t raw_sbw = 0;

	status = RFM96_get_raw_signal_bandwidth(sensor, &raw_sbw);
	RETURN_ON_ERROR(status);
	status = RFM96_raw_sbw_to_long(raw_sbw, sbw);
	return status;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_raw_sbw_to_long(uint8_t raw_sbw, long* sbw){
	switch(raw_sbw){
		case RFM96_LORA_BW_7_8:
			*sbw = 7.8E3;
			return HAL_OK;
		case RFM96_LORA_BW_10_4:
			*sbw = 10.4E3;
			return HAL_OK;
		case RFM96_LORA_BW_15_6:
			*sbw = 15.6E3;
			return HAL_OK;
		case RFM96_LORA_BW_20_8:
			*sbw = 20.8E3;
			return HAL_OK;
		case RFM96_LORA_BW_31_25:
			*sbw = 31.25E3;
			return HAL_OK;
		case RFM96_LORA_BW_41_7:
			*sbw = 41.7E3;
			return HAL_OK;
		case RFM96_LORA_BW_62_5:
			*sbw = 62.5E3;
			return HAL_OK;
		case RFM96_LORA_BW_125:
			*sbw = 125E3;
			return HAL_OK;
		case RFM96_LORA_BW_250:
			*sbw = 250E3;
			return HAL_OK;
		case RFM96_LORA_BW_500:
			*sbw = 500E3;
			return HAL_OK;
	}
	return HAL_ERROR;
}

// DONE - Need to test
HAL_StatusTypeDef RFM96_random(RFM96* sensor, uint8_t* random){
	/* RSSI Wideband can be used to generate a random number */
	return READ_REG(RFM96_REG_RSSI_WIDEBAND, random);
}

/*********************************
		Packet Transmission
*********************************/
// DONE - Need to test. Depending on SPI burst read/write
HAL_StatusTypeDef RFM96_send_packet(RFM96* sensor, void* packet, uint8_t length){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t reg_op_val = 0;
	uint8_t fifo_addr_ptr = 0;

	/* Verifies if is not transmiting */
	status = READ_REG(RFM96_REG_OP_MODE, &reg_op_val);
	RETURN_ON_ERROR(status);
	reg_op_val = RFM96_GET_LORA_MODE(reg_op_val);
	if(reg_op_val != RFM96_LORA_MODE_SLEEP && reg_op_val != RFM96_LORA_MODE_STANDBY){
		return HAL_ERROR;
	}

	/* Set Standby Mode */
	status = RFM96_set_op_mode(sensor, RFM96_LORA_MODE_STANDBY);
	RETURN_ON_ERROR(status);

	/* Tx Init */
	status = READ_REG(RFM96_REG_FIFO_TX_BASE_ADDR, &fifo_addr_ptr);
	RETURN_ON_ERROR(status);
	status = WRITE_REG(RFM96_REG_FIFO_ADDR_PTR, fifo_addr_ptr);
	RETURN_ON_ERROR(status);

	status = WRITE_REG(RFM96_REG_PAYLOAD_LENGTH, length);
	RETURN_ON_ERROR(status);
	status = WRITE_REG(RFM96_REG_FIFO, length);
	RETURN_ON_ERROR(status);

	/* Write Data FIFO */
	for(int i = 0; i < length; i++){
		status = WRITE_REG(RFM96_REG_FIFO, (((uint8_t*)packet)[i]));
		RETURN_ON_ERROR(status);
	}

	/* Set Tx Mode */
	//status = WRITE_REG(RFM96_REG_IRQ_FLAGS, 0b00001000);
	//RETURN_ON_ERROR(status);
	return RFM96_set_op_mode(sensor, RFM96_LORA_MODE_TX);
}

// TO-DO
HAL_StatusTypeDef RFM96_read_packet(RFM96* sensor, void* packet, uint8_t length){
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t reg_op_val, iqr_val, fifo_addr_ptr;
	uint8_t i;

	/* Verifies if is not transmiting */
	status = READ_REG(RFM96_REG_OP_MODE, &reg_op_val);
	RETURN_ON_ERROR(status);
	reg_op_val = RFM96_GET_LORA_MODE(reg_op_val);
	if(reg_op_val == RFM96_LORA_MODE_FSTX && reg_op_val == RFM96_LORA_MODE_TX ){
		return HAL_ERROR;
	}

	/* Set Standby Mode */
	//status = RFM96_set_op_mode(sensor, RFM96_LORA_MODE_STANDBY);
	//RETURN_ON_ERROR(status);

	/* Set Header Mode */
	//status = RFM96_explicit_header_mode(sensor);
	//RETURN_ON_ERROR(status);

	/* Verifies Received Packet and Consistency */
	status = READ_REG(RFM96_REG_IRQ_FLAGS, &iqr_val);
	RETURN_ON_ERROR(status);
	if((iqr_val & 0b01100000) != 0b01000000){
		return HAL_ERROR;
	}
	status = WRITE_REG(RFM96_REG_IRQ_FLAGS, iqr_val);
	RETURN_ON_ERROR(status);

	/* Packet Length */
	status = READ_REG(RFM96_REG_RX_NB_BYTES, &i);
	RETURN_ON_ERROR(status);
	length = (i < length)?(i):(length);

	/* RX Init */
	status = READ_REG(RFM96_REG_FIFO_RX_CURRENT_ADDR, &fifo_addr_ptr);
	RETURN_ON_ERROR(status);
	status = WRITE_REG(RFM96_REG_FIFO_ADDR_PTR, fifo_addr_ptr);
	RETURN_ON_ERROR(status);

	/* Read Packet Data */
	for(i = 0; i < length; i++){
		status = READ_REG(RFM96_REG_FIFO, ((uint8_t*)(packet + i)));
		RETURN_ON_ERROR(status);
	}

	status = RFM96_set_op_mode(sensor, RFM96_LORA_MODE_RX_CONT);
	return status;
}



#undef READ_REG
#undef BURST_READ
#undef WRITE_REG

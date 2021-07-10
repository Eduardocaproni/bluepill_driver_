/*
 *	Biblioteca do protocolo de comunicacao SPI para STM32
 *	Versao em C
 *
 *	Feito por: setor de Software Baixo Nivel - Zenith Aerospace
 *	Criado em: 18 jan 2020
 */

#ifndef LIB_ZENITH_SPI_H
#define LIB_ZENITH_SPI_H
#ifdef __cplusplus
extern "C" {
#endif

/* Platform Specific Includes */
#include "zstm32.h"

/* SPI Functions */
HAL_StatusTypeDef SPI_read_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t* pvalue, uint32_t timeout);					// received data is stored in value
HAL_StatusTypeDef SPI_burst_read(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t start_addr, uint8_t* pvalue, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI_write_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t value, uint32_t timeout);
HAL_StatusTypeDef SPI_burst_write(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t start_addr, uint8_t* value, uint16_t size, uint32_t timeout);

#ifdef __cplusplus
}
#endif
#endif//LIB_ZENITH_SPI_H
/*
 *	Biblioteca do protocolo de comunicacao SPI para STM32
 *	Versao em C
 *
 *	Feito por: setor de Software Baixo Nivel - Zenith Aerospace
 *	Criado em: 18 jan 2020
 */

#ifndef LIB_ZENITH_SPI_H
#define LIB_ZENITH_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

/* Platform Specific Includes */
#include "zstm32.h"

/* SPI Functions */
HAL_StatusTypeDef SPI_read_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t* pvalue, uint32_t timeout);
HAL_StatusTypeDef SPI_burst_read(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t start_addr, uint8_t* pvalue, uint16_t size, uint32_t timeout);
HAL_StatusTypeDef SPI_write_register(SPI_HandleTypeDef spi_bus, GPIO_TypeDef* ss_gpio_port, uint16_t ss_pin, uint8_t reg_addr, uint8_t value, uint32_t timeout);

#ifdef __cplusplus
}
#endif
#endif//LIB_ZENITH_SPI_H

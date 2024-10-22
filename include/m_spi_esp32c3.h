#pragma once

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "soc/gpio_reg.h"

#define SPI_WRITE_CMD 0U
#define SPI_READ_CMD 1U

/**
 * @brief Write an 8-bit value to a specific SPI device's 8-bit register
 *
 * @param handle (spi_device_handle_t) The SPI device handle
 * @param reg (uint8_t) 8-bit SPI device register
 * @param val (uint8_t) 8-bit value to write to SPI device register
 */
void m_spi_write_register(spi_device_handle_t handle, uint8_t reg, uint8_t val);

/**
 * @brief Read an 8-bit value from a specific SPI device's 8-bit register
 *
 * @param handle (spi_device_handle_t) The SPI device handle
 * @param reg (uint8_t) 8-bit SPI device register
 * @return (uint8_t) 8-bit value from SPI device register
 */
uint8_t m_spi_read_register(spi_device_handle_t handle, uint8_t reg);

/**
 * @brief Read an 16-bit value from a specific SPI device's 16-bit register
 *
 * @param handle (spi_device_handle_t) The SPI device handle
 * @param reg (uint8_t) 8-bit SPI device register
 * @return (uint16_t) 16-bit value from SPI device register
 */
uint16_t m_spi_read_16bit(spi_device_handle_t handle, uint8_t reg);

/**
 * @brief Serial read multiple SPI device bytes starting from an 8-bit register
 *
 * @details SPI device must support burst/serial register reads
 *
 * @param handle (spi_device_handle_t) The SPI device handle
 * @param start_reg (uint8_t) 8-bit starting SPI device register
 * @param num_bytes (uint16_t) Number of bytes to retrieve from SPI device
 * @param dest (uint8_t*) Pointer to a byte/char buffer for storing received bytes
 */
void m_spi_read_registers(spi_device_handle_t handle,
                          uint8_t start_reg,
                          uint16_t num_bytes,
                          uint8_t *dest);

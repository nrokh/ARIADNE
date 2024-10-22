#include "m_spi_esp32c3.h"

spi_transaction_t spi_write_register = {
    .flags = SPI_TRANS_USE_TXDATA,
    .cmd = SPI_WRITE_CMD,
    .length = 8,
    .rxlength = 0};
spi_transaction_t spi_read_register = {
    .flags = SPI_TRANS_USE_RXDATA,
    .cmd = SPI_READ_CMD,
    .length = 8,
    .rxlength = 8};

spi_transaction_t spi_read_16 = {
    .flags = SPI_TRANS_USE_RXDATA | SPI_DEVICE_HALFDUPLEX,
    .cmd = SPI_READ_CMD,
    .length = 16,
    .rxlength = 16};

// uint8_t rx_buffer[512 * 7];
spi_transaction_t spi_read_batch = {
    //.flags = SPI_DEVICE_HALFDUPLEX,
    .cmd = SPI_READ_CMD};
//  .rx_buffer = rx_buffer};

void m_spi_write_register(spi_device_handle_t handle, uint8_t reg, uint8_t val) {
    spi_write_register.tx_data[0] = val;
    spi_write_register.addr = reg;
    spi_device_polling_transmit(handle, &spi_write_register);
}

uint8_t m_spi_read_register(spi_device_handle_t handle, uint8_t reg) {
    spi_read_register.addr = reg;
    spi_device_polling_transmit(handle, &spi_read_register);
    return spi_read_register.rx_data[0];
}

uint16_t m_spi_read_16bit(spi_device_handle_t handle, uint8_t reg) {
    spi_read_16.addr = reg;
    spi_device_polling_transmit(handle, &spi_read_16);
    uint16_t ret = ((uint16_t)spi_read_16.rx_data[1] << 8) | spi_read_16.rx_data[0];
    return ret;
}

// FUNCTIONALITY:
//   Serial read multiple SPI device bytes starting from an 8-bit register
//   Note: SPI device must support burst/serial register reads
//
// ARGUMENTS:
//   cs_pin: an enumerated M2 general-purpose I/O (GPIO) pin
//   start_reg: 8-bit SPI device register
//   num_bytes: number of bytes to retrieve from SPI device
//   dest: pointer to a byte/char buffer for storing received bytes
void m_spi_read_registers(spi_device_handle_t handle, uint8_t start_reg, uint16_t num_bytes, uint8_t *dest) {
    spi_read_batch.addr = start_reg;
    spi_read_batch.length = num_bytes * 8;
    spi_read_batch.rxlength = num_bytes * 8;
    spi_read_batch.rx_buffer = dest;
    // spi_device_polling_transmit(handle, &spi_read_batch);
    spi_device_transmit(handle, &spi_read_batch);
}

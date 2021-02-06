// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/
#include "cc1101_spi.h"

#include "cc1101_internal.h"

#define MAX_REPEATS 10

#define SPI_BURST 0x40
#define SPI_READ 0x80

/* 
* Read a single byte from one of the device's configuration registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   reg: address of CC1101 register to read
*
* Returns:
*   Transaction struct containing register contents and device status
*/
spi_transaction_t cc1101_spi_read_config_register(cc1101_t* cc1101, unsigned char reg) {
    struct spi_transfer t = {0};
    struct spi_message m;
    spi_transaction_t transaction;

    WARN_ON(reg > 0x2E);

    transaction.header = reg | SPI_READ;
    transaction.data = 0;

    t.tx_buf = &transaction;
    t.rx_buf = &transaction;
    t.len = 2;
    t.cs_change = 0;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_sync(cc1101->spi, &m);

    return transaction;
}

/* 
* Writes a single byte to one of the device's registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   reg: address of CC1101 register to write
*   value: byte to write to the register
*
* Returns:
*   Device status
*/
unsigned char cc1101_spi_write_config_register(cc1101_t* cc1101, unsigned char reg, unsigned char value) {
    struct spi_transfer t = {0};
    struct spi_message m;
    spi_transaction_t transaction;

    WARN_ON(reg > 0x2E);

    transaction.header = reg;
    transaction.data = value;

    t.tx_buf = &transaction,
    t.rx_buf = &transaction,
    t.len = 2,
    t.cs_change = 0,

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_sync(cc1101->spi, &m);

    return transaction.header;
}

/* 
* Read a sequence of bytes from the device's registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   reg: address of CC1101 registers to start reading from.
*   out: pointer to a buffer that will contain the register values read from the device
*   len: size of out, number of bytes to read from the device
*
* Returns:
*   Device status
*/
static unsigned char read_registers(cc1101_t* cc1101, unsigned char reg, unsigned char* out, unsigned char len) {
    struct spi_transfer t = {0};
    struct spi_message m;
    unsigned char transaction[FIFO_LEN + 1];

    transaction[0] = reg | SPI_BURST | SPI_READ;

    t.tx_buf = &transaction;
    t.rx_buf = &transaction;
    t.len = len + 1;
    t.cs_change = 0;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_sync(cc1101->spi, &m);

    memcpy(out, &transaction[1], len);

    return transaction[0];
}

/* 
* Read a sequence of bytes from the device's config registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   registers: pointer to a buffer that will contain the values read from the device
*   len: size of out, number of bytes to read from the device
*
* Returns:
*   Device status
*
*/
unsigned char cc1101_spi_read_config_registers(cc1101_t* cc1101, unsigned char* registers, unsigned char len) {
    BUG_ON(len > CONFIG_REGISTERS_LEN);
    return read_registers(cc1101, 0x00, registers, len);
}

/* 
* Read a sequence of bytes from the device's PATABLE via SPI
* 
* Arguments:
*   cc1101: device struct
*   patable: pointer to a buffer that will contain the values read from the device
*   len: size of out, number of bytes to read from the device
*
* Returns:
*   Device status
*/
unsigned char cc1101_spi_read_patable(cc1101_t* cc1101, unsigned char* patable, unsigned char len) {
    BUG_ON(len > PATABLE_LEN);
    return read_registers(cc1101, PATABLE, patable, len);
}

/* 
* Read a sequence of bytes from the device's RXFIFO via SPI
* 
* Arguments:
*   cc1101: device struct
*   rx_bytes: pointer to a buffer that will contain the values read from the device
*   len: size of out, number of bytes to read from the device
*
* Returns:
*   Device status
*/
unsigned char cc1101_spi_read_rxfifo(cc1101_t* cc1101, unsigned char* rx_bytes, unsigned char len) {
    BUG_ON(len > FIFO_LEN);
    return read_registers(cc1101, FIFO, rx_bytes, len);
}

/* 
* Write a sequence of bytes to the device's registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   reg: address of CC1101 registers to start writing to. Must have the burst bit set
*   in: pointer to a buffer containing a byte sequence to write to the device registers
*   len: number of bytes in buffer that will be written to the device
*
* Returns:
*   Device status
*/
static unsigned char write_registers(cc1101_t* cc1101, unsigned char reg, const unsigned char* in, unsigned char len) {
    struct spi_transfer t = {0};
    struct spi_message m;
    unsigned char transaction[FIFO_LEN + 1];

    transaction[0] = reg | SPI_BURST;
    memcpy(&transaction[1], in, len);

    t.tx_buf = &transaction;
    t.rx_buf = &transaction;
    t.len = len + 1;
    t.cs_change = 0;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_sync(cc1101->spi, &m);

    return transaction[0];
}

/* 
* Write a sequence of bytes to the device's configuration registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   registers: pointer to a buffer containing a byte sequence to write to the device registers
*   len: number of bytes in buffer that will be written to the device
*
* Returns:
*   Device status
*/
unsigned char cc1101_spi_write_config_registers(cc1101_t* cc1101, const unsigned char* registers, unsigned char len) {
    BUG_ON(len > CONFIG_REGISTERS_LEN);
    return write_registers(cc1101, 0x00, registers, len);
}

/* 
* Write a sequence of bytes to the device's PATABLE registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   patable: pointer to a buffer containing a byte sequence to write to the PATABLE
*   len: number of bytes in buffer that will be written to the device
*
* Returns:
*   Device status
*/
unsigned char cc1101_spi_write_patable(cc1101_t* cc1101, const unsigned char* patable, unsigned char len) {
    BUG_ON(len > PATABLE_LEN);
    return write_registers(cc1101, PATABLE, patable, len);
}

/* 
* Write a sequence of bytes to the device's TXFIFO via SPI
* 
* Arguments:
*   cc1101: device struct
*   tx_bytes: pointer to a buffer containing a byte sequence to write to the TXFIFO
*   len: number of bytes in buffer that will be written to the device
*
* Returns:
*   Device status
*/
unsigned char cc1101_spi_write_txfifo(cc1101_t* cc1101, const unsigned char* tx_bytes, unsigned char len) {
    BUG_ON(len > FIFO_LEN);
    return write_registers(cc1101, FIFO, tx_bytes, len);
}

/* 
* Sends a command to the device via SPI
* 
* Arguments:
*   cc1101: device struct
*   command: command to send to the device
*
* Returns:
*   Device status
*/
unsigned char cc1101_spi_send_command(cc1101_t* cc1101, unsigned char command) {
    struct spi_transfer t = {0};
    struct spi_message m;
    unsigned char transaction;

    WARN_ON(command < 0x30 || command > 0x3D);

    transaction = command;

    t.tx_buf = &transaction;
    t.rx_buf = &transaction;
    t.len = 1;
    t.cs_change = 0;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_sync(cc1101->spi, &m);

    return transaction;
}

/* 
* Read a single byte from one of the device's status registers via SPI
* 
* Arguments:
*   cc1101: device struct
*   reg: address of CC1101 status register to read
*
* Returns:
*   Transaction struct containing register contents and device status
*/
static spi_transaction_t cc1101_spi_read_status_register_int(cc1101_t* cc1101, unsigned char reg) {
    struct spi_transfer t = {0};
    struct spi_message m;
    spi_transaction_t transaction;

    WARN_ON(reg < 0x30 || reg > 0x3D);

    // Status registers require the burst bit to be set
    transaction.header = reg | SPI_BURST | SPI_READ;
    transaction.data = 0;

    t.tx_buf = &transaction;
    t.rx_buf = &transaction;
    t.len = 2;
    t.cs_change = 0;

    spi_message_init(&m);
    spi_message_add_tail(&t, &m);
    spi_sync(cc1101->spi, &m);

    return transaction;
}

/* 
* Read a single byte from one of the device's status registers via SPI
* 
* The double-read implemented here is recommended by the CC1101 Errata to prevent corrupt values 
* from being read from the device.
*
* Arguments:
*   cc1101: device struct
*   reg: address of CC1101 status register to read
*
* Returns:
*   Transaction struct containing register contents and device status
*/
spi_transaction_t cc1101_spi_read_status_register(cc1101_t* cc1101, unsigned char reg) {
    spi_transaction_t result, result_check;

    // Read status register
    result = cc1101_spi_read_status_register_int(cc1101, reg);
    while (1) {
        // Read again
        result_check = cc1101_spi_read_status_register_int(cc1101, reg);

        // If the values match, the value was valid
        if (result.data == result_check.data) {
            break;
        }
        // If not, continue until two reads agree
        else {
            result = result_check;
        }
    }

    return result;
}
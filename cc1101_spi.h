// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/
#ifndef CC1101_SPI_H
#define CC1101_SPI_H

#include <linux/spi/spi.h>

#include "cc1101_internal.h"

typedef struct {
    unsigned char header;
    unsigned char data;
} spi_transaction_t;

spi_transaction_t cc1101_spi_read_config_register(cc1101_t* cc1101, unsigned char reg);
unsigned char cc1101_spi_write_config_register(cc1101_t* cc1101, unsigned char reg, unsigned char value);

unsigned char cc1101_spi_read_config_registers(cc1101_t* cc1101, unsigned char* registers, unsigned char len);
unsigned char cc1101_spi_read_patable(cc1101_t* cc1101, unsigned char* patable, unsigned char len);
unsigned char cc1101_spi_read_rxfifo(cc1101_t* cc1101, unsigned char* rx_bytes, unsigned char len);

unsigned char cc1101_spi_write_config_registers(cc1101_t* cc1101, const unsigned char* registers, unsigned char len);
unsigned char cc1101_spi_write_patable(cc1101_t* cc1101, const unsigned char* patable, unsigned char len);
unsigned char cc1101_spi_write_txfifo(cc1101_t* cc1101, const unsigned char* tx_bytes, unsigned char len);

spi_transaction_t cc1101_spi_read_status_register_once(cc1101_t* cc1101, unsigned char reg);
spi_transaction_t cc1101_spi_read_status_register(cc1101_t* cc1101, unsigned char reg);
unsigned char cc1101_spi_send_command(cc1101_t* cc1101, unsigned char command);

#endif
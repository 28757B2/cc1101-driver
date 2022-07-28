// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/
#ifndef CC1101_H
#define CC1101_H

#define CONFIG_REGISTERS_LEN    0x2F
#define PATABLE_LEN             8

typedef unsigned char cc1101_device_config_t[CONFIG_REGISTERS_LEN];
typedef unsigned char cc1101_patable_t[PATABLE_LEN];

#define MOD_2FSK    0
#define MOD_GFSK    1
#define MOD_OOK     3
#define MOD_4FSK    4
#define MOD_MSK     7

#define CS_DISABLED 0
#define CS_RELATIVE 1
#define CS_ABSOLUTE 2

struct cc1101_common_config {
    unsigned frequency;
    unsigned char modulation;
    unsigned char baud_rate_mantissa;
    unsigned char baud_rate_exponent;
    unsigned char deviation_mantissa;
    unsigned char deviation_exponent;
    unsigned long sync_word;
} __attribute__ ((packed));

// Message sent from userspace via IOCTL containing RX mode configuration parameters
struct cc1101_rx_config {
    struct cc1101_common_config common;
    unsigned char bandwidth_mantissa;
    unsigned char bandwidth_exponent;
    unsigned char max_lna_gain;
    unsigned char max_dvga_gain;
    unsigned char magn_target;
    unsigned char carrier_sense_mode;
    signed char carrier_sense;
    unsigned packet_length;
} __attribute__ ((packed));

// Message sent from userspace via IOCTL containing TX mode configuration parameters
struct cc1101_tx_config {
    struct cc1101_common_config common;
    unsigned char tx_power;
} __attribute__ ((packed));

#endif
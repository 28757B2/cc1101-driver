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
    __u32 frequency;
    __u8 modulation;
    __u8 baud_rate_mantissa;
    __u8 baud_rate_exponent;
    __u8 deviation_mantissa;
    __u8 deviation_exponent;
    __u32 sync_word;
};

// Message sent from userspace via IOCTL containing RX mode configuration parameters
struct cc1101_rx_config {
    struct cc1101_common_config common;
    __u8 bandwidth_mantissa;
    __u8 bandwidth_exponent;
    __u8 max_lna_gain;
    __u8 max_dvga_gain;
    __u8 magn_target;
    __u8 carrier_sense_mode;
    __s8 carrier_sense;
    __u32 packet_length;
};

// Message sent from userspace via IOCTL containing TX mode configuration parameters
struct cc1101_tx_config {
    struct cc1101_common_config common;
    __u8 tx_power;
};

#endif
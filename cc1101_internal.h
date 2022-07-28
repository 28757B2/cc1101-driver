// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/
#ifndef CC1101_INTERNAL_H
#define CC1101_INTERNAL_H

#include <linux/kernel.h>
#include <linux/kfifo.h>

#include "cc1101.h"

#define DRIVER_VERSION 3

// Configuration Registers - CC1101 Datasheet Table 43
// Generated in SmartRF Studio with config string #define @RN@ @<<@ 0x@AH@ @<<@ // @Rd@
#define IOCFG2    0x00      // GDO2 Output Pin Configuration
#define IOCFG1    0x01      // GDO1 Output Pin Configuration
#define IOCFG0    0x02      // GDO0 Output Pin Configuration
#define FIFOTHR   0x03      // RX FIFO and TX FIFO Thresholds
#define SYNC1     0x04      // Sync Word, High Byte
#define SYNC0     0x05      // Sync Word, Low Byte
#define PKTLEN    0x06      // Packet Length
#define PKTCTRL1  0x07      // Packet Automation Control
#define PKTCTRL0  0x08      // Packet Automation Control
#define ADDR      0x09      // Device Address
#define CHANNR    0x0A      // Channel Number
#define FSCTRL1   0x0B      // Frequency Synthesizer Control
#define FSCTRL0   0x0C      // Frequency Synthesizer Control
#define FREQ2     0x0D      // Frequency Control Word, High Byte
#define FREQ1     0x0E      // Frequency Control Word, Middle Byte
#define FREQ0     0x0F      // Frequency Control Word, Low Byte
#define MDMCFG4   0x10      // Modem Configuration
#define MDMCFG3   0x11      // Modem Configuration
#define MDMCFG2   0x12      // Modem Configuration
#define MDMCFG1   0x13      // Modem Configuration
#define MDMCFG0   0x14      // Modem Configuration
#define DEVIATN   0x15      // Modem Deviation Setting
#define MCSM2     0x16      // Main Radio Control State Machine Configuration
#define MCSM1     0x17      // Main Radio Control State Machine Configuration
#define MCSM0     0x18      // Main Radio Control State Machine Configuration
#define FOCCFG    0x19      // Frequency Offset Compensation Configuration
#define BSCFG     0x1A      // Bit Synchronization Configuration
#define AGCCTRL2  0x1B      // AGC Control
#define AGCCTRL1  0x1C      // AGC Control
#define AGCCTRL0  0x1D      // AGC Control
#define WOREVT1   0x1E      // High Byte Event0 Timeout
#define WOREVT0   0x1F      // Low Byte Event0 Timeout
#define WORCTRL   0x20      // Wake On Radio Control
#define FREND1    0x21      // Front End RX Configuration
#define FREND0    0x22      // Front End TX Configuration
#define FSCAL3    0x23      // Frequency Synthesizer Calibration
#define FSCAL2    0x24      // Frequency Synthesizer Calibration
#define FSCAL1    0x25      // Frequency Synthesizer Calibration
#define FSCAL0    0x26      // Frequency Synthesizer Calibration
#define RCCTRL1   0x27      // RC Oscillator Configuration
#define RCCTRL0   0x28      // RC Oscillator Configuration
#define FSTEST    0x29      // Frequency Synthesizer Calibration Control
#define PTEST     0x2A      // Production Test
#define AGCTEST   0x2B      // AGC Test
#define TEST2     0x2C      // Various Test Settings
#define TEST1     0x2D      // Various Test Settings
#define TEST0     0x2E      // Various Test Settings

// Command strobes - CC1101 Datasheet Table 42
#define SRES      0x30
#define SFSTXON   0x31
#define SXOFF     0x32
#define SCAL      0x33
#define SRX       0x34
#define STX       0x35
#define SIDLE     0x36
#define SAFC      0x37
#define SWOR      0x38
#define SPWD      0x39
#define SFRX      0x3A
#define SFTX      0x3B
#define SWORRST   0x3C
#define SNOP      0x3D

// Status Registers - CC1101 Datasheet Table 44
#define PARTNUM         0x30
#define VERSION         0x31
#define FREQEST         0x32
#define LQI             0x33
#define RSSI            0x34
#define MARCSTATE       0x35
#define WORTIME1        0x36
#define WORTIME0        0x37
#define PKTSTATUS       0x38
#define VCO_VC_DAC      0x39
#define TXBYTES         0x3A
#define RXBYTES         0x3B
#define RCCTRL1_STATUS  0x3C
#define RCCTRL0_STATUS  0x3D

#define PATABLE         0x3E
#define FIFO            0x3F

#define FIFO_LEN                64

// State transition times - CC1101 Datasheet Page 54/55
#define TIME_IDLE_TO_RX_NOCAL   76      // Rounded up from 75.1
#define TIME_IDLE_TO_RX_CAL     799
#define TIME_IDLE_TO_TX_NOCAL   76      // Rounded up from 75.2
#define TIME_IDLE_TO_TX_CAL     799
#define TIME_TX_TO_RX           32      // Rounded up from 31.1 - compensates for variable baud rate
#define TIME_RX_TO_TX           32
#define TIME_TX_TO_IDLE_NOCAL   1       // Rounded up from 0.25/baud - compensates for variable baud rate
#define TIME_TX_TO_IDLE_CAL     726     // Rounded up from 725 to compensate for variable baud rate
#define TIME_RX_TO_IDLE_NOCAL   1       // Rounded up from ~0.1 
#define TIME_RX_TO_IDLE_CAL     724
#define TIME_MANUAL_CAL         735

// Error macros
#ifndef CONFIG_DYNAMIC_DEBUG
extern uint debug;
#define CC1101_DEBUG(cc1101, format, ...)    if (debug) dev_info(&cc1101->spi->dev, format "\n", ##__VA_ARGS__)
#else
#define CC1101_DEBUG(cc1101, format, ...)    dev_dbg(&cc1101->spi->dev, format "\n", ##__VA_ARGS__)
#endif

#define CC1101_INFO(cc1101, format, ...)     dev_info(&cc1101->spi->dev, format "\n", ##__VA_ARGS__)
#define CC1101_ERROR(cc1101, format, ...)    dev_err(&cc1101->spi->dev, format "\n", ##__VA_ARGS__)

// Driver state
typedef enum {
    MODE_IDLE,
    MODE_TX,
    MODE_RX
} cc1101_mode_t;


// Struct to hold per-device state
typedef struct {
    struct spi_device *spi;
    dev_t devt;
    int irq;
    struct mutex chrdev_lock;
    struct mutex device_lock;
    cc1101_mode_t mode;
    struct cc1101_tx_config tx_config;
    struct cc1101_rx_config rx_config;
    unsigned char *current_packet;
    unsigned int bytes_remaining;
    DECLARE_KFIFO_PTR(rx_fifo, unsigned char);
    struct timer_list rx_timeout;
    struct work_struct rx_timeout_work;
} cc1101_t;

#endif
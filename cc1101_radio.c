// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/
#include "cc1101_internal.h"
#include "cc1101_radio.h"
#include "cc1101_config.h"
#include "cc1101_spi.h"

#include <linux/delay.h>
#include <linux/kfifo.h>
#include <linux/jiffies.h>

/*
*   Change the hardware and driver state. Delays based on the state transition times in the datasheet
*
*   Arguments:
*   cc1101: device struct
*   to: mode to change the device state to
*/
static void change_state(cc1101_t* cc1101, cc1101_mode_t to){

    unsigned long delay;
    unsigned char command;

    switch(to){
        case MODE_IDLE:
            command = SIDLE;
            switch(cc1101->mode) {
                case MODE_TX:
                    CC1101_DEBUG(cc1101, "TX -> IDLE");
                    delay = TIME_TX_TO_IDLE_CAL;
                    break;
                case MODE_RX:
                    CC1101_DEBUG(cc1101, "RX -> IDLE");
                    delay = TIME_RX_TO_IDLE_CAL;
                    break;
                case MODE_IDLE:
                    CC1101_DEBUG(cc1101, "IDLE -> IDLE");
                    return;
                default: 
                    CC1101_DEBUG(cc1101, "%d -> IDLE", to);
                    return;
            }
            break;
        case MODE_TX:
            command = STX;
            switch(cc1101->mode) {
                case MODE_IDLE:
                    CC1101_DEBUG(cc1101, "IDLE -> TX");
                    delay = TIME_IDLE_TO_TX_CAL;
                    break;
                case MODE_RX:
                    CC1101_DEBUG(cc1101, "RX -> TX");
                    delay = TIME_RX_TO_TX;
                    break;
                case MODE_TX:
                    CC1101_DEBUG(cc1101, "TX -> TX");
                    return;
                default: 
                    CC1101_DEBUG(cc1101, "%d -> TX", to);
                    return;
            }
            break;
        case MODE_RX:
            command = SRX;
            switch(cc1101->mode) {
                case MODE_IDLE:
                    CC1101_DEBUG(cc1101, "IDLE -> RX");
                    delay = TIME_IDLE_TO_RX_CAL;
                    break;
                case MODE_TX:
                    CC1101_DEBUG(cc1101, "TX -> RX");
                    delay = TIME_TX_TO_RX;
                    break;
                case MODE_RX:
                    CC1101_DEBUG(cc1101, "RX -> RX");
                    return;
                default: 
                    CC1101_DEBUG(cc1101, "%d -> RX", to);
                    return;
            }
            break;
        default:
            return;
    }
    // Send the command via SPI
    cc1101->mode = to;
    cc1101_spi_send_command(cc1101, command);
    udelay(delay);
}

/*
* Flush the device's RXFIFO, returning to the idle state
*
* Arguments:
*   cc1101: device struct
*/
void cc1101_flush_rx_fifo(cc1101_t *cc1101){
    cc1101_spi_send_command(cc1101, SFRX);
    cc1101->mode = MODE_IDLE;
}

/*
* Flush the device's TXFIFO, returning to the idle state
*
* Arguments:
*   cc1101: device struct
*/
void cc1101_flush_tx_fifo(cc1101_t *cc1101){
    cc1101_spi_send_command(cc1101, SFTX);
    cc1101->mode = MODE_IDLE;
}

/*
* Set the device to idle
*
* Arguments:
*   cc1101: device struct
*/
void cc1101_idle(cc1101_t* cc1101)
{
    CC1101_DEBUG(cc1101, "Idle Mode");
    change_state(cc1101, MODE_IDLE);
}

/*
* Set the device to RX
*
* Arguments:
*   cc1101: device struct
*/
void cc1101_rx(cc1101_t* cc1101)
{
    CC1101_DEBUG(cc1101, "Receive Mode");
    change_state(cc1101, MODE_RX);
}

# ifndef RXONLY
/*
* Function to transmit an arbitrary length packet (> 64 bytes)
* Arguments:
*   cc1101: device struct
*   buf: bytes to transmit
*   len: length of buf
*
*
*/
static void tx_multi(cc1101_t* cc1101, const char* buf, size_t len){

    size_t bytes_remaining, fragment_size;
    unsigned char tx_bytes;
    int fixed_packet_mode = 0;

    // Before transmission, the full packet length is left to transmit
    bytes_remaining = len;

    // Write the value that the packet counter will be at when transmission finishes into PKTLEN
    // This is recommended by the datasheet, but will not be used until the radio
    // is placed in fixed length packet mode 
    cc1101_spi_write_config_register(cc1101, PKTLEN, len % 256);
    
    // Set to continual transmit mode
    cc1101_spi_write_config_register(cc1101, PKTCTRL0, 0x02);

    // Write the first 32 byte fragment of the packet to the device
    cc1101_spi_write_txfifo(cc1101, &buf[0], 32);

    // Decrement the number of bytes remaining to transmit
    bytes_remaining -= 32;

    // Instruct the device to begin transmission
    change_state(cc1101, MODE_TX);

    CC1101_DEBUG(cc1101, "Transmitting 32 bytes, %zu remaining, %zu Total", bytes_remaining, len);

    // While there are still bytes to transmit
    while(bytes_remaining != 0){

        // Read the current number bytes in the FIFO to be transmitted
        tx_bytes = cc1101_spi_read_status_register(cc1101, TXBYTES).data;

        // Check for underflow, exit 
        // Caller will change state to IDLE and flush the TXFIFO, so handle the error by returning
        if(tx_bytes > FIFO_LEN) {
            CC1101_DEBUG(cc1101, "TXFIFO Underflow, Aborting Transmission");
            return;
        }
        
        // If there are less than 32 bytes left of the current fragment to transmit, there is room in the 
        // device's FIFO to fit another fragment
        if(tx_bytes < 32){

            // Switch to fixed packet mode if we're safely within the last 256 bytes and haven't already
            if(!fixed_packet_mode && bytes_remaining < 128){
                cc1101_spi_write_config_register(cc1101, PKTCTRL0, 0x00);
                fixed_packet_mode = 1;
            }

            // If there are less than 32 bytes left to transmit, set the number of bytes to transmit to the remaining length
            // Otherwise, transmit a full 32 bytes
            fragment_size = bytes_remaining < 32 ? bytes_remaining : 32;

            // Write the fragment to the device's TX FIFO and decrement the number of bytes left in the packet
            cc1101_spi_write_txfifo(cc1101, &buf[len-bytes_remaining], fragment_size);
            bytes_remaining-=fragment_size;
            CC1101_DEBUG(cc1101, "Transmitting %zu bytes, %zu remaining, %zu Total", fragment_size, bytes_remaining, len);
        }
    }

    // Wait until transmission has finished
    do {
        bytes_remaining = cc1101_spi_read_status_register(cc1101, TXBYTES).data;

        // Check for underflow, this shouldn't occur in fixed length mode.
        // Caller will change state to IDLE and flush the TXFIFO, so handle the error by exiting the loop
        if(bytes_remaining > FIFO_LEN){ 
            CC1101_DEBUG(cc1101, "TXFIFO Underflow");
            break;
        }

    } while(bytes_remaining > 0);
}

/*
* Function to transmit a packet that will fit completely within the CC1101's TX FIFO (<= 64 bytes)
*
* Arguments:
*   cc1101: device struct
*   buf: bytes to transmit
*   len: length of buf
*
*/
static void tx_single(cc1101_t* cc1101, const char* buf, size_t len)
{
    int bytes_remaining;

    // Write the packet to device's TX FIFO
    cc1101_spi_write_txfifo(cc1101, buf, len);

    // Write the length of the packet to the device's Packet Length register
    cc1101_spi_write_config_register(cc1101, PKTLEN, len);

    // Set the device's transmission mode to fixed length 
    cc1101_spi_write_config_register(cc1101, PKTCTRL0, 0x00);

    // Instruct the device to begin transmission
    change_state(cc1101, MODE_TX);
    
    CC1101_DEBUG(cc1101, "Transmitting %zu bytes, 0 remaining, %zu Total", len, len);

    // Wait until transmission has finished. Handle overflow condition
    do {
        bytes_remaining = cc1101_spi_read_status_register(cc1101, TXBYTES).data;

        // Check for underflow, this shouldn't occur in fixed length mode.
        // Handle the error by exiting the loop, as the device will be sent to idle afterwards anyway
        if(bytes_remaining > FIFO_LEN){ 
            CC1101_DEBUG(cc1101, "TXFIFO Underflow");
            break;
        }

    } while(bytes_remaining > 0) ;
}

/*
* Function to transmit a packet
*
* Arguments:
*   cc1101: device struct
*   buf: bytes to transmit
*   len: length of buf
*
*/
void cc1101_tx(cc1101_t* cc1101, const char* buf, size_t len){

    CC1101_DEBUG(cc1101, "Transmit Mode");
    // Put the device into idle mode
    change_state(cc1101, MODE_IDLE);

    // TX method based on whether the packet will fully fit in the device's FIFO
    if(len > FIFO_LEN){
        tx_multi(cc1101, buf, len);
    }
    else{
        tx_single(cc1101, buf, len);
    }

    // Set the device to idle
    cc1101_idle(cc1101);
    
    // Flush the TXFIFO
    cc1101_flush_tx_fifo(cc1101);
    
    // Return to RX mode if configured
    if(cc1101->rx_config.packet_length > 0){
        // Restore RX config
        cc1101_config_apply_rx(cc1101);
        cc1101_rx(cc1101);
    }
}
#endif 

/*
* Function to reset the device and the driver's internal state
*
* Arguments:
*   cc1101: device struct
*/
void cc1101_reset(cc1101_t* cc1101)
{
    CC1101_DEBUG(cc1101, "Reset");

    // Reset the device
    cc1101->mode = MODE_IDLE;
    cc1101_spi_send_command(cc1101, SRES);

    // Reset the current packet counter
    cc1101->bytes_remaining = 0;

    // Clear the RX FIFO
    kfifo_reset(&cc1101->rx_fifo);
}

// Longest to expect between interrupts. At 0.6 kBaud (75 bytes/sec), time to fill to FIFOTHR (32 bytes) should be ~425ms, so 1000ms here seems reasonable
// If an interrupt is missed, the RXFIFO will overflow and the device will not send any further interrupts, causing the device to lock indefinitely.
// This value is used to set a timer that ensures the interrupt handler will trigger and unlock the device if this occurs.
#define RX_TIMEOUT_MS 1000

/*
* Interrupt handler function called when the device raises GDO2
* The default receive configuration instructs it to raise GDO2 when the RX FIFO has received x bytes 
* 
* Arguments:
*   irq: IRQ number
*   handle: device struct
*/
irqreturn_t cc1101_rx_interrupt(int irq, void *handle)
{   
    cc1101_t* cc1101 = handle;

    size_t i;
    int fifo_available;
    unsigned char rx_bytes;
    
    CC1101_DEBUG(cc1101, "Interrupt");

    // Interrupt is only used when the driver is in receive mode
    if(cc1101->mode == MODE_RX){

        // Stop the RX timeout timer (if it's running) while processing the interrupt
        del_timer(&cc1101->rx_timeout);

        // Read the number of bytes in the device's RX FIFO
        rx_bytes = cc1101_spi_read_status_register(cc1101, RXBYTES).data;

        // If an overflow has occured part of the packet will have been missed, so reset and wait for the next packet
        if(rx_bytes > FIFO_LEN) {
            CC1101_ERROR(cc1101, "RXFIFO Overflow. If this error persists, decrease baud rate");
            
            // Flush the RXFIFO
            cc1101_flush_rx_fifo(cc1101);

            // Reset SYNC_MODE to the value from the config
            cc1101_spi_write_config_register(cc1101, MDMCFG2, cc1101_get_mdmcfg2(&cc1101->rx_config.common, &cc1101->rx_config));

            // Put the device back into receive mode ready to receive the next packet
            change_state(cc1101, MODE_RX);

            // Unlock mutex and reset packet count
            cc1101->bytes_remaining = 0;
            mutex_unlock(&cc1101->device_lock);

            return IRQ_HANDLED;
        }

        // If 0 bytes remaining, this is a new packet
        if(cc1101->bytes_remaining == 0){

            // Try to lock the device
            if(mutex_trylock(&cc1101->device_lock) != 1){
                // If this fails, it is because a process has /dev/cc1101.x.x open
                CC1101_DEBUG(cc1101, "Interrupt Handler Failed To Acquire Lock");

                // Drain the device's RX FIFO, otherwise it will overflow, causing RX to stop
                // This can be drained into the temp buffer as the next interrupt will start a new packet and overwrite it anyway
                cc1101_spi_read_rxfifo(cc1101, &cc1101->current_packet[0], rx_bytes - 1);

                // Return and wait for the next interrupt  
                return IRQ_HANDLED;
            }
            // If the lock is held, a packet can be received  
            CC1101_DEBUG(cc1101, "Receiving Packet");

            // Update the number of bytes to receive from the RX configuration
            cc1101->bytes_remaining = cc1101->rx_config.packet_length;

            // Set SYNC_MODE to "No Preamble/Sync" - this will cause RX to continue even if carrier-sense drops below the defined threshold
            // This prevents a situation where more bytes are expected for the current packet but another interrupt doesn't occur
            cc1101_spi_write_config_register(cc1101, MDMCFG2, cc1101_get_mdmcfg2(&cc1101->rx_config.common, &cc1101->rx_config) & 0xF8);

            // Start a timer for how long we should wait for another interrupt to arrive
            mod_timer(&cc1101->rx_timeout, jiffies + msecs_to_jiffies(RX_TIMEOUT_MS));
        }

        // Something went wrong and there aren't any bytes in the RX FIFO even though GDO2 went high
        if(rx_bytes == 0){
            // Reset the receive counter, so the next interrupt will be the start of a new packet 
            CC1101_ERROR(cc1101, "Receive Error, Waiting for Next Packet");
            cc1101->bytes_remaining = 0;

            // Reset SYNC_MODE to the value from the config
            cc1101_spi_write_config_register(cc1101, MDMCFG2, cc1101_get_mdmcfg2(&cc1101->rx_config.common, &cc1101->rx_config));

            // Release the device lock
            mutex_unlock(&cc1101->device_lock);
        }
        // Received some bytes, but there are still some remaining in the packet to be received
        else if(rx_bytes < cc1101->bytes_remaining){

            CC1101_DEBUG(cc1101, "Received %d Bytes, Read %d Bytes, %d Bytes Remaining", rx_bytes, rx_bytes - 1, cc1101->bytes_remaining - (rx_bytes - 1)); 
            
            // Read the received number of bytes from the device's RX FIFO into the temporary buffer
            cc1101_spi_read_rxfifo(cc1101, &cc1101->current_packet[cc1101->rx_config.packet_length - cc1101->bytes_remaining], rx_bytes - 1);

            // Decrement the number of bytes left to receive in the packet
            cc1101->bytes_remaining = cc1101->bytes_remaining - (rx_bytes - 1);

            //  Restart the timer for how long we should wait for another interrupt to arrive
            mod_timer(&cc1101->rx_timeout, jiffies + msecs_to_jiffies(RX_TIMEOUT_MS));

            //Return without releasing the lock. The device is in the middle of a receive and can't be reconfigured
        }
        // Received a number of bytes greater than or equal to the number left in the packet 
        else {
            del_timer(&cc1101->rx_timeout);

            CC1101_DEBUG(cc1101, "Received %d Bytes, Read %d Bytes, %d Bytes Remaining", rx_bytes, cc1101->bytes_remaining, 0); 

            // RX has finished and the required bytes are in the device's RX FIFO, so put the device in idle mode
            change_state(cc1101, MODE_IDLE);

            // Read the remaining bytes from the device's RX FIFO
            cc1101_spi_read_rxfifo(cc1101, &cc1101->current_packet[cc1101->rx_config.packet_length - cc1101->bytes_remaining], cc1101->bytes_remaining);

            // Get the amount of space left in the received packet buffer
            fifo_available = kfifo_avail(&cc1101->rx_fifo);

            // Remove oldest packet from the received packet buffer if there is less than is required to hold the newly received packet
            if(fifo_available < cc1101->rx_config.packet_length){
                CC1101_DEBUG(cc1101, "RX FIFO Full - Removing Packet");
                for(i = 0; i < cc1101->rx_config.packet_length; i++){
                    kfifo_skip(&cc1101->rx_fifo);
                }
            }

            // Add the new packet to the received packet buffer
            kfifo_in(&cc1101->rx_fifo, cc1101->current_packet, cc1101->rx_config.packet_length);

            CC1101_DEBUG(cc1101, "Packet Received");

            // Reset the number of bytes remaining
            cc1101->bytes_remaining = 0;

            // Flush the device's RX FIFO
            cc1101_flush_rx_fifo(cc1101);

            // Reset SYNC_MODE to the value from the config
            cc1101_spi_write_config_register(cc1101, MDMCFG2, cc1101_get_mdmcfg2(&cc1101->rx_config.common, &cc1101->rx_config));

            // Put the device back into receive mode ready to receive the next packet
            change_state(cc1101, MODE_RX);

            // Release the lock so the device can be reconfigured if necessary
            mutex_unlock(&cc1101->device_lock);    
        }
    }
    return IRQ_HANDLED;
}

/*
* Work function to recover from a missed interrupt in RX
*
* Arguments:
*   work: rx_timeout_work struct of the device that has missed an interrupt 
*/
void cc1101_rx_timeout_work(struct work_struct *work){
    // Get the device from the work_struct
    cc1101_t *cc1101;
    cc1101 = container_of(work, cc1101_t, rx_timeout_work);

    // Call the interrupt handler, which will detect the RXFIFO overflow state from the device and recover
    cc1101_rx_interrupt(cc1101->irq, cc1101);
}

/*
* Receive timer callback. Called when an interrupt is expected during RX, but never arrives
*
* This can occur at high baud rates if the interrupt handler has not finished execution when the next interrupt arrives
*
* RXFIFO will have overflowed by the time this is called.
*
* Arguments:
*   cc1101: device struct
*/
void cc1101_rx_timeout(struct timer_list *t){
    // Get the device the timeout has occured on
    cc1101_t *cc1101 = from_timer(cc1101, t, rx_timeout);
    CC1101_ERROR(cc1101, "RX Interrupt Missed");

    // Schedule the handler to be called in the process context to recover
    INIT_WORK(&cc1101->rx_timeout_work, cc1101_rx_timeout_work);
    schedule_work(&cc1101->rx_timeout_work);
}


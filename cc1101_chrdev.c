// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/
#include <linux/cdev.h>
#include <linux/uaccess.h>

#include "cc1101_internal.h"
#include "cc1101_chrdev.h"
#include "cc1101_radio.h"
#include "cc1101_spi.h"
#include "cc1101_config.h"

// IOCTL Definitions
#define CC1101_BASE 'c'
// Get the driver IOCTL version
#define CC1101_GET_VERSION _IOR(CC1101_BASE, 0, uint)
// Reset the device
#define CC1101_RESET _IO(CC1101_BASE, 1)
// Set TX configuration within driver
#define CC1101_SET_TX_CONF _IOW(CC1101_BASE, 2, struct cc1101_tx_config)
// Set RX configuration within driver
#define CC1101_SET_RX_CONF _IOW(CC1101_BASE, 3, struct cc1101_rx_config)
// Get TX configuration from driver
#define CC1101_GET_TX_CONF _IOR(CC1101_BASE, 4, struct cc1101_tx_config)
// Get TX configuration registers from driver
#define CC1101_GET_TX_RAW_CONF _IOR(CC1101_BASE, 5, cc1101_device_config_t)
// Get RX configuration from driver
#define CC1101_GET_RX_CONF _IOR(CC1101_BASE, 6, struct cc1101_rx_config)
// Get RX configuration registers from driver
#define CC1101_GET_RX_RAW_CONF _IOR(CC1101_BASE, 7, cc1101_device_config_t)
// Read configuration registers from hardware
#define CC1101_GET_DEV_RAW_CONF _IOR(CC1101_BASE, 8, cc1101_device_config_t)
// Get the current RSSI
#define CC1101_GET_RSSI _IOR(CC1101_BASE, 9, unsigned char)
// Get the configured maximum packet size
#define CC1101_GET_MAX_PACKET_SIZE _IOR(CC1101_BASE, 10, u32)

#define SPI_MAJOR_NUMBER 153
#define N_SPI_MINOR_NUMBERS 12

extern uint max_packet_size;
extern uint rx_fifo_size;

static struct class *dev_class;
cc1101_t* device_list[N_SPI_MINOR_NUMBERS] = {0};
static DEFINE_MUTEX(device_list_lock);

/*
* Handler for open events to /dev/cc1101.x.x
* Only one handle is allowed to transmit, receive or configure the device at a time
* Operations will block until /dev/cc1101.x.x is closed  
*/
static int chrdev_open(struct inode *inode, struct file *file)
{
    cc1101_t* cc1101 = NULL;
    int device_index;
    
    // Search the device list for the cc1101 struct relating to the chardev
    if(mutex_lock_interruptible(&device_list_lock) != 0) {
        return -EBUSY;
    }

    for(device_index = 0; device_index < N_SPI_MINOR_NUMBERS; device_index++){
        if (device_list[device_index] != NULL){
            if (inode->i_rdev == device_list[device_index]->devt) {
                cc1101 = device_list[device_index];
            }
        }
    }
    
	mutex_unlock(&device_list_lock);

    // This should never occur - a chardev shouldn't be created without a CC1101 being present
    if(cc1101 == NULL){
        return -ENODEV;
    }

    // Once found, lock the device and save the pointer to private data for the subsequent functions
    if(mutex_lock_interruptible(&cc1101->chrdev_lock) != 0) {
        return -EBUSY;
    };

    file->private_data = cc1101;
    return 0;
}

/*
* Handler for close events to /dev/cc1101.x.x
* Releases the device lock obtained at open
*/
static int chrdev_release(struct inode *inode, struct file *file)
{
    cc1101_t* cc1101 = file->private_data;
    mutex_unlock(&cc1101->chrdev_lock);
    file->private_data = NULL;
    return 0;
}

/*
* Handler for iotcl commands to /dev/cc1101.x.x
* IOCTLs can be used to set and get the TX and RX config, reset the device
* and retrieve the contents of the CC1101's registers 
*/
static long chrdev_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    cc1101_t* cc1101 = file->private_data;
    spi_transaction_t rssi;
    int version = DRIVER_VERSION;
    int ret = 0;

    // Temporary holding variables for new TX and RX configs
    cc1101_device_config_t device_config;
#ifndef RXONLY
    struct cc1101_tx_config tx_config;
#endif
    struct cc1101_rx_config rx_config;

    if(_IOC_TYPE(cmd) != CC1101_BASE){
        CC1101_ERROR(cc1101, "Invalid IOCTL\n");
        return -EIO;
    }

    // Lock the device for reconfiguration
    if(mutex_lock_interruptible(&cc1101->device_lock) != 0) {
        return -EBUSY;
    };

    switch(cmd){

        // Get the userspace API version
        case CC1101_GET_VERSION:
            ret = copy_to_user((unsigned char*) arg, &version, sizeof(version));
            break;

        // Reset the device and driver state
        case CC1101_RESET:
            CC1101_INFO(cc1101, "Reset");
            cc1101_reset(cc1101);
            ret = 0;
            break;

        // Set the RX config for the device
        case CC1101_SET_RX_CONF:

            // Copy the config provided in userspace to the kernel
            if(copy_from_user(&rx_config, (unsigned char*) arg, sizeof(rx_config)) != 0){
                CC1101_ERROR(cc1101, "Error Copying Device RX Config\n");
                ret = -EFAULT;
                goto done;
            }

            // Validate the provided config
            if(!cc1101_config_validate_rx(cc1101, &rx_config)){
                ret = -EINVAL;
                goto done;
            }

            // Replace the RX FIFO with a new one based on the provided packet size and the maximum number of queued packets
            kfifo_free(&cc1101->rx_fifo);
            if(kfifo_alloc(&cc1101->rx_fifo, rx_config.packet_length * rx_fifo_size, GFP_KERNEL) != 0) {
                CC1101_ERROR(cc1101, "Failed to allocate packet FIFO memory");
                ret = -ENOMEM;
                goto done;
            }

            // Store the new RX config in the device struct
            memcpy(&cc1101->rx_config, &rx_config, sizeof(struct cc1101_rx_config));

            // Set the device to idle before reconfiguring
            cc1101_idle(cc1101);

            // Write the configuration to the device
            cc1101_config_apply_rx(cc1101);
            
            // Enter RX mode on the device
            cc1101_rx(cc1101);

            ret = 0;
            break;

        // Returns the RX config configured in the driver to userspace
        case CC1101_GET_RX_CONF:
            ret = copy_to_user((unsigned char*) arg, &cc1101->rx_config, sizeof(struct cc1101_rx_config));
            break;

        // Returns the register values for the RX configuration to userspace
        case CC1101_GET_RX_RAW_CONF:
            cc1101_config_rx_to_registers(device_config, &cc1101->rx_config);
            ret = copy_to_user((unsigned char*) arg, device_config, sizeof(device_config));
            break;

        // Set the TX config to use for the next packet written to /dev/cc1101.x.x
        case CC1101_SET_TX_CONF:
#ifndef RXONLY
            // Copy the config provided in userspace to the kernel
            if(copy_from_user(&tx_config, (unsigned char*) arg, sizeof(tx_config)) != 0){
                CC1101_ERROR(cc1101, "Error Copying Device TX Config\n");
                ret = -EFAULT;
                goto done;
            }

            // Validate the provided config
            if(!cc1101_config_validate_tx(cc1101, &tx_config)){
                ret = -EINVAL;
                goto done;
            }

            // Store the new TX config in the device struct
            memcpy(&cc1101->tx_config, &tx_config, sizeof(struct cc1101_tx_config));

            ret = 0;
#else
            ret = -EPERM;
#endif 
            break;

        // Returns the TX config configured in the driver to userspace
        case CC1101_GET_TX_CONF:
#ifndef RXONLY
            ret = copy_to_user((unsigned char*) arg, &cc1101->tx_config, sizeof(struct cc1101_tx_config));
#else
            ret = -EPERM;
#endif
            break;

        // Returns the register values for the TX configuration to userspace
        case CC1101_GET_TX_RAW_CONF:
#ifndef RXONLY
            cc1101_config_tx_to_registers(device_config, &cc1101->tx_config);
            ret = copy_to_user((unsigned char*) arg, device_config, sizeof(device_config));
#else
            ret = -EPERM;
#endif
            break;

        // Reads the current state of the CC1101's registers and returns them to userspace
        case CC1101_GET_DEV_RAW_CONF:
            cc1101_spi_read_config_registers(cc1101, device_config, sizeof(device_config));
            ret = copy_to_user((unsigned char*) arg, device_config, sizeof(device_config));
            break;

        case CC1101_GET_RSSI:
            rssi = cc1101_spi_read_status_register_once(cc1101, RSSI);
            ret = copy_to_user((unsigned char*) arg, &rssi.data, sizeof(rssi.data));
            break;

        case CC1101_GET_MAX_PACKET_SIZE:
            ret = copy_to_user((unsigned char*) arg, &max_packet_size, sizeof(max_packet_size));
            break;

        default:
            CC1101_ERROR(cc1101, "Unknown Command %d, %zu, %zu\n", cmd, sizeof(struct cc1101_rx_config), sizeof(struct cc1101_tx_config));
            ret = -EIO;
            break;
    }

done:
    mutex_unlock(&cc1101->device_lock);
    return ret;
}

/*
* Handler for read events to /dev/cc1101.x.x
* A read request will return one packet from the receive buffer, if present
*/
static ssize_t chrdev_read(struct file *file, char __user *buf, size_t len, loff_t *off)
{
    cc1101_t* cc1101 = file->private_data;
    ssize_t ret;
    unsigned int out_bytes;

    // Check a RX config has been set and that the out buffer is the correct size
    if (cc1101->rx_config.packet_length == 0 || len != cc1101->rx_config.packet_length) {
        return -EMSGSIZE;
    }

    if(mutex_lock_interruptible(&cc1101->device_lock) != 0) {
        return -EBUSY;
    };

    // Check there is at least one packet in the RX FIFO
    if (kfifo_len(&cc1101->rx_fifo) < cc1101->rx_config.packet_length) {
        ret = -ENOMSG;
        goto done;
    }

    // Copy the packet out to userspace
    if (kfifo_to_user(&cc1101->rx_fifo, buf, len, &out_bytes) != 0) {
        ret = -EFAULT;
        goto done;
    }

    // Check the number of bytes copied out matches the expected number
    if (out_bytes == cc1101->rx_config.packet_length) {
        ret = out_bytes;
    }
    else {
        ret = -EFAULT;
    }

done:
    mutex_unlock(&cc1101->device_lock);
    return ret;
}

/*
* Handler for write events to /dev/cc1101.x.x
* Written bytes are transmitted by the CC1101 according the TX config 
*/
static ssize_t chrdev_write(struct file *file, const char __user *buf, size_t len, loff_t *off)
{
#ifndef RXONLY
    cc1101_t* cc1101 = file->private_data;
    ssize_t ret;
    unsigned char *tx_bytes;
 
 
    // Check the number of bytes to be transmitted are allowed
    if (len > max_packet_size) {
        ret = -EMSGSIZE;
        goto done;
    }

    // Allocate a temporary buffer for the bytes to be transmitted
    tx_bytes = kmalloc(len, GFP_KERNEL);
    if(tx_bytes == NULL) {
        ret = -ENOMEM;
        goto done;
    }

    // Copy from userspace to temporary buffer in kernel space
    if(copy_from_user(tx_bytes, buf, len) != 0) {
        ret = -EFAULT;
        goto err_copy;
    }

    // Lock the device for reconfiguration
    if(mutex_lock_interruptible(&cc1101->device_lock) != 0) {
        ret = -EBUSY;
        goto err_lock;
    };

    // Set the device to idle before configuring
    cc1101_idle(cc1101);

    // Apply the TX config
    cc1101_config_apply_tx(cc1101);

    // Transmit bytes using the device and return the number of transmitted bytes
    cc1101_tx(cc1101, tx_bytes, len);
    ret = len;

    mutex_unlock(&cc1101->device_lock);
err_lock:
err_copy:
    kfree(tx_bytes);
done:
    return ret;
#else
    return -EPERM;
#endif 
}

/*
*   Add a character device for a cc1101
*/
int cc1101_chrdev_add_device(cc1101_t * cc1101) {
    int ret;
    int device_index;
 
    mutex_lock(&device_list_lock);

    // Search for a free minor number
    for(device_index = 0; device_index < N_SPI_MINOR_NUMBERS; device_index++){
        if(device_list[device_index] == NULL){
            // Allocate the minor number
            cc1101->devt = MKDEV(SPI_MAJOR_NUMBER, device_index);

            // Create a /dev/cc1101.x.x character device
            if(IS_ERR(device_create(dev_class, &cc1101->spi->dev, cc1101->devt, cc1101, "cc1101.%d.%d", cc1101->spi->master->bus_num, cc1101->spi->chip_select))) {
                ret = -ENODEV;
                goto done;
            }

            // Insert the device in the device list
            device_list[device_index] = cc1101;
            ret = 0;
            goto done;
        }
    }
    ret = -ENODEV;

done:
    mutex_unlock(&device_list_lock);
    return ret;
}

/*
*   Remove a cc1101 character device
*/
void cc1101_chrdev_remove_device(cc1101_t * cc1101) {
    mutex_lock(&device_list_lock);

    // Destroy the character device and remove the entry from the device list    
    device_destroy(dev_class, cc1101->devt);	
    device_list[MINOR(cc1101->devt)] = NULL;

    mutex_unlock(&device_list_lock);
}

// Chardev operation functions
static struct file_operations fops =
{
    .owner = THIS_MODULE,
    .read = chrdev_read,
    .write = chrdev_write,
    .unlocked_ioctl = chrdev_ioctl,
    .open = chrdev_open,
    .release = chrdev_release,
};

/*
*   Setup the CC1101 character device class
*/
int cc1101_chrdev_setup(struct spi_driver* cc1101_driver)
{
	int ret;

	ret = register_chrdev(SPI_MAJOR_NUMBER, "spi", &fops);
	if (ret < 0) {
        goto err_register;
    }

	dev_class = class_create(THIS_MODULE, "cc1101");
	if (IS_ERR(dev_class)) {
		ret = PTR_ERR(dev_class);
        goto err_class_create;
	}

	ret = spi_register_driver(cc1101_driver);
	if (ret < 0) {
        goto err_register_driver;
	}

	goto done;

err_register_driver:
    class_destroy(dev_class);
err_class_create:
    unregister_chrdev(SPI_MAJOR_NUMBER, cc1101_driver->driver.name);
err_register:
done:
    return ret;
}

/*
*   Remove the CC1101 character device class
*/
void cc1101_chrdev_teardown(struct spi_driver* cc1101_driver)
{
    spi_unregister_driver(cc1101_driver);
	class_destroy(dev_class);
	unregister_chrdev(SPI_MAJOR_NUMBER, cc1101_driver->driver.name);
}
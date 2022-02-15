// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of_gpio.h>
#include <linux/input.h>
#include <generated/autoconf.h>

#include "cc1101_internal.h"
#include "cc1101_chrdev.h"
#include "cc1101_spi.h"
#include "cc1101_radio.h"

// Module parameter for the largest packet that can be transmitted/received
uint max_packet_size = 1024;
module_param(max_packet_size, uint, 0660);

// Module parameter for the number of received packets to be held in the kernel FIFO waiting to be read
uint rx_fifo_size = 10;
module_param(rx_fifo_size, uint, 0660);

#ifndef CONFIG_DYNAMIC_DEBUG
uint debug = 0;
module_param(debug, uint, 0660);
#endif

#ifdef RXONLY
    #define EXTRA_STATUS "[RX Only] -"
#else
    #define EXTRA_STATUS "-"
#endif

/*
*   Function called on module insertion for each CC1101 entry in device tree
*/
static int cc1101_spi_probe(struct spi_device *spi)
{
    cc1101_t *cc1101;
    struct gpio_desc *gpio;
    spi_transaction_t partnum, version;

    // Memory allocations - use devm_* functions so they are automatically freed
    // Allocate a new CC1101 struct
    cc1101 = devm_kzalloc(&spi->dev, sizeof(*cc1101), GFP_KERNEL);
    if (cc1101 == NULL)
    {
        dev_err(&spi->dev, "Failed to allocate memory");
        return -ENOMEM;
    }

    // Allocate a buffer to hold partial packets being received
    cc1101->current_packet = devm_kzalloc(&spi->dev, max_packet_size, GFP_KERNEL);
    if (cc1101->current_packet == NULL)
    {
        dev_err(&spi->dev, "Failed to allocate packet buffer memory");
        return -ENOMEM;
    }

    // Associate the SPI device to the CC1101
    cc1101->spi = spi;

    // Initialise the device locks
    mutex_init(&cc1101->device_lock);
    mutex_init(&cc1101->chrdev_lock);

    // Setup the RX timeout timer
    timer_setup(&cc1101->rx_timeout, cc1101_rx_timeout, 0);

    // Read the CC1101 part and version numbers from the device registers
    partnum = cc1101_spi_read_status_register(cc1101, PARTNUM);
    version = cc1101_spi_read_status_register(cc1101, VERSION);

    // Check the device is a CC1101
    if (partnum.data != 0x00 || (version.data != 0x04 && version.data != 0x14))
    {
        dev_info(&spi->dev, "Device not found (Partnum: 0x%02x, Version: 0x%02x)", partnum.data, version.data);
        return -ENODEV;
    }

    // Reset the device, which will place it in idle and load the default config
    cc1101_reset(cc1101);
    
    // Get the GPIO associated with the device in device tree
    gpio = devm_gpiod_get_index(&spi->dev, "int", 0, GPIOD_IN);
    if (IS_ERR(gpio))
    {
        dev_err(&spi->dev, "Failed to get GPIO");
        return -ENODEV;
    }

    // Get an IRQ for the GPIO
    cc1101->irq = gpiod_to_irq(gpio);

    // Attach the interrupt handler to the GPIO
    if(devm_request_threaded_irq(&spi->dev, cc1101->irq, NULL, cc1101_rx_interrupt, IRQF_TRIGGER_RISING | IRQF_ONESHOT, dev_name(&spi->dev), cc1101) != 0){
        dev_err(&spi->dev, "Failed to setup interrupt handler");
        return -ENODEV;
    }

    // Add a /dev/cc1101.x.x character device for the device
    if(cc1101_chrdev_add_device(cc1101) != 0){
        dev_err(&spi->dev, "Failed to create character device");
        return -ENODEV;
    }

    // Associate the device struct to the parent SPI device
    spi_set_drvdata(spi, cc1101);

    CC1101_INFO(cc1101, "Ready " EXTRA_STATUS " (Partnum: 0x%02x, Version: 0x%02x)", partnum.data, version.data);

    return 0;
}

/*
*   Function called on module removal for each CC1101 entry in device tree
*/
static int cc1101_spi_remove(struct spi_device *spi)
{
    cc1101_t* cc1101;
    cc1101 = spi_get_drvdata(spi);

    // Remove the RX timeout timer
    del_timer(&cc1101->rx_timeout);

    // Reset the hardware, placing it in idle mode
    cc1101_reset(cc1101);

    // Remove /dev/cc1101.x.x
    cc1101_chrdev_remove_device(cc1101);
    CC1101_INFO(cc1101, "Removed");
    return 0;
}

/*
*   Register the module to handle cc1101 entries in device tree
*/
static const struct of_device_id cc1101_dt_ids[] = {
    {.compatible = "ti,cc1101"},
    {},
};
MODULE_DEVICE_TABLE(of, cc1101_dt_ids);

static const struct spi_device_id cc1101_id[] = {
    {
        .name = "cc1101",
    },
    {}
};
MODULE_DEVICE_TABLE(spi, cc1101_id);

static struct spi_driver cc1101_driver = {
    .driver = {
        .name = "cc1101",
        .owner = THIS_MODULE,
        .of_match_table = cc1101_dt_ids,
    },
    .probe = cc1101_spi_probe,
    .remove = cc1101_spi_remove,
    .id_table = cc1101_id
};

/* 
*   Functions executed on module insertion/removal
*   Setup/remove the CC1101 character device class
*/
static int __init cc1101_init(void)
{
    #ifndef CONFIG_DYNAMIC_DEBUG
    if(debug > 1) {
        return -EINVAL;
    }
    #endif

    return cc1101_chrdev_setup(&cc1101_driver);
}
module_init(cc1101_init);

static void __exit cc1101_exit(void)
{    
    cc1101_chrdev_teardown(&cc1101_driver);
}
module_exit(cc1101_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("");
MODULE_DESCRIPTION("TI CC1101 Device Driver");
MODULE_VERSION("1.3.2");
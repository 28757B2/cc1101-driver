// SPDX-License-Identifier: GPL-2.0-only
/*
* Copyright (c) 2021
*/
#ifndef CC1101_CHRDEV_H
#define CC1101_CHRDEV_H

#include <linux/spi/spi.h>

int cc1101_chrdev_setup(struct spi_driver *cc1101_driver);
void cc1101_chrdev_teardown(struct spi_driver *cc1101_driver);
int cc1101_chrdev_add_device(cc1101_t *cc1101);
void cc1101_chrdev_remove_device(cc1101_t *cc1101);

#endif
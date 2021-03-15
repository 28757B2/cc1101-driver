# CC1101 Linux Device Driver

This project implements a Linux device driver for the Texas Instruments CC1101 radio. It has been tested with the Raspberry Pi 3B+.

The CC1101 is a general purpose packet radio that operates in the Sub-GHz Industrial, Scientific and Medical (ISM) bands (315/433/868/915 MHz).

This driver allows the use of inexpensive CC1101 Serial Peripheral Interface (SPI) modules, which can be directly interfaced to the Pi's GPIOs. A kernel module allows the CC1101 to operate using hardware interrupts instead of polling, which increases the accuracy of packet reception. 

A [Python module](https://github.com/28757B2/cc1101-python) also exists to configure the driver and receive and transmit packets.

Use of this software is at your own risk. You are responsible for complying with local laws and regulations.

## Supported Features
The driver supports a subset of the CC1101 hardware's features and provides a high-level interface to the device that does not require setting of the individual hardware registers. 

* Frequencies - 300-348/387-464/778-928 MHz
* Modulation - OOK/2FSK/4FSK/GFSK/MSK
* Data Rate - 0.6 - 500 kBaud 
* RX Bandwidth - 58 - 812 kHz
* Arbitrary packet length RX/TX
* Sync word or carrier sense triggered RX
* 16/32 bit configurable sync word

## Physical Connection (Raspberry Pi)
The following connections should be made between the CC1101 module and the Raspberry Pi's GPIOs:

| CC1101 Pin | Raspberry Pi GPIO |
|------------|-------------------|
| VCC        | 17                |
| SI         | 19                |
| SO         | 21                |
| SCK        | 23                |
| CSN        | 24                |
| GDO2       | 22                |
| GND        | 25                |

A second radio can be connected to SPI bus 0 using the second chip enable pin: 

| CC1101 Pin | Raspberry Pi GPIO |
|------------|-------------------|
| CSN        | 26                |
| GDO2       | 18                |

Note: SPI Bus 1 on the Raspberry Pi 3 and earlier does not support the SPI mode required for the CC1101.

## Quick Start Setup (Raspberry Pi OS)

```bash
# Install dependencies
sudo apt install raspberrypi-kernel-headers dkms git

# Clone repository
sudo mkdir /usr/src/cc1101-1.0.0
sudo chown -R pi:pi /usr/src/cc1101-1.0.0
cd /usr/src/cc1101-1.0.0
git clone https://github.com/28757B2/cc1101-driver.git .

# Only permit root to alter module source
sudo chown -R root:root /usr/src/cc1101-1.0.0

# Build with DKMS
sudo dkms add -m cc1101 -v 1.0.0
sudo dkms build -m cc1101 -v 1.0.0
sudo dkms install -m cc1101 -v 1.0.0

# Enable SPI
sudo sed -i "s/^#dtparam=spi=on$/dtparam=spi=on/" /boot/config.txt

# Compile Device Tree overlay
sudo dtc -@ -I dts -O dtb -o /boot/overlays/cc1101.dtbo cc1101.dts

# Enable Device Tree overlay
echo "dtoverlay=cc1101" | sudo tee -a /boot/config.txt

# Enable module loading at boot
echo "cc1101" | sudo tee -a /etc/modules

# Assign /dev/cc1101.x.x devices to pi user/group
echo 'SUBSYSTEM=="cc1101", OWNER="pi", GROUP="pi", MODE="0660"' | sudo tee -a /etc/udev/rules.d/50-cc1101.rules

# Reboot to apply config
reboot
```

After rebooting, entries should appear in the `dmesg` output depending on where the CC1101 is attached:

    [  726.808248] cc1101 spi0.1: Device not found (Partnum: 0x00, Version: 0x00)
    [  726.809093] cc1101 spi0.0: Ready (Partnum: 0x00, Version: 0x14)

## Setup

### Dependencies

    apt install raspberrypi-kernel-headers dkms

### SPI Configuration
SPI must be enabled in `raspi-config`.

### Device Tree
A device tree overlay is required to tell the driver where to find the connected CC1101's. A device tree file is provided that configures two CC1101's, one on each chip select of SPI bus 0. The driver will automatically detect if a device is present on one these addresses and add the corresponding `/dev/cc1101.x.x` device.

This overlay also disables the `spidev` userspace SPI driver, which is configured by default on Raspberry Pi OS.

To apply the device tree overlay, compile the dts file and move the compiled file to the boot overlays folder:

    dtc -@ -I dts -O dtb -o cc1101.dtbo cc1101.dts
    # Raspberry Pi OS
    mv cc1101.dtbo /boot/overlays/
    # Ubuntu
    mv cc1101.dtbo /boot/firmware/overlays

The following should then be added to /boot/config.txt (Raspberry Pi OS) or /boot/firmware/syscfg.txt (Ubuntu):

    dtoverlay=cc1101

### Compiling and Loading

    cd cc1101
    make
    insmod cc1101.ko

### Options
Two settings are configurable through module parameters. `max_packet_size` sets the maximum size (in bytes) of a packet that can be transmitted or received (default 1024). `rx_fifo_size` controls how many packets are buffered within the driver before being read out to userspace (default 10). If the number of buffered packets received exceeds this value, the oldest packet will be removed from the buffer.

These settings can be altered when the module is loaded:

    insmod cc1101.ko max_packet_size=2048 rx_fifo_size=20

### Loading Module At Boot
Add the following to `/etc/modules`

    cc1101

### udev Rule
By default, the `/dev/cc1101.x.x` devices will belong to `root:root`. The following udev rule in `/etc/udev/rules.d/50-cc1101.rules` will assign the devices to the `pi` user:

    SUBSYSTEM=="cc1101", OWNER="pi", GROUP="pi", MODE="0660"

### DKMS
DKMS can be used to ensure the module is re-compiled whenever a new kernel is installed. Checkout the project to /usr/src/cc1101-1.0.0/ and run the following DKMS commands from the folder:

    dkms add -m cc1101 -v 1.0.0
    dkms build -m cc1101 -v 1.0.0
    dkms install -m cc1101 -v 1.0.0


### Debugging
To enable debug messages:

    echo "module cc1101 +p" > /sys/kernel/debug/dynamic_debug/control

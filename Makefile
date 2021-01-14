obj-m := cc1101.o
cc1101-objs+= cc1101_main.o cc1101_chrdev.o cc1101_spi.o cc1101_radio.o cc1101_config.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
# SPDX-License-Identifier: GPL-2.0
#
#  Makefile for the Linux Controller Area Network drivers.
#
CFLAGS := -O2 -Wall -Wno-parentheses
CPPFLAGS +=	-D_GNU_SOURCE

obj-m+=hlcan.o

all:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
install:
	cp hlcan.ko /usr/lib/modules/$(shell uname -r)/kernel/drivers/net/can
remove:
	rm -rf /usr/lib/modules/$(shell uname -r)/kernel/drivers/net/can/hlcan.ko

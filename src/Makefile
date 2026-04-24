# SPDX-License-Identifier: GPL-2.0
#
# Makefile for ch341_can USB-CAN kernel module (out-of-tree build)
#

KVERSION ?= $(shell uname -r)
KDIR     ?= /lib/modules/$(KVERSION)/build
INSTALL_DIR ?= /lib/modules/$(KVERSION)/extra

obj-m += ch341_can.o

all:
	$(MAKE) -C $(KDIR) M=$(CURDIR) modules

clean:
	$(MAKE) -C $(KDIR) M=$(CURDIR) clean

install: all
	install -d $(INSTALL_DIR)
	install -m 644 ch341_can.ko $(INSTALL_DIR)/
	depmod -a $(KVERSION)

uninstall:
	rm -f $(INSTALL_DIR)/ch341_can.ko
	depmod -a $(KVERSION)

# Convenience: load the module after building
load: all
	-rmmod ch341_can 2>/dev/null || true
	insmod ch341_can.ko

unload:
	rmmod ch341_can

.PHONY: all clean install uninstall load unload

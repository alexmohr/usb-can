# CH341 USB-CAN Linux Kernel Module

Native SocketCAN driver for cheap USB-CAN analyzers based on the QinHeng CH341 (HL-340) USB-to-serial bridge.

![alt text](USB-CAN.jpg)

These adapters are widely available on eBay/AliExpress and show up as:
```
Bus 002 Device 006: ID 1a86:7523 QinHeng Electronics HL-340 USB-Serial adapter
```

## Architecture

This module talks **directly** to the USB device — no userspace daemon, no
TTY line discipline, and no dependency on the `ch341-uart` or `slcan` kernel
modules. It registers a standard SocketCAN network interface that works with
`ip link`, `candump`, `cansend`, and all other SocketCAN tools.

## Requirements

- Linux kernel headers (>= 6.0 recommended)
- `can-utils` (optional, for `candump`/`cansend`)

```bash
# Debian/Ubuntu
sudo apt install linux-headers-$(uname -r) can-utils
```

## Building

```bash
cd src
make
```

### Module signing (Secure Boot)

```bash
kmodsign sha512 /var/lib/shim-signed/mok/MOK.priv \
                 /var/lib/shim-signed/mok/MOK.der \
                 src/ch341_can.ko
```

## Installation

```bash
cd src
sudo make install     # installs + depmod
```

To remove:
```bash
cd src
sudo make uninstall
```

## Usage

**Important:** Unbind the `ch341` serial driver first if it has already
claimed the device:

```bash
# Check if ch341 grabbed the device
lsusb -t | grep ch341

# If so, either blacklist it or unbind:
echo '1a86 7523' | sudo tee /sys/bus/usb/drivers/ch341/unbind
# — or add to /etc/modprobe.d/blacklist.conf:
#   blacklist ch341
```

### Load the module

```bash
sudo modprobe can-dev          # CAN subsystem dependency
sudo insmod src/ch341_can.ko
# or, after 'make install':
sudo modprobe ch341_can
```

### Configure and bring up the interface

```bash
# Set bitrate (required before link-up)
sudo ip link set can0 type can bitrate 500000

# Optional: listen-only mode
sudo ip link set can0 type can listen-only on

# Optional: loopback mode
sudo ip link set can0 type can loopback on

# Bring the interface up
sudo ip link set can0 up
```

### Supported bitrates

| Bitrate   | Code |
|-----------|------|
| 1000000   | 0x01 |
| 800000    | 0x02 |
| 500000    | 0x03 |
| 400000    | 0x04 |
| 250000    | 0x05 |
| 200000    | 0x06 |
| 125000    | 0x07 |
| 100000    | 0x08 |
| 50000     | 0x09 |
| 20000     | 0x0a |
| 10000     | 0x0b |
| 5000      | 0x0c |

### Send / receive CAN frames

```bash
# Monitor all traffic
candump can0

# Send a standard frame
cansend can0 123#DEADBEEF

# Send an extended frame
cansend can0 1ABCDEF0#0102030405060708
```

### Tear down

```bash
sudo ip link set can0 down
sudo rmmod ch341_can
```

## Module parameters

None currently — all configuration is done via standard SocketCAN `ip link`
commands.

## License

GPL-2.0-only

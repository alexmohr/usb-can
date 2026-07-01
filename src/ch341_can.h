/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * ch341_can.h - USB-CAN driver for CH341-based adapters
 *
 * Copyright (c) 2020-2026 Alexander Mohr <usb-can@mohr.io>
 */

#ifndef __CH341_CAN_H__
#define __CH341_CAN_H__

#include <linux/types.h>

/* USB device identifiers */
#define CH341_CAN_VENDOR_ID	0x1a86
#define CH341_CAN_PRODUCT_ID	0x7523

/* CH341 serial configuration requests */
#define CH341_REQ_READ_VERSION	0x5F
#define CH341_REQ_WRITE_REG	0x9A
#define CH341_REQ_READ_REG	0x95
#define CH341_REQ_SERIAL_INIT	0xA1
#define CH341_REQ_MODEM_CTRL	0xA4

/* CH341 register addresses (from kernel ch341.c) */
#define CH341_REG_BREAK		0x05
#define CH341_REG_PRESCALER	0x12
#define CH341_REG_DIVISOR	0x13
#define CH341_REG_LCR		0x18
#define CH341_REG_LCR2		0x25
#define CH341_REG_FLOW_CTL	0x27

#define CH341_NBREAK_BITS	0x01

/* Modem control bits */
#define CH341_BIT_RTS		(1 << 6)
#define CH341_BIT_DTR		(1 << 5)

/* LCR bits */
#define CH341_LCR_ENABLE_RX	0x80
#define CH341_LCR_ENABLE_TX	0x40
#define CH341_LCR_MARK_SPACE	0x20
#define CH341_LCR_PAR_EVEN	0x10
#define CH341_LCR_ENABLE_PAR	0x08
#define CH341_LCR_STOP_BITS_2	0x04
#define CH341_LCR_CS8		0x03
#define CH341_LCR_CS7		0x02
#define CH341_LCR_CS6		0x01
#define CH341_LCR_CS5		0x00

/* Flow control */
#define CH341_FLOW_CTL_NONE	0x0000
#define CH341_FLOW_CTL_RTSCTS	0x0001

/* Quirk flags */
#define CH341_QUIRK_LIMITED_PRESCALER	BIT(0)
#define CH341_QUIRK_SIMULATE_BREAK	BIT(1)

/* CAN adapter protocol constants */
#define CAN_PACKET_START	0xAA
#define CAN_PACKET_END		0x55
#define CAN_FRAME_PREFIX	0xC0
#define CAN_FLAG_RTR		0x10
#define CAN_FLAG_ID_EXT		0x20

#define CAN_CFG_PACKET_TYPE	0x55
#define CAN_CFG_PACKET_LEN	20
#define CAN_CFG_CRC_IDX		2

#define CAN_MAX_PAYLOAD		8

/* CAN adapter speed codes */
enum ch341_can_speed {
	CH341_CAN_SPEED_1000000	= 0x01,
	CH341_CAN_SPEED_800000	= 0x02,
	CH341_CAN_SPEED_500000	= 0x03,
	CH341_CAN_SPEED_400000	= 0x04,
	CH341_CAN_SPEED_250000	= 0x05,
	CH341_CAN_SPEED_200000	= 0x06,
	CH341_CAN_SPEED_125000	= 0x07,
	CH341_CAN_SPEED_100000	= 0x08,
	CH341_CAN_SPEED_50000	= 0x09,
	CH341_CAN_SPEED_20000	= 0x0a,
	CH341_CAN_SPEED_10000	= 0x0b,
	CH341_CAN_SPEED_5000	= 0x0c,
	CH341_CAN_SPEED_INVALID	= 0xff,
};

/* CAN adapter mode codes */
enum ch341_can_mode {
	CH341_CAN_MODE_NORMAL		= 0x00,
	CH341_CAN_MODE_LOOPBACK		= 0x01,
	CH341_CAN_MODE_SILENT		= 0x02,
	CH341_CAN_MODE_LOOPBACK_SILENT	= 0x03,
};

/* CAN adapter frame type codes */
enum ch341_can_frame_type {
	CH341_CAN_FRAME_STANDARD	= 0x01,
	CH341_CAN_FRAME_EXTENDED	= 0x02,
};

/* Per-device private data */
struct ch341_can_priv {
	struct can_priv can;		/* must be first for netdev_priv() */

	struct usb_device *udev;
	struct net_device *netdev;

	/* TX — a single frame is in flight at a time */
	struct urb *tx_urb;
	u8 *tx_buf;
	dma_addr_t tx_buf_dma;

	/* RX */
	struct urb *rx_urb;
	u8 *rx_buf;
	dma_addr_t rx_buf_dma;

	/* RX reassembly */
	u8 rx_assembly[64];
	int rx_count;
	int rx_expected;
	spinlock_t rx_lock;		/* protect RX state machine */

	/* Endpoints */
	u8 bulk_in_ep;
	u8 bulk_out_ep;
	size_t bulk_in_size;

	/* State */
	struct mutex cmd_lock;		/* serialise config commands */

	enum ch341_can_speed speed_code;	/* validated in do_set_bittiming */
	enum ch341_can_mode device_mode;
	enum ch341_can_frame_type frame_type;

	u8 version;			/* CH341 chip version */
	unsigned long quirks;		/* device quirks */
};

#endif /* __CH341_CAN_H__ */

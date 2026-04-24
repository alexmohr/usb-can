// SPDX-License-Identifier: GPL-2.0-only
/*
 * ch341_can.c - USB-CAN driver for CH341-based CAN adapters
 *
 * This driver provides native SocketCAN support for cheap USB-CAN analyzers
 * based on QinHeng CH341 (HL-340) USB-to-serial chips. Instead of relying on
 * the ch341-uart serial driver + a userspace daemon + a TTY line discipline,
 * this module talks directly to the USB device and registers a SocketCAN
 * network interface.
 *
 * Copyright (c) 2020-2026 Alexander Mohr <usb-can@mohr.io>
 *
 * Based on earlier work:
 *   - usb-can userspace tool by Torbjorn Tyridal / kobolt
 *   - Linux kernel drivers: gs_usb, usb_8dev, peak_usb (for reference)
 */

#include <linux/module.h>
#include <linux/usb.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/error.h>
#include <linux/can/skb.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include "ch341_can.h"

#define DRV_NAME	"ch341_can"
#define RX_BUF_SIZE	64
#define TX_BUF_SIZE	64

MODULE_DESCRIPTION("SocketCAN driver for CH341-based USB-CAN adapters");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Mohr <usb-can@mohr.io>");

/* --------------------------------------------------------------------------
 * Supported CAN bitrates (fixed set supported by the adapter firmware)
 * -------------------------------------------------------------------------- */

struct ch341_can_bitrate_entry {
	u32 bitrate;
	enum ch341_can_speed code;
};

static const struct ch341_can_bitrate_entry ch341_can_bitrates[] = {
	{ 1000000, CH341_CAN_SPEED_1000000 },
	{  800000, CH341_CAN_SPEED_800000  },
	{  500000, CH341_CAN_SPEED_500000  },
	{  400000, CH341_CAN_SPEED_400000  },
	{  250000, CH341_CAN_SPEED_250000  },
	{  200000, CH341_CAN_SPEED_200000  },
	{  125000, CH341_CAN_SPEED_125000  },
	{  100000, CH341_CAN_SPEED_100000  },
	{   50000, CH341_CAN_SPEED_50000   },
	{   20000, CH341_CAN_SPEED_20000   },
	{   10000, CH341_CAN_SPEED_10000   },
	{    5000, CH341_CAN_SPEED_5000    },
};

/*
 * The adapter does not expose real CAN bit-timing registers; it only accepts
 * a speed code from a fixed list. We still provide bittiming_const so the
 * SocketCAN core can validate user requests, but we override do_set_bittiming
 * to map the resulting bitrate to a firmware speed code.
 */
static const struct can_bittiming_const ch341_can_bittiming_const = {
	.name		= DRV_NAME,
	.tseg1_min	= 1,
	.tseg1_max	= 16,
	.tseg2_min	= 1,
	.tseg2_max	= 8,
	.sjw_max	= 4,
	.brp_min	= 1,
	.brp_max	= 1024,
	.brp_inc	= 1,
};

static enum ch341_can_speed ch341_can_bitrate_to_code(u32 bitrate)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ch341_can_bitrates); i++) {
		if (ch341_can_bitrates[i].bitrate == bitrate)
			return ch341_can_bitrates[i].code;
	}
	return CH341_CAN_SPEED_INVALID;
}

/* --------------------------------------------------------------------------
 * CH341 USB-serial low-level communication
 * -------------------------------------------------------------------------- */

/* Default UART baudrate for the adapter (2 Mbaud) */
#define CH341_DEFAULT_BAUD	2000000

static int ch341_control_out(struct ch341_can_priv *priv, u8 request,
			     u16 value, u16 index)
{
	int r;

	r = usb_control_msg(priv->udev, usb_sndctrlpipe(priv->udev, 0),
			    request, USB_TYPE_VENDOR | USB_RECIP_DEVICE |
			    USB_DIR_OUT, value, index, NULL, 0, 1000);
	if (r < 0)
		dev_err(&priv->udev->dev, "failed to send control message: %d\n", r);

	return r;
}

static int ch341_control_in(struct ch341_can_priv *priv, u8 request,
			    u16 value, u16 index, void *buf, int len)
{
	int r;

	r = usb_control_msg_recv(priv->udev, 0, request,
				 USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
				 value, index, buf, len, 1000, GFP_KERNEL);
	if (r) {
		dev_err(&priv->udev->dev, "failed to receive control message: %d\n", r);
		return r;
	}

	return 0;
}

/*
 * Assert/deassert modem control lines (DTR/RTS).
 * Matches ch341_set_handshake() from the kernel ch341.c driver.
 */
static int ch341_set_handshake(struct ch341_can_priv *priv, u8 control)
{
	return ch341_control_out(priv, CH341_REQ_MODEM_CTRL, ~control, 0);
}

/*
 * Baud-rate divisor calculation.
 * Ported from ch341_get_divisor() in the kernel ch341.c driver.
 *
 * The device line speed is:
 *   baudrate = 48000000 / (2^(12 - 3*ps - fact) * div)
 *
 * where 0 <= ps <= 3, 0 <= fact <= 1,
 *       2 <= div <= 256 if fact = 0, or
 *       9 <= div <= 256 if fact = 1.
 */
#define CH341_CLKRATE		48000000
#define CH341_CLK_DIV(ps, fact)	(1 << (12 - 3 * (ps) - (fact)))
#define CH341_MIN_RATE(ps)	(CH341_CLKRATE / (CH341_CLK_DIV((ps), 1) * 512))
#define CH341_MIN_BPS		DIV_ROUND_UP(CH341_CLKRATE, CH341_CLK_DIV(0, 0) * 256)
#define CH341_MAX_BPS		(CH341_CLKRATE / (CH341_CLK_DIV(3, 0) * 2))

static const unsigned int ch341_min_rates[] = {
	CH341_MIN_RATE(0),
	CH341_MIN_RATE(1),
	CH341_MIN_RATE(2),
	CH341_MIN_RATE(3),
};

static int ch341_get_divisor(struct ch341_can_priv *priv, unsigned int speed)
{
	unsigned int fact, div, clk_div;
	bool force_fact0 = false;
	int ps;

	/*
	 * Clamp to supported range, this makes the (ps < 0) and (div < 2)
	 * sanity checks below redundant.
	 */
	speed = clamp_val(speed, CH341_MIN_BPS, CH341_MAX_BPS);

	/*
	 * Start with highest possible base clock (fact = 1) that will give a
	 * divisor strictly less than 512.
	 */
	fact = 1;
	for (ps = 3; ps >= 0; ps--) {
		if (speed > ch341_min_rates[ps])
			break;
	}

	if (ps < 0)
		return -EINVAL;

	/* Determine corresponding divisor, rounding down. */
	clk_div = CH341_CLK_DIV(ps, fact);
	div = CH341_CLKRATE / (clk_div * speed);

	/* Some devices require a lower base clock if ps < 3. */
	if (ps < 3 && (priv->quirks & CH341_QUIRK_LIMITED_PRESCALER))
		force_fact0 = true;

	/* Halve base clock (fact = 0) if required. */
	if (div < 9 || div > 255 || force_fact0) {
		div /= 2;
		clk_div *= 2;
		fact = 0;
	}

	if (div < 2)
		return -EINVAL;

	/*
	 * Pick next divisor if resulting rate is closer to the requested one,
	 * scale up to avoid rounding errors on low rates.
	 */
	if (16 * CH341_CLKRATE / (clk_div * div) - 16 * speed >=
	    16 * speed - 16 * CH341_CLKRATE / (clk_div * (div + 1)))
		div++;

	/*
	 * Prefer lower base clock (fact = 0) if even divisor.
	 *
	 * Note that this makes the receiver more tolerant to errors.
	 */
	if (fact == 1 && div % 2 == 0) {
		div /= 2;
		fact = 0;
	}

	return (0x100 - div) << 8 | fact << 2 | ps;
}

/*
 * Set baud rate and LCR.  Matches ch341_set_baudrate_lcr() from
 * the kernel ch341.c driver.
 */
static int ch341_set_baudrate_lcr(struct ch341_can_priv *priv,
				  unsigned int baud_rate, u8 lcr)
{
	int val, ret;

	if (!baud_rate)
		return -EINVAL;

	val = ch341_get_divisor(priv, baud_rate);
	if (val < 0)
		return val;

	/*
	 * CH341A buffers data until a full endpoint-size packet (32 bytes)
	 * has been received unless bit 7 is set.
	 * At least one device with version 0x27 appears to have this bit
	 * inverted.
	 */
	if (priv->version > 0x27)
		val |= BIT(7);

	ret = ch341_control_out(priv, CH341_REQ_WRITE_REG,
				CH341_REG_DIVISOR << 8 | CH341_REG_PRESCALER,
				val);
	if (ret < 0)
		return ret;

	/*
	 * Chip versions >= 0x30 use CH341_REG_LCR for line control.
	 * Older versions used separate registers — just skip LCR for those.
	 */
	if (priv->version >= 0x30) {
		ret = ch341_control_out(priv, CH341_REQ_WRITE_REG,
					CH341_REG_LCR2 << 8 | CH341_REG_LCR,
					lcr);
		if (ret < 0)
			return ret;
	}

	return 0;
}

/*
 * Detect device quirks.
 * Matches ch341_detect_quirks() from the kernel ch341.c driver.
 */
static int ch341_detect_quirks(struct ch341_can_priv *priv)
{
	const unsigned int size = 2;
	unsigned long quirks = 0;
	u8 buffer[2];
	int r;

	/*
	 * A subset of CH34x devices does not support all features. The
	 * prescaler is limited and there is no support for sending a RS232
	 * break condition. A read failure when trying to set up the latter is
	 * used to detect these devices.
	 */
	r = usb_control_msg_recv(priv->udev, 0, CH341_REQ_READ_REG,
				 USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
				 CH341_REG_BREAK, 0, &buffer, size,
				 1000, GFP_KERNEL);
	if (r == -EPIPE) {
		dev_info(&priv->udev->dev,
			 "break control not supported, enabling limited prescaler quirk\n");
		quirks = CH341_QUIRK_LIMITED_PRESCALER | CH341_QUIRK_SIMULATE_BREAK;
		r = 0;
	} else if (r) {
		dev_err(&priv->udev->dev, "failed to read break control: %d\n", r);
	}

	if (quirks) {
		dev_dbg(&priv->udev->dev, "enabling quirk flags: 0x%02lx\n", quirks);
		priv->quirks |= quirks;
	}

	return r;
}

/*
 * Full CH341 chip initialisation.
 * Matches ch341_configure() + ch341_set_termios() + ch341_open() from
 * the kernel ch341.c driver.  Sets up 8N1 at the requested baud rate,
 * asserts DTR/RTS, and disables hardware flow control.
 */
static int ch341_configure_uart(struct ch341_can_priv *priv, u32 baud)
{
	u8 buf[2];
	int ret;
	u8 lcr;

	if (baud == 0)
		return -EINVAL;

	/* ---- ch341_configure() ---- */

	ret = ch341_control_in(priv, CH341_REQ_READ_VERSION, 0, 0, buf, 2);
	if (ret)
		return ret;
	priv->version = buf[0];
	dev_dbg(&priv->udev->dev, "CH341 chip version: 0x%02x\n",
		priv->version);

	ret = ch341_control_out(priv, CH341_REQ_SERIAL_INIT, 0, 0);
	if (ret < 0)
		return ret;

	/* 8N1 */
	lcr = CH341_LCR_ENABLE_RX | CH341_LCR_ENABLE_TX | CH341_LCR_CS8;

	ret = ch341_set_baudrate_lcr(priv, baud, lcr);
	if (ret < 0)
		return ret;

	/* Assert DTR + RTS */
	ret = ch341_set_handshake(priv, CH341_BIT_DTR | CH341_BIT_RTS);
	if (ret < 0)
		return ret;

	/* ---- ch341_set_flow_control(): no hardware flow control ---- */
	ret = ch341_control_out(priv, CH341_REQ_WRITE_REG,
				(CH341_REG_FLOW_CTL << 8) | CH341_REG_FLOW_CTL,
				CH341_FLOW_CTL_NONE);
	if (ret < 0)
		return ret;

	return 0;
}

/* --------------------------------------------------------------------------
 * CAN adapter configuration protocol
 * -------------------------------------------------------------------------- */

/*
 * Configuration-packet checksum: the low 8 bits of the arithmetic sum of
 * every byte from the TYPE field (index CAN_CFG_CRC_IDX) up to and including
 * the last payload byte before the checksum itself (index CAN_CFG_PACKET_LEN
 * - 2).  See "USB (Serial port) to CAN protocol defines" — the worked example
 * sums bytes [2..18] to 0x293, whose low byte 0x93 is the check code.
 */
static u8 ch341_can_calc_crc(const u8 *data)
{
	u8 crc = 0;
	int i;

	for (i = CAN_CFG_CRC_IDX; i < CAN_CFG_PACKET_LEN - 1; i++)
		crc += data[i];

	return crc & 0xFF;
}

/*
 * Send configuration frame to the CAN adapter over the USB bulk endpoint.
 * Must be called with priv->cmd_lock held.
 */
static int ch341_can_send_config(struct ch341_can_priv *priv,
				 enum ch341_can_speed speed,
				 enum ch341_can_mode mode,
				 enum ch341_can_frame_type frame_type)
{
	u8 *cmd;
	int actual_len, ret;

	cmd = kzalloc(CAN_CFG_PACKET_LEN, GFP_KERNEL);
	if (!cmd)
		return -ENOMEM;

	cmd[0]  = CAN_PACKET_START;
	cmd[1]  = CAN_CFG_PACKET_TYPE;
	cmd[2]  = 0x12;
	cmd[3]  = speed;
	cmd[4]  = frame_type;
	cmd[5]  = 0x00;	/* Filter ID (unused) */
	cmd[6]  = 0x00;
	cmd[7]  = 0x00;
	cmd[8]  = 0x00;
	cmd[9]  = 0x00;	/* Mask ID (unused) */
	cmd[10] = 0x00;
	cmd[11] = 0x00;
	cmd[12] = 0x00;
	cmd[13] = mode;
	cmd[14] = 0x01;
	cmd[15] = 0x00;
	cmd[16] = 0x00;
	cmd[17] = 0x00;
	cmd[18] = 0x00;
	cmd[19] = ch341_can_calc_crc(cmd);

	ret = usb_bulk_msg(priv->udev, usb_sndbulkpipe(priv->udev,
			   priv->bulk_out_ep), cmd, CAN_CFG_PACKET_LEN,
			   &actual_len, 1000);

	kfree(cmd);
	return ret;
}

/* --------------------------------------------------------------------------
 * CAN frame decapsulation (RX) — device protocol → SocketCAN
 * -------------------------------------------------------------------------- */

#define IS_EXT_ID(type)		((type) & CAN_FLAG_ID_EXT)
#define IS_REMOTE(type)		((type) & CAN_FLAG_RTR)
#define IS_DATA_FRAME(type)	(((type) >> 6) == 3)
#define GET_DLC(type)		((type) & 0x0F)

static void ch341_can_rx_frame(struct ch341_can_priv *priv)
{
	struct sk_buff *skb;
	struct can_frame *cf;
	u8 *buf = priv->rx_assembly;
	u8 type_byte = buf[1];
	u8 dlc = GET_DLC(type_byte);
	u8 data_start;
	u32 id;

	if (dlc > CAN_MAX_PAYLOAD)
		return;

	/* Validate buffer bounds and compute id + data_start before alloc */
	if (IS_EXT_ID(type_byte)) {
		if (priv->rx_count < 6) {
			priv->netdev->stats.rx_errors++;
			return;
		}
		id = ((u32)buf[5] << 24) | ((u32)buf[4] << 16) |
		     ((u32)buf[3] << 8)  |  (u32)buf[2];
		data_start = 6;
	} else {
		if (priv->rx_count < 4) {
			priv->netdev->stats.rx_errors++;
			return;
		}
		id = ((u32)buf[3] << 8) | (u32)buf[2];
		data_start = 4;
	}

	skb = alloc_can_skb(priv->netdev, &cf);
	if (!skb) {
		priv->netdev->stats.rx_dropped++;
		return;
	}

	if (IS_EXT_ID(type_byte))
		cf->can_id = id | CAN_EFF_FLAG;
	else
		cf->can_id = id;

	if (IS_REMOTE(type_byte))
		cf->can_id |= CAN_RTR_FLAG;

	cf->len = dlc;

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		/* Validate we have enough data bytes */
		if (priv->rx_count < data_start + dlc) {
			priv->netdev->stats.rx_errors++;
			kfree_skb(skb);
			return;
		}
		memcpy(cf->data, buf + data_start, dlc);
	}

	priv->netdev->stats.rx_packets++;
	priv->netdev->stats.rx_bytes += cf->len;

	/* Add software receive timestamp */
	skb->tstamp = ktime_get_real();

	netif_rx(skb);
}

static void ch341_can_rx_process_byte(struct ch341_can_priv *priv, u8 byte)
{
	unsigned long flags;

	spin_lock_irqsave(&priv->rx_lock, flags);

	/* Synchronise on start-of-frame */
	if (priv->rx_count == 0) {
		if (byte != CAN_PACKET_START) {
			spin_unlock_irqrestore(&priv->rx_lock, flags);
			return;
		}
		priv->rx_assembly[priv->rx_count++] = byte;
		priv->rx_expected = 0;
		spin_unlock_irqrestore(&priv->rx_lock, flags);
		return;
	}

	if (priv->rx_count >= (int)sizeof(priv->rx_assembly)) {
		priv->netdev->stats.rx_over_errors++;
		priv->rx_count = 0;
		spin_unlock_irqrestore(&priv->rx_lock, flags);
		return;
	}

	priv->rx_assembly[priv->rx_count++] = byte;

	/* After receiving the type byte we can compute expected length */
	if (priv->rx_count == 2) {
		u8 type_byte = priv->rx_assembly[1];

		if (!IS_DATA_FRAME(type_byte)) {
			/* Not a data/remote frame — reset */
			priv->rx_count = 0;
			spin_unlock_irqrestore(&priv->rx_lock, flags);
			return;
		}

		int id_len = IS_EXT_ID(type_byte) ? 4 : 2;
		int dlc = GET_DLC(type_byte);
		int data_len = IS_REMOTE(type_byte) ? 0 : dlc;

		priv->rx_expected = 1 +	/* start byte */
				    1 +	/* type byte */
				    id_len +
				    data_len +
				    1;	/* end byte */

		if (dlc > CAN_MAX_PAYLOAD) {
			priv->rx_count = 0;
			spin_unlock_irqrestore(&priv->rx_lock, flags);
			return;
		}
	}

	/* Wait until we have enough bytes */
	if (priv->rx_expected > 0 && priv->rx_count >= priv->rx_expected) {
		spin_unlock_irqrestore(&priv->rx_lock, flags);
		ch341_can_rx_frame(priv);
		spin_lock_irqsave(&priv->rx_lock, flags);
		priv->rx_count = 0;
	}

	spin_unlock_irqrestore(&priv->rx_lock, flags);
}

/* --------------------------------------------------------------------------
 * USB URB callbacks
 * -------------------------------------------------------------------------- */

static void ch341_can_rx_callback(struct urb *urb)
{
	struct ch341_can_priv *priv = urb->context;
	int i, ret;

	if (!netif_running(priv->netdev))
		return;

	switch (urb->status) {
	case 0:
		break;
	case -ENOENT:
	case -ESHUTDOWN:
	case -ECONNRESET:
		return;
	default:
		dev_warn(&priv->udev->dev, "RX URB error: %d\n", urb->status);
		goto resubmit;
	}

	for (i = 0; i < urb->actual_length; i++)
		ch341_can_rx_process_byte(priv, priv->rx_buf[i]);

resubmit:
	ret = usb_submit_urb(urb, GFP_ATOMIC);
	if (ret)
		dev_err(&priv->udev->dev, "RX URB resubmit failed: %d\n", ret);
}

/* --------------------------------------------------------------------------
 * CAN frame encapsulation (TX) — SocketCAN → device protocol
 * -------------------------------------------------------------------------- */

/*
 * TX URB completion.  Runs in interrupt context.  On success the echoed skb
 * is pushed to local listeners; on error it is freed.  Either way the TX
 * queue is re-woken so the next frame can be submitted.
 */
static void ch341_can_tx_callback(struct urb *urb)
{
	struct ch341_can_priv *priv = urb->context;
	struct net_device *netdev = priv->netdev;

	switch (urb->status) {
	case 0:
		netdev->stats.tx_packets++;
		netdev->stats.tx_bytes += can_get_echo_skb(netdev, 0, NULL);
		break;
	case -ENOENT:
	case -ESHUTDOWN:
	case -ECONNRESET:
		/* URB unlinked on close/disconnect/suspend — do not re-wake */
		can_free_echo_skb(netdev, 0, NULL);
		return;
	default:
		dev_warn(&priv->udev->dev, "TX URB error: %d\n", urb->status);
		can_free_echo_skb(netdev, 0, NULL);
		netdev->stats.tx_errors++;
		break;
	}

	netif_wake_queue(netdev);
}

/* --------------------------------------------------------------------------
 * Net device operations
 * -------------------------------------------------------------------------- */

static int ch341_can_open(struct net_device *netdev)
{
	struct ch341_can_priv *priv = netdev_priv(netdev);
	enum ch341_can_mode mode;
	int ret;

	/* The bitrate was validated and mapped in ch341_can_set_bittiming() */
	if (priv->speed_code == CH341_CAN_SPEED_INVALID) {
		dev_err(&priv->udev->dev,
			"Unsupported bitrate %u. Supported: 5000..1000000\n",
			priv->can.bittiming.bitrate);
		return -EINVAL;
	}

	ret = open_candev(netdev);
	if (ret)
		return ret;

	/* Map SocketCAN ctrlmode to device mode */
	if ((priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK) &&
	    (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY))
		mode = CH341_CAN_MODE_LOOPBACK_SILENT;
	else if (priv->can.ctrlmode & CAN_CTRLMODE_LOOPBACK)
		mode = CH341_CAN_MODE_LOOPBACK;
	else if (priv->can.ctrlmode & CAN_CTRLMODE_LISTENONLY)
		mode = CH341_CAN_MODE_SILENT;
	else
		mode = CH341_CAN_MODE_NORMAL;

	priv->device_mode = mode;

	/* Configure the CH341 UART */
	ret = ch341_configure_uart(priv, CH341_DEFAULT_BAUD);
	if (ret) {
		dev_err(&priv->udev->dev, "UART configuration failed: %d\n",
			ret);
		goto err_close_candev;
	}

	/* Send CAN configuration to adapter */
	mutex_lock(&priv->cmd_lock);
	ret = ch341_can_send_config(priv, priv->speed_code, mode,
				    priv->frame_type);
	mutex_unlock(&priv->cmd_lock);
	if (ret) {
		dev_err(&priv->udev->dev, "CAN config failed: %d\n", ret);
		goto err_close_candev;
	}

	/* Start RX URB */
	usb_fill_bulk_urb(priv->rx_urb, priv->udev,
			  usb_rcvbulkpipe(priv->udev, priv->bulk_in_ep),
			  priv->rx_buf, priv->bulk_in_size,
			  ch341_can_rx_callback, priv);
	priv->rx_urb->transfer_dma = priv->rx_buf_dma;
	priv->rx_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	ret = usb_submit_urb(priv->rx_urb, GFP_KERNEL);
	if (ret) {
		dev_err(&priv->udev->dev, "RX URB submit failed: %d\n", ret);
		goto err_close_candev;
	}

	priv->can.state = CAN_STATE_ERROR_ACTIVE;
	netif_start_queue(netdev);

	return 0;

err_close_candev:
	close_candev(netdev);
	return ret;
}

static int ch341_can_close(struct net_device *netdev)
{
	struct ch341_can_priv *priv = netdev_priv(netdev);

	netif_stop_queue(netdev);

	usb_kill_urb(priv->rx_urb);
	usb_kill_urb(priv->tx_urb);

	priv->can.state = CAN_STATE_STOPPED;
	close_candev(netdev);

	return 0;
}

static netdev_tx_t ch341_can_start_xmit(struct sk_buff *skb,
					 struct net_device *netdev)
{
	struct ch341_can_priv *priv = netdev_priv(netdev);
	struct can_frame *cf = (struct can_frame *)skb->data;
	u8 *buf = priv->tx_buf;
	int pos = 0, i, ret;
	u32 id;

	if (can_dev_dropped_skb(netdev, skb))
		return NETDEV_TX_OK;

	/*
	 * Only one frame may be in flight at a time; stop the queue until the
	 * TX URB completes and wakes it again.  This provides the backpressure
	 * that keeps the driver from queueing frames without bound.
	 */
	netif_stop_queue(netdev);

	buf[pos++] = CAN_PACKET_START;

	/* Type byte: 0xC0 | DLC, plus RTR / extended-ID flags */
	buf[pos] = CAN_FRAME_PREFIX | cf->len;

	if (cf->can_id & CAN_RTR_FLAG)
		buf[pos] |= CAN_FLAG_RTR;

	id = cf->can_id & CAN_EFF_MASK;

	if (cf->can_id & CAN_EFF_FLAG) {
		buf[pos] |= CAN_FLAG_ID_EXT;
		pos++;
		buf[pos++] = (u8)(id & 0xFF);
		buf[pos++] = (u8)((id >> 8) & 0xFF);
		buf[pos++] = (u8)((id >> 16) & 0xFF);
		buf[pos++] = (u8)((id >> 24) & 0xFF);
	} else {
		id &= CAN_SFF_MASK;
		pos++;
		buf[pos++] = (u8)(id & 0xFF);
		buf[pos++] = (u8)((id >> 8) & 0xFF);
	}

	if (!(cf->can_id & CAN_RTR_FLAG)) {
		for (i = 0; i < cf->len; i++)
			buf[pos++] = cf->data[i];
	}

	buf[pos++] = CAN_PACKET_END;

	/* Stash the skb for local echo; released in the TX completion */
	can_put_echo_skb(skb, netdev, 0, 0);

	usb_fill_bulk_urb(priv->tx_urb, priv->udev,
			  usb_sndbulkpipe(priv->udev, priv->bulk_out_ep),
			  buf, pos, ch341_can_tx_callback, priv);
	priv->tx_urb->transfer_dma = priv->tx_buf_dma;
	priv->tx_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

	ret = usb_submit_urb(priv->tx_urb, GFP_ATOMIC);
	if (ret) {
		dev_err(&priv->udev->dev, "TX URB submit failed: %d\n", ret);
		can_free_echo_skb(netdev, 0, NULL);
		netdev->stats.tx_dropped++;
		netif_wake_queue(netdev);
	}

	return NETDEV_TX_OK;
}

static int ch341_can_set_mode(struct net_device *netdev, enum can_mode mode)
{
	struct ch341_can_priv *priv = netdev_priv(netdev);

	switch (mode) {
	case CAN_MODE_START:
		priv->can.state = CAN_STATE_ERROR_ACTIVE;
		netif_wake_queue(netdev);
		return 0;
	default:
		return -EOPNOTSUPP;
	}
}

/*
 * The adapter only accepts a fixed set of speed codes rather than raw bit
 * timing.  Validate the requested bitrate here so an unsupported value is
 * rejected when it is set, not silently deferred to link-up.  The resulting
 * speed code is sent to the device in ch341_can_open().
 */
static int ch341_can_set_bittiming(struct net_device *netdev)
{
	struct ch341_can_priv *priv = netdev_priv(netdev);
	enum ch341_can_speed code;

	code = ch341_can_bitrate_to_code(priv->can.bittiming.bitrate);
	if (code == CH341_CAN_SPEED_INVALID) {
		netdev_err(netdev,
			   "Unsupported bitrate %u. Supported: 5000..1000000\n",
			   priv->can.bittiming.bitrate);
		return -EINVAL;
	}

	priv->speed_code = code;
	return 0;
}

static const struct net_device_ops ch341_can_netdev_ops = {
	.ndo_open	= ch341_can_open,
	.ndo_stop	= ch341_can_close,
	.ndo_start_xmit	= ch341_can_start_xmit,
	.ndo_change_mtu	= can_change_mtu,
};

/* --------------------------------------------------------------------------
 * USB probe / disconnect
 * -------------------------------------------------------------------------- */

static const struct usb_device_id ch341_can_id_table[] = {
	{ USB_DEVICE(CH341_CAN_VENDOR_ID, CH341_CAN_PRODUCT_ID) },
	{ }
};
MODULE_DEVICE_TABLE(usb, ch341_can_id_table);

static int ch341_can_probe(struct usb_interface *intf,
			   const struct usb_device_id *id)
{
	struct usb_device *udev = interface_to_usbdev(intf);
	struct usb_host_interface *iface_desc = intf->cur_altsetting;
	struct usb_endpoint_descriptor *ep;
	struct net_device *netdev;
	struct ch341_can_priv *priv;
	int i, ret;
	u8 bulk_in_ep = 0, bulk_out_ep = 0;
	size_t bulk_in_size = 0;

	/* Find bulk IN and OUT endpoints */
	for (i = 0; i < iface_desc->desc.bNumEndpoints; i++) {
		ep = &iface_desc->endpoint[i].desc;

		if (usb_endpoint_is_bulk_in(ep) && !bulk_in_ep) {
			bulk_in_ep = ep->bEndpointAddress;
			bulk_in_size = usb_endpoint_maxp(ep);
		}
		if (usb_endpoint_is_bulk_out(ep) && !bulk_out_ep)
			bulk_out_ep = ep->bEndpointAddress;
	}

	if (!bulk_in_ep || !bulk_out_ep) {
		dev_err(&intf->dev, "Could not find bulk endpoints\n");
		return -ENODEV;
	}

	/* Validate endpoint directions */
	if (!(bulk_in_ep & 0x80)) {
		dev_err(&intf->dev, "Invalid bulk IN endpoint address: 0x%02x\n", bulk_in_ep);
		return -ENODEV;
	}
	if (bulk_out_ep & 0x80) {
		dev_err(&intf->dev, "Invalid bulk OUT endpoint address: 0x%02x\n", bulk_out_ep);
		return -ENODEV;
	}

	/*
	 * The RX URB reads into a fixed RX_BUF_SIZE buffer, so never trust a
	 * device that advertises a larger max packet size than we allocate.
	 */
	if (bulk_in_size > RX_BUF_SIZE)
		bulk_in_size = RX_BUF_SIZE;

	/* Allocate SocketCAN netdev with room for one echo skb (local echo) */
	netdev = alloc_candev(sizeof(struct ch341_can_priv), 1);
	if (!netdev)
		return -ENOMEM;

	priv = netdev_priv(netdev);

	priv->udev = udev;
	priv->netdev = netdev;
	priv->bulk_in_ep = bulk_in_ep;
	priv->bulk_out_ep = bulk_out_ep;
	priv->bulk_in_size = bulk_in_size;

	priv->device_mode = CH341_CAN_MODE_NORMAL;
	priv->speed_code = CH341_CAN_SPEED_INVALID;
	/*
	 * The config command's frame-type field only seeds the adapter's
	 * default; standard and extended frames are still distinguished
	 * per-frame in the variable-length TX/RX protocol.
	 */
	priv->frame_type = CH341_CAN_FRAME_STANDARD;

	mutex_init(&priv->cmd_lock);
	spin_lock_init(&priv->rx_lock);

	priv->rx_count = 0;
	priv->rx_expected = 0;

	/* Detect device quirks */
	ret = ch341_detect_quirks(priv);
	if (ret < 0) {
		dev_err(&intf->dev, "quirk detection failed: %d\n", ret);
		goto err_free_bufs;
	}

	/* CAN device configuration */
	priv->can.clock.freq = 8000000; /* nominal, not actually used */
	priv->can.bittiming_const = &ch341_can_bittiming_const;
	priv->can.do_set_bittiming = ch341_can_set_bittiming;
	priv->can.do_set_mode = ch341_can_set_mode;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
				       CAN_CTRLMODE_LISTENONLY |
				       CAN_CTRLMODE_CC_LEN8_DLC;

	netdev->netdev_ops = &ch341_can_netdev_ops;
	netdev->flags |= IFF_ECHO;

	SET_NETDEV_DEV(netdev, &intf->dev);

	/* Allocate URBs and buffers */
	priv->rx_urb = usb_alloc_urb(0, GFP_KERNEL);
	priv->tx_urb = usb_alloc_urb(0, GFP_KERNEL);
	if (!priv->rx_urb || !priv->tx_urb) {
		ret = -ENOMEM;
		goto err_free_urbs;
	}

	priv->rx_buf = usb_alloc_coherent(udev, RX_BUF_SIZE, GFP_KERNEL,
					  &priv->rx_buf_dma);
	priv->tx_buf = usb_alloc_coherent(udev, TX_BUF_SIZE, GFP_KERNEL,
					  &priv->tx_buf_dma);
	if (!priv->rx_buf || !priv->tx_buf) {
		ret = -ENOMEM;
		goto err_free_bufs;
	}

	/* Register CAN device */
	ret = register_candev(netdev);
	if (ret) {
		dev_err(&intf->dev, "register_candev() failed: %d\n", ret);
		goto err_free_bufs;
	}

	usb_set_intfdata(intf, priv);

	dev_info(&intf->dev, "CH341 USB-CAN adapter registered as %s\n",
		 netdev->name);

	return 0;

err_free_bufs:
	if (priv->rx_buf)
		usb_free_coherent(udev, RX_BUF_SIZE, priv->rx_buf,
				  priv->rx_buf_dma);
	if (priv->tx_buf)
		usb_free_coherent(udev, TX_BUF_SIZE, priv->tx_buf,
				  priv->tx_buf_dma);
err_free_urbs:
	usb_free_urb(priv->rx_urb);
	usb_free_urb(priv->tx_urb);
	free_candev(netdev);
	return ret;
}

static void ch341_can_disconnect(struct usb_interface *intf)
{
	struct ch341_can_priv *priv = usb_get_intfdata(intf);
	struct net_device *netdev;

	if (!priv)
		return;

	netdev = priv->netdev;
	usb_set_intfdata(intf, NULL);

	/* Unregister netdev first to prevent new opens */
	unregister_candev(netdev);

	/* Stop all I/O */
	usb_kill_urb(priv->rx_urb);
	usb_kill_urb(priv->tx_urb);

	/* Free resources */
	usb_free_coherent(priv->udev, RX_BUF_SIZE, priv->rx_buf,
			  priv->rx_buf_dma);
	usb_free_coherent(priv->udev, TX_BUF_SIZE, priv->tx_buf,
			  priv->tx_buf_dma);
	usb_free_urb(priv->rx_urb);
	usb_free_urb(priv->tx_urb);

	free_candev(netdev);

	dev_info(&intf->dev, "CH341 USB-CAN adapter disconnected\n");
}

static int ch341_can_suspend(struct usb_interface *intf, pm_message_t message)
{
	struct ch341_can_priv *priv = usb_get_intfdata(intf);

	if (!priv)
		return 0;

	if (netif_running(priv->netdev)) {
		netif_stop_queue(priv->netdev);
		usb_kill_urb(priv->rx_urb);
		usb_kill_urb(priv->tx_urb);
	}

	return 0;
}

static int ch341_can_resume(struct usb_interface *intf)
{
	struct ch341_can_priv *priv = usb_get_intfdata(intf);
	int ret;

	if (!priv)
		return 0;

	if (netif_running(priv->netdev)) {
		/* Reconfigure the CH341 UART after suspend */
		ret = ch341_configure_uart(priv, CH341_DEFAULT_BAUD);
		if (ret) {
			dev_err(&priv->udev->dev,
				"UART reconfiguration on resume failed: %d\n",
				ret);
			return ret;
		}

		/* Resend CAN adapter configuration */
		if (priv->speed_code != CH341_CAN_SPEED_INVALID) {
			mutex_lock(&priv->cmd_lock);
			ret = ch341_can_send_config(priv, priv->speed_code,
						    priv->device_mode,
						    priv->frame_type);
			mutex_unlock(&priv->cmd_lock);
			if (ret) {
				dev_err(&priv->udev->dev,
					"CAN reconfiguration on resume failed: %d\n",
					ret);
				return ret;
			}
		}

		usb_fill_bulk_urb(priv->rx_urb, priv->udev,
				  usb_rcvbulkpipe(priv->udev, priv->bulk_in_ep),
				  priv->rx_buf, priv->bulk_in_size,
				  ch341_can_rx_callback, priv);
		priv->rx_urb->transfer_dma = priv->rx_buf_dma;
		priv->rx_urb->transfer_flags |= URB_NO_TRANSFER_DMA_MAP;

		ret = usb_submit_urb(priv->rx_urb, GFP_KERNEL);
		if (ret) {
			dev_err(&priv->udev->dev, "RX URB resume failed: %d\n", ret);
			return ret;
		}
		netif_wake_queue(priv->netdev);
	}

	return 0;
}

static int ch341_can_reset_resume(struct usb_interface *intf)
{
	return ch341_can_resume(intf);
}

static struct usb_driver ch341_can_driver = {
	.name		= DRV_NAME,
	.id_table	= ch341_can_id_table,
	.probe		= ch341_can_probe,
	.disconnect	= ch341_can_disconnect,
	.suspend	= ch341_can_suspend,
	.resume		= ch341_can_resume,
	.reset_resume	= ch341_can_reset_resume,
};

module_usb_driver(ch341_can_driver);

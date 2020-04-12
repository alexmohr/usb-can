// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2007, Frank A Kingswood <frank@kingswood-consulting.co.uk>
 * Copyright 2007, Werner Cornelius <werner@cornelius-consult.de>
 * Copyright 2009, Boris Hajduk <boris@hajduk.org>
 *
 * ch341.c implements a serial port driver for the Winchiphead CH341.
 *
 * The CH341 device can be used to implement an RS232 asynchronous
 * serial port, an IEEE-1284 parallel printer port or a memory-like
 * interface. In all cases the CH341 supports an I2C interface as well.
 * This driver only supports the asynchronous serial interface.
 */

#include <linux/kernel.h>
#include <linux/tty.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/usb.h>
#include <linux/usb/serial.h>
#include <linux/serial.h>
#include <asm/unaligned.h>

#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/skb.h>
#include <linux/can/error.h>

#define HLCAN_FRAME_PREFIX 	0xC0
#define HLCAN_FLAG_RTR	 	0x10
#define HLCAN_FLAG_ID_EXT 	0x20
#define HLCAN_TYPE_MASK		0xF0

#define HLCAN_STD_DATA_FRAME 	0xC0
#define HLCAN_EXT_DATA_FRAME 	0xE0
#define HCLAN_STD_REMOTE_FRAME 	0xD0
#define HCLAN_EXT_REMOTE_FRAME 	0xF0

#define HLCAN_PACKET_START 		0xAA
#define HLCAN_PACKET_END		0x55

#define HLCAN_CFG_PACKAGE_TYPE	0x55
#define HLCAN_CFG_PACKAGE_LEN	0x14
#define HLCAN_CFG_CRC_IDX		0x02

#define IO_CTL_MODE             0xF3

typedef enum {
	NONE,
	RECEIVING,
	COMPLETE,
	MISSED_HEADER
} FRAME_STATE;

typedef enum {
    HLCAN_SPEED_1000000 = 0x01,
    HLCAN_SPEED_800000 = 0x02,
    HLCAN_SPEED_500000 = 0x03,
    HLCAN_SPEED_400000 = 0x04,
    HLCAN_SPEED_250000 = 0x05,
    HLCAN_SPEED_200000 = 0x06,
    HLCAN_SPEED_125000 = 0x07,
    HLCAN_SPEED_100000 = 0x08,
    HLCAN_SPEED_50000 = 0x09,
    HLCAN_SPEED_20000 = 0x0a,
    HLCAN_SPEED_10000 = 0x0b,
    HLCAN_SPEED_5000 = 0x0c,
    HLCAN_SPEED_INVALID = 0xff,
} HLCAN_SPEED;

typedef enum {
    HLCAN_MODE_NORMAL = 0x00,
    HLCAN_MODE_LOOPBACK = 0x01,
    HLCAN_MODE_SILENT = 0x02,
    HLCAN_MODE_LOOPBACK_SILENT = 0x03,
} HLCAN_MODE;

typedef enum {
    HLCAN_FRAME_STANDARD = 0x01,
    HLCAN_FRAME_EXTENDED = 0x02,
} HLCAN_FRAME_TYPE;

#define DEFAULT_BAUD_RATE 9600
#define DEFAULT_TIMEOUT   1000

struct ch341_can_struct {
	struct can_priv can;
	struct usb_serial_port *port;
	struct net_device *netdev;
	
	int			rcount;         /* received chars counter    */
	int			rexpected;	/* expected chars counter    */
	FRAME_STATE 		rstate; 	/* state of current receive  */
	int mode;
};



/* flags for IO-Bits */
#define ch341_can_BIT_RTS (1 << 6)
#define ch341_can_BIT_DTR (1 << 5)

/******************************/
/* interrupt pipe definitions */
/******************************/
/* always 4 interrupt bytes */
/* first irq byte normally 0x08 */
/* second irq byte base 0x7d + below */
/* third irq byte base 0x94 + below */
/* fourth irq byte normally 0xee */

/* second interrupt byte */
#define ch341_can_MULT_STAT 0x04 /* multiple status since last interrupt event */

/* status returned in third interrupt answer byte, inverted in data
   from irq */
#define ch341_can_BIT_CTS 0x01
#define ch341_can_BIT_DSR 0x02
#define ch341_can_BIT_RI  0x04
#define ch341_can_BIT_DCD 0x08
#define ch341_can_BITS_MODEM_STAT 0x0f /* all bits */

/* Break support - the information used to implement this was gleaned from
 * the Net/FreeBSD uchcom.c driver by Takanori Watanabe.  Domo arigato.
 */

#define ch341_can_REQ_READ_VERSION 0x5F
#define ch341_can_REQ_WRITE_REG    0x9A
#define ch341_can_REQ_READ_REG     0x95
#define ch341_can_REQ_SERIAL_INIT  0xA1
#define ch341_can_REQ_MODEM_CTRL   0xA4

#define ch341_can_REG_BREAK        0x05
#define ch341_can_REG_LCR          0x18
#define ch341_can_NBREAK_BITS      0x01

#define ch341_can_LCR_ENABLE_RX    0x80
#define ch341_can_LCR_ENABLE_TX    0x40
#define ch341_can_LCR_MARK_SPACE   0x20
#define ch341_can_LCR_PAR_EVEN     0x10
#define ch341_can_LCR_ENABLE_PAR   0x08
#define ch341_can_LCR_STOP_BITS_2  0x04
#define ch341_can_LCR_CS8          0x03
#define ch341_can_LCR_CS7          0x02
#define ch341_can_LCR_CS6          0x01
#define ch341_can_LCR_CS5          0x00



static const struct usb_device_id id_table[] = {
	{ USB_DEVICE(0x1a86, 0x7523) },
	{ },
};
MODULE_DEVICE_TABLE(usb, id_table);

struct ch341_can_private {
	spinlock_t lock; /* access lock */
	unsigned baud_rate; /* set baud rate */
	u8 mcr;
	u8 msr;
	u8 lcr;
	struct ch341_can_struct* can;
};

static void ch341_can_set_termios(struct tty_struct *tty,
			      struct usb_serial_port *port,
			      struct ktermios *old_termios);

static int ch341_can_net_attach(struct usb_serial_port *port, struct ch341_can_private *serialPriv);

static int ch341_can_control_out(struct usb_device *dev, u8 request,
			     u16 value, u16 index)
{
	int r;

	dev_dbg(&dev->dev, "%s - (%02x,%04x,%04x)\n", __func__,
		request, value, index);

	r = usb_control_msg(dev, usb_sndctrlpipe(dev, 0), request,
			    USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_OUT,
			    value, index, NULL, 0, DEFAULT_TIMEOUT);
	if (r < 0)
		dev_err(&dev->dev, "failed to send control message: %d\n", r);

	return r;
}

static int ch341_can_control_in(struct usb_device *dev,
			    u8 request, u16 value, u16 index,
			    char *buf, unsigned bufsize)
{
	int r;

	dev_dbg(&dev->dev, "%s - (%02x,%04x,%04x,%u)\n", __func__,
		request, value, index, bufsize);

	r = usb_control_msg(dev, usb_rcvctrlpipe(dev, 0), request,
			    USB_TYPE_VENDOR | USB_RECIP_DEVICE | USB_DIR_IN,
			    value, index, buf, bufsize, DEFAULT_TIMEOUT);
	if (r < (int)bufsize) {
		if (r >= 0) {
			dev_err(&dev->dev,
				"short control message received (%d < %u)\n",
				r, bufsize);
			r = -EIO;
		}

		dev_err(&dev->dev, "failed to receive control message: %d\n",
			r);
		return r;
	}

	return 0;
}

#define ch341_can_CLKRATE		48000000
#define ch341_can_CLK_DIV(ps, fact)	(1 << (12 - 3 * (ps) - (fact)))
#define ch341_can_MIN_RATE(ps)	(ch341_can_CLKRATE / (ch341_can_CLK_DIV((ps), 1) * 512))

static const speed_t ch341_can_min_rates[] = {
	ch341_can_MIN_RATE(0),
	ch341_can_MIN_RATE(1),
	ch341_can_MIN_RATE(2),
	ch341_can_MIN_RATE(3),
};

/*
 * The device line speed is given by the following equation:
 *
 *	baudrate = 48000000 / (2^(12 - 3 * ps - fact) * div), where
 *
 *		0 <= ps <= 3,
 *		0 <= fact <= 1,
 *		2 <= div <= 256 if fact = 0, or
 *		9 <= div <= 256 if fact = 1
 */
static int ch341_can_get_divisor(speed_t speed)
{
	unsigned int fact, div, clk_div;
	int ps;

	/*
	 * Clamp to supported range, this makes the (ps < 0) and (div < 2)
	 * sanity checks below redundant.
	 */
	speed = clamp(speed, 46U, 3000000U);

	/*
	 * Start with highest possible base clock (fact = 1) that will give a
	 * divisor strictly less than 512.
	 */
	fact = 1;
	for (ps = 3; ps >= 0; ps--) {
		if (speed > ch341_can_min_rates[ps])
			break;
	}

	if (ps < 0)
		return -EINVAL;

	/* Determine corresponding divisor, rounding down. */
	clk_div = ch341_can_CLK_DIV(ps, fact);
	div = ch341_can_CLKRATE / (clk_div * speed);

	/* Halve base clock (fact = 0) if required. */
	if (div < 9 || div > 255) {
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
	if (16 * ch341_can_CLKRATE / (clk_div * div) - 16 * speed >=
			16 * speed - 16 * ch341_can_CLKRATE / (clk_div * (div + 1)))
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

static int ch341_can_set_baudrate_lcr(struct usb_device *dev,
				  struct ch341_can_private *priv, u8 lcr)
{
	int val;
	int r;

	if (!priv->baud_rate)
		return -EINVAL;

	val = ch341_can_get_divisor(priv->baud_rate);
	if (val < 0)
		return -EINVAL;

	/*
	 * CH341A buffers data until a full endpoint-size packet (32 bytes)
	 * has been received unless bit 7 is set.
	 */
	val |= BIT(7);

	r = ch341_can_control_out(dev, ch341_can_REQ_WRITE_REG, 0x1312, val);
	if (r)
		return r;

	r = ch341_can_control_out(dev, ch341_can_REQ_WRITE_REG, 0x2518, lcr);
	if (r)
		return r;

	return r;
}

static int ch341_can_set_handshake(struct usb_device *dev, u8 control)
{
	return ch341_can_control_out(dev, ch341_can_REQ_MODEM_CTRL, ~control, 0);
}

static int ch341_can_get_status(struct usb_device *dev, struct ch341_can_private *priv)
{
	const unsigned int size = 2;
	char *buffer;
	int r;
	unsigned long flags;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	r = ch341_can_control_in(dev, ch341_can_REQ_READ_REG, 0x0706, 0, buffer, size);
	if (r < 0)
		goto out;

	spin_lock_irqsave(&priv->lock, flags);
	priv->msr = (~(*buffer)) & ch341_can_BITS_MODEM_STAT;
	spin_unlock_irqrestore(&priv->lock, flags);

out:	kfree(buffer);
	return r;
}

/* -------------------------------------------------------------------------- */

static int ch341_can_configure(struct usb_device *dev, struct ch341_can_private *priv)
{
	const unsigned int size = 2;
	char *buffer;
	int r;

	buffer = kmalloc(size, GFP_KERNEL);
	if (!buffer)
		return -ENOMEM;

	/* expect two bytes 0x27 0x00 */
	r = ch341_can_control_in(dev, ch341_can_REQ_READ_VERSION, 0, 0, buffer, size);
	if (r < 0)
		goto out;
	dev_dbg(&dev->dev, "Chip version: 0x%02x\n", buffer[0]);

	r = ch341_can_control_out(dev, ch341_can_REQ_SERIAL_INIT, 0, 0);
	if (r < 0)
		goto out;

	r = ch341_can_set_baudrate_lcr(dev, priv, priv->lcr);
	if (r < 0)
		goto out;

	r = ch341_can_set_handshake(dev, priv->mcr);

out:	kfree(buffer);
	return r;
}

static int ch341_can_port_probe(struct usb_serial_port *port)
{
	struct ch341_can_private *priv;
	int r;

	priv = kzalloc(sizeof(struct ch341_can_private), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	priv->baud_rate = DEFAULT_BAUD_RATE;
	/*
	 * Some CH340 devices appear unable to change the initial LCR
	 * settings, so set a sane 8N1 default.
	 */
	priv->lcr = ch341_can_LCR_ENABLE_RX | ch341_can_LCR_ENABLE_TX | ch341_can_LCR_CS8;

	r = ch341_can_configure(port->serial->dev, priv);
	if (r < 0)
		goto error;

	usb_set_serial_port_data(port, priv);
	return ch341_can_net_attach(port, priv);

error:	kfree(priv);
	return r;
}

static int ch341_can_port_remove(struct usb_serial_port *port)
{
	struct ch341_can_private *priv;

	priv = usb_get_serial_port_data(port);

	unregister_netdev(priv->can->netdev);
	free_candev(priv->can->netdev);
	
	kfree(priv);


	return 0;
}

static int ch341_can_carrier_raised(struct usb_serial_port *port)
{
	struct ch341_can_private *priv = usb_get_serial_port_data(port);
	if (priv->msr & ch341_can_BIT_DCD)
		return 1;
	return 0;
}

static void ch341_can_dtr_rts(struct usb_serial_port *port, int on)
{
	struct ch341_can_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;

	/* drop DTR and RTS */
	spin_lock_irqsave(&priv->lock, flags);
	if (on)
		priv->mcr |= ch341_can_BIT_RTS | ch341_can_BIT_DTR;
	else
		priv->mcr &= ~(ch341_can_BIT_RTS | ch341_can_BIT_DTR);
	spin_unlock_irqrestore(&priv->lock, flags);
	ch341_can_set_handshake(port->serial->dev, priv->mcr);
}

static void ch341_can_close(struct usb_serial_port *port)
{
	usb_serial_generic_close(port);
	usb_kill_urb(port->interrupt_in_urb);
}

/* open this device, set default parameters */
static int ch341_can_open(struct tty_struct *tty, struct usb_serial_port *port)
{
	struct ch341_can_private *priv = usb_get_serial_port_data(port);
	int r;

	if (tty)
		ch341_can_set_termios(tty, port, NULL);

	dev_dbg(&port->dev, "%s - submitting interrupt urb\n", __func__);
	r = usb_submit_urb(port->interrupt_in_urb, GFP_KERNEL);
	if (r) {
		dev_err(&port->dev, "%s - failed to submit interrupt urb: %d\n",
			__func__, r);
		return r;
	}

	r = ch341_can_get_status(port->serial->dev, priv);
	if (r < 0) {
		dev_err(&port->dev, "failed to read modem status: %d\n", r);
		goto err_kill_interrupt_urb;
	}

	r = usb_serial_generic_open(tty, port);
	if (r)
		goto err_kill_interrupt_urb;

	return 0;

err_kill_interrupt_urb:
	usb_kill_urb(port->interrupt_in_urb);

	return r;
}

/* Old_termios contains the original termios settings and
 * tty->termios contains the new setting to be used.
 */
static void ch341_can_set_termios(struct tty_struct *tty,
		struct usb_serial_port *port, struct ktermios *old_termios)
{
	struct ch341_can_private *priv = usb_get_serial_port_data(port);
	unsigned baud_rate;
	unsigned long flags;
	u8 lcr;
	int r;

	/* redundant changes may cause the chip to lose bytes */
	if (old_termios && !tty_termios_hw_change(&tty->termios, old_termios))
		return;

	baud_rate = tty_get_baud_rate(tty);

	lcr = ch341_can_LCR_ENABLE_RX | ch341_can_LCR_ENABLE_TX;

	switch (C_CSIZE(tty)) {
	case CS5:
		lcr |= ch341_can_LCR_CS5;
		break;
	case CS6:
		lcr |= ch341_can_LCR_CS6;
		break;
	case CS7:
		lcr |= ch341_can_LCR_CS7;
		break;
	case CS8:
		lcr |= ch341_can_LCR_CS8;
		break;
	}

	if (C_PARENB(tty)) {
		lcr |= ch341_can_LCR_ENABLE_PAR;
		if (C_PARODD(tty) == 0)
			lcr |= ch341_can_LCR_PAR_EVEN;
		if (C_CMSPAR(tty))
			lcr |= ch341_can_LCR_MARK_SPACE;
	}

	if (C_CSTOPB(tty))
		lcr |= ch341_can_LCR_STOP_BITS_2;

	if (baud_rate) {
		priv->baud_rate = baud_rate;

		r = ch341_can_set_baudrate_lcr(port->serial->dev, priv, lcr);
		if (r < 0 && old_termios) {
			priv->baud_rate = tty_termios_baud_rate(old_termios);
			tty_termios_copy_hw(&tty->termios, old_termios);
		} else if (r == 0) {
			priv->lcr = lcr;
		}
	}

	spin_lock_irqsave(&priv->lock, flags);
	if (C_BAUD(tty) == B0)
		priv->mcr &= ~(ch341_can_BIT_DTR | ch341_can_BIT_RTS);
	else if (old_termios && (old_termios->c_cflag & CBAUD) == B0)
		priv->mcr |= (ch341_can_BIT_DTR | ch341_can_BIT_RTS);
	spin_unlock_irqrestore(&priv->lock, flags);

	ch341_can_set_handshake(port->serial->dev, priv->mcr);
}

static void ch341_can_break_ctl(struct tty_struct *tty, int break_state)
{
	const uint16_t ch341_can_break_reg =
			((uint16_t) ch341_can_REG_LCR << 8) | ch341_can_REG_BREAK;
	struct usb_serial_port *port = tty->driver_data;
	int r;
	uint16_t reg_contents;
	uint8_t *break_reg;

	break_reg = kmalloc(2, GFP_KERNEL);
	if (!break_reg)
		return;

	r = ch341_can_control_in(port->serial->dev, ch341_can_REQ_READ_REG,
			ch341_can_break_reg, 0, break_reg, 2);
	if (r < 0) {
		dev_err(&port->dev, "%s - USB control read error (%d)\n",
				__func__, r);
		goto out;
	}
	dev_dbg(&port->dev, "%s - initial ch341 break register contents - reg1: %x, reg2: %x\n",
		__func__, break_reg[0], break_reg[1]);
	if (break_state != 0) {
		dev_dbg(&port->dev, "%s - Enter break state requested\n", __func__);
		break_reg[0] &= ~ch341_can_NBREAK_BITS;
		break_reg[1] &= ~ch341_can_LCR_ENABLE_TX;
	} else {
		dev_dbg(&port->dev, "%s - Leave break state requested\n", __func__);
		break_reg[0] |= ch341_can_NBREAK_BITS;
		break_reg[1] |= ch341_can_LCR_ENABLE_TX;
	}
	dev_dbg(&port->dev, "%s - New ch341 break register contents - reg1: %x, reg2: %x\n",
		__func__, break_reg[0], break_reg[1]);
	reg_contents = get_unaligned_le16(break_reg);
	r = ch341_can_control_out(port->serial->dev, ch341_can_REQ_WRITE_REG,
			ch341_can_break_reg, reg_contents);
	if (r < 0)
		dev_err(&port->dev, "%s - USB control write error (%d)\n",
				__func__, r);
out:
	kfree(break_reg);
}

static int ch341_can_tiocmset(struct tty_struct *tty,
			  unsigned int set, unsigned int clear)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ch341_can_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;
	u8 control;

	spin_lock_irqsave(&priv->lock, flags);
	if (set & TIOCM_RTS)
		priv->mcr |= ch341_can_BIT_RTS;
	if (set & TIOCM_DTR)
		priv->mcr |= ch341_can_BIT_DTR;
	if (clear & TIOCM_RTS)
		priv->mcr &= ~ch341_can_BIT_RTS;
	if (clear & TIOCM_DTR)
		priv->mcr &= ~ch341_can_BIT_DTR;
	control = priv->mcr;
	spin_unlock_irqrestore(&priv->lock, flags);

	return ch341_can_set_handshake(port->serial->dev, control);
}

static void ch341_can_update_status(struct usb_serial_port *port,
					unsigned char *data, size_t len)
{
	struct ch341_can_private *priv = usb_get_serial_port_data(port);
	struct tty_struct *tty;
	unsigned long flags;
	u8 status;
	u8 delta;

	if (len < 4)
		return;

	status = ~data[2] & ch341_can_BITS_MODEM_STAT;

	spin_lock_irqsave(&priv->lock, flags);
	delta = status ^ priv->msr;
	priv->msr = status;
	spin_unlock_irqrestore(&priv->lock, flags);

	if (data[1] & ch341_can_MULT_STAT)
		dev_dbg(&port->dev, "%s - multiple status change\n", __func__);

	if (!delta)
		return;

	if (delta & ch341_can_BIT_CTS)
		port->icount.cts++;
	if (delta & ch341_can_BIT_DSR)
		port->icount.dsr++;
	if (delta & ch341_can_BIT_RI)
		port->icount.rng++;
	if (delta & ch341_can_BIT_DCD) {
		port->icount.dcd++;
		tty = tty_port_tty_get(&port->port);
		if (tty) {
			usb_serial_handle_dcd_change(port, tty,
						status & ch341_can_BIT_DCD);
			tty_kref_put(tty);
		}
	}

	wake_up_interruptible(&port->port.delta_msr_wait);
}

static void ch341_can_read_int_callback(struct urb *urb)
{
	struct usb_serial_port *port = urb->context;
	unsigned char *data = urb->transfer_buffer;
	unsigned int len = urb->actual_length;
	int status;

	switch (urb->status) {
	case 0:
		/* success */
		break;
	case -ECONNRESET:
	case -ENOENT:
	case -ESHUTDOWN:
		/* this urb is terminated, clean up */
		dev_dbg(&urb->dev->dev, "%s - urb shutting down: %d\n",
			__func__, urb->status);
		return;
	default:
		dev_dbg(&urb->dev->dev, "%s - nonzero urb status: %d\n",
			__func__, urb->status);
		goto exit;
	}

	usb_serial_debug_data(&port->dev, __func__, len, data);
	ch341_can_update_status(port, data, len);
exit:
	status = usb_submit_urb(urb, GFP_ATOMIC);
	if (status) {
		dev_err(&urb->dev->dev, "%s - usb_submit_urb failed: %d\n",
			__func__, status);
	}
}

static int ch341_can_tiocmget(struct tty_struct *tty)
{
	struct usb_serial_port *port = tty->driver_data;
	struct ch341_can_private *priv = usb_get_serial_port_data(port);
	unsigned long flags;
	u8 mcr;
	u8 status;
	unsigned int result;

	spin_lock_irqsave(&priv->lock, flags);
	mcr = priv->mcr;
	status = priv->msr;
	spin_unlock_irqrestore(&priv->lock, flags);

	result = ((mcr & ch341_can_BIT_DTR)		? TIOCM_DTR : 0)
		  | ((mcr & ch341_can_BIT_RTS)	? TIOCM_RTS : 0)
		  | ((status & ch341_can_BIT_CTS)	? TIOCM_CTS : 0)
		  | ((status & ch341_can_BIT_DSR)	? TIOCM_DSR : 0)
		  | ((status & ch341_can_BIT_RI)	? TIOCM_RI  : 0)
		  | ((status & ch341_can_BIT_DCD)	? TIOCM_CD  : 0);

	dev_dbg(&port->dev, "%s - result = %x\n", __func__, result);

	return result;
}

static int ch341_can_reset_resume(struct usb_serial *serial)
{
	struct usb_serial_port *port = serial->port[0];
	struct ch341_can_private *priv;
	int ret;

	priv = usb_get_serial_port_data(port);
	if (!priv)
		return 0;

	/* reconfigure ch341 serial port after bus-reset */
	ch341_can_configure(serial->dev, priv);

	if (tty_port_initialized(&port->port)) {
		ret = usb_submit_urb(port->interrupt_in_urb, GFP_NOIO);
		if (ret) {
			dev_err(&port->dev, "failed to submit interrupt urb: %d\n",
				ret);
			return ret;
		}

		ret = ch341_can_get_status(port->serial->dev, priv);
		if (ret < 0) {
			dev_err(&port->dev, "failed to read modem status: %d\n",
				ret);
		}
	}

	return usb_serial_generic_resume(serial);
}




static struct usb_serial_driver ch341_can_device = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "ch341-can",
	},
	.id_table          = id_table,
	.num_ports         = 1,
	.open              = ch341_can_open,
	.dtr_rts	   	   = ch341_can_dtr_rts,
	.carrier_raised	   = ch341_can_carrier_raised,
	.close             = ch341_can_close,
	.set_termios       = ch341_can_set_termios,
	.break_ctl         = ch341_can_break_ctl,
	.tiocmget          = ch341_can_tiocmget,
	.tiocmset          = ch341_can_tiocmset,
	.tiocmiwait        = usb_serial_generic_tiocmiwait,
	.read_int_callback = ch341_can_read_int_callback,
	.port_probe        = ch341_can_port_probe,
	.port_remove       = ch341_can_port_remove,
	.reset_resume      = ch341_can_reset_resume,
};

static struct usb_serial_driver * const serial_drivers[] = {
	&ch341_can_device, NULL
};

module_usb_serial_driver(serial_drivers, id_table);

MODULE_LICENSE("GPL v2");

/**************************************************************
**************************************************************
******************** CAN NET **********************************
***************************************************************
**************************************************************/

static const struct can_bittiming_const ch341_bittiming_const = {
	.name = "ch341-can",
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

static int ch341_can_net_do_set_mode(struct net_device *dev, enum can_mode mode){
	int ret;
	struct ch341_can_struct *sl = netdev_priv(dev);

	switch (mode) {
	case CAN_MODE_START:
		sl->can.state = CAN_STATE_ERROR_ACTIVE;
		return ret;
	default:
		return -EOPNOTSUPP;
	}
}

static int ch341_can_net_open_uart(struct ch341_can_struct *priv) {
	
	int fd;
	char name[50];
	
	sprintf(name, "/dev/ttyUSB%i", priv->port->minor);
	printk(name);
	printk("\n");

	return -1;
#if 0
	fd = open(ttypath, O_RDWR | O_NONBLOCK | O_NOCTTY);
	if (fd < 0) {
		syslogger(LOG_NOTICE, "failed to open TTY device %s\n", ttypath);
		perror(ttypath);
		exit(EXIT_FAILURE);
	}

	if (ioctl(fd, TCGETS2, &tios) < 0) {
		syslogger(LOG_NOTICE, "ioctl() failed: %s\n", strerror(errno));
		close(fd);
		exit(EXIT_FAILURE);
	}

	tios.c_cflag &= ~CBAUD;
	tios.c_cflag = BOTHER | CS8 | CSTOPB;
	tios.c_iflag = IGNPAR;
	tios.c_oflag = 0;
	tios.c_lflag = 0;
	tios.c_ispeed = (speed_t) uart_speed;
	tios.c_ospeed = (speed_t) uart_speed;

	// Because of a recent change in linux - https://patchwork.kernel.org/patch/9589541/
	// we need to set low latency flag to get proper receive latency
	struct serial_struct snew;
	ioctl (fd, TIOCGSERIAL, &snew);
	snew.flags |= ASYNC_LOW_LATENCY;
	ioctl (fd, TIOCSSERIAL, &snew);

	if (ioctl(fd, TCSETS2, &tios) < 0) {
		syslogger(LOG_NOTICE, "ioctl() failed: %s\n", strerror(errno));
		close(fd);
		exit(EXIT_FAILURE);
	}

	if (command_settings(speed, mode, type, fd) < 0){
		close(fd);
        	exit(EXIT_FAILURE);
	}

	/* set hlcan like discipline on given tty */
	if (ioctl(fd, TIOCSETD, &ldisc) < 0) {
		perror("ioctl TIOCSETD");
		exit(EXIT_FAILURE);
	}
	
	/* retrieve the name of the created CAN netdevice */
	if (ioctl(fd, SIOCGIFNAME, ifr.ifr_name) < 0) {
		perror("ioctl SIOCGIFNAME");
		exit(EXIT_FAILURE);
	}

	/* Change the mode according to the given command line paramter */
	if (ioctl(fd, IO_CTL_MODE, mode) < 0) {
		perror("ioctl mode");
		exit(EXIT_FAILURE);
	}
#endif
}

// /home/me/dev/linux/drivers/net/can/usb/esd_usb2.c
static int ch341_can_net_open(struct net_device *netdev)
{
	struct ch341_can_struct *priv = netdev_priv(netdev);
	int err;

	/* common open */
	err = open_candev(netdev);
	if (err != 0)
		return err;

	/* finally start device */
	err = ch341_can_net_open_uart(priv);
	if (err) {
		netdev_warn(netdev, "couldn't start device: %d\n", err);
		close_candev(netdev);
		return err;
	}

	netif_start_queue(netdev);

	return 0;
}

static netdev_tx_t ch341_can_net_start_xmit(struct sk_buff *skb,
				      struct net_device *netdev)
{
	return -1;
}

static int ch341_can_net_close(struct net_device *netdev)
{
	struct ch341_can_struct *priv;
	priv = netdev_priv(netdev);
	priv->can.state = CAN_STATE_STOPPED;
	netif_stop_queue(netdev);
	close_candev(netdev);
	return 0;
}

static const struct net_device_ops ch341_can_net_netdev_ops = {
	.ndo_open = ch341_can_net_open,
	.ndo_stop = ch341_can_net_close,
	.ndo_start_xmit = ch341_can_net_start_xmit,
	.ndo_change_mtu = can_change_mtu,
};


static int ch341_can_net_attach(struct usb_serial_port *port, struct ch341_can_private *serialPriv){
	struct net_device *netdev;
	struct ch341_can_struct *priv;
	struct device *dev = &port->serial->interface->dev;
	int err = 0;

	netdev = alloc_candev(sizeof(*priv), 1);
	if (!netdev) {
		dev_err(dev, "couldn't alloc candev\n");
		err = -ENOMEM;
		return err;
	}

	priv = netdev_priv(netdev);
	priv->netdev = netdev;
	priv->port = port;
	serialPriv->can = priv;

	priv->can.state = CAN_STATE_STOPPED;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LISTENONLY;
	
	/* todo set this to a propper value */
	priv->can.clock.freq = 3686400000; 
	priv->can.data_bittiming_const = &ch341_bittiming_const;
	priv->can.bittiming.bitrate = 800000;
	priv->can.do_set_mode = ch341_can_net_do_set_mode;
	priv->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_3_SAMPLES | 
		CAN_CTRLMODE_LISTENONLY;
	netdev->netdev_ops = &ch341_can_net_netdev_ops;

	SET_NETDEV_DEV(netdev, dev);
	netdev->dev_id = 0;
	err = register_candev(netdev);
	if (err) {
		dev_err(dev, "couldn't register CAN device: %d\n", err);
		free_candev(netdev);
		err = -ENOMEM;
		return err;
	}

	netdev_info(netdev, "device %s registered\n", netdev->name);
	return err;
}

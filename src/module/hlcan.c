/*
 * hl340.c - CAN driver interface for 
 *
 * This file is derived from linux/drivers/net/can/slcan.c
 *
 *
 * slip.c Authors  : Laurence Culhane <loz@holmes.demon.co.uk>
 *                   Fred N. van Kempen <waltje@uwalt.nl.mugnet.org>
 * slcan.c Author  : Oliver Hartkopp <socketcan@hartkopp.net>
 * hl340.c Author  : Alexander Mohr <usbcan@mohr.io.net>
 *
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see http://www.gnu.org/licenses/gpl.html
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>

#include <linux/uaccess.h>
#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/tty.h>
#include <linux/errno.h>
#include <linux/netdevice.h>
#include <linux/skbuff.h>
#include <linux/rtnetlink.h>
#include <linux/if_arp.h>
#include <linux/if_ether.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/can/skb.h>

#include "hlcan.h"

MODULE_ALIAS_LDISC(N_HLCAN);
MODULE_DESCRIPTION("hl340 CAN interface");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Mohr <hlcan@mohr.io>");

static int maxdev = 10;		/* MAX number of SLCAN channels;
				   This can be overridden with
				   insmod slcan.ko maxdev=nnn	*/
module_param(maxdev, int, 0);
MODULE_PARM_DESC(maxdev, "Maximum number of hlcan interfaces");

/* maximum rx buffer len: 20 should be enough as config command is largest cmd*/
#define SLC_MTU (128)
#define DRV_NAME			"hlcan"
#define SLF_INUSE		0		/* Channel in use            */
#define SLF_ERROR		1		/* Parity, etc. error        */
spinlock_t		global_lock;

struct slcan {
	struct can_priv can;
	int magic;
	struct tty_struct	*tty;		/* ptr to TTY structure	     */
	struct net_device	*dev;		/* easy for intr handling    */
	spinlock_t		lock;
	struct work_struct	tx_work;	/* Flushes transmit buffer   */

	/* These are pointers to the malloc()ed frame buffers. */
	unsigned char		rbuff[SLC_MTU];	/* receiver buffer	     */
	int			rcount;         /* received chars counter    */
	int			rexpected;	/* expected chars counter    */
	FRAME_STATE 		rstate; 	/* state of current receive  */
	unsigned char		xbuff[SLC_MTU];	/* transmitter buffer	     */
	unsigned char		*xhead;         /* pointer to next XMIT byte */
	int			xleft;          /* bytes left in XMIT queue  */

	unsigned long		flags;		/* Flag values/ mode etc     */
	int candev_registered;
	int mode;
};

static const struct can_bittiming_const hlcan_bittiming_const = {
	.name = DRV_NAME,
	.tseg1_min = 2,
	.tseg1_max = 16,
	.tseg2_min = 2,
	.tseg2_max = 8,
	.sjw_max = 4,
	.brp_min = 1,
	.brp_max = 64,
	.brp_inc = 1,
};

static struct net_device **slcan_devs;

/*
 * Protocol handling
 */
#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

#define IS_EXT_ID(type)(CHECK_BIT(type, 5))
#define IS_REMOTE(type)(CHECK_BIT(type, 4))
		
/* checks if bit 7 and 6 is set */
#define IS_DATA_PACKAGE(type) ({ \
		((type >> 6) ^ 3) == 0;})

#define GET_FRAME_ID(frame)({ 	   		\
	 IS_EXT_ID(frame[1]) 			\
		?	(frame[5] << 24) |	\
			(frame[4] << 16) |	\
			(frame[3] << 8)  |	\
			(frame[2])		\
		: 	((frame[3] << 8) |	\
			frame[2]);})

#define GET_DLC(type) ({type & 0x0f;})



 /************************************************************************
  *			HL340 ENCAPSULATION FORMAT			 *
  ************************************************************************/

/*
 * A CAN frame has a can_id (11 bit standard frame format OR 29 bit extended
 * frame format) a data length code (can_dlc) which can be from 0 to 8
 * and up to <can_dlc> data bytes as payload.
 * Additionally a CAN frame may become a remote transmission frame if the
 * RTR-bit is set. This causes another ECU to send a CAN frame with the
 * given can_id.
 *
 * The HL340 ASCII representation of these different frame types is:
 * <start> <type> <id> <dlc> <data>* <end>
 *
 * Extended frames (29 bit) are defined by the type byte
 * Type byte is defined as 0xc0 as constant and the following values for type
 * bit 5: frame type
 *  0 = 11 bit frame
 *  1 = 29 bit frame
 * bit 4:
 *  0 = data frame 
 *  1 = remote frame
 * bit 0-3: dlc
 *
 * The <id> is 2 (standard) or 4 (extended) bytes
 * The <dlc> is encoded in 4 bit
 * The <data> has as much data bytes in it as defined dlc
 */

 /************************************************************************
  *			STANDARD SLCAN DECAPSULATION			 *
  ************************************************************************/

static inline void *skb_put_data(struct sk_buff *skb, const void *data,
				 unsigned int len)
{
	void *tmp = skb_put(skb, len);

	memcpy(tmp, data, len);

	return tmp;
}

/* Send one completely decapsulated can_frame to the network layer */
static void slc_bump(struct slcan *sl)
{
	struct sk_buff *skb;
	struct can_frame cf;
	unsigned char data_start = 3;
	/* idx 0 = packet header, skip it */
	unsigned char *cmd = sl->rbuff + 1;
	
	cf.can_dlc = GET_DLC(*cmd);
	cf.can_id = GET_FRAME_ID(sl->rbuff);
	
	if (IS_REMOTE(*cmd)){
		cf.can_id |= CAN_RTR_FLAG;
	}

	if (IS_EXT_ID(*cmd)) {
		cf.can_id |= CAN_EFF_FLAG;
		data_start = 5;
	}

	*(u64 *) (&cf.data) = 0; /* clear payload */
	/* RTR frames may have a dlc > 0 but they never have any data bytes */
	if (!(cf.can_id & CAN_RTR_FLAG)) {
		memcpy(cf.data, 
			cmd + data_start,
			cf.can_dlc);
	}

	skb = dev_alloc_skb(sizeof(struct can_frame) +
			    sizeof(struct can_skb_priv));
	if (!skb)
		return;

	skb->dev = sl->dev;
	skb->protocol = htons(ETH_P_CAN);
	skb->pkt_type = PACKET_BROADCAST;
	skb->ip_summed = CHECKSUM_UNNECESSARY;

	can_skb_reserve(skb);
	can_skb_prv(skb)->ifindex = sl->dev->ifindex;
	can_skb_prv(skb)->skbcnt = 0;

	skb_put_data(skb, &cf, sizeof(struct can_frame));

	sl->dev->stats.rx_packets++;
	sl->dev->stats.rx_bytes += cf.can_dlc;
	netif_rx(skb);
}

/* get the state of the current receive transmission */
static void hlcan_update_rstate(struct slcan *sl)
{
	if (sl->rcount > 0) {
		if (sl->rbuff[0] != HLCAN_PACKET_START) {
			/* Need to sync on 0xaa at start of frames, so just skip. */
			sl->rstate = MISSED_HEADER;
			 return;
		}
	}

	if (sl->rcount < 2) {
		sl->rstate = RECEIVING;
		return;
	}

	if (sl->rbuff[1] == HLCAN_CFG_PACKAGE_TYPE) {
		if (sl->rcount >= HLCAN_CFG_PACKAGE_LEN) {
			/* will be handled by userspace tool */
			sl->rstate = COMPLETE;
		} else {
			sl->rstate = RECEIVING;
		}
	} else if (IS_DATA_PACKAGE(sl->rbuff[1])) {
		/* Data frame... */
		int ext_id = IS_EXT_ID(sl->rbuff[1]) ? 4 : 2;
		int dlc = GET_DLC(sl->rbuff[1]);
		
		sl->rexpected =	1 + // HLCAN_PACKET_START
			1 + // type byte
			ext_id +
			dlc + 
			1; // HLCAN_PACKET_END
		
		if (sl->rcount >= sl->rexpected){
			sl->rstate = COMPLETE;
		} else {
			sl->rstate = RECEIVING;
		}
	} else {
		/* Unhandled frame type. */
		sl->rstate = NONE;
	}
}

/* parse tty input stream */
static void slcan_unesc(struct slcan *sl, unsigned char s)
{
	if (test_and_clear_bit(SLF_ERROR, &sl->flags)) {
		return;
	}

	if (sl->rcount > SLC_MTU) {
		sl->dev->stats.rx_over_errors++;
		set_bit(SLF_ERROR, &sl->flags);
		return;
	}

	sl->rbuff[sl->rcount++] = s;

	/* Only check state again after we received enough data */
	if (RECEIVING == sl->rstate
		&& sl->rexpected > 0
		&& sl->rcount < sl->rexpected) {
		return;
	}
	
	hlcan_update_rstate(sl);
	switch(sl->rstate) {
		case COMPLETE:
			if (IS_DATA_PACKAGE(sl->rbuff[1])) {
				slc_bump(sl);
			}
			sl->rexpected = 0;
			/* fall through */
		case MISSED_HEADER:
			sl->rcount = 0;
			break;
		default: break;
	}
}

/************************************************************************
 *			STANDARD SLCAN ENCAPSULATION			*
 ************************************************************************/

/* Encapsulate one can_frame and stuff into a TTY queue. */
static void slc_encaps(struct slcan *sl, struct can_frame *cf)
{
	int actual, i;
	unsigned char *pos;
	u32 id;


	pos = sl->xbuff;
	*pos++ = HLCAN_PACKET_START;
	
	*pos = HLCAN_FRAME_PREFIX;
	*pos |= cf->can_dlc;
	if (cf->can_id & CAN_RTR_FLAG) {
		*pos |= HLCAN_FLAG_RTR;
	}

	/* setup the frame id id */
	/* mask the upper 3 bits because they are used for flags */
	id = cf->can_id & 0x1FFFFFFF;
	if (cf->can_id & CAN_EFF_FLAG) {
		*pos++ &= HLCAN_FLAG_ID_EXT;
		*pos++ = (unsigned char) (id & 0xFF);
		*pos++ = (unsigned char) ((id >> 8) & 0xFF);
		*pos++ = (unsigned char) ((id >> 16) & 0xFF);
		*pos++ = (unsigned char) ((id >> 24) & 0xFF);
	} else {
 		++pos;
		*pos++ = (unsigned char) (id & 0xFF);
		*pos++ = (unsigned char) ((id >> 8) & 0xFF);
	}

	/* RTR frames may have a dlc > 0 but they never have any data bytes */
	if (!(cf->can_id & CAN_RTR_FLAG)) {
		for (i = 0; i < cf->can_dlc; i++)
			*pos++ = cf->data[i];
	}

	*pos++ = HLCAN_PACKET_END;

	/* Order of next two lines is *very* important.
	 * When we are sending a little amount of data,
	 * the transfer may be completed inside the ops->write()
	 * routine, because it's running with interrupts enabled.
	 * In this case we *never* got WRITE_WAKEUP event,
	 * if we did not request it before write operation.
	 *       14 Oct 1994  Dmitry Gorodchanin.
	 */
	set_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	actual = sl->tty->ops->write(sl->tty, sl->xbuff, pos - sl->xbuff);
	sl->xleft = (pos - sl->xbuff) - actual;
	sl->xhead = sl->xbuff + actual;
	sl->dev->stats.tx_bytes += cf->can_dlc;
}

/* Write out any remaining transmit buffer. Scheduled when tty is writable */
static void slcan_transmit(struct work_struct *work)
{
	struct slcan *sl = container_of(work, struct slcan, tx_work);
	int actual;

	spin_lock_bh(&sl->lock);
	/* First make sure we're connected. */
	if (!sl->tty || sl->magic != HLCAN_MAGIC || !netif_running(sl->dev)) {
		spin_unlock_bh(&sl->lock);
		return;
	}

	if (sl->xleft <= 0)  {
		/* Now serial buffer is almost free & we can start
		 * transmission of another packet */
		sl->dev->stats.tx_packets++;
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
		spin_unlock_bh(&sl->lock);
		netif_wake_queue(sl->dev);
		return;
	}

	actual = sl->tty->ops->write(sl->tty, sl->xhead, sl->xleft);
	sl->xleft -= actual;
	sl->xhead += actual;
	spin_unlock_bh(&sl->lock);
}

/*
 * Called by the driver when there's room for more data.
 * Schedule the transmit.
 */
static void slcan_write_wakeup(struct tty_struct *tty)
{
	struct slcan *sl = tty->disc_data;

	schedule_work(&sl->tx_work);
}

/* Send a can_frame to a TTY queue. */
static netdev_tx_t slc_xmit(struct sk_buff *skb, struct net_device *dev)
{
	struct slcan *sl = netdev_priv(dev);

	if (skb->len != CAN_MTU)
		goto out;

	spin_lock(&sl->lock);
	if (!netif_running(dev))  {
		spin_unlock(&sl->lock);
		printk(KERN_WARNING "%s: xmit: iface is down\n", dev->name);
		goto out;
	}
	if (sl->tty == NULL) {
		spin_unlock(&sl->lock);
		goto out;
	}

	netif_stop_queue(sl->dev);
	slc_encaps(sl, (struct can_frame *) skb->data); /* encaps & send */
	spin_unlock(&sl->lock);

out:
	kfree_skb(skb);
	return NETDEV_TX_OK;
}


/******************************************
 *   Routines looking at netdevice side.
 ******************************************/

/* Netdevice UP -> DOWN routine */
static int slc_close(struct net_device *dev)
{
	struct slcan *sl = netdev_priv(dev);

	spin_lock_bh(&sl->lock);
	if (sl->tty) {
		/* TTY discipline is running. */
		clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
	}
	netif_stop_queue(dev);
	sl->rcount   = 0;
	sl->xleft    = 0;
	spin_unlock_bh(&sl->lock);

	sl->can.state = CAN_STATE_STOPPED;
	close_candev(dev);

	return 0;
}

/* Netdevice DOWN -> UP routine */
static int slc_open(struct net_device *dev)
{
	int ret;
	struct slcan *sl = netdev_priv(dev);

	if (sl->tty == NULL)
		return -ENODEV;

	/* Common open */
	ret = open_candev(dev);
	if (ret) {
		return ret;
	}

	sl->flags &= (1 << SLF_INUSE);
	sl->can.state = CAN_STATE_ERROR_ACTIVE;
	netif_start_queue(dev);
	return 0;
}

static const struct net_device_ops slc_netdev_ops = {
	.ndo_open               = slc_open,
	.ndo_stop               = slc_close,
	.ndo_start_xmit         = slc_xmit,
	.ndo_change_mtu         = can_change_mtu,
};


/******************************************
  Routines looking at TTY side.
 ******************************************/

/*
 * Handle the 'receiver data ready' interrupt.
 * This function is called by the 'tty_io' module in the kernel when
 * a block of SLCAN data has been received, which can now be decapsulated
 * and sent on to some IP layer for further processing. This will not
 * be re-entered while running but other ldisc functions may be called
 * in parallel
 */
static void slcan_receive_buf(struct tty_struct *tty,
			      const unsigned char *cp, char *fp, int count)
{
	struct slcan *sl = (struct slcan *) tty->disc_data;

	if (!sl || sl->magic != HLCAN_MAGIC || !netif_running(sl->dev)){
		printk("hlcan: Serial device not ready\n");
		return;
	}

	/* Read the characters out of the buffer */
	while (count--) {
		if (fp && *fp++) {
			if (!test_and_set_bit(SLF_ERROR, &sl->flags))
				sl->dev->stats.rx_errors++;
			cp++;
			continue;
		}
		slcan_unesc(sl, *cp++);
	}
}

/************************************
 *  slcan_open helper routines.
 ************************************/

/* Collect hanged up channels */
static void slc_sync(void)
{
	int i;
	struct net_device *dev;
	struct slcan	  *sl;

	for (i = 0; i < maxdev; i++) {
		dev = slcan_devs[i];
		if (dev == NULL)
			break;

		sl = netdev_priv(dev);
		if (sl->tty)
			continue;
		if (dev->flags & IFF_UP)
			dev_close(dev);
	}
}


static int hlcan_do_set_mode(struct net_device *dev, enum can_mode mode){
	int ret;
	struct slcan *sl = netdev_priv(dev);

	switch (mode) {
	case CAN_MODE_START:
		sl->can.state = CAN_STATE_ERROR_ACTIVE;
		return ret;
	default:
		return -EOPNOTSUPP;
	}
}


/* Find a free SLCAN channel, and link in this `tty' line. */
static struct slcan *slc_alloc(void)
{
	int i;
	char name[IFNAMSIZ];
	struct net_device *dev = NULL;
	struct slcan       *sl;

	for (i = 0; i < maxdev; i++) {
		dev = slcan_devs[i];
		if (dev == NULL)
			break;
	}

	/* Sorry, too many, all slots in use */
	if (i >= maxdev)
		return NULL;

	sprintf(name, "hlcan%d", i);
	dev = alloc_candev(sizeof(*sl), 1);
	if (!dev)
		return NULL;

	sl = netdev_priv(dev);
	
	dev->netdev_ops = &slc_netdev_ops;
	// Device does NOT echo on itself
	// dev->flags |= IFF_ECHO;

	/* this does not actually matter when we use the serial port */
	/* todo set this to a propper value */
	sl->can.clock.freq = 3686400000; 
	sl->can.data_bittiming_const = &hlcan_bittiming_const;
	sl->can.bittiming.bitrate = 800000;
	sl->can.do_set_mode = hlcan_do_set_mode;
	sl->can.ctrlmode_supported = CAN_CTRLMODE_LOOPBACK |
		CAN_CTRLMODE_3_SAMPLES | 
		CAN_CTRLMODE_FD |
		CAN_CTRLMODE_LISTENONLY;

	dev->base_addr = i;

	/* Initialize channel control data */
	sl->magic = HLCAN_MAGIC;
	sl->rstate = NONE;
	sl->dev	= dev;
	sl->mode = 0;
	spin_lock_init(&sl->lock);
	INIT_WORK(&sl->tx_work, slcan_transmit);
	slcan_devs[i] = dev;

	return sl;
}

/*
 * Open the high-level part of the SLCAN channel.
 * This function is called by the TTY module when the
 * SLCAN line discipline is called for.  Because we are
 * sure the tty line exists, we only have to link it to
 * a free SLCAN channel...
 *
 * Called in process context serialized from other ldisc calls.
 */
static int slcan_open(struct tty_struct *tty)
{
	struct slcan *sl;
	int err;

	if (!capable(CAP_NET_ADMIN))
		return -EPERM;

	if (tty->ops->write == NULL)
		return -EOPNOTSUPP;

	/* sync concurrent opens on global lock */
	spin_lock_bh(&global_lock);

	/* Collect hanged up channels. */
	slc_sync();

	sl = tty->disc_data;

	err = -EEXIST;
	/* First make sure we're not already connected. */
	if (sl && sl->magic == HLCAN_MAGIC)
		goto err_exit;

	/* OK.  Find a free SLCAN channel to use. */
	err = -ENFILE;
	sl = slc_alloc();
	
	SET_NETDEV_DEV(sl->dev, tty->dev);
	if (sl == NULL) {
		err = -ENOMEM;
		goto err_exit;
	}

	sl->tty = tty;
	tty->disc_data = sl;

	if (!test_bit(SLF_INUSE, &sl->flags)) {
		/* Perform the low-level SLCAN initialization. */
		sl->rcount   = 0;
		sl->xleft    = 0;

		set_bit(SLF_INUSE, &sl->flags);

		err = register_candev(sl->dev);
		if (err)
			goto err_free_chan;

		sl->candev_registered = 1;
	}

	/* Done.  We have linked the TTY line to a channel. */
	spin_unlock_bh(&global_lock);
	tty->receive_room = 65536;	/* We don't flow control */

	/* TTY layer expects 0 on success */
	return 0;

err_free_chan:
	sl->tty = NULL;
	tty->disc_data = NULL;
	clear_bit(SLF_INUSE, &sl->flags);

err_exit:
	spin_unlock_bh(&global_lock);
	/* Count references from TTY module */
	return err;
}

/*
 * Close down a SLCAN channel.
 * This means flushing out any pending queues, and then returning. This
 * call is serialized against other ldisc functions.
 *
 * We also use this method for a hangup event.
 */

static void slcan_close(struct tty_struct *tty)
{
	struct slcan *sl = (struct slcan *) tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != HLCAN_MAGIC || sl->tty != tty)
		return;

	spin_lock_bh(&sl->lock);
	tty->disc_data = NULL;
	sl->tty = NULL;
	spin_unlock_bh(&sl->lock);

	flush_work(&sl->tx_work);

	/* Flush network side */
	unregister_candev(sl->dev);
	sl->candev_registered = 0;
	/* This will complete via sl_free_netdev */
}

static int slcan_hangup(struct tty_struct *tty)
{
	slcan_close(tty);
	return 0;
}

/* Perform I/O control on an active SLCAN channel. */
static int slcan_ioctl(struct tty_struct *tty, struct file *file,
		       unsigned int cmd, unsigned long arg)
{
	struct slcan *sl = (struct slcan *) tty->disc_data;

	/* First make sure we're connected. */
	if (!sl || sl->magic != HLCAN_MAGIC)
		return -EINVAL;

	switch (cmd) {
	case SIOCSIFHWADDR:
		return -EINVAL;

	case IO_CTL_MODE:
		sl->mode = arg;
		printk("hlcan: new device mode %i\n", sl->mode);
		if (sl->mode == 1 || sl->mode == 3){
			sl->dev->flags |= IFF_ECHO;
		}
			
		return 0;

	default:
		return tty_mode_ioctl(tty, file, cmd, arg);
	}
}



static struct tty_ldisc_ops slc_ldisc = {
	.owner		= THIS_MODULE,
	.magic		= TTY_LDISC_MAGIC,
	.name		= "hlcan",
	.open		= slcan_open,
	.close		= slcan_close,
	.hangup		= slcan_hangup,
	.ioctl		= slcan_ioctl,
	.receive_buf	= slcan_receive_buf,
	.write_wakeup	= slcan_write_wakeup,
};

static int __init slcan_init(void)
{
	int status;

	if (maxdev < 4)
		maxdev = 4; /* Sanity */

	pr_info("hlcan: QinHeng serial line CAN interface driver\n");
	pr_info("hlcan: %d dynamic interface channels.\n", maxdev);

	slcan_devs = kcalloc(maxdev, sizeof(struct net_device *), GFP_KERNEL);
	if (!slcan_devs)
		return -ENOMEM;

	/* Fill in our line protocol discipline, and register it */
	status = tty_register_ldisc(N_HLCAN, &slc_ldisc);
	if (status)  {
		printk(KERN_ERR "hlcan: can't register line discipline\n");
		kfree(slcan_devs);
	}
	spin_lock_init(&global_lock);

	return status;
}

static void __exit slcan_exit(void)
{
	unsigned long timeout = jiffies + HZ;
	struct net_device *dev;
	struct slcan *sl;
	int busy = 0;
	int i;

	if (slcan_devs == NULL)
		return;

	/*
	 * First of all: check for active disciplines and hangup them.
	 */
	do {
		if (busy)
			msleep_interruptible(100);

		busy = 0;
		for (i = 0; i < maxdev; i++) {
			dev = slcan_devs[i];
			if (!dev)
				continue;
			sl = netdev_priv(dev);
			spin_lock_bh(&sl->lock);
			if (sl->tty) {
				busy++;
				tty_hangup(sl->tty);
			}
			spin_unlock_bh(&sl->lock);
		}
	} while (busy && time_before(jiffies, timeout));

	/* FIXME: hangup is async so we should wait when doing this second
	   phase */

	for (i = 0; i < maxdev; i++) {
		dev = slcan_devs[i];
		if (!dev)
			continue;

		slcan_devs[i] = NULL;

		sl = netdev_priv(dev);
		if (sl->tty) {
			printk(KERN_ERR "%s: tty discipline still running\n",
			       dev->name);
		}

		if (sl->candev_registered)
			unregister_candev(dev);
	}

	kfree(slcan_devs);
	slcan_devs = NULL;

	i = tty_unregister_ldisc(N_HLCAN);
	if (i)
		printk(KERN_ERR "hlcan: can't unregister ldisc (err %d)\n", i);
}

module_init(slcan_init);
module_exit(slcan_exit);

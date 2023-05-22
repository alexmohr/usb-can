#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/fs.h>
#include <linux/tty.h>
#include <linux/serial.h>

#define DRIVER_NAME "ch341_uart"
#define MAX_DEVICES 4

static char *device_names[MAX_DEVICES] = {
    "/dev/ttyUSB0",
    "/dev/ttyUSB1",
    "/dev/ttyUSB2",
    "/dev/ttyUSB3"
};

static int num_devices = 1;
module_param(num_devices, int, S_IRUGO);
MODULE_PARM_DESC(num_devices, "Number of CAN devices");

module_param_array(device_names, charp, NULL, S_IRUGO);
MODULE_PARM_DESC(device_names, "CAN device names");

static int uart_baudrates[MAX_DEVICES] = {
    9600,
    115200,
    57600,
    19200
};
module_param_array(uart_baudrates, int, NULL, S_IRUGO);
MODULE_PARM_DESC(uart_baudrates, "UART baud rates");

struct hl340_can_device {
  struct platform_device *pdev;
  char *device_name;
  int uart_baudrate;
  struct net_device *netdev;
  struct can_priv can;
  struct file *serial_filp;
};

// Function to send CAN data
static int hl340_can_send(struct hl340_can_device *dev, struct can_frame *frame)
{
  return 0;
}

// Function to receive CAN data
static int hl340_can_receive(struct hl340_can_device *dev, struct can_frame *frame)
{
  return 0;
}

// Function to open the serial device
static int hl340_can_open_serial(struct hl340_can_device *dev)
{
  struct tty_struct *tty;
  struct file *filp;
  int err;

  // Open the serial device
  filp = filp_open(dev->device_name, O_RDWR | O_NOCTTY | O_NONBLOCK, 0);
  if (IS_ERR(filp)) {
    err = PTR_ERR(filp);
    dev_err(&dev->pdev->dev, "Failed to open serial device %s: %d\n", dev->device_name, err);
    return err;
  }

  // Get the tty structure associated with the file
  tty = filp->private_data;

  // Set the serial port configuration
  if (tty && tty->ops && tty->ops->ioctl) {
    struct serial_struct ser;

    ser.baud_base = dev->uart_baudrate;
    ser.close_delay = 0;
    ser.closing_wait = ASYNC_CLOSING_WAIT_NONE;
    ser.custom_divisor = 0;
    ser.flags = ASYNC_SKIP_TEST | ASYNC_LOW_LATENCY;
    ser.xmit_fifo_size = 0;
//    ser.real_num_bits = 8;
    ser.irq = 0;

    // Call the TTY ioctl to set the serial port configuration
    err = tty->ops->ioctl(tty, TIOCSSERIAL, &ser);
    if (err) {
      dev_err(&dev->pdev->dev, "Failed to set serial port configuration: %d\n", err);
      filp_close(filp, NULL);
      return err;
    }
  }

  // Save the file pointer to the hl340_can_device structure for future reference
  dev->serial_filp = filp;

  return 0;
}


/******************************************
 *   Routines looking at netdevice side.
 ******************************************/


/* Send a can_frame to a TTY queue. */
static netdev_tx_t hlcan_netdev_xmit(struct sk_buff *skb, struct net_device *dev)
{
//  struct slcan *sl = netdev_priv(dev);
//
//  if (skb->len != CAN_MTU)
//    goto out;
//
//  spin_lock(&sl->lock);
//  if (!netif_running(dev))  {
//    spin_unlock(&sl->lock);
//    printk(KERN_WARNING "%s: xmit: iface is down\n", dev->name);
//    goto out;
//  }
//  if (sl->tty == NULL) {
//    spin_unlock(&sl->lock);
//    goto out;
//  }
//
//  netif_stop_queue(sl->dev);
//  slc_encaps(sl, (struct can_frame *) skb->data); /* encaps & send */
//  spin_unlock(&sl->lock);
//
//out:
//  kfree_skb(skb);
  return NETDEV_TX_OK;
}

/* Netdevice UP -> DOWN routine */
static int hlcan_netdev_close(struct net_device *dev)
{
//  struct slcan *sl = netdev_priv(dev);
//
//  spin_lock_bh(&sl->lock);
//  if (sl->tty) {
//    /* TTY discipline is running. */
//    clear_bit(TTY_DO_WRITE_WAKEUP, &sl->tty->flags);
//  }
//  netif_stop_queue(dev);
//  sl->rcount   = 0;
//  sl->xleft    = 0;
//  spin_unlock_bh(&sl->lock);
//
//  sl->can.state = CAN_STATE_STOPPED;
//  close_candev(dev);
//
  return 0;
}

/* Netdevice DOWN -> UP routine */
static int hlcan_netdev_open(struct net_device *dev)
{
//  int ret;
//  struct slcan *sl = netdev_priv(dev);
//
//  if (sl->tty == NULL)
//    return -ENODEV;
//
//  /* Common open */
//  ret = open_candev(dev);
//  if (ret) {
//    return ret;
//  }
//
//  sl->flags &= (1 << SLF_INUSE);
//  sl->can.state = CAN_STATE_ERROR_ACTIVE;
  netif_start_queue(dev);
  return 0;
}

static const struct net_device_ops hl340_net_device_ops = {
    .ndo_open               = hlcan_netdev_open,
    .ndo_stop               = hlcan_netdev_close,
    .ndo_start_xmit         = hlcan_netdev_xmit,
    .ndo_change_mtu         = can_change_mtu,
};

static int hl340_can_probe(struct platform_device *pdev)
{
  struct hl340_can_device *hl340_dev;
  struct net_device *netdev;

  printk(KERN_WARNING "probing");

  // Get the device index
  int dev_index = pdev->id;

  // Check if the device index is within the valid range
  if (dev_index < 0 || dev_index >= num_devices) {
    dev_err(&pdev->dev, "Invalid device index\n");
    return -EINVAL;
  }

  // Allocate memory for the hl340_can_device structure
  hl340_dev = devm_kzalloc(&pdev->dev, sizeof(struct hl340_can_device), GFP_KERNEL);
  if (!hl340_dev) {
    dev_err(&pdev->dev, "Memory allocation failed\n");
    return -ENOMEM;
  }

  // Get the network device associated with the platform device
  netdev = platform_get_drvdata(pdev);
  if (!netdev) {
    dev_err(&pdev->dev, "No network device associated with platform device\n");
    return -EINVAL;
  }

  // Store the platform device pointer, device name, and baudrate in the hl340_can_device structure
  hl340_dev->pdev = pdev;
  hl340_dev->device_name = device_names[dev_index];
  hl340_dev->uart_baudrate = uart_baudrates[dev_index];
  hl340_dev->netdev = netdev;
  netdev->netdev_ops = &hl340_net_device_ops;


  // Set the CAN device name
  strcpy(netdev->name, "hl340_can");

  // Open the serial device
  int ret = hl340_can_open_serial(hl340_dev);
  if (ret) {
    dev_err(&pdev->dev, "Failed to open serial device\n");
    return ret;
  }

//  // Set the CAN device capabilities
//  netdev->can.state = CAN_STATE_ERROR_ACTIVE;
//  netdev->can.can_stats = netdev_stats_alloc();

  // Set the CAN device operations
//  netdev->can.do_setsockopt = can_do_setsockopt;
//  netdev->can.do_getsockopt = can_do_getsockopt;

  // todo
//  // Set the send and receive functions
//  netdev->can.ops.send = hl340_can_send;
//  netdev->can.ops.recv = hl340_can_receive;

  // Register the CAN device
  if (register_candev(netdev) < 0) {
    dev_err(&pdev->dev, "Failed to register CAN device\n");
    filp_close(hl340_dev->serial_filp, NULL);
    return -ENODEV;
  }

  // Set the driver data to the hl340_can_device structure for future reference
  platform_set_drvdata(pdev, hl340_dev);

  dev_info(&pdev->dev, "Device registered: %s\n", hl340_dev->device_name);
  return 0;
}

static int hl340_can_remove(struct platform_device *pdev)
{
  struct hl340_can_device *hl340_dev = platform_get_drvdata(pdev);
  struct net_device *netdev = hl340_dev->netdev;

  // Unregister the CAN device
  unregister_candev(netdev);

  // Close the serial device
  filp_close(hl340_dev->serial_filp, NULL);

  dev_info(&pdev->dev, "Device unregistered: %s\n", hl340_dev->device_name);
  return 0;
}

static struct platform_driver hl340_can_driver = {
    .probe = hl340_can_probe,
    .remove = hl340_can_remove,
    .driver = {
        .name = DRIVER_NAME,
        .owner = THIS_MODULE,
    },
};

static int __init hl340_can_init(void)
{
  int ret;

  // Register the platform driver
  ret = platform_driver_register(&hl340_can_driver);
  if (ret) {
    pr_err("Failed to register hl340_can driver: %d\n", ret);
    return ret;
  }

  pr_info("hl340_can driver loaded\n");
  return 0;
}

static void __exit hl340_can_exit(void)
{
  // Unregister the platform driver
  platform_driver_unregister(&hl340_can_driver);

  pr_info("hl340_can driver unloaded\n");
}

module_init(hl340_can_init);
module_exit(hl340_can_exit);

MODULE_AUTHOR("Your Name");
MODULE_DESCRIPTION("Sample hl340_can driver with multiple configurable devices");
MODULE_LICENSE("GPL");

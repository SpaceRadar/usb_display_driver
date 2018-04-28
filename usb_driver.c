#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/uaccess.h>


#define ML_VENDOR_ID    0x0403
#define ML_PRODUCT_ID   0x6001

#define BULK_BUFFER_SIZE   8192

#define DEVNAME   "usb_monitor"
#define LCD_MINOR_BASE  0

#define IOCTL_CMD_MASK          0xFFFFFF00
#define IOCTL_PACKET_SIZE_MASK  0x000000FF

#define IOCTL_SET_REPORT        0x00000000
#define IOCTL_GET_REPORT        0x80000000


struct device_data_t
{
	struct usb_device 		*udev;
	struct usb_interface 		*interface;
	unsigned char			minor;
        struct mutex                    io_mutex;

	struct usb_endpoint_descriptor	*bulk_out_ep;
	unsigned int 			bulk_out_packet_size;

	struct usb_endpoint_descriptor	*interrupt_in_ep;
	unsigned int 			interrupt_in_packet_size;

	unsigned char			*bulk_buffer;
};

static struct usb_driver pen_driver;
static DEFINE_MUTEX(open_disc_mutex);

static int open(struct inode *inode, struct file *file)
{
	struct device_data_t *device_data;
	struct usb_interface *interface;
	int subminor;

	subminor = iminor(inode);

	interface = usb_find_interface(&pen_driver, subminor);
	if (!interface)
        {
		printk(KERN_ERR DEVNAME": %s - error, can't find device for minor %d\n",__func__, subminor);
		return -ENODEV;
	}

	device_data = usb_get_intfdata(interface);
	if (!device_data)
        {
		return -ENODEV;
	}
	file->private_data = device_data;
	return 0;
}

static int     release(struct inode *inode, struct file *file)
{
        return 0;
}

static long unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct device_data_t*	device_data;
    int                         result;
    char                        ctrl_buff[256];
    unsigned long               count;

    printk(KERN_INFO DEVNAME": ioctl request cmd %d \n",cmd);

    device_data = file->private_data;

    if (!device_data)
      return -ENODEV;

   count= cmd & IOCTL_PACKET_SIZE_MASK;

    switch(cmd & IOCTL_CMD_MASK)
    {
      case (IOCTL_SET_REPORT):
        if(!count)
          return 0;

        if(copy_from_user(ctrl_buff,(void*) arg,count))
        {
          return -EFAULT;
        }

        result = usb_control_msg(device_data->udev,
                                 usb_sndctrlpipe(device_data->udev,0),
                                 0x09,   // set report
                                 0x21,   //type | dir | recip
                                 0x200,
                                 0,      //
                                 ctrl_buff,
                                 count,  //message size
                                 0);     //Timeout

      break;

      case (IOCTL_GET_REPORT):
        if(!count)
          return 0;

        result = usb_control_msg(device_data->udev,
                                 usb_rcvctrlpipe(device_data->udev,0),
                                 0x01,  // get report
                                 0x21 | USB_DIR_IN,  //type | dir | recip
                                 0x200,
                                 0,     //
                                 ctrl_buff,
                                 count,     //message size
                                 0);    //Timeout

        if(result < 1)
        {
          return result;
        }

        if(copy_to_user((void*) arg,ctrl_buff,result))
        {
          return -EFAULT;
        }

      break;

      default:
        result=-EINVAL;
      break;
    }

    printk(KERN_INFO DEVNAME": ioctl request result %d \n",result);
    return result;
}

ssize_t read(struct file *file, char __user *user_buff, size_t count, loff_t *f_pos)
{
    struct device_data_t*	device_data;
    int                         readed;
    int                         result;
    char                        bulk_buffer[512];  /////!!!!!!!!!!!!!


    device_data = file->private_data;

    if (!device_data)
      return -ENODEV;

    //Check if interrupt-in endpoint is exists
    if(!device_data->interrupt_in_ep)
    {
      printk(KERN_ERR DEVNAME": Can not read! Interrupt-in endpoint does not exist\n");
      return -EIO;
    }

    printk(KERN_INFO DEVNAME": read %d bytes 0x%08x\n",(unsigned int) count,(unsigned int)file->private_data);

    if(count>device_data->interrupt_in_packet_size)  /////!!!!!!!!!!!!!
    {
      count=device_data->interrupt_in_packet_size;
    }

    result = usb_interrupt_msg(device_data->udev,
                               usb_rcvintpipe(device_data->udev,
                                              device_data->interrupt_in_ep->bEndpointAddress),
                               bulk_buffer,
                               count,
                               &readed,
                               0);

    if(result<0)
    {
      return result;
    }
    printk(KERN_INFO DEVNAME": result %d bytes %d\n",(unsigned int) result,(unsigned int)readed);

    if(copy_to_user(user_buff, bulk_buffer, readed))
        return -EFAULT;

    *f_pos+=readed;

    return readed;
}

static ssize_t write(struct file *file, const char __user *user_buff, size_t count, loff_t *f_pos)
{
    struct device_data_t*	device_data;
    int                         written;
    int                         result;
//    char                        bulk_buffer[4096];  /////!!!!!!!!!!!!!

//    printk(KERN_INFO DEVNAME": write %d bytes 0x%08x\n",(unsigned int) count,(unsigned int)filp->private_data);

    device_data = file->private_data;

    if (!device_data)
      return -ENODEV;

    //Check if bulk-out endpoint is exists
    if(!device_data->bulk_out_ep)
    {
      printk(KERN_ERR DEVNAME": Can not write! Bulk-out endpoint does not exist\n");
      return -EIO;
    }
#if 0
    if(count>device_data->bulk_out_packet_size)  /////!!!!!!!!!!!!!
    {
      count=device_data->bulk_out_packet_size;
    }
#endif
    if(count>BULK_BUFFER_SIZE)  /////!!!!!!!!!!!!!
    {
      count=BULK_BUFFER_SIZE;
    }


    if(copy_from_user(device_data->bulk_buffer,user_buff,count))
    {
        return -EFAULT;
    }

    result = usb_bulk_msg(device_data->udev,
                          usb_sndbulkpipe(device_data->udev,
                                          device_data->bulk_out_ep->bEndpointAddress),
                          device_data->bulk_buffer,
                          count,
                          &written,
                          0);

    if(result<0)
    {
      printk(KERN_ERR DEVNAME": Can not write! Bulk-out endpoint write error!!\n");
      return result;
    }

    *f_pos+=written;

//    printk(KERN_INFO DEVNAME": written %d bytes\n",written);

    return written;
}

static struct file_operations fops = {
	.owner 		=	THIS_MODULE,
	.write 		=	write,
        .read           =       read,
	.open 		=	open,
	.release 	=	release,
        .unlocked_ioctl =       unlocked_ioctl,
};

static struct usb_class_driver usblcd_class = {
	.name       =	"lcd%d",
	.fops       =	&fops,
	.minor_base =	LCD_MINOR_BASE,
};
static int pen_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct device_data_t*           device_data;
    struct usb_host_interface*      iface_desc;
    struct usb_endpoint_descriptor* endpoint;
    int                             i,result;
/*
    struct usblcd				*dev;
    struct usb_host_interface 			*iface_desc;
    struct usb_endpoint_descriptor		*endpoint;

    	int retval = -ENODEV;
	int i;
*/
    printk(KERN_INFO DEVNAME"Pen drive (%04X:%04X) plugged\n", id->idVendor, id->idProduct);
    printk(KERN_INFO DEVNAME"Endpoints count %d\n",interface->cur_altsetting->desc.bNumEndpoints);

    device_data = kzalloc(sizeof(struct device_data_t),GFP_KERNEL);
    if(!device_data)
    {
     	printk(KERN_ERR DEVNAME": Can not allocate mem for service buffer\n");
        return -ENOMEM;
    }
    printk(KERN_INFO DEVNAME": Service buffer memory is allocated\n");

    device_data->bulk_buffer = kzalloc(BULK_BUFFER_SIZE, GFP_KERNEL);
    if(!device_data->bulk_buffer)
    {
     	printk(KERN_ERR DEVNAME": Can not allocate mem for bulk buffer\n");
	result = -ENOMEM;
	goto free_device_data;
    }
    printk(KERN_INFO DEVNAME": Bulk buffer is allocated\n");

    device_data->udev=interface_to_usbdev(interface);
    device_data->interface = interface;

    iface_desc = interface->cur_altsetting;

    device_data->bulk_out_ep=NULL;
    device_data->interrupt_in_ep=NULL;

    for (i = 0; i < iface_desc->desc.bNumEndpoints; ++i)
    {
	endpoint = &iface_desc->endpoint[i].desc;
	if(usb_endpoint_is_bulk_out(endpoint))
	{
		device_data->bulk_out_ep=endpoint;
		device_data->bulk_out_packet_size = le16_to_cpu(endpoint->wMaxPacketSize);
	}

	if(usb_endpoint_is_int_in(endpoint))
	{
		device_data->interrupt_in_ep=endpoint;
		device_data->interrupt_in_packet_size = le16_to_cpu(endpoint->wMaxPacketSize);
	}

    }

    if(!device_data->bulk_out_ep)
    {
     	printk(KERN_ERR DEVNAME": Can not find bulk-out endpoint!\n");
	//result = -EIO;    /////!!!!!!!!!!! Only for debug purpose
	//goto free_bulk_buffer;
    }
    else
    {
      printk(KERN_INFO DEVNAME": Bulk-out endpoint is found 0x%02x size %d\n", device_data->bulk_out_ep->bEndpointAddress, device_data->bulk_out_packet_size);
    }

    if(!device_data->interrupt_in_ep)
    {
     	printk(KERN_ERR DEVNAME": Can not find interrupt-in endpoint!\n");
	//result = -EIO;    /////!!!!!!!!!!! Only for debug purpose
	//goto free_bulk_buffer;
    }
    else
    {
      printk(KERN_INFO DEVNAME": Interrupt-in endpoint is found 0x%02x size %d\n", device_data->interrupt_in_ep->bEndpointAddress, device_data->interrupt_in_packet_size);
    }

    printk(KERN_INFO DEVNAME": device_data 0x%08x\n",(unsigned int)device_data);
    usb_set_intfdata(interface, device_data);

    result = usb_register_dev(interface, &usblcd_class);
    if (result)
    {
     	printk(KERN_ERR DEVNAME": Not able to get a minor for this device\n");
	usb_set_intfdata(interface, NULL);
	goto free_bulk_buffer;
    }

    device_data->minor = interface->minor;
    printk(KERN_INFO DEVNAME": USB STM32-based LCD module connected as lcd%d\n",device_data->minor-LCD_MINOR_BASE);

    return 0;

free_bulk_buffer:
    kfree(device_data->bulk_buffer);
free_device_data:
    kfree(device_data);
    return result;
}
 
static void pen_disconnect(struct usb_interface *interface)
{
//	int minor = interface->minor;
	struct device_data_t *device_data;

        printk(KERN_INFO DEVNAME": USB device disconnected\n");

	mutex_lock(&open_disc_mutex);
	device_data = usb_get_intfdata(interface);
	usb_set_intfdata(interface, NULL);
	mutex_unlock(&open_disc_mutex);

	/* give back our minor */
	usb_deregister_dev(interface, &usblcd_class);

        kfree(device_data->bulk_buffer);
	kfree(device_data);
}
 
static struct usb_device_id pen_table[] =
{
    { USB_DEVICE(ML_VENDOR_ID, ML_PRODUCT_ID) },
    {} /* Terminating entry */
};
MODULE_DEVICE_TABLE (usb, pen_table);
 
static struct usb_driver pen_driver =
{
    .name = "pen_driver",
    .id_table = pen_table,
    .probe = pen_probe,
    .disconnect = pen_disconnect,
};
 
static int __init pen_init(void)
{
    return usb_register(&pen_driver);
}
 
static void __exit pen_exit(void)
{
    usb_deregister(&pen_driver);
}
 
module_init(pen_init);
module_exit(pen_exit);
 
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Anil Kumar Pugalia <email_at_sarika-pugs_dot_com>");
MODULE_DESCRIPTION("USB Pen Registration Driver");
/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * Poutas Sokratis, poutasok@gmail.com
 * Stais Aggelos,   aggelosstaisv@gmail.com
 *
 */

#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/ioctl.h>
#include <linux/types.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/mmzone.h>
#include <linux/vmalloc.h>
#include <linux/spinlock.h>

#include "lunix.h"
#include "lunix-chrdev.h"
#include "lunix-lookup.h"

/*
 * Global data
 */
struct cdev lunix_chrdev_cdev;

/*
 * Just a quick [unlocked] check to see if the cached
 * chrdev state needs to be updated from sensor measurements.
 */
static int lunix_chrdev_state_needs_refresh(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint32_t time;

	// Will warn in kernel log if the allocation fails ????????
	WARN_ON (!(sensor = state->sensor));

	// It says unlocked. Why though ? Locked seems more safe.

	// Timestamp (last_update) is used to figure out whether 
	// there is a new measure in sensor_struct
	time = sensor->msr_data[state->type]->last_update;

	if(state->buf_timestamp < time) return 1;
	/* The following return is bogus, just for the stub to compile */
	debug("Update at state_struct is needed.\n");
	return -EAGAIN; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 * 
 * It is executed only when data are available for update.
 */

static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint16_t raw_data;
	uint32_t time;
	long lookup_data;
	long *lookup[N_LUNIX_MSR] = {lookup_voltage, lookup_temperature, lookup_light};
	unsigned int int_part,dec_part,i;
   	unsigned char sign;
	unsigned long flags;

	debug("Entering.\n");
	
	sensor = state->sensor;
	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	
	/* 
	 * Why spinlock ? See LDD3, p. 119 
	 * Because we don't want the code to be put
	 * to sleep during update of the data. 
	 * The data may become corrupt. 
	 * (Because of an interrupt for example)
	 */

	/* 
	 * In which context is this executed ?
	 * Interrupt context, so interrupts should
	 * be disabled while holding the lock so irqsave is used
	 */

	spin_lock_irqsave(&sensor->lock,flags);

	raw_data = sensor->msr_data[state->type]->values[0];
	time = sensor->msr_data[state->type]->last_update;

	spin_unlock_irqrestore(&sensor->lock,flags);

	/*
	 * Any new data available?
	 * There are available data
	 */
	state->buf_timestamp = time; //update timestamp
	
	/* Data Formation:
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */

	lookup_data= lookup[state->type][raw_data];
	sign = (int) lookup_data >= 0 ? ' ' : '-';
    int_part = lookup_data / 1000;
    dec_part = lookup_data % 1000;
	sprintf(state->buf_data,"%c%d.%d\n",sign,int_part,dec_part);	
	
	// Set buf_lim to the number of data bytes available 
	// minus -1 because there is also \n in the buffer
	// and shouldn't be counted as data byte
	state->buf_lim = strnlen(state->buf_data, 20)-1;

	debug("data returned = %s", state->buf_data);

	for(i = 0; i < state->buf_lim; i++)
		debug("state->buf_data[%d] = %c", i, state->buf_data[i]);
	debug("leaving\n");
	return 0;
}

/*************************************
 * Implementation of file operations
 * for the Lunix character device
 *************************************/

static int lunix_chrdev_open(struct inode *inode, struct file *filp)
{
	/* Declarations */
	struct lunix_chrdev_state_struct *state;
	int ret, major, minor,sensor_type;

	debug("entering\n");
	ret = -ENODEV; // Returns no device error code

	// Informs the kernel that the file is not seekable
	if ((ret = nonseekable_open(inode, filp)) < 0)
		goto out;
	debug("after if ret = %d", ret); //mine
	/*
	 * Associate this open file with the relevant sensor based on
	 * the minor number of the device node [/dev/sensor<NO>-<TYPE>]
	 */
	major = imajor(inode);
	minor = iminor(inode);
	debug("device number: major = %d, minor = %d", major, minor);

	sensor_type=minor%8;
	if(sensor_type >= N_LUNIX_MSR) { // Sensor number doesn't exist
		ret = -ENODEV;								
		debug("Leaving, with return number = %d\n", ret);
		return ret;
	 }

	debug("device type = %d", minor % 8);

	/* Allocate a new Lunix character device private state structure.
	 * Normal kernel ram is allocated
	 */
	state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if(!state) debug("state = NULL");
	else debug("state = valid?");

	// state_struct fields are filled
	state->type = sensor_type;
	state->sensor = &lunix_sensors[minor / 8]; 	//lunix lunix_sensors is externally defined in lunix.h and initialized in lunix-model.c/(init)
												//lunix_sensors[0] ... lunix_sensors[15]
	//initialize semaphore:  lock ??
	sema_init(&state->lock, 1);
	state->buf_timestamp = 0;
	state->buf_lim = 0;
	filp->private_data = state;

out:
	debug("leaving, with ret = %d\n", ret);
	return ret;
}

static int lunix_chrdev_release(struct inode *inode, struct file *filp)
{
	/* ? */
	//mine begin
	kfree(filp->private_data);
	//mine end
	return 0;
}

static long lunix_chrdev_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	/* Why? */
	return -EINVAL;
}

/* filp:file pointer, cnt: size of the requested data to transfer
*/
static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t return_bytes,try_bytes;
	int rest;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	debug("entering\n");

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	// Try to get the semaphore
	// if it returns nonzero, the operation was interrupted. 
	if (down_interruptible(&state->lock))
 		return -ERESTARTSYS;

	debug("Got the state_semaphore.\n");
	debug("Asked to read from device type = %d.\n", state->type);

	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */

	// If f_pos==0 all the bytes of the previous measurement were read so
	// we check if there's a new measurement.
	if (*f_pos == 0) {	
		// While there is no need for an update in state_struct
		while (lunix_chrdev_state_needs_refresh(state) == -EAGAIN) {
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			up(&state->lock); //release the lock
			debug("reading going to sleep");
			// process put in list of waiting processes for new sersor data
			if (wait_event_interruptible(sensor->wq, (lunix_chrdev_state_needs_refresh(state) != -EAGAIN)))
				return -ERESTARTSYS; /* our process was woken up by signal: tell the fs layer to handle it */
				/* otherwise loop, but first reacquire the lock */

			// Try to get the lock
			if (down_interruptible(&state->lock))
				return -ERESTARTSYS;
		}
		// We have the state semaphore and an update in state struct data is needed
		lunix_chrdev_state_update(state);
	}

	/* Determine the number of cached bytes to copy to userspace.
	 * If the asked bytes are more that the ones left 
	 * from the measurement try to return the left ones.
	 */
	if(cnt > state->buf_lim- *f_pos)
		try_bytes=state->buf_lim - *f_pos;
	else
		try_bytes=cnt; // Try to return as many bytes as asked
	
	// If the last bytes of the measurement are asked 
	// increase the bytes trying to return to include the \n character.
	if(try_bytes+*f_pos==state->buf_lim)
		try_bytes+=1;

	// Try to copy the bytes to userspace
	if((rest = copy_to_user(usrbuf, state->buf_data+*f_pos, try_bytes))){
			return_bytes= -EFAULT; // If fails return error
			goto out;
	}
	
	// The return bytes are those tried minus those failed
	return_bytes = try_bytes - rest;
	
	debug("Bytes read %ld.\n", return_bytes);

	// Increase the f_pos as many bytes as returned to userspace
	*f_pos += return_bytes; 

	/* Auto-rewind on EOF mode?
	 * If given all the data of the measurement,
	 * start from the begining of the file.
	 */
	if (*f_pos == state->buf_lim+1)
		*f_pos = 0;

out:
	/* Unlock? */
	up(&state->lock);
	debug("Leaving Read.\n");
	return return_bytes;
}

static int lunix_chrdev_mmap(struct file *filp, struct vm_area_struct *vma)
{
	return -EINVAL;
}

static struct file_operations lunix_chrdev_fops =
{
        .owner          = THIS_MODULE,
	.open           = lunix_chrdev_open,
	.release        = lunix_chrdev_release,
	.read           = lunix_chrdev_read,
	.unlocked_ioctl = lunix_chrdev_ioctl,
	.mmap           = lunix_chrdev_mmap,
	.llseek			= no_llseek ////maybe not working
};

int lunix_chrdev_init(void)
{
	/*
	 * Register the character device with the kernel, asking for
	 * a range of minor numbers (number of sensors * 8 measurements / sensor)
	 * beginning with LINUX_CHRDEV_MAJOR:0
	 */
	int ret;
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3; // Why?

	debug("initializing character device\n");
	
	// Intializes chrdev_cdev based on chrdev_fops file operations struct
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	// Creates a device number from major 60 and minor 0
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	
	// mine begin
	// Allocate the needed device IDs
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix_device");
	// mine end
	/* register_chrdev_region? */

	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}

	//mine begin
	// Inserts the device (represented by chrdev_struct) to the kernel
	ret = cdev_add(&lunix_chrdev_cdev, dev_no, lunix_minor_cnt);
	//mine end
	/* cdev_add? */
	if (ret < 0) {
		debug("failed to add character device\n");
		goto out_with_chrdev_region;
	}
	debug("completed successfully\n");
	return 0;

out_with_chrdev_region:
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
out:
	return ret;
}

void lunix_chrdev_destroy(void)
{
	dev_t dev_no;
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("entering\n");
	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	cdev_del(&lunix_chrdev_cdev);
	unregister_chrdev_region(dev_no, lunix_minor_cnt);
	debug("leaving\n");
}
/*
 * lunix-chrdev.c
 *
 * Implementation of character devices
 * for Lunix:TNG
 *
 * < Your name here >
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

	WARN_ON ( !(sensor = state->sensor));
	/* ? */

	spin_lock(&sensor->lock);
	time = sensor->msr_data[state->type]->last_update;
	//should we grab magic number?
	spin_unlock(&sensor->lock);


	if(state->buf_timestamp < time) return 1;
	/* The following return is bogus, just for the stub to compile */
	debug("needs update");
	return -EAGAIN; /* ? */
}

/*
 * Updates the cached state of a character device
 * based on sensor data. Must be called with the
 * character device state lock held.
 */
static int lunix_chrdev_state_update(struct lunix_chrdev_state_struct *state)
{
	struct lunix_sensor_struct *sensor;
	uint16_t raw_data;
	uint32_t time;
	long lookup_data;
	int i, int_part, dec_part, digit_point;

	debug("entering\n");

	/*
	 * Grab the raw data quickly, hold the
	 * spinlock for as little as possible.
	 */
	 sensor = state->sensor;

	 spin_lock(&sensor->lock);
	 raw_data = sensor->msr_data[state->type]->values[0];
	 time = sensor->msr_data[state->type]->last_update;
	 //should we grab magic number?
	 spin_unlock(&sensor->lock);

	/* ? */
	/* Why use spinlocks? See LDD3, p. 119 */

	/*
	 * Any new data available?
	 */
	state->buf_timestamp = time; //update timestamp
	lookup_data = 0;
	switch (state->type) {
		case 0:
			lookup_data = lookup_voltage[raw_data];
			break;
		case 1:
			lookup_data = lookup_temperature[raw_data];
			break;
		case 2:
			lookup_data = lookup_light[raw_data];
			break;
		case 3:
			debug("invalid switch case!");
			break;
	}
	debug("lookup_data = %ld", lookup_data);
	/* ? */

	/*
	 * Now we can take our time to format them,
	 * holding only the private state semaphore
	 */
	i = 0;
	if(lookup_data < 0){
		state->buf_data[i++] = '-';
		lookup_data *= (-1);
	}
	dec_part = lookup_data % 1000;
	int_part = lookup_data / 1000;
	debug("int_part = %d", int_part);
	debug("dec_part = %d", dec_part);

	if(int_part <= 0){
		state->buf_data[i++] = '0';
	}
	else{
		digit_point = 10000;
		while (int_part / digit_point == 0){
			int_part %= digit_point;
			digit_point /= 10; //move digit_point to the right
			if(digit_point == 0){
				debug("point where digit_point = 0");
				state->buf_data[i++] = int_part + 48; //int_part is only one digit
				goto dec;
			}
		}
		while(digit_point > 0){
			state->buf_data[i++] = (int_part / digit_point) + 48;
			int_part %= digit_point;
			digit_point /= 10; //move digit_point to the right
		}
	}//int part complete
dec:
	state->buf_data[i++] = '.';
	digit_point = 100;
	while(digit_point > 0){
		state->buf_data[i++] = (dec_part / digit_point) + 48;
		dec_part %= digit_point;
		digit_point /= 10; //move digit_point to the right
	}
	state->buf_data[i++] = '\n'; //add a newline char
	state->buf_lim = i;
	/* ? */
	debug("data returned = %s", state->buf_data);
	for(i = 0; i < state->buf_lim; i++ )
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
	/* ? */
	int ret, major, minor;

	debug("entering\n");
	ret = -ENODEV;
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

	debug("device type = %d", minor % 8);

	/* Allocate a new Lunix character device private state structure */
	//mine begin
	state = kmalloc(sizeof(struct lunix_chrdev_state_struct), GFP_KERNEL);
	if(!state) debug("state = NULL");
	else debug("state = valid?");

	state->type = minor % 8;
	state->sensor = &lunix_sensors[minor / 8]; 	//lunix lunix_sensors is externally defined in lunix.h and initialized in lunix-model.c/(init)
												//lunix_sensors[0] ... lunix_sensors[15]
	//initialize semaphore:  lock ??
	sema_init(&state->lock, 1);
	state->buf_timestamp = 0;
	state->buf_lim = 0;
	filp->private_data = state;
	//mine end
	/* ? */
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

static ssize_t lunix_chrdev_read(struct file *filp, char __user *usrbuf, size_t cnt, loff_t *f_pos)
{
	ssize_t ret;
	int rest;

	struct lunix_sensor_struct *sensor;
	struct lunix_chrdev_state_struct *state;

	debug("entering\n");

	state = filp->private_data;
	WARN_ON(!state);

	sensor = state->sensor;
	WARN_ON(!sensor);

	//mine begin
	debug("NULL checks passed");

	if (down_interruptible(&state->lock))
 		return -ERESTARTSYS;
	debug("got the semaphore");
	debug("asked to read from device type = %d", state->type);

	// if(lunix_chrdev_state_needs_refresh(state))
	// 	lunix_chrdev_state_update(state);

	//mine end
	/* Lock? */
	/*
	 * If the cached character device state needs to be
	 * updated by actual sensor data (i.e. we need to report
	 * on a "fresh" measurement, do so
	 */
	// if (*f_pos == 0) {
	// 	while (lunix_chrdev_state_update(state) == -EAGAIN) {
	// 		/* ? */
	// 		/* The process needs to sleep */
	// 		/* See LDD3, page 153 for a hint */
	// 	}
	// }
	if (*f_pos == 0) {
		while (lunix_chrdev_state_needs_refresh(state) == -EAGAIN) {
			/* ? */
			/* The process needs to sleep */
			/* See LDD3, page 153 for a hint */
			up(&state->lock); //release the lock
			debug("reading going to sleep");
			if (wait_event_interruptible(sensor->wq, (lunix_chrdev_state_needs_refresh(state) != -EAGAIN)))
				return -ERESTARTSYS; /* signal: tell the fs layer to handle it */
				/* otherwise loop, but first reacquire the lock */
			if (down_interruptible(&state->lock))
				return -ERESTARTSYS;
		}
	}
	//when we exit from the while loop, we know that the semaphore is
	//held and an update is needed.
	lunix_chrdev_state_update(state);
	if(cnt > state->buf_lim){
		rest = copy_to_user(usrbuf, state->buf_data, state->buf_lim);
		ret = state->buf_lim - rest;
	}
	else{
		rest = copy_to_user(usrbuf, state->buf_data, cnt);
		ret = cnt - rest;
	}
	debug("bytes read %ld", ret);
	/* End of file */
	/* ? */

	/* Determine the number of cached bytes to copy to userspace */
	/* ? */

	/* Auto-rewind on EOF mode? */
	/* ? */
//out:
	/* Unlock? */
	up(&state->lock);
	debug("leaving\n");
	return ret;
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
	.mmap           = lunix_chrdev_mmap
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
	unsigned int lunix_minor_cnt = lunix_sensor_cnt << 3;

	debug("initializing character device\n");
	cdev_init(&lunix_chrdev_cdev, &lunix_chrdev_fops);
	lunix_chrdev_cdev.owner = THIS_MODULE;

	dev_no = MKDEV(LUNIX_CHRDEV_MAJOR, 0);
	/* ? */
	// mine begin
	ret = register_chrdev_region(dev_no, lunix_minor_cnt, "lunix_device");
	// mine end
	/* register_chrdev_region? */
	if (ret < 0) {
		debug("failed to register region, ret = %d\n", ret);
		goto out;
	}
	/* ? */
	//mine begin
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

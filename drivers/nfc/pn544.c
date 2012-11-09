/*
 * Driver for the PN544 NFC chip.
 *
 * Copyright (C) Nokia Corporation
 *
 * Author: Jari Vanhala <ext-jari.vanhala@nokia.com>
 * Contact: Matti Aaltonen <matti.j.aaltonen@nokia.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 */

#include <linux/completion.h>
#include <linux/crc-ccitt.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nfc/pn544.h>
#include <linux/poll.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/atomic.h>

#define PN544_TAG "PN544_NFC"
#define DRIVER_CARD "PN544 NFC"
#define DRIVER_DESC "NFC driver for PN544"

static unsigned int cur_dbg_level;
typedef enum __dbg_level
{
	dbg_level_critical = 0,
	dbg_level_error,
	dbg_level_warning,
	dbg_level_info,
	dbg_level_debug,
	dbg_level_verbose,
	dbg_level_lowest
}dbg_level;
#define DBG_PRINT(msg_level,...) \
	{ \
		if((msg_level) <= (cur_dbg_level)) \
		{ \
			printk(KERN_INFO __VA_ARGS__); \
		} \
	}
#define DBG_PRINT_HEX(msg_level,...) \
	{ \
		if((msg_level) <= (cur_dbg_level)) \
		{ \
			print_hex_dump(KERN_INFO, ##__VA_ARGS__); \
		} \
	}

#define PN544_RSET_CMD 0xF9

static struct i2c_device_id pn544_id_table[] = {
	{PN544_DRIVER_NAME, 0},
	{ }
};
MODULE_DEVICE_TABLE(i2c, pn544_id_table);

#define HCI_MODE 0
#define FW_MODE	1

enum pn544_state
{
	PN544_ST_COLD,
	PN544_ST_FW_READY,
	PN544_ST_READY,
};

struct circular_buffer
{
	u8 *buffer;
	long buf_len;
	u8 *read;
	u8 *write;
	struct mutex buffer_mutex;
};

#define PN544_READ_BUF_SIZE (256)
struct pn544_info
{
	struct miscdevice miscdev;
	struct i2c_client *i2c_dev;
	struct work_struct irq_work;
	atomic_t pn544_state;
	wait_queue_head_t read_wait;
	atomic_t irq_state;
	atomic_t read_irq;
	atomic_t open_counter;
	atomic_t max_fd;
	atomic_t bytes_to_read;
	struct circular_buffer response;
	struct regulator *vdd;
};

static void pn544_disable(struct pn544_info *info);
static int pn544_enable(struct pn544_info *info, int mode, int force);

static int circular_buffer_init(struct circular_buffer *c_buf, long size)
{
	DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Initializing circular buffer.\n", __func__);
	c_buf->buf_len = size;
	c_buf->buffer = kzalloc(c_buf->buf_len, GFP_KERNEL);
	if(!c_buf->buffer)
	{
		/* Error */
		DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: Could not allocate memory for circular buffer.\n", __func__);
		c_buf->buf_len = 0;
		return -ENOMEM;
	}
	c_buf->read = 0;
	c_buf->write = c_buf->buffer;
	mutex_init(&c_buf->buffer_mutex);
	return 0;
}

static void circular_buffer_destroy(struct circular_buffer *c_buf)
{
	DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Freeing memory for circular buffer.\n", __func__);
	kfree(c_buf->buffer);
}

static void circular_buffer_flush(struct circular_buffer *c_buf)
{
	DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Flushing circular buffer.\n", __func__);
	c_buf->read = 0;
	c_buf->write = c_buf->buffer;
}

static int circular_buffer_fill(struct circular_buffer *c_buf, u8 *buf, long size) /* write into the circular buffer */
{
	long free_bytes, wrap_bytes;
	int filled_bytes;

	DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Filling circular buffer.\n", __func__);
	/* acquire the synchronisation structure */
	mutex_lock(&c_buf->buffer_mutex);
	if(c_buf->read == c_buf->write) /* Buffer full */
	{
		/* Error */
		DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: Circular buffer full.\n", __func__);
		filled_bytes = -EAGAIN;
	}
	else if(c_buf->read == 0) /* Buffer empty */
	{
		DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Circular buffer empty.\n", __func__);
		if(c_buf->buf_len >= size)
		{
			memcpy(c_buf->buffer, buf, size);
			c_buf->read = c_buf->buffer;
			if(c_buf->buf_len == size) /* The buffer will become full */
			{
				c_buf->write = c_buf->read;
			}
			else
			{
				c_buf->write = c_buf->buffer + size;
			}
			filled_bytes = size;
		}
		else
		{
			/* Error */
			DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: Circular buffer does not have enough free bytes.\n", __func__);
			filled_bytes = -ENOMEM;
		}
	}
	else /* Buffer partially full */
	{
		DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Circular buffer partially empty.\n", __func__);
		if(c_buf->read > c_buf->write)
		{
			free_bytes = c_buf->read - c_buf->write;
			wrap_bytes = 0;
		}
		else if(c_buf->read < c_buf->write)
		{
			free_bytes = c_buf->read + c_buf->buf_len - c_buf->write;
			wrap_bytes = c_buf->buffer + c_buf->buf_len - c_buf->write;
		}
		if(free_bytes >= size)
		{
			if((wrap_bytes > 0) && (wrap_bytes <= size))
			{
				memcpy(c_buf->write, buf, wrap_bytes);
				memcpy(c_buf->buffer, buf + wrap_bytes, size - wrap_bytes);
				c_buf->write = c_buf->buffer + size - wrap_bytes;
			}
			else
			{
				memcpy(c_buf->write, buf, size);
				c_buf->write += size;
			}
			filled_bytes = size;
		}
		else
		{
			/* Error */
			DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: Circular buffer does not have enough free bytes.\n", __func__);
			filled_bytes = -ENOMEM;
		}
	}
	/* release the synchronisation structure */
	mutex_unlock(&c_buf->buffer_mutex);

	return filled_bytes;
}

static int circular_buffer_drain(struct circular_buffer *c_buf, u8 *buf, long size) /* write into the circular buffer */
{
	long filled_bytes, wrap_bytes;
	int drained_bytes;

	DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Draining circular buffer.\n", __func__);
	/* acquire the synchronisation structure */
	mutex_lock(&c_buf->buffer_mutex);
	if(c_buf->read == 0) /* Buffer empty */
	{
		/* Error */
		DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: Circular buffer empty.\n", __func__);
		drained_bytes = -ENOMEM;
	}
	else /* Buffer partially/fully full */
	{
		if(c_buf->read == c_buf->write) /* Buffer full */
		{
			DBG_PRINT(dbg_level_debug, PN544_TAG ": %s(): DEBUG: Circular buffer full.\n", __func__);
			filled_bytes = c_buf->buf_len;
			wrap_bytes = c_buf->buffer + c_buf->buf_len - c_buf->read;
		}
		else if(c_buf->read > c_buf->write)
		{
			filled_bytes = c_buf->write + c_buf->buf_len - c_buf->read;
			wrap_bytes = c_buf->buffer + c_buf->buf_len - c_buf->read;
		}
		else if(c_buf->read < c_buf->write)
		{
			filled_bytes = c_buf->write - c_buf->read;
			wrap_bytes = 0;
		}
		if(filled_bytes >= size)
		{
			if((wrap_bytes == 0) || ((wrap_bytes > 0) && (wrap_bytes >= size)))
			{
				memcpy(buf, c_buf->read, size);
				if(wrap_bytes == size)
				{
					c_buf->read = c_buf->buffer;
				}
				else
				{
					c_buf->read += size;
				}
			}
			else
			{
				memcpy(buf, c_buf->read, wrap_bytes);
				memcpy(buf + wrap_bytes, c_buf->buffer, size - wrap_bytes);
				c_buf->read = c_buf->buffer + size - wrap_bytes;
			}
			if(filled_bytes == size) /* The buffer will be empty now */
			{
				c_buf->read = 0;
				c_buf->write = c_buf->buffer;
			}
			drained_bytes = size;
		}
		else
		{
			/* Error */
			DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: Circular buffer does not have enough data bytes.\n", __func__);
			drained_bytes = -ENOMEM;
		}
	}
	/* release the synchronisation structure */
	mutex_unlock(&c_buf->buffer_mutex);

	return drained_bytes;
}

static ssize_t pn544_response_buffer_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	int count = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);

	count += sprintf(buf + count, "echo 0 to flush response buffer");
	count += sprintf(buf + count, "\n");

	return count;
}
static ssize_t pn544_response_buffer_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	unsigned int input;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);
	if(sscanf(buf, "%u", &input) >= 1)
	{
		if(input == 0)
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: flushing response buffer\n", dev_name(dev), __func__);
			circular_buffer_flush(&info->response);
		}
	}
	return count;
}
static DEVICE_ATTR(response_buffer, S_IRUGO| S_IWUSR | S_IWGRP, pn544_response_buffer_show, pn544_response_buffer_store);

static ssize_t pn544_enable_controller_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	int count = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);

	count += sprintf(buf + count,   "0 - disable (now %s)",atomic_read(&info->pn544_state) == PN544_ST_COLD ? "disabled" : "enabled");
	count += sprintf(buf + count, "\n1 - enable in normal mode (now %s in normal mode)", atomic_read(&info->pn544_state) == PN544_ST_READY ? "" : "not");
	count += sprintf(buf + count, "\n2 - enable in fw mode (now %s in fw mode)", atomic_read(&info->pn544_state) == PN544_ST_FW_READY ? "" : "not");
	count += sprintf(buf + count, "\n");
	return count;
}
static ssize_t pn544_enable_controller_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	unsigned int input;
	int ret;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);
	if(sscanf(buf, "%u", &input) >= 1)
	{
		switch(input)
		{
		case 0:
			{
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling controller\n", dev_name(dev), __func__);
				pn544_disable(info);
			}
			break;
		case 1:
			{
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enabling controller in HCI mode\n", dev_name(dev), __func__);
				ret = pn544_enable(info, HCI_MODE, 1);
				if(ret < 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): EROR: Error in enabling the controller in HCI mode\n", dev_name(dev), __func__);
				}
			}
			break;
		case 2:
			{
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enabling controller in FW mode\n", dev_name(dev), __func__);
				ret = pn544_enable(info, FW_MODE, 1);
				if(ret < 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): EROR: Error in enabling the controller in FW mode\n", dev_name(dev), __func__);
				}
			}
			break;
		default:
			{
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Unsupported input (%s)\n", dev_name(dev), __func__, buf);
			}
			break;
		}
	}
	return count;
}
static DEVICE_ATTR(enable_controller, S_IRUGO| S_IWUSR | S_IWGRP, pn544_enable_controller_show, pn544_enable_controller_store);

static ssize_t pn544_max_fds_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	int count = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);

	count += sprintf(buf + count,   "Max number of FDs = %d", atomic_read(&info->max_fd));
	count += sprintf(buf + count, "\nCurrently %d open FDs", atomic_read(&info->open_counter));
	count += sprintf(buf + count, "\n");
	return count;
}
static ssize_t pn544_max_fds_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	unsigned int input;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);
	if(sscanf(buf, "%u", &input) >= 1)
	{
		atomic_set(&info->max_fd, input);
	}
	return count;
}
static DEVICE_ATTR(max_fds, S_IRUGO| S_IWUSR | S_IWGRP, pn544_max_fds_show, pn544_max_fds_store);

static ssize_t pn544_debug_trigger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct pn544_info *info = dev_get_drvdata(dev);
	int count = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);

	count += sprintf(buf + count,   "Nothing here");
	count += sprintf(buf + count, "\n");

	return count;
}
static ssize_t pn544_debug_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(dev), __func__);

	return count;
}
static DEVICE_ATTR(debug_trigger, S_IRUGO| S_IWUSR | S_IWGRP, pn544_debug_trigger_show, pn544_debug_trigger_store);

static ssize_t pn544_dbg_level_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", cur_dbg_level);
}
static ssize_t pn544_dbg_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int dbg_val;
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: setting the debug level\n", dev_name(dev), __func__);
	if(sscanf(buf, "%u", &dbg_val) >= 1)
	{
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: changing the debug level from %d to %d\n", dev_name(dev), __func__, cur_dbg_level, dbg_val);
		cur_dbg_level = dbg_val;
	}
	return count;
}
static DEVICE_ATTR(dbg_level, S_IRUGO| S_IWUSR | S_IWGRP, pn544_dbg_level_show, pn544_dbg_level_store);

static struct attribute *pn544_attributes[] = {
	&dev_attr_debug_trigger.attr,
	&dev_attr_dbg_level.attr,
	&dev_attr_response_buffer.attr,
	&dev_attr_enable_controller.attr,
	&dev_attr_max_fds.attr,
	NULL
};

static struct attribute_group pn544_attribute_group = {
	.attrs = pn544_attributes
};

static void pn544_disable(struct pn544_info *info)
{
	struct i2c_client *client = info->i2c_dev;
	struct pn544_nfc_platform_data *pdata = info->i2c_dev->dev.platform_data;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling controller\n", dev_name(&client->dev), __func__);
	if(0 != atomic_read(&info->irq_state))
	{
		/* First disable the irq to prevent any false triggering */
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling irq\n", dev_name(&client->dev), __func__);
		disable_irq(client->irq);
		atomic_set(&info->irq_state, 0);
	}

	/* Cancel the interrupt processor work */
	cancel_work_sync(&info->irq_work);

	/* Clear the receive buffer */
	circular_buffer_flush(&info->response);

	atomic_set(&info->read_irq, 0);

	if(atomic_read(&info->pn544_state) != PN544_ST_COLD)
	{
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: powering off the controller\n", dev_name(&client->dev), __func__);
		/* Powerdown the chip */
		gpio_set_value(pdata->gpio_ven, 0);
		usleep_range(100, 150); /* Recommended to sleep for at least 100us */
		regulator_disable(info->vdd);
		/* Recommended to sleep for at least 0us */
		gpio_set_value(pdata->gpio_vbat, 0);
		/* Power down complete */

		atomic_set(&info->pn544_state, PN544_ST_COLD);
		msleep(PN544_RESETVEN_TIME);
		DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: powered off the controller\n", dev_name(&client->dev), __func__);
	}
}

static int pn544_enable(struct pn544_info *info, int mode, int force)
{
	int ret;
	struct i2c_client *client = info->i2c_dev;
	struct pn544_nfc_platform_data *pdata = info->i2c_dev->dev.platform_data;

	ret = 0;

	if(0 != atomic_read(&info->irq_state))
	{
		/* First disable the irq to prevent any false triggering */
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling irq\n", dev_name(&client->dev), __func__);
		disable_irq(client->irq);
		atomic_set(&info->irq_state, 0);
	}

	/* Cancel the interrupt processor work */
	cancel_work_sync(&info->irq_work);

	/* Clear the receive buffer */
	circular_buffer_flush(&info->response);

	atomic_set(&info->read_irq, 0);

	if(atomic_read(&info->pn544_state) == PN544_ST_COLD)
	{
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: powering on the controller\n", dev_name(&client->dev), __func__);
		/* Enable the power */
		regulator_enable(info->vdd);
		msleep(10);
		gpio_set_value(pdata->gpio_vbat, 1);
		gpio_set_value(pdata->gpio_ven, 0);
		msleep(PN544_RESETVEN_TIME);
		gpio_set_value(pdata->gpio_ven, 1);
		usleep_range(3000, 4500); /* Boot time is maximum 3ms */
		DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: powered on the controller in HCI mode\n", dev_name(&client->dev), __func__);
	}

	switch(mode)
	{
	case FW_MODE:
		{
			/* Pull the firmware download pin high */
			gpio_set_value(pdata->gpio_firm, 1);

			/* Pulse the VEN pin */
			gpio_set_value(pdata->gpio_ven, 0);
			msleep(PN544_RESETVEN_TIME);
			gpio_set_value(pdata->gpio_ven, 1);
			atomic_set(&info->pn544_state, PN544_ST_FW_READY);
			DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: reset the controller in FW mode\n", dev_name(&client->dev), __func__);
		}
		break;
	case HCI_MODE:
		{
			/* Pull the firmware download pin low */
			gpio_set_value(pdata->gpio_firm, 0);

			/* Pulse the VEN pin */
			gpio_set_value(pdata->gpio_ven, 0);
			msleep(PN544_RESETVEN_TIME);
			gpio_set_value(pdata->gpio_ven, 1);
			atomic_set(&info->pn544_state, PN544_ST_READY);
			DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: reset the controller in HCI mode\n", dev_name(&client->dev), __func__);
		}
		break;
	default:
		{
		}
		break;
	}
	usleep_range(3000, 4500); /* Boot time is maximum 3ms */

	if(0 == atomic_read(&info->irq_state))
	{
		/* Now enable the irq */
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enabling irq\n", dev_name(&client->dev), __func__);
		enable_irq(client->irq);
		atomic_set(&info->irq_state, 1);
	}

	usleep_range(10000, 15000);

	return ret;
}

static int check_crc(u8 *buf, int buflen)
{
	u8 len;
	u16 crc;

	len = buf[0] + 1;
	if(len < 4 || len != buflen || len > PN544_MSG_MAX_SIZE)
	{
		DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: CRC; corrupt packet length %u (%d)\n", __func__, len, buflen);
		DBG_PRINT_HEX(dbg_level_error, "crc: ", DUMP_PREFIX_NONE, 16, 2, buf, buflen, false);
		return -EPERM;
	}
	crc = crc_ccitt(0xffff, buf, len - 2);
	crc = ~crc;

	if(buf[len-2] != (crc & 0xff) || buf[len-1] != (crc >> 8))
	{
		pr_err(PN544_DRIVER_NAME ": CRC error 0x%x != 0x%x 0x%x\n", crc, buf[len-1], buf[len-2]);
		DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): ERROR: CRC error 0x%04x != 0x%02x%02x\n", __func__, crc, buf[len-1], buf[len-2]);
		DBG_PRINT_HEX(dbg_level_error, "crc: ", DUMP_PREFIX_NONE, 16, 2, buf, buflen, false);
		return -EPERM;
	}
	return 0;
}

static int pn544_i2c_write(struct i2c_client *client, u8 *buf, int len)
{
	int retries;
	int ret;

	if(len < 4 || len != (buf[0] + 1))
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Illegal message length: %d\n", dev_name(&client->dev), __func__, len);
		return -EINVAL;
	}

	if(check_crc(buf, len))
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: CRC check failed\n", dev_name(&client->dev), __func__);
		return -EINVAL;
	}

	retries = 1;
	do
	{
		ret = i2c_master_send(client, buf, len);
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: send-%02d: %d\n", dev_name(&client->dev), __func__, retries, ret);
		if(ret == -EREMOTEIO)/* Retry after a small delay, chip was in standby */
		{
			usleep_range(6000, 10000);
		}
		else
		{
			/* Success */
			break;
		}
	}while(retries--);

	if(ret != len)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: send-%02d error: %d %d\n", dev_name(&client->dev), __func__, retries, len, ret);
		return -EREMOTEIO;
	}
	return ret;
}

static int pn544_i2c_read(struct i2c_client *client, struct circular_buffer *resp)
{
	struct pn544_info *info = i2c_get_clientdata(client);
	struct pn544_nfc_platform_data *pdata = client->dev.platform_data;
	int ret;
	u8 len;
	u8 read_buf[PN544_READ_BUF_SIZE];
	int total_bytes_read = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(&client->dev), __func__);

	/*
	 * You could read a packet in one go, but then you'd need to read
	 * max size and rest would be 0xff fill, so we do split reads.
	 */
	while(gpio_get_value(pdata->gpio_irq))
	{
		ret = i2c_master_recv(client, &len, 1);
		if(ret != 1)
		{
			DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: read err1 = %d\n", dev_name(&client->dev), __func__, ret);
			ret = -EREMOTEIO;
			goto err_no_cleanup;
		}
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: len1 = %d\n", dev_name(&client->dev), __func__, len);

		if(len == ((client->addr << 1) | 0x01))
		{
			ret = -EREMOTEIO;
			break;
		}
		if(len < PN544_LLC_HCI_OVERHEAD)
		{
			len = PN544_LLC_HCI_OVERHEAD;
		}
		else if(len > (PN544_MSG_MAX_SIZE - 1))
		{
			len = PN544_MSG_MAX_SIZE - 1;
		}
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: len2 = %d\n", dev_name(&client->dev), __func__, len);

		read_buf[0] = len;
		ret = i2c_master_recv(client, read_buf + 1, len);
		if(ret != len)
		{
			DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: read err2 = %d\n", dev_name(&client->dev), __func__, ret);
			ret = -EREMOTEIO;
			goto err_no_cleanup;
		}
		len += 1; /* This is the total number of bytes read including the length */
		DBG_PRINT_HEX(dbg_level_debug, PN544_TAG ": pn544_i2c_read: read buf: ", DUMP_PREFIX_NONE, 16, 1, read_buf, len, false);

		ret  = circular_buffer_fill(&info->response, read_buf, len);
		if(ret > 0)
		{
			total_bytes_read += ret;
		}
	}
err_no_cleanup:
	if(total_bytes_read > 0)
	{
		return total_bytes_read;
	}
	else
	{
		return ret;
	}
}

static int pn544_fw_write(struct i2c_client *client, u8 *buf, int len)
{
	int ret = 0;

	DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: entering...\n", dev_name(&client->dev), __func__);
	return ret;
}

static irqreturn_t pn544_irq_thread_fn(int irq, void *dev_id)
{
	struct pn544_info *info = dev_id;
	struct i2c_client *client = info->i2c_dev;

	BUG_ON(!info);
	BUG_ON(irq != info->i2c_dev->irq);

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: entering... \n", dev_name(&client->dev), __func__);

	/* Indicate that interrupt processing is pending */
	atomic_inc(&info->read_irq);

	/* Schedule the interrupt processor work */
	schedule_work(&info->irq_work);

	return IRQ_HANDLED;
}

static void pn544_irq_worker(struct work_struct *work)
{
	struct pn544_info *info = container_of(work, struct pn544_info, irq_work);
	struct i2c_client *client = info->i2c_dev;
	int ret;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: entering...\n", dev_name(&client->dev), __func__);
	if(atomic_read(&info->read_irq) == 0) /* No interrupts pending; return */
	{
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: No pending interrupts...\n", dev_name(&client->dev), __func__);
		return;
	}

	ret = pn544_i2c_read(info->i2c_dev, &info->response);
	if(ret < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: read failed with error=%d, %d pending interupts\n", dev_name(&client->dev), __func__, ret, atomic_read(&info->read_irq));
		goto out;
	}
	else
	{
		atomic_add(ret, &info->bytes_to_read);
	}

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: waking up any waiting process\n", dev_name(&client->dev), __func__);
	wake_up_interruptible(&info->read_wait);

out:
	atomic_set(&info->read_irq, 0);
}

static ssize_t pn544_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;
	int ret = 0;
	u8 *read_buf;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: entering... count = %zu\n", dev_name(&client->dev), __func__, count);

	if(atomic_read(&info->pn544_state) == PN544_ST_COLD)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: controller switched off\n", dev_name(&client->dev), __func__);
		ret = -ENODEV;
		goto out_no_cleanup;
	}

	if(atomic_read(&info->bytes_to_read) <= 0) /* no bytes in the buffer, wait for the buffer to fill or return */
	{
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: response buffer empty\n", dev_name(&client->dev), __func__);
		if(file->f_flags & O_NONBLOCK)
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: device opened with O_NONBLOCK flag...returning\n", dev_name(&client->dev), __func__);
			ret = -EAGAIN;
			goto out_no_cleanup;
		}
		if(wait_event_interruptible(info->read_wait, (atomic_read(&info->bytes_to_read) > 0)))
		{
			DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: wait interrupted\n", dev_name(&client->dev), __func__);
			ret = -ERESTARTSYS;
			goto out_no_cleanup;
		}
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: woken up from wait\n", dev_name(&client->dev), __func__);
	}

	read_buf = kzalloc(count, GFP_KERNEL);
	if (!read_buf)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Cannot allocate memory for read buf\n", dev_name(&client->dev), __func__);
		ret = -ENOMEM;
		goto out_no_cleanup;
	}

	ret = circular_buffer_drain(&info->response, read_buf,count);
	DBG_PRINT_HEX(dbg_level_debug, PN544_TAG ": pn544_read(): read buf: ", DUMP_PREFIX_NONE, 16, 1, read_buf, ret, false);
	if((ret > 0) && (copy_to_user(buf, read_buf, ret) != 0))
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not copy read buf to user space\n", dev_name(&client->dev), __func__);
		ret = -EFAULT;
		goto out_free_read_buf;
	}

out_free_read_buf:
	kfree(read_buf);
out_no_cleanup:
	if(ret > 0)
	{
		atomic_sub(ret, &info->bytes_to_read);
	}

	return ret;
}

static ssize_t pn544_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;
	u8 *write_buf;
	ssize_t	len;
	int ret;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: entering... count = %zu\n", dev_name(&client->dev), __func__, count);

	if(atomic_read(&info->pn544_state) == PN544_ST_COLD)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: controller switched off\n", dev_name(&client->dev), __func__);
		ret = -ENODEV;
		goto out;
	}

	/*
	 * XXX: should we detect rset-writes and clean possible
	 * read_irq state
	 */
	if(atomic_read(&info->pn544_state) == PN544_ST_FW_READY)
	{
		dev_err(&client->dev, PN544_TAG ": %s: controller in FW mode\n", __func__);
	}
	else
	{
		if(count < PN544_LLC_MIN_SIZE)
		{
			ret = -EINVAL;
			goto out;
		}

		len = count;
		write_buf = kzalloc(len, GFP_KERNEL);
		if(!write_buf)
		{
			DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Cannot allocate memory for write buf\n", dev_name(&client->dev), __func__);
			ret = -ENOMEM;
			goto out;
		}
		if(copy_from_user(write_buf, buf, len))
		{
			DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not copy write buf from user space\n", dev_name(&client->dev), __func__);
			ret = -EFAULT;
			kfree(write_buf);
			goto out;
		}
		DBG_PRINT_HEX(dbg_level_debug, PN544_TAG ": pn544_write(): write buf: ", DUMP_PREFIX_NONE, 16, 1, write_buf, len, false);

		if(len > (write_buf[0] + 1)) /* 1 msg at a time */
		{
			len  = write_buf[0] + 1;
		}

		if(write_buf[1] == PN544_RSET_CMD) /* RSET frame */
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: RSET command issued by user; clearing the receive buffer\n", dev_name(&client->dev), __func__);
			/* Clear the receive buffer */
			circular_buffer_flush(&info->response);
		}

		ret = pn544_i2c_write(info->i2c_dev, write_buf, len);
	}
out:

	return ret;

}

static long pn544_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct pn544_info *info = container_of(file->private_data,
					       struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;
	unsigned int val;
	int ret = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: cmd = 0x%08x, arg = 0x%08x\n", dev_name(&client->dev), __func__, cmd, arg);

	switch(cmd)
	{
	case PN544_SET_PWR:
		{
			/*
			 * PN544 power control via ioctl
			 * PN544_SET_PWR(0): power off
			 * PN544_SET_PWR(1): power on
			 * PN544_SET_PWR(2): reset and power on with firmware download enabled
			 */
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: PN544_SET_PWR(%lu)\n", dev_name(&client->dev), __func__, arg);
			switch(arg)
			{
			case 0:
				{
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling the controller\n", dev_name(&client->dev), __func__);
					pn544_disable(info);
					msleep(PN544_RESETVEN_TIME);
				}
				break;
			case 1:
				{
					if(atomic_read(&info->pn544_state) == PN544_ST_READY)
					{
						DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Controller already in HCI mode\n", dev_name(&client->dev), __func__);
						break;
					}
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling the controller\n", dev_name(&client->dev), __func__);
					pn544_disable(info);
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enabling the controller in HCI mode\n", dev_name(&client->dev), __func__);
					ret = pn544_enable(info, HCI_MODE, 1);
					if(ret < 0)
					{
						DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): EROR: Error in enabling the controller in HCI mode\n", dev_name(&client->dev), __func__);
						goto out;
					}
				}
				break;
			case 2:
				{
					if(atomic_read(&info->pn544_state) == PN544_ST_FW_READY)
					{
						DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Controller already in FW mode\n", dev_name(&client->dev), __func__);
						break;
					}
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling the controller\n", dev_name(&client->dev), __func__);
					pn544_disable(info);
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enabling the controller in FW mode\n", dev_name(&client->dev), __func__);
					ret = pn544_enable(info, FW_MODE, 1);
					if(ret < 0)
					{
						DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): EROR: Error in enabling the controller in FW mode\n", dev_name(&client->dev), __func__);
						goto out;
					}
				}
				break;
			default:
				{
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Unsupported argument for PN544_SET_PWR\n", dev_name(&client->dev), __func__);
				}
				break;
			}
		}
		break;
	case PN544_GET_FW_MODE:
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(&client->dev), __func__);
			val = (atomic_read(&info->pn544_state) == PN544_ST_FW_READY);
			if(copy_to_user((void __user *)arg, &val, sizeof(val)))
			{
				ret = -EFAULT;
				goto out;
			}
		}
		break;
	case PN544_SET_FW_MODE:
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(&client->dev), __func__);
			if(copy_from_user(&val, (void __user *)arg, sizeof(val)))
			{
				DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Error in copying argument from userspace\n", dev_name(&client->dev), __func__);
				ret = -EFAULT;
				goto out;
			}

			if(val)
			{
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Switching the controller to FW mode\n", dev_name(&client->dev), __func__);
				if(atomic_read(&info->pn544_state) == PN544_ST_FW_READY)
				{
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Controller already in FW mode\n", dev_name(&client->dev), __func__);
					break;
				}
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling the controller\n", dev_name(&client->dev), __func__);
				pn544_disable(info);
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enabling the controller in FW mode\n", dev_name(&client->dev), __func__);
				ret = pn544_enable(info, FW_MODE, 1);
				if(ret < 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): EROR: Error in enabling the controller in FW mode\n", dev_name(&client->dev), __func__);
					goto out;
				}
			}
			else
			{
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Switching the controller to HCI mode\n", dev_name(&client->dev), __func__);
				if (atomic_read(&info->pn544_state) == PN544_ST_READY)
				{
					DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Controller already in HCI mode\n", dev_name(&client->dev), __func__);
					break;
				}
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: disabling the controller\n", dev_name(&client->dev), __func__);
				pn544_disable(info);
				DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enabling the controller in HCI mode\n", dev_name(&client->dev), __func__);
				ret = pn544_enable(info, HCI_MODE, 1);
				if(ret < 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): EROR: Error in enabling the controller in HCI mode\n", dev_name(&client->dev), __func__);
					goto out;
				}
			}
		}
		break;
	default:
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Unsupported ioctl 0x%08x\n", dev_name(&client->dev), __func__, cmd);
			ret = -ENOIOCTLCMD;
		}
		break;
	}
out:

	return ret;
}

static int pn544_open(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;
	int ret = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(&client->dev), __func__);

	if(atomic_read(&info->open_counter) > atomic_read(&info->max_fd))
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: too many open descriptors(%d)\n", dev_name(&client->dev), __func__, atomic_read(&info->open_counter));
		ret = -EBUSY;
		goto out;
	}

	if(atomic_read(&info->open_counter) == 0)
	{
		DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: first open descriptor...enabling the controller\n", dev_name(&client->dev), __func__);
		ret = pn544_enable(info, HCI_MODE, 1);
	}

	atomic_inc(&info->open_counter);
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: now %d open descriptors\n", dev_name(&client->dev), __func__, atomic_read(&info->open_counter));
out:
	return ret;
}

static int pn544_release(struct inode *inode, struct file *file)
{
	struct pn544_info *info = container_of(file->private_data, struct pn544_info, miscdev);
	struct i2c_client *client = info->i2c_dev;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: enter...\n", dev_name(&client->dev), __func__);

	if(atomic_read(&info->open_counter) > 0)
	{
		atomic_dec(&info->open_counter);
		DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: now %d open descriptors\n", dev_name(&client->dev), __func__, atomic_read(&info->open_counter));
	}
	if(atomic_read(&info->open_counter) == 0)
	{
		DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: last open descriptor...disabling the controller\n", dev_name(&client->dev), __func__);
		pn544_disable(info);
	}

	return 0;
}

static const struct file_operations pn544_fops = {
	.owner		= THIS_MODULE,
	.read		= pn544_read,
	.write		= pn544_write,
	.open		= pn544_open,
	.release	= pn544_release,
	.unlocked_ioctl	= pn544_ioctl,
};

#ifdef CONFIG_PM
static int pn544_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn544_info *info = i2c_get_clientdata(client);
	int r = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: suspending...\n", dev_name(&client->dev), __func__);

	switch(atomic_read(&info->pn544_state))
	{
	case PN544_ST_FW_READY:
		{
			/* Do not suspend while upgrading FW, please! */
			DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Cannot suspend while in FW mode\n", dev_name(&client->dev), __func__);
			r = -EPERM;
		}
		break;
	case PN544_ST_READY:
		{
			/*
			 * CHECK: Device should be in standby-mode. No way to check?
			 * Allowing low power mode for the regulator is potentially
			 * dangerous if pn544 does not go to suspension.
			 */
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: suspending while in HCI mode\n", dev_name(&client->dev), __func__);
		}
		break;
	case PN544_ST_COLD:
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: nothing to do...contoller is already powered OFF\n", dev_name(&client->dev), __func__);
		}
		break;
	};
	DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: suspended\n", dev_name(&client->dev), __func__);

	return r;
}
static int pn544_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct pn544_info *info = i2c_get_clientdata(client);
	int ret = 0;

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: resuming...\n", dev_name(&client->dev), __func__);
	switch(atomic_read(&info->pn544_state))
	{
	case PN544_ST_READY:
		{
			/*
			 * CHECK: If regulator low power mode is allowed in
			 * pn544_suspend, we should go back to normal mode
			 * here.
			 */
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: nothing to do...contoller is already powered ON\n", dev_name(&client->dev), __func__);
		}
		break;
	case PN544_ST_COLD:
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: resuming from powered OFF mode\n", dev_name(&client->dev), __func__);
		}
		break;
	case PN544_ST_FW_READY:
		{
			DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: nothing to do...contoller is already powered ON\n", dev_name(&client->dev), __func__);
		}
		break;
	};
	DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: resumed\n", dev_name(&client->dev), __func__);
	return ret;
}
static SIMPLE_DEV_PM_OPS(pn544_pm_ops, pn544_suspend, pn544_resume);
#endif

static int __devinit pn544_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct pn544_info *info;
	struct pn544_nfc_platform_data *pdata;
	int ret = 0;

	DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: probing for pn544 nfc controller @ 0x%02x\n", dev_name(&client->dev), __func__, client->addr);

	pdata = client->dev.platform_data;
	if(!pdata)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: No platform data supplied...exiting\n", dev_name(&client->dev), __func__);
		return -ENODEV;
	}
	/* private data allocation */
	info = kzalloc(sizeof(struct pn544_info), GFP_KERNEL);
	if(!info)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not allocate memory for private structure...exiting\n", dev_name(&client->dev), __func__);
		return -ENOMEM;
	}

	ret = circular_buffer_init(&info->response, PN544_READ_BUF_SIZE);
	if(ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not initialize response buffer...exiting\n", dev_name(&client->dev), __func__);
		goto err_response_buf_alloc;
	}

	ret = gpio_request(pdata->gpio_vbat, "nfc_vbat");
	if(ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not acquire nfc_vbat(gpio %d)...exiting\n", dev_name(&client->dev), __func__, pdata->gpio_vbat);
		goto err_gpio_vbat_req;
	}

	ret = gpio_request(pdata->gpio_ven, "nfc_ven");
	if(ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not acquire nfc_ven(gpio %d)...exiting\n", dev_name(&client->dev), __func__, pdata->gpio_ven);
		goto err_gpio_ven_req;
	}

	ret = gpio_request(pdata->gpio_irq, "nfc_irq");
	if(ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not acquire nfc_irq(gpio %d)...exiting\n", dev_name(&client->dev), __func__, pdata->gpio_irq);
		goto err_gpio_irq_req;
	}
	ret = gpio_direction_input(pdata->gpio_irq);
	if(ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not configure nfc_irq(gpio %d) as input...exiting\n", dev_name(&client->dev), __func__, pdata->gpio_irq);
		goto err_gpio_irq_cfg;
	}

	ret = gpio_request(pdata->gpio_firm, "nfc_fw");
	if(ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not acquire nfc_fw(gpio %d)...exiting\n", dev_name(&client->dev), __func__, pdata->gpio_firm);
		goto err_gpio_firm_req;
	}

	/* Request the nfc power supply */
	info->vdd = regulator_get(&client->dev, "nfc");
	if(IS_ERR(info->vdd))
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not acquire nfc regulator...exiting\n", dev_name(&client->dev), __func__);
		ret = -EBUSY;
		goto err_regulator_get;
	}

//	/* Enable the controller */
//	/* Pull the firmware download pin low */
//	gpio_set_value(pdata->gpio_firm, 0);
//	/* Enable the power */
//	gpio_set_value(pdata->gpio_vbat, 1);
//	regulator_enable(info->vdd);
//	gpio_set_value(pdata->gpio_ven, 1);
//	usleep_range(3000, 4500); /* Boot time is maximum 3ms */
//	atomic_set(&info->pn544_state, PN544_ST_READY);
	atomic_set(&info->pn544_state, PN544_ST_COLD);

	info->i2c_dev = client;
	atomic_set(&info->read_irq, 0);
	atomic_set(&info->max_fd, 1);
	atomic_set(&info->open_counter, 0);
	init_waitqueue_head(&info->read_wait);
	i2c_set_clientdata(client, info);

	/* Prepare our work structure prior to resgistering the interrupt handler */
	INIT_WORK(&info->irq_work, pn544_irq_worker);

	ret = request_threaded_irq(client->irq, NULL, pn544_irq_thread_fn, IRQF_TRIGGER_RISING, PN544_DRIVER_NAME, info);
	if(ret < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not register nfc IRQ handler...exiting\n", dev_name(&client->dev), __func__);
		goto err_irq_hndl_req;
	}
	atomic_set(&info->irq_state, 1);

	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = PN544_DRIVER_NAME;
	info->miscdev.fops = &pn544_fops;
	info->miscdev.parent = &client->dev;
	ret = misc_register(&info->miscdev);
	if(ret < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Could not register misc device...exiting\n", dev_name(&client->dev), __func__);
		goto err_misc_reg;
	}

//	/* Disable the controller till someone starts using it */
//	pn544_disable(info);

	ret = sysfs_create_group(&client->dev.kobj, &pn544_attribute_group);
	if(ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " PN544_TAG ": %s(): ERROR: Unable to create sysfs entries...exiting\n", dev_name(&client->dev), __func__);
		goto err_sys_fs;
	}

	DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: Successfully probed pn544 nfc controller @ 0x%02x\n", dev_name(&client->dev), __func__, client->addr);

	return 0;

err_sys_fs:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Deregistering misc device\n", dev_name(&client->dev), __func__);
	misc_deregister(&info->miscdev);
err_misc_reg:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing IRQ\n", dev_name(&client->dev), __func__);
	free_irq(client->irq, info);
err_irq_hndl_req:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc regulator\n", dev_name(&client->dev), __func__);
	regulator_put(info->vdd);
err_regulator_get:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_fw(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_firm);
	gpio_free(pdata->gpio_firm);
err_gpio_firm_req:
err_gpio_irq_cfg:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_irq(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_irq);
	gpio_free(pdata->gpio_irq);
err_gpio_irq_req:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_ven(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_ven);
	gpio_free(pdata->gpio_ven);
err_gpio_ven_req:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_vbat(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_vbat);
	gpio_free(pdata->gpio_vbat);
err_gpio_vbat_req:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Destroying response buffer\n", dev_name(&client->dev), __func__);
	circular_buffer_destroy(&info->response);
err_response_buf_alloc:
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Freeing private structure\n", dev_name(&client->dev), __func__);
	kfree(info);
	return ret;
}

static __devexit int pn544_remove(struct i2c_client *client)
{
	struct pn544_info *info = i2c_get_clientdata(client);
	struct pn544_nfc_platform_data *pdata = client->dev.platform_data;

	DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: Removing pn544 nfc driver\n", dev_name(&client->dev), __func__);

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Deregistering misc device\n", dev_name(&client->dev), __func__);
	misc_deregister(&info->miscdev);

	/* Disable the controller */
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Disabling controller\n", dev_name(&client->dev), __func__);
	pn544_disable(info);

	/* Cancel the interrupt processor work */
	cancel_work_sync(&info->irq_work);

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing IRQ\n", dev_name(&client->dev), __func__);
	free_irq(client->irq, info);

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc regulator\n", dev_name(&client->dev), __func__);
	regulator_put(info->vdd);
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_fw(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_firm);
	gpio_free(pdata->gpio_firm);
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_irq(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_irq);
	gpio_free(pdata->gpio_irq);
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_ven(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_ven);
	gpio_free(pdata->gpio_ven);
	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Releasing nfc_vbat(gpio %d)\n", dev_name(&client->dev), __func__, pdata->gpio_vbat);
	gpio_free(pdata->gpio_vbat);

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Destroying response buffer\n", dev_name(&client->dev), __func__);
	circular_buffer_destroy(&info->response);

	DBG_PRINT(dbg_level_debug, "%s: " PN544_TAG ": %s(): DEBUG: Freeing private structure\n", dev_name(&client->dev), __func__);
	kfree(info);

	DBG_PRINT(dbg_level_info, "%s: " PN544_TAG ": %s(): INFO: Successfully removed pn544 nfc driver\n", dev_name(&client->dev), __func__);

	return 0;
}

static struct i2c_driver pn544_driver = {
	.driver = {
			.name = PN544_DRIVER_NAME,
#ifdef CONFIG_PM
			.pm = &pn544_pm_ops,
#endif
		},
	.probe = pn544_probe,
	.id_table = pn544_id_table,
	.remove = __devexit_p(pn544_remove),
};

static int __init pn544_init(void)
{
	int ret;

	/* Initialize the cur_dbg_level to error */
	cur_dbg_level = dbg_level_info;

	DBG_PRINT(dbg_level_critical, PN544_TAG ": %s(): INFO: Initializing pn544 nfc driver\n", __func__);

	ret = i2c_add_driver(&pn544_driver);
	if(ret)
	{
		DBG_PRINT(dbg_level_error, PN544_TAG ": %s(): INFO: Failed to initialize pn544 nfc driver\n", __func__);
		return ret;
	}
	return 0;
}

static void __exit pn544_exit(void)
{
	DBG_PRINT(dbg_level_critical, PN544_TAG ": %s(): INFO: Exiting pn544 nfc driver\n", __func__);
	i2c_del_driver(&pn544_driver);
}

module_init(pn544_init);
module_exit(pn544_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION(DRIVER_DESC);

/*
 * drivers/input/touchscreen/liquidware_pcap.c
 *
 * Copyright (c) 2011 Liquidware
 *	Chris Ladden <chris.ladden@liquidware.com>
 *
 * Using code from:
 *  - ads7846.c
 *	Copyright (c) 2005 David Brownell
 *	Copyright (c) 2006 Nokia Corporation
 *  - corgi_ts.c
 *	Copyright (C) 2004-2005 Richard Purdie
 *  - omap_ts.[hc], ads7846.h, ts_osk.c
 *	Copyright (C) 2002 MontaVista Software
 *	Copyright (C) 2004 Texas Instruments
 * 	Copyright (C) 2005 Dirk Behme
 *
 * 	Copyright (C) 2011 Chris Ladden
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  published by the Free Software Foundation.
 */
#define DEBUG 1

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/i2c/liquidware_pcap.h>
#include <linux/gpio.h>

#define TS_POLL_DELAY			1 /* ms delay between samples */
#define TS_POLL_PERIOD			10 /* ms delay between samples */

struct ts_event {
	u8 id;
	bool penDown;
	bool prevPenDown;
	bool isValidSample;
	s8	isValidSampleCnt;
	s8 isValidSampleClampMin;
	s8 isValidSampleClampMax;
	u16 x;
	u16 y;
	u16 prevx;
	u16 prevy;
	u16 sx;
	u16 sy;
	u16 sampleValueMax;
};

static struct ts_event tc[] = {
	 {
		 .id = 0,
		 .penDown = false,
		 .prevPenDown = true,
		 .isValidSample = false,
		 .isValidSampleCnt = 0,
		 .isValidSampleClampMin = 0,
		 .isValidSampleClampMax = 1,
		 .x = 0,
		 .y = 0,
		 .prevx = 0,
		 .prevy = 0,
		 .sampleValueMax = 2047,
	 },
	 {
		 .id = 1,
		 .penDown = false,
		 .prevPenDown = true,
		 .isValidSample = false,
		 .isValidSampleCnt = 0,
		 .isValidSampleClampMin = 0,
		 .isValidSampleClampMax = 1,
		 .x = 0,
		 .y = 0,
		 .prevx = 0,
		 .prevy = 0,
		 .sampleValueMax = 2047,
	 },
};

struct liquidware_pcap {
	struct input_dev	*input;
	char			phys[32];
	struct delayed_work	work;

	struct i2c_client	*client;

	u16			model;
	u16			x_plate_ohms;

	bool			pendown;
	int			irq;

	int			(*get_pendown_state)(void);
	void		(*clear_penirq)(void);
	void		(*perform_reset)(void);
};

static inline int liquidware_pcap_xfer(struct liquidware_pcap *tsc)
{
	s32 data;

	data = i2c_smbus_read_byte_data(tsc->client,'U');
	if (data < 0) {
		dev_err(&tsc->client->dev, "i2c io error: %d\n", data);
		return data;
	}

	dev_dbg(&tsc->client->dev, "data: 0x%x\n", data);

	return data;
}

#define POLY    (0x1070U << 3)
static u8 crc8(u16 data)
{
	int i;

	for(i = 0; i < 8; i++) {
		if (data & 0x8000)
			data = data ^ POLY;
		data = data << 1;
	}
	return (u8)(data >> 8);
}

/* Incremental CRC8 over count bytes in the array pointed to by p */
static u8 i2c_smbus_pec(u8 crc, u8 *p, size_t count)
{
	int i;

	for(i = 0; i < count; i++)
		crc = crc8((crc ^ p[i]) << 8);
	return crc;
}

/* Assume a 7-bit address, which is reasonable for SMBus */
static u8 i2c_smbus_msg_pec(u8 pec, struct i2c_msg *msg)
{
	/* The address will be sent first */
	u8 addr = (msg->addr << 1) | !!(msg->flags & I2C_M_RD);
	pec = i2c_smbus_pec(pec, &addr, 1);

	/* The data buffer follows */
	return i2c_smbus_pec(pec, msg->buf, msg->len);
}

/* Used for write only transactions */
static inline void i2c_smbus_add_pec(struct i2c_msg *msg)
{
	msg->buf[msg->len] = i2c_smbus_msg_pec(0, msg);
	msg->len++;
}

#define I2C_SMBUS_NOSTART_BYTE 9

/* Simulate a SMBus command using the i2c protocol
   No checking of parameters is done!  */
static s32 pcap_i2c_smbus_xfer_emulated(struct i2c_adapter * adapter, u16 addr,
                                   unsigned short flags,
                                   char read_write, u8 command, int size,
                                   union i2c_smbus_data * data)
{
	/* So we need to generate a series of msgs. In the case of writing, we
	  need to use only one message; when reading, we need two. We initialize
	  most things with sane defaults, to keep the code below somewhat
	  simpler. */
	unsigned char msgbuf0[I2C_SMBUS_BLOCK_MAX+3];
	unsigned char msgbuf1[I2C_SMBUS_BLOCK_MAX+2];
	int num = read_write == I2C_SMBUS_READ?2:1;
	struct i2c_msg msg[2] = { { addr, flags, 1, msgbuf0 },				//write msg?
	                          { addr, flags | I2C_M_RD, 0, msgbuf1 }	//read msg?
	                        };
	int i;
	u8 partial_pec = 0;
	int status;

	msgbuf0[0] = command;
	switch(size) {
	case I2C_SMBUS_QUICK:
		msg[0].len = 0;
		/* Special case: The read/write field is used as data */
		msg[0].flags = flags | (read_write == I2C_SMBUS_READ ?
					I2C_M_RD : 0);
		num = 1;
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_READ) {
			/* Special case: only a read! */
			msg[0].flags = I2C_M_RD | flags;
			num = 1;
		}
		break;
	case I2C_SMBUS_NOSTART_BYTE:
		if (read_write == I2C_SMBUS_READ) {
			/* Special case: only a read! */
			msg[0].flags = I2C_M_RD | I2C_M_NOSTART | flags;
			num = 1;
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 1;
		else {
			msg[0].len = 2;
			msgbuf0[1] = data->byte;
		}
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_READ)
			msg[1].len = 2;
		else {
			msg[0].len=3;
			msgbuf0[1] = data->word & 0xff;
			msgbuf0[2] = data->word >> 8;
		}
		break;
	case I2C_SMBUS_PROC_CALL:
		num = 2; /* Special case */
		read_write = I2C_SMBUS_READ;
		msg[0].len = 3;
		msg[1].len = 2;
		msgbuf0[1] = data->word & 0xff;
		msgbuf0[2] = data->word >> 8;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].flags |= I2C_M_RECV_LEN;
			msg[1].len = 1; /* block length will be added by
					   the underlying bus driver */
		} else {
			msg[0].len = data->block[0] + 2;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 2) {
				dev_err(&adapter->dev,
					"Invalid block write size %d\n",
					data->block[0]);
				return -EINVAL;
			}
			for (i = 1; i < msg[0].len; i++)
				msgbuf0[i] = data->block[i-1];
		}
		break;
	case I2C_SMBUS_BLOCK_PROC_CALL:
		num = 2; /* Another special case */
		read_write = I2C_SMBUS_READ;
		if (data->block[0] > I2C_SMBUS_BLOCK_MAX) {
			dev_err(&adapter->dev,
				"Invalid block write size %d\n",
				data->block[0]);
			return -EINVAL;
		}
		msg[0].len = data->block[0] + 2;
		for (i = 1; i < msg[0].len; i++)
			msgbuf0[i] = data->block[i-1];
		msg[1].flags |= I2C_M_RECV_LEN;
		msg[1].len = 1; /* block length will be added by
				   the underlying bus driver */
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_READ) {
			msg[1].len = data->block[0];
			msg[0].len = 0; 				//don't write data
			num = 1;						//only 1 read transaction
		} else {
			msg[0].len = data->block[0] + 1;
			if (msg[0].len > I2C_SMBUS_BLOCK_MAX + 1) {
				dev_err(&adapter->dev,
					"Invalid block write size %d\n",
					data->block[0]);
				return -EINVAL;
			}
			for (i = 1; i <= data->block[0]; i++)
				msgbuf0[i] = data->block[i];
		}
		break;
	default:
		dev_err(&adapter->dev, "Unsupported transaction %d\n", size);
		return -EOPNOTSUPP;
	}

	i = ((flags & I2C_CLIENT_PEC) && size != I2C_SMBUS_QUICK
				      && size != I2C_SMBUS_I2C_BLOCK_DATA);
	if (i) {
		/* Compute PEC if first message is a write */
		if (!(msg[0].flags & I2C_M_RD)) {
			if (num == 1) /* Write only */
				i2c_smbus_add_pec(&msg[0]);
			else /* Write followed by read */
				partial_pec = i2c_smbus_msg_pec(0, &msg[0]);
		}
		/* Ask for PEC if last message is a read */
		if (msg[num-1].flags & I2C_M_RD)
			msg[num-1].len++;
	}

	dev_dbg(&adapter->dev,"DEBUG: msg[0].len=%d, msg[1].len=%d, msgbuf0[0]=%d, msgbuf0[1]=%d, msgbuf1[0]=%d, msgbuf1[1]=%d\n",
			msg[0].len,
			msg[1].len,
			msgbuf0[0],
			msgbuf0[1],
			msgbuf1[0],
			msgbuf1[1]);

	/* CML Changed msg to only read: msg[1], and num=1 */
	status = i2c_transfer(adapter, &msg[1], num);
	if (status < 0)
		return status;

	/* Check PEC if last message is a read */
	#if 0
	if (i && (msg[num-1].flags & I2C_M_RD)) {
		status = i2c_smbus_check_pec(partial_pec, &msg[num-1]);
		if (status < 0)
			return status;
	}
	#endif

	if (read_write == I2C_SMBUS_READ)
		switch(size) {
			case I2C_SMBUS_BYTE:
				data->byte = msgbuf0[0];
				break;
			case I2C_SMBUS_NOSTART_BYTE:
				data->byte = msgbuf0[0];
				break;
			case I2C_SMBUS_BYTE_DATA:
				data->byte = msgbuf1[0];
				break;
			case I2C_SMBUS_WORD_DATA:
			case I2C_SMBUS_PROC_CALL:
				data->word = msgbuf1[0] | (msgbuf1[1] << 8);
				break;
			case I2C_SMBUS_I2C_BLOCK_DATA:
				for (i = 0; i < data->block[0]; i++)
					data->block[i+1] = msgbuf1[i];
				break;
			case I2C_SMBUS_BLOCK_DATA:
			case I2C_SMBUS_BLOCK_PROC_CALL:
				for (i = 0; i < msgbuf1[0] + 1; i++)
					data->block[i] = msgbuf1[i];
				break;
		}
	return 0;
}

/**
 * i2c_smbus_read_byte - SMBus "receive byte" protocol
 * @client: Handle to slave device
 *
 * This executes the SMBus "receive byte" protocol, returning negative errno
 * else the byte received from the device.
 */
s32 i2c_smbus_read_block(struct i2c_client *client,
				  u8 length, u8 *values)
{
	union i2c_smbus_data data;
	int status;

	if (length > I2C_SMBUS_BLOCK_MAX)
		length = I2C_SMBUS_BLOCK_MAX;
	data.block[0] = length;
	status = pcap_i2c_smbus_xfer_emulated(client->adapter, client->addr, client->flags,
				I2C_SMBUS_READ, 'U',
				I2C_SMBUS_I2C_BLOCK_DATA, &data);
	if (status < 0)
		return status;

	memcpy(values, &data.block[1], data.block[0]);
	return data.block[0];
}

/**
 * i2c_smbus_read_byte - SMBus "receive byte" protocol
 * @client: Handle to slave device
 *
 * This executes the SMBus "receive byte" protocol, returning negative errno
 * else the byte received from the device.
 */
s32 i2c_smbus_read_nostart_byte(struct i2c_client *client)
{
	union i2c_smbus_data data;
	int status;

	status = pcap_i2c_smbus_xfer_emulated(client->adapter, client->addr, client->flags,
				I2C_SMBUS_READ, 0,
				I2C_SMBUS_NOSTART_BYTE, &data);
	return (status < 0) ? status : data.byte;
}

static void liquidware_pcap_read_values(struct liquidware_pcap *tsc, struct ts_event *tc)
{
	s32 result = 0;
	u8 v[32];

	result = i2c_smbus_read_block(tsc->client, 9, &v[0]);
	if (result < 0) {
		/* Reseting Touchscreen Controller */
		printk(KERN_ERR "i2c io error: %d, reseting device.\n", result);
		tsc->perform_reset();
		return;
	}

	/* Save first mt point */
	tc[0].sx = ((u8)v[0] << 8) | (u8)v[1];
	tc[0].sy = ((u8)v[2] << 8) | (u8)v[3];

	/* Save second mt point */
	tc[1].sx = ((u8)v[4] << 8) | (u8)v[5];
	tc[1].sy = ((u8)v[6] << 8) | (u8)v[7];
}

static void liquidware_pcap_work(struct work_struct *work)
{
	struct liquidware_pcap *ts =
		container_of(to_delayed_work(work), struct liquidware_pcap, work);
	struct input_dev *input = ts->input;
	u8 id;
	char msg[512];

	memset(msg,0, sizeof(msg));

	//read the entire packet of events
	liquidware_pcap_read_values(ts, &tc[0]);

	//handle the mt events
	for (id=0; id<ARRAY_SIZE(tc); id++) {
		struct ts_event *t = &tc[id];
		bool inrange;

		//range check
		//filter pen down/up
		//report pen down location or up
		inrange = (t->sx > 0) && (t->sx < t->sampleValueMax) &&
				(t->sy > 0) && (t->sy < t->sampleValueMax);

		if (inrange) {
			t->x = t->sx;
			t->y = t->sy;

			//ratchet pen down
			if (t->isValidSampleCnt > t->isValidSampleClampMin) {
				t->isValidSampleCnt--;
			}
		} else {
			//ratchet pen up
			if (t->isValidSampleCnt < t->isValidSampleClampMax) {
				t->isValidSampleCnt++;
			}
		}

		if (t->isValidSampleCnt == t->isValidSampleClampMin)
			t->penDown = true;
		if (t->isValidSampleCnt == t->isValidSampleClampMax)
			t->penDown = false;

		sprintf(msg,"%s id:%d", msg, id);
		if (t->penDown) {
			//report mt DOWN event
			sprintf(msg,"%s TOUCH_DOWN [x,y]=[%d,%d]", msg, t->x, t->y);

			if (id==0) {
				input_report_key(input, BTN_TOUCH, 1);
				input_report_abs(input, ABS_X, t->x);
				input_report_abs(input, ABS_Y, t->y);
				input_report_abs(input, ABS_PRESSURE, 1);
				//input_sync(input);
			}

    		input_report_abs(input, ABS_MT_TRACKING_ID, id);
    		input_report_abs(input, ABS_MT_TOUCH_MAJOR, 1);
    		input_report_abs(input, ABS_MT_POSITION_X, t->x);
    		input_report_abs(input, ABS_MT_POSITION_Y, t->y);

    		input_mt_sync(input);
		} else {
			//debounce
			if (t->prevPenDown) {
				sprintf(msg,"%s TOUCH_UP", msg);
				if (id==0) {
					input_report_key(input, BTN_TOUCH, 0);
					input_report_abs(input, ABS_PRESSURE, 0);
					//input_sync(input);
				}

				input_report_abs(input, ABS_MT_TRACKING_ID, id);
				input_event(input, EV_ABS, ABS_MT_TOUCH_MAJOR, 0);
				input_mt_sync(input);
			}
		}

		//store the previous value
		t->prevPenDown = t->penDown;
	}

	//printk(KERN_ERR "liquidware_pcap %s\n", msg);
	input_sync(input);

	//if (ts->pendown)
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_PERIOD));
	//else
	//	enable_irq(ts->irq);
}

static irqreturn_t liquidware_pcap_irq(int irq, void *handle)
{
	struct liquidware_pcap *ts = handle;

	if (!ts->get_pendown_state || likely(ts->get_pendown_state())) {
		disable_irq_nosync(ts->irq);
		schedule_delayed_work(&ts->work,
				      msecs_to_jiffies(TS_POLL_DELAY));
	}

	if (ts->clear_penirq)
		ts->clear_penirq();

	return IRQ_HANDLED;
}

static void liquidware_pcap_free_irq(struct liquidware_pcap *ts)
{
	free_irq(ts->irq, ts);
	if (cancel_delayed_work_sync(&ts->work)) {
		/*
		 * Work was pending, therefore we need to enable
		 * IRQ here to balance the disable_irq() done in the
		 * interrupt handler.
		 */
		enable_irq(ts->irq);
	}
}

static int __devinit liquidware_pcap_probe(struct i2c_client *client,
				   const struct i2c_device_id *id)
{
	struct liquidware_pcap *ts;
	struct liquidware_pcap_platform_data *pdata = pdata = client->dev.platform_data;
	struct input_dev *input_dev;
	int err;

	if (!pdata) {
		dev_err(&client->dev, "platform data is required!\n");
		return -EINVAL;
	}

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_READ_WORD_DATA))
		return -EIO;

	ts = kzalloc(sizeof(struct liquidware_pcap), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev) {
		err = -ENOMEM;
		goto err_free_mem;
	}

	ts->client = client;
	ts->irq = client->irq;
	ts->input = input_dev;
	INIT_DELAYED_WORK(&ts->work, liquidware_pcap_work);

	ts->model             = pdata->model;
	ts->x_plate_ohms      = pdata->x_plate_ohms;
	ts->get_pendown_state = pdata->get_pendown_state;
	ts->clear_penirq      = pdata->clear_penirq;
	ts->perform_reset	  = pdata->perform_reset;

	snprintf(ts->phys, sizeof(ts->phys),
		 "%s/input0", dev_name(&client->dev));

	input_dev->name = "LIQUIDWARE_PCAP Touchscreen";
	input_dev->phys = ts->phys;
	input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(ABS_X, input_dev->absbit);
	__set_bit(ABS_Y, input_dev->absbit);
	__set_bit(ABS_MT_TRACKING_ID, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_X, input_dev->absbit);
	__set_bit(ABS_MT_POSITION_Y, input_dev->absbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X, 0, tc[0].sampleValueMax, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, tc[0].sampleValueMax, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, tc[0].sampleValueMax, 0, 0);

	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, ARRAY_SIZE(tc), 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, tc[0].sampleValueMax, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, tc[0].sampleValueMax, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, 0xF, 0, 0);


	if (pdata->init_platform_hw)
		pdata->init_platform_hw(pdata);

	if (request_irq(ts->irq, liquidware_pcap_irq, IRQF_TRIGGER_FALLING,
			client->dev.driver->name, ts)) {
		dev_info(&client->dev,
			"trying pin change workaround on irq %d\n", ts->irq);
		err = request_irq(ts->irq, liquidware_pcap_irq,
				  IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				  client->dev.driver->name, ts);
		if (err) {
			dev_err(&client->dev, "irq %d busy?\n", ts->irq);
			goto err_free_mem;
		}
	}

	err = input_register_device(input_dev);
	if (err)
		goto err_free_irq;

	i2c_set_clientdata(client, ts);

	printk(KERN_INFO "liquidware_pcap_probe: Scheduling delayed work\n");
	schedule_delayed_work(&ts->work,
						  msecs_to_jiffies(TS_POLL_DELAY));

	return 0;

 err_free_irq:
	liquidware_pcap_free_irq(ts);
	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();
 err_free_mem:
	input_free_device(input_dev);
	kfree(ts);
	return err;
}

static int __devexit liquidware_pcap_remove(struct i2c_client *client)
{
	struct liquidware_pcap	*ts = i2c_get_clientdata(client);
	struct liquidware_pcap_platform_data *pdata = client->dev.platform_data;

	liquidware_pcap_free_irq(ts);

	if (pdata->exit_platform_hw)
		pdata->exit_platform_hw();

	input_unregister_device(ts->input);
	kfree(ts);

	return 0;
}

static const struct i2c_device_id liquidware_pcap_idtable[] = {
	{ "liquidware_pcap", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, liquidware_pcap_idtable);

static struct i2c_driver liquidware_pcap_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "liquidware_pcap"
	},
	.id_table	= liquidware_pcap_idtable,
	.probe		= liquidware_pcap_probe,
	.remove		= __devexit_p(liquidware_pcap_remove),
};

static int __init liquidware_pcap_init(void)
{
	return i2c_add_driver(&liquidware_pcap_driver);
}

static void __exit liquidware_pcap_exit(void)
{
	i2c_del_driver(&liquidware_pcap_driver);
}

module_init(liquidware_pcap_init);
module_exit(liquidware_pcap_exit);

MODULE_AUTHOR("Chris Ladden <chris.ladden@liquidware.com>");
MODULE_DESCRIPTION("Liquidware PCap TouchScreen Driver");
MODULE_LICENSE("GPL");

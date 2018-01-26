/*
 * ddbridge.c: Digital Devices PCIe bridge driver
 *
 * Copyright (C) 2010-2011 Digital Devices GmbH
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 only, as published by the Free Software Foundation.
 *
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * To obtain the license, point your browser to
 * http://www.gnu.org/copyleft/gpl.html
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/pci.h>
#include <linux/pci_ids.h>
#include <linux/timer.h>
#include <linux/i2c.h>
#include <linux/swab.h>
#include <linux/vmalloc.h>
#include "ddbridge.h"

#include "ddbridge-regs.h"

#include "tda18271c2dd.h"
#include "stv6110x.h"
#include "stv090x.h"
#include "lnbh24.h"
#include "drxk.h"
#include "stv0367.h"
#include "stv0367_priv.h"
#include "cxd2841er.h"
#include "tda18212.h"

static int xo2_speed = 2;
module_param(xo2_speed, int, 0444);
MODULE_PARM_DESC(xo2_speed, "default transfer speed for xo2 based duoflex, 0=55,1=75,2=90,3=104 MBit/s, default=2, use attribute to change for individual cards");

DVB_DEFINE_MOD_OPT_ADAPTER_NR(adapter_nr);

/* MSI had problems with lost interrupts, fixed but needs testing */
#undef CONFIG_PCI_MSI

/******************************************************************************/

static int i2c_io(struct i2c_adapter *adapter, u8 adr,
		  u8 *wbuf, u32 wlen, u8 *rbuf, u32 rlen)
{
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = wbuf, .len   = wlen },
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = rbuf,  .len   = rlen } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_write(struct i2c_adapter *adap, u8 adr, u8 *data, int len)
{
	struct i2c_msg msg = {.addr = adr, .flags = 0,
			      .buf = data, .len = len};

	return (i2c_transfer(adap, &msg, 1) == 1) ? 0 : -1;
}

static int i2c_read(struct i2c_adapter *adapter, u8 adr, u8 *val)
{
	struct i2c_msg msgs[1] = {{.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = 1 } };
	return (i2c_transfer(adapter, msgs, 1) == 1) ? 0 : -1;
}

static int i2c_read_regs(struct i2c_adapter *adapter,
			 u8 adr, u8 reg, u8 *val, u8 len)
{
	struct i2c_msg msgs[2] = {{.addr = adr,  .flags = 0,
				   .buf  = &reg, .len   = 1 },
				  {.addr = adr,  .flags = I2C_M_RD,
				   .buf  = val,  .len   = len } };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_read_reg(struct i2c_adapter *adapter, u8 adr, u8 reg, u8 *val)
{
	return i2c_read_regs(adapter, adr, reg, val, 1);
}

static int i2c_read_reg16(struct i2c_adapter *adapter, u8 adr,
			  u16 reg, u8 *val)
{
	u8 msg[2] = {reg>>8, reg&0xff};
	struct i2c_msg msgs[2] = {{.addr = adr, .flags = 0,
				   .buf  = msg, .len   = 2},
				  {.addr = adr, .flags = I2C_M_RD,
				   .buf  = val, .len   = 1} };
	return (i2c_transfer(adapter, msgs, 2) == 2) ? 0 : -1;
}

static int i2c_write_reg(struct i2c_adapter *adap, u8 adr,
			 u8 reg, u8 val)
{
	u8 msg[2] = {reg, val};

	return i2c_write(adap, adr, msg, 2);
}

static inline u32 safe_ddbreadl(struct ddb *dev, u32 adr)
{
	u32 val = ddbreadl(adr);

	/* (ddb)readl returns (uint)-1 (all bits set) on failure, catch that */
	if (val == ~0) {
		dev_err(&dev->pdev->dev, "ddbreadl failure, adr=%08x\n", adr);
		return 0;
	}

	return val;
}

static int ddb_i2c_cmd(struct ddb_i2c *i2c, u32 adr, u32 cmd)
{
	struct ddb *dev = i2c->dev;
	long stat;
	u32 val;

	i2c->done = 0;
	ddbwritel((adr << 9) | cmd, i2c->regs + I2C_COMMAND);
	stat = wait_event_timeout(i2c->wq, i2c->done == 1, HZ);
	if (stat == 0) {
		dev_err(&dev->pdev->dev, "I2C timeout\n");
		{ /* MSI debugging*/
			u32 istat = ddbreadl(INTERRUPT_STATUS);
			dev_err(&dev->pdev->dev, "IRS %08x\n", istat);
			ddbwritel(istat, INTERRUPT_ACK);
		}
		return -EIO;
	}
	val = ddbreadl(i2c->regs+I2C_COMMAND);
	if (val & 0x70000)
		return -EIO;
	return 0;
}

static int ddb_i2c_master_xfer(struct i2c_adapter *adapter,
			       struct i2c_msg msg[], int num)
{
	struct ddb_i2c *i2c = (struct ddb_i2c *)i2c_get_adapdata(adapter);
	struct ddb *dev = i2c->dev;
	u8 addr = 0;

	if (num)
		addr = msg[0].addr;

	if (num == 2 && msg[1].flags & I2C_M_RD &&
	    !(msg[0].flags & I2C_M_RD)) {
		memcpy_toio(dev->regs + I2C_TASKMEM_BASE + i2c->wbuf,
			    msg[0].buf, msg[0].len);
		ddbwritel(msg[0].len|(msg[1].len << 16),
			  i2c->regs+I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 1)) {
			memcpy_fromio(msg[1].buf,
				      dev->regs + I2C_TASKMEM_BASE + i2c->rbuf,
				      msg[1].len);
			return num;
		}
	}

	if (num == 1 && !(msg[0].flags & I2C_M_RD)) {
		ddbcpyto(I2C_TASKMEM_BASE + i2c->wbuf, msg[0].buf, msg[0].len);
		ddbwritel(msg[0].len, i2c->regs + I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 2))
			return num;
	}
	if (num == 1 && (msg[0].flags & I2C_M_RD)) {
		ddbwritel(msg[0].len << 16, i2c->regs + I2C_TASKLENGTH);
		if (!ddb_i2c_cmd(i2c, addr, 3)) {
			ddbcpyfrom(msg[0].buf,
				   I2C_TASKMEM_BASE + i2c->rbuf, msg[0].len);
			return num;
		}
	}
	return -EIO;
}


static u32 ddb_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_SMBUS_EMUL;
}

static struct i2c_algorithm ddb_i2c_algo = {
	.master_xfer   = ddb_i2c_master_xfer,
	.functionality = ddb_i2c_functionality,
};

static void ddb_i2c_release(struct ddb *dev)
{
	int i;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;

	for (i = 0; i < dev->info->port_num; i++) {
		i2c = &dev->i2c[i];
		adap = &i2c->adap;
		i2c_del_adapter(adap);
	}
}

static int ddb_i2c_init(struct ddb *dev)
{
	int i, j, stat = 0;
	struct ddb_i2c *i2c;
	struct i2c_adapter *adap;

	for (i = 0; i < dev->info->port_num; i++) {
		i2c = &dev->i2c[i];
		i2c->dev = dev;
		i2c->nr = i;
		i2c->wbuf = i * (I2C_TASKMEM_SIZE / 4);
		i2c->rbuf = i2c->wbuf + (I2C_TASKMEM_SIZE / 8);
		i2c->regs = 0x80 + i * 0x20;
		ddbwritel(I2C_SPEED_100, i2c->regs + I2C_TIMING);
		ddbwritel((i2c->rbuf << 16) | i2c->wbuf,
			  i2c->regs + I2C_TASKADDRESS);
		init_waitqueue_head(&i2c->wq);

		adap = &i2c->adap;
		i2c_set_adapdata(adap, i2c);
#ifdef I2C_ADAP_CLASS_TV_DIGITAL
		adap->class = I2C_ADAP_CLASS_TV_DIGITAL|I2C_CLASS_TV_ANALOG;
#else
#ifdef I2C_CLASS_TV_ANALOG
		adap->class = I2C_CLASS_TV_ANALOG;
#endif
#endif
		strcpy(adap->name, "ddbridge");
		adap->algo = &ddb_i2c_algo;
		adap->algo_data = (void *)i2c;
		adap->dev.parent = &dev->pdev->dev;
		stat = i2c_add_adapter(adap);
		if (stat)
			break;
	}
	if (stat)
		for (j = 0; j < i; j++) {
			i2c = &dev->i2c[j];
			adap = &i2c->adap;
			i2c_del_adapter(adap);
		}
	return stat;
}


/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#if 0
static void set_table(struct ddb *dev, u32 off,
		      dma_addr_t *pbuf, u32 num)
{
	u32 i, base;
	u64 mem;

	base = DMA_BASE_ADDRESS_TABLE + off;
	for (i = 0; i < num; i++) {
		mem = pbuf[i];
		ddbwritel(mem & 0xffffffff, base + i * 8);
		ddbwritel(mem >> 32, base + i * 8 + 4);
	}
}
#endif

static void ddb_address_table(struct ddb *dev)
{
	u32 i, j, base;
	u64 mem;
	dma_addr_t *pbuf;

	for (i = 0; i < dev->info->port_num * 2; i++) {
		base = DMA_BASE_ADDRESS_TABLE + i * 0x100;
		pbuf = dev->input[i].pbuf;
		for (j = 0; j < dev->input[i].dma_buf_num; j++) {
			mem = pbuf[j];
			ddbwritel(mem & 0xffffffff, base + j * 8);
			ddbwritel(mem >> 32, base + j * 8 + 4);
		}
	}
	for (i = 0; i < dev->info->port_num; i++) {
		base = DMA_BASE_ADDRESS_TABLE + 0x800 + i * 0x100;
		pbuf = dev->output[i].pbuf;
		for (j = 0; j < dev->output[i].dma_buf_num; j++) {
			mem = pbuf[j];
			ddbwritel(mem & 0xffffffff, base + j * 8);
			ddbwritel(mem >> 32, base + j * 8 + 4);
		}
	}
}

static void io_free(struct pci_dev *pdev, u8 **vbuf,
		    dma_addr_t *pbuf, u32 size, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		if (vbuf[i]) {
			pci_free_consistent(pdev, size, vbuf[i], pbuf[i]);
			vbuf[i] = NULL;
		}
	}
}

static int io_alloc(struct pci_dev *pdev, u8 **vbuf,
		    dma_addr_t *pbuf, u32 size, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		vbuf[i] = pci_alloc_consistent(pdev, size, &pbuf[i]);
		if (!vbuf[i])
			return -ENOMEM;
	}
	return 0;
}

static int ddb_buffers_alloc(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			if (io_alloc(dev->pdev, port->input[0]->vbuf,
				     port->input[0]->pbuf,
				     port->input[0]->dma_buf_size,
				     port->input[0]->dma_buf_num) < 0)
				return -1;
			if (io_alloc(dev->pdev, port->input[1]->vbuf,
				     port->input[1]->pbuf,
				     port->input[1]->dma_buf_size,
				     port->input[1]->dma_buf_num) < 0)
				return -1;
			break;
		case DDB_PORT_CI:
			if (io_alloc(dev->pdev, port->input[0]->vbuf,
				     port->input[0]->pbuf,
				     port->input[0]->dma_buf_size,
				     port->input[0]->dma_buf_num) < 0)
				return -1;
			if (io_alloc(dev->pdev, port->output->vbuf,
				     port->output->pbuf,
				     port->output->dma_buf_size,
				     port->output->dma_buf_num) < 0)
				return -1;
			break;
		default:
			break;
		}
	}
	ddb_address_table(dev);
	return 0;
}

static void ddb_buffers_free(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		io_free(dev->pdev, port->input[0]->vbuf,
			port->input[0]->pbuf,
			port->input[0]->dma_buf_size,
			port->input[0]->dma_buf_num);
		io_free(dev->pdev, port->input[1]->vbuf,
			port->input[1]->pbuf,
			port->input[1]->dma_buf_size,
			port->input[1]->dma_buf_num);
		io_free(dev->pdev, port->output->vbuf,
			port->output->pbuf,
			port->output->dma_buf_size,
			port->output->dma_buf_num);
	}
}

static void ddb_input_start(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;

	spin_lock_irq(&input->lock);
	input->cbuf = 0;
	input->coff = 0;

	/* reset */
	ddbwritel(0, TS_INPUT_CONTROL(input->nr));
	ddbwritel(2, TS_INPUT_CONTROL(input->nr));
	ddbwritel(0, TS_INPUT_CONTROL(input->nr));

	ddbwritel((1 << 16) |
		  (input->dma_buf_num << 11) |
		  (input->dma_buf_size >> 7),
		  DMA_BUFFER_SIZE(input->nr));
	ddbwritel(0, DMA_BUFFER_ACK(input->nr));

	ddbwritel(1, DMA_BASE_WRITE);
	ddbwritel(3, DMA_BUFFER_CONTROL(input->nr));
	ddbwritel(9, TS_INPUT_CONTROL(input->nr));
	input->running = 1;
	spin_unlock_irq(&input->lock);
}

static void ddb_input_stop(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;

	spin_lock_irq(&input->lock);
	ddbwritel(0, TS_INPUT_CONTROL(input->nr));
	ddbwritel(0, DMA_BUFFER_CONTROL(input->nr));
	input->running = 0;
	spin_unlock_irq(&input->lock);
}

static void ddb_output_start(struct ddb_output *output)
{
	struct ddb *dev = output->port->dev;

	spin_lock_irq(&output->lock);
	output->cbuf = 0;
	output->coff = 0;
	ddbwritel(0, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(2, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(0, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(0x3c, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel((1 << 16) |
		  (output->dma_buf_num << 11) |
		  (output->dma_buf_size >> 7),
		  DMA_BUFFER_SIZE(output->nr + 8));
	ddbwritel(0, DMA_BUFFER_ACK(output->nr + 8));

	ddbwritel(1, DMA_BASE_READ);
	ddbwritel(3, DMA_BUFFER_CONTROL(output->nr + 8));
	/* ddbwritel(0xbd, TS_OUTPUT_CONTROL(output->nr)); */
	ddbwritel(0x1d, TS_OUTPUT_CONTROL(output->nr));
	output->running = 1;
	spin_unlock_irq(&output->lock);
}

static void ddb_output_stop(struct ddb_output *output)
{
	struct ddb *dev = output->port->dev;

	spin_lock_irq(&output->lock);
	ddbwritel(0, TS_OUTPUT_CONTROL(output->nr));
	ddbwritel(0, DMA_BUFFER_CONTROL(output->nr + 8));
	output->running = 0;
	spin_unlock_irq(&output->lock);
}

static u32 ddb_output_free(struct ddb_output *output)
{
	u32 idx, off, stat = output->stat;
	s32 diff;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (output->cbuf != idx) {
		if ((((output->cbuf + 1) % output->dma_buf_num) == idx) &&
		    (output->dma_buf_size - output->coff <= 188))
			return 0;
		return 188;
	}
	diff = off - output->coff;
	if (diff <= 0 || diff > 188)
		return 188;
	return 0;
}

static ssize_t ddb_output_write(struct ddb_output *output,
				const __user u8 *buf, size_t count)
{
	struct ddb *dev = output->port->dev;
	u32 idx, off, stat = output->stat;
	u32 left = count, len;

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	while (left) {
		len = output->dma_buf_size - output->coff;
		if ((((output->cbuf + 1) % output->dma_buf_num) == idx) &&
		    (off == 0)) {
			if (len <= 188)
				break;
			len -= 188;
		}
		if (output->cbuf == idx) {
			if (off > output->coff) {
#if 1
				len = off - output->coff;
				len -= (len % 188);
				if (len <= 188)

#endif
					break;
				len -= 188;
			}
		}
		if (len > left)
			len = left;
		if (copy_from_user(output->vbuf[output->cbuf] + output->coff,
				   buf, len))
			return -EIO;
		left -= len;
		buf += len;
		output->coff += len;
		if (output->coff == output->dma_buf_size) {
			output->coff = 0;
			output->cbuf = ((output->cbuf + 1) % output->dma_buf_num);
		}
		ddbwritel((output->cbuf << 11) | (output->coff >> 7),
			  DMA_BUFFER_ACK(output->nr + 8));
	}
	return count - left;
}

static u32 ddb_input_avail(struct ddb_input *input)
{
	struct ddb *dev = input->port->dev;
	u32 idx, off, stat = input->stat;
	u32 ctrl = ddbreadl(DMA_BUFFER_CONTROL(input->nr));

	idx = (stat >> 11) & 0x1f;
	off = (stat & 0x7ff) << 7;

	if (ctrl & 4) {
		dev_err(&dev->pdev->dev, "IA %d %d %08x\n", idx, off, ctrl);
		ddbwritel(input->stat, DMA_BUFFER_ACK(input->nr));
		return 0;
	}
	if (input->cbuf != idx)
		return 188;
	return 0;
}

static ssize_t ddb_input_read(struct ddb_input *input, __user u8 *buf, size_t count)
{
	struct ddb *dev = input->port->dev;
	u32 left = count;
	u32 idx, free, stat = input->stat;
	int ret;

	idx = (stat >> 11) & 0x1f;

	while (left) {
		if (input->cbuf == idx)
			return count - left;
		free = input->dma_buf_size - input->coff;
		if (free > left)
			free = left;
		ret = copy_to_user(buf, input->vbuf[input->cbuf] +
				   input->coff, free);
		if (ret)
			return -EFAULT;
		input->coff += free;
		if (input->coff == input->dma_buf_size) {
			input->coff = 0;
			input->cbuf = (input->cbuf+1) % input->dma_buf_num;
		}
		left -= free;
		ddbwritel((input->cbuf << 11) | (input->coff >> 7),
			  DMA_BUFFER_ACK(input->nr));
	}
	return count;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

#if 0
static struct ddb_input *fe2input(struct ddb *dev, struct dvb_frontend *fe)
{
	int i;

	for (i = 0; i < dev->info->port_num * 2; i++) {
		if (dev->input[i].fe == fe)
			return &dev->input[i];
	}
	return NULL;
}
#endif

static int drxk_gate_ctrl(struct dvb_frontend *fe, int enable)
{
	struct ddb_input *input = fe->sec_priv;
	struct ddb_port *port = input->port;
	int status;

	if (enable) {
		mutex_lock(&port->i2c_gate_lock);
		status = input->gate_ctrl(fe, 1);
	} else {
		status = input->gate_ctrl(fe, 0);
		mutex_unlock(&port->i2c_gate_lock);
	}
	return status;
}

static int demod_attach_drxk(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct dvb_frontend *fe;
	struct drxk_config config;
	struct device *dev = &input->port->dev->pdev->dev;

	memset(&config, 0, sizeof(config));
	config.microcode_name = "drxk_a3.mc";
	config.qam_demod_parameter_count = 4;
	config.adr = 0x29 + (input->nr & 1);

	fe = input->fe = dvb_attach(drxk_attach, &config, i2c);
	if (!input->fe) {
		dev_err(dev, "No DRXK found!\n");
		return -ENODEV;
	}
	fe->sec_priv = input;
	input->gate_ctrl = fe->ops.i2c_gate_ctrl;
	fe->ops.i2c_gate_ctrl = drxk_gate_ctrl;
	return 0;
}

static int tuner_attach_tda18271(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct dvb_frontend *fe;
	struct device *dev = &input->port->dev->pdev->dev;

	if (input->fe->ops.i2c_gate_ctrl)
		input->fe->ops.i2c_gate_ctrl(input->fe, 1);
	fe = dvb_attach(tda18271c2dd_attach, input->fe, i2c, 0x60);
	if (!fe) {
		dev_err(dev, "No TDA18271 found!\n");
		return -ENODEV;
	}
	if (input->fe->ops.i2c_gate_ctrl)
		input->fe->ops.i2c_gate_ctrl(input->fe, 0);
	return 0;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static struct stv0367_config ddb_stv0367_config[] = {
	{
		.demod_address = 0x1f,
		.xtal = 27000000,
		.if_khz = 0,
		.if_iq_mode = FE_TER_NORMAL_IF_TUNER,
		.ts_mode = STV0367_SERIAL_PUNCT_CLOCK,
		.clk_pol = STV0367_CLOCKPOLARITY_DEFAULT,
	}, {
		.demod_address = 0x1e,
		.xtal = 27000000,
		.if_khz = 0,
		.if_iq_mode = FE_TER_NORMAL_IF_TUNER,
		.ts_mode = STV0367_SERIAL_PUNCT_CLOCK,
		.clk_pol = STV0367_CLOCKPOLARITY_DEFAULT,
	},
};

static int demod_attach_stv0367(struct ddb_input *input)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct device *dev = &input->port->dev->pdev->dev;

	/* attach frontend */
	input->fe = dvb_attach(stv0367ddb_attach,
		&ddb_stv0367_config[(input->nr & 1)], i2c);

	if (!input->fe) {
		dev_err(dev, "stv0367ddb_attach failed (not found?)\n");
		return -ENODEV;
	}

	input->fe->sec_priv = input;
	input->gate_ctrl = input->fe->ops.i2c_gate_ctrl;
	input->fe->ops.i2c_gate_ctrl = drxk_gate_ctrl;

	return 0;
}

static int tuner_tda18212_ping(struct ddb_input *input, unsigned short adr)
{
	struct i2c_adapter *adapter = &input->port->i2c->adap;
	struct device *dev = &input->port->dev->pdev->dev;

	u8 tda_id[2];
	u8 subaddr = 0x00;

	dev_dbg(dev, "stv0367-tda18212 tuner ping\n");
	if (input->fe->ops.i2c_gate_ctrl)
		input->fe->ops.i2c_gate_ctrl(input->fe, 1);

	if (i2c_read_regs(adapter, adr, subaddr, tda_id, sizeof(tda_id)) < 0)
		dev_dbg(dev, "tda18212 ping 1 fail\n");
	if (i2c_read_regs(adapter, adr, subaddr, tda_id, sizeof(tda_id)) < 0)
		dev_warn(dev, "tda18212 ping failed, expect problems\n");

	if (input->fe->ops.i2c_gate_ctrl)
		input->fe->ops.i2c_gate_ctrl(input->fe, 0);

	return 0;
}

static int demod_attach_cxd28xx(struct ddb_input *input, int par, int osc24)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct device *dev = &input->port->dev->pdev->dev;
	struct cxd2841er_config cfg;

	/* the cxd2841er driver expects 8bit/shifted I2C addresses */
	cfg.i2c_addr = ((input->nr & 1) ? 0x6d : 0x6c) << 1;

	cfg.xtal = osc24 ? SONY_XTAL_24000 : SONY_XTAL_20500;
	cfg.flags = CXD2841ER_AUTO_IFHZ | CXD2841ER_EARLY_TUNE |
		CXD2841ER_NO_WAIT_LOCK | CXD2841ER_NO_AGCNEG |
		CXD2841ER_TSBITS;

	if (!par)
		cfg.flags |= CXD2841ER_TS_SERIAL;

	/* attach frontend */
	input->fe = dvb_attach(cxd2841er_attach_t_c, &cfg, i2c);

	if (!input->fe) {
		dev_err(dev, "No Sony CXD28xx found!\n");
		return -ENODEV;
	}

	input->fe->sec_priv = input;
	input->gate_ctrl = input->fe->ops.i2c_gate_ctrl;
	input->fe->ops.i2c_gate_ctrl = drxk_gate_ctrl;

	return 0;
}

static int tuner_attach_tda18212(struct ddb_input *input, u32 porttype)
{
	struct i2c_adapter *adapter = &input->port->i2c->adap;
	struct device *dev = &input->port->dev->pdev->dev;
	struct i2c_client *client;
	struct tda18212_config config = {
		.fe = input->fe,
		.if_dvbt_6 = 3550,
		.if_dvbt_7 = 3700,
		.if_dvbt_8 = 4150,
		.if_dvbt2_6 = 3250,
		.if_dvbt2_7 = 4000,
		.if_dvbt2_8 = 4000,
		.if_dvbc = 5000,
	};
	struct i2c_board_info board_info = {
		.type = "tda18212",
		.platform_data = &config,
	};

	if (input->nr & 1)
		board_info.addr = 0x63;
	else
		board_info.addr = 0x60;

	/* due to a hardware quirk with the I2C gate on the stv0367+tda18212
	 * combo, the tda18212 must be probed by reading it's id _twice_ when
	 * cold started, or it very likely will fail.
	 */
	if (porttype == DDB_TUNER_DVBCT_ST)
		tuner_tda18212_ping(input, board_info.addr);

	request_module(board_info.type);

	/* perform tuner init/attach */
	client = i2c_new_device(adapter, &board_info);
	if (client == NULL || client->dev.driver == NULL)
		goto err;

	if (!try_module_get(client->dev.driver->owner)) {
		i2c_unregister_device(client);
		goto err;
	}

	input->i2c_client[0] = client;

	return 0;
err:
	dev_warn(dev, "TDA18212 tuner not found. Device is not fully operational.\n");
	return -ENODEV;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static struct stv090x_config stv0900 = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x69,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,
};

static struct stv090x_config stv0900_aa = {
	.device         = STV0900,
	.demod_mode     = STV090x_DUAL,
	.clk_mode       = STV090x_CLK_EXT,

	.xtal           = 27000000,
	.address        = 0x68,

	.ts1_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,
	.ts2_mode       = STV090x_TSMODE_SERIAL_PUNCTURED,

	.repeater_level = STV090x_RPTLEVEL_16,

	.adc1_range	= STV090x_ADC_1Vpp,
	.adc2_range	= STV090x_ADC_1Vpp,

	.diseqc_envelope_mode = true,
};

static struct stv6110x_config stv6110a = {
	.addr    = 0x60,
	.refclk	 = 27000000,
	.clk_div = 1,
};

static struct stv6110x_config stv6110b = {
	.addr    = 0x63,
	.refclk	 = 27000000,
	.clk_div = 1,
};

static int demod_attach_stv0900(struct ddb_input *input, int type)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct device *dev = &input->port->dev->pdev->dev;
	struct stv090x_config *feconf = type ? &stv0900_aa : &stv0900;

	input->fe = dvb_attach(stv090x_attach, feconf, i2c,
			       (input->nr & 1) ? STV090x_DEMODULATOR_1
			       : STV090x_DEMODULATOR_0);
	if (!input->fe) {
		dev_err(dev, "No STV0900 found!\n");
		return -ENODEV;
	}
	if (!dvb_attach(lnbh24_attach, input->fe, i2c, 0,
			0, (input->nr & 1) ?
			(0x09 - type) : (0x0b - type))) {
		dev_err(dev, "No LNBH24 found!\n");
		return -ENODEV;
	}
	return 0;
}

static int tuner_attach_stv6110(struct ddb_input *input, int type)
{
	struct i2c_adapter *i2c = &input->port->i2c->adap;
	struct device *dev = &input->port->dev->pdev->dev;
	struct stv090x_config *feconf = type ? &stv0900_aa : &stv0900;
	struct stv6110x_config *tunerconf = (input->nr & 1) ?
		&stv6110b : &stv6110a;
	const struct stv6110x_devctl *ctl;

	ctl = dvb_attach(stv6110x_attach, input->fe, tunerconf, i2c);
	if (!ctl) {
		dev_err(dev, "No STV6110X found!\n");
		return -ENODEV;
	}
	dev_info(dev, "attach tuner input %d adr %02x\n",
			 input->nr, tunerconf->addr);

	feconf->tuner_init          = ctl->tuner_init;
	feconf->tuner_sleep         = ctl->tuner_sleep;
	feconf->tuner_set_mode      = ctl->tuner_set_mode;
	feconf->tuner_set_frequency = ctl->tuner_set_frequency;
	feconf->tuner_get_frequency = ctl->tuner_get_frequency;
	feconf->tuner_set_bandwidth = ctl->tuner_set_bandwidth;
	feconf->tuner_get_bandwidth = ctl->tuner_get_bandwidth;
	feconf->tuner_set_bbgain    = ctl->tuner_set_bbgain;
	feconf->tuner_get_bbgain    = ctl->tuner_get_bbgain;
	feconf->tuner_set_refclk    = ctl->tuner_set_refclk;
	feconf->tuner_get_status    = ctl->tuner_get_status;

	return 0;
}

static int my_dvb_dmx_ts_card_init(struct dvb_demux *dvbdemux, char *id,
			    int (*start_feed)(struct dvb_demux_feed *),
			    int (*stop_feed)(struct dvb_demux_feed *),
			    void *priv)
{
	dvbdemux->priv = priv;

	dvbdemux->filternum = 256;
	dvbdemux->feednum = 256;
	dvbdemux->start_feed = start_feed;
	dvbdemux->stop_feed = stop_feed;
	dvbdemux->write_to_decoder = NULL;
	dvbdemux->dmx.capabilities = (DMX_TS_FILTERING |
				      DMX_SECTION_FILTERING |
				      DMX_MEMORY_BASED_FILTERING);
	return dvb_dmx_init(dvbdemux);
}

static int my_dvb_dmxdev_ts_card_init(struct dmxdev *dmxdev,
			       struct dvb_demux *dvbdemux,
			       struct dmx_frontend *hw_frontend,
			       struct dmx_frontend *mem_frontend,
			       struct dvb_adapter *dvb_adapter)
{
	int ret;

	dmxdev->filternum = 256;
	dmxdev->demux = &dvbdemux->dmx;
	dmxdev->capabilities = 0;
	ret = dvb_dmxdev_init(dmxdev, dvb_adapter);
	if (ret < 0)
		return ret;

	hw_frontend->source = DMX_FRONTEND_0;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, hw_frontend);
	mem_frontend->source = DMX_MEMORY_FE;
	dvbdemux->dmx.add_frontend(&dvbdemux->dmx, mem_frontend);
	return dvbdemux->dmx.connect_frontend(&dvbdemux->dmx, hw_frontend);
}

static int start_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ddb_input *input = dvbdmx->priv;

	if (!input->users)
		ddb_input_start(input);

	return ++input->users;
}

static int stop_feed(struct dvb_demux_feed *dvbdmxfeed)
{
	struct dvb_demux *dvbdmx = dvbdmxfeed->demux;
	struct ddb_input *input = dvbdmx->priv;

	if (--input->users)
		return input->users;

	ddb_input_stop(input);
	return 0;
}


static void dvb_input_detach(struct ddb_input *input)
{
	struct dvb_adapter *adap = &input->adap;
	struct dvb_demux *dvbdemux = &input->demux;
	struct i2c_client *client;

	switch (input->attached) {
	case 5:
		client = input->i2c_client[0];
		if (client) {
			module_put(client->dev.driver->owner);
			i2c_unregister_device(client);
		}
		if (input->fe2) {
			dvb_unregister_frontend(input->fe2);
			input->fe2 = NULL;
		}
		if (input->fe) {
			dvb_unregister_frontend(input->fe);
			dvb_frontend_detach(input->fe);
			input->fe = NULL;
		}
		/* fall-through */
	case 4:
		dvb_net_release(&input->dvbnet);
		/* fall-through */
	case 3:
		dvbdemux->dmx.close(&dvbdemux->dmx);
		dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
					      &input->hw_frontend);
		dvbdemux->dmx.remove_frontend(&dvbdemux->dmx,
					      &input->mem_frontend);
		dvb_dmxdev_release(&input->dmxdev);
		/* fall-through */
	case 2:
		dvb_dmx_release(&input->demux);
		/* fall-through */
	case 1:
		dvb_unregister_adapter(adap);
	}
	input->attached = 0;
}

static int dvb_input_attach(struct ddb_input *input)
{
	int ret;
	struct ddb_port *port = input->port;
	struct dvb_adapter *adap = &input->adap;
	struct dvb_demux *dvbdemux = &input->demux;
	struct device *dev = &input->port->dev->pdev->dev;
	int sony_osc24 = 0, sony_tspar = 0;

	ret = dvb_register_adapter(adap, "DDBridge", THIS_MODULE,
				   &input->port->dev->pdev->dev,
				   adapter_nr);
	if (ret < 0) {
		dev_err(dev, "Could not register adapter. Check if you enabled enough adapters in dvb-core!\n");
		return ret;
	}
	input->attached = 1;

	ret = my_dvb_dmx_ts_card_init(dvbdemux, "SW demux",
				      start_feed,
				      stop_feed, input);
	if (ret < 0)
		return ret;
	input->attached = 2;

	ret = my_dvb_dmxdev_ts_card_init(&input->dmxdev, &input->demux,
					 &input->hw_frontend,
					 &input->mem_frontend, adap);
	if (ret < 0)
		return ret;
	input->attached = 3;

	ret = dvb_net_init(adap, &input->dvbnet, input->dmxdev.demux);
	if (ret < 0)
		return ret;
	input->attached = 4;

	input->fe = NULL;
	switch (port->type) {
	case DDB_TUNER_DVBS_ST:
		if (demod_attach_stv0900(input, 0) < 0)
			return -ENODEV;
		if (tuner_attach_stv6110(input, 0) < 0)
			return -ENODEV;
		if (input->fe) {
			if (dvb_register_frontend(adap, input->fe) < 0)
				return -ENODEV;
		}
		break;
	case DDB_TUNER_DVBS_ST_AA:
		if (demod_attach_stv0900(input, 1) < 0)
			return -ENODEV;
		if (tuner_attach_stv6110(input, 1) < 0)
			return -ENODEV;
		if (input->fe) {
			if (dvb_register_frontend(adap, input->fe) < 0)
				return -ENODEV;
		}
		break;
	case DDB_TUNER_DVBCT_TR:
		if (demod_attach_drxk(input) < 0)
			return -ENODEV;
		if (tuner_attach_tda18271(input) < 0)
			return -ENODEV;
		if (dvb_register_frontend(adap, input->fe) < 0)
			return -ENODEV;
		if (input->fe2) {
			if (dvb_register_frontend(adap, input->fe2) < 0)
				return -ENODEV;
			input->fe2->tuner_priv = input->fe->tuner_priv;
			memcpy(&input->fe2->ops.tuner_ops,
			       &input->fe->ops.tuner_ops,
			       sizeof(struct dvb_tuner_ops));
		}
		break;
	case DDB_TUNER_DVBCT_ST:
		if (demod_attach_stv0367(input) < 0)
			return -ENODEV;
		if (tuner_attach_tda18212(input, port->type) < 0)
			return -ENODEV;
		if (input->fe) {
			if (dvb_register_frontend(adap, input->fe) < 0)
				return -ENODEV;
		}
		break;
	case DDB_TUNER_DVBC2T2I_SONY_P:
	case DDB_TUNER_DVBCT2_SONY_P:
	case DDB_TUNER_DVBC2T2_SONY_P:
	case DDB_TUNER_ISDBT_SONY_P:
		if (port->type == DDB_TUNER_DVBC2T2I_SONY_P)
			sony_osc24 = 1;
		if (input->port->dev->info->ts_quirks & TS_QUIRK_ALT_OSC)
			sony_osc24 = 0;
		if (input->port->dev->info->ts_quirks & TS_QUIRK_SERIAL)
			sony_tspar = 0;
		else
			sony_tspar = 1;

		if (demod_attach_cxd28xx(input, sony_tspar, sony_osc24) < 0)
			return -ENODEV;
		if (tuner_attach_tda18212(input, port->type) < 0)
			return -ENODEV;
		if (input->fe) {
			if (dvb_register_frontend(adap, input->fe) < 0)
				return -ENODEV;
		}
		break;
	case DDB_TUNER_XO2_DVBC2T2I_SONY:
	case DDB_TUNER_XO2_DVBCT2_SONY:
	case DDB_TUNER_XO2_DVBC2T2_SONY:
	case DDB_TUNER_XO2_ISDBT_SONY:
		if (port->type == DDB_TUNER_XO2_DVBC2T2I_SONY)
			sony_osc24 = 1;

		if (demod_attach_cxd28xx(input, 0, sony_osc24) < 0)
			return -ENODEV;
		if (tuner_attach_tda18212(input, port->type) < 0)
			return -ENODEV;
		if (input->fe) {
			if (dvb_register_frontend(adap, input->fe) < 0)
				return -ENODEV;
		}
		break;
	}

	input->attached = 5;
	return 0;
}

/****************************************************************************/
/****************************************************************************/

static ssize_t ts_write(struct file *file, const __user char *buf,
			size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	size_t left = count;
	int stat;

	while (left) {
		if (ddb_output_free(output) < 188) {
			if (file->f_flags & O_NONBLOCK)
				break;
			if (wait_event_interruptible(
				    output->wq, ddb_output_free(output) >= 188) < 0)
				break;
		}
		stat = ddb_output_write(output, buf, left);
		if (stat < 0)
			break;
		buf += stat;
		left -= stat;
	}
	return (left == count) ? -EAGAIN : (count - left);
}

static ssize_t ts_read(struct file *file, __user char *buf,
		       size_t count, loff_t *ppos)
{
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];
	int left, read;

	count -= count % 188;
	left = count;
	while (left) {
		if (ddb_input_avail(input) < 188) {
			if (file->f_flags & O_NONBLOCK)
				break;
			if (wait_event_interruptible(
				    input->wq, ddb_input_avail(input) >= 188) < 0)
				break;
		}
		read = ddb_input_read(input, buf, left);
		if (read < 0)
			return read;
		left -= read;
		buf += read;
	}
	return (left == count) ? -EAGAIN : (count - left);
}

static unsigned int ts_poll(struct file *file, poll_table *wait)
{
	/*
	struct dvb_device *dvbdev = file->private_data;
	struct ddb_output *output = dvbdev->priv;
	struct ddb_input *input = output->port->input[0];
	*/
	unsigned int mask = 0;

#if 0
	if (data_avail_to_read)
		mask |= POLLIN | POLLRDNORM;
	if (data_avail_to_write)
		mask |= POLLOUT | POLLWRNORM;

	poll_wait(file, &read_queue, wait);
	poll_wait(file, &write_queue, wait);
#endif
	return mask;
}

static const struct file_operations ci_fops = {
	.owner   = THIS_MODULE,
	.read    = ts_read,
	.write   = ts_write,
	.open    = dvb_generic_open,
	.release = dvb_generic_release,
	.poll    = ts_poll,
};

static struct dvb_device dvbdev_ci = {
	.readers = -1,
	.writers = -1,
	.users   = -1,
	.fops    = &ci_fops,
};

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void input_tasklet(unsigned long data)
{
	struct ddb_input *input = (struct ddb_input *) data;
	struct ddb *dev = input->port->dev;

	spin_lock(&input->lock);
	if (!input->running) {
		spin_unlock(&input->lock);
		return;
	}
	input->stat = ddbreadl(DMA_BUFFER_CURRENT(input->nr));

	if (input->port->class == DDB_PORT_TUNER) {
		if (4&ddbreadl(DMA_BUFFER_CONTROL(input->nr)))
			dev_err(&dev->pdev->dev, "Overflow input %d\n", input->nr);
		while (input->cbuf != ((input->stat >> 11) & 0x1f)
		       || (4 & safe_ddbreadl(dev, DMA_BUFFER_CONTROL(input->nr)))) {
			dvb_dmx_swfilter_packets(&input->demux,
						 input->vbuf[input->cbuf],
						 input->dma_buf_size / 188);

			input->cbuf = (input->cbuf + 1) % input->dma_buf_num;
			ddbwritel((input->cbuf << 11),
				  DMA_BUFFER_ACK(input->nr));
			input->stat = ddbreadl(DMA_BUFFER_CURRENT(input->nr));
		       }
	}
	if (input->port->class == DDB_PORT_CI)
		wake_up(&input->wq);
	spin_unlock(&input->lock);
}

static void output_tasklet(unsigned long data)
{
	struct ddb_output *output = (struct ddb_output *) data;
	struct ddb *dev = output->port->dev;

	spin_lock(&output->lock);
	if (!output->running) {
		spin_unlock(&output->lock);
		return;
	}
	output->stat = ddbreadl(DMA_BUFFER_CURRENT(output->nr + 8));
	wake_up(&output->wq);
	spin_unlock(&output->lock);
}


static struct cxd2099_cfg cxd_cfg = {
	.bitrate =  62000,
	.adr     =  0x40,
	.polarity = 1,
	.clock_mode = 1,
	.max_i2c = 512,
};

static int ddb_ci_attach(struct ddb_port *port)
{
	int ret;

	ret = dvb_register_adapter(&port->output->adap,
				   "DDBridge",
				   THIS_MODULE,
				   &port->dev->pdev->dev,
				   adapter_nr);
	if (ret < 0)
		return ret;
	port->en = cxd2099_attach(&cxd_cfg, port, &port->i2c->adap);
	if (!port->en) {
		dvb_unregister_adapter(&port->output->adap);
		return -ENODEV;
	}
	ddb_input_start(port->input[0]);
	ddb_output_start(port->output);
	dvb_ca_en50221_init(&port->output->adap,
			    port->en, 0, 1);
	ret = dvb_register_device(&port->output->adap, &port->output->dev,
				  &dvbdev_ci, (void *) port->output,
				  DVB_DEVICE_SEC, 0);
	return ret;
}

static int ddb_port_attach(struct ddb_port *port)
{
	struct device *dev = &port->dev->pdev->dev;
	int ret = 0;

	switch (port->class) {
	case DDB_PORT_TUNER:
		ret = dvb_input_attach(port->input[0]);
		if (ret < 0)
			break;
		ret = dvb_input_attach(port->input[1]);
		break;
	case DDB_PORT_CI:
		ret = ddb_ci_attach(port);
		break;
	default:
		break;
	}
	if (ret < 0)
		dev_err(dev, "port_attach on port %d failed\n", port->nr);
	return ret;
}

static int ddb_ports_attach(struct ddb *dev)
{
	int i, ret = 0;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		ret = ddb_port_attach(port);
		if (ret < 0)
			break;
	}
	return ret;
}

static void ddb_ports_detach(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		switch (port->class) {
		case DDB_PORT_TUNER:
			dvb_input_detach(port->input[0]);
			dvb_input_detach(port->input[1]);
			break;
		case DDB_PORT_CI:
			dvb_unregister_device(port->output->dev);
			if (port->en) {
				ddb_input_stop(port->input[0]);
				ddb_output_stop(port->output);
				dvb_ca_en50221_release(port->en);
				kfree(port->en);
				port->en = NULL;
				dvb_unregister_adapter(&port->output->adap);
			}
			break;
		}
	}
}

/****************************************************************************/
/****************************************************************************/

static int init_xo2(struct ddb_port *port)
{
	struct i2c_adapter *i2c = &port->i2c->adap;
	struct device *dev = &port->dev->pdev->dev;
	u8 val, data[2];
	int res;

	res = i2c_read_regs(i2c, 0x10, 0x04, data, 2);
	if (res < 0)
		return res;

	if (data[0] != 0x01)  {
		dev_info(dev, "Port %d: invalid XO2\n", port->nr);
		return -1;
	}

	i2c_read_reg(i2c, 0x10, 0x08, &val);
	if (val != 0) {
		i2c_write_reg(i2c, 0x10, 0x08, 0x00);
		msleep(100);
	}
	/* Enable tuner power, disable pll, reset demods */
	i2c_write_reg(i2c, 0x10, 0x08, 0x04);
	usleep_range(2000, 3000);
	/* Release demod resets */
	i2c_write_reg(i2c, 0x10, 0x08, 0x07);

	/* speed: 0=55,1=75,2=90,3=104 MBit/s */
	i2c_write_reg(i2c, 0x10, 0x09,
		((xo2_speed >= 0 && xo2_speed <= 3) ? xo2_speed : 2));

	i2c_write_reg(i2c, 0x10, 0x0a, 0x01);
	i2c_write_reg(i2c, 0x10, 0x0b, 0x01);

	usleep_range(2000, 3000);
	/* Start XO2 PLL */
	i2c_write_reg(i2c, 0x10, 0x08, 0x87);

	return 0;
}

static int port_has_xo2(struct ddb_port *port, u8 *type, u8 *id)
{
	u8 probe[1] = { 0x00 }, data[4];

	*type = DDB_XO2_TYPE_NONE;

	if (i2c_io(&port->i2c->adap, 0x10, probe, 1, data, 4))
		return 0;
	if (data[0] == 'D' && data[1] == 'F') {
		*id = data[2];
		*type = DDB_XO2_TYPE_DUOFLEX;
		return 1;
	}
	if (data[0] == 'C' && data[1] == 'I') {
		*id = data[2];
		*type = DDB_XO2_TYPE_CI;
		return 1;
	}
	return 0;
}

/****************************************************************************/
/****************************************************************************/

static int port_has_ci(struct ddb_port *port)
{
	u8 val;
	return i2c_read_reg(&port->i2c->adap, 0x40, 0, &val) ? 0 : 1;
}

static int port_has_stv0900(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x69, 0xf100, &val) < 0)
		return 0;
	return 1;
}

static int port_has_stv0900_aa(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x68, 0xf100, &val) < 0)
		return 0;
	return 1;
}

static int port_has_drxks(struct ddb_port *port)
{
	u8 val;
	if (i2c_read(&port->i2c->adap, 0x29, &val) < 0)
		return 0;
	if (i2c_read(&port->i2c->adap, 0x2a, &val) < 0)
		return 0;
	return 1;
}

static int port_has_stv0367(struct ddb_port *port)
{
	u8 val;
	if (i2c_read_reg16(&port->i2c->adap, 0x1e, 0xf000, &val) < 0)
		return 0;
	if (val != 0x60)
		return 0;
	if (i2c_read_reg16(&port->i2c->adap, 0x1f, 0xf000, &val) < 0)
		return 0;
	if (val != 0x60)
		return 0;
	return 1;
}

static int port_has_cxd28xx(struct ddb_port *port, u8 *id)
{
	struct i2c_adapter *i2c = &port->i2c->adap;
	int status;

	status = i2c_write_reg(&port->i2c->adap, 0x6e, 0, 0);
	if (status)
		return 0;
	status = i2c_read_reg(i2c, 0x6e, 0xfd, id);
	if (status)
		return 0;
	return 1;
}

static void ddb_port_probe(struct ddb_port *port)
{
	struct ddb *dev = port->dev;
	char *modname = "NO MODULE";
	u8 xo2_type, xo2_id, cxd_id;

	port->class = DDB_PORT_NONE;

	if (port_has_ci(port)) {
		modname = "CI";
		port->class = DDB_PORT_CI;
		ddbwritel(I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
	} else if (port_has_xo2(port, &xo2_type, &xo2_id)) {
		dev_dbg(&dev->pdev->dev, "Port %d (TAB %d): XO2 type: %d, id: %d\n",
			port->nr, port->nr+1, xo2_type, xo2_id);

		ddbwritel(I2C_SPEED_400, port->i2c->regs + I2C_TIMING);

		switch (xo2_type) {
		case DDB_XO2_TYPE_DUOFLEX:
			init_xo2(port);
			switch (xo2_id >> 2) {
			case 0:
				modname = "DUAL DVB-S2 (unsupported)";
				port->class = DDB_PORT_NONE;
				port->type = DDB_TUNER_XO2_DVBS_STV0910;
				break;
			case 1:
				modname = "DUAL DVB-C/T/T2";
				port->class = DDB_PORT_TUNER;
				port->type = DDB_TUNER_XO2_DVBCT2_SONY;
				break;
			case 2:
				modname = "DUAL DVB-ISDBT";
				port->class = DDB_PORT_TUNER;
				port->type = DDB_TUNER_XO2_ISDBT_SONY;
				break;
			case 3:
				modname = "DUAL DVB-C/C2/T/T2";
				port->class = DDB_PORT_TUNER;
				port->type = DDB_TUNER_XO2_DVBC2T2_SONY;
				break;
			case 4:
				modname = "DUAL ATSC (unsupported)";
				port->class = DDB_PORT_NONE;
				port->type = DDB_TUNER_XO2_ATSC_ST;
				break;
			case 5:
				modname = "DUAL DVB-C/C2/T/T2/ISDBT";
				port->class = DDB_PORT_TUNER;
				port->type = DDB_TUNER_XO2_DVBC2T2I_SONY;
				break;
			default:
				modname = "Unknown XO2 DuoFlex module\n";
				break;
			}
			break;
		case DDB_XO2_TYPE_CI:
			dev_info(&dev->pdev->dev, "DuoFlex CI modules not supported\n");
			break;
		default:
			dev_info(&dev->pdev->dev, "Unknown XO2 DuoFlex module\n");
			break;
		}
	} else if (port_has_cxd28xx(port, &cxd_id)) {
		switch (cxd_id) {
		case 0xa4:
			modname = "DUAL DVB-C2T2 CXD2843";
			port->class = DDB_PORT_TUNER;
			port->type = DDB_TUNER_DVBC2T2_SONY_P;
			break;
		case 0xb1:
			modname = "DUAL DVB-CT2 CXD2837";
			port->class = DDB_PORT_TUNER;
			port->type = DDB_TUNER_DVBCT2_SONY_P;
			break;
		case 0xb0:
			modname = "DUAL ISDB-T CXD2838";
			port->class = DDB_PORT_TUNER;
			port->type = DDB_TUNER_ISDBT_SONY_P;
			break;
		case 0xc1:
			modname = "DUAL DVB-C2T2 ISDB-T CXD2854";
			port->class = DDB_PORT_TUNER;
			port->type = DDB_TUNER_DVBC2T2I_SONY_P;
			break;
		default:
			modname = "Unknown CXD28xx tuner";
			break;
		}
		ddbwritel(I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0900(port)) {
		modname = "DUAL DVB-S2";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBS_ST;
		ddbwritel(I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0900_aa(port)) {
		modname = "DUAL DVB-S2";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBS_ST_AA;
		ddbwritel(I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	} else if (port_has_drxks(port)) {
		modname = "DUAL DVB-C/T";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBCT_TR;
		ddbwritel(I2C_SPEED_400, port->i2c->regs + I2C_TIMING);
	} else if (port_has_stv0367(port)) {
		modname = "DUAL DVB-C/T";
		port->class = DDB_PORT_TUNER;
		port->type = DDB_TUNER_DVBCT_ST;
		ddbwritel(I2C_SPEED_100, port->i2c->regs + I2C_TIMING);
	}

	dev_info(&dev->pdev->dev, "Port %d (TAB %d): %s\n",
			 port->nr, port->nr+1, modname);
}

static void ddb_input_init(struct ddb_port *port, int nr)
{
	struct ddb *dev = port->dev;
	struct ddb_input *input = &dev->input[nr];

	input->nr = nr;
	input->port = port;
	input->dma_buf_num = INPUT_DMA_BUFS;
	input->dma_buf_size = INPUT_DMA_SIZE;
	ddbwritel(0, TS_INPUT_CONTROL(nr));
	ddbwritel(2, TS_INPUT_CONTROL(nr));
	ddbwritel(0, TS_INPUT_CONTROL(nr));
	ddbwritel(0, DMA_BUFFER_ACK(nr));
	tasklet_init(&input->tasklet, input_tasklet, (unsigned long) input);
	spin_lock_init(&input->lock);
	init_waitqueue_head(&input->wq);
}

static void ddb_output_init(struct ddb_port *port, int nr)
{
	struct ddb *dev = port->dev;
	struct ddb_output *output = &dev->output[nr];
	output->nr = nr;
	output->port = port;
	output->dma_buf_num = OUTPUT_DMA_BUFS;
	output->dma_buf_size = OUTPUT_DMA_SIZE;

	ddbwritel(0, TS_OUTPUT_CONTROL(nr));
	ddbwritel(2, TS_OUTPUT_CONTROL(nr));
	ddbwritel(0, TS_OUTPUT_CONTROL(nr));
	tasklet_init(&output->tasklet, output_tasklet, (unsigned long) output);
	init_waitqueue_head(&output->wq);
}

static void ddb_ports_init(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		port->dev = dev;
		port->nr = i;
		port->i2c = &dev->i2c[i];
		port->input[0] = &dev->input[2 * i];
		port->input[1] = &dev->input[2 * i + 1];
		port->output = &dev->output[i];

		mutex_init(&port->i2c_gate_lock);
		ddb_port_probe(port);
		ddb_input_init(port, 2 * i);
		ddb_input_init(port, 2 * i + 1);
		ddb_output_init(port, i);
	}
}

static void ddb_ports_release(struct ddb *dev)
{
	int i;
	struct ddb_port *port;

	for (i = 0; i < dev->info->port_num; i++) {
		port = &dev->port[i];
		port->dev = dev;
		tasklet_kill(&port->input[0]->tasklet);
		tasklet_kill(&port->input[1]->tasklet);
		tasklet_kill(&port->output->tasklet);
	}
}

/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void irq_handle_i2c(struct ddb *dev, int n)
{
	struct ddb_i2c *i2c = &dev->i2c[n];

	i2c->done = 1;
	wake_up(&i2c->wq);
}

static irqreturn_t irq_handler(int irq, void *dev_id)
{
	struct ddb *dev = (struct ddb *) dev_id;
	u32 s = ddbreadl(INTERRUPT_STATUS);

	if (!s)
		return IRQ_NONE;

	do {
		ddbwritel(s, INTERRUPT_ACK);

		if (s & 0x00000001)
			irq_handle_i2c(dev, 0);
		if (s & 0x00000002)
			irq_handle_i2c(dev, 1);
		if (s & 0x00000004)
			irq_handle_i2c(dev, 2);
		if (s & 0x00000008)
			irq_handle_i2c(dev, 3);

		if (s & 0x00000100)
			tasklet_schedule(&dev->input[0].tasklet);
		if (s & 0x00000200)
			tasklet_schedule(&dev->input[1].tasklet);
		if (s & 0x00000400)
			tasklet_schedule(&dev->input[2].tasklet);
		if (s & 0x00000800)
			tasklet_schedule(&dev->input[3].tasklet);
		if (s & 0x00001000)
			tasklet_schedule(&dev->input[4].tasklet);
		if (s & 0x00002000)
			tasklet_schedule(&dev->input[5].tasklet);
		if (s & 0x00004000)
			tasklet_schedule(&dev->input[6].tasklet);
		if (s & 0x00008000)
			tasklet_schedule(&dev->input[7].tasklet);

		if (s & 0x00010000)
			tasklet_schedule(&dev->output[0].tasklet);
		if (s & 0x00020000)
			tasklet_schedule(&dev->output[1].tasklet);
		if (s & 0x00040000)
			tasklet_schedule(&dev->output[2].tasklet);
		if (s & 0x00080000)
			tasklet_schedule(&dev->output[3].tasklet);

		/* if (s & 0x000f0000)	printk(KERN_DEBUG "%08x\n", istat); */
	} while ((s = ddbreadl(INTERRUPT_STATUS)));

	return IRQ_HANDLED;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static int flashio(struct ddb *dev, u8 *wbuf, u32 wlen, u8 *rbuf, u32 rlen)
{
	u32 data, shift;

	if (wlen > 4)
		ddbwritel(1, SPI_CONTROL);
	while (wlen > 4) {
		/* FIXME: check for big-endian */
		data = swab32(*(u32 *)wbuf);
		wbuf += 4;
		wlen -= 4;
		ddbwritel(data, SPI_DATA);
		while (safe_ddbreadl(dev, SPI_CONTROL) & 0x0004)
			;
	}

	if (rlen)
		ddbwritel(0x0001 | ((wlen << (8 + 3)) & 0x1f00), SPI_CONTROL);
	else
		ddbwritel(0x0003 | ((wlen << (8 + 3)) & 0x1f00), SPI_CONTROL);

	data = 0;
	shift = ((4 - wlen) * 8);
	while (wlen) {
		data <<= 8;
		data |= *wbuf;
		wlen--;
		wbuf++;
	}
	if (shift)
		data <<= shift;
	ddbwritel(data, SPI_DATA);
	while (safe_ddbreadl(dev, SPI_CONTROL) & 0x0004)
		;

	if (!rlen) {
		ddbwritel(0, SPI_CONTROL);
		return 0;
	}
	if (rlen > 4)
		ddbwritel(1, SPI_CONTROL);

	while (rlen > 4) {
		ddbwritel(0xffffffff, SPI_DATA);
		while (safe_ddbreadl(dev, SPI_CONTROL) & 0x0004)
			;
		data = ddbreadl(SPI_DATA);
		*(u32 *) rbuf = swab32(data);
		rbuf += 4;
		rlen -= 4;
	}
	ddbwritel(0x0003 | ((rlen << (8 + 3)) & 0x1F00), SPI_CONTROL);
	ddbwritel(0xffffffff, SPI_DATA);
	while (safe_ddbreadl(dev, SPI_CONTROL) & 0x0004)
		;

	data = ddbreadl(SPI_DATA);
	ddbwritel(0, SPI_CONTROL);

	if (rlen < 4)
		data <<= ((4 - rlen) * 8);

	while (rlen > 0) {
		*rbuf = ((data >> 24) & 0xff);
		data <<= 8;
		rbuf++;
		rlen--;
	}
	return 0;
}

#define DDB_MAGIC 'd'

struct ddb_flashio {
	__user __u8 *write_buf;
	__u32 write_len;
	__user __u8 *read_buf;
	__u32 read_len;
};

#define IOCTL_DDB_FLASHIO  _IOWR(DDB_MAGIC, 0x00, struct ddb_flashio)

#define DDB_NAME "ddbridge"

static u32 ddb_num;
static struct ddb *ddbs[32];
static struct class *ddb_class;
static int ddb_major;

static int ddb_open(struct inode *inode, struct file *file)
{
	struct ddb *dev = ddbs[iminor(inode)];

	file->private_data = dev;
	return 0;
}

static long ddb_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct ddb *dev = file->private_data;
	__user void *parg = (__user void *)arg;
	int res;

	switch (cmd) {
	case IOCTL_DDB_FLASHIO:
	{
		struct ddb_flashio fio;
		u8 *rbuf, *wbuf;

		if (copy_from_user(&fio, parg, sizeof(fio)))
			return -EFAULT;

		if (fio.write_len > 1028 || fio.read_len > 1028)
			return -EINVAL;
		if (fio.write_len + fio.read_len > 1028)
			return -EINVAL;

		wbuf = &dev->iobuf[0];
		rbuf = wbuf + fio.write_len;

		if (copy_from_user(wbuf, fio.write_buf, fio.write_len))
			return -EFAULT;
		res = flashio(dev, wbuf, fio.write_len, rbuf, fio.read_len);
		if (res)
			return res;
		if (copy_to_user(fio.read_buf, rbuf, fio.read_len))
			return -EFAULT;
		break;
	}
	default:
		return -ENOTTY;
	}
	return 0;
}

static const struct file_operations ddb_fops = {
	.unlocked_ioctl = ddb_ioctl,
	.open           = ddb_open,
};

static char *ddb_devnode(struct device *device, umode_t *mode)
{
	struct ddb *dev = dev_get_drvdata(device);

	return kasprintf(GFP_KERNEL, "ddbridge/card%d", dev->nr);
}

static int ddb_class_create(void)
{
	ddb_major = register_chrdev(0, DDB_NAME, &ddb_fops);
	if (ddb_major < 0)
		return ddb_major;

	ddb_class = class_create(THIS_MODULE, DDB_NAME);
	if (IS_ERR(ddb_class)) {
		unregister_chrdev(ddb_major, DDB_NAME);
		return PTR_ERR(ddb_class);
	}
	ddb_class->devnode = ddb_devnode;
	return 0;
}

static void ddb_class_destroy(void)
{
	class_destroy(ddb_class);
	unregister_chrdev(ddb_major, DDB_NAME);
}

static int ddb_device_create(struct ddb *dev)
{
	dev->nr = ddb_num++;
	dev->ddb_dev = device_create(ddb_class, NULL,
				     MKDEV(ddb_major, dev->nr),
				     dev, "ddbridge%d", dev->nr);
	ddbs[dev->nr] = dev;
	if (IS_ERR(dev->ddb_dev))
		return -1;
	return 0;
}

static void ddb_device_destroy(struct ddb *dev)
{
	ddb_num--;
	if (IS_ERR(dev->ddb_dev))
		return;
	device_destroy(ddb_class, MKDEV(ddb_major, 0));
}


/****************************************************************************/
/****************************************************************************/
/****************************************************************************/

static void ddb_unmap(struct ddb *dev)
{
	if (dev->regs)
		iounmap(dev->regs);
	vfree(dev);
}


static void ddb_remove(struct pci_dev *pdev)
{
	struct ddb *dev = pci_get_drvdata(pdev);

	ddb_ports_detach(dev);
	ddb_i2c_release(dev);

	ddbwritel(0, INTERRUPT_ENABLE);
	free_irq(dev->pdev->irq, dev);
#ifdef CONFIG_PCI_MSI
	if (dev->msi)
		pci_disable_msi(dev->pdev);
#endif
	ddb_ports_release(dev);
	ddb_buffers_free(dev);
	ddb_device_destroy(dev);

	ddb_unmap(dev);
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
}


static int ddb_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct ddb *dev;
	int stat = 0;
	int irq_flag = IRQF_SHARED;

	if (pci_enable_device(pdev) < 0)
		return -ENODEV;

	dev = vzalloc(sizeof(struct ddb));
	if (dev == NULL)
		return -ENOMEM;

	dev->pdev = pdev;
	pci_set_drvdata(pdev, dev);
	dev->info = (struct ddb_info *) id->driver_data;
	dev_info(&pdev->dev, "Detected %s\n", dev->info->name);

	dev->regs = ioremap(pci_resource_start(dev->pdev, 0),
			    pci_resource_len(dev->pdev, 0));
	if (!dev->regs) {
		stat = -ENOMEM;
		goto fail;
	}
	dev_info(&pdev->dev, "HW %08x FW %08x\n", ddbreadl(0), ddbreadl(4));

#ifdef CONFIG_PCI_MSI
	if (pci_msi_enabled())
		stat = pci_enable_msi(dev->pdev);
	if (stat) {
		dev_info(&pdev->dev, "MSI not available.\n");
	} else {
		irq_flag = 0;
		dev->msi = 1;
	}
#endif
	stat = request_irq(dev->pdev->irq, irq_handler,
			   irq_flag, "DDBridge", (void *) dev);
	if (stat < 0)
		goto fail1;
	ddbwritel(0, DMA_BASE_WRITE);
	ddbwritel(0, DMA_BASE_READ);
	ddbwritel(0xffffffff, INTERRUPT_ACK);
	ddbwritel(0xfff0f, INTERRUPT_ENABLE);
	ddbwritel(0, MSI1_ENABLE);

	/* board control */
	if (dev->info->board_control) {
		ddbwritel(0, DDB_LINK_TAG(0) | BOARD_CONTROL);
		msleep(100);
		ddbwritel(dev->info->board_control_2,
			DDB_LINK_TAG(0) | BOARD_CONTROL);
		usleep_range(2000, 3000);
		ddbwritel(dev->info->board_control_2
			| dev->info->board_control,
			DDB_LINK_TAG(0) | BOARD_CONTROL);
		usleep_range(2000, 3000);
	}

	if (ddb_i2c_init(dev) < 0)
		goto fail1;
	ddb_ports_init(dev);
	if (ddb_buffers_alloc(dev) < 0) {
		dev_err(&pdev->dev, "Could not allocate buffer memory\n");
		goto fail2;
	}
	if (ddb_ports_attach(dev) < 0)
		goto fail3;
	ddb_device_create(dev);
	return 0;

fail3:
	ddb_ports_detach(dev);
	dev_err(&pdev->dev, "fail3\n");
	ddb_ports_release(dev);
fail2:
	dev_err(&pdev->dev, "fail2\n");
	ddb_buffers_free(dev);
fail1:
	dev_err(&pdev->dev, "fail1\n");
	if (dev->msi)
		pci_disable_msi(dev->pdev);
	if (stat == 0)
		free_irq(dev->pdev->irq, dev);
fail:
	dev_err(&pdev->dev, "fail\n");
	ddb_unmap(dev);
	pci_set_drvdata(pdev, NULL);
	pci_disable_device(pdev);
	return -1;
}

/******************************************************************************/
/******************************************************************************/
/******************************************************************************/

static const struct ddb_info ddb_none = {
	.type     = DDB_NONE,
	.name     = "Digital Devices PCIe bridge",
};

static const struct ddb_info ddb_octopus = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus DVB adapter",
	.port_num = 4,
};

static const struct ddb_info ddb_octopus_le = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus LE DVB adapter",
	.port_num = 2,
};

static const struct ddb_info ddb_octopus_oem = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus OEM",
	.port_num = 4,
};

static const struct ddb_info ddb_octopus_mini = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus Mini",
	.port_num = 4,
};

static const struct ddb_info ddb_v6 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6 DVB adapter",
	.port_num = 3,
};
static const struct ddb_info ddb_v6_5 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine S2 V6.5 DVB adapter",
	.port_num = 4,
};

static const struct ddb_info ddb_dvbct = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices DVBCT V6.1 DVB adapter",
	.port_num = 3,
};

static const struct ddb_info ddb_ctv7 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Cine CT V7 DVB adapter",
	.port_num = 4,
	.board_control   = 3,
	.board_control_2 = 4,
};

static const struct ddb_info ddb_satixS2v3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Mystique SaTiX-S2 V3 DVB adapter",
	.port_num = 3,
};

static const struct ddb_info ddb_octopusv3 = {
	.type     = DDB_OCTOPUS,
	.name     = "Digital Devices Octopus V3 DVB adapter",
	.port_num = 4,
};

/*** MaxA8 adapters ***********************************************************/

static struct ddb_info ddb_ct2_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 CT2",
	.port_num = 4,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
};

static struct ddb_info ddb_c2t2_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2",
	.port_num = 4,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
};

static struct ddb_info ddb_isdbt_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 ISDBT",
	.port_num = 4,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
};

static struct ddb_info ddb_c2t2i_v0_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2I V0",
	.port_num = 4,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL | TS_QUIRK_ALT_OSC,
};

static struct ddb_info ddb_c2t2i_8 = {
	.type     = DDB_OCTOPUS_MAX_CT,
	.name     = "Digital Devices MAX A8 C2T2I",
	.port_num = 4,
	.board_control   = 0x0ff,
	.board_control_2 = 0xf00,
	.ts_quirks = TS_QUIRK_SERIAL,
};

/******************************************************************************/

#define DDVID 0xdd01 /* Digital Devices Vendor ID */

#define DDB_ID(_vend, _dev, _subvend, _subdev, _driverdata) {	\
	.vendor      = _vend,    .device    = _dev, \
	.subvendor   = _subvend, .subdevice = _subdev, \
	.driver_data = (unsigned long)&_driverdata }

static const struct pci_device_id ddb_id_tbl[] = {
	DDB_ID(DDVID, 0x0002, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0001, ddb_octopus),
	DDB_ID(DDVID, 0x0005, DDVID, 0x0004, ddb_octopusv3),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0002, ddb_octopus_le),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0003, ddb_octopus_oem),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0010, ddb_octopus_mini),
	DDB_ID(DDVID, 0x0005, DDVID, 0x0011, ddb_octopus_mini),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0020, ddb_v6),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0021, ddb_v6_5),
	DDB_ID(DDVID, 0x0003, DDVID, 0x0030, ddb_dvbct),
	DDB_ID(DDVID, 0x0003, DDVID, 0xdb03, ddb_satixS2v3),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0031, ddb_ctv7),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0032, ddb_ctv7),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0033, ddb_ctv7),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0034, ddb_ct2_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0035, ddb_c2t2_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0036, ddb_isdbt_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0037, ddb_c2t2i_v0_8),
	DDB_ID(DDVID, 0x0008, DDVID, 0x0038, ddb_c2t2i_8),
	DDB_ID(DDVID, 0x0006, DDVID, 0x0039, ddb_ctv7),
	/* in case sub-ids got deleted in flash */
	DDB_ID(DDVID, 0x0003, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0005, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0006, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0007, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0008, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0011, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0013, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0201, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	DDB_ID(DDVID, 0x0320, PCI_ANY_ID, PCI_ANY_ID, ddb_none),
	{0}
};
MODULE_DEVICE_TABLE(pci, ddb_id_tbl);


static struct pci_driver ddb_pci_driver = {
	.name        = "DDBridge",
	.id_table    = ddb_id_tbl,
	.probe       = ddb_probe,
	.remove      = ddb_remove,
};

static __init int module_init_ddbridge(void)
{
	int ret;

	pr_info("Digital Devices PCIE bridge driver, Copyright (C) 2010-11 Digital Devices GmbH\n");

	ret = ddb_class_create();
	if (ret < 0)
		return ret;
	ret = pci_register_driver(&ddb_pci_driver);
	if (ret < 0)
		ddb_class_destroy();
	return ret;
}

static __exit void module_exit_ddbridge(void)
{
	pci_unregister_driver(&ddb_pci_driver);
	ddb_class_destroy();
}

module_init(module_init_ddbridge);
module_exit(module_exit_ddbridge);

MODULE_DESCRIPTION("Digital Devices PCIe Bridge");
MODULE_AUTHOR("Ralph Metzler");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.5");

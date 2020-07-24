/*
 * Copyright (c) 2018 IMD Technologies.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <device.h>
#include <drivers/i2c.h>
#include <sys/byteorder.h>
#include <sys/util.h>
#include <kernel.h>
#include <drivers/sensor.h>
#include <sys/__assert.h>
#include <sys/printk.h>
#include <math.h>

#include "shtc3.h"
#include "__temphum9_driver.h"

#define DT_DRV_COMPAT sensirion_shtc3

#include <logging/log.h>
LOG_MODULE_REGISTER(SHTC3, CONFIG_SENSOR_LOG_LEVEL);

static int shtc3_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	struct shtc3_data *drv_data = dev->driver_data;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		drv_data->sample[SHTC3_SAMPLE_TEMPERATURE_INDEX] = temphum9_getTemperature(dev, _TEMPHUM9_NORMAL_MODE);
	}
	else if (chan == SENSOR_CHAN_HUMIDITY) {
		drv_data->sample[SHTC3_SAMPLE_HUMIDITY_INDEX] = temphum9_getRelativeHumidity(dev, _TEMPHUM9_NORMAL_MODE);
	}
	else if (chan == SENSOR_CHAN_ALL) {
		temhum9_getTemperatureAndHumidity(dev, _TEMPHUM9_NORMAL_MODE, &drv_data->sample[0]);	
	}	
	else {
		LOG_ERR("error: unsupported channel\n");
		return -ENOTSUP;
	}

	LOG_DBG("read: chan %d T:%f C    H:%f\n", chan,
		drv_data->sample[SHTC3_SAMPLE_TEMPERATURE_INDEX], 
		drv_data->sample[SHTC3_SAMPLE_HUMIDITY_INDEX]);

	return 0;
}

static int shtc3_channel_get(struct device *dev,
		enum sensor_channel chan,
		struct sensor_value *val)
{
	struct shtc3_data *drv_data = dev->driver_data;
	int i;

	if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		i = SHTC3_SAMPLE_TEMPERATURE_INDEX;
	}
	else if (chan == SENSOR_CHAN_HUMIDITY) {
		i = SHTC3_SAMPLE_HUMIDITY_INDEX;
	}
	else {
		return -ENOTSUP;
	}

	val->val1 = (s32_t)(drv_data->sample[i]);
	if(drv_data->sample[i] > 0) {
		val->val2 = (s32_t)((drv_data->sample[i] - (float)(val->val1)) * 1000000);
	}
	else {
		val->val2 = (s32_t)((-drv_data->sample[i] + (float)(val->val1)) * 1000000);
	}
	return 0;
}

static int shtc3_id_read(struct device *dev)
{
	struct shtc3_data *drv_data = dev->driver_data;
	const struct shtc3_dev_config *cfg = dev->config_info;
	int ret = 0; 
	struct i2c_msg msgs[1];
	u8_t buf[] = {0xEF, 0xC8};
	
	LOG_DBG("i2c address 0x%x\n", cfg->i2c_addr);
	/* Send the address to read from */
	msgs[0].buf = buf;
	msgs[0].len = 2;
	msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
	if (i2c_transfer(drv_data->i2c, &msgs[0], 1, cfg->i2c_addr) == 0) {
		LOG_DBG("success ID command\n");
	} else {
		LOG_ERR("failure ID command\n");
		ret = -1;
	}

	if(ret == 0)
	{
		msgs[0].buf = buf;
		msgs[0].len = 2;
		msgs[0].flags = I2C_MSG_READ | I2C_MSG_STOP;
		if (i2c_transfer(drv_data->i2c, &msgs[0], 1, cfg->i2c_addr) == 0) {
			LOG_DBG("success ID read: 0x%x 0x%x\n",buf[0],buf[1]);
		} else {
			LOG_ERR("failure ID read\n");
			ret = -1;
		}
	}

	return ret;	
}

static const struct sensor_driver_api shtc3_driver_api = {
	.sample_fetch = shtc3_sample_fetch,
	.channel_get = shtc3_channel_get,
};

static int shtc3_probe(struct device *dev)
{
	int ret;

	ret = shtc3_id_read(dev);
	if (ret) {
		return ret;
	}

	return 0;
}

static int shtc3_init(struct device *dev)
{
	struct shtc3_data *drv_data = dev->driver_data;
	const struct shtc3_dev_config *cfg = dev->config_info;

	LOG_DBG("shtc3 i2c address 0x%x\n", cfg->i2c_addr);

	drv_data->i2c = device_get_binding(cfg->i2c_port);
	if (drv_data->i2c == NULL) {
		printk("Failed to get pointer to %s device!\n",
			    cfg->i2c_port);
		return -EINVAL;
	}

	return shtc3_probe(dev);
}

static struct shtc3_data shtc3_driver;

static const struct shtc3_dev_config shtc3_config = {
	.i2c_port = DT_INST_BUS_LABEL(0),
	.i2c_addr = DT_INST_REG_ADDR(0),
};

DEVICE_AND_API_INIT(shtc3, DT_INST_LABEL(0), shtc3_init, &shtc3_driver,
		    &shtc3_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &shtc3_driver_api);

/*
 * Copyright (c) 2020 IMD Technologies
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_SHTC3_SHTC3_H_
#define ZEPHYR_DRIVERS_SENSOR_SHTC3_SHTC3_H_

#include <zephyr/types.h>
#include <device.h>

#define SHTC3_SAMPLE_TEMPERATURE_INDEX (0)
#define SHTC3_SAMPLE_HUMIDITY_INDEX    (1)

struct shtc3_data {
	struct device *i2c;
	float sample[2];
};

struct shtc3_dev_config {
	const char *i2c_port;
	u16_t i2c_addr;
};

#endif /* ZEPHYR_DRIVERS_SENSOR_SHTC3_SHTC3_H_ */

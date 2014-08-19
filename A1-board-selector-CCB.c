/*
 * board selector support for TCA9535 used in Bitmine's CoinCraft Blade
 *
 * Copyright 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */


#include <sys/ioctl.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

#include <stdint.h>
#include <stdbool.h>
#include <fcntl.h>

#include "miner.h"

#include "A1-board-selector.h"
#include "i2c-context.h"

static struct board_selector ccb_selector;

static struct i2c_ctx *U1_tca9535;
static uint8_t chain_mask = 0xff;
static uint8_t active_chain = 255;
static pthread_mutex_t lock;

static void ccb_unlock(void)
{
	mutex_unlock(&lock);
}

static void ccb_exit(void)
{
	if (U1_tca9535 != NULL)
		U1_tca9535->exit(U1_tca9535);
}

extern struct board_selector *ccb_board_selector_init(void)
{
	mutex_init(&lock);
	U1_tca9535 = i2c_slave_open(I2C_BUS, 0x27);
	if (U1_tca9535 == NULL)
		return NULL;
	bool retval =	U1_tca9535->write(U1_tca9535, 0x07, 0x00) &&
			U1_tca9535->write(U1_tca9535, 0x03, 0xff) &&
			U1_tca9535->write(U1_tca9535, 0x06, 0x00) &&
			U1_tca9535->write(U1_tca9535, 0x02, 0x00);
	if (retval)
		return &ccb_selector;
	ccb_exit();
	return NULL;
}

static bool ccb_select(uint8_t chain)
{
	if (chain >= CCB_MAX_CHAINS)
		return false;

	mutex_lock(&lock);
	if (active_chain == chain)
		return true;

	active_chain = chain;
	chain_mask = 1 << active_chain;
	return U1_tca9535->write(U1_tca9535, 0x03, ~chain_mask);
}

static bool __ccb_board_selector_reset(uint8_t mask)
{
	if (!U1_tca9535->write(U1_tca9535, 0x02, mask))
		return false;
	cgsleep_ms(RESET_LOW_TIME_MS);
	if (!U1_tca9535->write(U1_tca9535, 0x02, 0x00))
		return false;
	cgsleep_ms(RESET_HI_TIME_MS);
	return true;
}
// we assume we are already holding the mutex
static bool ccb_reset(void)
{
	return __ccb_board_selector_reset(chain_mask);
}

static bool ccb_reset_all(void)
{
	mutex_lock(&lock);
	bool retval = __ccb_board_selector_reset(0xff);
	mutex_unlock(&lock);
	return retval;
}

static uint8_t ccb_get_temp(uint8_t sensor_id)
{
	if (sensor_id != 0)
		return 0;

	static uint8_t temp_slaves[8] = {
		0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f
	};
	struct i2c_ctx *t = i2c_slave_open(I2C_BUS, temp_slaves[active_chain]);
	if (t == NULL)
		return 0;

	uint8_t retval = 0;
	if (!t->read(t, 0, &retval))
		retval = 0;
	t->exit(t);
	return retval;
}

static struct board_selector ccb_selector = {
	.select = ccb_select,
	.release = ccb_unlock,
	.exit = ccb_exit,
	.reset = ccb_reset,
	.reset_all = ccb_reset_all,
	.get_temp = ccb_get_temp,
};



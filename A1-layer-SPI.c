/*
 * SPI protocol for Coincraft A1
 *
 * Copyright 2013, 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 3 of the License, or (at your option)
 * any later version.  See COPYING for more details.
 */

#include <stdlib.h>
#include <assert.h>
#include <fcntl.h>
#include <limits.h>
#include <unistd.h>
#include <stdbool.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"


#include "A1-common.h"

void flush_spi(struct A1_chain *a1)
{
	memset(a1->spi_tx, 0, 64);
	spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, 64);
}


/********** upper layer SPI functions */
static uint8_t *exec_cmd(struct A1_chain *a1,
			  uint8_t cmd, uint8_t chip_id,
			  uint8_t *data, uint8_t len,
			  uint8_t resp_len)
{
	int tx_len = 4 + len;
	memset(a1->spi_tx, 0, tx_len);
	a1->spi_tx[0] = cmd;
	a1->spi_tx[1] = chip_id;

	if (data != NULL)
		memcpy(a1->spi_tx + 2, data, len);

	assert(spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len));
	hexdump("send: TX", a1->spi_tx, tx_len);
	hexdump("send: RX", a1->spi_rx, tx_len);

	int poll_len = resp_len;
	if (chip_id == 0) {
		if (a1->num_chips == 0) {
			applog(LOG_INFO, "%d: unknown chips in chain, "
			       "assuming 8", a1->chain_id);
			poll_len += 32;
		}
		poll_len += 4 * a1->num_chips;
	}
	else {
		poll_len += 4 * chip_id - 2;
	}

	assert(spi_transfer(a1->spi_ctx, NULL, a1->spi_rx + tx_len, poll_len));
	hexdump("poll: RX", a1->spi_rx + tx_len, poll_len);
	int ack_len = tx_len + resp_len;
	int ack_pos = tx_len + poll_len - ack_len;
	hexdump("poll: ACK", a1->spi_rx + ack_pos, ack_len - 2);

	return (a1->spi_rx + ack_pos);
}


/********** A1 SPI commands */
uint8_t *cmd_BIST_START_BCAST(struct A1_chain *a1)
{
	uint8_t *ret = exec_cmd(a1, A1_BIST_START, 0x00, NULL, 2, 0);
	if (ret == NULL || ret[0] != A1_BIST_START) {
		applog(LOG_ERR, "%d: cmd_BIST_START_BCAST failed", a1->chain_id);
		return NULL;
	}
	return ret;
}

uint8_t *cmd_BIST_FIX_BCAST(struct A1_chain *a1)
{
	uint8_t *ret = exec_cmd(a1, A1_BIST_FIX, 0x00, NULL, 0, 0);
	if (ret == NULL || ret[0] != A1_BIST_FIX) {
		applog(LOG_ERR, "%d: cmd_BIST_FIX_BCAST failed", a1->chain_id);
		return NULL;
	}
	return ret;
}

uint8_t *cmd_RESET_BCAST(struct A1_chain *a1, uint8_t strategy)
{
	static uint8_t s[2];
	s[0] = strategy;
	s[1] = strategy;
	uint8_t *ret = exec_cmd(a1, A1_RESET, 0x00, s, 2, 0);
	if (ret == NULL || (ret[0] != A1_RESET && a1->num_chips != 0)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", a1->chain_id);
		return NULL;
	}
	return ret;
}

uint8_t *cmd_RESET(struct A1_chain *a1, int cid, uint8_t strategy)
{
	static uint8_t s[2];
	s[0] = strategy;
	s[1] = strategy;
	uint8_t *ret = exec_cmd(a1, A1_RESET, cid, s, 2, 0);
	if (ret == NULL || (ret[0] != A1_RESET && a1->num_chips != 0)) {
		applog(LOG_ERR, "%d: cmd_RESET_BCAST failed", a1->chain_id);
		return NULL;
	}
	return ret;
}

uint8_t *cmd_READ_RESULT_BCAST(struct A1_chain *a1)
{
	int tx_len = 8;
	memset(a1->spi_tx, 0, tx_len);
	a1->spi_tx[0] = A1_READ_RESULT;

	assert(spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len));
	hexdump("send: TX", a1->spi_tx, tx_len);
	hexdump("send: RX", a1->spi_rx, tx_len);

	int poll_len = tx_len + 4 * a1->num_chips;
	assert(spi_transfer(a1->spi_ctx, NULL, a1->spi_rx + tx_len, poll_len));
	hexdump("poll: RX", a1->spi_rx + tx_len, poll_len);

	uint8_t *scan = a1->spi_rx;
	int i;
	for (i = 0; i < poll_len; i += 2) {
		if ((scan[i] & 0x0f) == A1_READ_RESULT)
			return scan + i;
	}
	applog(LOG_ERR, "%d: cmd_READ_RESULT_BCAST failed", a1->chain_id);
	return NULL;
}

uint8_t *cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg)
{
	uint8_t *ret = exec_cmd(a1, A1_WRITE_REG, chip, reg, 6, 0);
	if (ret == NULL || ret[0] != A1_WRITE_REG) {
		applog(LOG_ERR, "%d: cmd_WRITE_REG failed", a1->chain_id);
		return NULL;
	}
	return ret;
}

uint8_t *cmd_READ_REG(struct A1_chain *a1, uint8_t chip)
{
	uint8_t *ret = exec_cmd(a1, A1_READ_REG, chip, NULL, 0, 6);
	if (ret == NULL || ret[0] != A1_READ_REG_RESP || ret[1] != chip) {
		applog(LOG_ERR, "%d: cmd_READ_REG chip %d failed",
		       a1->chain_id, chip);
		return NULL;
	}
	memcpy(a1->spi_rx, ret, 8);
	return ret;
}

uint8_t *cmd_WRITE_JOB(struct A1_chain *a1, uint8_t chip_id, uint8_t *job)
{
	/* ensure we push the SPI command to the last chip in chain */
	int tx_len = WRITE_JOB_LENGTH + 2;
	memcpy(a1->spi_tx, job, WRITE_JOB_LENGTH);
	memset(a1->spi_tx + WRITE_JOB_LENGTH, 0, tx_len - WRITE_JOB_LENGTH);

	assert(spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len));
	hexdump("send: TX", a1->spi_tx, tx_len);
	hexdump("send: RX", a1->spi_rx, tx_len);

	int poll_len = 4 * chip_id - 2;

	assert(spi_transfer(a1->spi_ctx, NULL, a1->spi_rx + tx_len, poll_len));
	hexdump("poll: RX", a1->spi_rx + tx_len, poll_len);

	int ack_len = tx_len;
	int ack_pos = tx_len + poll_len - ack_len;
	hexdump("poll: ACK", a1->spi_rx + ack_pos, tx_len);

	uint8_t *ret = a1->spi_rx + ack_pos;
	if (ret[0] != a1->spi_tx[0] || ret[1] != a1->spi_tx[1]){
		applog(LOG_ERR, "%d: cmd_WRITE_JOB failed: "
			"0x%02x%02x/0x%02x%02x", a1->chain_id,
			ret[0], ret[1], a1->spi_tx[0], a1->spi_tx[1]);
		return NULL;
	}
	return ret;
}

///////////////////////////////////////////////////////////////////////////////

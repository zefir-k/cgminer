/*
 * Common generic functions used in A1 driver
 *
 * Copyright 2014 Zefir Kurtisi <zefir.kurtisi@gmail.com>
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
#include <float.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "A1-private.h"

int get_current_ms(void)
{
	cgtimer_t ct;
	cgtimer_time(&ct);
	return cgtimer_to_ms(&ct);
}


/********** temporary helper for hexdumping SPI traffic */
static void applog_hexdump(char *prefix, uint8_t *buff, int len, int level)
{
	static char line[256];
	char *pos = line;
	int i;
	if (len < 1)
		return;

	pos += sprintf(pos, "%s: %d bytes:", prefix, len);
	for (i = 0; i < len; i++) {
		if (i > 0 && (i % 32) == 0) {
			applog(LOG_DEBUG, "%s", line);
			pos = line;
			pos += sprintf(pos, "\t");
		}
		pos += sprintf(pos, "%.2X ", buff[i]);
	}
	applog(level, "%s", line);
}

void hexdump(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_DEBUG);
}

void hexdump_error(char *prefix, uint8_t *buff, int len)
{
	applog_hexdump(prefix, buff, len, LOG_ERR);
}

const char *time_string(void)
{
	static char datetime[64];
	struct timeval tv = {0, 0};
	struct tm *tm;

	cgtime(&tv);

	const time_t tmp_time = tv.tv_sec;
	tm = localtime(&tmp_time);

	snprintf(datetime, sizeof(datetime), " [%d-%02d-%02d %02d:%02d:%02d] ",
		tm->tm_year + 1900,
		tm->tm_mon + 1,
		tm->tm_mday,
		tm->tm_hour,
		tm->tm_min,
		tm->tm_sec);

	return datetime;
}

/********** job creation and result evaluation */
uint32_t get_target(double diff)
{
	static double prev_diff;
	static uint32_t n_bits;

	/* don't re-calculate target on unchanged diff */
	if (diff == prev_diff)
		return n_bits;
	prev_diff = diff;
	int shift = 29;
	double f = (double) 0x0000ffff / diff;
	while (f < (double) 0x00008000) {
		shift--;
		f *= 256.0;
	}
	while (f >= (double) 0x00800000) {
		shift++;
		f /= 256.0;
	}
	n_bits = (int) f + (shift << 24);
	return n_bits;
}

/*****************************************************************************/
/********** work queue */
bool A1_wq_enqueue(struct work_queue *wq, struct work *work)
{
	if (work == NULL)
		return false;
	struct work_ent *we = malloc(sizeof(*we));
	assert(we != NULL);

	we->work = work;
	INIT_LIST_HEAD(&we->head);
	list_add_tail(&we->head, &wq->head);
	wq->num_elems++;
	return true;
}

struct work *A1_wq_dequeue(struct work_queue *wq)
{
	if (wq == NULL)
		return NULL;
	if (wq->num_elems == 0)
		return NULL;
	struct work_ent *we;
	we = list_entry(wq->head.next, struct work_ent, head);
	struct work *work = we->work;

	list_del(&we->head);
	free(we);
	wq->num_elems--;
	return work;
}

/********** config options */
/* override values with --bitmine-a1-options ref:sys:spi: - use 0 for default */
static struct A1_config_options *parsed_config_options;

char *opt_bitmine_a1_options = NULL;
struct A1_extra_options A1_extra_options;

static void A1_parse_option_array(char *opt, int *c, char *info, bool is_hex)
{
	int m[MAX_BOARDS];

	if (opt[0] == 0)
		return;
	const char *format = is_hex ?
		"%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x-%x" :
		"%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d-%d";
	memset(m, 0, sizeof(m));
	applog(LOG_DEBUG, "%s: %s", info, opt);
	int n = sscanf(opt, format,
			m + 0, m + 1, m + 2, m + 3,
			m + 4, m + 5, m + 6, m + 7,
			m + 8, m + 9, m + 10, m + 11,
			m + 12, m + 13, m + 14, m + 15);
	if (n > 0) {
		int i;
		int last = m[n - 1];
		for (i = 0; i < MAX_BOARDS; i++)
			c[i] = (i < n) ? m[i] : last;

		char prefix[80];
		sprintf(prefix, "%s: %d entries scanned: %s", info, n, format);
		applog(LOG_WARNING, prefix,
			c[0], c[1], c[2], c[3], c[4], c[5], c[6], c[7],
			c[8], c[9], c[10], c[11], c[12], c[13], c[14], c[15]);
	}
}

static void A1_print_config_options(struct A1_config_options *c)
{
	/* config checking */
	if (A1_config_options.sys_clk_khz < 100000)
		quit(1, "system clock must be above 100MHz");

	if (A1_config_options.override_diff == -1)
		bitmineA1_drv.max_diff = DBL_MAX;
	else if (A1_config_options.override_diff > 1)
		bitmineA1_drv.max_diff = A1_config_options.override_diff;

	/* print the config */
	applog(LOG_WARNING, "A1 config:");
	applog(LOG_WARNING, "  ref=%d, sys=%d, spi=%d",
		c->ref_clk_khz, c->sys_clk_khz, c->spi_clk_khz);
	applog(LOG_WARNING, "  override_diff=%d, chain_mask=0x%x",
		c->override_diff, c->chain_mask);
	applog(LOG_WARNING, "  enable_auto_tune=%d, lower_ratio_pm=%d, "
		"upper_ratio_pm=%d, lower_clk_khz=%d, upper_clk_khz=%d",
		c->enable_auto_tune, c->lower_ratio_pm, c->upper_ratio_pm,
		c->lower_clk_khz, c->upper_clk_khz);
	applog(LOG_WARNING, "  stats_fname=%s, config_fname=%s",
		c->stats_fname ? c->stats_fname : "",
		c->config_fname ? c->config_fname : "");

}

void A1_parse_options(void)
{
	/* parse bimine-a1-options */
	if (opt_bitmine_a1_options == NULL) {
		A1_print_config_options(&A1_config_options);
		return;
	}
	if (parsed_config_options != NULL)
		return;

	int ref_clk = 0;
	int sys_clk = 0;
	int spi_clk = 0;
	int override_chip_num = 0;
	int wiper = 0;
	int board_mask = 0;
	int override_diff = 0;


	static char clk_tmp[128];
	static char wiper_tmp[128];
	static char cmask_tmp[128];
	static char sclk_tmp[128];

	sscanf(opt_bitmine_a1_options, "%d:%d:%d:%d:%d:%d:%x %s %s %s %s",
	       &ref_clk, &sys_clk, &spi_clk, &override_chip_num,
	       &wiper, &override_diff, &board_mask,
	       clk_tmp, wiper_tmp, cmask_tmp, sclk_tmp);
	if (ref_clk != 0)
		A1_config_options.ref_clk_khz = ref_clk;
	if (sys_clk != 0)
		A1_config_options.sys_clk_khz = sys_clk;
	if (spi_clk != 0)
		A1_config_options.spi_clk_khz = spi_clk;
	if (override_chip_num != 0)
		A1_config_options.override_chip_num = override_chip_num;
	if (wiper != 0)
		A1_config_options.wiper = wiper;

	/* extra options */
	struct A1_extra_options *eo = &A1_extra_options;
	memset(eo, 0, sizeof(*eo));
	if (override_diff != 0) {
		applog(LOG_WARNING, "Using diff: %d", override_diff);
		A1_config_options.override_diff = override_diff;
	}
	if (board_mask != 0) {
		applog(LOG_WARNING, "Board mask: 0x%0x", board_mask);
		A1_config_options.chain_mask = board_mask;
	}

	A1_parse_option_array(clk_tmp, eo->sys_clk_khz, "sys_clk", false);
	A1_parse_option_array(wiper_tmp, eo->wiper, "wiper", true);
	A1_parse_option_array(cmask_tmp, eo->chip_bitmask, "chip_bitmask", true);
	A1_parse_option_array(sclk_tmp, eo->spi_clk_khz, "spi_clk", false);
	/* config options are global, scan them once */
	parsed_config_options = &A1_config_options;

	A1_print_config_options(parsed_config_options);
}


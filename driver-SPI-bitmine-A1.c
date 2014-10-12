/*
 * cgminer SPI driver for Bitmine.ch A1 devices
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
#include <float.h>

#include "spi-context.h"
#include "logging.h"
#include "miner.h"
#include "util.h"

#include "A1-common.h"
#include "A1-board-selector.h"
#include "A1-trimpot-mcp4x.h"


static void reset_nonce_stats(struct A1_chip *chip);


/* one global board_selector and spi context is enough */
static struct board_selector *board_selector;
static struct spi_ctx *spi0, *spi1;

/********** work queue */
static bool wq_enqueue(struct work_queue *wq, struct work *work)
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

static struct work *wq_dequeue(struct work_queue *wq)
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

/*
 * if not cooled sufficiently, communication fails and chip is temporary
 * disabled. we let it inactive for 30 seconds to cool down
 *
 * TODO: to be removed after bring up / test phase
 */
#define COOLDOWN_MS (30 * 1000)
/* if after this number of retries a chip is still inaccessible, disable it */
#define DISABLE_CHIP_FAIL_THRESHOLD	3


/*
 * for now, we have one global config, defaulting values:
 * - ref_clk 16MHz / sys_clk 800MHz
 * - 2000 kHz SPI clock
 */
struct A1_config_options A1_config_options = {
	.ref_clk_khz = 16000,
	.sys_clk_khz = 800000,
	.spi_clk_khz = 2000,
	.lower_clk_khz = 400000,
	.upper_clk_khz = 1100000,
	.lower_ratio_pm = 3,
	.upper_ratio_pm = 20,
};

/* override values with --bitmine-a1-options ref:sys:spi: - use 0 for default */
static struct A1_config_options *parsed_config_options;

#define MAX_BOARDS 16
struct A1_extra_options {
	int sys_clk_khz[MAX_BOARDS];
	int wiper[MAX_BOARDS];
	int chip_bitmask[MAX_BOARDS];
	int spi_clk_khz[MAX_BOARDS];
};

static struct A1_extra_options extra_options;

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

static void A1_parse_options(void)
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
	struct A1_extra_options *eo = &extra_options;
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


/********** A1 low level functions */
#define MAX_PLL_WAIT_CYCLES 25
#define PLL_CYCLE_WAIT_TIME 40
static bool check_chip_pll_lock(struct A1_chain *a1, int chip_id, uint8_t *wr)
{
	int n;
	for (n = 0; n < MAX_PLL_WAIT_CYCLES; n++) {
		/* check for PLL lock status */
		if (cmd_READ_REG(a1, chip_id) && (a1->spi_rx[4] & 1) == 1)
			/* double check that we read back what we set before */
			return wr[0] == a1->spi_rx[2] && wr[1] == a1->spi_rx[3];

		cgsleep_ms(PLL_CYCLE_WAIT_TIME);
	}
	applog(LOG_ERR, "%2d/%2d: failed PLL lock", a1->chain_id, chip_id);
	return false;
}

static uint8_t *get_pll_reg(struct A1_chain *a1, int ref_clock_khz,
			    int sys_clock_khz)
{
	/*
	 * PLL parameters after:
	 * sys_clk = (ref_clk * pll_fbdiv) / (pll_prediv * 2^(pll_postdiv - 1))
	 *
	 * with a higher pll_postdiv being desired over a higher pll_prediv
	 */

	static uint8_t writereg[6] = { 0x00, 0x00, 0x21, 0x84, };
	uint8_t pre_div = 1;
	uint8_t post_div = 1;
	uint32_t fb_div;

	int cid = a1->chain_id;

	applog(LOG_WARNING, "chain %d: Setting PLL: CLK_REF=%dMHz, SYS_CLK=%dMHz",
	       cid, ref_clock_khz / 1000, sys_clock_khz / 1000);

	/* Euclidean search for GCD */
	int a = ref_clock_khz;
	int b = sys_clock_khz;
	while (b != 0) {
		int h = a % b;
		a = b;
		b = h;
	}
	fb_div = sys_clock_khz / a;
	int n = ref_clock_khz / a;
	/* approximate multiplier if not exactly matchable */
	if (fb_div > 511) {
		int f = fb_div / n;
		int m = (f < 32) ? 16 : (f < 64) ? 8 :
			(f < 128) ? 4 : (256 < 2) ? 2 : 1;
		fb_div = m * fb_div / n;
		n =  m;
	}
	/* try to maximize post divider */
	if ((n & 3) == 0)
		post_div = 3;
	else if ((n & 1) == 0)
		post_div = 2;
	else
		post_div = 1;
	/* remainder goes to pre_div */
	pre_div = n / (1 << (post_div - 1));
	/* correct pre_div overflow */
	if (pre_div > 31) {
		fb_div = 31 * fb_div / pre_div;
		pre_div = 31;
	}
	writereg[0] = (post_div << 6) | (pre_div << 1) | (fb_div >> 8);
	writereg[1] = fb_div & 0xff;

	applog(LOG_WARNING, "chain %d: setting PLL: pre_div=%d, post_div=%d, "
	       "fb_div=%d: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x", cid,
	       pre_div, post_div, fb_div,
	       writereg[0], writereg[1], writereg[2],
	       writereg[3], writereg[4], writereg[5]);
	return writereg;
}

static bool set_pll_config(struct A1_chain *a1, int chip_id,
			   int ref_clock_khz, int sys_clock_khz)
{
	uint8_t *writereg = get_pll_reg(a1, ref_clock_khz, sys_clock_khz);
	if (writereg == NULL)
		return false;
	if (!cmd_WRITE_REG(a1, chip_id, writereg))
		return false;

	int from = (chip_id == 0) ? 0 : chip_id - 1;
	int to = (chip_id == 0) ? a1->num_active_chips : chip_id - 1;

	int i;
	for (i = from; i < to; i++) {
		int cid = i + 1;
		if (!check_chip_pll_lock(a1, chip_id, writereg)) {
			applog(LOG_ERR, "%2d/%2d: failed PLL lock",
			       a1->chain_id, cid);
			return false;
		}
	}
	return true;
}

#define WEAK_CHIP_THRESHOLD	30
#define BROKEN_CHIP_THRESHOLD	26
#define WEAK_CHIP_SYS_CLK	(600 * 1000)
#define BROKEN_CHIP_SYS_CLK	(400 * 1000)
static bool check_chip(struct A1_chain *a1, int i)
{
	struct A1_chip *chip = &a1->chips[i];
	int chip_id = i + 1;
	int cid = a1->chain_id;

	chip->a1 = a1;
	chip->chip_id = chip_id;
	chip->at_current.sys_clk = a1->sys_clk;
	/* reset twice for current and prev stats */
	reset_nonce_stats(chip);
	reset_nonce_stats(chip);

	if (extra_options.chip_bitmask[cid] & (1 << i)) {
		applog(LOG_WARNING, "%d: bypassing chip %d",
		       a1->chain_id, i);
		chip->num_cores = 0;
		chip->disabled = 1;
		return false;
	}

	if (!cmd_READ_REG(a1, chip_id)) {
		applog(LOG_WARNING, "%d: Failed to read register for "
		       "chip %d -> disabling", cid, chip_id);
		chip->num_cores = 0;
		chip->disabled = 1;
		return false;
	}
	chip->num_cores = a1->spi_rx[7];
	a1->num_cores += chip->num_cores;
	applog(LOG_WARNING, "%d: Found chip %d with %d active cores",
	       cid, chip_id, chip->num_cores);
	if (chip->num_cores < BROKEN_CHIP_THRESHOLD) {
		applog(LOG_WARNING, "%d: broken chip %d with %d active "
		       "cores (threshold = %d)", cid, chip_id,
		       chip->num_cores, BROKEN_CHIP_THRESHOLD);
		set_pll_config(a1, chip_id, A1_config_options.ref_clk_khz,
				BROKEN_CHIP_SYS_CLK);
		cmd_READ_REG(a1, chip_id);
		hexdump_error("new.PLL", a1->spi_rx, 8);
		chip->disabled = true;
		a1->num_cores -= chip->num_cores;
		return false;
	}

	if (chip->num_cores < WEAK_CHIP_THRESHOLD) {
		applog(LOG_WARNING, "%d: weak chip %d with %d active "
		       "cores (threshold = %d)", cid,
		       chip_id, chip->num_cores, WEAK_CHIP_THRESHOLD);
		set_pll_config(a1, chip_id, A1_config_options.ref_clk_khz,
			       WEAK_CHIP_SYS_CLK);
		cmd_READ_REG(a1, chip_id);
		hexdump_error("new.PLL", a1->spi_rx, 8);
		return false;
	}
	return true;
}

static int chain_detect(struct A1_chain *a1)
{
	int tx_len = 6;

	memset(a1->spi_tx, 0, tx_len);
	a1->spi_tx[0] = A1_RESET;
	a1->spi_tx[1] = 0;

	if (!spi_transfer(a1->spi_ctx, a1->spi_tx, a1->spi_rx, tx_len))
		return 0;
	hexdump("TX", a1->spi_tx, 6);
	hexdump("RX", a1->spi_rx, 6);

	int i;
	int cid = a1->chain_id;
	int max_poll_words = MAX_CHAIN_LENGTH * 2;
	for(i = 1; i < max_poll_words; i++) {
		if (a1->spi_rx[0] == A1_RESET && a1->spi_rx[1] == 0) {
			a1->num_chips = (i / 2) + 1;
			applog(LOG_WARNING, "%d: detected %d chips",
			       cid, a1->num_chips);
			return a1->num_chips;
		}
		bool s = spi_transfer(a1->spi_ctx, NULL, a1->spi_rx, 2);
		hexdump("RX", a1->spi_rx, 2);
		if (!s)
			return 0;
	}
	applog(LOG_WARNING, "%d: no A1 chip-chain detected", cid);
	return 0;
}

/********** disable / re-enable related section (temporary for testing) */
static bool is_chip_disabled(struct A1_chain *a1, uint8_t chip_id)
{
	struct A1_chip *chip = &a1->chips[chip_id - 1];
	return chip->disabled || chip->cooldown_begin != 0;
}

/* check and disable chip, remember time */
static void disable_chip(struct A1_chain *a1, uint8_t chip_id)
{
	flush_spi(a1);
	struct A1_chip *chip = &a1->chips[chip_id - 1];
	int cid = a1->chain_id;
	if (is_chip_disabled(a1, chip_id)) {
		applog(LOG_WARNING, "%2d/%2d: already disabled",
		       cid, chip_id);
		return;
	}
	applog(LOG_WARNING, "%2d/%2d: temporary disabling chip", cid, chip_id);
	chip->cooldown_begin = get_current_ms();
}

/* check if disabled chips can be re-enabled */
void check_disabled_chips(struct A1_chain *a1)
{
	int i;
	int cid = a1->chain_id;
	for (i = 0; i < a1->num_active_chips; i++) {
		int chip_id = i + 1;
		struct A1_chip *chip = &a1->chips[i];
		if (!is_chip_disabled(a1, chip_id))
			continue;
		/* do not re-enable fully disabled chips */
		if (chip->disabled)
			continue;
		if (chip->cooldown_begin + COOLDOWN_MS > get_current_ms())
			continue;
		if (!cmd_READ_REG(a1, chip_id)) {
			chip->fail_count++;
			applog(LOG_WARNING, "%2d/%2d: not yet working - %d",
			       cid, chip_id, chip->fail_count);
			if (chip->fail_count > DISABLE_CHIP_FAIL_THRESHOLD) {
				applog(LOG_WARNING,
				       "%2d/%2d: completely disabling chip at %d",
				       cid, chip_id, chip->fail_count);
				chip->disabled = true;
				a1->num_cores -= chip->num_cores;
				continue;
			}
			/* restart cooldown period */
			chip->cooldown_begin = get_current_ms();
			continue;
		}
		applog(LOG_WARNING, "%2d/%2d: chip is working again",
		       cid, chip_id);
		chip->cooldown_begin = 0;
		chip->fail_count = 0;
	}
}

///////////////////////////////////////////////////////////////////////////////
static void flush_chip(struct A1_chip *chip)
{
	int j;
	for (j = 0; j < 4; j++) {
		struct work *work = chip->work[j];
		if (work == NULL)
			continue;
		work_completed(chip->a1->cgpu, work);
		chip->work[j] = NULL;
	}
	chip->last_queued_id = 0;
}


static bool restart_chip(struct A1_chip *chip, int new_clk)
{
	if (!cmd_RESET(chip->a1, chip->chip_id, 0xe5)) {
		applog(LOG_WARNING, "%2d/%2d: chip reset failed",
		       chip->a1->chain_id, chip->chip_id);
		return false;
	}
	flush_chip(chip);
	return set_pll_config(chip->a1, chip->chip_id,
			      A1_config_options.ref_clk_khz, new_clk);

}
///////////////////////////////////////////////////////////////////////////////


#define BAD_NONCE_COUNT 5
#define NONCE_INTERVAL_N 200

static void reset_nonce_stats(struct A1_chip *chip)
{
	int now = get_current_ms();
	chip->at_prev = chip->at_current;
	chip->at_current.shares_ok = 0;
	chip->at_current.shares_nok = 0;
	chip->at_current.start_time = now;

	float nonces_per_sec = (chip->num_cores * chip->at_current.sys_clk) / 4294967.296;
	int wtime = (NONCE_INTERVAL_N * 1000.0) / nonces_per_sec;
	chip->at_current.end_time = now + wtime;
}
/* error ratio in permill */
#define MIN_NUM_NONCES 30
static int get_nonce_ratio(struct A1_chip *chip)
{
	int shares_all = chip->at_current.shares_nok + chip->at_current.shares_ok;
	if (shares_all < MIN_NUM_NONCES)
		return -1;
	return (chip->at_current.shares_nok * 1000 + shares_all / 2) / shares_all;
}

static FILE *stats_file;
static void init_stats(void)
{
	if (A1_config_options.stats_fname == NULL)
		return;
	if (stats_file != NULL)
		return;
	stats_file = fopen(A1_config_options.stats_fname, "w+");
	if (stats_file == NULL)
		applog(LOG_WARNING, "Failed to open stats file %s",
		       A1_config_options.stats_fname);

}
static void log_stat(struct A1_chip *chip, int ratio, bool bad)
{
	if (stats_file == NULL)
		return;
	fprintf(stats_file, "%s %s %d/%d: %d/%d-%d, %d (%d)\n",
		time_string(), bad ? "---" : "+++",
		chip->a1->chain_id, chip->chip_id,
		chip->at_current.shares_nok, chip->at_current.shares_ok,
		ratio, chip->at_current.sys_clk / 1000,
		chip->at_prev.sys_clk / 1000);
	fflush(stats_file);
}
static void log_change(struct A1_chip *chip, int ratio)
{
	if (stats_file == NULL)
		return;
	fprintf(stats_file, "%s %s CHANGE: %d/%d: %d/%d/%d %d->%d\n",
		time_string(),
		chip->at_prev.sys_clk < chip->at_current.sys_clk ? "+++" : "---",
		chip->a1->chain_id, chip->chip_id,
		chip->at_prev.shares_nok, chip->at_prev.shares_ok, ratio,
		chip->at_prev.sys_clk / 1000, chip->at_current.sys_clk / 1000);
	fflush(stats_file);
}

static void exit_stats(void)
{
	if (stats_file == NULL)
		return;
	fclose(stats_file);
	stats_file = NULL;
}
#define CLOCK_DELTA	(4 * 1000)
static bool adjust_clock(struct A1_chip *chip, int clock_delta, int ratio)
{
	reset_nonce_stats(chip);

	int new_clk = chip->at_current.sys_clk + clock_delta;
	if (new_clk == chip->at_current.sys_clk)
		return false;

	if (new_clk > A1_config_options.upper_clk_khz)
		new_clk = A1_config_options.upper_clk_khz;
	else if (new_clk < A1_config_options.lower_clk_khz)
		new_clk = A1_config_options.lower_clk_khz;
	if (!restart_chip(chip, new_clk))
		return false;
	chip->at_current.sys_clk = new_clk;
	log_change(chip, ratio);
	return true;
}
static bool add_nonce_bad(struct A1_chip *chip)
{
	chip->hw_errors++;
	chip->at_current.shares_nok++;
	if (chip->at_current.shares_nok < BAD_NONCE_COUNT)
		return false;

	int ratio = get_nonce_ratio(chip);
	if (ratio < 0)
		return false;
	log_stat(chip, ratio, true);

	if (!A1_config_options.enable_auto_tune)
		return false;
	if (ratio > A1_config_options.upper_ratio_pm) {
		if (chip->at_current.sys_clk > A1_config_options.lower_clk_khz)
			return adjust_clock(chip, -CLOCK_DELTA, ratio);

		if (stats_file != NULL)
			fprintf(stats_file, "%s %d/%d: limit reached: clk=%d\n",
				time_string(), chip->a1->chain_id, chip->chip_id,
				chip->at_current.sys_clk);
	}
	reset_nonce_stats(chip);
	return false;
}
/* check if chip can be uptuned, returns true if it did */
static bool check_uptune(struct A1_chip *chip)
{
	if (chip->at_current.sys_clk >= A1_config_options.upper_clk_khz) {
		// TODO: compare expected with real hash-rate
		return false;
	}
	int now = get_current_ms();
	if (/*chip->at_current.end_time == 0 || */(chip->at_current.end_time > now))
		return false;

	int ratio = get_nonce_ratio(chip);
	if (ratio < 0)
		return false;
	log_stat(chip, ratio, false);

	if (!A1_config_options.enable_auto_tune)
		return false;

	if (ratio < A1_config_options.lower_ratio_pm) {
		return adjust_clock(chip, CLOCK_DELTA, ratio);
	}
	reset_nonce_stats(chip);
	return false;
}
static bool add_nonce_good(struct A1_chip *chip)
{
	chip->nonces_found++;
	chip->at_current.shares_ok++;

	/* don't increase if we passed top already */
	if (chip->at_current.sys_clk <= chip->at_prev.sys_clk)
		return false;
	return check_uptune(chip);
}

///////////////////////////////////////////////////////////////////////////////
static uint8_t *create_job(uint8_t chip_id, uint8_t job_id, struct work *work)
{
	static uint8_t job[WRITE_JOB_LENGTH] = {
		/* command */
		0x00, 0x00,
		/* midstate */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		/* wdata */
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00,
		/* start nonce */
		0x00, 0x00, 0x00, 0x00,
		/* difficulty 1 */
		0xff, 0xff, 0x00, 0x1d,
		/* end nonce */
		0xff, 0xff, 0xff, 0xff,
	};
	uint8_t *midstate = work->midstate;
	uint8_t *wdata = work->data + 64;

	uint32_t *p1 = (uint32_t *) &job[34];
	uint32_t *p2 = (uint32_t *) wdata;

	job[0] = (job_id << 4) | A1_WRITE_JOB;
	job[1] = chip_id;

	swab256(job + 2, midstate);
	p1[0] = bswap_32(p2[0]);
	p1[1] = bswap_32(p2[1]);
	p1[2] = bswap_32(p2[2]);

	if (A1_config_options.override_diff != 0) {
		double diff = work->device_diff;
		int od = A1_config_options.override_diff;
		if (od != -1 && od < diff)
			diff = od;
		p1[4] = get_target(diff);
	}
	int rdd = (int)round(work->device_diff);
	if (A1_config_options.override_diff != rdd) {
		applog(LOG_WARNING, "job-target: %d / %d / %f",
			A1_config_options.override_diff, rdd, work->sdiff);
	}
	return job;
}

/* set work for given chip, returns true if a nonce range was finished */
static bool set_work(struct A1_chain *a1, uint8_t chip_id, struct work *work,
		     uint8_t queue_states)
{
	int cid = a1->chain_id;
	struct A1_chip *chip = &a1->chips[chip_id - 1];
	bool retval = false;

	int job_id = chip->last_queued_id + 1;

	applog(LOG_INFO, "%2d/%2d: queuing job_id %d, state=0x%02x",
	       cid, chip_id, job_id, queue_states);
	if (job_id == (queue_states & 0x0f) || job_id == (queue_states >> 4))
		applog(LOG_WARNING, "%2d/%2d: job overlap: %d, 0x%02x",
		       cid, chip_id, job_id, queue_states);

	if (chip->work[chip->last_queued_id] != NULL) {
		work_completed(a1->cgpu, chip->work[chip->last_queued_id]);
		chip->work[chip->last_queued_id] = NULL;
		retval = true;
	}
	uint8_t *jobdata = create_job(chip_id, job_id, work);
	if (!cmd_WRITE_JOB(a1, chip_id, jobdata)) {
		/* give back work */
		work_completed(a1->cgpu, work);

		applog(LOG_ERR, "%2d/%2d: failed to set work id %d",
		       cid, chip_id, job_id);
		disable_chip(a1, chip_id);
	} else {
		chip->work[chip->last_queued_id] = work;
		chip->last_queued_id++;
		chip->last_queued_id &= 3;
	}
	return retval;
}

static bool get_nonce(struct A1_chain *a1, uint8_t *nonce,
		      uint8_t *chip, uint8_t *job_id)
{
	uint8_t *ret = cmd_READ_RESULT_BCAST(a1);
	if (ret == NULL)
		return false;
	if (ret[1] == 0) {
		applog(LOG_DEBUG, "chain %d: output queue empty", a1->chain_id);
		return false;
	}
	*job_id = ret[0] >> 4;
	*chip = ret[1];
	memcpy(nonce, ret + 2, 4);
	return true;
}

/* reset input work queues in chip chain */
static bool abort_work(struct A1_chain *a1)
{
	/* drop jobs already queued: reset strategy 0xe5 */
	return cmd_RESET_BCAST(a1, 0xe5);
}

/********** driver interface */
void exit_A1_chain(struct A1_chain *a1)
{
	if (a1 == NULL)
		return;
	free(a1->chips);
	a1->chips = NULL;
	a1->spi_ctx = NULL;
	free(a1);
}

static void set_spi_clk(struct A1_chain *a1)
{
	int cid = a1->chain_id;
	if (extra_options.spi_clk_khz[cid] != 0)
		a1->spi_ctx->config.speed = extra_options.spi_clk_khz[cid] * 1000;
	else
		a1->spi_ctx->config.speed = A1_config_options.spi_clk_khz * 1000;
}

struct A1_chain *init_A1_chain(struct spi_ctx *ctx, int chain_id)
{
	if (A1_config_options.chain_mask & (1 << chain_id)) {
		applog(LOG_WARNING, "chain %d: masked -> bypassing", chain_id);
		return false;
	}

	int i;
	struct A1_chain *a1 = malloc(sizeof(*a1));
	assert(a1 != NULL);

	applog(LOG_DEBUG, "%d: A1 init chain", chain_id);
	memset(a1, 0, sizeof(*a1));
	a1->spi_ctx = ctx;
	a1->chain_id = chain_id;

	a1->num_chips = chain_detect(a1);
	if (a1->num_chips == 0)
		goto failure;

	applog(LOG_WARNING, "spidev%d.%d: %d: Found %d A1 chips",
	       a1->spi_ctx->config.bus, a1->spi_ctx->config.cs_line,
	       a1->chain_id, a1->num_chips);

	// do the BIST with a clock-multiplier of 12.5 (200MHz @ 16MHz)
	static uint8_t initial_pll[6] = { 0x82, 0x19, 0x21, 0x84, };
	// start with a 100kHz SPI clock
	a1->spi_ctx->config.speed = 100 * 1000;

	if (cmd_WRITE_REG(a1, 0, initial_pll) == NULL)
		goto failure;

	if (cmd_BIST_START_BCAST(a1) == NULL)
		goto failure;

	int sys_clk = A1_config_options.sys_clk_khz;
	if (extra_options.sys_clk_khz[a1->chain_id] != 0)
		sys_clk = extra_options.sys_clk_khz[a1->chain_id];
	a1->sys_clk = sys_clk;
	if (!set_pll_config(a1, 0, A1_config_options.ref_clk_khz, sys_clk))
		goto failure;

	set_spi_clk(a1);
	applog(LOG_WARNING, "%d: spi_clk = %d kHz",
	       chain_id, a1->spi_ctx->config.speed / 1000);

	/* override max number of active chips if requested */
	a1->num_active_chips = a1->num_chips;
	if (A1_config_options.override_chip_num > 0 &&
	    a1->num_chips > A1_config_options.override_chip_num) {
		a1->num_active_chips = A1_config_options.override_chip_num;
		applog(LOG_WARNING, "%d: limiting chain to %d chips",
		       a1->chain_id, a1->num_active_chips);
	}

	a1->chips = calloc(a1->num_active_chips, sizeof(struct A1_chip));
	assert (a1->chips != NULL);

	if (!cmd_BIST_FIX_BCAST(a1))
		goto failure;

	for (i = 0; i < a1->num_active_chips; i++)
		check_chip(a1, i);

	applog(LOG_WARNING, "%d: found %d chips with total %d active cores",
	       a1->chain_id, a1->num_active_chips, a1->num_cores);

	mutex_init(&a1->lock);
	INIT_LIST_HEAD(&a1->active_wq.head);

	return a1;

failure:
	exit_A1_chain(a1);
	return NULL;
}

static void A1_add_cgpu(struct A1_chain *a1, const char *name)
{
	struct cgpu_info *cgpu = malloc(sizeof(*cgpu));
	assert(cgpu != NULL);

	memset(cgpu, 0, sizeof(*cgpu));
	cgpu->drv = &bitmineA1_drv;
	cgpu->name = (char*)name;
	cgpu->threads = 1;

	cgpu->device_data = a1;

	a1->cgpu = cgpu;
	add_cgpu(cgpu);
}
static bool detect_single_chain(void)
{
	board_selector = (struct board_selector*)&dummy_board_selector;
	applog(LOG_WARNING, "A1: checking single chain");
	struct A1_chain *a1 = init_A1_chain(spi0, 0);
	if (a1 == NULL)
		return false;

	A1_add_cgpu(a1, "BitmineA1.SingleChain");
	applog(LOG_WARNING, "Detected single A1 chain with %d chips / %d cores",
	       a1->num_active_chips, a1->num_cores);
	return true;
}

static void set_ccd_wiper(struct mcp4x *mcp, int board_id)
{
	if (extra_options.wiper[board_id] != 0) {
		applog(LOG_WARNING, "%d: setting individual wiper 0x%x",
		       board_id, extra_options.wiper[board_id]);
		mcp->set_wiper(mcp, 0, extra_options.wiper[board_id]);
	} else if (A1_config_options.wiper != 0) {
		mcp->set_wiper(mcp, 0, A1_config_options.wiper);
		applog(LOG_WARNING, "%d: setting global wiper 0x%x",
		       board_id, A1_config_options.wiper);
	}

}
bool detect_coincraft_desk(void)
{
	static const uint8_t mcp4x_mapping[] = { 0x2c, 0x2b, 0x2a, 0x29, 0x28 };
	board_selector = ccd_board_selector_init();
	if (board_selector == NULL) {
		applog(LOG_INFO, "No CoinCrafd Desk backplane detected.");
		return false;
	}
	board_selector->reset_all();

	int boards_detected = 0;
	int board_id;
	for (board_id = 0; board_id < CCD_MAX_CHAINS; board_id++) {
		uint8_t mcp_slave = mcp4x_mapping[board_id];
		struct mcp4x *mcp = mcp4x_init(mcp_slave);
		if (mcp == NULL)
			continue;

		set_ccd_wiper(mcp, board_id);

		applog(LOG_WARNING, "checking board %d...", board_id);
		board_selector->select(board_id);

		struct A1_chain *a1 = init_A1_chain(spi0, board_id);
		board_selector->release();
		if (a1 == NULL)
			continue;

		A1_add_cgpu(a1, "BitmineA1.CCD");
		boards_detected++;
	}
	if (boards_detected == 0)
		return false;

	applog(LOG_WARNING, "Detected CoinCraft Desk with %d boards",
	       boards_detected);
	return true;
}

bool detect_coincraft_blade(void)
{
	board_selector = ccb_board_selector_init();
	if (board_selector == NULL) {
		applog(LOG_INFO, "No CoinCraft Blade backplane detected.");
		return false;
	}
	board_selector->reset_all();

	int boards_detected = 0;
	int board_id;
	for (board_id = 0; board_id < CCB_MAX_CHAINS; board_id++) {
		if (A1_config_options.chain_mask & (1 << board_id))
			continue;
		applog(LOG_WARNING, "checking board %d...", board_id);
		board_selector->select(board_id);

//		board_selector->reset();
//		cgsleep_ms(250);

		struct spi_ctx *spi = (board_id & 1) ? spi1 : spi0;
		struct A1_chain *a1 = init_A1_chain(spi, board_id);
		board_selector->release();
		if (a1 == NULL)
			continue;

		A1_add_cgpu(a1, "BitmineA1.CCB");
		boards_detected++;
	}
	if (boards_detected == 0)
		return false;

	applog(LOG_WARNING, "Detected CoinCraft Blade with %d boards",
	       boards_detected);
	return true;
}

bool detect_coincraft_rig_v3(void)
{
	board_selector = ccr_board_selector_init();
	if (board_selector == NULL)
		return false;

	board_selector->reset_all();
	int chains_detected = 0;
	int c;
	for (c = 0; c < CCR_MAX_CHAINS; c++) {
		applog(LOG_WARNING, "checking RIG chain %d...", c);

		if (!board_selector->select(c))
			continue;

		struct A1_chain *a1 = init_A1_chain(spi0, c);
		board_selector->release();

		if (a1 == NULL)
			continue;

		if (A1_config_options.wiper != 0 && (c & 1) == 0) {
			struct mcp4x *mcp = mcp4x_init(0x28);
			if (mcp == NULL) {
				applog(LOG_ERR, "%d: Cant access poti", c);
			} else {
				mcp->set_wiper(mcp, 0, A1_config_options.wiper);
				mcp->set_wiper(mcp, 1, A1_config_options.wiper);
				mcp->exit(mcp);
				applog(LOG_WARNING, "%d: set wiper to 0x%02x",
					c, A1_config_options.wiper);
			}
		}

		A1_add_cgpu(a1, "BitmineA1.CCR");
		chains_detected++;
	}
	if (chains_detected == 0)
		return false;

	applog(LOG_WARNING, "Detected CoinCraft Rig with %d chains",
	       chains_detected);
	return true;
}

/* Probe SPI channel and register chip chain */
void A1_detect(bool hotplug)
{
	/* no hotplug support for SPI */
	if (hotplug)
		return;

	bool detected = false;
	A1_parse_options();
	applog(LOG_DEBUG, "A1 detect");

	/* register global SPI context */
	struct spi_config cfg = default_spi_config;
	cfg.mode = SPI_MODE_1;
	cfg.speed = A1_config_options.spi_clk_khz * 1000;
	spi0 = spi_init(&cfg);
	cfg.cs_line = 1;
	spi1 = spi_init(&cfg);
	if (spi0 == NULL || spi1 == NULL)
		return;

	/* detect and register supported products */
	detected = detect_coincraft_desk() ||
		   detect_coincraft_blade() ||
		   detect_coincraft_rig_v3() ||
		   detect_single_chain();

	if (detected) {
		init_stats();
		return;
	}
	/* release SPI context if no A1 products found */
	spi_exit(spi0);
	spi_exit(spi1);
}

#define TEMP_UPDATE_INT_MS	2000
#define TEMP_THROTTLE_SLEEP_MS	5000
#define IDLE_SLEEP_MS		120
static int64_t A1_scanwork(struct thr_info *thr)
{
	int i;
	struct cgpu_info *cgpu = thr->cgpu;
	struct A1_chain *a1 = cgpu->device_data;
	int sleep_ms = IDLE_SLEEP_MS;

	if (a1->num_cores == 0) {
		cgpu->deven = DEV_DISABLED;
		return 0;
	}
	if (thr->work_restart)
		return 0;

	board_selector->select(a1->chain_id);

	set_spi_clk(a1);

	applog(LOG_DEBUG, "A1 running scanwork");

	uint32_t nonce;
	uint8_t chip_id;
	uint8_t job_id;

	mutex_lock(&a1->lock);

	if (a1->last_temp_time + TEMP_UPDATE_INT_MS < get_current_ms()) {
		a1->temp = board_selector->get_temp(0);
		a1->last_temp_time = get_current_ms();
		cgpu->temp = a1->temp;
	}
	int cid = a1->chain_id;
	/* poll queued results */
	while (true) {
		if (thr->work_restart)
			goto abort;
		if (!get_nonce(a1, (uint8_t*)&nonce, &chip_id, &job_id))
			break;
		nonce = bswap_32(nonce);
		if (chip_id < 1 || chip_id > a1->num_active_chips) {
			applog(LOG_WARNING, "chain %d: wrong chip_id %d",
			       cid, chip_id);
			continue;
		}
		if (job_id < 1 && job_id > 4) {
			applog(LOG_WARNING, "%2d/%2d: wrong result job_id %d",
			       cid, chip_id, job_id);
			flush_spi(a1);
			continue;
		}

		struct A1_chip *chip = &a1->chips[chip_id - 1];
		struct work *work = chip->work[job_id - 1];
		if (work == NULL) {
			/* already been flushed => stale */
			applog(LOG_WARNING, "%2d/%2d: stale nonce 0x%08x",
			       cid, chip_id, nonce);
			chip->stales++;
			continue;
		}
		if (!submit_nonce(thr, work, nonce)) {
			int penalty = (int)work->device_diff;
			a1->nonce_ranges_processed -= penalty;
			applog(LOG_WARNING, "%2d/%2d: invalid nonce 0x%08x, penalty=%d (%d)",
			       cid, chip_id, nonce, penalty, a1->nonce_ranges_processed);
			add_nonce_bad(chip);
			continue;
		}
		applog(LOG_DEBUG, "YEAH: %d: chip %d / job_id %d: nonce 0x%08x",
		       cid, chip_id, job_id, nonce);
		add_nonce_good(chip);
	}

	if (cgpu->cutofftemp > 0 && a1->temp > cgpu->cutofftemp) {
		applog(LOG_WARNING, "%d: throttling at %d (%d)",
		       cid, a1->temp, cgpu->cutofftemp);
		sleep_ms = TEMP_THROTTLE_SLEEP_MS;
		goto done;
	}
	/* check for completed works */
	for (i = a1->num_active_chips; i > 0; i--) {
		if (thr->work_restart)
			goto abort;
		uint8_t c = i;
		if (is_chip_disabled(a1, c))
			continue;
		if (!cmd_READ_REG(a1, c)) {
			disable_chip(a1, c);
			continue;
		}
		uint8_t qstate = a1->spi_rx[5] & 3;
		uint8_t qbuff = a1->spi_rx[6];
		struct work *work;
		struct A1_chip *chip = &a1->chips[i - 1];
		switch(qstate) {
		case 3:
			continue;
		case 2:
			applog(LOG_ERR, "%d: chip %d: invalid state = 2",
			       cid, c);
			continue;
		case 0:
			work = wq_dequeue(&a1->active_wq);
			if (work == NULL) {
				applog(LOG_INFO, "%d: chip %d: work underflow",
				       cid, c);
				break;
			}
			if (set_work(a1, c, work, qbuff)) {
				chip->nonce_ranges_done++;
				a1->nonce_ranges_processed++;
			}
			/* fall through */
		case 1:
			work = wq_dequeue(&a1->active_wq);
			if (work == NULL) {
				applog(LOG_INFO, "%d: chip %d: work underflow",
				       cid, c);
				break;
			}
			if (set_work(a1, c, work, qbuff)) {
				chip->nonce_ranges_done++;
				a1->nonce_ranges_processed++;
			}
			applog(LOG_DEBUG, "%d: chip %d: job done: %d/%d/%d/%d",
			       cid, c,
			       chip->nonce_ranges_done, chip->nonces_found,
			       chip->hw_errors, chip->stales);
			break;
		}
	}
	check_disabled_chips(a1);
done:
	mutex_unlock(&a1->lock);

	board_selector->release();

	int64_t ret = 0;
	if (a1->nonce_ranges_processed < 0) {
		applog(LOG_DEBUG, "%d, negative nonces processed %d",
		       cid, a1->nonce_ranges_processed);
	} else {
		applog(LOG_DEBUG, "%d, nonces processed %d",
		       cid, a1->nonce_ranges_processed);
		ret = a1->nonce_ranges_processed;
		ret <<= 32;
		a1->nonce_ranges_processed = 0;
	}
	cgsleep_ms(sleep_ms);
	return ret;

abort:
	mutex_unlock(&a1->lock);
	board_selector->release();
	return 0;
}


/* queue two work items per chip in chain */
static bool A1_queue_full(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int queue_full = false;

	mutex_lock(&a1->lock);
	applog(LOG_DEBUG, "%d, A1 running queue_full: %d/%d",
	       a1->chain_id, a1->active_wq.num_elems, a1->num_active_chips);

	if (a1->active_wq.num_elems >= a1->num_active_chips * 2)
		queue_full = true;
	else
		wq_enqueue(&a1->active_wq, get_queued(cgpu));

	mutex_unlock(&a1->lock);

	return queue_full;
}

static void A1_flush_work(struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	int cid = a1->chain_id;
	board_selector->select(cid);

	set_spi_clk(a1);

	applog(LOG_DEBUG, "%d: A1 running flushwork", cid);

	int i;

	mutex_lock(&a1->lock);
	/* stop chips hashing current work */
	if (!abort_work(a1)) {
		applog(LOG_ERR, "%d: failed to abort work in chip chain!", cid);
	}
	/* flush the work chips were currently hashing */
	for (i = 0; i < a1->num_active_chips; i++) {
		struct A1_chip *chip = &a1->chips[i];

		/* if chip was uptuned, it is already reset */
		if (check_uptune(chip))
			continue;
		flush_chip(chip);
	}
	/* flush queued work */
	applog(LOG_DEBUG, "%d: flushing queued work...", cid);
	while (a1->active_wq.num_elems > 0) {
		struct work *work = wq_dequeue(&a1->active_wq);
		assert(work != NULL);
		work_completed(cgpu, work);
	}
	mutex_unlock(&a1->lock);

	board_selector->release();
}

static void A1_get_statline_before(char *buf, size_t len,
				   struct cgpu_info *cgpu)
{
	struct A1_chain *a1 = cgpu->device_data;
	char temp[10];
	if (a1->temp != 0)
		snprintf(temp, 9, "%2dC", a1->temp);
	tailsprintf(buf, len, " %2d:%2d/%3d %s",
		    a1->chain_id, a1->num_active_chips, a1->num_cores,
		    a1->temp == 0 ? "   " : temp);
}

static void A1_shutdown(struct thr_info *thr)
{
	struct cgpu_info *cgpu = thr->cgpu;
	exit_stats();
	applog(LOG_DEBUG, "Closing %s", cgpu->name);
	spi_exit(spi0);
	spi_exit(spi1);
	spi0 = NULL;
	spi1 = NULL;
}

struct device_drv bitmineA1_drv = {
	.drv_id = DRIVER_bitmineA1,
	.dname = "BitmineA1",
	.name = "BA1",
	.drv_detect = A1_detect,
	.thread_shutdown = A1_shutdown,

	.hash_work = hash_queued_work,
	.scanwork = A1_scanwork,
	.queue_full = A1_queue_full,
	.flush_work = A1_flush_work,
	.get_statline_before = A1_get_statline_before,
};

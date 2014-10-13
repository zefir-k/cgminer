#ifndef A1_COMMON_H
#define A1_COMMON_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/* everything cgminer needs to know from A1 */


/********** global config paramters */
struct A1_config_options {
	int ref_clk_khz;
	int sys_clk_khz;
	int spi_clk_khz;
	int chain_mask;
	int override_diff; /* -1 = real diff, 0 = diff1 */
	const char *stats_fname;
	const char *config_fname;
	bool enable_auto_tune;
	int lower_ratio_pm;
	int upper_ratio_pm;
	int lower_clk_khz;
	int upper_clk_khz;

	/* limit chip chain to this number of chips (testing only) */
	int override_chip_num;
	int wiper;
};

/* global configuration instance */
extern struct A1_config_options A1_config_options;

extern char *opt_bitmine_a1_options;

#endif /* A1_COMMON_H */

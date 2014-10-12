#ifndef A1_COMMON_H
#define A1_COMMON_H

#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

/********** work queue */
struct work_ent {
	struct work *work;
	struct list_head head;
};

struct work_queue {
	int num_elems;
	struct list_head head;
};

/********** chip and chain context structures */
/* the WRITE_JOB command is the largest (2 bytes command, 56 bytes payload) */
#define WRITE_JOB_LENGTH	58
#define MAX_CHAIN_LENGTH	64
/*
 * For commands to traverse the chain, we need to issue dummy writes to
 * keep SPI clock running. To reach the last chip in the chain, we need to
 * write the command, followed by chain-length words to pass it through the
 * chain and another chain-length words to get the ACK back to host
 */
#define MAX_CMD_LENGTH		(WRITE_JOB_LENGTH + MAX_CHAIN_LENGTH * 2 * 2)

struct A1_chain;

struct A1_autotune_stats {
	int shares_ok;
	int shares_nok;
	int start_time;
	int end_time;
	int sys_clk;
};
struct A1_chip {
	int chip_id;
	struct A1_chain *a1;
	int num_cores;
	int last_queued_id;
	struct work *work[4];
	/* stats */
	int hw_errors;
	int stales;
	int nonces_found;
	int nonce_ranges_done;

	/* systime in ms when chip was disabled */
	int cooldown_begin;
	/* number of consecutive failures to access the chip */
	int fail_count;
	/* mark chip disabled, do not try to re-enable it */
	bool disabled;

	/* frequency tuning */
	struct A1_autotune_stats at_prev;
	struct A1_autotune_stats at_current;
};

struct A1_chain {
	int chain_id;
	struct cgpu_info *cgpu;
	struct mcp4x *trimpot;
	int num_chips;
	int num_cores;
	int num_active_chips;
	int chain_skew;
	uint8_t spi_tx[MAX_CMD_LENGTH];
	uint8_t spi_rx[MAX_CMD_LENGTH];
	struct spi_ctx *spi_ctx;
	struct A1_chip *chips;
	pthread_mutex_t lock;

	struct work_queue active_wq;

	/* mark chain disabled, do not try to re-enable it */
	bool disabled;
	uint8_t temp;
	int last_temp_time;

	int sys_clk;

	/* accounting nonces processed over error penalty */
	int nonce_ranges_processed;
};

#define MAX_CHAINS_PER_BOARD	2
struct A1_board {
	int board_id;
	int num_chains;
	struct A1_chain *chain[MAX_CHAINS_PER_BOARD];
};

/********** config paramters */
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

/* A1-utils */
extern int get_current_ms(void);
extern void hexdump(char *prefix, uint8_t *buff, int len);
extern void hexdump_error(char *prefix, uint8_t *buff, int len);
extern const char *time_string(void);
extern uint32_t get_target(double diff);

/* A1-layer-SPI */
enum A1_command {
	A1_BIST_START		= 0x01,
	A1_BIST_FIX		= 0x03,
	A1_RESET		= 0x04,
	A1_WRITE_JOB		= 0x07,
	A1_READ_RESULT		= 0x08,
	A1_WRITE_REG		= 0x09,
	A1_READ_REG		= 0x0a,
	A1_READ_REG_RESP	= 0x1a,
};

extern void flush_spi(struct A1_chain *a1);
extern uint8_t *cmd_BIST_START_BCAST(struct A1_chain *a1);
extern uint8_t *cmd_BIST_FIX_BCAST(struct A1_chain *a1);
extern uint8_t *cmd_RESET_BCAST(struct A1_chain *a1, uint8_t strategy);
extern uint8_t *cmd_RESET(struct A1_chain *a1, int cid, uint8_t strategy);
extern uint8_t *cmd_READ_RESULT_BCAST(struct A1_chain *a1);
extern uint8_t *cmd_WRITE_REG(struct A1_chain *a1, uint8_t chip, uint8_t *reg);
extern uint8_t *cmd_READ_REG(struct A1_chain *a1, uint8_t chip);
extern uint8_t *cmd_WRITE_JOB(struct A1_chain *a1, uint8_t chip_id, uint8_t *job);


#endif /* A1_COMMON_H */

/*
 * Live update hooks
 */

#ifndef ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_
#define ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_

#include <autoconf.h>
#include <drivers/uart.h>
#include <drivers/gpio.h>
#include <zephyr/types.h>

#define LIVE_UPDATE_CURRENT_VERSION 11
#define LIVE_UPDATE_MAX_BYTES 0x6000
#define LIVE_UPDATE_READ_SIZE 1024 // bytes read at a time in idle loop

extern volatile u32_t __update_flag;

struct update_header {
    u32_t version;
    u32_t main_ptr_addr;
    u32_t main_ptr;
    u32_t update_flag_addr;
    u32_t text_start;
    u32_t text_size;
    u32_t rodata_start;
    u32_t rodata_size;
    u32_t bss_start;
    u32_t bss_size;
    u32_t bss_start_addr;
    u32_t bss_size_addr;
    u32_t payload_size;
} __attribute__((packed));

struct predicate_header {
    u32_t size;
    u32_t *event_handler_addr;
    u32_t *updated_event_handler_addr;
    u32_t n_inactive_ops;
    u32_t n_constraints;
    u32_t n_state_init;
    u32_t hw_transfer_size;
} __attribute__((packed));

struct predicate_inactive_operation {
    u32_t inactive_op_ptr;
} __attribute__((packed));

struct predicate_constraint {
    u32_t size;
    u32_t symbol_addr;
    u32_t n_ranges;
} __attribute__((packed));

struct predicate_constraint_range {
    u32_t lower;
    u32_t upper;
} __attribute__((packed));

struct predicate_state_transfer {
    u32_t *addr;
    u32_t offset;
    u32_t val;
} __attribute__((packed));

struct predicate_hw_transfer {
    u32_t size;
    u32_t fn_ptr;
    u32_t args; 
} __attribute__((packed));

struct predicates_header {
    u32_t size;
} __attribute__((packed));

struct transfer {
    u32_t *origin;
    u32_t *dest;
    u32_t size;
} __attribute__((packed));

struct transfers_header {
    u32_t size;
} __attribute__((packed));

struct hw_init {
    u32_t size;
    u32_t fn_ptr;
    u32_t args;
} __attribute__((packed));

struct hw_init_header {
    u32_t size;
} __attribute__((packed));

struct mem_init {
    u32_t *addr;
    u32_t offset;
    u32_t val;
} __attribute__((packed));

struct mem_init_header {
    u32_t size;
} __attribute__((packed));


//} __attribute__((packed, aligned(1)));

void lu_main(void);

bool lu_trigger_on_timer(struct k_timer *);
bool lu_trigger_on_gpio(u32_t);

void lu_update_at_timer(struct k_timer **);
void lu_update_at_gpio(struct gpio_callback **);

void lu_write_step(void);
void lu_uart_reset(void);

#endif // ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_


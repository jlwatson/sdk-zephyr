/*
 * Live update hooks
 */

#ifndef ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_
#define ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_

#include <autoconf.h>
#include <drivers/uart.h>
#include <zephyr/types.h>

#include <tfm_gpio_veneers.h>
#include <tfm_flash_veneers.h>

#define LIVE_UPDATE_CURRENT_VERSION 10
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
    u32_t n_predicates;
    u32_t predicates_size;
    u32_t transfers_size;
} __attribute__((packed));

struct predicate_header {
    u32_t size;
    u32_t event_handler_addr;
    u32_t n_memory_checks;
    u32_t n_active_timers;
    u32_t n_gpio_interrupt_cbs;
    u32_t gpio_interrupt_enabled;
    u32_t gpio_out_enabled;
    u32_t gpio_out_set;
} __attribute__((packed));

struct predicate_memory_check {
    u32_t check_addr;
    u32_t check_size;
    u8_t *check_value;
} __attribute__((packed, aligned(1)));

struct predicate_active_timer {
    u32_t base_addr;
    u32_t duration;
    u32_t period; 
} __attribute__((packed));

struct predicate_gpio_interrupt_cb {
    u32_t pin;
    u32_t cb_addr;
} __attribute__((packed));

struct transfer_header {
    u32_t size;
    u32_t new_event_handler_addr;
    u32_t n_memory;
    u32_t n_init_memory;
    u32_t n_timers;
    u32_t n_active_timers;
    u32_t n_gpio_interrupt_cbs;
    u32_t gpio_interrupt_enabled;
    u32_t gpio_out_enabled;
    u32_t gpio_out_set;
} __attribute__((packed, aligned(1)));

struct transfer_memory {
    u32_t src_addr;
    u32_t dst_addr;
    u32_t size; 
} __attribute__((packed, aligned(1)));

struct transfer_init_memory {
    u32_t addr;
    u32_t size;
    u8_t *value;
} __attribute__((packed, aligned(1)));

struct transfer_timer {
    u32_t base_addr;
    u32_t expire_cb;
    u32_t stop_cb;
    u32_t duration;
    u32_t period;
} __attribute__((packed, aligned(1)));

struct transfer_active_timer {
    u32_t base_addr;
    u32_t duration;
    u32_t period;
} __attribute__((packed, aligned(1)));

struct transfer_gpio_interrupt_cb {
    u32_t pin;
    u32_t cb_addr;
} __attribute__((packed, aligned(1)));

void lu_main(void);

bool lu_trigger_on_timer(struct k_timer *);
bool lu_trigger_on_gpio(u32_t);

void lu_update_at_timer(struct k_timer **);
void lu_update_at_gpio();

void lu_uart_idle_read(void);
void lu_uart_reset(void);

#endif // ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_


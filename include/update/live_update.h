/*
 * Live update hooks
 */

#ifndef ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_
#define ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_

#include <autoconf.h>
#include <drivers/uart.h>
#include <zephyr/types.h>

#include <tfm_flash_veneers.h>

#define LIVE_UPDATE_CURRENT_VERSION 0x8
#define LIVE_UPDATE_MAX_BYTES 4096
#define LIVE_UPDATE_READ_SIZE 32 // bytes read at a time in idle loop

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
    u32_t transfer_triples_size;
    u32_t init_size;
};

void lu_main(void);
bool lu_trigger_on_timer(void);
void lu_state_transfer_timer(struct k_timer **);
void lu_uart_idle_read(void);

#endif // ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_


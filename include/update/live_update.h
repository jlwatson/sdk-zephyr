/*
 * Live update hooks
 */

#ifndef ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_
#define ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_

#include <autoconf.h>
#include <drivers/uart.h>
#include <tfm_flash_veneers.h>
#include <zephyr/types.h>

#define LIVE_UPDATE_CURRENT_VERSION 0x6
#define LIVE_UPDATE_MAX_BYTES 4096

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
};

void lu_main(void);

#endif // ZEPHYR_INCLUDE_UPDATE_LIVE_UPDATE_H_


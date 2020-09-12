/*
 * Live update - write update to flash
 */

#include <drivers/flash.h>
#include <kernel.h>
#include <string.h>
#include <timeout_q.h>
#include <wait_q.h>
#include <update/live_update.h>

#define FLASH_PAGE_SIZE 0x1000

extern volatile u32_t *DWT_CYCCNT;
extern u32_t update_counter;

// prototypes
void lu_write_update(struct update_header *hdr);

void _lu_write_text();
void _lu_write_rodata();
void _lu_write_bss_loc();
void _lu_write_bss_size();
void _lu_write_main_ptr();
void _lu_write_update_flag();
void _lu_write_finalize();

struct update_header *lu_hdr;
u8_t update_write_completed = 0;

static struct device *flash_dev;

typedef void _cont(void);
_cont *continuation = NULL;

// Info to sequence single flash page writes, then move to the next higher-level operation
size_t flash_page_size;
u8_t flash_page_buffer[FLASH_PAGE_SIZE]; // I cheat since I actually know the flash page size :)

_cont *next = NULL;
u8_t *write_data = NULL;
u32_t write_dest_addr = 0;
u32_t write_bytes_remaining = 0;

void lu_write_init() {
    flash_dev = device_get_binding("NRF_FLASH_DRV_NAME");
}

void _lu_write_single_page() {

    u32_t page_offset = write_dest_addr % FLASH_PAGE_SIZE;
    u32_t page_base = write_dest_addr - page_offset;
    u32_t bytes_left_in_page = FLASH_PAGE_SIZE - page_offset;

    // cap write to the extent of a single page
    u32_t bytes_to_write = bytes_left_in_page >= write_bytes_remaining ? write_bytes_remaining : bytes_left_in_page;

    // if we aren't writing a whole page, read the contents so they aren't erased
    if (bytes_to_write != FLASH_PAGE_SIZE) {
        int ret = flash_read(flash_dev, page_base, flash_page_buffer, FLASH_PAGE_SIZE);
        if (ret) printk("flash_read failed with code: %d\n", ret);
    }
    memcpy(flash_page_buffer + page_offset, write_data, bytes_to_write);

    int ret = flash_write_protection_set(flash_dev, false);
    if (ret) printk("flash_write_protection_set (1) failed with code: %d\n", ret);
    ret = flash_erase(flash_dev, page_base, FLASH_PAGE_SIZE);
    if (ret) printk("flash_erase failed with code: %d\n", ret);

    ret = flash_write_protection_set(flash_dev, false);
    if (ret) printk("flash_write_protection_set (2) failed with code: %d\n", ret);
    ret = flash_write(flash_dev, page_base, flash_page_buffer, FLASH_PAGE_SIZE);
    if (ret) printk("flash_write failed with code: %d\n", ret);

    // update housekeeping vars
    write_data += bytes_to_write;
    write_dest_addr += bytes_to_write;
    write_bytes_remaining -= bytes_to_write;

    // trigger next high level write if that's a thing
    if (write_bytes_remaining == 0) {
        next = continuation;
        continuation = NULL;
    }
}

void lu_write_update(struct update_header *hdr) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("lu_write_update called for hdr at %p\n", hdr);
#endif

    lu_hdr = hdr;
    next = _lu_write_text;
}

void lu_write_step() {
    if (next) next();
}

void _lu_write_text() {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("writing .text\n");
#endif

    write_data = (u8_t *)lu_hdr + sizeof(struct update_header);
    write_dest_addr = lu_hdr->text_start;
    write_bytes_remaining = lu_hdr->text_size;

    next = _lu_write_single_page;
    continuation = _lu_write_rodata;
}

void _lu_write_rodata() {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("writing .rodata\n");
#endif

    write_data = (u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size;
    write_dest_addr = lu_hdr->rodata_start;
    write_bytes_remaining = lu_hdr->rodata_size;

    next = _lu_write_single_page;
    if (lu_hdr->bss_start == 0) {
        continuation = _lu_write_main_ptr; 
    } else {
        continuation = _lu_write_bss_loc;
    }
}

// XXX not writing bss start because it's crashing and we don't need persistance
void _lu_write_bss_loc() {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("clearing .bss, NOT writing .bss location\n");
#endif
    memset((u8_t *)lu_hdr->bss_start, 0, lu_hdr->bss_size);

    next = _lu_write_finalize;

    /*
    write_data = (u8_t *) &(lu_hdr->bss_start);
    write_dest_addr = lu_hdr->bss_start_addr;
    write_bytes_remaining = 4;

    next = _lu_write_single_page;
    continuation = _lu_write_bss_size;
    */
}

// XXX not writing bss size because it's crashing and we don't need persistance
void _lu_write_bss_size() {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("writing .bss size\n");
#endif

    write_data = (u8_t *) &(lu_hdr->bss_size);
    write_dest_addr = lu_hdr->bss_size_addr;
    write_bytes_remaining = 4;

    next = _lu_write_single_page;
    continuation = _lu_write_main_ptr;
}

// XXX not writing main ptr because it's crashing and we don't need persistance
void _lu_write_main_ptr() {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("writing main ptr\n");
#endif

    write_data = (u8_t *) &(lu_hdr->main_ptr);
    write_dest_addr = lu_hdr->main_ptr_addr;
    write_bytes_remaining = 4;

    next = _lu_write_single_page;
    continuation = _lu_write_update_flag;
}

// XXX not writing update flag because it's crashing and we don't need persistance
void _lu_write_update_flag() {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("writing update flag\n");
#endif

    u8_t update_flag = 1;
    write_data = &update_flag;
    write_dest_addr = lu_hdr->update_flag_addr;
    write_bytes_remaining = 1;

    next = _lu_write_single_page;
    continuation = _lu_write_finalize;
}

void _lu_write_finalize() {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("update write complete!\n");
#endif

    if (!lu_hdr->write_only_flag) {
        update_write_completed = 1;
    }

    write_data = NULL;
    write_dest_addr = 0;
    write_bytes_remaining = 0;

    next = NULL;
    continuation = NULL;

    gpio_pin_set(update_gpio_dev, LIVE_UPDATE_WRITTEN_PIN, 1);
    for(volatile int i = 0; i < 1000; i++);
    gpio_pin_set(update_gpio_dev, LIVE_UPDATE_WRITTEN_PIN, 0);

    //update_counter = *DWT_CYCCNT;
}

/*
 * Copyright (c) 2016 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <toolchain.h>
#include <linker/sections.h>
#include <drivers/timer/system_timer.h>
#include <drivers/uart.h>
#include <wait_q.h>
#include <power/power.h>
#include <stdbool.h>

#include <tfm_flash_veneers.h>

#ifdef CONFIG_TICKLESS_IDLE_THRESH
#define IDLE_THRESH CONFIG_TICKLESS_IDLE_THRESH
#else
#define IDLE_THRESH 1
#endif

/* Fallback idle spin loop for SMP platforms without a working IPI */
#if (defined(CONFIG_SMP) && !defined(CONFIG_SCHED_IPI_SUPPORTED))
#define SMP_FALLBACK 1
#else
#define SMP_FALLBACK 0
#endif

#ifdef CONFIG_SYS_POWER_MANAGEMENT
/*
 * Used to allow _sys_suspend() implementation to control notification
 * of the event that caused exit from kernel idling after pm operations.
 */
unsigned char sys_pm_idle_exit_notify;


/* LCOV_EXCL_START
 * These are almost certainly overidden and in any event do nothing
 */
#if defined(CONFIG_SYS_POWER_SLEEP_STATES)
void __attribute__((weak)) _sys_resume(void)
{
}
#endif

#if defined(CONFIG_SYS_POWER_DEEP_SLEEP_STATES)
void __attribute__((weak)) _sys_resume_from_deep_sleep(void)
{
}
#endif
/* LCOV_EXCL_STOP */

#endif /* CONFIG_SYS_POWER_MANAGEMENT */

/**
 *
 * @brief Indicate that kernel is idling in tickless mode
 *
 * Sets the kernel data structure idle field to either a positive value or
 * K_FOREVER.
 *
 * @param ticks the number of ticks to idle
 *
 * @return N/A
 */
#if !SMP_FALLBACK
static void set_kernel_idle_time_in_ticks(s32_t ticks)
{
#ifdef CONFIG_SYS_POWER_MANAGEMENT
	_kernel.idle = ticks;
#endif
}

static void sys_power_save_idle(void)
{
	s32_t ticks = z_get_next_timeout_expiry();

	/* The documented behavior of CONFIG_TICKLESS_IDLE_THRESH is
	 * that the system should not enter a tickless idle for
	 * periods less than that.  This seems... silly, given that it
	 * saves no power and does not improve latency.  But it's an
	 * API we need to honor...
	 */
#ifdef CONFIG_SYS_CLOCK_EXISTS
	z_set_timeout_expiry((ticks < IDLE_THRESH) ? 1 : ticks, true);
#endif

	set_kernel_idle_time_in_ticks(ticks);
#if (defined(CONFIG_SYS_POWER_SLEEP_STATES) || \
	defined(CONFIG_SYS_POWER_DEEP_SLEEP_STATES))

	sys_pm_idle_exit_notify = 1U;

	/*
	 * Call the suspend hook function of the soc interface to allow
	 * entry into a low power state. The function returns
	 * SYS_POWER_STATE_ACTIVE if low power state was not entered, in which
	 * case, kernel does normal idle processing.
	 *
	 * This function is entered with interrupts disabled. If a low power
	 * state was entered, then the hook function should enable inerrupts
	 * before exiting. This is because the kernel does not do its own idle
	 * processing in those cases i.e. skips k_cpu_idle(). The kernel's
	 * idle processing re-enables interrupts which is essential for
	 * the kernel's scheduling logic.
	 */
	if (_sys_suspend(ticks) == SYS_POWER_STATE_ACTIVE) {
		sys_pm_idle_exit_notify = 0U;
		k_cpu_idle();
	}
#else
	k_cpu_idle();
#endif
}
#endif

void z_sys_power_save_idle_exit(s32_t ticks)
{
#if defined(CONFIG_SYS_POWER_SLEEP_STATES)
	/* Some CPU low power states require notification at the ISR
	 * to allow any operations that needs to be done before kernel
	 * switches task or processes nested interrupts. This can be
	 * disabled by calling _sys_pm_idle_exit_notification_disable().
	 * Alternatively it can be simply ignored if not required.
	 */
	if (sys_pm_idle_exit_notify) {
		_sys_resume();
	}
#endif

	z_clock_idle_exit();
}

#define CURRENT_VERSION 0x5
#define UPDATE_MAX_BYTES 4096

static struct device *uart1_dev;
static u32_t rx_buf[UPDATE_MAX_BYTES / sizeof(u32_t)];
static u32_t rx_bytes = 0;

struct update_header {
    u32_t version;
    u32_t main_ptr_addr;
    u32_t main_ptr;
    u32_t update_flag_addr;
    u32_t text_start;
    u32_t text_size;
    u32_t rodata_start;
    u32_t rodata_size;
    /*
    u32_t update_rodata_start_addr;
    u32_t update_rodata_rom_start_addr;
    u32_t update_rodata_size_addr;
    */
};

void apply_update_blocking(struct update_header *hdr) {
    printk("main_ptr@%x: %x -> %x\n", hdr->main_ptr_addr, *(u32_t *)hdr->main_ptr_addr, hdr->main_ptr);
    printk("update_flag@%x: %x -> %x\n", hdr->update_flag_addr, *(u32_t *)hdr->update_flag_addr, 1);

    u32_t *update_text = (u32_t *)((u8_t *)hdr + sizeof(struct update_header));
    u32_t *update_rodata = (u32_t *)((u8_t *)hdr + sizeof(struct update_header) + hdr->text_size);

    // write app .text
    while(tfm_flash_is_busy());
    /*
    printk("writing following text (%d bytes) to %x: ", hdr->text_size, hdr->text_start);
    for (int i = 0; i < (hdr->text_size / 4); i++) {
        printk("%x ", update_text[i]);
    }
    printk("\n");
    */
    int rc = tfm_flash_write(hdr->text_start, update_text, hdr->text_size);
    if (rc != 0) {
        printk("text flash write returned with code %d\n", rc);
    }

    // write app .rodata
    while(tfm_flash_is_busy());
    /*
    printk("writing following rodata (%d bytes) to %x: ", hdr->rodata_size, hdr->rodata_start);
    for (int i = 0; i < (hdr->rodata_size / 4); i++) {
        printk("%x ", update_rodata[i]);
    }
    printk("\n");
    */
    rc = tfm_flash_write(hdr->rodata_start, update_rodata, hdr->rodata_size);
    if (rc != 0) {
        printk("rodata flash write returned with code %d\n", rc);
    }

    // write updated main_ptr
    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->main_ptr_addr, &hdr->main_ptr, 4);
    if (rc != 0) {
        printk("main ptr flash write returned with code %d\n", rc);
    }

    // write update flag
    while(tfm_flash_is_busy());
    u32_t update_flag = 1;
    rc = tfm_flash_write(hdr->update_flag_addr, &update_flag, 1);
    if (rc != 0) {
        printk("update flag flash write returned with code %d\n", rc);
    }

    // write code relocation addresses
    /*
    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->update_text_start_addr, &hdr->text_start, 4);
    if (rc != 0) {
        printk("update_text_start flash write returned with code %d\n", rc);
    }

    while(tfm_flash_is_busy());
    u32_t rom_start = hdr->text_start + 0x20000000;
    rc = tfm_flash_write(hdr->update_text_rom_start_addr, &rom_start, 4);
    if (rc != 0) {
        printk("update_text_rom_start flash write returned with code %d\n", rc);
    }

    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->update_text_size_addr, &hdr->text_size, 4);
    if (rc != 0) {
        printk("update_text_size flash write returned with code %d\n", rc);
    }

    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->update_rodata_start_addr, &hdr->rodata_start, 4);
    if (rc != 0) {
        printk("update_rodata_start flash write returned with code %d\n", rc);
    }

    while(tfm_flash_is_busy());
    rom_start = hdr->rodata_start + 0x20000000;
    rc = tfm_flash_write(hdr->update_rodata_rom_start_addr, &rom_start, 4);
    if (rc != 0) {
        printk("update_rodata_rom_start flash write returned with code %d\n", rc);
    }

    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->update_rodata_size_addr, &hdr->rodata_size, 4);
    if (rc != 0) {
        printk("update_rodata_size flash write returned with code %d\n", rc);
    }
    */

    // read main ptr for a sanity check
    while(tfm_flash_is_busy());
    u32_t buf;
    rc = tfm_flash_read(hdr->main_ptr_addr, &buf, 4);
    if (rc != 0) {
        printk("flash read returned with code %d\n", rc);
    }
    printk("*main_ptr_addr(%x) = %x\n", hdr->main_ptr_addr, buf);

    printk("\n");
    while(tfm_flash_is_busy());
    rc = tfm_flash_read(0xe0000, &buf, 4);
    if (rc != 0) {
        printk("flash read returned with code %d\n", rc);
    }
    printk("*0xe0000 = %x\n", buf);

    while(tfm_flash_is_busy());
    rc = tfm_flash_read(0xffe00, &buf, 4);
    if (rc != 0) {
        printk("flash read returned with code %d\n", rc);
    }
    printk("*0xffe00 = %x\n", buf);
}

void update_uart_rx_cb(struct device *x) {
    if (uart_irq_rx_ready(x)) {
        while (true) {
            //printk("already read %d bytes, reading at most %d bytes to %x\n", rx_bytes, UPDATE_MAX_BYTES - rx_bytes, ((unsigned char *)rx_buf) + rx_bytes);
            int len = uart_fifo_read(x, ((unsigned char *)rx_buf) + rx_bytes, UPDATE_MAX_BYTES - rx_bytes);
            if (len == 0) break;
            rx_bytes += len;
            struct update_header *hdr = (struct update_header *)((void *)rx_buf);
            //printk("    read %d additional bytes, %d total, waiting for %d bytes total\n", len, rx_bytes, sizeof(struct update_header) + hdr->text_size + hdr->rodata_size);
        }

        if (rx_bytes >= sizeof(struct update_header)) {
            struct update_header *hdr = (struct update_header *)((void *)rx_buf);
            if (hdr->version != CURRENT_VERSION) {
                printk("expected version %d, got version %d\n", CURRENT_VERSION, hdr->version);
            } else if (hdr->version == CURRENT_VERSION && rx_bytes == sizeof(struct update_header) + hdr->text_size + hdr->rodata_size) {
                printk("received: hdr->text_size=%d, hdr->rodata_size=%d, rx_bytes total=%d\n", hdr->text_size, hdr->rodata_size, rx_bytes);
                apply_update_blocking(hdr);
            }
        }
    }
}

void update_uart_init() {
    uart1_dev = device_get_binding("UART_1");

    uart_irq_callback_set(uart1_dev, update_uart_rx_cb);
    uart_irq_rx_enable(uart1_dev);

    // TODO: probably only init when necessary and de-init when not in use
    int rc = tfm_flash_init();
    if(rc != 0) {
        printk("flash init failed with code %d\n", rc);
    }
}

#if K_IDLE_PRIO < 0
#define IDLE_YIELD_IF_COOP() k_yield()
#else
#define IDLE_YIELD_IF_COOP() do { } while (false)
#endif

void idle(void *unused1, void *unused2, void *unused3)
{
	ARG_UNUSED(unused1);
	ARG_UNUSED(unused2);
	ARG_UNUSED(unused3);

#ifdef CONFIG_BOOT_TIME_MEASUREMENT
	/* record timestamp when idling begins */

	extern u32_t z_timestamp_idle;

	z_timestamp_idle = k_cycle_get_32();
#endif

    update_uart_init();

	while (true) {
#if SMP_FALLBACK
		k_busy_wait(100);
		k_yield();
#else
		(void)arch_irq_lock();
		sys_power_save_idle();
		IDLE_YIELD_IF_COOP();
#endif
	}
}

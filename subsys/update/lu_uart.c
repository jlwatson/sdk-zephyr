/*
 * Live update serial protocol
 */

#include <update/live_update.h>

static struct device *uart1_dev;
static u32_t rx_buf[LIVE_UPDATE_MAX_BYTES / sizeof(u32_t)];
static u32_t rx_bytes = 0;

extern void lu_apply_blocking(struct update_header *);
void lu_uart_rx_cb (struct device *);

void lu_uart_init(void) {
    uart1_dev = device_get_binding("UART_1");

    uart_irq_callback_set(uart1_dev, lu_uart_rx_cb);
    uart_irq_rx_enable(uart1_dev);

    int rc = tfm_flash_init();
    if (rc != 0) {
        printk("lu_init: flash init failed with code %d\n", rc);
    }
}

void lu_uart_rx_cb (struct device *x) {
    if (uart_irq_rx_ready(x)) {
        while (true) {

//#ifdef CONFIG_LIVE_UPDATE_DEBUG
            //printk("already read %d bytes, reading at most %d bytes to %x\n", rx_bytes, LIVE_UPDATE_MAX_BYTES - rx_bytes, ((unsigned char *)rx_buf) + rx_bytes);
//#endif // CONFIG_LIVE_UPDATE_DEBUG
//
            int len = uart_fifo_read(x, ((unsigned char *)rx_buf) + rx_bytes, LIVE_UPDATE_MAX_BYTES - rx_bytes);
            if (len == 0) break;
            rx_bytes += len;

//#ifdef CONFIG_LIVE_UPDATE_DEBUG
            // struct update_header *hdr = (struct update_header *)((void *)rx_buf);
            //printk("    read %d additional bytes, %d total, waiting for %d bytes total\n", len, rx_bytes, sizeof(struct update_header) + hdr->text_size + hdr->rodata_size);
//#endif // CONFIG_LIVE_UPDATE_DEBUG
        }

        if (rx_bytes >= sizeof(struct update_header)) {
            struct update_header *hdr = (struct update_header *)((void *)rx_buf);
            if (hdr->version != LIVE_UPDATE_CURRENT_VERSION) {
                printk("lu_uart_rx_cb: expected version %d, got version %d\n", LIVE_UPDATE_CURRENT_VERSION, hdr->version);
            } else if (hdr->version == LIVE_UPDATE_CURRENT_VERSION && rx_bytes == sizeof(struct update_header) + hdr->text_size + hdr->rodata_size) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
                printk("lu_uart_rx_cb: received: hdr->text_size=%d, hdr->rodata_size=%d, rx_bytes total=%d\n", hdr->text_size, hdr->rodata_size, rx_bytes);
#endif // CONFIG_LIVE_UPDATE_DEBUG
                lu_apply_blocking(hdr);
            }
        }
    }
}


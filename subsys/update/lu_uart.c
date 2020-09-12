/*
 * Live update serial protocol
 */

#include <update/live_update.h>
#include <string.h>

// Prototypes

extern void lu_write_update(struct update_header *);
extern void lu_write_step();
void lu_uart_rx_cb (struct device *);

// UART USB interface (at least on the Musca)
static struct device *uart_dev;

// Full update payload is placed here before being applied. The (not
// fundamental) assumption is that application will fit, could eventually write
// through to flash
static u32_t rx_buf[LIVE_UPDATE_MAX_BYTES / sizeof(u32_t)];
static u32_t rx_bytes = 0;

/*
 * Initialize UART receive and start the secure-side flash driver to eventually write the update with.
 */
void lu_uart_init(void) {
    uart_dev = device_get_binding("UART_1");

    //char *str = "hello? this is UART 1\n";
    //uart_fifo_fill(uart_dev, str, strlen(str));
    printk("uart dev: %p\n", uart_dev);

    uart_irq_callback_set(uart_dev, lu_uart_rx_cb);
    uart_irq_rx_enable(uart_dev);
}

/*
 * Reset UART update state. Normally called at the end of an update application
 * to prep the OS for a future update. Assumption that updates don't happen
 * back-to-back-to-back by just resetting received bytes counter.
 */
void lu_uart_reset(void) {
    rx_bytes = 0;
}

/* * Receive callback. Sets a flag to trigger short reads in the idle loop.
 */
void lu_uart_rx_cb (struct device *x) {

    if (x != uart_dev) return;

    uart_irq_update(uart_dev);

    while(uart_irq_rx_ready(uart_dev)) {
        u32_t num_bytes_to_read = LIVE_UPDATE_MAX_BYTES - rx_bytes;

        int len = uart_fifo_read(uart_dev, ((unsigned char *)rx_buf) + rx_bytes, num_bytes_to_read);
        rx_bytes += len;
    }

    if (rx_bytes >= sizeof(struct update_header)) {
        struct update_header *hdr = (struct update_header *)((void *)rx_buf);

        if (hdr->version != LIVE_UPDATE_CURRENT_VERSION) {
            printk("lu_uart_rx_cb: expected protocol version %d, got version %d instead\n", LIVE_UPDATE_CURRENT_VERSION, hdr->version);
        } else {
            u32_t expected_payload_size = sizeof(struct update_header) + hdr->payload_size;
            if (rx_bytes == expected_payload_size) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
                //printk("Received complete header, starting update write: hdr->payload_size=%d, rx_bytes total=%d, rx_buf at %p\n",
                //        hdr->payload_size, rx_bytes, (unsigned char *)rx_buf);
#endif
                lu_write_update(hdr);
            }
        }
    }
}


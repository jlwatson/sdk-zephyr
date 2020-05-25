/*
 * Live update serial protocol
 */

#include <update/live_update.h>

// Prototypes

extern void lu_write_update(struct update_header *);
extern void lu_write_update_step();
void lu_uart_rx_cb (struct device *);

// UART USB interface (at least on the Musca)
static struct device *uart_dev;

// Full update payload is placed here before being applied. The (not
// fundamental) assumption is that application will fit, could eventually write
// through to flash
static u32_t rx_buf[LIVE_UPDATE_MAX_BYTES / sizeof(u32_t)];
static u32_t rx_bytes = 0;

// keep track of UART rx state so we can read over multiple idle intervals
static atomic_t rx_ready;

/*
 * Initialize UART receive and start the secure-side flash driver to eventually write the update with.
 */
void lu_uart_init(void) {
    uart_dev = device_get_binding("UART_1");

    uart_irq_callback_set(uart_dev, lu_uart_rx_cb);
    uart_irq_rx_enable(uart_dev);

    atomic_set(&rx_ready, 0);

    int rc = tfm_flash_init();
    if (rc != 0) {
        printk("lu_init: flash init failed with code %d\n", rc);
    }
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
    if (x == uart_dev && uart_irq_rx_ready(x)) {
        atomic_set(&rx_ready, 1);
    }
}

/*
 * Idle handler - reads payload from UART and triggers update when complete
 */
void lu_uart_idle_read () {
    lu_write_update_step();    

    // don't do anything if no bytes are pending
    if (!atomic_get(&rx_ready)) return;

    u32_t num_bytes_to_read = LIVE_UPDATE_MAX_BYTES - rx_bytes;

    // cap read size to something small
    if (num_bytes_to_read > LIVE_UPDATE_READ_SIZE) {
        num_bytes_to_read = LIVE_UPDATE_READ_SIZE;
    }

    int len = uart_fifo_read(uart_dev, ((unsigned char *)rx_buf) + rx_bytes, num_bytes_to_read);
    if (len == 0) {
        atomic_set(&rx_ready, 0);
        return;
    }
    rx_bytes += len;

    if (rx_bytes >= sizeof(struct update_header)) {
        struct update_header *hdr = (struct update_header *)((void *)rx_buf);

        if (hdr->version != LIVE_UPDATE_CURRENT_VERSION) {
            printk("lu_uart_idle_read: expected version %d, got version %d\n", LIVE_UPDATE_CURRENT_VERSION, hdr->version);
            return;
        }

        u32_t expected_payload_size = sizeof(struct update_header) + 
                              hdr->text_size +
                              hdr->rodata_size +
                              hdr->predicates_size +
                              hdr->transfers_size;
        if (rx_bytes == expected_payload_size) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("Received complete header, starting update write: hdr->text_size=%d, hdr->rodata_size=%d, hdr->predicates_size=%d, hdr->transfers_size=%d, rx_bytes total=%d, rx_buf at %p\n",
                    hdr->text_size, hdr->rodata_size, hdr->predicates_size, hdr->transfers_size, rx_bytes, (unsigned char *)rx_buf);
#endif // CONFIG_LIVE_UPDATE_DEBUG
            lu_write_update(hdr);
        }
    }
}


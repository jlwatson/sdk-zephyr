/*
 * Live update serial protocol
 */

#include <update/live_update.h>

// Prototypes

extern void lu_write_update(struct update_header *);
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

// TODO: add extern void app_test_rx_cb();

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
 * Receive callback. Sets a flag to trigger short reads in the idle loop.
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
    // don't do anything if no bytes are pending
    if (!atomic_get(&rx_ready)) return;

    u32_t num_bytes_to_read = LIVE_UPDATE_MAX_BYTES - rx_bytes;

    // cap read size to something small
    if (num_bytes_to_read > LIVE_UPDATE_READ_SIZE) {
        num_bytes_to_read = LIVE_UPDATE_READ_SIZE;
    }

//#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("already read %d bytes, reading at most %d bytes to %x\n", rx_bytes, num_bytes_to_read, ((unsigned char *)rx_buf) + rx_bytes);
//#endif // CONFIG_LIVE_UPDATE_DEBUG
    
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
                              hdr->transfer_triples_size;
        if (rx_bytes == expected_payload_size) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("lu_uart_rx_cb: hdr->text_size=%d, hdr->rodata_size=%d, hdr->transfer_triples_size=%d, rx_bytes total=%d\n",
                    hdr->text_size, hdr->rodata_size, hdr->transfer_triples_size, rx_bytes);
#endif // CONFIG_LIVE_UPDATE_DEBUG
            lu_write_update(hdr);
        }

    }
}


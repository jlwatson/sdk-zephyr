/*
 * Live update startup/system init
 */

#include <update/live_update.h>

extern void lu_uart_init(void);
extern void lu_write_init(void);

extern volatile u32_t *DWT_CONTROL;
extern volatile u32_t *DWT_CYCCNT;
extern volatile u32_t *DEMCR;
extern volatile u32_t *LAR;

struct device *update_gpio_dev;

void lu_main(void) {
    
    extern void main(void);
    static void (*volatile main_ptr)(void) __attribute__((section(".rodata")));

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("*update_flag_addr(%p): %x\n", &__update_flag, __update_flag);
    printk("*main_ptr_addr(%p): %p\n", &main_ptr, main_ptr);

    if (__update_flag) {
        printk("calling updated main_ptr @ %p (old main @ %p)\n", main_ptr, &main);
    } else {
        printk("calling main @ %p\n", &main);
    }
#endif

    update_gpio_dev = device_get_binding("GPIO_0");
    gpio_pin_configure(update_gpio_dev, LIVE_UPDATE_WRITTEN_PIN, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure(update_gpio_dev, LIVE_UPDATE_FINISHED_PIN, GPIO_OUTPUT_INACTIVE);
    
    lu_uart_init();
    lu_write_init();

    // set up cycle counter
    *DEMCR = *DEMCR | 0x01000000;
    *LAR = 0xC5ACCE55;
    *DWT_CYCCNT = 0;
    *DWT_CONTROL = *DWT_CONTROL | 1;

    if (__update_flag) {
        main_ptr();
    } else {
        main();
    }
}


/*
 * Live update startup/system init
 */

#include <update/live_update.h>

extern void lu_uart_init(void);

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
#endif // CONFIG_LIVE_UPDATE_DEBUG
    
    lu_uart_init();

    if (__update_flag) {
        main_ptr();
    } else {
        main();
    }
}


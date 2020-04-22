/*
 * Live update startup/system init
 */

#include <update/live_update.h>

void lu_apply_blocking(struct update_header *hdr) {
#ifdef CONFIG_CONFIG_LIVE_UPDATE_DEBUG
    printk("main_ptr@%x: %x -> %x\n", hdr->main_ptr_addr, *(u32_t *)hdr->main_ptr_addr, hdr->main_ptr);
    printk("update_flag@%x: %x -> %x\n", hdr->update_flag_addr, *(u32_t *)hdr->update_flag_addr, 1);
#endif // CONFIG_LIVE_UPDATE_DEBUG

    u32_t *update_text = (u32_t *)((u8_t *)hdr + sizeof(struct update_header));
    u32_t *update_rodata = (u32_t *)((u8_t *)hdr + sizeof(struct update_header) + hdr->text_size);

    // write app .text
    while(tfm_flash_is_busy());
    /*
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("writing following text (%d bytes) to %x: ", hdr->text_size, hdr->text_start);
    for (int i = 0; i < (hdr->text_size / 4); i++) {
        printk("%x ", update_text[i]);
    }
    printk("\n");
#endif // CONFIG_LIVE_UPDATE_DEBUG
    */
    int rc = tfm_flash_write(hdr->text_start, update_text, hdr->text_size);
    if (rc != 0) {
        printk("lu_apply_blocking: text flash write returned with code %d\n", rc);
    }

    // write app .rodata
    while(tfm_flash_is_busy());
    /*
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("writing following rodata (%d bytes) to %x: ", hdr->rodata_size, hdr->rodata_start);
    for (int i = 0; i < (hdr->rodata_size / 4); i++) {
        printk("%x ", update_rodata[i]);
    }
    printk("\n");
#endif // CONFIG_LIVE_UPDATE_DEBUG
    */
    rc = tfm_flash_write(hdr->rodata_start, update_rodata, hdr->rodata_size);
    if (rc != 0) {
        printk("lu_apply_blocking: rodata flash write returned with code %d\n", rc);
    }

    // write bss values
    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->bss_start_addr, &hdr->bss_start, 4);
    if (rc != 0) {
        printk("lu_apply_blocking: bss start flash write returned with code %d\n", rc);
    }

    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->bss_size_addr, &hdr->bss_size, 4);
    if (rc != 0) {
        printk("lu_apply_blocking: bss size flash write returned with code %d\n", rc);
    }

    // write updated main_ptr
    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->main_ptr_addr, &hdr->main_ptr, 4);
    if (rc != 0) {
        printk("lu_apply_blocking: main ptr flash write returned with code %d\n", rc);
    }

    // write update flag
    while(tfm_flash_is_busy());
    u32_t update_flag = 1;
    rc = tfm_flash_write(hdr->update_flag_addr, &update_flag, 1);
    if (rc != 0) {
        printk("lu_apply_blocking: update flag flash write returned with code %d\n", rc);
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("-- sanity check --\n");
    u32_t buf;

    while(tfm_flash_is_busy());
    rc = tfm_flash_read(hdr->update_flag_addr, &buf, 4);
    if (rc != 0) {
        printk("flash read returned with code %d\n", rc);
    }
    printk("*update_flag_addr(%x) = %x\n", hdr->update_flag_addr, buf);

    while(tfm_flash_is_busy());
    rc = tfm_flash_read(hdr->main_ptr_addr, &buf, 4);
    if (rc != 0) {
        printk("flash read returned with code %d\n", rc);
    }
    printk("*main_ptr_addr(%x) = %x\n", hdr->main_ptr_addr, buf);
#endif // CONFIG_LIVE_UPDATE_DEBUG

      // set update flag in RAM
      //extern volatile u32_t __update_flag;
      //__update_flag = 1;    
}


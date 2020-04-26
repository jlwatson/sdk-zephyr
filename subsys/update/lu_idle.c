/*
 * Live update startup/system init
 */

#include <string.h>
#include <timeout_q.h>
#include <update/live_update.h>

static u8_t update_ready;

extern u32_t *state_transfer_triples;

inline bool lu_trigger_on_timer(void) {
#ifdef CONFIG_LIVE_UPDATE_FUTURE
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    if (update_ready && z_get_next_timeout_expiry() == INT_MAX) {
        printk("live_update: update triggered on timer\n");
    }
#endif // CONFIG_LIVE_UPDATE_DEBUG
    return update_ready &&
           z_get_next_timeout_expiry() == INT_MAX;  // don't bother if there are other timers active or no update
#else
    return false;
#endif // CONFIG_LIVE_UPDATE_FUTURE
}

void lu_state_transfer(void) {
#ifdef CONFIG_LIVE_UPDATE_FUTURE
    u32_t *triple = state_transfer_triples;
    printk("triple[0] = %x\n", triple[0]);
    while (triple[0]) {
        u32_t *from_addr = (u32_t *) triple[0];
        u32_t *to_addr = (u32_t *) triple[1];
        u32_t size = triple[2];

        printk("\t%p -> %p (%d bytes)\n", from_addr, to_addr, size);
        memcpy(to_addr, from_addr, size);

        // XXX very janky
        if (size == 44) { // struct k_timer
            struct k_timer *t = (struct k_timer *) to_addr;
            switch ((u32_t) t->expiry_fn) {
                case 0xffe19: // aei expire
                    t->expiry_fn = (k_timer_expiry_t) 0xe0019;
                    break;
                case 0xe0019: // aei expire
                    t->expiry_fn = (k_timer_expiry_t) 0xffe19;
                    break;

                case 0xffe75: // avi expire
                    t->expiry_fn = (k_timer_expiry_t) 0xe0075;
                    break; 
                case 0xe0075: // avi expire
                    t->expiry_fn = (k_timer_expiry_t) 0xffe75;
                    break; 

                case 0x100021: // uri expire
                    t->expiry_fn = (k_timer_expiry_t) 0xe0221;
                    break;
                case 0xe0221: // uri expire
                    t->expiry_fn = (k_timer_expiry_t) 0x100021;
                    break;

                case 0x100065: // vrp expire
                    t->expiry_fn = (k_timer_expiry_t) 0xe0265;
                    break;
                case 0xe0265: // vrp expire
                    t->expiry_fn = (k_timer_expiry_t) 0x100065;
                    break;
            }

            switch ((u32_t) t->stop_fn) {

                case 0xffe39: // aei_stop
                    t->stop_fn = (k_timer_stop_t) 0xe0039;
                    break;
                case 0xe0039: // aei_stop
                    t->stop_fn = (k_timer_stop_t) 0xffe39;
                    break;

                case 0xffec1: // lri stop
                    t->stop_fn = (k_timer_stop_t) 0xe00c1;
                    break;
                case 0xe00c1: // lri stop
                    t->stop_fn = (k_timer_stop_t) 0xffec1;
                    break;
            }
        }

        triple += 3;
    } 
#endif // CONFIG_LIVE_UPDATE_FUTURE
}

void lu_state_transfer_timer(struct k_timer **t) {
#ifdef CONFIG_LIVE_UPDATE_FUTURE
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("transferring state...\n");
#endif // CONFIG_LIVE_UPDATE_DEBUG
    lu_state_transfer();

    // modify what timer we're interested in to new
    //printk("rewiring timer... &t: %p t: %p *t: %p \n", &t, t, *t);

    u32_t *triple = state_transfer_triples;
    while (triple[0]) {
        u32_t *from_addr = (u32_t *) triple[0];
        u32_t *to_addr = (u32_t *) triple[1];

        if ((struct k_timer *) from_addr == *t) {
            //printk("rewire *t: %p -> %p\n", *t, (struct k_timer *) to_addr);
            *t = (struct k_timer *) to_addr;
        }

        triple += 3;
    }

    update_ready = 0;
#endif // CONFIG_LIVE_UPDATE_FUTURE
}

void lu_write_update(struct update_header *hdr) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("main_ptr@%x: %x -> %x\n", hdr->main_ptr_addr, *(u32_t *)hdr->main_ptr_addr, hdr->main_ptr);
    printk("update_flag@%x: %x -> %x\n", hdr->update_flag_addr, *(u32_t *)hdr->update_flag_addr, 1);
#endif // CONFIG_LIVE_UPDATE_DEBUG

    u32_t *update_text = (u32_t *)((u8_t *)hdr + sizeof(struct update_header));
    u32_t *update_rodata = (u32_t *)((u8_t *)hdr + sizeof(struct update_header) + hdr->text_size);
    state_transfer_triples = (u32_t *)((u8_t *)hdr + sizeof(struct update_header) + hdr->text_size + hdr->rodata_size);

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
        printk("lu_write_update: text flash write returned with code %d\n", rc);
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
        printk("lu_write_update: rodata flash write returned with code %d\n", rc);
    }

    // write bss values
    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->bss_start_addr, &hdr->bss_start, 4);
    if (rc != 0) {
        printk("lu_write_update: bss start flash write returned with code %d\n", rc);
    }

    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->bss_size_addr, &hdr->bss_size, 4);
    if (rc != 0) {
        printk("lu_write_update: bss size flash write returned with code %d\n", rc);
    }

    // write updated main_ptr
    while(tfm_flash_is_busy());
    rc = tfm_flash_write(hdr->main_ptr_addr, &hdr->main_ptr, 4);
    if (rc != 0) {
        printk("lu_write_update: main ptr flash write returned with code %d\n", rc);
    }

    // write update flag
    while(tfm_flash_is_busy());
    u32_t update_flag = 1;
    rc = tfm_flash_write(hdr->update_flag_addr, &update_flag, 1);
    if (rc != 0) {
        printk("lu_write_update: update flag flash write returned with code %d\n", rc);
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
    update_ready = 1;    
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("lu write done\n");
#endif // CONFIG_LIVE_UPDATE_DEBUG
}


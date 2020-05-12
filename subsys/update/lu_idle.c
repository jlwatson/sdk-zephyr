/*
 * Live update idle
 */

#include <string.h>
#include <timeout_q.h>
#include <wait_q.h>
#include <update/live_update.h>

static u8_t update_ready;
static struct update_header *lu_hdr;

inline bool lu_trigger_on_timer(void) {
#ifdef CONFIG_LIVE_UPDATE_FUTURE
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("timer trigger: ready? %d next expiry %x\n", update_ready, z_get_next_timeout_expiry());
    if (update_ready && z_get_next_timeout_expiry() == INT_MAX) {
        //printk("live_update: update triggered on timer\n");
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
    //u32_t start_tick = *(u32_t *)0xE000E018;
    u32_t *triple = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size + lu_hdr->rodata_size);
    u32_t *init_triples = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size + lu_hdr->rodata_size + lu_hdr->transfer_triples_size);
    u32_t *end = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size + lu_hdr->rodata_size + lu_hdr->transfer_triples_size + lu_hdr->init_size);

    // copy data
    //printk("state transfer copy: triple[0] = %x\n", triple[0]);
    while (triple != init_triples) {
        u32_t *from_addr = (u32_t *) triple[0];
        u32_t *to_addr = (u32_t *) triple[1];
        u32_t size = triple[2];

        //printk("\t%p -> %p (%d bytes)\n", from_addr, to_addr, size);
        memcpy(to_addr, from_addr, size);

        triple += 3;
    } 
    //u32_t end_tick = *(u32_t *)0xE000E018;
    //printk("Data copy: %d ticks\n", start_tick - end_tick);

    //start_tick = *(u32_t *)0xE000E018;
    // set timer callbacks correctly
    while (triple != end) {
        struct k_timer *t = (struct k_timer *) triple[0];
        t->expiry_fn = (k_timer_expiry_t) triple[1];
        t->stop_fn = (k_timer_stop_t) triple[2];

        // for now, just empty timer waitq XXX remove this eventually
        z_waitq_init(&(t->wait_q));

        triple += 3;
    }
    //end_tick = *(u32_t *)0xE000E018;
    //printk("Timer setup: %d ticks\n", start_tick - end_tick);

#endif // CONFIG_LIVE_UPDATE_FUTURE
}

void lu_state_transfer_timer(struct k_timer **t) {
#ifdef CONFIG_LIVE_UPDATE_FUTURE
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("transferring state...\n");
#endif // CONFIG_LIVE_UPDATE_DEBUG
    lu_state_transfer();

    // modify what timer we're interested in to new
    //printk("rewiring timer... &t: %p t: %p *t: %p \n", &t, t, *t);
    
    //u32_t start_tick = *(u32_t *)0xE000E018;
    u32_t *triple = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size + lu_hdr->rodata_size);
    u32_t *init_triples = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size + lu_hdr->rodata_size + lu_hdr->transfer_triples_size);
    //u32_t *end = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size + lu_hdr->rodata_size + lu_hdr->transfer_triples_size + lu_hdr->init_size);

    //u32_t *updated_timer_location = 0x0;
    //printk("  looking for timer at *t = %p\n", *t);
    while (triple != init_triples) {
        u32_t *from_addr = (u32_t *) triple[0];
        u32_t *to_addr = (u32_t *) triple[1];

        //printk("    considering transfer item %p -> %p\n", from_addr, to_addr);

        if (from_addr == (u32_t *) *t) {
            //printk("    it's a match\n");
            //triple = init_triples;
            //updated_timer_location = to_addr;
            *t = (struct k_timer *) to_addr;
            break;
        }
        triple += 3;
    } 

    // rewire timer callbacks into new application
    /*
    printk("  looking for init item at timer address %p\n", *t);
    while (triple != end) {
        printk("    considering init item at %x\n", triple[0]);
        if (updated_timer_location == triple[0]) {
            printk("    it's a match, modifying expiry and stop functions\n");
            (*t)->expiry_fn = (k_timer_expiry_t) triple[1];
            (*t)->stop_fn = (k_timer_stop_t) triple[2];
            break;
        }
        triple += 3;
    }
    */
    //u32_t end_tick = *(u32_t *)0xE000E018;
    //printk("Timer rewire: %d ticks\n", start_tick - end_tick);

    //printk("DONE\n");
    update_ready = 0;
    lu_hdr = NULL;
    lu_uart_reset();
#endif // CONFIG_LIVE_UPDATE_FUTURE
}

/*
 * Assumes fully-received update payload
 */
void lu_write_update(struct update_header *hdr) {

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("main_ptr@%x: %x -> %x\n", hdr->main_ptr_addr, *(u32_t *)hdr->main_ptr_addr, hdr->main_ptr);
    //printk("update_flag@%x: %x -> %x\n", hdr->update_flag_addr, *(u32_t *)hdr->update_flag_addr, 1);
#endif // CONFIG_LIVE_UPDATE_DEBUG

    lu_hdr = hdr;

    u32_t *update_text = (u32_t *)((u8_t *)hdr + sizeof(struct update_header));
    u32_t *update_rodata = (u32_t *)((u8_t *)hdr + sizeof(struct update_header) + hdr->text_size);

    // write app .text
    memcpy((u32_t *) hdr->text_start, update_text, hdr->text_size);

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

    memcpy((u32_t *) hdr->rodata_start, update_rodata, hdr->rodata_size);

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

    // write bss location
    memset((u32_t *)hdr->bss_start, 0, hdr->bss_size);

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
    /*
    //printk("-- sanity check --\n");
    u32_t buf;

    while(tfm_flash_is_busy());
    rc = tfm_flash_read(hdr->update_flag_addr, &buf, 4);
    if (rc != 0) {
        printk("flash read returned with code %d\n", rc);
    }
    //printk("*update_flag_addr(%x) = %x\n", hdr->update_flag_addr, buf);

    while(tfm_flash_is_busy());
    rc = tfm_flash_read(hdr->main_ptr_addr, &buf, 4);
    if (rc != 0) {
        printk("flash read returned with code %d\n", rc);
    }
    //printk("*main_ptr_addr(%x) = %x\n", hdr->main_ptr_addr, buf);
    */
#endif // CONFIG_LIVE_UPDATE_DEBUG

    // set update flag in RAM
    update_ready = 1;    
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("lu write done\n");
#endif // CONFIG_LIVE_UPDATE_DEBUG
}


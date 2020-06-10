/*
 * Live update idle
 */

#include <kernel.h>
#include <string.h>
#include <timeout_q.h>
#include <wait_q.h>
#include <update/live_update.h>

static u8_t update_ready;
static struct update_header *lu_hdr;
static u32_t satisfied_predicate_index = 0;

bool _predicate_satisfied(struct predicate_header *p) {

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("    checking memory\n");
#endif

    struct predicate_memory_check *curr_m = (struct predicate_memory_check *) 
        ((u8_t *)p + sizeof(struct predicate_header));

    for (int i = 0; i< p->n_memory_checks; i++) {
        if (memcmp((void *)curr_m->check_addr, (void *) &curr_m->check_value, curr_m->check_size) != 0) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("return false\n");
#endif
            return false;
        }

        curr_m = (struct predicate_memory_check *) ((u8_t *)curr_m + curr_m->check_size + (2 * sizeof(u32_t)));
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("    checking active timers\n");
#endif

    struct predicate_active_timer *curr_a = (struct predicate_active_timer *)curr_m;

    for (int i = 0; i < p->n_active_timers; i++) {

        struct k_timer *a_t = (struct k_timer *) curr_a->base_addr;
        if (z_is_inactive_timeout(&a_t->timeout)) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("timer at %x inactive, continue\n", curr_a->base_addr);
#endif
            //return false;
            curr_a++;
            continue;
        }
        if ((curr_a->period != 0 || a_t->period != 0) &&
            curr_a->period != k_ticks_to_ms_floor64(a_t->period)) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("timer period %d ticks, %d ms, expected period %d, return false\n", curr_a->period, a_t->period, k_ticks_to_ms_floor64(a_t->period));
#endif
            return false;
        }

        curr_a++;
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("    checking enabled interrupts\n");
#endif

    if (tfm_gpio_interrupts_enabled() != p->gpio_interrupt_enabled) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("return false\n");
#endif
        return false;
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("    checking gpio out enabled\n");
#endif

    if (tfm_gpio_output_enabled() != p->gpio_out_enabled) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("return false\n");
#endif
        return false;
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("    checking gpio out set\n");
#endif
    
    if (tfm_gpio_dataout() != p->gpio_out_set) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("return false\n");
#endif
        return false;
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("    checking gpio interrupt callbacks\n");
#endif

    // take advantage of the fact we just iterated to the callback list
    struct predicate_gpio_interrupt_cb *curr_g = (struct predicate_gpio_interrupt_cb *) curr_a;

    for (int i = 0; i < p->n_gpio_interrupt_cbs; i++) {
        
        if (tfm_gpio_interrupt_callback_for_pin(curr_g->pin) != curr_g->cb_addr) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("return false\n");
#endif
            return false;
        }

        curr_g++;
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("return TRUE!\n");
#endif

    return true;
}

bool _predicate_satisfied_timer(struct predicate_header *p, struct k_timer *t) {

    /*
    printk("checking predicate header at %p:\n", p);
    printk("    size: %x\n", p->size);
    printk("    event_handler_addr: %x\n", p->event_handler_addr);
    printk("    n_memory_checks: %x\n", p->n_memory_checks);
    printk("    n_active_timers: %x\n", p->n_active_timers);
    printk("    n_gpio_interrupt_cbs: %x\n", p->n_gpio_interrupt_cbs);
    printk("    gpio_interrupt_enabled: %x\n", p->gpio_interrupt_enabled);
    printk("    gpio_out_enabled: %x\n", p->gpio_out_enabled);
    printk("    gpio_out_set: %x\n", p->gpio_out_set);
    */

    if (p->event_handler_addr != (u32_t) t->expiry_fn) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("failed timer predicate: %x not right callback %x\n", p->event_handler_addr, (u32_t) t->expiry_fn);
#endif
        return false;
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("event: %x\n", p->event_handler_addr);
#endif
   
    return _predicate_satisfied(p); 
}

bool _predicate_satisfied_gpio(struct predicate_header *p, u32_t cb_addr) {
    if (p->event_handler_addr != cb_addr) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("gpio event handler expected %x got %x\n", p->event_handler_addr, cb_addr);
#endif
        return false;
    }

    return _predicate_satisfied(p);
}

bool lu_trigger_on_timer(struct k_timer *t) {

    if (!update_ready) return false;    

    while (!t) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("trigger t is null\n");
#endif
    }

    while (!lu_hdr) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("lu_hdr is null\n");
#endif
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("timer check start\n");
#endif
    struct predicate_header *p = (struct predicate_header *) ((u8_t *)lu_hdr +
        sizeof(struct update_header) +
        lu_hdr->text_size +
        lu_hdr->rodata_size);

    for (int p_idx = 0; p_idx < lu_hdr->n_predicates; p_idx++, p = (struct predicate_header *)((u8_t *)p + p->size)) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("predicate #%d\n", p_idx);
#endif
        if (_predicate_satisfied_timer(p, t)) {
            satisfied_predicate_index = p_idx;
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("timer update triggered\n");
#endif
            return true;
        }
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("timer check end\n");
#endif
    return false;
}

bool lu_trigger_on_gpio(u32_t cb_addr) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    //printk("trigger on gpio?\n");
#endif
    if (!update_ready) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        //printk("  not ready\n");
#endif
        return false;    
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("gpio check start\n");
#endif
    struct predicate_header *p = (struct predicate_header *) ((u8_t *)lu_hdr +
        sizeof(struct update_header) +
        lu_hdr->text_size +
        lu_hdr->rodata_size);

    for (int p_idx = 0; p_idx < lu_hdr->n_predicates; p_idx++, p = (struct predicate_header *)((u8_t *)p + p->size)) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("predicate #%d\n", p_idx);
#endif
        if (_predicate_satisfied_gpio(p, cb_addr)) {
            satisfied_predicate_index = p_idx;
#ifdef CONFIG_LIVE_UPDATE_DEBUG
            printk("gpio update triggered\n");
#endif
            return true;
        }
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("gpio check end\n");
#endif
    return false;
}

void _apply_transfer(struct transfer_header *t) {

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("transferring memory...");
#endif
    struct transfer_memory *curr_m = (struct transfer_memory *)((u8_t *)t + sizeof(struct transfer_header));
    for (int i = 0; i < t->n_memory; i++, curr_m++) {
        memcpy((u32_t *)curr_m->dst_addr, (u32_t *)curr_m->src_addr, curr_m->size * sizeof(u32_t));
    }
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("done\n");
#endif

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("transferring init memory...");
#endif
    struct transfer_init_memory *curr_i = (struct transfer_init_memory *)curr_m;
    for (int i = 0; i < t->n_init_memory;
        i++, curr_i = (struct transfer_init_memory *)((u8_t *)curr_i + curr_i->size + (2 * sizeof(u32_t)))) {
        memcpy((u32_t *)curr_i->addr, &(curr_i->value), curr_i->size);
    }
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("done\n");
#endif

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("transferring timers...");
#endif
    struct transfer_timer *curr_t = (struct transfer_timer *)curr_i;
    for (int i = 0; i < t->n_timers; i++, curr_t++) {
        struct k_timer *k = (struct k_timer *)curr_t->base_addr;
        k->expiry_fn = (k_timer_expiry_t) curr_t->expire_cb;
        k->stop_fn = (k_timer_stop_t) curr_t->stop_cb;
    }
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("done\n");
#endif

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("transferring active timers...");
#endif
    struct transfer_active_timer *curr_at = (struct transfer_active_timer *)curr_t;
    for (int i = 0; i < t->n_active_timers; i++, curr_at++) {
        struct k_timer *k = (struct k_timer *)curr_at->base_addr;
        printk("starting active timer %d\n", i);
        z_impl_k_timer_start(k, curr_at->duration, curr_at->period); // start the timer so we can trigger stop events if needed
    }
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("done\n");
#endif

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("gpio configuration...");
#endif
    for (int i = 0; i < 32; i++) tfm_gpio_interrupt_disable(i);

    // Note: TODO does not support polarity/type configuration
    struct transfer_gpio_interrupt_cb *curr_cb = (struct transfer_gpio_interrupt_cb *)curr_at;

    gpio_int_config cfg = {
        .type = 1,
        .polarity = 1,
        .cb = NULL 
    };

    for (int i = 0; i < t->n_gpio_interrupt_cbs; i++, curr_cb++) {
        cfg.cb = (void (*)(void)) curr_cb->cb_addr;
        tfm_gpio_interrupt_enable(curr_cb->pin, &cfg);       
    }

    tfm_gpio_disable_all_outputs();
    tfm_gpio_enable_outputs(t->gpio_out_enabled);
    tfm_gpio_write_all(t->gpio_out_set);
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("done\n");
#endif
}

u32_t get_timer_base_for_expiry(struct transfer_header *transfer, u32_t expiry_addr) {

    struct transfer_memory *curr_m = (struct transfer_memory *)((u8_t *)transfer + sizeof(struct transfer_header));
    for (int i = 0; i < transfer->n_memory; i++, curr_m++);

    struct transfer_init_memory *curr_i = (struct transfer_init_memory *)curr_m;
    for (int i = 0; i < transfer->n_init_memory;
        i++, curr_i = (struct transfer_init_memory *)((u8_t *)curr_i + curr_i->size + (2 * sizeof(u32_t))));

    struct transfer_timer *curr_t = (struct transfer_timer *)curr_i;
    for (int i = 0; i < transfer->n_timers; i++, curr_t++) {
        struct k_timer *k = (struct k_timer *)curr_t->base_addr;
        if ((u32_t) k->expiry_fn == expiry_addr) {
            k->period = k_ms_to_ticks_ceil32(curr_t->period);
            return curr_t->base_addr;
        }
    }
    return 0;
}

void _apply_transfer_timer(struct transfer_header *transfer, struct k_timer **timer) {

    // Rewire timer expiry to the new version
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("rewiring expiry\n");
#endif
    (*timer)->expiry_fn = (k_timer_expiry_t) transfer->new_event_handler_addr;

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("applying generic transfer\n");
#endif
    _apply_transfer(transfer);

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("modifying timer\n");
#endif
    *timer = (struct k_timer *) get_timer_base_for_expiry(transfer, transfer->new_event_handler_addr);

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("apply transfer done\n");
#endif
}

void lu_update_at_timer(struct k_timer **timer) {
    
    if (!update_ready) return;

    while (!timer) {
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("update t is null\n");
#endif
    }

    struct transfer_header *transfer = (struct transfer_header *) ((u8_t *)lu_hdr +
        sizeof(struct update_header) +
        lu_hdr->text_size +
        lu_hdr->rodata_size +
        lu_hdr->predicates_size);

    for (int t_idx = 0; t_idx < satisfied_predicate_index;
            t_idx++, transfer = (struct transfer_header *)((u8_t *)transfer + transfer->size));

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("lu_update_at_timer:\n");
    printk("    size: %x\n", transfer->size);
    printk("    handler: %x\n", transfer->new_event_handler_addr);
    printk("    n_memory: %x\n", transfer->n_memory);
    printk("    n_init_memory: %x\n", transfer->n_init_memory);
    printk("    n_timers: %x\n", transfer->n_timers);
    printk("    n_active_timers: %x\n", transfer->n_active_timers);
    printk("    n_gpio_interrupt_cbs: %x\n", transfer->n_gpio_interrupt_cbs);
    printk("    gpio_interrupt_enabled: %x\n", transfer->gpio_interrupt_enabled);
    printk("    gpio_out_enabled: %x\n", transfer->gpio_out_enabled);
    printk("    gpio_out_set: %x\n", transfer->gpio_out_set);
#endif

    _apply_transfer_timer(transfer, timer); 

    satisfied_predicate_index = 0;

    update_ready = 0;
    lu_hdr = NULL;
    lu_uart_reset();
}

void lu_update_at_gpio() {
    
    if (!update_ready) return;

    struct transfer_header *transfer = (struct transfer_header *) ((u8_t *)lu_hdr +
        sizeof(struct update_header) +
        lu_hdr->text_size +
        lu_hdr->rodata_size +
        lu_hdr->predicates_size);

    for (int t_idx = 0; t_idx < satisfied_predicate_index;
            t_idx++, transfer = (struct transfer_header *)((u8_t *)transfer + transfer->size)) {
    }

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("lu_update_at_gpio:\n");
    printk("    size: %x\n", transfer->size);
    printk("    handler: %x\n", transfer->new_event_handler_addr);
    printk("    n_memory: %x\n", transfer->n_memory);
    printk("    n_init_memory: %x\n", transfer->n_init_memory);
    printk("    n_timers: %x\n", transfer->n_timers);
    printk("    n_active_timers: %x\n", transfer->n_active_timers);
    printk("    n_gpio_interrupt_cbs: %x\n", transfer->n_gpio_interrupt_cbs);
    printk("    gpio_interrupt_enabled: %x\n", transfer->gpio_interrupt_enabled);
    printk("    gpio_out_enabled: %x\n", transfer->gpio_out_enabled);
    printk("    gpio_out_set: %x\n", transfer->gpio_out_set);
#endif

    _apply_transfer(transfer); 

    // Existing interrupt callback already set by transfer to updated application, so nothing else needed

    satisfied_predicate_index = 0;

    update_ready = 0;
    lu_hdr = NULL;
    lu_uart_reset();
}

/* --------- flash writes in background --------- */

typedef void _next_cb(void);
_next_cb *next = NULL;

void _lu_write_update_flag() {
    //while (n_updates >= 2);

    u32_t update_flag = 1;
    if (tfm_flash_write(lu_hdr->update_flag_addr, &update_flag, 1) == 0) {
        next = NULL;
    }

    // set update flag in RAM
    update_ready = 1;    
#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("update flag done\n"); 
#endif
}

void _lu_write_main_ptr() {
    //while (n_updates >= 2);

    if (tfm_flash_write(lu_hdr->main_ptr_addr, &lu_hdr->main_ptr, 4) == 0) {
        next = _lu_write_update_flag;
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("write main done\n");
#endif
    }
}

void _lu_write_bss_size() {
    //while (n_updates >= 2);

#ifdef CONFIG_LIVE_UPDATE_DEBUG
    printk("lu_hdr: %p, lu_hdr->bss_size_addr: %x, &lu_hdr->bss_size: %x\n", lu_hdr, lu_hdr->bss_size_addr, &lu_hdr->bss_size);
#endif
    if (tfm_flash_write(lu_hdr->bss_size_addr, &lu_hdr->bss_size, 4) == 0) {
        next = _lu_write_main_ptr;
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("write bss size\n");
#endif
    }
}

void _lu_write_bss_loc() {
    //while (n_updates >= 2);

    memset((u32_t *)lu_hdr->bss_start, 0, lu_hdr->bss_size);

    if (tfm_flash_write(lu_hdr->bss_start_addr, &lu_hdr->bss_start, 4) == 0) {
        next = _lu_write_bss_size;
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("write bss loc\n");
#endif
    }
}

void _lu_write_rodata() {
    //while (n_updates >= 2);

    u32_t *update_rodata = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header) + lu_hdr->text_size);

    memcpy((u32_t *) lu_hdr->rodata_start, update_rodata, lu_hdr->rodata_size);

    if (tfm_flash_write(lu_hdr->rodata_start, update_rodata, lu_hdr->rodata_size) == 0) {
        if (lu_hdr->bss_start == 0) {
            next = _lu_write_main_ptr;
        } else {
            next = _lu_write_bss_loc;
        }
    }
}

void _lu_write_text() {
    //while (n_updates >= 2);

    u32_t *update_text = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header));

    // write app .text
    memcpy((u32_t *) lu_hdr->text_start, update_text, lu_hdr->text_size);

    if (tfm_flash_write(lu_hdr->text_start, update_text, lu_hdr->text_size) == 0) {
        next = _lu_write_rodata;
#ifdef CONFIG_LIVE_UPDATE_DEBUG
        printk("write text\n");
#endif
    }
}

void _lu_write_only_ram(struct update_header *hdr) {
    u32_t *update_text = (u32_t *)((u8_t *)hdr + sizeof(struct update_header));
    u32_t *update_rodata = (u32_t *)((u8_t *)hdr + sizeof(struct update_header) + hdr->text_size);

    memcpy((u32_t *) hdr->text_start, update_text, hdr->text_size);
    memcpy((u32_t *) hdr->rodata_start, update_rodata, hdr->rodata_size);
    memset((u32_t *) hdr->bss_start, 0, hdr->bss_size);

    update_ready = 1;    
}

void lu_write_update(struct update_header *hdr) {

    //printk("write update with hdr %p\n", hdr);
    lu_hdr = hdr;
    _lu_write_only_ram(hdr);

    /*
    next = _lu_write_text;
    if (!tfm_flash_write_step()) {
        next();
    }
    */
}

void lu_write_update_step() {
    /*
    if (!tfm_flash_write_step()) {
        if (next) next();
    }
    */
}


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

    struct predicate_memory_check *curr_m = (struct predicate_memory_check *) ((u8_t *)p) + sizeof(struct predicate_header);
    for (int i = 0; i < p->n_memory_checks; 
            i++, curr_m = (struct predicate_memory_check *) (((u8_t *)curr_m) + curr_m->check_size + (2 * sizeof(u32_t)))) {

        if (memcmp((u8_t *)curr_m->check_addr, curr_m->check_value, curr_m->check_size) != 0) {
            return false;
        }
    }

    // take advantage of the fact we just iterated to the active timer list
    struct predicate_active_timer *curr_a = (struct predicate_active_timer *)curr_m;
    for (int i = 0; i < p->n_active_timers; i++, curr_a++) {
        struct k_timer *a_t = (struct k_timer *) curr_a->base_addr;
        if (a_t->period != k_ticks_to_ms_floor64(a_t->period) ||
            z_is_inactive_timeout(&a_t->timeout)) {
            return false;
        }
    }

    if (tfm_gpio_interrupts_enabled() != p->gpio_interrupt_enabled) {
        return false;
    }

    if (tfm_gpio_output_enabled() != p->gpio_out_enabled) {
        return false;
    }

    if (tfm_gpio_dataout() != p->gpio_out_set) {
        return false;
    }

    // take advantage of the fact we just iterated to the callback list
    struct predicate_gpio_interrupt_cb *curr_g = (struct predicate_gpio_interrupt_cb *)curr_a;
    for (int i = 0; i < p->n_gpio_interrupt_cbs; i++, curr_g++) {
        if (tfm_gpio_interrupt_callback_for_pin(curr_g->pin) != curr_g->cb_addr) {
            return false;
        }
    }

    return true;
}

bool _predicate_satisfied_timer(struct predicate_header *p, struct k_timer *t) {
    if (p->event_handler_addr != (u32_t) t->expiry_fn) {
        return false;
    }
   
    return _predicate_satisfied(p); 
}

bool _predicate_satisfied_gpio(struct predicate_header *p, u32_t cb_addr) {
    if (p->event_handler_addr != cb_addr) {
        return false;
    }

    return _predicate_satisfied(p);
}

bool lu_trigger_on_timer(struct k_timer *t) {

    if (!update_ready) return false;    

    struct predicate_header *p = (struct predicate_header *) ((u8_t *)lu_hdr) + sizeof(struct update_header);
    for (int p_idx = 0; p_idx < lu_hdr->n_predicates; p_idx++, p = (struct predicate_header *)(((u8_t *)p) + p->size)) {
        if (_predicate_satisfied_timer(p, t)) {
            satisfied_predicate_index = p_idx;
            return true;
        }
    }

    return false;
}

bool lu_trigger_on_gpio(u32_t cb_addr) {
    if (!update_ready) return false;    

    struct predicate_header *p = (struct predicate_header *)(((u8_t *)lu_hdr) + sizeof(struct update_header));
    for (int p_idx = 0; p_idx < lu_hdr->n_predicates; p_idx++, p = (struct predicate_header *)(((u8_t *)p) + p->size)) {
        if (_predicate_satisfied_gpio(p, cb_addr)) {
            satisfied_predicate_index = p_idx;
            return true;
        }
    }

    return false;
}

void _apply_transfer(struct transfer_header *t) {

    struct transfer_memory *curr_m = (struct transfer_memory *)(((u8_t *)t) + sizeof(struct transfer_header));
    for (int i = 0; i < t->n_memory; i++, curr_m++) {
        memcpy((u32_t *)curr_m->dst_addr, (u32_t *)curr_m->src_addr, curr_m->size * sizeof(u32_t));
    }

    struct transfer_init_memory *curr_i = (struct transfer_init_memory *)curr_m;
    for (int i = 0; i < t->n_init_memory;
        i++, curr_i = (struct transfer_init_memory *)(((u8_t *)curr_i) + curr_i->size + (2 * sizeof(u32_t)))) {
        memcpy((u32_t *)curr_i->addr, curr_i->value, curr_i->size);
    }

    struct transfer_timer *curr_t = (struct transfer_timer *)curr_i;
    for (int i = 0; i < t->n_timers; i++, curr_t++) {
        struct k_timer *k = (struct k_timer *)curr_t->base_addr;
        k->expiry_fn = (k_timer_expiry_t) curr_t->expire_cb;
        k->stop_fn = (k_timer_stop_t) curr_t->stop_cb;
    }

    struct transfer_active_timer *curr_at = (struct transfer_active_timer *)curr_t;
    for (int i = 0; i < t->n_active_timers; i++, curr_at++) {
        struct k_timer *k = (struct k_timer *)curr_at->base_addr;
        z_impl_k_timer_start(k, curr_at->duration, curr_at->period); // start the timer so we can trigger stop events if needed
    }

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
}

void _apply_transfer_timer(struct transfer_header *transfer, struct k_timer *timer) {
    _apply_transfer(transfer);

    // Rewire timer expiry to the new version
    timer->expiry_fn = (k_timer_expiry_t) transfer->new_event_handler_addr;
}

void lu_update_at_timer(struct k_timer *timer) {
    
    if (!update_ready) return;

    struct transfer_header *transfer = (struct transfer_header *)(((u8_t *)lu_hdr) + sizeof(struct update_header) + lu_hdr->predicates_size);
    for (int t_idx = 0; t_idx < satisfied_predicate_index;
            t_idx++, transfer = (struct transfer_header *)(((u8_t *)transfer) + transfer->size));

    _apply_transfer_timer(transfer, timer); 

    satisfied_predicate_index = 0;

    update_ready = 0;
    lu_hdr = NULL;
    lu_uart_reset();
}

void lu_update_at_gpio() {
    
    if (!update_ready) return;

    struct transfer_header *transfer = (struct transfer_header *)(((u8_t *)lu_hdr) + sizeof(struct update_header) + lu_hdr->predicates_size);
    for (int t_idx = 0; t_idx < satisfied_predicate_index;
            t_idx++, transfer = (struct transfer_header *)(((u8_t *)transfer) + transfer->size));

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

    u32_t update_flag = 1;
    if (tfm_flash_write(lu_hdr->update_flag_addr, &update_flag, 1) == 0) {
        next = NULL;
    }

    // set update flag in RAM
    update_ready = 1;    
}

void _lu_write_main_ptr() {
    if (tfm_flash_write(lu_hdr->main_ptr_addr, &lu_hdr->main_ptr, 4) == 0) {
        next = _lu_write_update_flag;
    }
}

void _lu_write_bss_size() {
    if (tfm_flash_write(lu_hdr->bss_size_addr, &lu_hdr->bss_size, 4) == 0) {
        next = _lu_write_main_ptr;
    }
}

void _lu_write_bss_loc() {
    memset((u32_t *)lu_hdr->bss_start, 0, lu_hdr->bss_size);

    if (tfm_flash_write(lu_hdr->bss_start_addr, &lu_hdr->bss_start, 4) == 0) {
        next = _lu_write_bss_size;
    }
}

void _lu_write_rodata() {
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
    u32_t *update_text = (u32_t *)((u8_t *)lu_hdr + sizeof(struct update_header));

    // write app .text
    memcpy((u32_t *) lu_hdr->text_start, update_text, lu_hdr->text_size);

    if (tfm_flash_write(lu_hdr->text_start, update_text, lu_hdr->text_size) == 0) {
        next = _lu_write_rodata;
    }
}

void lu_write_update(struct update_header *hdr) {

    lu_hdr = hdr;
    next = _lu_write_text;
    if (!tfm_flash_write_step()) {
        next();
    }
}

void lu_write_update_step() {
    if (!tfm_flash_write_step()) {
        if (next) next();
    }
}


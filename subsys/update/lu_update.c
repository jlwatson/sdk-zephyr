/*
 * Live update - core update logic
 */

#include <kernel.h>
#include <device.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <hal/nrf_uarte.h>
#include <string.h>
#include <timeout_q.h>
#include <wait_q.h>
#include <update/live_update.h>

extern struct update_header *lu_hdr;
extern u8_t update_write_completed;
extern void gpio_nrfx_init_callback(struct gpio_callback *, gpio_callback_handler_t, gpio_port_pins_t);

volatile u32_t *DWT_CONTROL = (u32_t *) 0xE0001000;
volatile u32_t *DWT_CYCCNT = (u32_t *) 0xE0001004;
volatile u32_t *DEMCR = (u32_t *) 0xE000EDFC;
volatile u32_t *LAR = (u32_t *) 0xE0001FB0;

u32_t update_counter = 0;

bool _lu_update_predicate_satisfied(struct predicate_header *p, u32_t event_addr);
void _lu_state_transfer();
void _cancel_gpio_callbacks();
struct k_timer *_lu_get_timer_for_expiry();
struct gpio_callback *_lu_get_gpio_callback_for_event();

static struct predicate_header *matched_predicate = NULL;

bool lu_trigger_on_timer(struct k_timer *t) {
    update_counter = *DWT_CYCCNT;

    if (!update_write_completed || !t || !lu_hdr) {
        update_counter = 0;
        return false;    
    }

    //printk("checking predicates for timer %p\n", t);

    struct predicates_header *predicates = (struct predicates_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size);

    struct predicate_header *curr_predicate = (struct predicate_header *)(
            (u8_t *)predicates + sizeof(struct predicates_header));
//printk("lu_hdr: %p, predicates_header: %p, predicates->size: %d, curr_predicate: %p\n", lu_hdr, predicates, predicates->size, curr_predicate);

    // Search for a matching predicate; if found, return true
    int ctr = 0;
    while ((u32_t)curr_predicate < ((u32_t)predicates + predicates->size)) {
        //printk("current predicate: size = %d, event_handler_addr = %p, n_inactive_ops = %d, n_constraints = %d, n_state_init = %d, hw_transfer_size = %d\n", curr_predicate->size, curr_predicate->event_handler_addr, curr_predicate->n_inactive_ops, curr_predicate->n_constraints, curr_predicate->n_state_init, curr_predicate->hw_transfer_size);

        if (_lu_update_predicate_satisfied(curr_predicate, (u32_t)t->expiry_fn)) {
            matched_predicate = curr_predicate;

            u32_t end_counter = *DWT_CYCCNT;
            u32_t cycles_elapsed = 0;
            if (end_counter < update_counter) {
                cycles_elapsed = end_counter + (0xffffffff - update_counter); 
            } else {
                cycles_elapsed = end_counter - update_counter;
            }
            //printk("PREDICATE DURATION: %d cycles\n", cycles_elapsed);
            update_counter = 0;

            //printk("PREDICATE %d SATSIFIED\n", ctr);
            return true;            
        }
        //printk("  no dice\n");
        curr_predicate = (struct predicate_header *)((u8_t *)curr_predicate + curr_predicate->size);
        ctr++;
    }

    u32_t end_counter = *DWT_CYCCNT;
    u32_t cycles_elapsed = 0;
    if (end_counter < update_counter) {
    cycles_elapsed = end_counter + (0xffffffff - update_counter); 
    } else {
    cycles_elapsed = end_counter - update_counter;
    }
    //printk("PREDICATE DURATION: %d cycles\n", cycles_elapsed);
    update_counter = 0;

    //printk("returning false\n");
    return false;
}

bool lu_trigger_on_gpio(u32_t cb_addr) {

    update_counter = *DWT_CYCCNT;
    if (!update_write_completed || !lu_hdr) {
        update_counter = 0;
        return false;
    }

    //printk("checking predicates for gpio callback %x\n", cb_addr);

    struct predicates_header *predicates = (struct predicates_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size);

    struct predicate_header *curr_predicate = (struct predicate_header *)(
            (u8_t *)predicates + sizeof(struct predicates_header));

    int ctr = 0;
    // Search for a matching predicate; if found, return true
    while ((u32_t)curr_predicate < ((u32_t)predicates + predicates->size)) {
        if (_lu_update_predicate_satisfied(curr_predicate, cb_addr)) {
            matched_predicate = curr_predicate;

            u32_t end_counter = *DWT_CYCCNT;
            u32_t cycles_elapsed = 0;
            if (end_counter < update_counter) {
                cycles_elapsed = end_counter + (0xffffffff - update_counter); 
            } else {
                cycles_elapsed = end_counter - update_counter;
            }
            //printk("PREDICATE DURATION: %d cycles\n", cycles_elapsed);
            update_counter = 0;

            //printk("PREDICATE %d SATSIFIED\n", ctr);
            return true;            
        }
        //printk("  no dice\n");
        curr_predicate = (struct predicate_header *)((u8_t *)curr_predicate + curr_predicate->size);
        ctr++;
    }

    //printk("returning false\n");
    u32_t end_counter = *DWT_CYCCNT;
    u32_t cycles_elapsed = 0;
    if (end_counter < update_counter) {
        cycles_elapsed = end_counter + (0xffffffff - update_counter); 
    } else {
        cycles_elapsed = end_counter - update_counter;
    }
    //printk("PREDICATE DURATION: %d cycles\n", cycles_elapsed);
    update_counter = 0;

    return false;
}

bool lu_trigger_on_uart(u32_t cb_addr) {

    update_counter = *DWT_CYCCNT;
    if (!update_write_completed || !lu_hdr) {
        update_counter = 0;
        return false;
    }

    //printk("checking predicates for gpio callback %x\n", cb_addr);

    struct predicates_header *predicates = (struct predicates_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size);

    struct predicate_header *curr_predicate = (struct predicate_header *)(
            (u8_t *)predicates + sizeof(struct predicates_header));

    int ctr = 0;
    // Search for a matching predicate; if found, return true
    while ((u32_t)curr_predicate < ((u32_t)predicates + predicates->size)) {
        if (_lu_update_predicate_satisfied(curr_predicate, cb_addr)) {
            matched_predicate = curr_predicate;

            u32_t end_counter = *DWT_CYCCNT;
            u32_t cycles_elapsed = 0;
            if (end_counter < update_counter) {
                cycles_elapsed = end_counter + (0xffffffff - update_counter); 
            } else {
                cycles_elapsed = end_counter - update_counter;
            }
            //printk("PREDICATE DURATION: %d cycles\n", cycles_elapsed);
            update_counter = 0;

            //printk("PREDICATE %d SATSIFIED\n", ctr);
            return true;            
        }
        //printk("  no dice\n");
        curr_predicate = (struct predicate_header *)((u8_t *)curr_predicate + curr_predicate->size);
        ctr++;
    }

    //printk("returning false\n");
    u32_t end_counter = *DWT_CYCCNT;
    u32_t cycles_elapsed = 0;
    if (end_counter < update_counter) {
        cycles_elapsed = end_counter + (0xffffffff - update_counter); 
    } else {
        cycles_elapsed = end_counter - update_counter;
    }
    //printk("PREDICATE DURATION: %d cycles\n", cycles_elapsed);
    update_counter = 0;

    return false;
}

bool _lu_update_predicate_satisfied(struct predicate_header *p, u32_t event_addr) {

    //printk("---- %d constraints\n", p->n_constraints);

    // (1) check event handler address, masking out thumb bit just in case
    if (((u32_t)(p->event_handler_addr) & ~1) != (event_addr & ~1)) {
        return false;
    }
    
    // (2) check inactive and reset operations (aka timers)
    struct predicate_operation *curr_op = (struct predicate_operation *)(
            (u8_t *)p +
            sizeof(struct predicate_header));
    for (int i = 0; i < p->n_inactive_ops; i++, curr_op++) {
        // XXX assume it's a timer
        struct k_timer *inactive_t = (struct k_timer *)curr_op->op_ptr;
        if (!z_is_inactive_timeout(&inactive_t->timeout)) {
            return false;
        } 
    }
    for (int i = 0; i < p->n_reset_ops; i++, curr_op++);

    // (3) check constraints
    struct predicate_constraint *curr_constraint = (struct predicate_constraint *) curr_op;
    for (int i = 0; i < p->n_constraints; i++) {

        //printk("constraint %d: %d bytes at %x\n", i, curr_constraint->bytes, curr_constraint->symbol_addr);
        bool in_range = false;
        if (curr_constraint->bytes == 4) {
            u32_t val = *(u32_t *)curr_constraint->symbol_addr;
            //printk("  val = %d\n", val);

            struct predicate_constraint_range *curr_range = (struct predicate_constraint_range *)(
                    (u8_t *)curr_constraint + sizeof(struct predicate_constraint));
            for (int j = 0; j < curr_constraint->n_ranges; j++, curr_range++) {
                if (curr_range->lower <= val && val <= curr_range->upper) {
                    in_range = true;
                    break;
                } 
            }
        } else if (curr_constraint->bytes == 2) {
            u16_t val = *(u16_t *)curr_constraint->symbol_addr;
            //printk("  val = %d\n", val);

            struct predicate_constraint_range *curr_range = (struct predicate_constraint_range *)(
                    (u8_t *)curr_constraint + sizeof(struct predicate_constraint));
            for (int j = 0; j < curr_constraint->n_ranges; j++, curr_range++) {
                if (curr_range->lower <= val && val <= curr_range->upper) {
                    in_range = true;
                    break;
                } 
            }
        } else if (curr_constraint->bytes == 1) {
            u8_t val = *(u8_t *)curr_constraint->symbol_addr;
            //printk("  val = %d\n", val);

            struct predicate_constraint_range *curr_range = (struct predicate_constraint_range *)(
                    (u8_t *)curr_constraint + sizeof(struct predicate_constraint));
            for (int j = 0; j < curr_constraint->n_ranges; j++, curr_range++) {
                if (curr_range->lower <= val && val <= curr_range->upper) {
                    in_range = true;
                    break;
                } 
            }
        }
        
        // value does not meet constraint
        /* XXX only disabled for predicate testing XXX
        if (!in_range) {
            return false;
        }
        */

        curr_constraint = (struct predicate_constraint *)((u8_t *)curr_constraint + curr_constraint->size);
    }

    // never allow an update if we only want to see predicate evals
    if (lu_hdr->predicate_only_flag != 0) {
        return false;
    }
    return true;
}

void lu_update_at_timer(struct k_timer **timer) {

    if (!update_write_completed || !timer) return;

    update_counter = *DWT_CYCCNT;

    _cancel_gpio_callbacks();
    _lu_state_transfer();

    // Swap timer to the new application version. We can find it by going
    // through the hw inits and finding the one that writes the correct
    // callback into the new application.
    struct k_timer *new_timer = (struct k_timer *) _lu_get_timer_for_expiry();
    while (!new_timer) {
        printk("Error: couldn't resolve timer for expiry function in timer-triggered live update\n");
    }

    *timer = new_timer;

    // cleanup
    matched_predicate = NULL;
    update_write_completed = 0;
    lu_hdr = NULL;
    lu_uart_reset();

    u32_t end_counter = *DWT_CYCCNT;
    u32_t cycles_elapsed = 0;
    if (end_counter < update_counter) {
        cycles_elapsed = end_counter + (0xffffffff - update_counter); 
    } else {
        cycles_elapsed = end_counter - update_counter;
    }
    //printk("TRANSFER DURATION: %d cycles\n", cycles_elapsed);
    update_counter = 0;
}

void lu_update_at_gpio(struct gpio_callback **callback) {

    if (!update_write_completed || !callback) return;

    update_counter = *DWT_CYCCNT;

    _cancel_gpio_callbacks();
    _lu_state_transfer();

    // Swap callback to the new app version. We can find it by going through
    // the hw inits.
    struct gpio_callback *new_callback = (struct gpio_callback *) _lu_get_gpio_callback_for_event();
    while (!new_callback) {
        printk("Error couldn't resolve gpio callback for event in interrupt-triggered live update\n");
    }

    *callback = new_callback;
    
    // cleanup
    matched_predicate = NULL;
    update_write_completed = 0;
    lu_hdr = NULL;
    lu_uart_reset();

    u32_t end_counter = *DWT_CYCCNT;
    u32_t cycles_elapsed = 0;
    if (end_counter < update_counter) {
        cycles_elapsed = end_counter + (0xffffffff - update_counter); 
    } else {
        cycles_elapsed = end_counter - update_counter;
    }
    //printk("TRANSFER DURATION: %d cycles\n", cycles_elapsed);
    update_counter = 0;
}

void lu_update_at_uart(uart_irq_callback_user_data_t *callback) {

    if (!update_write_completed || !callback) return;

    update_counter = *DWT_CYCCNT;

    _cancel_gpio_callbacks();
    _lu_state_transfer();

    // Swap callback to the new app version. We can find it by going through
    // the hw inits.
    *callback = (uart_irq_callback_user_data_t) matched_predicate->updated_event_handler_addr;
    
    // cleanup
    matched_predicate = NULL;
    update_write_completed = 0;
    lu_hdr = NULL;
    lu_uart_reset();

    u32_t end_counter = *DWT_CYCCNT;
    u32_t cycles_elapsed = 0;
    if (end_counter < update_counter) {
        cycles_elapsed = end_counter + (0xffffffff - update_counter); 
    } else {
        cycles_elapsed = end_counter - update_counter;
    }
    //printk("TRANSFER DURATION: %d cycles\n", cycles_elapsed);
    update_counter = 0;
}

void _lu_state_transfer() {

    struct predicates_header *predicates = (struct predicates_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size);

    // (0) Cancel reset timers
    struct predicate_operation *curr_op = (struct predicate_operation *)(
            (u8_t *)matched_predicate +
            sizeof(struct predicate_header));
    for (int i = 0; i < matched_predicate->n_inactive_ops; i++, curr_op++);
    for (int i = 0; i < matched_predicate->n_reset_ops; i++, curr_op++) {
        struct k_timer *reset_t = (struct k_timer *)curr_op->op_ptr;
	    z_abort_timeout(&reset_t->timeout);
    }

    struct predicate_constraint *c = (struct predicate_constraint *) curr_op;
    for (int i = 0; i < matched_predicate->n_constraints; i++) {
        c = (struct predicate_constraint *)((u8_t *)c + c->size);
    }

    // (1) Apply predicate-specific state initialization
    struct predicate_state_transfer *state_transfer = (struct predicate_state_transfer *)c;
    for (int i = 0; i < matched_predicate->n_state_init; i++, state_transfer++) {
        *(u32_t *)((u8_t *)state_transfer->addr + state_transfer->offset) = state_transfer->val;
    }        

    // (2) Apply predicate-specific HW initialization
    struct predicate_hw_transfer *hw_transfer = (struct predicate_hw_transfer *)state_transfer;
    while ((u32_t)hw_transfer < (u32_t)state_transfer + matched_predicate->hw_transfer_size) {

        // XXX hacky just hard code the functions we know about
        u32_t fn_thumb = hw_transfer->fn_ptr | 1;

        struct device *port = device_get_binding("GPIO_0");
        const struct gpio_driver_api *api =
		    (const struct gpio_driver_api *)port->driver_api;

        if (fn_thumb == (u32_t) api->port_set_bits_raw) {
            api->port_set_bits_raw(port, *(&hw_transfer->args + 1));
        } else if (fn_thumb == (u32_t) api->port_clear_bits_raw) {
            api->port_clear_bits_raw(port, *(&hw_transfer->args + 1));
        }

        hw_transfer = (struct predicate_hw_transfer *)((u8_t *)hw_transfer + hw_transfer->size);
    }

    // (3) Apply state transfers
    struct transfers_header *transfers = (struct transfers_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size +
            predicates->size
            );

    struct transfer *curr_transfer = (struct transfer *)(
            (u8_t *)transfers + sizeof(struct transfers_header));
    for (int i = 0; i < (transfers->size - sizeof(struct transfers_header)) / sizeof(struct transfer); i++, curr_transfer++) {
        memcpy(curr_transfer->dest, curr_transfer->origin, curr_transfer->size);    
    }

    // (4) Apply hw initialization calls
    struct hw_init_header *hw_inits = (struct hw_init_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size +
            predicates->size +
            transfers->size
            );

    struct hw_init *curr_hw = (struct hw_init *)(
            (u8_t *)hw_inits + sizeof(struct hw_init_header));
    while ((u32_t)curr_hw < (u32_t)hw_inits + hw_inits->size) {
    
        // XXX hacky again just hard code the functions we care about
        volatile u32_t fn_thumb = curr_hw->fn_ptr | 1;

        struct device *port = device_get_binding("GPIO_0");
        const struct gpio_driver_api *api =
		    (const struct gpio_driver_api *)port->driver_api;
	    const struct gpio_driver_data *const data =
		    (const struct gpio_driver_data *)port->driver_data;

        if (fn_thumb == (u32_t) k_timer_init) {

            u32_t expiry_cb = *(&curr_hw->args + 1);
            if (expiry_cb) expiry_cb |= 1;

            u32_t stop_cb = *(&curr_hw->args + 2);
            if (stop_cb) stop_cb |= 1;

            struct k_timer *t = (struct k_timer *) curr_hw->args;
            k_timer_init(t, (k_timer_expiry_t) expiry_cb, (k_timer_stop_t) stop_cb);

        } else if (fn_thumb == (u32_t) api->pin_configure) {
            api->pin_configure(port,
                    *(&curr_hw->args + 1),
                    *(&curr_hw->args + 2));

        } else if (fn_thumb == (u32_t) z_impl_gpio_pin_interrupt_configure) {

            if ((( *(&curr_hw->args + 2) & GPIO_INT_LEVELS_LOGICAL) != 0) &&
                ((data->invert & (gpio_port_pins_t)BIT(*(&curr_hw->args + 1))) != 0)) {
                /* Invert signal bits */
                *(&curr_hw->args + 2) ^= (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1);
            }

            enum gpio_int_trig trig = (enum gpio_int_trig)(*(&curr_hw->args + 2) & (GPIO_INT_LOW_0 | GPIO_INT_HIGH_1));
            enum gpio_int_mode mode = (enum gpio_int_mode)(*(&curr_hw->args + 2) & (GPIO_INT_EDGE | GPIO_INT_DISABLE | GPIO_INT_ENABLE));

            api->pin_interrupt_configure(port, *(&curr_hw->args + 1), mode, trig);

        } else if (fn_thumb == (u32_t) gpio_nrfx_init_callback) {
            gpio_nrfx_init_callback((struct gpio_callback *)curr_hw->args,
                    (gpio_callback_handler_t) (*(&curr_hw->args + 1) | 1),
                    *(&curr_hw->args + 2));

        } else if (fn_thumb == (u32_t) api->manage_callback) {
            //printk("adding %p\n", (void *)*(&curr_hw->args + 1));
            api->manage_callback(port,
                    (struct gpio_callback *)*(&curr_hw->args + 1),
                    true);
        }

        curr_hw = (struct hw_init *)((u8_t *)curr_hw + curr_hw->size);
    }

    // (5) Apply mem initialization values
    struct mem_init_header *mem_inits = (struct mem_init_header*)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size +
            predicates->size +
            transfers->size +
            hw_inits->size
            );

    struct mem_init *curr_mem = (struct mem_init *)(
            (u8_t *)mem_inits + sizeof(struct mem_init_header));
    for (int i = 0; i < (mem_inits->size - sizeof(struct mem_init_header)) / sizeof(struct mem_init); i++, curr_mem++) {
        *(u32_t *)((u8_t *)curr_mem->addr + curr_mem->offset) = curr_mem->val;
    }
}

struct k_timer * _lu_get_timer_for_expiry() {
    struct predicates_header *predicates = (struct predicates_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size);

    struct transfers_header *transfers = (struct transfers_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size +
            predicates->size
            );

    struct hw_init_header *hw_inits = (struct hw_init_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size +
            predicates->size +
            transfers->size
            );

    struct hw_init *curr_hw = (struct hw_init *)(
            (u8_t *)hw_inits + sizeof(struct hw_init_header));

    while ((u32_t)curr_hw < (u32_t)hw_inits + hw_inits->size) {
    
        // XXX hacky again just hard code the k_timer_init we care about
        volatile u32_t fn_thumb = curr_hw->fn_ptr | 1;
        if (fn_thumb == (u32_t) k_timer_init) {

            u32_t expiry_cb = *(&curr_hw->args + 1);
            //if (expiry_cb) expiry_cb |= 1;
            if (expiry_cb == (u32_t) matched_predicate->updated_event_handler_addr) {
                return (struct k_timer *) curr_hw->args;
            }

            u32_t stop_cb = *(&curr_hw->args + 2);
            if (stop_cb == (u32_t) matched_predicate->updated_event_handler_addr) {
                return (struct k_timer *) curr_hw->args;
            }
        }
        curr_hw = (struct hw_init *)((u8_t *)curr_hw + curr_hw->size);
    }

    return NULL;
}

struct _lu_gpio_nrfx_data {
	/* gpio_driver_data needs to be first */
	struct gpio_driver_data common;
	sys_slist_t callbacks;

	/* Mask holding information about which pins have been configured to
	 * trigger interrupts using gpio_nrfx_config function.
	 */
	u32_t pin_int_en;

	/* Mask holding information about which pins have enabled callbacks
	 * using gpio_nrfx_enable_callback function.
	 */
	u32_t int_en;

	u32_t int_active_level;
	u32_t trig_edge;
	u32_t double_edge;
};

void _cancel_gpio_callbacks() {

    struct device *port = device_get_binding("GPIO_0");
    const struct gpio_driver_api *api =
		(const struct gpio_driver_api *)port->driver_api;
    struct _lu_gpio_nrfx_data *data = port->driver_data;
	sys_slist_t *list = &data->callbacks;

    struct gpio_callback *old_cb, *tmp;
    SYS_SLIST_FOR_EACH_CONTAINER_SAFE(list, old_cb, tmp, node) {
        api->manage_callback(port, old_cb, false);
    }
}

struct gpio_callback *_lu_get_gpio_callback_for_event() {
    struct predicates_header *predicates = (struct predicates_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size);

    struct transfers_header *transfers = (struct transfers_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size +
            predicates->size
            );

    struct hw_init_header *hw_inits = (struct hw_init_header *)(
            (u8_t *)lu_hdr +
            sizeof(struct update_header) +
            lu_hdr->text_size +
            lu_hdr->rodata_size +
            predicates->size +
            transfers->size
            );

    struct hw_init *curr_hw = (struct hw_init *)(
            (u8_t *)hw_inits + sizeof(struct hw_init_header));

    while ((u32_t)curr_hw < (u32_t)hw_inits + hw_inits->size) {
    
        // XXX hacky again just hard code the gpio_init_callback we care about
        volatile u32_t fn_thumb = curr_hw->fn_ptr | 1;
        if (fn_thumb == (u32_t) gpio_nrfx_init_callback) {

            u32_t cb = *(&curr_hw->args + 1);
            if (cb == (u32_t) matched_predicate->updated_event_handler_addr) {
                return (struct gpio_callback *) curr_hw->args;
            }
        }
        curr_hw = (struct hw_init *)((u8_t *)curr_hw + curr_hw->size);
    }

    return NULL;
}

#define DATA_SIZE 8
// DATA
u32_t glucose_data[] = { 0xfdfdfdfd, 0x00000003, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0x3ed9999a, 0x00000000, 0xfdfdfdfd, 0x00000003,
    0x00000000, 0x00002328, 0x00000000, 0x00000000, 0x3eb33333, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00003024, 0x00000000, 0x00000000,
    0x00000000, 0x42a00000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000302f,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00003150, 0x00000000, 0x00000000, 0x00000000, 0x42a00000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000327c, 0x00000000, 0x00000000,
    0x00000000, 0x429e0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000033a8,
    0x00000000, 0x00000000, 0x00000000, 0x429a0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000034d4, 0x00000000, 0x00000000, 0x00000000, 0x42a00000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x000034e3, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00003601,
    0x00000000, 0x00000000, 0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x00003653, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000372c, 0x00000000, 0x00000000,
    0x00000000, 0x42a60000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00003858,
    0x00000000, 0x00000000, 0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00003984, 0x00000000, 0x00000000, 0x00000000, 0x42a40000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000398f, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00003a58,
    0x00000000, 0x00000000, 0x00000000, 0x42ba0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00003ab2, 0x00000000, 0x00000000, 0x00000000, 0x42ac0000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00003abb, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00003bdc,
    0x00000000, 0x00000000, 0x00000000, 0x42a60000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00003d08, 0x00000000, 0x00000000, 0x00000000, 0x42a40000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00003e34, 0x00000000, 0x00000000,
    0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00003e3f,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00003f60, 0x00000000, 0x00000000, 0x00000000, 0x42a60000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00003f6c, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000408c,
    0x00000000, 0x00000000, 0x00000000, 0x42aa0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000041b8, 0x00000000, 0x00000000, 0x00000000, 0x42b00000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x000042e4, 0x00000000, 0x00000000,
    0x00000000, 0x42b40000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00004410,
    0x00000000, 0x00000000, 0x00000000, 0x42b40000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000453c, 0x00000000, 0x00000000, 0x00000000, 0x42b80000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00004668, 0x00000000, 0x00000000,
    0x00000000, 0x42c20000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00004794,
    0x00000000, 0x00000000, 0x00000000, 0x42d00000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x000047a2, 0x00000000, 0x41f00000, 0x3f99999a, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x000048c0, 0x00000000, 0x00000000,
    0x00000000, 0x42d80000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000049ec,
    0x00000000, 0x00000000, 0x00000000, 0x42dc0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00004b18, 0x00000000, 0x00000000, 0x00000000, 0x42dc0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00004c44, 0x00000000, 0x00000000,
    0x00000000, 0x42e00000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00004c51,
    0x00000000, 0x41f00000, 0x3f933333, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00004d70, 0x00000000, 0x00000000, 0x00000000, 0x42e20000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00004d7d, 0x00000000, 0x41f00000,
    0x3f800000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00004e9c,
    0x00000000, 0x00000000, 0x00000000, 0x42e80000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x00004ea9, 0x00000000, 0x41f00000, 0x3f533333, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00004fc8, 0x00000000, 0x00000000,
    0x00000000, 0x42f00000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00004fd5,
    0x00000000, 0x41f00000, 0x3f4ccccd, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000050f4, 0x00000000, 0x00000000, 0x00000000, 0x42f80000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00005102, 0x00000000, 0x41f00000,
    0x3f8ccccd, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00005220,
    0x00000000, 0x00000000, 0x00000000, 0x43010000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000522d, 0x00000000, 0x41f00000, 0x3f933333, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000534c, 0x00000000, 0x00000000,
    0x00000000, 0x43060000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000535a,
    0x00000000, 0x41f00000, 0x3f99999a, 0x00000000, 0xfdfdfdfd, 0x00000003,
    0x00000000, 0x00005460, 0x00000000, 0x00000000, 0x3ecccccd, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00005478, 0x00000000, 0x00000000,
    0x00000000, 0x430b0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000055a4,
    0x00000000, 0x00000000, 0x00000000, 0x430f0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000056d0, 0x00000000, 0x00000000, 0x00000000, 0x43120000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x000057c5, 0x00000000, 0x41f00000,
    0x3f8ccccd, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000057fc,
    0x00000000, 0x00000000, 0x00000000, 0x43130000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x000058f2, 0x00000000, 0x41f00000, 0x3f200000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00005928, 0x00000000, 0x00000000,
    0x00000000, 0x43130000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00005a1e,
    0x00000000, 0x41f00000, 0x3f066666, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00005a54, 0x00000000, 0x00000000, 0x00000000, 0x43120000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00005b4a, 0x00000000, 0x41f00000,
    0x3ee66666, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00005b80,
    0x00000000, 0x00000000, 0x00000000, 0x43110000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x00005c76, 0x00000000, 0x41f00000, 0x3e4ccccd, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00005cac, 0x00000000, 0x00000000,
    0x00000000, 0x430f0000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00005da1,
    0x00000000, 0x41f00000, 0x3e19999a, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00005dd8, 0x00000000, 0x00000000, 0x00000000, 0x430e0000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00005ece, 0x00000000, 0x41f00000,
    0x3e666666, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00005f04,
    0x00000000, 0x00000000, 0x00000000, 0x430d0000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x00005fec, 0x00000000, 0x41f00000, 0x3e19999a, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00006030, 0x00000000, 0x00000000,
    0x00000000, 0x43090000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00006126,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000615c, 0x00000000, 0x00000000, 0x00000000, 0x43050000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00006288, 0x00000000, 0x00000000,
    0x00000000, 0x43000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000063b4,
    0x00000000, 0x00000000, 0x00000000, 0x42f60000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000064e0, 0x00000000, 0x00000000, 0x00000000, 0x42ec0000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x000065d5, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000660c,
    0x00000000, 0x00000000, 0x00000000, 0x42e20000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00006738, 0x00000000, 0x00000000, 0x00000000, 0x42da0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00006864, 0x00000000, 0x00000000,
    0x00000000, 0x42d40000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00006990,
    0x00000000, 0x00000000, 0x00000000, 0x42cc0000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x00006a85, 0x00000000, 0x41f00000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00006abc, 0x00000000, 0x00000000,
    0x00000000, 0x42c60000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00006be8,
    0x00000000, 0x00000000, 0x00000000, 0x42c00000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00006d14, 0x00000000, 0x00000000, 0x00000000, 0x42b60000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00006e40, 0x00000000, 0x00000000,
    0x00000000, 0x42b00000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00006f35,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00006f6c, 0x00000000, 0x00000000, 0x00000000, 0x42ac0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00007098, 0x00000000, 0x00000000,
    0x00000000, 0x42a80000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000071c4,
    0x00000000, 0x00000000, 0x00000000, 0x42a60000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000072f0, 0x00000000, 0x00000000, 0x00000000, 0x42a40000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x000073e5, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000741c,
    0x00000000, 0x00000000, 0x00000000, 0x42a80000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00007548, 0x00000000, 0x00000000, 0x00000000, 0x42a60000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00007674, 0x00000000, 0x00000000,
    0x00000000, 0x42a60000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000077a0,
    0x00000000, 0x00000000, 0x00000000, 0x42a60000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000078cc, 0x00000000, 0x00000000, 0x00000000, 0x42a20000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x000079f8, 0x00000000, 0x00000000,
    0x00000000, 0x42a00000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00007b24,
    0x00000000, 0x00000000, 0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00007c50, 0x00000000, 0x00000000, 0x00000000, 0x42a40000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00007d7c, 0x00000000, 0x00000000,
    0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000003, 0x00000000, 0x00007e90,
    0x00000000, 0x00000000, 0x3eb33333, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00007ea8, 0x00000000, 0x00000000, 0x00000000, 0x42a20000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00007fd4, 0x00000000, 0x00000000,
    0x00000000, 0x429e0000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x000080c9,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000080ff, 0x00000000, 0x00000000, 0x00000000, 0x429c0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000822c, 0x00000000, 0x00000000,
    0x00000000, 0x429c0000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00008321,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00008357, 0x00000000, 0x00000000, 0x00000000, 0x429a0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00008484, 0x00000000, 0x00000000,
    0x00000000, 0x42980000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000085af,
    0x00000000, 0x00000000, 0x00000000, 0x42980000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000086db, 0x00000000, 0x00000000, 0x00000000, 0x429a0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00008808, 0x00000000, 0x00000000,
    0x00000000, 0x429c0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00008933,
    0x00000000, 0x00000000, 0x00000000, 0x429a0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00008a5f, 0x00000000, 0x00000000, 0x00000000, 0x42960000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00008b55, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00008b8c,
    0x00000000, 0x00000000, 0x00000000, 0x42980000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x00008c82, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00008cb7, 0x00000000, 0x00000000,
    0x00000000, 0x42960000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00008de3,
    0x00000000, 0x00000000, 0x00000000, 0x42980000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00008f0f, 0x00000000, 0x00000000, 0x00000000, 0x42980000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000903b, 0x00000000, 0x00000000,
    0x00000000, 0x42960000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00009131,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00009167, 0x00000000, 0x00000000, 0x00000000, 0x42940000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00009293, 0x00000000, 0x00000000,
    0x00000000, 0x42920000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x000093bf,
    0x00000000, 0x00000000, 0x00000000, 0x42920000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x000094eb, 0x00000000, 0x00000000, 0x00000000, 0x429a0000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x000095e1, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00009617,
    0x00000000, 0x00000000, 0x00000000, 0x429e0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00009743, 0x00000000, 0x00000000, 0x00000000, 0x42960000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000986f, 0x00000000, 0x00000000,
    0x00000000, 0x42980000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x00009966,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000999b, 0x00000000, 0x00000000, 0x00000000, 0x429c0000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00009a91, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00009ac7,
    0x00000000, 0x00000000, 0x00000000, 0x42980000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x00009bf4, 0x00000000, 0x00000000, 0x00000000, 0x42940000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x00009cf7, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00009d1f,
    0x00000000, 0x00000000, 0x00000000, 0x42960000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x00009e23, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x00009e4b, 0x00000000, 0x00000000,
    0x00000000, 0x429c0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x00009f77,
    0x00000000, 0x00000000, 0x00000000, 0x42a00000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000a0a3, 0x00000000, 0x00000000, 0x00000000, 0x42a00000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000a1d0, 0x00000000, 0x00000000,
    0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000a2fb,
    0x00000000, 0x00000000, 0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000a427, 0x00000000, 0x00000000, 0x00000000, 0x42a40000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000a553, 0x00000000, 0x00000000,
    0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000a680,
    0x00000000, 0x00000000, 0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000a775, 0x00000000, 0x41f00000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000a7ab, 0x00000000, 0x00000000,
    0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000a8a2,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000a8d7, 0x00000000, 0x00000000, 0x00000000, 0x42a20000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000aa03, 0x00000000, 0x00000000,
    0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000ab2f,
    0x00000000, 0x00000000, 0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000ac26, 0x00000000, 0x41f00000, 0x3d99999a, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000ac5b, 0x00000000, 0x00000000,
    0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000ad52,
    0x00000000, 0x00000000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000ad87, 0x00000000, 0x00000000, 0x00000000, 0x42a60000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000ae4e, 0x00000000, 0x00000000,
    0x00000000, 0x42ac0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000aeb3,
    0x00000000, 0x00000000, 0x00000000, 0x42a60000, 0xfdfdfdfd, 0x00000002,
    0x00000000, 0x0000aebe, 0x40066666, 0x00000000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000afe1, 0x00000000, 0x00000000,
    0x00000000, 0x42a60000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000b0d6,
    0x00000000, 0x41f00000, 0x3dcccccd, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000b10b, 0x00000000, 0x00000000, 0x00000000, 0x42a20000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000b202, 0x00000000, 0x41f00000,
    0x3d4ccccd, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000b237,
    0x00000000, 0x00000000, 0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000b32e, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000b363, 0x00000000, 0x00000000,
    0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000b48f,
    0x00000000, 0x00000000, 0x00000000, 0x42a20000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000b5ba, 0x00000000, 0x00000000, 0x00000000, 0x42b80000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000b6e6, 0x00000000, 0x00000000,
    0x00000000, 0x42ca0000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000b7de,
    0x00000000, 0x41f00000, 0x3f99999a, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000b812, 0x00000000, 0x00000000, 0x00000000, 0x42e00000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000b93e, 0x00000000, 0x00000000,
    0x00000000, 0x42f80000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000ba6a,
    0x00000000, 0x00000000, 0x00000000, 0x43040000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000bb96, 0x00000000, 0x00000000, 0x00000000, 0x43080000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000bc8e, 0x00000000, 0x41f00000,
    0x3f99999a, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000bcc2,
    0x00000000, 0x00000000, 0x00000000, 0x43110000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000bdee, 0x00000000, 0x00000000, 0x00000000, 0x431b0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000bf1a, 0x00000000, 0x00000000,
    0x00000000, 0x43230000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000c046,
    0x00000000, 0x00000000, 0x00000000, 0x432a0000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000c13e, 0x00000000, 0x41f00000, 0x3f99999a, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000c172, 0x00000000, 0x00000000,
    0x00000000, 0x43280000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000c29e,
    0x00000000, 0x00000000, 0x00000000, 0x432e0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000c3ca, 0x00000000, 0x00000000, 0x00000000, 0x43270000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000c4c2, 0x00000000, 0x00000000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000003, 0x00000000, 0x0000c4e0,
    0x00000000, 0x00000000, 0x3ecccccd, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000c4f6, 0x00000000, 0x00000000, 0x00000000, 0x43280000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000c5ee, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000c622,
    0x00000000, 0x00000000, 0x00000000, 0x43270000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000c74e, 0x00000000, 0x00000000, 0x00000000, 0x43220000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000c87a, 0x00000000, 0x00000000,
    0x00000000, 0x431d0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000c9a6,
    0x00000000, 0x00000000, 0x00000000, 0x43180000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000ca9e, 0x00000000, 0x41f00000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000cad2, 0x00000000, 0x00000000,
    0x00000000, 0x43140000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000cbfe,
    0x00000000, 0x00000000, 0x00000000, 0x430f0000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000ce57, 0x00000000, 0x00000000, 0x00000000, 0x43020000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000cf4f, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000cf83,
    0x00000000, 0x00000000, 0x00000000, 0x42f40000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000d0af, 0x00000000, 0x00000000, 0x00000000, 0x42e80000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000d1da, 0x00000000, 0x00000000,
    0x00000000, 0x42da0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000d306,
    0x00000000, 0x00000000, 0x00000000, 0x42ce0000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000d400, 0x00000000, 0x41f00000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000d402, 0x00000000, 0x41f00000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000d432,
    0x00000000, 0x00000000, 0x00000000, 0x42be0000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000d52b, 0x00000000, 0x41f00000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000d55e, 0x00000000, 0x00000000,
    0x00000000, 0x42b00000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000d68a,
    0x00000000, 0x00000000, 0x00000000, 0x42a40000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000d7b6, 0x00000000, 0x00000000, 0x00000000, 0x42960000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000d8e2, 0x00000000, 0x00000000,
    0x00000000, 0x42900000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000d99c,
    0x00000000, 0x00000000, 0x00000000, 0x427c0000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000d9de, 0x00000000, 0x41f00000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000da0e, 0x00000000, 0x00000000,
    0x00000000, 0x428a0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000db3a,
    0x00000000, 0x00000000, 0x00000000, 0x42880000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000dc66, 0x00000000, 0x00000000, 0x00000000, 0x42900000,
    0xfdfdfdfd, 0x00000002, 0x00000000, 0x0000dcbd, 0x3ef33333, 0x00000000,
    0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000dd92,
    0x00000000, 0x00000000, 0x00000000, 0x429e0000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000de8b, 0x00000000, 0x00000000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000debe, 0x00000000, 0x00000000,
    0x00000000, 0x42b40000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000dfea,
    0x00000000, 0x00000000, 0x00000000, 0x42ce0000, 0xfdfdfdfd, 0x00000002,
    0x00000000, 0x0000dff8, 0x3f800000, 0x00000000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000e116, 0x00000000, 0x00000000,
    0x00000000, 0x42ea0000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000e20d,
    0x00000000, 0x41f00000, 0x3f99999a, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000e242, 0x00000000, 0x00000000, 0x00000000, 0x43020000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000e36e, 0x00000000, 0x00000000,
    0x00000000, 0x430e0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000e49b,
    0x00000000, 0x00000000, 0x00000000, 0x43160000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000e5c7, 0x00000000, 0x00000000, 0x00000000, 0x43190000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000e6f3, 0x00000000, 0x00000000,
    0x00000000, 0x431e0000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000e6fb,
    0x00000000, 0x41f00000, 0x3f99999a, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000e81f, 0x00000000, 0x00000000, 0x00000000, 0x431d0000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000e94b, 0x00000000, 0x00000000,
    0x00000000, 0x43170000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000ea77,
    0x00000000, 0x00000000, 0x00000000, 0x43130000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000ea7f, 0x00000000, 0x41f00000, 0x3f600000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000eba3, 0x00000000, 0x00000000,
    0x00000000, 0x43120000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000ebac,
    0x00000000, 0x41f00000, 0x3e19999a, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000eccf, 0x00000000, 0x00000000, 0x00000000, 0x430e0000,
    0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000ecd8, 0x00000000, 0x41f00000,
    0x3e000000, 0x00000000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000edcd,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000edfa, 0x00000000, 0x00000000, 0x00000000, 0x43080000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000ef26, 0x00000000, 0x00000000,
    0x00000000, 0x43010000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000f052,
    0x00000000, 0x00000000, 0x00000000, 0x42f00000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000f17e, 0x00000000, 0x00000000, 0x00000000, 0x42e40000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000f2aa, 0x00000000, 0x00000000,
    0x00000000, 0x42d00000, 0xfdfdfdfd, 0x00000001, 0x00000000, 0x0000f2b4,
    0x00000000, 0x41f00000, 0x00000000, 0x00000000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000f3d6, 0x00000000, 0x00000000, 0x00000000, 0x42b00000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000f503, 0x00000000, 0x00000000,
    0x00000000, 0x42960000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000f62e,
    0x00000000, 0x00000000, 0x00000000, 0x42840000, 0xfdfdfdfd, 0x00000001,
    0x00000000, 0x0000f72d, 0x00000000, 0x41f00000, 0x00000000, 0x00000000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000f75a, 0x00000000, 0x00000000,
    0x00000000, 0x426c0000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000f7e4,
    0x00000000, 0x00000000, 0x00000000, 0x42700000, 0xfdfdfdfd, 0x00000000,
    0x00000000, 0x0000f886, 0x00000000, 0x00000000, 0x00000000, 0x42580000,
    0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000f9b2, 0x00000000, 0x00000000,
    0x00000000, 0x42700000, 0xfdfdfdfd, 0x00000000, 0x00000000, 0x0000fade,
    0x00000000, 0x00000000, 0x00000000, 0x42680000 };

u32_t *lu_get_data(int n) {
    return &glucose_data[n*DATA_SIZE];
}


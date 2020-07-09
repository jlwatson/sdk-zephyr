/*
 * Live update - core update logic
 */

#include <kernel.h>
#include <device.h>
#include <drivers/gpio.h>
#include <string.h>
#include <timeout_q.h>
#include <wait_q.h>
#include <update/live_update.h>

extern struct update_header *lu_hdr;
extern u8_t update_write_completed;
extern void gpio_nrfx_init_callback(struct gpio_callback *, gpio_callback_handler_t, gpio_port_pins_t);

bool _lu_update_predicate_satisfied(struct predicate_header *p, u32_t event_addr);
void _lu_state_transfer();
void _cancel_gpio_callbacks();
struct k_timer *_lu_get_timer_for_expiry();
struct gpio_callback *_lu_get_gpio_callback_for_event();

static struct predicate_header *matched_predicate = NULL;

bool lu_trigger_on_timer(struct k_timer *t) {

    if (!update_write_completed || !t || !lu_hdr) {
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
    while ((u32_t)curr_predicate < ((u32_t)predicates + predicates->size)) {
        //printk("current predicate: size = %d, event_handler_addr = %p, n_inactive_ops = %d, n_constraints = %d, n_state_init = %d, hw_transfer_size = %d\n", curr_predicate->size, curr_predicate->event_handler_addr, curr_predicate->n_inactive_ops, curr_predicate->n_constraints, curr_predicate->n_state_init, curr_predicate->hw_transfer_size);

        if (_lu_update_predicate_satisfied(curr_predicate, (u32_t)t->expiry_fn)) {
            matched_predicate = curr_predicate;
            return true;            
        }
        //printk("  no dice\n");
        curr_predicate = (struct predicate_header *)((u8_t *)curr_predicate + curr_predicate->size);
    }

    //printk("returning false\n");
    return false;
}

bool lu_trigger_on_gpio(u32_t cb_addr) {

    if (!update_write_completed || !lu_hdr) {
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

    // Search for a matching predicate; if found, return true
    while ((u32_t)curr_predicate < ((u32_t)predicates + predicates->size)) {
        if (_lu_update_predicate_satisfied(curr_predicate, cb_addr)) {
            matched_predicate = curr_predicate;
            return true;            
        }
        //printk("  no dice\n");
        curr_predicate = (struct predicate_header *)((u8_t *)curr_predicate + curr_predicate->size);
    }

    //printk("returning false\n");
    return false;
}

bool _lu_update_predicate_satisfied(struct predicate_header *p, u32_t event_addr) {

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
        u32_t val = *(u32_t *)curr_constraint->symbol_addr;
        
        struct predicate_constraint_range *curr_range = (struct predicate_constraint_range *)(
                (u8_t *)curr_constraint + sizeof(struct predicate_constraint));
        bool in_range = false;
        for (int j = 0; j < curr_constraint->n_ranges; j++, curr_range++) {
            if (curr_range->lower <= val && val <= curr_range->upper) {
                in_range = true;
                break;
            } 
        }

        // value does not meet constraint
        if (!in_range) {
            return false;
        }

        curr_constraint = (struct predicate_constraint *)((u8_t *)curr_constraint + curr_constraint->size);
    }

    return true;
}

void lu_update_at_timer(struct k_timer **timer) {

    if (!update_write_completed || !timer) return;

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

    printk("timer triggered update done\n");

    // cleanup
    matched_predicate = NULL;
    update_write_completed = 0;
    lu_hdr = NULL;
    lu_uart_reset();
}

void lu_update_at_gpio(struct gpio_callback **callback) {

    if (!update_write_completed | !callback) return;

    _cancel_gpio_callbacks();
    _lu_state_transfer();

    // Swap callback to the new app version. We can find it by going through
    // the hw inits.
    struct gpio_callback *new_callback = (struct gpio_callback *) _lu_get_gpio_callback_for_event();
    while (!new_callback) {
        printk("Error couldn't resolve gpio callback for event in interrupt-triggered live update\n");
    }

    *callback = new_callback;
    
    printk("gpio triggered update done\n");

    // cleanup
    matched_predicate = NULL;
    update_write_completed = 0;
    lu_hdr = NULL;
    lu_uart_reset();
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
            printk("adding %p\n", (void *)*(&curr_hw->args + 1));
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


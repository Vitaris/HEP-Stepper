#include "pico/stdlib.h"
#include <string.h>
#include "pico/multicore.h"
#include "core1_main.h"
#include "hardware/timer.h"
#include "pico/time.h"
#include <stdio.h>
#include "servo_control.h"
#include "stepper_controller.h"
#include <inttypes.h>

// Mark the pointer itself as volatile to prevent cross-core compiler optimization
extern stepper_t * volatile stepper;

static struct repeating_timer comm_refresh_timer;

// Flag to tell the main loop when to print
volatile bool request_print = false;

// Private timer callback (Runs in Hardware Interrupt Context)
static bool comm_refresh_timer_callback(struct repeating_timer *t) {
    request_print = true; // Only set the flag, do NOT printf here
    return true;
}

// Private entry point for Core 1
void core1_main(void) {
    static alarm_pool_t *core1_pool;
    core1_pool = alarm_pool_create(2, 4); 
    
    alarm_pool_add_repeating_timer_ms(
        core1_pool, 
        -100, 
        comm_refresh_timer_callback, 
        NULL, 
        &comm_refresh_timer
    );

    while (true) {
        if (request_print) {
            request_print = false; // Clear the flag
            
            // Print safely in the main thread context
            if (stepper != NULL) {
                printf("Error: Stepper is NOT NULL\n");
                // printf("Position: %" PRId32 "\n", stepper_get_step_count(stepper));
            } else {
                printf("Error: Stepper is NULL\n");
            }
        }
        tight_loop_contents(); 
    }
}
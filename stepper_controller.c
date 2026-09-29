#include "pico/stdlib.h"
#include <stdio.h>
#include <stdlib.h>  
#include "hardware/pio.h"
#include "stepper_controller.h"
#include "stepdir_counter.pio.h"
#include "step_generator.pio.h"
#include "servo_control.h"
#include <math.h>

#define CYCLE_TIME 0.001 // 1 ms cycle time
#define MIN_STEP_FREQ 0.025f  // steps/s; below this the axis is treated as stopped (avoids div-by-zero / huge delays)

void update_step_generator(stepper_t* stepper, float freq, bool direction);

static bool pio_initialized = false;
static uint step_generator_offset = 0; 
static uint offset_encoder = 0;

static pio_hw_t* stepping_pio = pio0;
static pio_hw_t* encoder_pio = pio1;

struct stepper {
    bool steppping_is_active;
    bool enable;
    uint8_t sm;
    uint8_t pin;
    uint32_t step;
    int32_t steps_count;
    float position;
    float microsteps_per_rev;
    float max_speed;
    float max_acceleration;
    servo_control_t* servo_control; // Pointer to servo control structure
};

stepper_t* stepper_init(const uint8_t pin, const uint8_t sm, 
                        const float steps_per_rev, 
                        const float microsteps, 
                        const float max_speed, 
                        const float max_acceleration) {
    stepper_t* stepper = calloc(1, sizeof(struct stepper));
    stepper->pin = pin;
    stepper->sm = sm;
    stepper->microsteps_per_rev = steps_per_rev * microsteps;
    stepper->max_speed = max_speed;
    stepper->max_acceleration = max_acceleration;

    stepper->servo_control = servo_control_init(&stepper->position, &stepper->enable);

    if (!pio_initialized) {
        pio_clear_instruction_memory(stepping_pio);     // Clear pio0
        pio_clear_instruction_memory(encoder_pio);    // Clear pio1

        step_generator_offset = pio_add_program(stepping_pio, &step_generator_program);
        offset_encoder = pio_add_program(encoder_pio, &stepdir_counter_program);
        pio_initialized = true;
    }

    // Step Generator
    step_generator_program_init(stepping_pio, stepper->sm, step_generator_offset, stepper->pin);

    // Stepper Counter
    stepdir_counter_program_init(encoder_pio, stepper->sm, offset_encoder, stepper->pin, 0);

    // Set the Direction pin
    gpio_init(stepper->pin + 1);
    gpio_set_dir(stepper->pin + 1, GPIO_OUT);

    return stepper;
}

void stepper_compute(stepper_t* const stepper) {
    stepper->steps_count = stepdir_counter_get_count(encoder_pio, stepper->sm);

    // Convert to steps, steps per revolution, microsteps
    stepper->position = (float)(stepper->steps_count) / stepper->microsteps_per_rev; 

    // Compute the next position based on the servo control logic
    servo_control_compute(stepper->servo_control);

    // Calculate the speed based on the next position
    float position_error = servo_control_get_next_position(stepper->servo_control) - stepper->position;
    float steps_per_second = position_error * stepper->microsteps_per_rev * 0.5 / CYCLE_TIME;

    // Halt only once the motion profile is done; stopping mid-ramp causes the jerk.
    if (false && servo_control_is_standstill(stepper->servo_control) && fabs(steps_per_second) < MIN_STEP_FREQ) {
        stepper_stop(stepper);
        return;
    }

    stepper_update_speed(stepper, steps_per_second);
}

bool stepper_is_active(stepper_t* stepper) {
    return stepper->enable;
}

void stepper_stop(stepper_t* stepper) {
    step_generator_stop(stepping_pio, stepper->sm);
    stepper->enable = false;
}

void stepper_update_speed(stepper_t* stepper, float speed)
{
    float abs_speed = fabsf(speed);
    bool direction = speed >= 0.0f;

    update_step_generator(stepper, abs_speed, direction);
}

void update_step_generator(stepper_t* stepper, float freq, bool direction)
{
    if (freq < MIN_STEP_FREQ) {
        stepper_stop(stepper);
        return;
    }

    float delay_f = ((float)clock_get_hz(clk_sys) / (2.0f * freq)) - 3.0f;
    uint32_t delay = (uint32_t)delay_f;
    
    // Set the direction pin
    gpio_put(stepper->pin + 1, direction);

    if (stepper->enable) {
        step_generator_update_freq(stepping_pio, stepper->sm, delay);
    }
    else {
        step_generator_init_stepping(stepping_pio, stepper->sm, delay);
        stepper->enable = true;
    }
}

int32_t stepper_get_position(stepper_t* stepper) {
    return stepper->position;
}

void stepper_goto(stepper_t* stepper, float position, float speed) {
    servo_control_goto(stepper->servo_control, position, speed);
}
void stepper_change_acc(stepper_t* stepper, float acc) {
    servo_control_change_acc(stepper->servo_control, acc);
}

bool stepper_is_standstill(stepper_t* stepper) {
    return servo_control_is_standstill(stepper->servo_control);
}

int32_t stepper_get_step_count(stepper_t* stepper) {
    return stepper->steps_count;
}
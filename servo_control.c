#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "servo_control.h"

#define CYCLE_TIME 0.001f
#define FOLLOWING_ERROR 1.0 // Maximum permisible position deviation

void servo_control_calculate_next_position(servo_control_t* servo_control);

struct servo_control {
    
    
    enum positioning{
        IDLE,
        REQUESTED,
        ACCELERATING,
        BRAKING,
        POSITION_REACHED
    } positioning;

    float* current_position;    // Pointer to the current position of the servo motor
    float next_position;

    float next_stop;
    float servo_position;
	float servo_speed;
	uint32_t delay_start;
	bool *enable;
	bool enable_previous;
	float computed_speed;
	bool positive_direction;
	bool set_zero;
	bool nominal_speed_reached;
	float enc_offset;

	// Default movement
	float nominal_speed; 	// Desired motor speed
	float nominal_acc;		// Motor acceleration
	float current_speed; 	// Desired motor speed
	float current_acc;		// Motor acceleration
	float scale;			// Scale factor of the servo_control motor
};

servo_control_t* servo_control_init(float* current_position, bool* enable, float speed, float acc, float scale) {
    servo_control_t* servo_control = calloc(1, sizeof(struct servo_control));
    if (servo_control == NULL) {
        fprintf(stderr, "Failed to allocate memory for servo_control control\n");
        return NULL;
    }
	servo_control->enable = enable;

    servo_control->current_position = current_position;

    // Default values
    servo_control->nominal_speed = speed;
    servo_control->nominal_acc = acc;
    servo_control->current_speed = servo_control->nominal_speed;
    servo_control->current_acc = servo_control->nominal_acc;
    servo_control->scale = scale;

    return servo_control;
}

void servo_control_compute(servo_control_t* servo_control) {
    servo_control_calculate_next_position(servo_control);
}

float get_breaking_distance(const servo_control_t* const servo_control) {
	return 0.5f * servo_control->computed_speed * servo_control->computed_speed / fabsf(servo_control->current_acc);
}

void servo_control_calculate_next_position(servo_control_t* servo_control) {
	switch(servo_control->positioning) {
		case IDLE:
			servo_control->nominal_speed_reached = false;
			break;

		case REQUESTED:
			servo_control->positive_direction = servo_control->next_stop >= *servo_control->current_position;
			servo_control->current_acc = servo_control->positive_direction ? servo_control->nominal_acc : -servo_control->nominal_acc;
			servo_control->current_speed = servo_control->positive_direction ? servo_control->nominal_speed : -servo_control->nominal_speed;

			if (servo_control->delay_start > 0) {
				servo_control->delay_start--;
				break;
			}
			servo_control->computed_speed = 0.0f;
			servo_control->positioning = ACCELERATING;
			break;

		case ACCELERATING:
			servo_control->computed_speed += servo_control->current_acc * CYCLE_TIME;

			if (fabsf(servo_control->computed_speed) > fabsf(servo_control->current_speed)) {
				servo_control->nominal_speed_reached = true;
				servo_control->computed_speed = servo_control->current_speed;
			}

			servo_control->next_position += servo_control->computed_speed * CYCLE_TIME;

			if (fabsf(servo_control->next_stop - servo_control->next_position) < get_breaking_distance(servo_control)) {
				servo_control->positioning = BRAKING;
			}
			break;

		case BRAKING:
			servo_control->computed_speed -= servo_control->current_acc * CYCLE_TIME;
			servo_control->next_position += servo_control->computed_speed * CYCLE_TIME;
			servo_control->nominal_speed_reached = false;

			// Speed has crossed zero (sign opposite to current_acc) -> motion finished
			if (servo_control->computed_speed * servo_control->current_acc <= 0.0f) {
				servo_control->positioning = POSITION_REACHED;
			}
			break;

		case POSITION_REACHED:
			servo_control->next_position = servo_control->next_stop;
			servo_control->positioning = IDLE;
			break;
	}
}

void _servo_goto(servo_control_t* const const servo_control, const float position, const float speed) {
	servo_control->next_stop = position / servo_control->scale;
	servo_control->nominal_speed = speed / servo_control->scale;
	if (servo_control->delay_start == 0) {
		servo_control->delay_start = 500;
	}
	else if (servo_control->delay_start == UINT32_MAX) { // TODO, add some sign to not add delay, e.g. maximum number
		servo_control->delay_start = 0;
	}
	servo_control->positioning = REQUESTED;
}

float servo_control_get_next_position(servo_control_t* servo_control) {
	return servo_control->next_position;
}

void servo_control_goto(servo_control_t* const servo_control, const float position, const float speed) {
    // Check if the servo control is already in a positioning state
	if (servo_control->positioning != IDLE) {
		// If it is, we can just update the next stop position and speed
		servo_control->next_stop = position / servo_control->scale;
		servo_control->nominal_speed = speed / servo_control->scale;
		return;
	}
    
    // If not, we can start a new positioning request
    _servo_goto(servo_control, position, speed);
}

void servo_control_change_acc(servo_control_t* const servo_control, const float acc) {
	servo_control->nominal_acc = acc;
}

bool servo_control_is_standstill(servo_control_t* servo_control) {
	return servo_control->positioning == IDLE;
}

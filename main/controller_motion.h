#pragma once
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t last_active_ms;
    uint8_t state; /* 0=unknown, 1=still, 2=moving (input intent, not odometry). */
} controller_motion_t;

static inline uint8_t controller_motion_update(controller_motion_t *motion,
    uint32_t now_ms, bool fresh, bool connected, bool allowed,
    int8_t left_x, int8_t left_y, uint8_t buttons)
{
    if (!fresh || !connected) {
        motion->last_active_ms = now_ms;
        motion->state = 0U;
    } else if (allowed && (left_x != 0 || left_y != 0 || (buttons & 0xC0U) != 0U)) {
        motion->last_active_ms = now_ms;
        motion->state = 2U;
    } else if (now_ms - motion->last_active_ms >= 300U) {
        motion->state = 1U;
    }
    return motion->state;
}

#pragma once

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint16_t hp;
    uint16_t heat;
    uint16_t power;
    bool alive;
    bool shoot_enabled;
    bool power_on;
    uint8_t sequence;
} l431_status_t;

typedef enum {
    L431_EVENT_ATTACK = 0x05,
    L431_EVENT_HIT = 0x04,
    L431_EVENT_DEATH = 0x02,
    L431_EVENT_REVIVE = 0x03,
    L431_EVENT_SHOOT_ENABLED = 0x06,
    L431_EVENT_SHOOT_DISABLED = 0x07,
    L431_EVENT_COMBAT_END = 0x0B,
} l431_event_t;

typedef void (*l431_status_callback_t)(const l431_status_t *status, void *context);
typedef void (*l431_event_callback_t)(l431_event_t event, uint8_t sequence, void *context);
typedef void (*l431_ack_callback_t)(uint8_t command, uint32_t transaction_id,
                                    uint8_t result, void *context);

typedef struct {
    l431_status_callback_t on_status;
    l431_event_callback_t on_event;
    l431_ack_callback_t on_ack;
    void *context;
} l431_link_callbacks_t;

void l431_link_init(const l431_link_callbacks_t *callbacks);
void l431_link_feed(uint8_t byte);
uint8_t l431_link_crc8(const uint8_t *data, uint8_t length);

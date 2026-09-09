#include "l431_link.h"

#include <string.h>

static l431_link_callbacks_t s_callbacks;
static uint8_t s_rx[11];
static uint8_t s_length;

uint8_t l431_link_crc8(const uint8_t *data, uint8_t length)
{
    uint8_t crc = 0;
    while (length-- != 0U) {
        uint8_t bit;
        crc ^= *data++;
        for (bit = 0; bit < 8U; ++bit)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
    }
    return crc;
}

static uint16_t le16(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static uint32_t le32(const uint8_t *data)
{
    return (uint32_t)data[0] | ((uint32_t)data[1] << 8U) |
           ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
}

void l431_link_init(const l431_link_callbacks_t *callbacks)
{
    s_length = 0;
    s_callbacks = (callbacks != NULL) ? *callbacks : (l431_link_callbacks_t){0};
}

void l431_link_feed(uint8_t byte)
{
    uint8_t expected = 0;
    if (s_length == 0U) { if (byte == 0xA5U || (byte >= 0xB1U && byte <= 0xB6U) || (byte >= 0xD1U && byte <= 0xD6U)) s_rx[s_length++] = byte; return; }
    if (s_length == 1U) {
        const uint8_t first = s_rx[0];
        const uint8_t second = (first == 0xA5U) ? 0x5AU :
            (uint8_t)((first << 4U) | (first >> 4U));
        if (byte == second) s_rx[s_length++] = byte; else { s_length = 0; l431_link_feed(byte); }
        return;
    }
    expected = (s_rx[0] == 0xA5U) ? 11U : ((s_rx[0] >= 0xB1U && s_rx[0] <= 0xB6U) ? 4U : 8U);
    s_rx[s_length++] = byte;
    if (s_length < expected) return;
    if (l431_link_crc8(s_rx, (uint8_t)(expected - 1U)) == s_rx[expected - 1U]) {
        if (s_rx[0] == 0xA5U && s_callbacks.on_status != NULL) {
            l431_status_t status = {.sequence = s_rx[2], .alive = (s_rx[3] & 1U) != 0U,
                .shoot_enabled = (s_rx[3] & 2U) != 0U, .power_on = (s_rx[3] & 4U) != 0U,
                .hp = le16(&s_rx[4]),
                .heat = le16(&s_rx[6]), .power = le16(&s_rx[8])};
            s_callbacks.on_status(&status, s_callbacks.context);
        } else if (s_rx[0] >= 0xB1U && s_rx[0] <= 0xB6U && s_callbacks.on_event != NULL) {
            static const l431_event_t events[] = {L431_EVENT_ATTACK, L431_EVENT_HIT,
                L431_EVENT_DEATH, L431_EVENT_REVIVE, L431_EVENT_SHOOT_ENABLED, L431_EVENT_SHOOT_DISABLED};
            s_callbacks.on_event(events[s_rx[0] - 0xB1U], s_rx[2], s_callbacks.context);
        } else if (s_callbacks.on_ack != NULL) {
            s_callbacks.on_ack((uint8_t)(s_rx[0] - 0x10U), le32(&s_rx[2]), s_rx[6], s_callbacks.context);
        }
    }
    s_length = 0;
}

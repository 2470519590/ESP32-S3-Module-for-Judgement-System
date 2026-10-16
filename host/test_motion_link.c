#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "../main/controller_motion.h"
#include "../main/l431_link.h"
#include "../main/robot_protocol.h"
static unsigned threshold_count, ack_count;
static void threshold(const l431_threshold_t *state, void *context)
{
    (void)context; assert(state->node == 2 && state->still == 700000);
    assert(state->moving == 1000000 && state->applied == 1000000);
    threshold_count++;
}
static void ack(uint8_t command, uint32_t tx, uint8_t result, void *context)
{
    (void)context; assert(command == 0xC9 && tx == 1234 && result == 0); ack_count++;
}
int main(void)
{
    controller_motion_t motion = {0};
    assert(controller_motion_update(&motion, 300, true, true, true, 0, 0, 0) == 1);
    assert(controller_motion_update(&motion, 310, true, true, true, 1, 0, 0) == 2);
    assert(controller_motion_update(&motion, 609, true, true, true, 0, 0, 0) == 2);
    assert(controller_motion_update(&motion, 610, true, true, true, 0, 0, 0) == 1);
    assert(controller_motion_update(&motion, 620, true, true, true, 0, 0, 0x40) == 2);
    assert(controller_motion_update(&motion, 920, true, true, true, 0, 0, 0x80) == 2);
    assert(controller_motion_update(&motion, 921, false, true, true, 0, 0, 0) == 0);
    assert(controller_motion_update(&motion, 922, true, false, true, 0, 0, 0x40) == 0);
    assert(controller_motion_update(&motion, 1222, true, true, false, 100, 0, 0xC0) == 1);
    motion.last_active_ms = UINT32_MAX - 100; motion.state = 2;
    assert(controller_motion_update(&motion, 200, true, true, true, 0, 0, 0) == 1);
    l431_link_init(&(l431_link_callbacks_t){.on_threshold = threshold, .on_ack = ack});
    uint8_t frame[21] = {0xA6, 0x6A, 1, 2, 15, 0, 1, 2};
    uint32_t still = 700000, moving = 1000000;
    memcpy(frame + 8, &still, 4); memcpy(frame + 12, &moving, 4); memcpy(frame + 16, &moving, 4);
    frame[20] = l431_link_crc8(frame, 20);
    for (unsigned i = 0; i < 21; i++) l431_link_feed(frame[i]);
    assert(threshold_count == 1);
    frame[20] ^= 1;
    for (unsigned i = 0; i < 21; i++) l431_link_feed(frame[i]);
    assert(threshold_count == 1);
    uint8_t reply[8] = {0xD9, 0x9D, 0xD2, 4, 0, 0, 0};
    reply[7] = l431_link_crc8(reply, 7);
    for (unsigned i = 0; i < 8; i++) l431_link_feed(reply[i]);
    assert(ack_count == 1);
    puts("ESP32 motion: axes, LB/RB, 300-ms release, unknown, wraparound; A6/D9 parser passed");
}

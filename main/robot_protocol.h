#pragma once

#include <stdbool.h>
#include <stdint.h>

/* All multi-byte fields are little-endian (ESP32 native byte order). */
#define ROBOT_PROTOCOL_MAGIC 0x5254U
#define ROBOT_PROTOCOL_VERSION 1U

typedef enum {
    ROBOT_FRAME_STATUS = 1,
    ROBOT_FRAME_EVENT = 2,
} robot_frame_type_t;

typedef enum {
    ROBOT_EVENT_DEATH = 1,
    ROBOT_EVENT_REVIVE = 2,
    ROBOT_EVENT_HIT = 3,
    /* Send once when attack/combat starts, not once per projectile. */
    ROBOT_EVENT_ATTACK = 4,
    ROBOT_EVENT_SHOOT_ENABLED = 5,
    ROBOT_EVENT_SHOOT_DISABLED = 6,
} robot_event_type_t;

/* Sent periodically at 10 Hz. Size: 10 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
    uint16_t hp;
    uint8_t alive;          /* 0 = dead, 1 = alive */
    uint8_t shoot_enabled;  /* 0 = forbidden, 1 = allowed */
} robot_status_frame_t;

/* Sent once for each state-changing broadcast. Size: 12 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
    uint8_t event_type;
    uint8_t subject_robot_id; /* 0 = this robot; hit target otherwise */
    uint16_t value;           /* HP after hit; otherwise 0 */
    uint16_t event_id;        /* increments for duplicate/loss diagnosis */
} robot_event_frame_t;

/* Call these from normal FreeRTOS task context after obtaining data from L431. */
void robot_network_set_status(uint16_t hp, bool alive, bool shoot_enabled);
bool robot_network_publish_event(robot_event_type_t event_type,
                                 uint8_t subject_robot_id, uint16_t value);

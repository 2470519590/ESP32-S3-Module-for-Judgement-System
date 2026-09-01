#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* All multi-byte fields are little-endian (ESP32 native byte order). */
#define ROBOT_PROTOCOL_MAGIC 0x5254U
#define ROBOT_PROTOCOL_VERSION 1U

typedef enum {
    ROBOT_FRAME_STATUS = 1,
    ROBOT_FRAME_DEATH = 2,
    ROBOT_FRAME_REVIVE = 3,
    ROBOT_FRAME_HIT = 4,
    ROBOT_FRAME_ATTACK = 5,
    ROBOT_FRAME_SHOOT_ENABLED = 6,
    ROBOT_FRAME_SHOOT_DISABLED = 7,
} robot_frame_type_t;

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

/* Sent once when this robot dies. Size: 6 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
} robot_death_frame_t;

/* Sent once when this robot revives. Size: 6 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
} robot_revive_frame_t;

/* Sent once when this robot is hit. Size: 8 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
    uint16_t hp;
} robot_hit_frame_t;

/* Sent once when this robot enters attack/combat state. Size: 6 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
} robot_attack_frame_t;

/* Sent once when this robot is allowed to shoot. Size: 6 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
} robot_shoot_enabled_frame_t;

/* Sent once when this robot is forbidden from shooting. Size: 6 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t team;
} robot_shoot_disabled_frame_t;

/* Call these from normal FreeRTOS task context after obtaining data from L431. */
void robot_network_set_status(uint16_t hp, bool alive, bool shoot_enabled);
bool robot_network_publish_death(void);
bool robot_network_publish_revive(void);
bool robot_network_publish_hit(uint16_t hp);
bool robot_network_publish_attack(void);
bool robot_network_publish_shoot_enabled(void);
bool robot_network_publish_shoot_disabled(void);

/* Raw UDP downlink reservation only. No downlink frame format is defined yet.
 * Later, define separate frame types per business command before using it.
 */
void robot_network_on_server_datagram(const uint8_t *data, size_t length);

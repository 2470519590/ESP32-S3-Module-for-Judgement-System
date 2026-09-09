#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* All multi-byte fields are little-endian (ESP32 native byte order). */
#define ROBOT_PROTOCOL_MAGIC 0x5254U
#define ROBOT_PROTOCOL_VERSION 2U

typedef enum {
    ROBOT_FRAME_STATUS = 0x01,
    ROBOT_FRAME_DEATH = 0x02,
    ROBOT_FRAME_REVIVE = 0x03,
    ROBOT_FRAME_HIT = 0x04,
    ROBOT_FRAME_ATTACK = 0x05,
    ROBOT_FRAME_SHOOT_ENABLED = 0x06,
    ROBOT_FRAME_SHOOT_DISABLED = 0x07,
    ROBOT_FRAME_REFEREE_LINK_DOWN = 0x08,
    ROBOT_FRAME_REFEREE_LINK_UP = 0x09,
    ROBOT_FRAME_GAME_START = 0x81,
    ROBOT_FRAME_GAME_END = 0x82,
    ROBOT_FRAME_ASSIGNMENT = 0x83,
    ROBOT_FRAME_STATUS_REQUEST = 0x84,
    ROBOT_FRAME_SET_HP = 0x85,
    ROBOT_FRAME_YELLOW_CARD = 0x86,
    ROBOT_FRAME_FORCE_POWER_OFF = 0x87,
    ROBOT_FRAME_FORCE_POWER_ON = 0x88,
    ROBOT_FRAME_ACK = 0xF0,
} robot_frame_type_t;

/* ESP32 -> server, 10 Hz. Size: 14 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint16_t hp;
    uint16_t heat;
    uint16_t power;
    uint8_t alive;
    uint8_t shoot_enabled;
    uint8_t power_on;
} robot_status_v2_frame_t;

/* ESP32 -> server, non-reliable events. Size: 5 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
} robot_event_v2_frame_t;


/* ESP32 -> server, death/revive only. Size: 9 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint32_t transaction_id;
} robot_reliable_event_v2_frame_t;

/* Server -> ESP32, game start/end. Size: 9 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t target_robot_id;
    uint32_t transaction_id;
} robot_game_control_v2_frame_t;

/* Server -> ESP32, one yellow-card penalty. Size: 9 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t target_robot_id;
    uint32_t transaction_id;
} robot_yellow_card_v2_frame_t;

/* Server -> ESP32, force L431PM to switch off chassis output. Size: 9 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t target_robot_id;
    uint32_t transaction_id;
} robot_force_power_off_v2_frame_t;

/* Server -> ESP32, request L431PM to switch on chassis output. Size: 9 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t target_robot_id;
    uint32_t transaction_id;
} robot_force_power_on_v2_frame_t;

/* Server -> ESP32, persistent robot and controller assignment. Size: 15 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t robot_id;
    uint8_t controller_mac[6];
    uint32_t transaction_id;
} robot_assignment_v2_frame_t;

/* Server -> ESP32, request one immediate status frame. Size: 5 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t target_robot_id;
} robot_status_request_v2_frame_t;

/* Server -> ESP32, set HP through L431. Size: 11 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t target_robot_id;
    uint16_t hp;
    uint32_t transaction_id;
} robot_set_hp_v2_frame_t;

/* Both directions, transport acknowledgement. Size: 10 bytes. */
typedef struct __attribute__((packed)) {
    uint16_t magic;
    uint8_t version;
    uint8_t frame_type;
    uint8_t acked_frame_type;
    uint32_t transaction_id;
    uint8_t result; /* 0 success, 1 rejected, 2 failed */
} robot_ack_v2_frame_t;

/* Call these from normal FreeRTOS task context after obtaining data from L431. */
void robot_network_set_status(uint16_t hp, bool alive, bool shoot_enabled);
bool robot_network_publish_death(void);
bool robot_network_publish_revive(void);
bool robot_network_publish_hit(uint16_t hp);
bool robot_network_publish_attack(void);
bool robot_network_publish_shoot_enabled(void);
bool robot_network_publish_shoot_disabled(void);

/* V2 UDP downlink is implemented in main.c. Each business command has its own
 * fixed frame type; this callback handles server ACKs for reliable events. */
void robot_network_on_server_datagram(const uint8_t *data, size_t length);

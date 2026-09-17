#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_random.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "robot_protocol.h"
#include "l431_link.h"
#include "xbox_ble.h"

/* Older generated sdkconfig files do not yet contain the test-only fallback
 * Kconfig entries. Keep this source buildable until the local ESP-IDF Python
 * environment is repaired and configuration can be regenerated. */
#ifndef CONFIG_ROBOT_TEST_FALLBACK_WIFI_SSID
#define CONFIG_ROBOT_TEST_FALLBACK_WIFI_SSID "evil rats crazily squeak"
#endif
#ifndef CONFIG_ROBOT_TEST_FALLBACK_WIFI_PASSWORD
#define CONFIG_ROBOT_TEST_FALLBACK_WIFI_PASSWORD "66666666"
#endif
#ifndef CONFIG_ROBOT_TEST_MODE
#define CONFIG_ROBOT_TEST_MODE 0
#endif
#define TEST_SERVER_IP "10.25.81.216"
/* Current formal-business integration uses the temporary test network.
 * This selects only Wi-Fi/server endpoints; simulated data remains disabled. */
#define USE_INTEGRATION_TEST_NETWORK 1

#define WIFI_CONNECTED_BIT BIT0
#define STATUS_PERIOD_MS 100U
#define EVENT_QUEUE_LENGTH 16U
#define DOWNLINK_BUFFER_SIZE 64U
#define COMPLETED_DOWNLINK_CAPACITY 32U
#define RELIABLE_EVENT_CAPACITY 4U
#define RELIABLE_RETRY_MS 100U
#define RELIABLE_RETRY_BACKOFF_MS 1000U
#define DEVICE_ANNOUNCE_PERIOD_MS 2000U

/* UART0 is the controller-facing port; UART1 is the L431PM link. */
#define CONTROLLER_UART_TX_GPIO 43
#define CONTROLLER_UART_RX_GPIO 44
#define L431_UART_TX_GPIO 17
#define L431_UART_RX_GPIO 18
#define UART_RX_BUFFER_SIZE 512

static const char *TAG = "robot_net";
static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_status_mutex;
static QueueHandle_t s_event_queue;
static QueueHandle_t s_l431_ack_queue;
static QueueHandle_t s_downlink_queue;
static SemaphoreHandle_t s_reliable_mutex;
static SemaphoreHandle_t s_completed_mutex;
static SemaphoreHandle_t s_identity_mutex;
static volatile bool s_l431_status_seen;
static volatile TickType_t s_l431_last_status_tick;
static volatile uint8_t s_l431_valid_status_count;
static volatile bool s_l431_link_up;
/* A physical ESP32 is anonymous until the server assigns a robot ID to its
 * factory Wi-Fi MAC.  This prevents two newly flashed boards from both
 * claiming robot 1 on the same test network. */
static uint8_t s_robot_id;
static uint8_t s_device_mac[6];
static uint8_t s_controller_mac[6];
static bool s_controller_assigned;
static volatile uint32_t s_event_queue_drops;
static volatile uint32_t s_udp_send_failures;
/* Set after an NVS assignment commit so the server's current device table is
 * refreshed immediately instead of waiting for the next 2 s heartbeat. */
static volatile bool s_device_announce_pending = true;

static uint8_t robot_id_snapshot(void)
{
    uint8_t robot_id;
    xSemaphoreTake(s_identity_mutex, portMAX_DELAY);
    robot_id = s_robot_id;
    xSemaphoreGive(s_identity_mutex);
    return robot_id;
}

typedef struct {
    uint16_t hp;
    uint16_t heat;
    uint16_t power;
    bool alive;
    bool shoot_enabled;
    bool power_on;
} robot_state_t;

typedef struct {
    robot_frame_type_t frame_type;
    uint16_t hp;
} pending_event_t;

typedef struct {
    uint8_t command;
    uint32_t transaction_id;
    uint8_t result;
} l431_ack_t;

typedef struct {
    uint8_t frame_type;
    uint8_t l431_command;
    uint16_t hp;
    uint32_t transaction_id;
    struct sockaddr_in peer;
} downlink_command_t;

typedef struct __attribute__((packed)) {
    uint8_t frame_type;
    uint8_t robot_id;
    /* 0xFF means a locally generated test event.  Real L431 events retain
     * their UART sequence so a retransmission cannot create another UDP
     * transaction. */
    uint8_t l431_sequence;
    uint32_t transaction_id;
} persisted_reliable_event_t;

typedef struct {
    persisted_reliable_event_t persisted;
    uint8_t attempts;
    TickType_t next_send_tick;
} reliable_event_t;

static reliable_event_t s_reliable_events[RELIABLE_EVENT_CAPACITY];

typedef struct __attribute__((packed)) {
    uint8_t frame_type;
    uint32_t transaction_id;
    uint8_t result;
} completed_downlink_t;

static completed_downlink_t s_completed_downlinks[COMPLETED_DOWNLINK_CAPACITY];
static uint8_t s_completed_downlink_next;

static bool queue_event(robot_frame_type_t frame_type, uint16_t hp);
static bool enqueue_reliable_event(robot_frame_type_t frame_type, uint8_t l431_sequence);
static bool find_completed_downlink(uint8_t frame_type, uint32_t transaction_id,
                                    uint8_t *result);
static void remember_completed_downlink(uint8_t frame_type, uint32_t transaction_id,
                                        uint8_t result);
static void send_server_ack(int sock, const struct sockaddr_in *peer,
                            uint8_t acked_type, uint32_t transaction_id,
                            uint8_t result);

static bool send_l431_event_ack(l431_event_t event, uint8_t sequence)
{
    uint8_t frame[4];
    if (event == L431_EVENT_DEATH) frame[0] = 0xE3U;
    else if (event == L431_EVENT_REVIVE) frame[0] = 0xE4U;
    else return false;
    frame[1] = (uint8_t)((frame[0] << 4U) | (frame[0] >> 4U));
    frame[2] = sequence;
    frame[3] = l431_link_crc8(frame, 3U);
    return uart_write_bytes(UART_NUM_1, frame, sizeof(frame)) == sizeof(frame);
}

static robot_state_t s_state = {
    .hp = 200,
    .alive = true,
    .shoot_enabled = true,
};

static void l431_on_status(const l431_status_t *status, void *context)
{
    (void)context;
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_state.hp = status->hp;
    s_state.heat = status->heat;
    s_state.power = status->power;
    s_state.alive = status->alive;
    s_state.shoot_enabled = status->shoot_enabled;
    s_state.power_on = status->power_on;
    xSemaphoreGive(s_status_mutex);
    s_l431_status_seen = true;
    s_l431_last_status_tick = xTaskGetTickCount();
    if (s_l431_valid_status_count < 3U) ++s_l431_valid_status_count;
    if (!s_l431_link_up && s_l431_valid_status_count >= 3U) {
        s_l431_link_up = true;
        if (robot_id_snapshot() != 0U)
            (void)queue_event(ROBOT_FRAME_REFEREE_LINK_UP, 0U);
    }
}

static void l431_on_event(l431_event_t event, uint8_t sequence, void *context)
{
    bool accepted = false;
    (void)context;
    switch (event) {
    /* Persist before acknowledging L431.  This prevents an ESP32 reset in
     * the UART-to-UDP handoff window from losing a death/revive event. */
    case L431_EVENT_DEATH:
        accepted = enqueue_reliable_event(ROBOT_FRAME_DEATH, sequence);
        break;
    case L431_EVENT_REVIVE:
        accepted = enqueue_reliable_event(ROBOT_FRAME_REVIVE, sequence);
        break;
    case L431_EVENT_HIT:
        if (robot_id_snapshot() != 0U) (void)robot_network_publish_hit(0);
        break;
    case L431_EVENT_ATTACK:
        if (robot_id_snapshot() != 0U) (void)robot_network_publish_attack();
        break;
    case L431_EVENT_COMBAT_END:
        if (robot_id_snapshot() != 0U) (void)robot_network_publish_combat_end();
        break;
    case L431_EVENT_SHOOT_ENABLED:
        if (robot_id_snapshot() != 0U) (void)robot_network_publish_shoot_enabled();
        break;
    case L431_EVENT_SHOOT_DISABLED:
        if (robot_id_snapshot() != 0U) (void)robot_network_publish_shoot_disabled();
        break;
    default: break;
    }
    if (accepted && !send_l431_event_ack(event, sequence))
        ESP_LOGW(TAG, "Cannot acknowledge L431 event 0x%02X seq=%u", event, sequence);
}

static void l431_on_ack(uint8_t command, uint32_t transaction_id,
                        uint8_t result, void *context)
{
    const l431_ack_t ack = {
        .command = command,
        .transaction_id = transaction_id,
        .result = result,
    };
    (void)context;
    (void)xQueueSend(s_l431_ack_queue, &ack, 0);
}

static void __attribute__((unused)) l431_uart_task(void *arg)
{
    uint8_t buffer[64];
    (void)arg;
    while (true) {
        const int count = uart_read_bytes(UART_NUM_1, buffer, sizeof(buffer),
                                          pdMS_TO_TICKS(100));
        for (int index = 0; index < count; ++index) l431_link_feed(buffer[index]);
    }
}

static void __attribute__((unused)) l431_link_monitor_task(void *arg)
{
    (void)arg;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(100));
        if (s_l431_link_up &&
            (xTaskGetTickCount() - s_l431_last_status_tick) >= pdMS_TO_TICKS(300)) {
            s_l431_link_up = false;
            s_l431_valid_status_count = 0U;
            s_l431_status_seen = false;
            (void)queue_event(ROBOT_FRAME_REFEREE_LINK_DOWN, 0U);
        }
    }
}

void robot_network_on_server_datagram(
    const uint8_t *data, size_t length)
{
    if (length == 10U && data[0] == 0x54U && data[1] == 0x52U &&
        data[2] == ROBOT_PROTOCOL_VERSION && data[3] == ROBOT_FRAME_ACK) {
        const robot_ack_v2_frame_t *ack = (const void *)data;
        if (ack->result != 0U) return;
        xSemaphoreTake(s_reliable_mutex, portMAX_DELAY);
        for (uint8_t index = 0; index < RELIABLE_EVENT_CAPACITY; ++index) {
            reliable_event_t *event = &s_reliable_events[index];
            if (event->persisted.frame_type == ack->acked_frame_type &&
                event->persisted.transaction_id == ack->transaction_id) {
                memset(event, 0, sizeof(*event));
                nvs_handle_t handle;
                if (nvs_open("robot", NVS_READWRITE, &handle) == ESP_OK) {
                    (void)nvs_set_blob(handle, "pending", s_reliable_events,
                                       sizeof(s_reliable_events));
                    (void)nvs_commit(handle);
                    nvs_close(handle);
                }
                break;
            }
        }
        xSemaphoreGive(s_reliable_mutex);
    }
}

void robot_network_set_status(uint16_t hp, bool alive, bool shoot_enabled)
{
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_state.hp = hp;
    s_state.alive = alive;
    s_state.shoot_enabled = shoot_enabled;
    xSemaphoreGive(s_status_mutex);
}

static bool queue_event(robot_frame_type_t frame_type, uint16_t hp)
{
    pending_event_t event = {
        .frame_type = frame_type,
        .hp = hp,
    };
    if (xQueueSend(s_event_queue, &event, 0) == pdPASS) return true;
    const uint32_t drops = ++s_event_queue_drops;
    if (drops == 1U || (drops % 16U) == 0U)
        ESP_LOGW(TAG, "Event queue full; %" PRIu32 " frames dropped", drops);
    return false;
}

static bool persist_reliable_events(void)
{
    nvs_handle_t handle;
    if (nvs_open("robot", NVS_READWRITE, &handle) != ESP_OK) return false;
    esp_err_t error = nvs_set_blob(handle, "pending", s_reliable_events,
                                   sizeof(s_reliable_events));
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    return error == ESP_OK;
}

static bool enqueue_reliable_event(robot_frame_type_t frame_type, uint8_t l431_sequence)
{
    bool queued = false;
    const uint8_t robot_id = robot_id_snapshot();
    if (robot_id == 0U) return false;
    xSemaphoreTake(s_reliable_mutex, portMAX_DELAY);
    for (uint8_t index = 0; index < RELIABLE_EVENT_CAPACITY; ++index) {
        reliable_event_t *event = &s_reliable_events[index];
        if (l431_sequence != 0xFFU &&
            event->persisted.frame_type == (uint8_t)frame_type &&
            event->persisted.l431_sequence == l431_sequence) {
            queued = true;
            break;
        }
        if (event->persisted.frame_type == 0U) {
            event->persisted.frame_type = (uint8_t)frame_type;
            event->persisted.robot_id = robot_id;
            event->persisted.l431_sequence = l431_sequence;
            event->persisted.transaction_id = esp_random();
            if (event->persisted.transaction_id == 0U)
                event->persisted.transaction_id = 1U;
            event->next_send_tick = 0;
            queued = persist_reliable_events();
            if (!queued) memset(event, 0, sizeof(*event));
            break;
        }
    }
    xSemaphoreGive(s_reliable_mutex);
    return queued;
}

bool robot_network_publish_death(void)
{
    return queue_event(ROBOT_FRAME_DEATH, 0);
}

bool robot_network_publish_revive(void)
{
    return queue_event(ROBOT_FRAME_REVIVE, 0);
}

bool robot_network_publish_hit(uint16_t hp)
{
    return queue_event(ROBOT_FRAME_HIT, hp);
}

bool robot_network_publish_attack(void)
{
    return queue_event(ROBOT_FRAME_ATTACK, 0);
}

bool robot_network_publish_combat_end(void)
{
    return queue_event(ROBOT_FRAME_COMBAT_END, 0);
}

bool robot_network_publish_shoot_enabled(void)
{
    return queue_event(ROBOT_FRAME_SHOOT_ENABLED, 0);
}

bool robot_network_publish_shoot_disabled(void)
{
    return queue_event(ROBOT_FRAME_SHOOT_DISABLED, 0);
}

#if CONFIG_ROBOT_TEST_MODE
static void robot_test_task(void *arg)
{
    while (true) {
        const uint32_t delay_ms = 3000U + (esp_random() % 3001U);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));

        robot_state_t state;
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        state = s_state;
        xSemaphoreGive(s_status_mutex);

        if (!state.alive) {
            robot_network_set_status(200, true, true);
            (void)robot_network_publish_revive();
            ESP_LOGI(TAG, "TEST: revive, hp=200");
            continue;
        }

        switch (esp_random() % 4U) {
        case 0: { /* Hit: reduce HP, then report the updated value. */
            const uint16_t damage = 5U + (esp_random() % 26U);
            const uint16_t hp = (state.hp > damage) ? state.hp - damage : 0U;
            robot_network_set_status(hp, hp != 0U, state.shoot_enabled);
            if (hp == 0U) {
                (void)robot_network_publish_death();
                ESP_LOGI(TAG, "TEST: death");
            } else {
                (void)robot_network_publish_hit(hp);
                ESP_LOGI(TAG, "TEST: hit, hp=%u", hp);
            }
            break;
        }
        case 1:
            if (state.shoot_enabled) {
                (void)robot_network_publish_attack();
                ESP_LOGI(TAG, "TEST: attack");
            } else {
                robot_network_set_status(state.hp, true, true);
                (void)robot_network_publish_shoot_enabled();
                ESP_LOGI(TAG, "TEST: shoot enabled");
            }
            break;
        case 2:
            robot_network_set_status(state.hp, true, false);
            (void)robot_network_publish_shoot_disabled();
            ESP_LOGI(TAG, "TEST: shoot disabled");
            break;
        default:
            robot_network_set_status(state.hp, true, true);
            (void)robot_network_publish_shoot_enabled();
            ESP_LOGI(TAG, "TEST: shoot enabled");
            break;
        }
    }
}
#endif

static bool send_frame(int sock, const struct sockaddr_in *server,
                       const void *frame, size_t frame_size)
{
    const int sent = sendto(sock, frame, frame_size, 0,
                            (const struct sockaddr *)server, sizeof(*server));
    if (sent == (int)frame_size) return true;
    const uint32_t failures = ++s_udp_send_failures;
    if (failures == 1U || (failures % 16U) == 0U)
        ESP_LOGW(TAG, "UDP send failures=%" PRIu32 " (last %d, expected %u)",
                 failures, sent, (unsigned int)frame_size);
    return false;
}

static bool downlink_targets_this_robot(uint8_t target_robot_id)
{
    return target_robot_id != 0U && target_robot_id == robot_id_snapshot();
}

static uint8_t forward_l431_command(uint8_t command, uint32_t transaction_id,
                                    uint16_t hp)
{
    uint8_t frame[9];
    const uint8_t length = (command == 0xC3U) ? 9U : 7U;
    l431_ack_t ack;

    /* Do not acknowledge a command as executed when the referee link is absent. */
    if (!s_l431_status_seen) return 2U;

    frame[0] = command;
    frame[1] = (uint8_t)((command << 4U) | (command >> 4U));
    frame[2] = (uint8_t)transaction_id;
    frame[3] = (uint8_t)(transaction_id >> 8U);
    frame[4] = (uint8_t)(transaction_id >> 16U);
    frame[5] = (uint8_t)(transaction_id >> 24U);
    if (command == 0xC3U) {
        frame[6] = (uint8_t)hp;
        frame[7] = (uint8_t)(hp >> 8U);
    }
    frame[length - 1U] = l431_link_crc8(frame, (uint8_t)(length - 1U));
    ESP_LOGI(TAG, "L431 TX cmd=0x%02X tx=%" PRIu32 " len=%u crc=0x%02X",
             command, transaction_id, length, frame[length - 1U]);
    (void)uart_write_bytes(UART_NUM_1, frame, length);

    const TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(200);
    while (xTaskGetTickCount() < deadline) {
        const TickType_t remaining = deadline - xTaskGetTickCount();
        if (xQueueReceive(s_l431_ack_queue, &ack, remaining) != pdPASS) break;
        if (ack.command == command && ack.transaction_id == transaction_id) {
            ESP_LOGI(TAG, "L431 ACK cmd=0x%02X tx=%" PRIu32 " result=%u",
                     command, transaction_id, ack.result);
            return (ack.result <= 2U) ? ack.result : 2U;
        }
    }
    ESP_LOGW(TAG, "L431 ACK timeout cmd=0x%02X tx=%" PRIu32, command, transaction_id);
    return 2U;
}

static void downlink_command_task(void *arg)
{
    downlink_command_t work;
    const int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    (void)arg;
    if (sock < 0) {
        ESP_LOGE(TAG, "Cannot create UDP command-ACK socket");
        vTaskDelete(NULL);
        return;
    }
    while (true) {
        if (xQueueReceive(s_downlink_queue, &work, portMAX_DELAY) != pdPASS) continue;
        uint8_t result;
        if (!find_completed_downlink(work.frame_type, work.transaction_id, &result)) {
            result = forward_l431_command(work.l431_command, work.transaction_id, work.hp);
            if (result != 2U)
                remember_completed_downlink(work.frame_type, work.transaction_id, result);
        }
        send_server_ack(sock, &work.peer, work.frame_type, work.transaction_id, result);
    }
}

static void dispatch_l431_command(int sock, const struct sockaddr_in *peer,
                                  uint8_t frame_type, uint8_t l431_command,
                                  uint32_t transaction_id, uint16_t hp)
{
    uint8_t result;
    if (find_completed_downlink(frame_type, transaction_id, &result)) {
        send_server_ack(sock, peer, frame_type, transaction_id, result);
        return;
    }
    const downlink_command_t work = {
        .frame_type = frame_type,
        .l431_command = l431_command,
        .hp = hp,
        .transaction_id = transaction_id,
        .peer = *peer,
    };
    if (xQueueSend(s_downlink_queue, &work, 0) != pdPASS) {
        ESP_LOGW(TAG, "Downlink command queue full; frame 0x%02X", frame_type);
        send_server_ack(sock, peer, frame_type, transaction_id, 2U);
    }
}

static void send_server_ack(int sock, const struct sockaddr_in *peer,
                            uint8_t acked_type, uint32_t transaction_id,
                            uint8_t result)
{
    const robot_ack_v2_frame_t ack = {
        .magic = ROBOT_PROTOCOL_MAGIC,
        .version = ROBOT_PROTOCOL_VERSION,
        .frame_type = ROBOT_FRAME_ACK,
        .acked_frame_type = acked_type,
        .transaction_id = transaction_id,
        .result = result,
    };
    send_frame(sock, peer, &ack, sizeof(ack));
}

static void send_current_status(int sock, const struct sockaddr_in *server)
{
    robot_state_t state;
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    state = s_state;
    xSemaphoreGive(s_status_mutex);
    const robot_status_v2_frame_t status = {
        .magic = ROBOT_PROTOCOL_MAGIC,
        .version = ROBOT_PROTOCOL_VERSION,
        .frame_type = ROBOT_FRAME_STATUS,
        .robot_id = robot_id_snapshot(),
        .hp = state.hp,
        .heat = state.heat,
        .power = state.power,
        .alive = state.alive ? 1U : 0U,
        .shoot_enabled = state.shoot_enabled ? 1U : 0U,
        .power_on = state.power_on ? 1U : 0U,
    };
    send_frame(sock, server, &status, sizeof(status));
}

static bool send_device_announce(int sock, const struct sockaddr_in *server)
{
    robot_device_announce_v2_frame_t announce = {
        .magic = ROBOT_PROTOCOL_MAGIC,
        .version = ROBOT_PROTOCOL_VERSION,
        .frame_type = ROBOT_FRAME_DEVICE_ANNOUNCE,
    };
    xSemaphoreTake(s_identity_mutex, portMAX_DELAY);
    announce.robot_id = s_robot_id;
    if (s_controller_assigned) {
        memcpy(announce.controller_mac, s_controller_mac,
               sizeof(announce.controller_mac));
    }
    xSemaphoreGive(s_identity_mutex);
    memcpy(announce.device_mac, s_device_mac, sizeof(announce.device_mac));
    announce.event_queue_drops = s_event_queue_drops;
    announce.udp_send_failures = s_udp_send_failures;
    return send_frame(sock, server, &announce, sizeof(announce));
}


static uint8_t save_assignment(uint8_t robot_id, const uint8_t mac[6])
{
    nvs_handle_t handle;
    bool all_zero = true;
    for (uint8_t index = 0U; index < 6U; ++index)
        if (mac[index] != 0U) all_zero = false;
    if (robot_id == 0U || all_zero) return 2U;
    if (nvs_open("robot", NVS_READWRITE, &handle) != ESP_OK) return 2U;
    esp_err_t error = nvs_set_u8(handle, "robot_id", robot_id);
    if (error == ESP_OK) error = nvs_set_blob(handle, "xbox_mac", mac, 6U);
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK) return 2U;
    if (!xbox_ble_set_target_address(mac)) return 2U;
    xSemaphoreTake(s_identity_mutex, portMAX_DELAY);
    s_robot_id = robot_id;
    memcpy(s_controller_mac, mac, sizeof(s_controller_mac));
    s_controller_assigned = true;
    s_device_announce_pending = true;
    xSemaphoreGive(s_identity_mutex);
    return 0U;
}

static bool find_completed_downlink(uint8_t frame_type, uint32_t transaction_id,
                                    uint8_t *result)
{
    xSemaphoreTake(s_completed_mutex, portMAX_DELAY);
    for (uint8_t index = 0; index < COMPLETED_DOWNLINK_CAPACITY; ++index) {
        const completed_downlink_t *entry = &s_completed_downlinks[index];
        if (entry->frame_type == frame_type && entry->transaction_id == transaction_id) {
            *result = entry->result;
            xSemaphoreGive(s_completed_mutex);
            return true;
        }
    }
    xSemaphoreGive(s_completed_mutex);
    return false;
}

static void remember_completed_downlink(uint8_t frame_type, uint32_t transaction_id,
                                        uint8_t result)
{
    xSemaphoreTake(s_completed_mutex, portMAX_DELAY);
    const uint8_t index = s_completed_downlink_next;
    s_completed_downlinks[index] = (completed_downlink_t){frame_type, transaction_id, result};
    s_completed_downlink_next = (uint8_t)((index + 1U) % COMPLETED_DOWNLINK_CAPACITY);
    nvs_handle_t handle;
    if (nvs_open("robot", NVS_READWRITE, &handle) == ESP_OK) {
        (void)nvs_set_blob(handle, "down_ack", s_completed_downlinks,
                           sizeof(s_completed_downlinks));
        (void)nvs_commit(handle);
        nvs_close(handle);
    }
    xSemaphoreGive(s_completed_mutex);
}

static void load_assignment(void)
{
    nvs_handle_t handle;
    uint8_t stored_robot_id;
    uint8_t mac[6];
    size_t mac_length = sizeof(mac);
    if (nvs_open("robot", NVS_READONLY, &handle) != ESP_OK) return;
    if (nvs_get_u8(handle, "robot_id", &stored_robot_id) == ESP_OK &&
        nvs_get_blob(handle, "xbox_mac", mac, &mac_length) == ESP_OK && mac_length == sizeof(mac))
    {
        (void)xbox_ble_set_target_address(mac);
        s_robot_id = stored_robot_id;
        memcpy(s_controller_mac, mac, sizeof(s_controller_mac));
        s_controller_assigned = true;
    }
    size_t pending_length = sizeof(s_reliable_events);
    if (nvs_get_blob(handle, "pending", s_reliable_events, &pending_length) == ESP_OK &&
        pending_length == sizeof(s_reliable_events)) {
        for (uint8_t index = 0; index < RELIABLE_EVENT_CAPACITY; ++index) {
            s_reliable_events[index].attempts = 0U;
            s_reliable_events[index].next_send_tick = 0;
        }
    } else {
        memset(s_reliable_events, 0, sizeof(s_reliable_events));
    }
    size_t completed_length = sizeof(s_completed_downlinks);
    if (nvs_get_blob(handle, "down_ack", s_completed_downlinks, &completed_length) != ESP_OK ||
        completed_length != sizeof(s_completed_downlinks))
        memset(s_completed_downlinks, 0, sizeof(s_completed_downlinks));
    s_completed_downlink_next = 0U;
    nvs_close(handle);
}

static void handle_server_datagram(int sock, const uint8_t *data, size_t length,
                                   const struct sockaddr_in *peer)
{
    const in_addr_t expected_server_ip = inet_addr(USE_INTEGRATION_TEST_NETWORK ?
                                                   TEST_SERVER_IP : CONFIG_ROBOT_SERVER_IP);
    if (peer->sin_addr.s_addr != expected_server_ip ||
        peer->sin_port != htons(CONFIG_ROBOT_SERVER_PORT)) return;
    if (length < 4U || data[0] != 0x54U || data[1] != 0x52U ||
        data[2] != ROBOT_PROTOCOL_VERSION) return;

    if (data[3] == ROBOT_FRAME_ACK) {
        robot_network_on_server_datagram(data, length);
        return;
    }

    if (data[3] == ROBOT_FRAME_STATUS_REQUEST && length == sizeof(robot_status_request_v2_frame_t)) {
        const robot_status_request_v2_frame_t *request = (const void *)data;
        if (downlink_targets_this_robot(request->target_robot_id) &&
            (s_l431_status_seen || CONFIG_ROBOT_TEST_MODE)) send_current_status(sock, peer);
        return;
    }

    if (data[3] == ROBOT_FRAME_ASSIGNMENT && length == sizeof(robot_assignment_v2_frame_t)) {
        const robot_assignment_v2_frame_t *assignment = (const void *)data;
        uint8_t result;
        if (memcmp(assignment->target_device_mac, s_device_mac,
                   sizeof(s_device_mac)) != 0) return;
        if (!find_completed_downlink(assignment->frame_type, assignment->transaction_id, &result)) {
            result = save_assignment(assignment->robot_id, assignment->controller_mac);
            if (result != 2U) remember_completed_downlink(assignment->frame_type, assignment->transaction_id, result);
        }
        send_server_ack(sock, peer, assignment->frame_type, assignment->transaction_id, result);
        return;
    }

    if ((data[3] == ROBOT_FRAME_GAME_START || data[3] == ROBOT_FRAME_GAME_END) &&
        length == sizeof(robot_game_control_v2_frame_t)) {
        const robot_game_control_v2_frame_t *command = (const void *)data;
        if (!downlink_targets_this_robot(command->target_robot_id)) return;
        const uint8_t l431_command = (command->frame_type == ROBOT_FRAME_GAME_START) ? 0xC1U : 0xC2U;
        dispatch_l431_command(sock, peer, command->frame_type, l431_command,
                              command->transaction_id, 0U);
        return;
    }

    if (data[3] == ROBOT_FRAME_YELLOW_CARD &&
        length == sizeof(robot_yellow_card_v2_frame_t)) {
        const robot_yellow_card_v2_frame_t *command = (const void *)data;
        /* A penalty is always for one explicitly identified robot. */
        if (command->target_robot_id == 0U ||
            !downlink_targets_this_robot(command->target_robot_id)) return;
        dispatch_l431_command(sock, peer, command->frame_type, 0xC4U,
                              command->transaction_id, 0U);
        return;
    }

    if (data[3] == ROBOT_FRAME_FORCE_POWER_OFF &&
        length == sizeof(robot_force_power_off_v2_frame_t)) {
        const robot_force_power_off_v2_frame_t *command = (const void *)data;
        if (command->target_robot_id == 0U ||
            !downlink_targets_this_robot(command->target_robot_id)) return;
        dispatch_l431_command(sock, peer, command->frame_type, 0xC5U,
                              command->transaction_id, 0U);
        return;
    }

    if (data[3] == ROBOT_FRAME_FORCE_POWER_ON &&
        length == sizeof(robot_force_power_on_v2_frame_t)) {
        const robot_force_power_on_v2_frame_t *command = (const void *)data;
        if (command->target_robot_id == 0U ||
            !downlink_targets_this_robot(command->target_robot_id)) return;
        dispatch_l431_command(sock, peer, command->frame_type, 0xC6U,
                              command->transaction_id, 0U);
        return;
    }

    if (data[3] == ROBOT_FRAME_SET_HP && length == sizeof(robot_set_hp_v2_frame_t)) {
        const robot_set_hp_v2_frame_t *command = (const void *)data;
        if (!downlink_targets_this_robot(command->target_robot_id) || command->hp > 300U) return;
        dispatch_l431_command(sock, peer, command->frame_type, 0xC3U,
                              command->transaction_id, command->hp);
    }
}

static void service_reliable_events(int sock, const struct sockaddr_in *server)
{
    const TickType_t now = xTaskGetTickCount();
    xSemaphoreTake(s_reliable_mutex, portMAX_DELAY);
    for (uint8_t index = 0; index < RELIABLE_EVENT_CAPACITY; ++index) {
        reliable_event_t *event = &s_reliable_events[index];
        if (event->persisted.frame_type == 0U || now < event->next_send_tick) continue;
        const robot_reliable_event_v2_frame_t frame = {
            .magic = ROBOT_PROTOCOL_MAGIC,
            .version = ROBOT_PROTOCOL_VERSION,
            .frame_type = event->persisted.frame_type,
            .robot_id = event->persisted.robot_id,
            .transaction_id = event->persisted.transaction_id,
        };
        send_frame(sock, server, &frame, sizeof(frame));
        ++event->attempts;
        event->next_send_tick = now + pdMS_TO_TICKS(
            event->attempts < 3U ? RELIABLE_RETRY_MS : RELIABLE_RETRY_BACKOFF_MS);
        if (event->attempts == 3U)
            ESP_LOGW(TAG, "V2 event 0x%02X tx=%" PRIu32 " unacknowledged; retaining it",
                     event->persisted.frame_type, event->persisted.transaction_id);
    }
    xSemaphoreGive(s_reliable_mutex);
}

static void robot_send_task(void *arg)
{
    struct sockaddr_in server = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_ROBOT_SERVER_PORT),
        .sin_addr.s_addr = inet_addr(USE_INTEGRATION_TEST_NETWORK ?
                                      TEST_SERVER_IP : CONFIG_ROBOT_SERVER_IP),
    };
    int sock = -1;
    TickType_t last_status_tick = xTaskGetTickCount();
    TickType_t last_announce_tick = last_status_tick - pdMS_TO_TICKS(DEVICE_ANNOUNCE_PERIOD_MS);

    while (true) {
        if (!(xEventGroupGetBits(s_events) & WIFI_CONNECTED_BIT)) {
            if (sock >= 0) {
                close(sock);
                sock = -1;
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        /* Recreate the uplink socket after a Wi-Fi reconnect or a failed
         * allocation.  A permanent invalid descriptor used to make uplink
         * silently stop until a reset. */
        if (sock < 0) {
            sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
            if (sock < 0) {
                ESP_LOGW(TAG, "Cannot create UDP uplink socket");
                vTaskDelay(pdMS_TO_TICKS(1000));
                continue;
            }
        }

        if (s_device_announce_pending ||
            (xTaskGetTickCount() - last_announce_tick) >=
            pdMS_TO_TICKS(DEVICE_ANNOUNCE_PERIOD_MS)) {
            if (send_device_announce(sock, &server))
                s_device_announce_pending = false;
            last_announce_tick = xTaskGetTickCount();
        }

        /* Do not make events depend on a current L431 status snapshot.  In
         * particular, LINK_DOWN is produced precisely when that snapshot has
         * timed out.  An identity is still mandatory because a server cannot
         * attribute an event from robot 0. */
        const uint8_t robot_id = robot_id_snapshot();
        if (robot_id != 0U) {
            pending_event_t event;
            while (xQueueReceive(s_event_queue, &event, 0) == pdPASS) {
                switch (event.frame_type) {
                case ROBOT_FRAME_DEATH:
                case ROBOT_FRAME_REVIVE:
                    (void)enqueue_reliable_event(event.frame_type, 0xFFU);
                    break;
                case ROBOT_FRAME_HIT:
                case ROBOT_FRAME_ATTACK:
                case ROBOT_FRAME_COMBAT_END:
                case ROBOT_FRAME_SHOOT_ENABLED:
                case ROBOT_FRAME_SHOOT_DISABLED:
                case ROBOT_FRAME_REFEREE_LINK_DOWN:
                case ROBOT_FRAME_REFEREE_LINK_UP: {
                    robot_event_v2_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                        event.frame_type, robot_id};
                    send_frame(sock, &server, &frame, sizeof(frame));
                    break;
                }
                default:
                    break;
                }
            }
            service_reliable_events(sock, &server);
        }

        /* Only the periodic state frame needs a real L431 snapshot. */
        if (!CONFIG_ROBOT_TEST_MODE && !s_l431_status_seen) {
            vTaskDelayUntil(&last_status_tick, pdMS_TO_TICKS(STATUS_PERIOD_MS));
            continue;
        }

        if (robot_id_snapshot() != 0U) {
            send_current_status(sock, &server);
        }
        vTaskDelayUntil(&last_status_tick, pdMS_TO_TICKS(STATUS_PERIOD_MS));
    }
}

static void robot_receive_task(void *arg)
{
    const struct sockaddr_in listen_addr = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_ROBOT_LISTEN_PORT),
        .sin_addr.s_addr = htonl(INADDR_ANY),
    };

    while (true) {
        if (!(xEventGroupGetBits(s_events) & WIFI_CONNECTED_BIT)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
        if (sock < 0) {
            ESP_LOGW(TAG, "Cannot create UDP downlink socket");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        int reuse = 1;
        struct timeval receive_timeout = {.tv_sec = 1, .tv_usec = 0};
        (void)setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
        (void)setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO,
                         &receive_timeout, sizeof(receive_timeout));
        if (bind(sock, (const struct sockaddr *)&listen_addr, sizeof(listen_addr)) < 0) {
            ESP_LOGW(TAG, "Cannot listen for UDP downlink on port %d",
                     CONFIG_ROBOT_LISTEN_PORT);
            close(sock);
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        ESP_LOGI(TAG, "Listening for server UDP downlink on port %d",
                 CONFIG_ROBOT_LISTEN_PORT);
        while (xEventGroupGetBits(s_events) & WIFI_CONNECTED_BIT) {
            uint8_t data[DOWNLINK_BUFFER_SIZE];
            struct sockaddr_in peer = {0};
            socklen_t peer_length = sizeof(peer);
            const int received = recvfrom(sock, data, sizeof(data), 0,
                                          (struct sockaddr *)&peer, &peer_length);
            const char *server_ip = USE_INTEGRATION_TEST_NETWORK ?
                TEST_SERVER_IP : CONFIG_ROBOT_SERVER_IP;
            if (received > 0 && peer.sin_addr.s_addr == inet_addr(server_ip)) {
                handle_server_datagram(sock, data, (size_t)received, &peer);
            }
        }
        close(sock);
    }
}

static void wifi_event_handler(void *arg, esp_event_base_t base,
                               int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(s_events, WIFI_CONNECTED_BIT);
        ESP_LOGW(TAG, "Wi-Fi disconnected; reconnecting");
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        xEventGroupSetBits(s_events, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "Wi-Fi connected; robot UDP enabled");
    }
}

static void wifi_set_station_config(const char *ssid, const char *password)
{
    wifi_config_t wifi = {0};
    strlcpy((char *)wifi.sta.ssid, ssid, sizeof(wifi.sta.ssid));
    strlcpy((char *)wifi.sta.password, password, sizeof(wifi.sta.password));
    wifi.sta.scan_method = WIFI_FAST_SCAN;
    wifi.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));
}

static void wifi_start(void)
{
    s_events = xEventGroupCreate();
    esp_netif_t *sta = esp_netif_create_default_wifi_sta();
    configASSERT(sta != NULL);
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               wifi_event_handler, NULL));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
#if USE_INTEGRATION_TEST_NETWORK
    /* Real L431 data, but current integration network. */
    wifi_set_station_config(CONFIG_ROBOT_TEST_FALLBACK_WIFI_SSID,
                            CONFIG_ROBOT_TEST_FALLBACK_WIFI_PASSWORD);
#else
    wifi_set_station_config(CONFIG_ROBOT_WIFI_SSID, CONFIG_ROBOT_WIFI_PASSWORD);
#endif
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

static void uart_start(uart_port_t uart_num, int tx_gpio, int rx_gpio, int baud_rate)
{
    const uart_config_t config = {
        .baud_rate = baud_rate,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(uart_num, UART_RX_BUFFER_SIZE, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(uart_num, &config));
    ESP_ERROR_CHECK(uart_set_pin(uart_num, tx_gpio, rx_gpio,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

typedef struct __attribute__((packed)) {
    uint8_t magic0, magic1, sequence, flags;
    int8_t left_x, left_y, right_x, right_y;
    uint8_t left_trigger, right_trigger, dpad, buttons_1, buttons_2, crc8;
} controller_state_frame_t;

static uint8_t crc8_atm(const uint8_t *data, size_t length)
{
    uint8_t crc = 0;
    while (length-- != 0U) {
        crc ^= *data++;
        for (uint8_t bit = 0; bit < 8U; ++bit)
            crc = (crc & 0x80U) ? (uint8_t)((crc << 1U) ^ 0x07U) : (uint8_t)(crc << 1U);
    }
    return crc;
}

static uint16_t read_u16le(const uint8_t *data)
{
    return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

static int8_t xbox_axis_to_i8(uint16_t value)
{
    int32_t scaled = ((int32_t)value - 32768) * 127 / 32767;
    if (scaled > 127) scaled = 127;
    if (scaled < -127) scaled = -127;
    return (scaled > -8 && scaled < 8) ? 0 : (int8_t)scaled;
}

static void controller_uart_task(void *arg)
{
    uint8_t sequence = 0;
    TickType_t last = xTaskGetTickCount();
    (void)arg;
    while (true) {
        uint8_t report[16] = {0}; uint32_t age_ms = UINT32_MAX; bool connected = false;
        const bool valid = xbox_ble_get_latest_report(report, &connected, &age_ms) && age_ms <= 100U;
        controller_state_frame_t frame = {.magic0 = 0xC3U, .magic1 = 0x3CU,
            .sequence = sequence++, .flags = (uint8_t)((connected ? 0x01U : 0U) | (valid ? 0x02U : 0U))};
        if (valid) {
            frame.left_x = xbox_axis_to_i8(read_u16le(&report[0]));
            frame.left_y = xbox_axis_to_i8(read_u16le(&report[2]));
            frame.right_x = xbox_axis_to_i8(read_u16le(&report[4]));
            frame.right_y = xbox_axis_to_i8(read_u16le(&report[6]));
            frame.left_trigger = (uint8_t)(read_u16le(&report[8]) >> 2U);
            frame.right_trigger = (uint8_t)(read_u16le(&report[10]) >> 2U);
            frame.dpad = report[12]; frame.buttons_1 = report[13]; frame.buttons_2 = report[14];
        }
        frame.crc8 = crc8_atm((const uint8_t *)&frame, sizeof(frame) - 1U);
        (void)uart_write_bytes(UART_NUM_0, &frame, sizeof(frame));
        vTaskDelayUntil(&last, pdMS_TO_TICKS(10));
    }
}

static void uart_reservation_start(void)
{
    uart_start(UART_NUM_0, CONTROLLER_UART_TX_GPIO, CONTROLLER_UART_RX_GPIO,
               CONFIG_ROBOT_REFEREE_UART_BAUD);
    uart_start(UART_NUM_1, L431_UART_TX_GPIO, L431_UART_RX_GPIO,
               CONFIG_ROBOT_RESERVED_UART_BAUD);
    ESP_LOGI(TAG, "UART0 controller: TX=%d RX=%d; UART1 L431PM: TX=%d RX=%d",
             CONTROLLER_UART_TX_GPIO, CONTROLLER_UART_RX_GPIO,
             L431_UART_TX_GPIO, L431_UART_RX_GPIO);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    load_assignment();
    ESP_ERROR_CHECK(esp_read_mac(s_device_mac, ESP_MAC_WIFI_STA));

    s_status_mutex = xSemaphoreCreateMutex();
    s_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(pending_event_t));
    s_l431_ack_queue = xQueueCreate(4, sizeof(l431_ack_t));
    s_downlink_queue = xQueueCreate(4, sizeof(downlink_command_t));
    s_reliable_mutex = xSemaphoreCreateMutex();
    s_completed_mutex = xSemaphoreCreateMutex();
    s_identity_mutex = xSemaphoreCreateMutex();
    configASSERT(s_status_mutex != NULL && s_event_queue != NULL &&
                 s_l431_ack_queue != NULL && s_downlink_queue != NULL &&
                 s_reliable_mutex != NULL && s_completed_mutex != NULL &&
                 s_identity_mutex != NULL);

    uart_reservation_start();
    l431_link_init(&(l431_link_callbacks_t){
        .on_status = l431_on_status,
        .on_event = l431_on_event,
        .on_ack = l431_on_ack,
    });
#if !CONFIG_ROBOT_TEST_MODE
    xTaskCreate(l431_uart_task, "l431_uart", 3072, NULL, 5, NULL);
    xTaskCreate(l431_link_monitor_task, "l431_link_mon", 2048, NULL, 4, NULL);
#endif
    xbox_ble_start();
    xTaskCreate(controller_uart_task, "controller_uart", 2048, NULL, 5, NULL);
    wifi_start();
    xTaskCreate(robot_send_task, "robot_udp", 3072, NULL, 4, NULL);
    xTaskCreate(robot_receive_task, "robot_udp_rx", 3072, NULL, 4, NULL);
    xTaskCreate(downlink_command_task, "robot_downlink", 3072, NULL, 4, NULL);
#if CONFIG_ROBOT_TEST_MODE
    xTaskCreate(robot_test_task, "robot_test", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "Simulated match-data test mode enabled");
#endif
}

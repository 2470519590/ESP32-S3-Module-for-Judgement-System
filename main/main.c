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
#define TEST_SERVER_IP "10.153.217.216"
/* Current formal-business integration uses the temporary test network.
 * This selects only Wi-Fi/server endpoints; simulated data remains disabled. */
#define USE_INTEGRATION_TEST_NETWORK 1

#define WIFI_CONNECTED_BIT BIT0
#define STATUS_PERIOD_MS 100U
#define EVENT_QUEUE_LENGTH 16U
#define DOWNLINK_BUFFER_SIZE 64U
#define RELIABLE_EVENT_CAPACITY 4U
#define RELIABLE_RETRY_MS 100U
#define RELIABLE_RETRY_BACKOFF_MS 1000U

/* UART0 is the controller-facing port; UART1 is the L431PM link. */
#define CONTROLLER_UART_TX_GPIO 43
#define CONTROLLER_UART_RX_GPIO 44
#define L431_UART_TX_GPIO 17
#define L431_UART_RX_GPIO 18
#define UART_RX_BUFFER_SIZE 512

/* Replace these with each robot's factory/programmed identity. */
#define LOCAL_ROBOT_ID 1U

static const char *TAG = "robot_net";
static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_status_mutex;
static QueueHandle_t s_event_queue;
static QueueHandle_t s_l431_ack_queue;
static SemaphoreHandle_t s_reliable_mutex;
static volatile bool s_l431_status_seen;
static volatile TickType_t s_l431_last_status_tick;
static volatile uint8_t s_l431_valid_status_count;
static volatile bool s_l431_link_up;
/* The current real-data integration has one robot and uses ID 1 so UART
 * testing can start without a server assignment. Production Wi-Fi keeps the
 * previous rule: ID comes from the persisted server assignment. */
static uint8_t s_robot_id = (CONFIG_ROBOT_TEST_MODE || USE_INTEGRATION_TEST_NETWORK) ?
                            LOCAL_ROBOT_ID : 0U;

typedef struct {
    uint16_t hp;
    uint16_t heat;
    uint16_t power;
    bool alive;
    bool shoot_enabled;
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

typedef struct __attribute__((packed)) {
    uint8_t frame_type;
    uint8_t robot_id;
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

static completed_downlink_t s_completed_downlinks[4];

static bool queue_event(robot_frame_type_t frame_type, uint16_t hp);

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
    xSemaphoreGive(s_status_mutex);
    s_l431_status_seen = true;
    s_l431_last_status_tick = xTaskGetTickCount();
    if (s_l431_valid_status_count < 3U) ++s_l431_valid_status_count;
    if (!s_l431_link_up && s_l431_valid_status_count >= 3U) {
        s_l431_link_up = true;
        (void)queue_event(ROBOT_FRAME_REFEREE_LINK_UP, 0U);
    }
}

static void l431_on_event(l431_event_t event, uint8_t sequence, void *context)
{
    (void)sequence;
    (void)context;
    switch (event) {
    case L431_EVENT_DEATH: (void)robot_network_publish_death(); break;
    case L431_EVENT_REVIVE: (void)robot_network_publish_revive(); break;
    case L431_EVENT_HIT: (void)robot_network_publish_hit(0); break;
    case L431_EVENT_ATTACK: (void)robot_network_publish_attack(); break;
    case L431_EVENT_SHOOT_ENABLED: (void)robot_network_publish_shoot_enabled(); break;
    case L431_EVENT_SHOOT_DISABLED: (void)robot_network_publish_shoot_disabled(); break;
    default: break;
    }
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
    return xQueueSend(s_event_queue, &event, 0) == pdPASS;
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

static bool enqueue_reliable_event(robot_frame_type_t frame_type)
{
    bool queued = false;
    xSemaphoreTake(s_reliable_mutex, portMAX_DELAY);
    for (uint8_t index = 0; index < RELIABLE_EVENT_CAPACITY; ++index) {
        reliable_event_t *event = &s_reliable_events[index];
        if (event->persisted.frame_type == 0U) {
            event->persisted.frame_type = (uint8_t)frame_type;
            event->persisted.robot_id = s_robot_id;
            event->persisted.transaction_id = esp_random();
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

static void send_frame(int sock, const struct sockaddr_in *server,
                       const void *frame, size_t frame_size)
{
    (void)sendto(sock, frame, frame_size, 0,
                 (const struct sockaddr *)server, sizeof(*server));
}

static bool downlink_targets_this_robot(uint8_t target_robot_id)
{
    return target_robot_id == 0U || target_robot_id == s_robot_id;
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
        .robot_id = s_robot_id,
        .hp = state.hp,
        .heat = state.heat,
        .power = state.power,
        .alive = state.alive ? 1U : 0U,
        .shoot_enabled = state.shoot_enabled ? 1U : 0U,
    };
    send_frame(sock, server, &status, sizeof(status));
}


static uint8_t save_assignment(uint8_t robot_id, const uint8_t mac[6])
{
    nvs_handle_t handle;
    if (robot_id == 0U || !xbox_ble_set_target_address(mac)) return 2U;
    if (nvs_open("robot", NVS_READWRITE, &handle) != ESP_OK) return 2U;
    esp_err_t error = nvs_set_u8(handle, "robot_id", robot_id);
    if (error == ESP_OK) error = nvs_set_blob(handle, "xbox_mac", mac, 6U);
    if (error == ESP_OK) error = nvs_commit(handle);
    nvs_close(handle);
    if (error != ESP_OK) return 2U;
    s_robot_id = robot_id;
    return 0U;
}

static bool find_completed_downlink(uint8_t frame_type, uint32_t transaction_id,
                                    uint8_t *result)
{
    for (uint8_t index = 0; index < 4U; ++index) {
        const completed_downlink_t *entry = &s_completed_downlinks[index];
        if (entry->frame_type == frame_type && entry->transaction_id == transaction_id) {
            *result = entry->result;
            return true;
        }
    }
    return false;
}

static void remember_completed_downlink(uint8_t frame_type, uint32_t transaction_id,
                                        uint8_t result)
{
    const uint8_t index = (uint8_t)(transaction_id % 4U);
    s_completed_downlinks[index] = (completed_downlink_t){frame_type, transaction_id, result};
    nvs_handle_t handle;
    if (nvs_open("robot", NVS_READWRITE, &handle) == ESP_OK) {
        (void)nvs_set_blob(handle, "down_ack", s_completed_downlinks,
                           sizeof(s_completed_downlinks));
        (void)nvs_commit(handle);
        nvs_close(handle);
    }
}

static void load_assignment(void)
{
    nvs_handle_t handle;
    uint8_t mac[6];
    size_t mac_length = sizeof(mac);
    if (nvs_open("robot", NVS_READONLY, &handle) != ESP_OK) return;
    if (nvs_get_u8(handle, "robot_id", &s_robot_id) == ESP_OK &&
        nvs_get_blob(handle, "xbox_mac", mac, &mac_length) == ESP_OK && mac_length == sizeof(mac))
        (void)xbox_ble_set_target_address(mac);
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
    nvs_close(handle);
}

static void handle_server_datagram(int sock, const uint8_t *data, size_t length,
                                   const struct sockaddr_in *peer)
{
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
        uint8_t result;
        if (!find_completed_downlink(command->frame_type, command->transaction_id, &result)) {
            result = forward_l431_command(l431_command, command->transaction_id, 0U);
            if (result != 2U) remember_completed_downlink(command->frame_type, command->transaction_id, result);
        }
        send_server_ack(sock, peer, command->frame_type, command->transaction_id, result);
        return;
    }

    if (data[3] == ROBOT_FRAME_SET_HP && length == sizeof(robot_set_hp_v2_frame_t)) {
        const robot_set_hp_v2_frame_t *command = (const void *)data;
        if (!downlink_targets_this_robot(command->target_robot_id) || command->hp > 300U) return;
        uint8_t result;
        if (!find_completed_downlink(command->frame_type, command->transaction_id, &result)) {
            result = forward_l431_command(0xC3U, command->transaction_id, command->hp);
            if (result != 2U) remember_completed_downlink(command->frame_type, command->transaction_id, result);
        }
        send_server_ack(sock, peer, command->frame_type, command->transaction_id, result);
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
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    TickType_t last_status_tick = xTaskGetTickCount();

    while (true) {
        if (!(xEventGroupGetBits(s_events) & WIFI_CONNECTED_BIT)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }
        if (!CONFIG_ROBOT_TEST_MODE && (!s_l431_status_seen || s_robot_id == 0U)) {
            vTaskDelayUntil(&last_status_tick, pdMS_TO_TICKS(STATUS_PERIOD_MS));
            continue;
        }

        robot_state_t state;
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        state = s_state;
        xSemaphoreGive(s_status_mutex);

        robot_status_v2_frame_t status = {
            .magic = ROBOT_PROTOCOL_MAGIC,
            .version = ROBOT_PROTOCOL_VERSION,
            .frame_type = ROBOT_FRAME_STATUS,
            .robot_id = s_robot_id,
            .hp = state.hp,
            .heat = state.heat,
            .power = state.power,
            .alive = state.alive ? 1U : 0U,
            .shoot_enabled = state.shoot_enabled ? 1U : 0U,
        };
        send_frame(sock, &server, &status, sizeof(status));
        service_reliable_events(sock, &server);

        pending_event_t event;
        while (xQueueReceive(s_event_queue, &event, 0) == pdPASS) {
            switch (event.frame_type) {
            case ROBOT_FRAME_DEATH: {
                (void)enqueue_reliable_event(ROBOT_FRAME_DEATH);
                break;
            }
            case ROBOT_FRAME_REVIVE: {
                (void)enqueue_reliable_event(ROBOT_FRAME_REVIVE);
                break;
            }
            case ROBOT_FRAME_HIT: {
                robot_event_v2_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                    ROBOT_FRAME_HIT, s_robot_id};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_ATTACK: {
                robot_event_v2_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                    ROBOT_FRAME_ATTACK, s_robot_id};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_SHOOT_ENABLED: {
                robot_event_v2_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                    ROBOT_FRAME_SHOOT_ENABLED, s_robot_id};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_SHOOT_DISABLED: {
                robot_event_v2_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                    ROBOT_FRAME_SHOOT_DISABLED, s_robot_id};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_REFEREE_LINK_DOWN:
            case ROBOT_FRAME_REFEREE_LINK_UP: {
                robot_event_v2_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                    event.frame_type, s_robot_id};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            default:
                break;
            }
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

    s_status_mutex = xSemaphoreCreateMutex();
    s_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(pending_event_t));
    s_l431_ack_queue = xQueueCreate(4, sizeof(l431_ack_t));
    s_reliable_mutex = xSemaphoreCreateMutex();
    configASSERT(s_status_mutex != NULL && s_event_queue != NULL &&
                 s_l431_ack_queue != NULL && s_reliable_mutex != NULL);

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
#if CONFIG_ROBOT_TEST_MODE
    xTaskCreate(robot_test_task, "robot_test", 3072, NULL, 4, NULL);
    ESP_LOGI(TAG, "Simulated match-data test mode enabled");
#endif
}

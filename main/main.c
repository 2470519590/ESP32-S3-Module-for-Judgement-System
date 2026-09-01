#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "robot_protocol.h"

#define WIFI_CONNECTED_BIT BIT0
#define STATUS_PERIOD_MS 100U
#define EVENT_QUEUE_LENGTH 16U

/* Replace these with each robot's factory/programmed identity. */
#define LOCAL_ROBOT_ID 1U
#define LOCAL_ROBOT_TEAM 1U

static const char *TAG = "robot_net";
static EventGroupHandle_t s_events;
static SemaphoreHandle_t s_status_mutex;
static QueueHandle_t s_event_queue;

typedef struct {
    uint16_t hp;
    bool alive;
    bool shoot_enabled;
} robot_state_t;

typedef struct {
    robot_event_type_t type;
    uint8_t subject_robot_id;
    uint16_t value;
    uint16_t event_id;
} pending_event_t;

static robot_state_t s_state = {
    .hp = 200,
    .alive = true,
    .shoot_enabled = true,
};
static uint16_t s_next_event_id;

void robot_network_set_status(uint16_t hp, bool alive, bool shoot_enabled)
{
    xSemaphoreTake(s_status_mutex, portMAX_DELAY);
    s_state.hp = hp;
    s_state.alive = alive;
    s_state.shoot_enabled = shoot_enabled;
    xSemaphoreGive(s_status_mutex);
}

bool robot_network_publish_event(robot_event_type_t event_type,
                                 uint8_t subject_robot_id, uint16_t value)
{
    pending_event_t event = {
        .type = event_type,
        .subject_robot_id = subject_robot_id,
        .value = value,
        .event_id = s_next_event_id++,
    };
    return xQueueSend(s_event_queue, &event, 0) == pdPASS;
}

static void send_frame(int sock, const struct sockaddr_in *server,
                       const void *frame, size_t frame_size)
{
    (void)sendto(sock, frame, frame_size, 0,
                 (const struct sockaddr *)server, sizeof(*server));
}

static void robot_send_task(void *arg)
{
    struct sockaddr_in server = {
        .sin_family = AF_INET,
        .sin_port = htons(CONFIG_ROBOT_SERVER_PORT),
        .sin_addr.s_addr = inet_addr(CONFIG_ROBOT_SERVER_IP),
    };
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    TickType_t last_status_tick = xTaskGetTickCount();

    while (true) {
        if (!(xEventGroupGetBits(s_events) & WIFI_CONNECTED_BIT)) {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        robot_state_t state;
        xSemaphoreTake(s_status_mutex, portMAX_DELAY);
        state = s_state;
        xSemaphoreGive(s_status_mutex);

        robot_status_frame_t status = {
            .magic = ROBOT_PROTOCOL_MAGIC,
            .version = ROBOT_PROTOCOL_VERSION,
            .frame_type = ROBOT_FRAME_STATUS,
            .robot_id = LOCAL_ROBOT_ID,
            .team = LOCAL_ROBOT_TEAM,
            .hp = state.hp,
            .alive = state.alive ? 1U : 0U,
            .shoot_enabled = state.shoot_enabled ? 1U : 0U,
        };
        send_frame(sock, &server, &status, sizeof(status));

        pending_event_t event;
        while (xQueueReceive(s_event_queue, &event, 0) == pdPASS) {
            robot_event_frame_t frame = {
                .magic = ROBOT_PROTOCOL_MAGIC,
                .version = ROBOT_PROTOCOL_VERSION,
                .frame_type = ROBOT_FRAME_EVENT,
                .robot_id = LOCAL_ROBOT_ID,
                .team = LOCAL_ROBOT_TEAM,
                .event_type = event.type,
                .subject_robot_id = event.subject_robot_id,
                .value = event.value,
                .event_id = event.event_id,
            };
            send_frame(sock, &server, &frame, sizeof(frame));
        }
        vTaskDelayUntil(&last_status_tick, pdMS_TO_TICKS(STATUS_PERIOD_MS));
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

    wifi_config_t wifi = {0};
    strlcpy((char *)wifi.sta.ssid, CONFIG_ROBOT_WIFI_SSID, sizeof(wifi.sta.ssid));
    strlcpy((char *)wifi.sta.password, CONFIG_ROBOT_WIFI_PASSWORD, sizeof(wifi.sta.password));
    wifi.sta.scan_method = WIFI_FAST_SCAN;
    wifi.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_status_mutex = xSemaphoreCreateMutex();
    s_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(pending_event_t));
    configASSERT(s_status_mutex != NULL && s_event_queue != NULL);

    wifi_start();
    xTaskCreate(robot_send_task, "robot_udp", 3072, NULL, 4, NULL);
}

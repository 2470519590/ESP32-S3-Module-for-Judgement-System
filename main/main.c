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
#include "driver/uart.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "robot_protocol.h"

#define WIFI_CONNECTED_BIT BIT0
#define STATUS_PERIOD_MS 100U
#define EVENT_QUEUE_LENGTH 16U
#define DOWNLINK_BUFFER_SIZE 64U

/* ESP32-S3 DevKit TX/RX header: GPIO43 = U0TXD, GPIO44 = U0RXD. */
#define REFEREE_UART_TX_GPIO 43
#define REFEREE_UART_RX_GPIO 44
#define RESERVED_UART_TX_GPIO 17
#define RESERVED_UART_RX_GPIO 18
#define UART_RX_BUFFER_SIZE 512

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
    robot_frame_type_t frame_type;
    uint16_t hp;
} pending_event_t;

typedef struct {
    uart_port_t uart_num;
    const char *pong_reply;
} uart_pingpong_context_t;

static robot_state_t s_state = {
    .hp = 200,
    .alive = true,
    .shoot_enabled = true,
};

static const uart_pingpong_context_t s_uart0_pingpong = {
    .uart_num = UART_NUM_0,
    .pong_reply = "u0:pong\r\n",
};
static const uart_pingpong_context_t s_uart1_pingpong = {
    .uart_num = UART_NUM_1,
    .pong_reply = "u1:pong\r\n",
};

void __attribute__((weak)) robot_network_on_server_datagram(
    const uint8_t *data, size_t length)
{
    (void)data;
    (void)length;
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
            switch (event.frame_type) {
            case ROBOT_FRAME_DEATH: {
                robot_death_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                                             ROBOT_FRAME_DEATH, LOCAL_ROBOT_ID, LOCAL_ROBOT_TEAM};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_REVIVE: {
                robot_revive_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                                              ROBOT_FRAME_REVIVE, LOCAL_ROBOT_ID, LOCAL_ROBOT_TEAM};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_HIT: {
                robot_hit_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                                           ROBOT_FRAME_HIT, LOCAL_ROBOT_ID, LOCAL_ROBOT_TEAM, event.hp};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_ATTACK: {
                robot_attack_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                                              ROBOT_FRAME_ATTACK, LOCAL_ROBOT_ID, LOCAL_ROBOT_TEAM};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_SHOOT_ENABLED: {
                robot_shoot_enabled_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                                                     ROBOT_FRAME_SHOOT_ENABLED, LOCAL_ROBOT_ID, LOCAL_ROBOT_TEAM};
                send_frame(sock, &server, &frame, sizeof(frame));
                break;
            }
            case ROBOT_FRAME_SHOOT_DISABLED: {
                robot_shoot_disabled_frame_t frame = {ROBOT_PROTOCOL_MAGIC, ROBOT_PROTOCOL_VERSION,
                                                      ROBOT_FRAME_SHOOT_DISABLED, LOCAL_ROBOT_ID, LOCAL_ROBOT_TEAM};
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
            const int received = recvfrom(sock, data, sizeof(data), 0,
                                          NULL, NULL);
            if (received > 0) {
                robot_network_on_server_datagram(data, (size_t)received);
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

static void uart_pingpong_task(void *arg)
{
    const uart_pingpong_context_t *context = arg;
    const char ping[] = "ping";
    size_t matched = 0;

    while (true) {
        uint8_t byte;
        const int received = uart_read_bytes(context->uart_num, &byte, 1,
                                             pdMS_TO_TICKS(1000));
        if (received != 1) {
            continue;
        }

        if (byte == (uint8_t)ping[matched]) {
            matched++;
            if (matched == sizeof(ping) - 1U) {
                (void)uart_write_bytes(context->uart_num, context->pong_reply,
                                       strlen(context->pong_reply));
                matched = 0;
            }
        } else {
            matched = (byte == (uint8_t)ping[0]) ? 1U : 0U;
        }
    }
}

static void uart_reservation_start(void)
{
    uart_start(UART_NUM_0, REFEREE_UART_TX_GPIO, REFEREE_UART_RX_GPIO,
               CONFIG_ROBOT_REFEREE_UART_BAUD);
    uart_start(UART_NUM_1, RESERVED_UART_TX_GPIO, RESERVED_UART_RX_GPIO,
               CONFIG_ROBOT_RESERVED_UART_BAUD);
    ESP_LOGI(TAG, "UART0 referee: TX=%d RX=%d; UART1 reserved: TX=%d RX=%d",
             REFEREE_UART_TX_GPIO, REFEREE_UART_RX_GPIO,
             RESERVED_UART_TX_GPIO, RESERVED_UART_RX_GPIO);
    xTaskCreate(uart_pingpong_task, "uart0_ping", 2048,
                (void *)&s_uart0_pingpong, 4, NULL);
    xTaskCreate(uart_pingpong_task, "uart1_ping", 2048,
                (void *)&s_uart1_pingpong, 4, NULL);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    s_status_mutex = xSemaphoreCreateMutex();
    s_event_queue = xQueueCreate(EVENT_QUEUE_LENGTH, sizeof(pending_event_t));
    configASSERT(s_status_mutex != NULL && s_event_queue != NULL);

    uart_reservation_start();
    wifi_start();
    xTaskCreate(robot_send_task, "robot_udp", 3072, NULL, 4, NULL);
    xTaskCreate(robot_receive_task, "robot_udp_rx", 3072, NULL, 4, NULL);
}

#include "xbox_ble.h"

#include <stdbool.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "os/os_mbuf.h"
#include "store/config/ble_store_config.h"

/* ESP-IDF's NimBLE store helper is implemented by the bt component, but its
 * public header intentionally does not declare this application entry point.
 * This matches the official blecent example. */
void ble_store_config_init(void);

#define XBOX_HID_SERVICE_UUID 0x1812U
#define BLE_CCCD_UUID 0x2902U
#define XBOX_REPORT_SIZE 16U
#define XBOX_NOTIFY_CHARACTERISTIC_MAX 8U
#define XBOX_UART_TRACE 0

static const char *TAG = "xbox_ble";

/* Native NimBLE stores BLE addresses least-significant byte first.
 * address.txt: 8d:23:ab:a5:3c:c9
 */
static uint8_t s_xbox_address[6] = {0xc9, 0x3c, 0xa5, 0xab, 0x23, 0x8d};

static uint8_t s_own_addr_type;
static uint16_t s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
static uint16_t s_hid_service_start;
static uint16_t s_hid_service_end;
typedef struct {
    uint16_t value_handle;
    uint16_t descriptor_end_handle;
    uint16_t cccd_handle;
} xbox_notify_characteristic_t;
static xbox_notify_characteristic_t s_notify_characteristics[XBOX_NOTIFY_CHARACTERISTIC_MAX];
static uint8_t s_notify_characteristic_count;
static uint8_t s_notify_characteristic_index;
static uint8_t s_report[XBOX_REPORT_SIZE];
static bool s_have_report;
static int64_t s_report_time_us;
static portMUX_TYPE s_report_lock = portMUX_INITIALIZER_UNLOCKED;

static int xbox_gap_event(struct ble_gap_event *event, void *arg);
static void xbox_subscribe_next_characteristic(void);

/* Temporary bring-up trace on UART0.  Binary receiver can always resync at
 * AA 55; these short lines make BLE pairing / HID discovery observable. */
static void xbox_trace(const char *format, ...)
{
#if XBOX_UART_TRACE
    char line[96];
    va_list args;
    va_start(args, format);
    const int length = vsnprintf(line, sizeof(line), format, args);
    va_end(args);
    if (length > 0) {
        const size_t count = (size_t)length < sizeof(line) ? (size_t)length : sizeof(line) - 1U;
        (void)uart_write_bytes(s_uart_a, line, count);
        (void)uart_write_bytes(s_uart_a, "\r\n", 2);
    }
#else
    (void)format;
#endif
}

static void xbox_scan(void)
{
    const struct ble_gap_disc_params params = {
        .itvl = 0x0060,
        .window = 0x0060,
        .filter_policy = 0,
        .limited = 0,
        .passive = 0,
        .filter_duplicates = 0,
    };
    const int rc = ble_gap_disc(s_own_addr_type, BLE_HS_FOREVER, &params,
                                xbox_gap_event, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "BLE scan failed: rc=%d", rc);
        xbox_trace("xbox:scan-fail %d", rc);
    } else if (rc == 0) {
        xbox_trace("xbox:scanning");
    }
}

static int xbox_cccd_written(uint16_t conn_handle,
                             const struct ble_gatt_error *error,
                             struct ble_gatt_attr *attr, void *arg)
{
    (void)conn_handle;
    (void)attr;
    (void)arg;
    if (error->status == 0) {
        ESP_LOGI(TAG, "Xbox HID notifications enabled");
        xbox_trace("xbox:subscribed %u", s_notify_characteristics[s_notify_characteristic_index].value_handle);
    } else {
        ESP_LOGW(TAG, "Xbox notification subscription failed: %d", error->status);
        xbox_trace("xbox:subscribe-fail %d", error->status);
    }
    s_notify_characteristic_index++;
    xbox_subscribe_next_characteristic();
    return 0;
}

static int xbox_dsc_discovered(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               uint16_t chr_val_handle,
                               const struct ble_gatt_dsc *dsc, void *arg)
{
    (void)chr_val_handle;
    (void)arg;
    if (error->status == 0) {
        if (ble_uuid_u16(&dsc->uuid.u) == BLE_CCCD_UUID) {
            s_notify_characteristics[s_notify_characteristic_index].cccd_handle = dsc->handle;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE ||
        s_notify_characteristics[s_notify_characteristic_index].cccd_handle == 0) {
        ESP_LOGW(TAG, "Xbox CCCD discovery failed: %d", error->status);
        xbox_trace("xbox:cccd-fail %d", error->status);
        s_notify_characteristic_index++;
        xbox_subscribe_next_characteristic();
        return 0;
    }

    const uint8_t enable_notification[] = {1, 0};
    const int rc = ble_gattc_write_flat(conn_handle,
                                        s_notify_characteristics[s_notify_characteristic_index].cccd_handle,
                                        enable_notification,
                                        sizeof(enable_notification),
                                        xbox_cccd_written, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "Cannot enable Xbox notifications: %d", rc);
        xbox_trace("xbox:cccd-write-fail %d", rc);
        s_notify_characteristic_index++;
        xbox_subscribe_next_characteristic();
    } else {
        xbox_trace("xbox:cccd %u",
                   s_notify_characteristics[s_notify_characteristic_index].cccd_handle);
    }
    return 0;
}

static int xbox_chr_discovered(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               const struct ble_gatt_chr *chr, void *arg)
{
    (void)arg;
    if (error->status == 0) {
        if (s_notify_characteristic_count > 0) {
            s_notify_characteristics[s_notify_characteristic_count - 1U].descriptor_end_handle =
                chr->def_handle - 1U;
        }
        if ((chr->properties & BLE_GATT_CHR_PROP_NOTIFY) &&
            s_notify_characteristic_count < XBOX_NOTIFY_CHARACTERISTIC_MAX) {
            xbox_notify_characteristic_t *entry =
                &s_notify_characteristics[s_notify_characteristic_count++];
            entry->value_handle = chr->val_handle;
            entry->descriptor_end_handle = s_hid_service_end;
            entry->cccd_handle = 0;
        }
        return 0;
    }
    if (error->status != BLE_HS_EDONE || s_notify_characteristic_count == 0) {
        ESP_LOGW(TAG, "Xbox HID notification characteristic not found");
        xbox_trace("xbox:notify-char-missing");
        return 0;
    }
    s_notify_characteristic_index = 0;
    xbox_trace("xbox:notify-char-count %u", s_notify_characteristic_count);
    xbox_subscribe_next_characteristic();
    return 0;
}

static void xbox_subscribe_next_characteristic(void)
{
    while (s_notify_characteristic_index < s_notify_characteristic_count) {
        xbox_notify_characteristic_t *entry =
            &s_notify_characteristics[s_notify_characteristic_index];
        entry->cccd_handle = 0;
        const int rc = ble_gattc_disc_all_dscs(s_conn_handle, entry->value_handle,
                                               entry->descriptor_end_handle,
                                               xbox_dsc_discovered, NULL);
        if (rc == 0) {
            return;
        }
        xbox_trace("xbox:dsc-start-fail %d", rc);
        s_notify_characteristic_index++;
    }
    xbox_trace("xbox:all-subscribed");
}

static int xbox_svc_discovered(uint16_t conn_handle,
                               const struct ble_gatt_error *error,
                               const struct ble_gatt_svc *svc, void *arg)
{
    (void)arg;
    if (error->status == 0) {
        s_hid_service_start = svc->start_handle;
        s_hid_service_end = svc->end_handle;
        return 0;
    }
    if (error->status == BLE_HS_EDONE && s_hid_service_start != 0) {
        const int rc = ble_gattc_disc_all_chrs(conn_handle, s_hid_service_start,
                                               s_hid_service_end,
                                               xbox_chr_discovered, NULL);
        if (rc != 0) {
            ESP_LOGW(TAG, "Xbox characteristic discovery failed: %d", rc);
            xbox_trace("xbox:char-fail %d", rc);
        } else {
            xbox_trace("xbox:hid-service %u-%u", s_hid_service_start, s_hid_service_end);
        }
    } else if (error->status != BLE_HS_EDONE) {
        ESP_LOGW(TAG, "Xbox HID service discovery failed: %d", error->status);
        xbox_trace("xbox:service-fail %d", error->status);
    }
    return 0;
}

static void xbox_discover_hid(uint16_t conn_handle)
{
    const ble_uuid16_t hid_uuid = BLE_UUID16_INIT(XBOX_HID_SERVICE_UUID);
    s_hid_service_start = 0;
    s_hid_service_end = 0;
    s_notify_characteristic_count = 0;
    s_notify_characteristic_index = 0;
    const int rc = ble_gattc_disc_svc_by_uuid(conn_handle, &hid_uuid.u,
                                              xbox_svc_discovered, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "Xbox HID service discovery start failed: %d", rc);
        xbox_trace("xbox:service-start-fail %d", rc);
    }
}

static void xbox_connect(const ble_addr_t *peer_address)
{
    (void)ble_gap_disc_cancel();
    const int rc = ble_gap_connect(s_own_addr_type, peer_address, 30000, NULL,
                                   xbox_gap_event, NULL);
    if (rc != 0) {
        ESP_LOGW(TAG, "Xbox connection start failed: %d", rc);
        xbox_trace("xbox:connect-start-fail %d", rc);
        xbox_scan();
    }
}

static int xbox_gap_event(struct ble_gap_event *event, void *arg)
{
    (void)arg;
    switch (event->type) {
    case BLE_GAP_EVENT_DISC:
        if (memcmp(event->disc.addr.val, s_xbox_address, sizeof(s_xbox_address)) == 0) {
            ESP_LOGI(TAG, "Xbox controller found; connecting");
            xbox_trace("xbox:found");
            xbox_connect(&event->disc.addr);
        }
        return 0;

    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status != 0) {
            ESP_LOGW(TAG, "Xbox connection failed: %d", event->connect.status);
            xbox_trace("xbox:connect-fail %d", event->connect.status);
            xbox_scan();
            return 0;
        }
        s_conn_handle = event->connect.conn_handle;
        ESP_LOGI(TAG, "Xbox connected; pairing");
        xbox_trace("xbox:connected");
        if (ble_gap_security_initiate(s_conn_handle) != 0) {
            ESP_LOGW(TAG, "Xbox pairing could not start");
            xbox_trace("xbox:pair-start-fail");
            (void)ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        }
        return 0;

    case BLE_GAP_EVENT_ENC_CHANGE:
        if (event->enc_change.status == 0) {
            ESP_LOGI(TAG, "Xbox encrypted; finding HID input report");
            xbox_trace("xbox:encrypted");
            xbox_discover_hid(event->enc_change.conn_handle);
        } else {
            ESP_LOGW(TAG, "Xbox pairing/encryption failed: %d", event->enc_change.status);
            xbox_trace("xbox:encrypt-fail %d", event->enc_change.status);
        }
        return 0;

    case BLE_GAP_EVENT_NOTIFY_RX: {
        const uint16_t length = OS_MBUF_PKTLEN(event->notify_rx.om);
        if (length == XBOX_REPORT_SIZE) {
            bool first_report;
            portENTER_CRITICAL(&s_report_lock);
            first_report = !s_have_report;
            (void)os_mbuf_copydata(event->notify_rx.om, 0, length, s_report);
            s_have_report = true;
            s_report_time_us = esp_timer_get_time();
            portEXIT_CRITICAL(&s_report_lock);
            if (first_report) {
                xbox_trace("xbox:first-report h=%u", event->notify_rx.attr_handle);
            }
        }
        return 0;
    }

    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGW(TAG, "Xbox disconnected: reason=%d", event->disconnect.reason);
        xbox_trace("xbox:disconnect %d", event->disconnect.reason);
        s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
        s_notify_characteristic_count = 0;
        s_have_report = false;
        xbox_scan();
        return 0;

    case BLE_GAP_EVENT_REPEAT_PAIRING:
        return BLE_GAP_REPEAT_PAIRING_RETRY;

    default:
        return 0;
    }
}

static void xbox_on_sync(void)
{
    const int rc = ble_hs_id_infer_auto(0, &s_own_addr_type);
    if (rc != 0) {
        ESP_LOGE(TAG, "Cannot infer BLE address type: %d", rc);
        xbox_trace("xbox:addr-fail %d", rc);
        return;
    }
    xbox_scan();
}

static void xbox_host_task(void *param)
{
    (void)param;
    nimble_port_run();
    nimble_port_freertos_deinit();
}

bool xbox_ble_get_latest_report(uint8_t report[16], bool *connected,
                                uint32_t *age_ms)
{
    const int64_t now = esp_timer_get_time();
    bool have_report;
    int64_t report_time;
    portENTER_CRITICAL(&s_report_lock);
    have_report = s_have_report;
    report_time = s_report_time_us;
    if (have_report && report != NULL) memcpy(report, s_report, XBOX_REPORT_SIZE);
    if (connected != NULL) *connected = s_conn_handle != BLE_HS_CONN_HANDLE_NONE;
    portEXIT_CRITICAL(&s_report_lock);
    if (age_ms != NULL) *age_ms = (have_report && now >= report_time) ?
        (uint32_t)((now - report_time) / 1000) : UINT32_MAX;
    return have_report;
}

void xbox_ble_start(void)
{
    ESP_ERROR_CHECK(nimble_port_init());
    ble_hs_cfg.sync_cb = xbox_on_sync;
    ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_store_config_init();
    nimble_port_freertos_init(xbox_host_task);
    xbox_trace("xbox:start 8d:23:ab:a5:3c:c9");
    ESP_LOGI(TAG, "Xbox BLE enabled for 8d:23:ab:a5:3c:c9");
}

bool xbox_ble_set_target_address(const uint8_t address[6])
{
    if (address == NULL) return false;
    bool all_zero = true;
    for (uint8_t index = 0; index < 6U; ++index)
        if (address[index] != 0U) all_zero = false;
    if (all_zero) return false;
    for (uint8_t index = 0; index < 6U; ++index)
        s_xbox_address[5U - index] = address[index];
    if (s_conn_handle != BLE_HS_CONN_HANDLE_NONE)
        (void)ble_gap_terminate(s_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
    return true;
}

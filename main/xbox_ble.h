#pragma once

#include <stdbool.h>
#include <stdint.h>

/*
 * Start the BLE central for the Xbox controller in ref/address.txt.
 * Reports are retained for the formal UART0 control-frame task.
 */
void xbox_ble_start(void);
bool xbox_ble_get_latest_report(uint8_t report[16], bool *connected,
                                uint32_t *age_ms);

/* address is in the normal printed order: xx:xx:xx:xx:xx:xx. */
bool xbox_ble_set_target_address(const uint8_t address[6]);

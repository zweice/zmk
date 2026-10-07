/*
 * Status-info behavior: types a one-line status report at the cursor, e.g.
 *
 *   Akku 73% | Dongle | an seit 3h12m | Keep-Awake an, Rest 12 von 30 min
 *
 * Only plain ASCII without ' " ` ~ ^, so nothing collides with the dead keys
 * of the US-International layout.
 *
 * "an seit" counts from power-on / wake-up from deep sleep (deep sleep is a
 * full power-off on the nRF52, so the uptime starts over).
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_status_info

#include <stdarg.h>
#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/battery.h>
#include <zmk/ble.h>
#include <zmk/endpoints.h>
#include <zmk/usb.h>
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_KB_CLOCK)
#include <zmk/kb_clock.h>
#endif
#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_KEEP_AWAKE)
#include <zmk/keep_awake.h>
#endif

#include "../launcher/send_string.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

uint8_t behavior_queue_is_full(void);

static int append(char *buf, size_t size, size_t pos, const char *fmt, ...) {
    if (pos >= size) {
        return pos;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(buf + pos, size - pos, fmt, ap);
    va_end(ap);
    return n < 0 ? pos : MIN(pos + n, size - 1);
}

static void build_status(char *buf, size_t size) {
    size_t p = 0;

    /* battery */
    p = append(buf, size, p, "Akku %u%%", zmk_battery_state_of_charge());
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    if (zmk_usb_is_powered()) {
        p = append(buf, size, p, " (am Kabel)");
    }
#endif

    /* connection */
    switch (zmk_endpoints_selected().transport) {
    case ZMK_TRANSPORT_USB:
        p = append(buf, size, p, " | USB");
        break;
#if IS_ENABLED(CONFIG_ZMK_BLE)
    case ZMK_TRANSPORT_BLE:
        p = append(buf, size, p, " | Bluetooth %d", zmk_ble_active_profile_index() + 1);
        break;
#endif
    case ZMK_TRANSPORT_24G:
        p = append(buf, size, p, " | Dongle");
        break;
    default:
        break;
    }

    /* uptime since power-on / wake-up */
    uint32_t up_min = (uint32_t)(k_uptime_get() / 60000);
    if (up_min < 60) {
        p = append(buf, size, p, " | an seit %u min", up_min);
    } else if (up_min < 48 * 60) {
        p = append(buf, size, p, " | an seit %uh%02um", up_min / 60, up_min % 60);
    } else {
        p = append(buf, size, p, " | an seit %u Tagen", up_min / (24 * 60));
    }

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_KB_CLOCK)
    p = append(buf, size, p, " | Schlaf %s", zmk_clock_light_sleep() ? "leicht" : "tief");
#endif

#if IS_ENABLED(CONFIG_ZMK_BEHAVIOR_KEEP_AWAKE)
    /* keep-awake */
    uint16_t limit = zmk_keep_awake_limit_min();
    if (!zmk_keep_awake_is_active()) {
        if (limit) {
            p = append(buf, size, p, " | Keep-Awake aus (Limit %u min)", limit);
        } else {
            p = append(buf, size, p, " | Keep-Awake aus (ohne Limit)");
        }
    } else {
        int32_t left = zmk_keep_awake_remaining_min();
        if (left >= 0) {
            p = append(buf, size, p, " | Keep-Awake an, Rest %d von %u min", left, limit);
        } else {
            p = append(buf, size, p, " | Keep-Awake an, ohne Limit");
        }
    }
#endif
}

static int behavior_status_info_init(const struct device *dev) { return 0; }

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    if (behavior_queue_is_full()) {
        LOG_WRN("status-info: still typing, ignored");
        return ZMK_BEHAVIOR_OPAQUE;
    }
    char buf[128];
    build_status(buf, sizeof(buf));
    LOG_INF("status-info: %s", buf);
    send_string_with_delay(buf, 0);
    send_string_end();
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_status_info_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

DEVICE_DT_INST_DEFINE(0, behavior_status_info_init, NULL, NULL, NULL, APPLICATION,
                      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_status_info_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

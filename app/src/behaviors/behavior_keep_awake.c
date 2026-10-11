/*
 * Keep-awake behavior.
 *
 *   &keep_awake KA_TOG    start / stop
 *   &keep_awake KA_CYCLE  cycle the time limit: 10 -> 30 -> 60 min -> unlimited
 *
 * While active, taps an otherwise unused key (F24 by default) so the host's
 * idle timer resets and Windows neither locks the screen nor goes to sleep.
 *
 * The tap never mixes with real typing:
 *  - Idle gate: it is only sent after `idle-ms` (30 s) without any physical
 *    key press or release, at most every `interval-ms`. While you type, real
 *    key strokes keep the PC awake anyway, so no tap is needed.
 *  - Clean report: it is skipped while any key or modifier is held, or while
 *    the firmware itself is typing (status line, clock, macros); the 30 s
 *    window then simply starts over.
 *  - Atomic: press and release are sent back to back inside one work item.
 *    Physical key events are processed on the same (single-threaded) system
 *    work queue, so none can be handled in between - F24 can never share a
 *    report with another key, and no real key is ever delayed.
 *
 * Stops on its own after the selected time limit (default 10 min, stored in
 * flash), on USB, Bluetooth and the 2.4G dongle alike. The window starts when
 * keep-awake is switched on and restarts whenever the limit is changed.
 *
 * The tap is injected straight into the HID report instead of raising a
 * keycode event, so caps-word, combos and the activity/sleep logic never see
 * it. Deep sleep (2 h without a real key press, battery only) is blocked
 * while keep-awake runs, so "unlimited" works on battery too.
 *
 * Indicator: the blue (BT) LED stays lit while active. Cycling the time limit
 * blinks it 1-4 times (1 = 10 min, 2 = 30 min, 3 = 60 min, 4 = unlimited).
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_keep_awake

#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <drivers/behavior.h>

#include <zmk/activity.h>
#include <zmk/behavior.h>
#include <zmk/endpoints.h>
#include <zmk/hid.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/keep_awake.h>
#include <zmk/keep_awake.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define KA_KEY DT_INST_PROP(0, keycode)
#define KA_INTERVAL_MS DT_INST_PROP(0, interval_ms)
#define KA_IDLE_MS DT_INST_PROP(0, idle_ms)

/* Wireless time limits in minutes; 0 = unlimited. */
static const uint16_t limits_min[] = {10, 30, 60, 0};
static uint8_t limit_idx; /* persisted */

static bool active;
static int64_t started_at;
static int64_t last_tap_at;
static uint32_t taps; /* since keep-awake was switched on */

bool zmk_behavior_queue_busy(void);

/* implemented in leds.c (blue/BT LED used as indicator) */
void led_keep_awake_set(bool on);
void led_keep_awake_refresh(void);
uint32_t led_keep_awake_blink(uint8_t count);

static void keep_awake_work_handler(struct k_work *work);
static K_WORK_DELAYABLE_DEFINE(keep_awake_work, keep_awake_work_handler);

static void led_restore_handler(struct k_work *work) { led_keep_awake_set(active); }
static K_WORK_DELAYABLE_DEFINE(led_restore_work, led_restore_handler);

#if IS_ENABLED(CONFIG_SETTINGS)
static void save_handler(struct k_work *work) {
    settings_save_one("keep_awake/limit", &limit_idx, sizeof(limit_idx));
}
static K_WORK_DELAYABLE_DEFINE(save_work, save_handler);

static int keep_awake_settings_load_cb(const char *name, size_t len, settings_read_cb read_cb,
                                       void *cb_arg, void *param) {
    const char *next;
    if (settings_name_steq(name, "limit", &next) && !next) {
        uint8_t v;
        if (len != sizeof(v)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &v, sizeof(v));
        if (rc >= 0 && v < ARRAY_SIZE(limits_min)) {
            limit_idx = v;
        }
        return MIN(rc, 0);
    }
    return -ENOENT;
}
#endif

/* ---- read-only state for the status-info behavior (include/zmk/keep_awake.h) ---- */
bool zmk_keep_awake_is_active(void) { return active; }

uint16_t zmk_keep_awake_limit_min(void) { return limits_min[limit_idx]; }

int32_t zmk_keep_awake_remaining_min(void) {
    uint16_t limit = limits_min[limit_idx];
    if (!active || !limit) {
        return -1;
    }
    int64_t left_ms = (int64_t)limit * 60 * 1000 - (k_uptime_get() - started_at);
    return left_ms <= 0 ? 0 : (int32_t)((left_ms + 59999) / 60000);
}

uint32_t zmk_keep_awake_taps(void) { return taps; }

/* nothing held: no key, no modifier (physically or by the firmware) */
static bool keyboard_report_empty(void) {
    const struct zmk_hid_keyboard_report *r = zmk_hid_get_keyboard_report();
    if (r->body.modifiers) {
        return false;
    }
    for (size_t i = 0; i < sizeof(r->body.keys); i++) {
        if (r->body.keys[i]) {
            return false;
        }
    }
    return true;
}

/* press + release back to back; see the header comment for why this is atomic */
static void tap(void) {
    zmk_hid_keyboard_press(KA_KEY);
    zmk_endpoints_send_report(HID_USAGE_KEY);
    zmk_hid_keyboard_release(KA_KEY);
    zmk_endpoints_send_report(HID_USAGE_KEY);
    taps++;
}

static void stop(void) {
    active = false;
    k_work_cancel_delayable(&keep_awake_work);
    k_work_cancel_delayable(&led_restore_work);
    led_keep_awake_set(false);
    zmk_activity_inhibit_sleep(false);
    LOG_INF("keep-awake off");
}

static void keep_awake_work_handler(struct k_work *work) {
    if (!active) {
        return;
    }

    int64_t now = k_uptime_get();
    uint16_t limit = limits_min[limit_idx];
    int64_t limit_ms = (int64_t)limit * 60 * 1000;
    if (limit && now - started_at >= limit_ms) {
        LOG_INF("keep-awake: limit of %u min reached", limit);
        stop();
        return;
    }

    int64_t idle = now - zmk_activity_last_key_ms();
    int64_t since_tap = now - last_tap_at;
    int64_t wait;
    if (idle < KA_IDLE_MS) {
        wait = KA_IDLE_MS - idle; /* still typing: real keys keep the PC awake */
    } else if (since_tap < KA_INTERVAL_MS) {
        wait = KA_INTERVAL_MS - since_tap;
    } else if (!keyboard_report_empty() || zmk_behavior_queue_busy()) {
        /* a key held for > idle-ms (or firmware still typing): start the window over.
         * Releasing the key is a key event, which resets the idle gate anyway. */
        wait = KA_IDLE_MS;
    } else {
        tap();
        last_tap_at = now;
        wait = KA_INTERVAL_MS;
    }

    /* wake up in time to honour the limit */
    if (limit) {
        int64_t left = limit_ms - (now - started_at);
        if (left < wait) {
            wait = left;
        }
    }

    led_keep_awake_refresh();
    k_work_schedule(&keep_awake_work, K_MSEC(MAX(wait, 50)));
}

static void toggle(void) {
    if (active) {
        stop();
        return;
    }
    active = true;
    started_at = k_uptime_get();
    last_tap_at = started_at - KA_INTERVAL_MS; /* only the idle gate counts for the first tap */
    taps = 0;
    LOG_INF("keep-awake on (limit %u min, 0 = unlimited)", limits_min[limit_idx]);
    led_keep_awake_set(true);
    zmk_activity_inhibit_sleep(true);
    k_work_schedule(&keep_awake_work, K_NO_WAIT);
}

static void cycle_limit(void) {
    limit_idx = (limit_idx + 1) % ARRAY_SIZE(limits_min);
    /* a new limit also restarts the running window */
    started_at = k_uptime_get();
    LOG_INF("keep-awake limit: %u min (0 = unlimited)", limits_min[limit_idx]);

    uint32_t ms = led_keep_awake_blink(limit_idx + 1);
    k_work_reschedule(&led_restore_work, K_MSEC(ms + 100));

#if IS_ENABLED(CONFIG_SETTINGS)
    /* debounce flash writes while cycling */
    k_work_reschedule(&save_work, K_SECONDS(5));
#endif
}

static int behavior_keep_awake_init(const struct device *dev) {
#if IS_ENABLED(CONFIG_SETTINGS)
    settings_subsys_init();
    int rc = settings_load_subtree_direct("keep_awake", keep_awake_settings_load_cb, NULL);
    if (rc != 0) {
        LOG_ERR("Failed to load keep-awake settings: %d", rc);
    }
#endif
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case KA_TOG:
        toggle();
        break;
    case KA_CYCLE:
        cycle_limit();
        break;
    default:
        LOG_ERR("keep-awake: unknown command %d", binding->param1);
        return -ENOTSUP;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_keep_awake_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

DEVICE_DT_INST_DEFINE(0, behavior_keep_awake_init, NULL, NULL, NULL, APPLICATION,
                      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_keep_awake_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

/*
 * Keyboard wall clock + sleep-mode switch.
 *
 *   &kb_clock CLK_TYPE   type the time as HH:MM (nothing typed if never set)
 *   &kb_clock CLK_SET    set mode: type 4 digits, e.g. 1934 or 19:34
 *   &kb_clock CLK_SLEEP  toggle deep <-> light sleep
 *
 * The keyboard has no RTC, only the uptime counter (32 kHz RC oscillator,
 * calibrated). The clock is an offset on top of it, so it is lost whenever
 * the counter restarts: power loss, flashing, and deep sleep (System OFF is
 * a full power-down on the nRF52840). In light-sleep mode the keyboard never
 * powers off; it silences the radio instead, and on the first key press it
 * warm-reboots (like waking from deep sleep) carrying the time across the
 * reboot via flash.
 *
 * Set mode: digits, ':' and Backspace are swallowed (never reach the host).
 * Esc cancels, any other key cancels and is passed through. 15 s timeout.
 * The blue LED blinks fast while waiting; 1 blink = set, 4 blinks = invalid.
 *
 * SPDX-License-Identifier: MIT
 */

#define DT_DRV_COMPAT zmk_behavior_kb_clock

#include <stdio.h>
#include <zephyr/device.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <drivers/behavior.h>

#include <zmk/behavior.h>
#include <zmk/event_manager.h>
#include <zmk/events/keycode_state_changed.h>
#include <zmk/kb_clock.h>
#include <zmk/keep_awake.h>
#include <dt-bindings/zmk/hid_usage.h>
#include <dt-bindings/zmk/hid_usage_pages.h>
#include <dt-bindings/zmk/kb_clock.h>

#include "../launcher/send_string.h"

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT)

#define MS_PER_DAY (24LL * 60 * 60 * 1000)
#define SET_TIMEOUT_MS 15000
#define REBOOT_MAGIC 0x4B434C4BU /* "KCLK" */

/* ---- state ---- */
static bool clock_valid;
static int64_t clock_base; /* wall_ms_of_day = (uptime - clock_base) mod MS_PER_DAY */
static uint8_t light_sleep; /* persisted */

static bool setting;
static uint8_t digits[4];
static uint8_t n_digits;
static uint32_t swallowed[8]; /* bitmap of HID usages whose press we ate */

struct reboot_blob {
    uint32_t magic;
    uint32_t ms_of_day;
};

/* implemented in leds.c */
uint32_t led_keep_awake_blink(uint8_t count);
void led_keep_awake_set(bool on);
void led_clock_set_mode(bool on);
uint8_t behavior_queue_is_full(void);

static void led_restore_handler(struct k_work *work) {
    led_keep_awake_set(zmk_keep_awake_is_active());
}
static K_WORK_DELAYABLE_DEFINE(led_restore_work, led_restore_handler);

static void blink(uint8_t n) {
    uint32_t ms = led_keep_awake_blink(n);
    k_work_reschedule(&led_restore_work, K_MSEC(ms + 100));
}

/* ---- public API ---- */
int32_t zmk_clock_minutes_of_day(void) {
    if (!clock_valid) {
        return -1;
    }
    int64_t ms = (k_uptime_get() - clock_base) % MS_PER_DAY;
    if (ms < 0) {
        ms += MS_PER_DAY;
    }
    return (int32_t)(ms / 60000);
}

bool zmk_clock_light_sleep(void) { return light_sleep; }

void zmk_clock_save_for_reboot(void) {
#if IS_ENABLED(CONFIG_SETTINGS)
    if (!clock_valid) {
        return;
    }
    int64_t ms = (k_uptime_get() - clock_base) % MS_PER_DAY;
    if (ms < 0) {
        ms += MS_PER_DAY;
    }
    struct reboot_blob blob = {.magic = REBOOT_MAGIC, .ms_of_day = (uint32_t)ms};
    settings_save_one("kbclock/reboot", &blob, sizeof(blob));
#endif
}

static void set_clock(uint8_t h, uint8_t m) {
    clock_base = k_uptime_get() - ((int64_t)h * 3600 + (int64_t)m * 60) * 1000;
    clock_valid = true;
    LOG_INF("clock set to %02u:%02u", h, m);
}

/* ---- settings ---- */
#if IS_ENABLED(CONFIG_SETTINGS)
static void save_sleep_handler(struct k_work *work) {
    settings_save_one("kbclock/sleep", &light_sleep, sizeof(light_sleep));
}
static K_WORK_DELAYABLE_DEFINE(save_sleep_work, save_sleep_handler);

static void clear_reboot_handler(struct k_work *work) {
    struct reboot_blob blob = {0};
    settings_save_one("kbclock/reboot", &blob, sizeof(blob));
}
static K_WORK_DEFINE(clear_reboot_work, clear_reboot_handler);

static int settings_cb(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg,
                       void *param) {
    const char *next;
    if (settings_name_steq(name, "sleep", &next) && !next) {
        uint8_t v;
        if (len != sizeof(v)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &v, sizeof(v));
        if (rc >= 0) {
            light_sleep = v ? 1 : 0;
        }
        return MIN(rc, 0);
    }
    if (settings_name_steq(name, "reboot", &next) && !next) {
        struct reboot_blob blob;
        if (len != sizeof(blob)) {
            return -EINVAL;
        }
        int rc = read_cb(cb_arg, &blob, sizeof(blob));
        if (rc >= 0 && blob.magic == REBOOT_MAGIC && blob.ms_of_day < MS_PER_DAY) {
            /* time spent rebooting is roughly the uptime so far; counts toward the clock */
            clock_base = -(int64_t)blob.ms_of_day;
            clock_valid = true;
            /* one-shot: a later cold boot must not resurrect a stale time */
            k_work_submit(&clear_reboot_work);
            LOG_INF("clock restored after warm reboot");
        }
        return MIN(rc, 0);
    }
    return -ENOENT;
}
#endif

/* ---- set mode ---- */
static void end_set_mode(void);

static void set_timeout_handler(struct k_work *work) {
    if (setting) {
        LOG_INF("clock set mode timed out");
        end_set_mode();
        k_work_reschedule(&led_restore_work, K_NO_WAIT);
    }
}
static K_WORK_DELAYABLE_DEFINE(set_timeout_work, set_timeout_handler);

static void begin_set_mode(void) {
    setting = true;
    n_digits = 0;
    led_clock_set_mode(true);
    k_work_reschedule(&set_timeout_work, K_MSEC(SET_TIMEOUT_MS));
}

static void end_set_mode(void) {
    setting = false;
    k_work_cancel_delayable(&set_timeout_work);
    led_clock_set_mode(false);
}

static int digit_of(uint32_t usage) {
    if (usage >= HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION &&
        usage <= HID_USAGE_KEY_KEYBOARD_9_AND_LEFT_PARENTHESIS) {
        return usage - HID_USAGE_KEY_KEYBOARD_1_AND_EXCLAMATION + 1;
    }
    if (usage == HID_USAGE_KEY_KEYBOARD_0_AND_RIGHT_PARENTHESIS) {
        return 0;
    }
    if (usage >= HID_USAGE_KEY_KEYPAD_1_AND_END && usage <= HID_USAGE_KEY_KEYPAD_9_AND_PAGEUP) {
        return usage - HID_USAGE_KEY_KEYPAD_1_AND_END + 1;
    }
    if (usage == HID_USAGE_KEY_KEYPAD_0_AND_INSERT) {
        return 0;
    }
    return -1;
}

static bool is_modifier(uint32_t usage) {
    return usage >= HID_USAGE_KEY_KEYBOARD_LEFTCONTROL && usage <= HID_USAGE_KEY_KEYBOARD_RIGHT_GUI;
}

static void mark(uint32_t usage, bool on) {
    if (usage < 256) {
        if (on) {
            swallowed[usage / 32] |= BIT(usage % 32);
        } else {
            swallowed[usage / 32] &= ~BIT(usage % 32);
        }
    }
}

static bool marked(uint32_t usage) {
    return usage < 256 && (swallowed[usage / 32] & BIT(usage % 32));
}

static void finish_digits(void) {
    uint8_t h = digits[0] * 10 + digits[1];
    uint8_t m = digits[2] * 10 + digits[3];
    end_set_mode();
    if (h < 24 && m < 60) {
        set_clock(h, m);
        blink(1);
    } else {
        LOG_WRN("invalid time %02u:%02u", h, m);
        blink(4);
    }
}

static int clock_keycode_listener(const zmk_event_t *eh) {
    const struct zmk_keycode_state_changed *ev = as_zmk_keycode_state_changed(eh);
    if (!ev || ev->usage_page != HID_USAGE_KEY) {
        return ZMK_EV_EVENT_BUBBLE;
    }
    uint32_t usage = ev->keycode;

    if (!ev->state) {
        /* release: eat it only if we ate the matching press */
        if (marked(usage)) {
            mark(usage, false);
            return ZMK_EV_EVENT_HANDLED;
        }
        return ZMK_EV_EVENT_BUBBLE;
    }

    if (!setting || is_modifier(usage)) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    int d = digit_of(usage);
    if (d >= 0) {
        mark(usage, true);
        digits[n_digits++] = d;
        k_work_reschedule(&set_timeout_work, K_MSEC(SET_TIMEOUT_MS));
        if (n_digits == 4) {
            finish_digits();
        }
        return ZMK_EV_EVENT_HANDLED;
    }

    switch (usage) {
    case HID_USAGE_KEY_KEYBOARD_SEMICOLON_AND_COLON: /* the ':' in 19:34 */
        mark(usage, true);
        return ZMK_EV_EVENT_HANDLED;
    case HID_USAGE_KEY_KEYBOARD_DELETE_BACKSPACE:
        mark(usage, true);
        if (n_digits) {
            n_digits--;
        }
        return ZMK_EV_EVENT_HANDLED;
    case HID_USAGE_KEY_KEYBOARD_ESCAPE:
        mark(usage, true);
        end_set_mode();
        k_work_reschedule(&led_restore_work, K_NO_WAIT);
        return ZMK_EV_EVENT_HANDLED;
    default:
        /* anything else: give up and let the key through */
        end_set_mode();
        k_work_reschedule(&led_restore_work, K_NO_WAIT);
        return ZMK_EV_EVENT_BUBBLE;
    }
}

ZMK_LISTENER(kb_clock, clock_keycode_listener);
ZMK_SUBSCRIPTION(kb_clock, zmk_keycode_state_changed);

/* ---- behavior ---- */
static void type_time(void) {
    int32_t mins = zmk_clock_minutes_of_day();
    if (mins < 0) {
        blink(4); /* not set */
        return;
    }
    if (behavior_queue_is_full()) {
        return;
    }
    char buf[8];
    snprintf(buf, sizeof(buf), "%02d:%02d", mins / 60, mins % 60);
    send_string_with_delay(buf, 0);
    send_string_end();
}

static void toggle_sleep_mode(void) {
    light_sleep = !light_sleep;
    LOG_INF("sleep mode: %s", light_sleep ? "light" : "deep");
    blink(light_sleep ? 2 : 1);
#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_reschedule(&save_sleep_work, K_SECONDS(5));
#endif
}

static int behavior_kb_clock_init(const struct device *dev) {
#if IS_ENABLED(CONFIG_SETTINGS)
    settings_subsys_init();
    int rc = settings_load_subtree_direct("kbclock", settings_cb, NULL);
    if (rc != 0) {
        LOG_ERR("Failed to load clock settings: %d", rc);
    }
#endif
    return 0;
}

static int on_keymap_binding_pressed(struct zmk_behavior_binding *binding,
                                     struct zmk_behavior_binding_event event) {
    switch (binding->param1) {
    case CLK_TYPE:
        type_time();
        break;
    case CLK_SET:
        begin_set_mode();
        break;
    case CLK_SLEEP:
        toggle_sleep_mode();
        break;
    default:
        return -ENOTSUP;
    }
    return ZMK_BEHAVIOR_OPAQUE;
}

static int on_keymap_binding_released(struct zmk_behavior_binding *binding,
                                      struct zmk_behavior_binding_event event) {
    return ZMK_BEHAVIOR_OPAQUE;
}

static const struct behavior_driver_api behavior_kb_clock_driver_api = {
    .binding_pressed = on_keymap_binding_pressed,
    .binding_released = on_keymap_binding_released,
};

DEVICE_DT_INST_DEFINE(0, behavior_kb_clock_init, NULL, NULL, NULL, APPLICATION,
                      CONFIG_KERNEL_INIT_PRIORITY_DEFAULT, &behavior_kb_clock_driver_api);

#endif /* DT_HAS_COMPAT_STATUS_OKAY(DT_DRV_COMPAT) */

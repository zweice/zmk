/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/pm.h>

#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#include <zmk/event_manager.h>
#include <zmk/events/activity_state_changed.h>
#include <zmk/events/position_state_changed.h>
#include <zmk/events/sensor_event.h>

#include <zmk/activity.h>
#include <zmk/endpoints.h>
#include <zmk/kb_clock.h>
#include <zephyr/sys/reboot.h>
#if IS_ENABLED(CONFIG_ZMK_BLE)
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/hci.h>
#endif

#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
#include <zmk/usb.h>
#endif
bool all_keys_up(void);
bool is_usb_power_present() {
#if IS_ENABLED(CONFIG_USB_DEVICE_STACK)
    return zmk_usb_is_powered();
#else
    return false;
#endif /* IS_ENABLED(CONFIG_USB_DEVICE_STACK) */
}

static enum zmk_activity_state activity_state;

static uint32_t activity_last_uptime;

/* Set while keep-awake runs so "unlimited" really is unlimited on battery. */
static bool sleep_inhibited;

void zmk_activity_inhibit_sleep(bool inhibit) {
    sleep_inhibited = inhibit;
    if (!inhibit) {
        /* start a fresh sleep countdown instead of sleeping right away */
        activity_last_uptime = k_uptime_get();
    }
}

#define MAX_IDLE_MS CONFIG_ZMK_IDLE_TIMEOUT

#if IS_ENABLED(CONFIG_ZMK_SLEEP)
#define MAX_SLEEP_MS CONFIG_ZMK_IDLE_SLEEP_TIMEOUT
#endif

int raise_event() {
    return ZMK_EVENT_RAISE(new_zmk_activity_state_changed(
        (struct zmk_activity_state_changed){.state = activity_state}));
}

int set_state(enum zmk_activity_state state) {
    if (activity_state == state)
        return 0;

    activity_state = state;
    return raise_event();
}

enum zmk_activity_state zmk_activity_get_state() { return activity_state; }

/*
 * Light sleep (selectable instead of deep sleep, see behavior_kb_clock.c):
 * the chip stays in System ON so the uptime counter - and with it the
 * keyboard clock - keeps running. LEDs and the radio are switched off; the
 * first key press then warm-reboots the keyboard exactly like waking from
 * deep sleep does (the waking key press is consumed, as with deep sleep),
 * and the clock is carried across the reboot.
 */
static bool light_sleeping;

#if IS_ENABLED(CONFIG_ZMK_BLE)
static void disconnect_cb(struct bt_conn *conn, void *data) {
    bt_conn_disconnect(conn, BT_HCI_ERR_REMOTE_USER_TERM_CONN);
}
#endif
#if CONFIG_ZMK_NRF_24G
extern struct k_timer poll_timer; /* Keychron 2.4G lib: drives all ESB traffic */
#endif

static void radio_quiet(void) {
    switch (zmk_endpoints_selected().transport) {
#if CONFIG_ZMK_NRF_24G
    case ZMK_TRANSPORT_24G:
        k_timer_stop(&poll_timer);
        break;
#endif
#if IS_ENABLED(CONFIG_ZMK_BLE)
    case ZMK_TRANSPORT_BLE:
        bt_le_adv_stop();
        bt_conn_foreach(BT_CONN_TYPE_LE, disconnect_cb, NULL);
        break;
#endif
    default:
        break;
    }
}

int activity_event_listener(const zmk_event_t *eh) {
    activity_last_uptime = k_uptime_get();
    struct zmk_position_state_changed *pos_state;
    
    pos_state = as_zmk_position_state_changed(eh);
    if (light_sleeping && pos_state && pos_state->state) {
        /* this listener is linked before keymap/combos, so nothing has been sent yet */
        LOG_INF("waking from light sleep: warm reboot");
        zmk_clock_save_for_reboot();
        sys_reboot(SYS_REBOOT_WARM);
        return ZMK_EV_EVENT_HANDLED;
    }
    if(pos_state)
    {
        if(pos_state->state)
        {
            void bat_low_check(void);
            bat_low_check();
        }
    }
    return set_state(ZMK_ACTIVITY_ACTIVE);
}

void activity_work_handler(struct k_work *work) {
    int32_t current = k_uptime_get();
    int32_t inactive_time = current - activity_last_uptime;
#if IS_ENABLED(CONFIG_ZMK_SLEEP)
    if (!sleep_inhibited && (inactive_time > MAX_SLEEP_MS || activity_state == ZMK_ACTIVITY_SLEEP) && !is_usb_power_present() && all_keys_up()) {
        bool bat_is_shutdown(void);
        if (zmk_clock_light_sleep() && !bat_is_shutdown()) {
            if (!light_sleeping) {
                void leds_turnoff(void);
                leds_turnoff();
                radio_quiet();
                light_sleeping = true;
                LOG_INF("light sleep (clock keeps running)");
                set_state(ZMK_ACTIVITY_SLEEP);
            }
            return;
        }
        // Put devices in suspend power mode before sleeping
        void leds_turnoff(void);
        leds_turnoff();
        void kscan_gpio_direct_enter_sleep(void);
        kscan_gpio_direct_enter_sleep();
        LOG_DBG("ZMK_ACTIVITY_SLEEP");
        set_state(ZMK_ACTIVITY_SLEEP);
        pm_state_force(0U, &(struct pm_state_info){PM_STATE_SOFT_OFF, 0, 0});
    } else
#endif /* IS_ENABLED(CONFIG_ZMK_SLEEP) */
        if (inactive_time > MAX_IDLE_MS) {
            set_state(ZMK_ACTIVITY_IDLE);
        }
}

K_WORK_DEFINE(activity_work, activity_work_handler);

void activity_expiry_function() { k_work_submit(&activity_work); }

K_TIMER_DEFINE(activity_timer, activity_expiry_function, NULL);

int activity_init() {
    activity_last_uptime = k_uptime_get();

    k_timer_start(&activity_timer, K_SECONDS(1), K_SECONDS(1));
    return 0;
}

ZMK_LISTENER(activity, activity_event_listener);
ZMK_SUBSCRIPTION(activity, zmk_position_state_changed);
ZMK_SUBSCRIPTION(activity, zmk_sensor_event);

SYS_INIT(activity_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/*
 * Copyright (c) 2020 The ZMK Contributors
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

enum zmk_activity_state { ZMK_ACTIVITY_ACTIVE, ZMK_ACTIVITY_IDLE, ZMK_ACTIVITY_SLEEP };

enum zmk_activity_state zmk_activity_get_state();
int set_state(enum zmk_activity_state state);

/* Block deep sleep while true (used by the keep-awake behavior). */
void zmk_activity_inhibit_sleep(bool inhibit);

/* Uptime (ms) of the last physical key press or release. */
int64_t zmk_activity_last_key_ms(void);

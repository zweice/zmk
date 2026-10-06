/*
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Read-only view of the keep-awake behavior's state. */
bool zmk_keep_awake_is_active(void);
/* Selected wireless limit in minutes, 0 = unlimited. */
uint16_t zmk_keep_awake_limit_min(void);
/* Minutes left before the wireless limit stops it, or -1 if no limit applies right now. */
int32_t zmk_keep_awake_remaining_min(void);

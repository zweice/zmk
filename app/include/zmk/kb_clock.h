/*
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

/* Wall clock kept on top of the uptime counter. -1 if the clock was never set. */
int32_t zmk_clock_minutes_of_day(void);

/* true: "light" sleep (System ON, clock survives); false: deep sleep (power off). */
bool zmk_clock_light_sleep(void);

/* Persist the current time so it survives the warm reboot used to wake from light sleep. */
void zmk_clock_save_for_reboot(void);

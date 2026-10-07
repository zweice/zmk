/*
 * In-order retransmission for Keychron's closed 2.4G (ESB) library.
 *
 * The library keeps outgoing reports in a ring buffer (zmk_24g_msgs). Its
 * poll timer sends the head entry and pops it in exactly three places:
 *
 *   poll_timer_expiry_function+0x72   give up after a full retry round  <-- we intercept
 *   poll_timer_expiry_function+0xd6   transmission acknowledged
 *   ringbuf_msg_put+0x3c              ring overflow (drop oldest)
 *
 * All three go through ringbuf_msg_get(), which tail-calls the Zephyr
 * function ring_buf_get(). We link with --wrap=ring_buf_get, so every pop
 * lands in __wrap_ring_buf_get() with the library's call site still in LR.
 *
 * On the "give up" site we peek instead of pop: the report stays at the head
 * and the library simply starts another retry round (16 more attempts). This
 * turns "drop silently after 6 tries" into "retry until acknowledged",
 * strictly in order, so no letter is lost or swapped - and without sending
 * anything twice. After ZMK_24G_ARQ_EXTRA_ROUNDS extra rounds (~1 s of radio
 * silence) we let it drop after all, so an unplugged dongle can't keep the
 * keyboard transmitting forever; by then the library has also flagged the
 * link as lost.
 *
 * Because this relies on fixed offsets in a binary blob, the call site is
 * verified at boot by decoding the BL instruction there. If it does not
 * point at ringbuf_msg_get(), interception stays off and endpoints.c falls
 * back to sending every report twice.
 *
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/kernel.h>
#include <zephyr/init.h>
#include <zephyr/sys/ring_buffer.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

#define GIVE_UP_BL_OFFSET 0x72 /* bl ringbuf_msg_get in poll_timer_expiry_function */
#define BL_LEN 4
#define ZMK_24G_ARQ_EXTRA_ROUNDS 5

extern struct ring_buf zmk_24g_msgs;
extern void poll_timer_expiry_function(void);
extern void ringbuf_msg_get(void);

uint32_t __real_ring_buf_get(struct ring_buf *buf, uint8_t *data, uint32_t size);

#define SUCCESS_BL_OFFSET 0xd6 /* bl ringbuf_msg_get after an acknowledged transmission */
#define OVERFLOW_BL_OFFSET 0x3c /* bl ringbuf_msg_get in ringbuf_msg_put (ring full) */
extern void ringbuf_msg_put(void);

static bool arq_ok;
static uint8_t held_rounds;

/* statistics since power-on / wake-up, shown in the Fn+- status line */
static uint32_t stat_rescued; /* reports the lib would have dropped, delivered after extra rounds */
static uint32_t stat_lost;    /* reports dropped anyway (link down > ~1 s, or ring overflow) */
static uint32_t stat_sent;    /* reports handed to the lib */

bool zmk_24g_arq_active(void) { return arq_ok; }
uint32_t zmk_24g_arq_rescued(void) { return stat_rescued; }
uint32_t zmk_24g_arq_lost(void) { return stat_lost; }
uint32_t zmk_24g_arq_sent(void) { return stat_sent; }
void zmk_24g_arq_count_sent(void) { stat_sent++; }

uint32_t __wrap_ring_buf_get(struct ring_buf *buf, uint8_t *data, uint32_t size) {
    if (arq_ok && buf == &zmk_24g_msgs) {
        uintptr_t ret = (uintptr_t)__builtin_return_address(0);
        uintptr_t poll = (uintptr_t)poll_timer_expiry_function;
        if (ret == poll + GIVE_UP_BL_OFFSET + BL_LEN) {
            if (held_rounds < ZMK_24G_ARQ_EXTRA_ROUNDS) {
                held_rounds++;
                /* leave the report at the head; the library retries it */
                return ring_buf_peek(buf, data, size);
            }
            LOG_WRN("2.4G: report dropped after %u extra retry rounds", held_rounds);
            stat_lost++;
        } else if (ret == poll + SUCCESS_BL_OFFSET + BL_LEN) {
            if (held_rounds) {
                stat_rescued++;
            }
        } else if (ret == (uintptr_t)ringbuf_msg_put + OVERFLOW_BL_OFFSET + BL_LEN) {
            /* overflow drops the oldest report and does not queue the new one */
            stat_lost += 2;
        }
        held_rounds = 0;
    }
    return __real_ring_buf_get(buf, data, size);
}

/* Decode a Thumb-2 BL (T1) at addr and return its target, or 0 if it is not a BL. */
static uintptr_t thumb_bl_target(uintptr_t addr) {
    uint16_t hw1 = *(volatile uint16_t *)addr;
    uint16_t hw2 = *(volatile uint16_t *)(addr + 2);
    if ((hw1 & 0xF800) != 0xF000 || (hw2 & 0xD000) != 0xD000) {
        return 0;
    }
    uint32_t s = (hw1 >> 10) & 1;
    uint32_t imm10 = hw1 & 0x3FF;
    uint32_t j1 = (hw2 >> 13) & 1;
    uint32_t j2 = (hw2 >> 11) & 1;
    uint32_t imm11 = hw2 & 0x7FF;
    uint32_t i1 = !(j1 ^ s);
    uint32_t i2 = !(j2 ^ s);
    int32_t off = (s << 24) | (i1 << 23) | (i2 << 22) | (imm10 << 12) | (imm11 << 1);
    if (s) {
        off |= 0xFE000000; /* sign-extend from bit 24 */
    }
    return addr + 4 + off;
}

static int zmk_24g_arq_init(const struct device *dev) {
    uintptr_t site = ((uintptr_t)poll_timer_expiry_function & ~1u) + GIVE_UP_BL_OFFSET;
    uintptr_t target = thumb_bl_target(site);
    arq_ok = target == ((uintptr_t)ringbuf_msg_get & ~1u);
    if (arq_ok) {
        LOG_INF("2.4G in-order retransmission enabled");
    } else {
        LOG_ERR("2.4G lib layout unexpected (bl target %p) - falling back to duplicate reports",
                (void *)target);
    }
    return 0;
}

SYS_INIT(zmk_24g_arq_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

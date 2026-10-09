/*
 * ESP32 ULP coprocessor: the ULP FSM running from RTC slow memory
 *
 * Copyright (c) 2026 Necessary Innovations AB
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * Reference: ESP32 TRM chapter 1 "ULP Coprocessor"; ESP-IDF's
 * ulp_fsm/include/esp32/ulp.h for the instruction encodings and
 * docs/en/api-reference/system/ulp_instruction_set.rst for their
 * semantics and cycle counts; ulp_common.c and ulp.c for how software
 * runs it.
 *
 * The ULP is in the RTC domain: it keeps running while the CPUs are in
 * light or deep sleep, and only a reset of the RTC domain resets it. It
 * executes 32-bit instructions from RTC slow memory, word-addressed with an
 * 11-bit PC, on RTC_FAST_CLK, taking the documented cycles per instruction
 * (execution plus the next instruction's fetch).
 *
 * Its FSM (TRM 1.5): with RTC_CNTL_ULP_CP_SLP_TIMER_EN set the ULP timer
 * counts the SENS_ULP_CP_SLEEP_CYCn the SLEEP instruction selected (CYC0
 * after reset) in RTC_SLOW_CLK cycles, then wakes the ULP, which takes 2
 * cycles and RTC_CNTL_ULPCP_TOUCH_START_WAIT more, and runs it from
 * SENS_PC_INIT. HALT stops it; 2 cycles later the timer starts again if it
 * is still enabled. With SENS_ULP_CP_FORCE_START_TOP the timer does not
 * start the ULP; SENS_ULP_CP_START_TOP does.
 *
 * Instructions with no effect outside the ULP (ALU, jumps, WAIT, SLEEP)
 * may run ahead of virtual time; every other instruction is carried out at
 * the moment its cycle count puts it.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "hw/core/irq.h"
#include "hw/core/qdev-properties.h"
#include "migration/vmstate.h"
#include "system/address-spaces.h"
#include "hw/misc/esp32_ulp.h"
#include "hw/misc/esp32_reg.h"
#include "hw/misc/esp32_rtc_cntl.h"
#include "hw/misc/esp32_sens.h"

#define PC_MASK         0x7ff
#define SLEEP_SEL_COUNT 5

/* The FSM's power-up and power-down, in RTC_SLOW_CLK cycles (ESP-IDF) */
#define WAKEUP_CYCLES   2
#define PREPARE_CYCLES  2

/* Instructions run ahead of virtual time at most this many in a row */
#define RUN_AHEAD_LIMIT 4096

enum {
    OP_WR_REG = 1,
    OP_RD_REG = 2,
    OP_I2C = 3,
    OP_DELAY = 4,
    OP_ADC = 5,
    OP_ST = 6,
    OP_ALU = 7,
    OP_BRANCH = 8,
    OP_END = 9,
    OP_TSENS = 10,
    OP_HALT = 11,
    OP_LD = 13,
};

enum {
    ALU_REG = 0,
    ALU_IMM = 1,
    ALU_CNT = 2,
};

enum {
    ALU_ADD, ALU_SUB, ALU_AND, ALU_OR, ALU_MOVE, ALU_LSH, ALU_RSH,
};

enum {
    STAGE_INC, STAGE_DEC, STAGE_RST,
};

enum {
    BRANCH_BX = 0,
    BRANCH_BR = 1,
    BRANCH_BS = 2,
};

#define SUB_OPCODE_ST   4
#define END_WAKE        0
#define END_SLEEP       1

/* Execution, and the next instruction's fetch (ESP-IDF's cycle counts) */
#define FETCH           4
#define FETCH_BRANCH    2

static inline uint32_t bits(uint32_t insn, unsigned hi, unsigned lo)
{
    return extract32(insn, lo, hi - lo + 1);
}

static uint8_t *slow_mem(Esp32UlpState *s)
{
    return memory_region_get_ram_ptr(&s->rtc_cntl->slow_mem);
}

static uint32_t mem_read(Esp32UlpState *s, uint32_t word)
{
    return ldl_le_p(slow_mem(s) + 4 * (word & PC_MASK));
}

static void mem_write(Esp32UlpState *s, uint32_t word, uint32_t v)
{
    word &= PC_MASK;
    stl_le_p(slow_mem(s) + 4 * word, v);
    memory_region_set_dirty(&s->rtc_cntl->slow_mem, 4 * word, 4);
}

static int64_t slow_ns(Esp32UlpState *s, uint64_t cycles)
{
    return muldiv64(cycles, NANOSECONDS_PER_SECOND,
                    s->rtc_cntl->rtc_slowclk_freq);
}

/* When the next instruction starts */
static int64_t run_next_ns(Esp32UlpState *s)
{
    return s->run_base_ns + muldiv64(s->run_cycles, NANOSECONDS_PER_SECOND,
                                     s->run_hz);
}

/* RTC_FAST_CLK may have changed: count on at its new rate from here. */
static void run_rebase(Esp32UlpState *s)
{
    uint32_t hz = s->rtc_cntl->rtc_fastclk_freq;

    if (hz != s->run_hz) {
        s->run_base_ns = run_next_ns(s);
        s->run_cycles = 0;
        s->run_hz = hz;
    }
}

static void ulp_set_phase(Esp32UlpState *s, Esp32UlpPhase phase,
                          int64_t deadline)
{
    s->phase = phase;
    timer_mod(&s->timer, deadline);
}

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * The ULP timer starts counting the selected period from `from`. A stopped
 * RTC_SLOW_CLK stops it.
 */
static void ulp_arm_sleep(Esp32UlpState *s, int64_t from)
{
    uint32_t period = s->sens->regs[A_SENS_ULP_CP_SLEEP_CYC0 / 4 +
                                    s->sleep_sel];

    s->phase = ESP32_ULP_SLEEP;
    timer_del(&s->timer);
    if (!s->rtc_cntl->rtc_slowclk_freq) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: the ULP timer counts RTC_SLOW_CLK, which "
                      "is stopped\n");
        return;
    }
    ulp_set_phase(s, ESP32_ULP_SLEEP, from + slow_ns(s, MAX(period, 1)));
}

/* [spec:nuos:req:emu.esp32.ulp] Power the ULP up and wait for its clock. */
static void ulp_wake(Esp32UlpState *s, int64_t from)
{
    uint32_t wait = FIELD_EX32(s->rtc_cntl->regs[A_RTC_CNTL_TIMER2 / 4],
                               RTC_CNTL_TIMER2, ULPCP_TOUCH_START_WAIT);

    if (!s->rtc_cntl->rtc_slowclk_freq) {
        ulp_set_phase(s, ESP32_ULP_WAKING, from);
        return;
    }
    ulp_set_phase(s, ESP32_ULP_WAKING,
                  from + slow_ns(s, WAKEUP_CYCLES + wait));
}

/* [spec:nuos:req:emu.esp32.ulp] WAKE: interrupt RTC_CNTL, wake the chip */
static void ulp_wake_chip(Esp32UlpState *s)
{
    qemu_irq_pulse(s->irq);
    qemu_irq_pulse(s->wakeup);
}

/*
 * Instructions whose effect reaches outside the ULP, or which read what
 * the rest of the chip may change, run at their own time.
 */
static bool insn_external(uint32_t insn)
{
    switch (bits(insn, 31, 28)) {
    case OP_ALU:
    case OP_BRANCH:
    case OP_DELAY:
        return false;
    case OP_END:
        return bits(insn, 27, 25) == END_WAKE;
    default:
        return true;
    }
}

/* [spec:nuos:req:emu.esp32.ulp] ALU instructions; returns their cycles. */
static uint32_t exec_alu(Esp32UlpState *s, uint32_t insn)
{
    unsigned sub = bits(insn, 27, 25), sel = bits(insn, 24, 21);
    unsigned rd = bits(insn, 1, 0), rs = bits(insn, 3, 2);
    uint32_t a = s->r[rs], b, res;

    if (sub == ALU_CNT) {
        uint8_t imm = bits(insn, 11, 4);

        switch (sel) {
        case STAGE_INC:
            s->stage_cnt += imm;
            break;
        case STAGE_DEC:
            s->stage_cnt -= imm;
            break;
        case STAGE_RST:
            s->stage_cnt = 0;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ulp: PC 0x%x: reserved stage count "
                          "operation %u\n", s->pc, sel);
            break;
        }
        return 2 + FETCH;
    }
    if (sub == ALU_REG) {
        b = s->r[bits(insn, 5, 4)];
    } else if (sub == ALU_IMM) {
        b = bits(insn, 19, 4);
    } else {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: PC 0x%x: reserved ALU type %u\n", s->pc,
                      sub);
        return 2 + FETCH;
    }

    switch (sel) {
    case ALU_ADD:
        res = a + b;
        s->overflow = res > 0xffff;
        break;
    case ALU_SUB:
        res = a - b;
        s->overflow = a < b;
        break;
    case ALU_AND:
        res = a & b;
        break;
    case ALU_OR:
        res = a | b;
        break;
    case ALU_MOVE:
        res = sub == ALU_REG ? a : b;
        break;
    case ALU_LSH:
        res = b >= 16 ? 0 : a << b;
        break;
    case ALU_RSH:
        res = b >= 16 ? 0 : a >> b;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: PC 0x%x: reserved ALU operation %u\n",
                      s->pc, sel);
        return 2 + FETCH;
    }
    s->r[rd] = res;
    s->zero = (res & 0xffff) == 0;
    return 2 + FETCH;
}

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * JUMP, JUMPR and JUMPS. Sets *next to the following PC; returns the
 * cycles.
 */
static uint32_t exec_branch(Esp32UlpState *s, uint32_t insn, uint32_t *next)
{
    unsigned sub = bits(insn, 27, 25);
    uint32_t step = bits(insn, 23, 17);
    uint32_t rel = bits(insn, 24, 24) ? s->pc - step : s->pc + step;
    bool take;

    switch (sub) {
    case BRANCH_BX:
        switch (bits(insn, 24, 22)) {
        case 0:
            take = true;
            break;
        case 1:
            take = s->zero;
            break;
        case 2:
            take = s->overflow;
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ulp: PC 0x%x: reserved JUMP type %u\n",
                          s->pc, bits(insn, 24, 22));
            take = false;
            break;
        }
        if (take) {
            *next = bits(insn, 21, 21) ? s->r[bits(insn, 1, 0)] :
                                         bits(insn, 12, 2);
        }
        break;
    case BRANCH_BR:
        if (bits(insn, 16, 16)) {
            take = s->r[0] >= bits(insn, 15, 0);
        } else {
            take = s->r[0] < bits(insn, 15, 0);
        }
        if (take) {
            *next = rel;
        }
        break;
    case BRANCH_BS:
        switch (bits(insn, 16, 15)) {
        case 0:
            take = s->stage_cnt < bits(insn, 7, 0);
            break;
        case 1:
            take = s->stage_cnt >= bits(insn, 7, 0);
            break;
        default:
            take = s->stage_cnt <= bits(insn, 7, 0);
            break;
        }
        if (take) {
            *next = rel;
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: PC 0x%x: reserved branch type %u\n",
                      s->pc, sub);
        break;
    }
    return 2 + FETCH_BRANCH;
}

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * REG_RD and REG_WR reach RTC_CNTL, RTCIO, SENS and RTC_I2C: register
 * address addr (in words) is DR_REG_RTCCNTL_BASE + 4 * addr.
 */
static uint32_t rtc_reg_read(Esp32UlpState *s, uint32_t addr)
{
    hwaddr a = DR_REG_RTCCNTL_BASE + 4 * addr;
    MemTxResult res;
    uint32_t v = address_space_ldl_le(&s->rtc_as, a, MEMTXATTRS_UNSPECIFIED,
                                      &res);

    if (res != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: PC 0x%x: REG_RD of 0x%" HWADDR_PRIx
                      " failed\n", s->pc, a);
        return 0;
    }
    return v;
}

static void rtc_reg_write(Esp32UlpState *s, uint32_t addr, uint32_t v)
{
    hwaddr a = DR_REG_RTCCNTL_BASE + 4 * addr;
    MemTxResult res;

    address_space_stl_le(&s->rtc_as, a, v, MEMTXATTRS_UNSPECIFIED, &res);
    if (res != MEMTX_OK) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: PC 0x%x: REG_WR of 0x%" HWADDR_PRIx
                      " failed\n", s->pc, a);
    }
}

/* The field [high:low] of a register, or -1 for an empty range */
static int field_width(Esp32UlpState *s, unsigned high, unsigned low)
{
    if (high < low) {
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: PC 0x%x: register bit range [%u:%u] is "
                      "empty\n", s->pc, high, low);
        return -1;
    }
    return high - low + 1;
}

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * Carry out the instruction at the PC; returns its cycles.
 */
static uint32_t exec_insn(Esp32UlpState *s, uint32_t insn)
{
    uint32_t next = (s->pc + 1) & PC_MASK;
    uint32_t cycles, v, addr;
    unsigned rd = bits(insn, 1, 0), rs = bits(insn, 3, 2);
    unsigned high, low;
    int width;

    switch (bits(insn, 31, 28)) {
    case OP_ALU:
        cycles = exec_alu(s, insn);
        break;

    case OP_BRANCH:
        cycles = exec_branch(s, insn, &next);
        next &= PC_MASK;
        break;

    case OP_ST:
        /*
         * The word at Rdst + offset gets Rsrc in its lower half and the
         * PC and Rdst's number in its upper half (TRM 1.4.2).
         */
        if (bits(insn, 27, 25) != SUB_OPCODE_ST) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ulp: PC 0x%x: reserved ST type %u\n",
                          s->pc, bits(insn, 27, 25));
        } else {
            addr = s->r[rs] + bits(insn, 20, 10);
            mem_write(s, addr, (s->pc << 21) | (rs << 16) | s->r[rd]);
        }
        cycles = 4 + FETCH;
        break;

    case OP_LD:
        /* The lower half of the word at Rsrc + offset */
        addr = s->r[rs] + bits(insn, 20, 10);
        s->r[rd] = mem_read(s, addr);
        cycles = 4 + FETCH;
        break;

    case OP_RD_REG:
        high = bits(insn, 27, 23);
        low = bits(insn, 22, 18);
        width = field_width(s, high, low);
        v = rtc_reg_read(s, bits(insn, 9, 0));
        /* More than 16 bits reads [low + 15:low] */
        s->r[0] = width < 0 ? 0 : extract32(v, low, MIN(width, 16));
        cycles = 4 + FETCH;
        break;

    case OP_WR_REG:
        high = bits(insn, 27, 23);
        low = bits(insn, 22, 18);
        width = field_width(s, high, low);
        if (width > 0) {
            addr = bits(insn, 9, 0);
            /* Data above its 8 bits is zero */
            v = deposit32(rtc_reg_read(s, addr), low, width,
                          bits(insn, 17, 10));
            rtc_reg_write(s, addr, v);
        }
        cycles = 8 + FETCH;
        break;

    case OP_DELAY:
        cycles = 2 + bits(insn, 15, 0) + FETCH;
        break;

    case OP_ADC:
        s->r[rd] = esp32_sens_ulp_adc(s->sens, bits(insn, 6, 6),
                                      bits(insn, 5, 2), &cycles);
        cycles += FETCH;
        break;

    case OP_TSENS:
        s->r[rd] = esp32_sens_ulp_tsens(s->sens, bits(insn, 15, 2), &cycles);
        cycles += FETCH;
        break;

    case OP_I2C:
        v = esp32_sens_rtc_i2c(s->sens, bits(insn, 27, 0), &cycles);
        if (!bits(insn, 27, 27)) {
            s->r[0] = v;
        }
        cycles += FETCH;
        break;

    case OP_END:
        switch (bits(insn, 27, 25)) {
        case END_WAKE:
            if (bits(insn, 0, 0)) {
                ulp_wake_chip(s);
            }
            break;
        case END_SLEEP:
            if (bits(insn, 3, 0) >= SLEEP_SEL_COUNT) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "esp32_ulp: PC 0x%x: SLEEP selects "
                              "SENS_ULP_CP_SLEEP_CYC%u of 0-4\n", s->pc,
                              bits(insn, 3, 0));
            } else {
                s->sleep_sel = bits(insn, 3, 0);
            }
            break;
        default:
            qemu_log_mask(LOG_GUEST_ERROR,
                          "esp32_ulp: PC 0x%x: reserved END type %u\n",
                          s->pc, bits(insn, 27, 25));
            break;
        }
        cycles = 2 + FETCH;
        break;

    case OP_HALT:
        s->phase = ESP32_ULP_HALTING;
        cycles = 2;
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "esp32_ulp: PC 0x%x: reserved opcode %u (0x%08x)\n",
                      s->pc, bits(insn, 31, 28), insn);
        cycles = 2 + FETCH;
        break;
    }
    s->pc = next;
    return cycles;
}

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * Run the program up to now: every instruction whose start time has come,
 * and those with no outside effect beyond, a bounded number of them.
 */
static void ulp_run(Esp32UlpState *s)
{
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    unsigned ahead = 0;

    run_rebase(s);
    while (s->phase == ESP32_ULP_RUNNING) {
        int64_t start = run_next_ns(s);
        uint32_t insn = mem_read(s, s->pc);

        if (start > now) {
            if (insn_external(insn) || ahead >= RUN_AHEAD_LIMIT) {
                timer_mod(&s->timer, start);
                return;
            }
            ahead++;
        }
        s->run_cycles += exec_insn(s, insn);
    }
    if (s->phase == ESP32_ULP_HALTING) {
        ulp_set_phase(s, ESP32_ULP_HALTING,
                      run_next_ns(s) +
                      (s->rtc_cntl->rtc_slowclk_freq ?
                       slow_ns(s, PREPARE_CYCLES) : 0));
    }
}

static void ulp_timer_cb(void *opaque)
{
    Esp32UlpState *s = opaque;
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    uint32_t force = s->sens->regs[A_SENS_SAR_START_FORCE / 4];

    switch (s->phase) {
    case ESP32_ULP_SLEEP:
        if (FIELD_EX32(force, SENS_SAR_START_FORCE, ULP_CP_FORCE_START_TOP)) {
            /* The timer's expiry does not start the ULP */
            ulp_arm_sleep(s, now);
        } else {
            ulp_wake(s, now);
        }
        break;
    case ESP32_ULP_WAKING:
        s->phase = ESP32_ULP_RUNNING;
        s->pc = FIELD_EX32(force, SENS_SAR_START_FORCE, PC_INIT);
        s->run_base_ns = now;
        s->run_cycles = 0;
        s->run_hz = s->rtc_cntl->rtc_fastclk_freq;
        ulp_run(s);
        break;
    case ESP32_ULP_RUNNING:
        ulp_run(s);
        break;
    case ESP32_ULP_HALTING:
        if (s->timer_en) {
            ulp_arm_sleep(s, now);
        } else {
            s->phase = ESP32_ULP_OFF;
        }
        break;
    default:
        break;
    }
}

/*
 * [spec:nuos:req:emu.esp32.ulp]
 * RTC_CNTL_ULP_CP_SLP_TIMER_EN: setting it starts the timer of a halted
 * ULP; clearing it stops the timer, and a running ULP halts for good.
 */
static void esp32_ulp_timer_en(void *opaque, int n, int level)
{
    Esp32UlpState *s = ESP32_ULP(opaque);
    bool en = level != 0;

    if (en == s->timer_en) {
        return;
    }
    s->timer_en = en;
    if (en && s->phase == ESP32_ULP_OFF) {
        ulp_arm_sleep(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    } else if (!en && (s->phase == ESP32_ULP_SLEEP ||
                       s->phase == ESP32_ULP_WAKING)) {
        timer_del(&s->timer);
        s->phase = ESP32_ULP_OFF;
    }
}

/* [spec:nuos:req:emu.esp32.ulp] SENS_ULP_CP_START_TOP starts a halted ULP */
static void esp32_ulp_start(void *opaque, int n, int level)
{
    Esp32UlpState *s = ESP32_ULP(opaque);
    bool rising = level && !s->start_level;

    s->start_level = level != 0;
    if (rising && (s->phase == ESP32_ULP_OFF ||
                   s->phase == ESP32_ULP_SLEEP)) {
        ulp_wake(s, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
    }
}

static void esp32_ulp_reset_hold(Object *obj, ResetType type)
{
    Esp32UlpState *s = ESP32_ULP(obj);

    timer_del(&s->timer);
    s->phase = ESP32_ULP_OFF;
    s->timer_en = false;
    s->start_level = false;
    memset(s->r, 0, sizeof(s->r));
    s->stage_cnt = 0;
    s->zero = false;
    s->overflow = false;
    s->pc = 0;
    s->sleep_sel = 0;
    s->run_base_ns = 0;
    s->run_cycles = 0;
    s->run_hz = 0;
}

static void esp32_ulp_realize(DeviceState *dev, Error **errp)
{
    Esp32UlpState *s = ESP32_ULP(dev);

    if (!s->rtc_cntl || !s->sens || !s->rtc_regs) {
        error_setg(errp, "esp32_ulp: rtc-cntl, sens and memory links must "
                   "be set");
        return;
    }
    address_space_init(&s->rtc_as, s->rtc_regs, "esp32-ulp");
}

static void esp32_ulp_init(Object *obj)
{
    Esp32UlpState *s = ESP32_ULP(obj);
    DeviceState *dev = DEVICE(obj);

    timer_init_ns(&s->timer, QEMU_CLOCK_VIRTUAL, ulp_timer_cb, s);
    qdev_init_gpio_in_named(dev, esp32_ulp_timer_en, ESP32_ULP_TIMER_EN_IN,
                            1);
    qdev_init_gpio_in_named(dev, esp32_ulp_start, ESP32_ULP_START_IN, 1);
    qdev_init_gpio_out_named(dev, &s->wakeup, ESP32_ULP_WAKEUP, 1);
    qdev_init_gpio_out_named(dev, &s->irq, ESP32_ULP_INT, 1);
}

static const Property esp32_ulp_properties[] = {
    DEFINE_PROP_LINK("rtc-cntl", Esp32UlpState, rtc_cntl,
                     TYPE_ESP32_RTC_CNTL, Esp32RtcCntlState *),
    DEFINE_PROP_LINK("sens", Esp32UlpState, sens, TYPE_ESP32_SENS,
                     Esp32SensState *),
    DEFINE_PROP_LINK("memory", Esp32UlpState, rtc_regs, TYPE_MEMORY_REGION,
                     MemoryRegion *),
};

static const VMStateDescription vmstate_esp32_ulp = {
    .name = TYPE_ESP32_ULP,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_TIMER(timer, Esp32UlpState),
        VMSTATE_UINT32(phase, Esp32UlpState),
        VMSTATE_BOOL(timer_en, Esp32UlpState),
        VMSTATE_BOOL(start_level, Esp32UlpState),
        VMSTATE_UINT16_ARRAY(r, Esp32UlpState, ESP32_ULP_REGS),
        VMSTATE_UINT8(stage_cnt, Esp32UlpState),
        VMSTATE_BOOL(zero, Esp32UlpState),
        VMSTATE_BOOL(overflow, Esp32UlpState),
        VMSTATE_UINT32(pc, Esp32UlpState),
        VMSTATE_UINT32(sleep_sel, Esp32UlpState),
        VMSTATE_INT64(run_base_ns, Esp32UlpState),
        VMSTATE_UINT64(run_cycles, Esp32UlpState),
        VMSTATE_UINT32(run_hz, Esp32UlpState),
        VMSTATE_END_OF_LIST()
    },
};

static void esp32_ulp_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    ResettableClass *rc = RESETTABLE_CLASS(klass);

    rc->phases.hold = esp32_ulp_reset_hold;
    dc->realize = esp32_ulp_realize;
    dc->vmsd = &vmstate_esp32_ulp;
    device_class_set_props(dc, esp32_ulp_properties);
}

/* [spec:nuos:req:emu.esp32.ulp] */
static const TypeInfo esp32_ulp_info = {
    .name = TYPE_ESP32_ULP,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(Esp32UlpState),
    .instance_init = esp32_ulp_init,
    .class_init = esp32_ulp_class_init,
};

static void esp32_ulp_register_types(void)
{
    type_register_static(&esp32_ulp_info);
}

type_init(esp32_ulp_register_types)

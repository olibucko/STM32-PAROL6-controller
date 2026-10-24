/**
 * command.c — Commands typed on (or sent by) the PC, and the replies
 * ==================================================================
 * See command.h for the list of commands and the rules of the protocol.
 *
 * HOW A LINE IS HANDLED
 *   1. Split the line into words ("tokens") at the spaces.
 *      "MOVE 1 90 30" -> ["MOVE", "1", "90", "30"]
 *   2. Look up the first word in the command table (s_commands, near the
 *      bottom of this file) to find the function that handles it.
 *   3. That function checks the rest of the words, does the work, and sends
 *      exactly one OK or ERR line.
 *   Adding a new command = write one handler function + add one table row.
 *
 * ABOUT PRINTING DECIMALS
 *   The small C library used on the STM32 ("newlib-nano") leaves out support
 *   for printing decimals with printf/snprintf ("%f") unless a project
 *   setting is changed, to save memory. Rather than depend on that setting,
 *   fmt_3dp() below prints numbers with 3 decimal places by hand.
 */

#include "command.h"
#include "serial.h"
#include "tmc5160.h"
#include <stdio.h>     /* snprintf                       */
#include <stdlib.h>    /* strtof, strtol, strtoul        */
#include <string.h>    /* strcmp, strlen                 */
#include <ctype.h>     /* toupper                        */
#include <math.h>      /* isfinite                       */

#define MAX_TOKENS  6          /* the longest command has 4 words; spare room */
#define REPLY_MAX   128        /* longest reply line, in characters          */

/* ---- State kept for each joint --------------------------------------- */
static axis_t   *s_axes[MOTION_MAX_AXES];
static uint8_t   s_count = 0;
static float     s_speed_rpm[MOTION_MAX_AXES];     /* speed used when a move
                                                    * command gives none   */
static uint32_t  s_done_reported[MOTION_MAX_AXES]; /* moves already told to
                                                    * the PC as DONE        */


/* ======================================================================== */
/* SMALL HELPERS                                                            */
/* ======================================================================== */

/* Write a number into `buf` with exactly 3 decimal places, e.g. -12.5 ->
 * "-12.500". Rounds to the nearest 0.001. */
static void fmt_3dp(char *buf, uint32_t size, float v)
{
    const char *sign = "";
    if (v < 0.0f) { sign = "-"; v = -v; }
    uint32_t milli = (uint32_t)(v * 1000.0f + 0.5f);   /* 12.5 -> 12500 */
    snprintf(buf, size, "%s%lu.%03lu", sign,
             (unsigned long)(milli / 1000u), (unsigned long)(milli % 1000u));
}

/* Read a decimal number such as "90", "-12.5" or "1e2".
 * Returns 1 if the WHOLE word was a valid number, else 0. */
static int parse_float(const char *s, float *out)
{
    char *end;
    float v = strtof(s, &end);      /* `end` = where reading stopped        */
    if (end == s || *end != '\0') return 0;  /* nothing read / junk after it */
    if (!isfinite(v)) return 0;     /* reject "inf" and "nan"               */
    *out = v;
    return 1;
}

/* Number base: "0x..." means hexadecimal (base 16), anything else decimal.
 * (Standard C would read "010" as octal = 8, which would surprise anyone
 * typing a register value, so that rule is deliberately not used.) */
static int base_of(const char *s)
{
    if (*s == '-' || *s == '+') s++;
    return (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) ? 16 : 10;
}

/* Read a whole number that may be negative (e.g. a step count). */
static int parse_int(const char *s, long *out)
{
    char *end;
    long v = strtol(s, &end, base_of(s));
    if (end == s || *end != '\0') return 0;
    *out = v;
    return 1;
}

/* Read a whole number that can't be negative (register addresses/values). */
static int parse_uint(const char *s, uint32_t *out)
{
    if (*s == '-') return 0;
    char *end;
    unsigned long v = strtoul(s, &end, base_of(s));
    if (end == s || *end != '\0') return 0;
    *out = (uint32_t)v;
    return 1;
}

/* Read a joint number ("1".."count") and turn it into an array index
 * (0..count-1). On failure, sends the ERR reply itself and returns -1. */
static int parse_joint(const char *s)
{
    long j;
    if (!parse_int(s, &j) || j < 1 || j > s_count)
    {
        char r[REPLY_MAX];
        snprintf(r, sizeof r, "ERR joint must be 1 to %u", (unsigned)s_count);
        serial_println(r);
        return -1;
    }
    return (int)(j - 1);
}

/* Turn a MOTION_ERR_... result into an ERR reply for joint index i.
 * `target_deg` is only used for MOTION_ERR_RANGE. */
static void reply_motion_error(int i, int result, float target_deg)
{
    char r[REPLY_MAX], lim[16];
    if (result == MOTION_ERR_BUSY)
    {
        snprintf(r, sizeof r, "ERR %d busy: wait for DONE, or send STOP", i + 1);
    }
    else if (result == MOTION_ERR_RANGE)
    {
        char tgt[16], lo[16], hi[16];
        fmt_3dp(tgt, sizeof tgt, target_deg);
        fmt_3dp(lo,  sizeof lo,  s_axes[i]->min_deg);
        fmt_3dp(hi,  sizeof hi,  s_axes[i]->max_deg);
        snprintf(r, sizeof r, "ERR %d target %s is outside the range %s to %s deg",
                 i + 1, tgt, lo, hi);
    }
    else
    {
        fmt_3dp(lim, sizeof lim, s_axes[i]->max_rpm);
        snprintf(r, sizeof r, "ERR %d speed must be above 0 and at most %s rpm",
                 i + 1, lim);
    }
    serial_println(r);
}

/* Start a move of n microsteps on joint index i, and send the reply:
 *   OK j from=<deg> to=<deg>     or     ERR ...
 * All three move commands (MOVE, MOVEREL, STEPS) come through here, so
 * they all report and refuse things the same way. */
static void start_move(int i, int32_t n, float rpm)
{
    axis_t *ax = s_axes[i];
    float from_deg = motion_get_angle_deg(ax);
    float to_deg   = (float)((int64_t)ax->position_steps + n) / ax->steps_per_deg;

    int result = motion_move_steps(ax, n, rpm);
    if (result != MOTION_OK) { reply_motion_error(i, result, to_deg); return; }

    char f[16], t[16], r[REPLY_MAX];
    fmt_3dp(f, sizeof f, from_deg);
    fmt_3dp(t, sizeof t, to_deg);
    snprintf(r, sizeof r, "OK %d from=%s to=%s", i + 1, f, t);
    serial_println(r);
}

/* Optional [rpm] word: use it if given, otherwise the joint's SPEED.
 * Returns 1 if fine, 0 (after sending ERR) if the word isn't a number. */
static int optional_rpm(int ntok, char **tok, int idx, int i, float *rpm)
{
    *rpm = s_speed_rpm[i];
    if (ntok > idx && !parse_float(tok[idx], rpm))
    {
        serial_println("ERR rpm must be a number");
        return 0;
    }
    return 1;
}


/* ======================================================================== */
/* COMMAND HANDLERS                                                         */
/* Each receives the words of the line: tok[0] is the command itself.       */
/* ======================================================================== */

static void cmd_help(int ntok, char **tok);   /* written after the table */

static void cmd_ping(int ntok, char **tok)
{
    (void)ntok; (void)tok;      /* not needed; "(void)" silences warnings */
    serial_println("OK PONG");
}

static void cmd_status(int ntok, char **tok)
{
    if (ntok != 2) { serial_println("ERR usage: STATUS j"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    axis_t *ax = s_axes[i];
    char pos[16], spd[16], acc[16], r[REPLY_MAX];
    fmt_3dp(pos, sizeof pos, motion_get_angle_deg(ax));
    fmt_3dp(spd, sizeof spd, s_speed_rpm[i]);
    fmt_3dp(acc, sizeof acc, ax->accel_rpm_s);
    snprintf(r, sizeof r, "OK %d pos=%s moving=%u left=%lu speed=%s accel=%s",
             i + 1, pos, (unsigned)motion_busy(ax),
             (unsigned long)motion_steps_remaining(ax), spd, acc);
    serial_println(r);
}

static void cmd_info(int ntok, char **tok)
{
    if (ntok != 2) { serial_println("ERR usage: INFO j"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    axis_t *ax = s_axes[i];
    char lo[16], hi[16], gr[16], vr[16], ar[16], r[REPLY_MAX];
    fmt_3dp(lo, sizeof lo, ax->min_deg);
    fmt_3dp(hi, sizeof hi, ax->max_deg);
    fmt_3dp(gr, sizeof gr, ax->gear_ratio);
    fmt_3dp(vr, sizeof vr, ax->max_rpm);
    fmt_3dp(ar, sizeof ar, ax->max_accel_rpm_s);
    snprintf(r, sizeof r, "OK %d min=%s max=%s gear=%s max_rpm=%s max_accel=%s",
             i + 1, lo, hi, gr, vr, ar);
    serial_println(r);
}

static void cmd_move(int ntok, char **tok)
{
    if (ntok < 3 || ntok > 4) { serial_println("ERR usage: MOVE j deg [rpm]"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    float deg, rpm;
    if (!parse_float(tok[2], &deg)) { serial_println("ERR deg must be a number"); return; }
    if (!optional_rpm(ntok, tok, 3, i, &rpm)) return;

    /* Absolute target in steps, minus where we are = how far to go.
     * (Same sum as motion_move_to_deg, done here so start_move can report
     * the plan.) */
    int32_t target = motion_deg_to_steps(s_axes[i], deg);
    start_move(i, target - s_axes[i]->position_steps, rpm);
}

static void cmd_moverel(int ntok, char **tok)
{
    if (ntok < 3 || ntok > 4) { serial_println("ERR usage: MOVEREL j deg [rpm]"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    float deg, rpm;
    if (!parse_float(tok[2], &deg)) { serial_println("ERR deg must be a number"); return; }
    if (!optional_rpm(ntok, tok, 3, i, &rpm)) return;

    /* Degrees -> steps, relative to wherever the joint is now. */
    int32_t n = motion_deg_to_steps(s_axes[i], deg);
    start_move(i, n, rpm);
}

static void cmd_steps(int ntok, char **tok)
{
    if (ntok < 3 || ntok > 4) { serial_println("ERR usage: STEPS j n [rpm]"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    long n;
    float rpm;
    if (!parse_int(tok[2], &n) || n > 100000000L || n < -100000000L)
    {
        /* 100 million microsteps ~ 2000 motor turns: far beyond any real
         * joint, but keeps the number safely inside a 32-bit integer. */
        serial_println("ERR n must be a whole number within +/-100000000");
        return;
    }
    if (!optional_rpm(ntok, tok, 3, i, &rpm)) return;

    start_move(i, (int32_t)n, rpm);
}

static void cmd_speed(int ntok, char **tok)
{
    if (ntok != 3) { serial_println("ERR usage: SPEED j rpm"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    float rpm;
    if (!parse_float(tok[2], &rpm) || rpm <= 0.0f || rpm > s_axes[i]->max_rpm)
    {
        reply_motion_error(i, MOTION_ERR_LIMIT, 0.0f);
        return;
    }
    s_speed_rpm[i] = rpm;   /* only used by later moves, so fine while moving */
    serial_println("OK");
}

static void cmd_accel(int ntok, char **tok)
{
    if (ntok != 3) { serial_println("ERR usage: ACCEL j rpm_per_s"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    float a;
    if (!parse_float(tok[2], &a)) { serial_println("ERR rpm_per_s must be a number"); return; }

    int result = motion_set_accel_rpm_s(s_axes[i], a);
    if (result == MOTION_OK)
    {
        serial_println("OK");
    }
    else if (result == MOTION_ERR_BUSY)
    {
        reply_motion_error(i, result, 0.0f);
    }
    else
    {
        char r[REPLY_MAX], lim[16];
        fmt_3dp(lim, sizeof lim, s_axes[i]->max_accel_rpm_s);
        snprintf(r, sizeof r, "ERR %d accel must be above 0 and at most %s rpm/s",
                 i + 1, lim);
        serial_println(r);
    }
}

static void cmd_zero(int ntok, char **tok)
{
    if (ntok != 2) { serial_println("ERR usage: ZERO j"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    int result = motion_set_zero(s_axes[i]);
    if (result == MOTION_OK) serial_println("OK");
    else                     reply_motion_error(i, result, 0.0f);
}

/* STOP and HALT share the same shape: one joint, or all if none given. */
static void stop_or_halt(int ntok, char **tok, void (*action)(axis_t *),
                         const char *usage)
{
    if (ntok == 1)
    {
        for (uint8_t i = 0; i < s_count; i++) action(s_axes[i]);
        serial_println("OK");
        return;
    }
    if (ntok != 2) { serial_println(usage); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;
    action(s_axes[i]);
    serial_println("OK");
}

static void cmd_stop(int ntok, char **tok)
{
    stop_or_halt(ntok, tok, motion_stop, "ERR usage: STOP [j]");
}

static void cmd_halt(int ntok, char **tok)
{
    stop_or_halt(ntok, tok, motion_halt, "ERR usage: HALT [j]");
}

/* Driver registers. There is only one TMC5160 wired up so far, so the joint
 * number is checked but every joint talks to that one driver. When more
 * drivers are added, tmc5160.c will need a "which driver" argument (each
 * driver gets its own chip-select pin) and this is where it gets passed. */
static void cmd_rreg(int ntok, char **tok)
{
    if (ntok != 3) { serial_println("ERR usage: RREG j reg"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    uint32_t reg;
    if (!parse_uint(tok[2], &reg) || reg > 0x7Fu)
    {
        /* TMC5160 register addresses are 7 bits: 0x00 to 0x7F. */
        serial_println("ERR reg must be 0x00 to 0x7F");
        return;
    }
    uint32_t value = tmc5160_read((uint8_t)reg);

    char r[REPLY_MAX];
    snprintf(r, sizeof r, "OK %d reg=0x%02lX val=0x%08lX", i + 1,
             (unsigned long)reg, (unsigned long)value);
    serial_println(r);
}

static void cmd_wreg(int ntok, char **tok)
{
    if (ntok != 4) { serial_println("ERR usage: WREG j reg value"); return; }
    int i = parse_joint(tok[1]);
    if (i < 0) return;

    uint32_t reg, value;
    if (!parse_uint(tok[2], &reg) || reg > 0x7Fu)
    {
        serial_println("ERR reg must be 0x00 to 0x7F");
        return;
    }
    if (!parse_uint(tok[3], &value)) { serial_println("ERR value must be a whole number >= 0"); return; }

    /* Changing driver settings (current, chopper, microsteps) in the middle
     * of a move could make the motor jerk or lose steps, so only allow it
     * while the joint is still. */
    if (motion_busy(s_axes[i])) { reply_motion_error(i, MOTION_ERR_BUSY, 0.0f); return; }

    tmc5160_write((uint8_t)reg, value);
    serial_println("OK");
}


/* ======================================================================== */
/* COMMAND TABLE                                                            */
/* ======================================================================== */

typedef struct
{
    const char *name;                        /* the word typed, in CAPITALS */
    void      (*handler)(int ntok, char **tok);
    const char *help;                        /* one line for HELP           */
} command_t;

static const command_t s_commands[] =
{
    { "HELP",    cmd_help,    "HELP                  list commands" },
    { "PING",    cmd_ping,    "PING                  check the link" },
    { "STATUS",  cmd_status,  "STATUS j              position and settings" },
    { "INFO",    cmd_info,    "INFO j                fixed range and limits" },
    { "MOVE",    cmd_move,    "MOVE j deg [rpm]      go to an absolute angle" },
    { "MOVEREL", cmd_moverel, "MOVEREL j deg [rpm]   move by an amount" },
    { "STEPS",   cmd_steps,   "STEPS j n [rpm]       move by n microsteps" },
    { "SPEED",   cmd_speed,   "SPEED j rpm           default speed" },
    { "ACCEL",   cmd_accel,   "ACCEL j rpm_per_s     acceleration" },
    { "ZERO",    cmd_zero,    "ZERO j                call this spot 0 deg" },
    { "STOP",    cmd_stop,    "STOP [j]              slow down and stop" },
    { "HALT",    cmd_halt,    "HALT [j]              stop instantly" },
    { "RREG",    cmd_rreg,    "RREG j reg            read driver register" },
    { "WREG",    cmd_wreg,    "WREG j reg value      write driver register" },
};
#define NUM_COMMANDS  (sizeof s_commands / sizeof s_commands[0])

static void cmd_help(int ntok, char **tok)
{
    (void)ntok; (void)tok;
    for (uint32_t k = 0; k < NUM_COMMANDS; k++)
    {
        char r[REPLY_MAX];
        snprintf(r, sizeof r, "# %s", s_commands[k].help);
        serial_println(r);
    }
    serial_println("OK");
}


/* ======================================================================== */
/* PUBLIC FUNCTIONS                                                         */
/* ======================================================================== */

void command_init(axis_t *const axes[], uint8_t count)
{
    if (count > MOTION_MAX_AXES) count = MOTION_MAX_AXES;
    s_count = count;
    for (uint8_t i = 0; i < count; i++)
    {
        s_axes[i]          = axes[i];
        s_speed_rpm[i]     = axes[i]->default_rpm;     /* from joints.c */
        s_done_reported[i] = axes[i]->moves_finished;  /* nothing to report yet */
    }
}

void command_execute(const char *line)
{
    /* Work on a copy, because splitting the line writes '\0' markers into
     * it and the caller's text shouldn't be changed. */
    char buf[SERIAL_LINE_MAX + 1];
    strncpy(buf, line, sizeof buf - 1);
    buf[sizeof buf - 1] = '\0';

    /* 1. Split into words. Each space/tab is replaced by '\0' (the C "end
     *    of text" marker), and tok[] remembers where each word starts. */
    char *tok[MAX_TOKENS];
    int   ntok = 0;
    char *p = buf;
    while (*p != '\0')
    {
        while (*p == ' ' || *p == '\t') *p++ = '\0';   /* skip spaces     */
        if (*p == '\0') break;
        if (ntok == MAX_TOKENS) { serial_println("ERR too many words"); return; }
        tok[ntok++] = p;                                /* a word starts   */
        while (*p != '\0' && *p != ' ' && *p != '\t') p++;   /* skip word */
    }
    if (ntok == 0) return;   /* blank line: nothing to do, no reply */

    /* 2. Command word to capitals, so "move" and "MOVE" both work. */
    for (char *c = tok[0]; *c; c++) *c = (char)toupper((unsigned char)*c);

    /* 3. Find it in the table and run its handler. */
    for (uint32_t k = 0; k < NUM_COMMANDS; k++)
    {
        if (strcmp(tok[0], s_commands[k].name) == 0)
        {
            s_commands[k].handler(ntok, tok);
            return;
        }
    }

    char r[REPLY_MAX];
    snprintf(r, sizeof r, "ERR unknown command '%.20s' (type HELP)", tok[0]);
    serial_println(r);
}

void command_report_finished_moves(void)
{
    for (uint8_t i = 0; i < s_count; i++)
    {
        /* One DONE per finished move. "while" rather than "if" so that two
         * moves finishing between checks still give two DONE lines. */
        while (s_done_reported[i] != s_axes[i]->moves_finished)
        {
            s_done_reported[i]++;
            char pos[16], r[REPLY_MAX];
            fmt_3dp(pos, sizeof pos, motion_get_angle_deg(s_axes[i]));
            snprintf(r, sizeof r, "DONE %d pos=%s", i + 1, pos);
            serial_println(r);
        }
    }
}

void command_stop_all(void)
{
    for (uint8_t i = 0; i < s_count; i++) motion_stop(s_axes[i]);
}

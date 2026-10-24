/**
 * motion.c — Step generation and smooth (trapezoidal) moves for stepper axes
 * ==========================================================================
 * See motion.h for what this module does and how to use it. This file
 * explains HOW it works.
 *
 * THE BIG PICTURE
 *   The timer runs in "PWM mode": it counts up from 0, the STEP pin is HIGH
 *   while the count is below the compare value (CCR), and the count restarts
 *   at 0 when it reaches the period value (ARR). So:
 *     - one timer period         = one STEP pulse
 *     - CCR (compare value)      = how long the pulse stays high
 *     - ARR + 1 (period, ticks)  = time from one step to the next
 *   Changing ARR from one step to the next changes the speed. That is how
 *   the speed-up and slow-down are made.
 *
 *   Each time the count restarts, the timer triggers an interrupt: a function
 *   the hardware calls automatically, pausing the main program for a moment.
 *   In it we count the step just sent and set up the timing of the NEXT one.
 *
 * WHY "PRELOAD" MATTERS
 *   Preload is switched on for ARR and CCR (in CubeMX and MX_TIM2_Init). With
 *   preload, a new value written to ARR or CCR waits in a holding register
 *   and is only copied into use at the start of the next period. Without it,
 *   a new value could take effect halfway through a pulse and produce a
 *   wrong-length or missing step.
 *   Consequence: when the interrupt runs at the start of period k, period k's
 *   values are already in use, so the interrupt writes the values for period
 *   k+1. The code is always "one step ahead".
 *
 * HOW A MOVE ENDS CLEANLY
 *   After the last real step, one extra "blank" period follows with CCR = 0,
 *   so the pin stays LOW. When that blank period begins, the interrupt stops
 *   the timer. This guarantees exactly the requested number of pulses.
 *
 * WHY THE STEP PIN IS HELD LOW WHEN IDLE (bug found 21 Oct 2026)
 *   Switching the timer output off leaves the STEP wire "floating" (connected
 *   to nothing). Electrical noise then looked like steps to the driver and the
 *   motor crept. So the output stays on with CCR = 0 (pin driven LOW), and
 *   only the counter is paused between moves.
 */

#include "motion.h"
#include <math.h>      /* sqrtf */
#include <stddef.h>    /* NULL  */

/* ------------------------------------------------------------------------ */
/* List of axes in use, so the shared timer interrupt can find the axis that */
/* belongs to the timer that fired.                                         */
/* ------------------------------------------------------------------------ */
static axis_t  *s_axes[MOTION_MAX_AXES];
static uint8_t  s_axis_count = 0;

/* Forward declarations of helpers used before they are written out below. */
static void     set_direction(axis_t *ax, uint8_t positive);
static uint32_t joint_rpm_to_step_hz(const axis_t *ax, float rpm);
static uint32_t ramp_arr(const axis_t *ax, uint32_t from_start, uint32_t to_end);
static void     load_next_period(axis_t *ax);
static void     axis_on_timer_period(axis_t *ax);


/* ======================================================================== */
/* SETUP                                                                    */
/* ======================================================================== */

int motion_init(axis_t *ax)
{
    /* Refuse settings that would cause a divide-by-zero or nonsense units.
     * Nothing is changed if any check fails. */
    if (ax == NULL || ax->tim == NULL || ax->dir_port == NULL)      return -1;
    if (ax->tick_hz <= 0.0f || ax->motor_steps_per_rev <= 0.0f)     return -1;
    if (ax->gear_ratio <= 0.0f || ax->v_start_hz <= 0.0f)           return -1;
    if (ax->step_pulse_us <= 0.0f)                                  return -1;
    if (ax->max_rpm <= 0.0f || ax->max_accel_rpm_s <= 0.0f)         return -1;
    if (ax->default_rpm <= 0.0f || ax->default_rpm > ax->max_rpm)   return -1;
    if (ax->default_accel_rpm_s <= 0.0f ||
        ax->default_accel_rpm_s > ax->max_accel_rpm_s)              return -1;
    if (!(ax->min_deg < ax->max_deg))                               return -1;
    /* 0 deg (the power-up position) must be inside the range, or the joint
     * could never move at all until re-zeroed. */
    if (ax->min_deg > 0.0f || ax->max_deg < 0.0f)                   return -1;
    if (s_axis_count >= MOTION_MAX_AXES)                            return -1;

    /* Unit conversions, worked out once here so the moves don't redo them.
     * One turn of the joint = gear_ratio turns of the motor. */
    float steps_per_joint_rev = ax->motor_steps_per_rev * ax->gear_ratio;
    ax->steps_per_deg = steps_per_joint_rev / 360.0f;
    ax->steps_per_rad = steps_per_joint_rev / 6.2831853f;   /* 2 x pi */

    /* Pulse length in timer ticks, rounded to the nearest tick, minimum 1.
     * At 1 MHz, 5 us -> 5 ticks. */
    ax->pulse_ticks = (uint32_t)(ax->step_pulse_us * ax->tick_hz / 1e6f + 0.5f);
    if (ax->pulse_ticks < 1u) ax->pulse_ticks = 1u;

    /* Largest period this timer can hold. Only matters for very slow speeds:
     * on a 16-bit timer at 1 MHz the slowest step rate is about 15 steps/s. */
    ax->arr_max = IS_TIM_32B_COUNTER_INSTANCE(ax->tim->Instance) ? 0xFFFFFFFFu
                                                                 : 0xFFFFu;

    ax->vstart_sq = ax->v_start_hz * ax->v_start_hz;

    /* Range limits in steps. Rounded inwards (towards 0), so rounding can
     * never let a move end fractionally outside the stated range. */
    ax->min_steps = (int32_t)(ax->min_deg * ax->steps_per_deg);
    ax->max_steps = (int32_t)(ax->max_deg * ax->steps_per_deg);

    /* Start from a known, stopped state. */
    ax->position_steps = 0;
    ax->steps_total    = 0;
    ax->steps_done     = 0;
    ax->step_dir       = 1;
    ax->moving         = 0;
    ax->moves_finished = 0;
    ax->vmax_hz        = 0.0f;

    /* Idle state for the STEP pin: output switched ON but held LOW, counter
     * stopped (see "WHY THE STEP PIN IS HELD LOW" at the top). */
    __HAL_TIM_SET_COMPARE(ax->tim, ax->tim_channel, 0);  /* CCR = 0 -> LOW    */
    ax->tim->Instance->EGR = TIM_EGR_UG;  /* "update event": copy the waiting
                                           * values into use right now        */
    __HAL_TIM_CLEAR_FLAG(ax->tim, TIM_FLAG_UPDATE);  /* that update raised the
                                           * interrupt flag; clear it so no
                                           * false interrupt fires later      */
    /* Switch the channel's output on. Each channel's on/off bit sits 4 bits
     * further along the CCER register, and TIM_CHANNEL_1..4 are defined as
     * 0, 4, 8, 12 — so shifting the channel-1 bit by the channel value
     * picks the right one. (This is the same trick the HAL library uses.) */
    ax->tim->Instance->CCER |= (TIM_CCER_CC1E << (ax->tim_channel & 0x1Fu));

    /* Register the axis so the interrupt can find it. */
    s_axes[s_axis_count++] = ax;

    /* Starting acceleration (checked above, so this can't be refused). */
    motion_set_accel_rpm_s(ax, ax->default_accel_rpm_s);
    return 0;
}

int motion_set_accel_rpm_s(axis_t *ax, float rpm_per_s)
{
    if (ax->moving) return MOTION_ERR_BUSY;   /* never change a move mid-move */
    if (rpm_per_s <= 0.0f || rpm_per_s > ax->max_accel_rpm_s) return MOTION_ERR_LIMIT;

    /* rpm/s of the joint -> steps/s^2 of the motor, then doubled because the
     * ramp formula uses 2a (see ramp_arr). */
    float steps_per_joint_rev = ax->motor_steps_per_rev * ax->gear_ratio;
    ax->two_a = 2.0f * rpm_per_s * steps_per_joint_rev / 60.0f;
    ax->accel_rpm_s = rpm_per_s;
    return MOTION_OK;
}

int motion_set_zero(axis_t *ax)
{
    if (ax->moving) return MOTION_ERR_BUSY;
    ax->position_steps = 0;
    return MOTION_OK;
}


/* ======================================================================== */
/* STARTING MOVES                                                           */
/* ======================================================================== */

int motion_move_steps(axis_t *ax, int32_t n, float rpm)
{
    if (ax->moving) return MOTION_ERR_BUSY;
    if (rpm <= 0.0f || rpm > ax->max_rpm) return MOTION_ERR_LIMIT;

    uint32_t hz = joint_rpm_to_step_hz(ax, rpm);
    if (hz == 0) return MOTION_ERR_LIMIT;   /* so slow it rounds to 0 steps/s */

    /* Where would this move end? Worked out in 64-bit numbers so that even
     * a huge n can't overflow and wrap round into a "valid" answer. */
    int64_t end = (int64_t)ax->position_steps + (int64_t)n;
    if (end < ax->min_steps || end > ax->max_steps) return MOTION_ERR_RANGE;

    if (n == 0)
    {
        /* Already there. Nothing to send, but still count it as a finished
         * move so the PC gets its "DONE" like for any other move. */
        ax->moves_finished++;
        return MOTION_OK;
    }

    /* Direction first, so the DIR pin has settled well before the first step. */
    ax->step_dir = (n > 0) ? 1 : -1;
    set_direction(ax, n > 0);

    ax->steps_total = (uint32_t)(n > 0 ? n : -n);   /* size of move, no sign */
    ax->vmax_hz     = (float)hz;
    ax->moving      = 1;

    /* Period 1: write its values, then force an update event so they are
     * copied into use immediately (normally they'd wait for the next period).
     * The update event also resets the count to 0. */
    __HAL_TIM_SET_AUTORELOAD(ax->tim, ramp_arr(ax, 0, ax->steps_total - 1u));
    __HAL_TIM_SET_COMPARE(ax->tim, ax->tim_channel, ax->pulse_ticks);
    ax->tim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(ax->tim, TIM_FLAG_UPDATE);   /* discard that flag */

    /* In PWM mode the pin goes HIGH the moment the counter starts (count 0 is
     * below CCR), so starting the timer sends pulse 1 by itself. No interrupt
     * marks that pulse, so count it here. */
    ax->steps_done      = 1;
    ax->position_steps += ax->step_dir;
    load_next_period(ax);           /* queue period 2 (or the blank period) */

    __HAL_TIM_ENABLE_IT(ax->tim, TIM_IT_UPDATE);   /* interrupt each period  */
    ax->tim->Instance->CR1 |= TIM_CR1_CEN;          /* start counting -> go   */
    return MOTION_OK;
}

/* Round a number of steps to the nearest whole step. Adding 0.5 and then
 * cutting off the fraction rounds to nearest for positive numbers; negative
 * numbers need 0.5 taken away instead (so -2.6 -> -3, not -2). */
static int32_t round_steps(float steps)
{
    return (steps >= 0.0f) ? (int32_t)(steps + 0.5f) : (int32_t)(steps - 0.5f);
}

int32_t motion_deg_to_steps(const axis_t *ax, float deg)
{
    return round_steps(deg * ax->steps_per_deg);
}

int motion_move_to_deg(axis_t *ax, float deg, float rpm)
{
    /* Absolute target in steps, minus where we are = how far to go. */
    int32_t target = motion_deg_to_steps(ax, deg);
    return motion_move_steps(ax, target - ax->position_steps, rpm);
}

int motion_move_to_rad(axis_t *ax, float rad, float rpm)
{
    int32_t target = round_steps(rad * ax->steps_per_rad);
    return motion_move_steps(ax, target - ax->position_steps, rpm);
}


/* ======================================================================== */
/* STOPPING EARLY                                                           */
/* ======================================================================== */

/* About "pausing interrupts" (used in both functions below):
 *   These functions run in the main loop, but the timer interrupt can jump
 *   in between any two instructions and change the same variables. To make
 *   a few lines happen as one uninterrupted block, interrupts are paused
 *   (__disable_irq) and then put back exactly as they were (saving and
 *   restoring PRIMASK, the processor's "interrupts paused" switch, so this
 *   is also safe if the caller had already paused them). The paused blocks
 *   are only a handful of instructions long — far shorter than the time
 *   between two steps — so no step timing is affected. */

void motion_stop(axis_t *ax)
{
    if (!ax->moving) return;

    /* How fast are we going? The timer's period register (ARR) holds the
     * period of the step already queued up next, so speed = ticks per
     * second / ticks per step. */
    float v = ax->tick_hz / (float)(ax->tim->Instance->ARR + 1u);

    /* How many steps does it take to slow from v down to the starting
     * speed? From v^2 = v0^2 + 2 a s, rearranged: s = (v^2 - v0^2) / 2a. */
    uint32_t stop_steps = 0;
    if (ax->two_a > 0.0f && v > ax->v_start_hz)
        stop_steps = (uint32_t)((v * v - ax->vstart_sq) / ax->two_a);

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (ax->moving)
    {
        /* Shorten the move so it ends after the slow-down. The ramp
         * (ramp_arr) works from "steps left to go", so shortening the move
         * makes it start slowing down straight away, smoothly.
         *
         * The "+ 1" is the step already queued in the timer: it will be
         * sent whatever we do now, so it must stay part of the move or it
         * would be sent but never counted. */
        uint32_t new_total = ax->steps_done + 1u + stop_steps;
        if (new_total < ax->steps_total)   /* only ever shorten, never lengthen */
            ax->steps_total = new_total;
    }
    __set_PRIMASK(primask);
}

void motion_halt(axis_t *ax)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    ax->tim->Instance->CR1 &= ~TIM_CR1_CEN;          /* stop counting        */
    __HAL_TIM_DISABLE_IT(ax->tim, TIM_IT_UPDATE);   /* no more interrupts   */

    /* Make sure the STEP pin ends LOW, even if we stopped in the middle of
     * a pulse: set CCR = 0, then force an update event, which resets the
     * count to 0 and puts CCR = 0 into use. In PWM mode the pin is high
     * only while count < CCR, and 0 < 0 is false, so the pin goes LOW. */
    __HAL_TIM_SET_COMPARE(ax->tim, ax->tim_channel, 0);
    ax->tim->Instance->EGR = TIM_EGR_UG;
    __HAL_TIM_CLEAR_FLAG(ax->tim, TIM_FLAG_UPDATE);  /* discard that update */

    if (ax->moving)
    {
        ax->moving = 0;
        ax->moves_finished++;
    }

    __set_PRIMASK(primask);
}


/* ======================================================================== */
/* CHECKING PROGRESS                                                        */
/* ======================================================================== */

void motion_wait(const axis_t *ax)
{
    while (ax->moving) { __NOP(); }   /* __NOP = "do nothing" instruction */
}

uint8_t motion_busy(const axis_t *ax)
{
    return ax->moving;
}

uint32_t motion_steps_remaining(const axis_t *ax)
{
    if (!ax->moving) return 0;
    return ax->steps_total - ax->steps_done;
}

float motion_get_angle_deg(const axis_t *ax)
{
    return (float)ax->position_steps / ax->steps_per_deg;
}

float motion_get_angle_rad(const axis_t *ax)
{
    return (float)ax->position_steps / ax->steps_per_rad;
}


/* ======================================================================== */
/* TIMER INTERRUPT                                                          */
/* ======================================================================== */

/* The HAL library calls this function whenever ANY timer finishes a period
 * (it's a "weak" function in the HAL, meaning ours replaces the empty
 * default). We look up which axis owns that timer and update it.
 *
 * NOTE: only one HAL_TIM_PeriodElapsedCallback may exist in the whole
 * program. If another timer ever needs one (e.g. a control-loop tick), add
 * it here rather than writing a second copy in another file. */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    for (uint8_t i = 0; i < s_axis_count; i++)
    {
        if (s_axes[i]->tim->Instance == htim->Instance)
        {
            axis_on_timer_period(s_axes[i]);
            return;
        }
    }
}

/* Runs at the START of every timer period for this axis. By this point the
 * hardware has already copied the waiting values into use, so either:
 *   - a real pulse has just started  -> count it, queue the next period, or
 *   - the blank period has started   -> the move is over, stop.            */
static void axis_on_timer_period(axis_t *ax)
{
    if (ax->steps_done >= ax->steps_total)
    {
        /* Blank period has begun: every pulse has been sent.
         * Stop the COUNTER only. The output stays switched on with CCR = 0,
         * so the STEP pin is driven LOW while idle (never left floating). */
        ax->tim->Instance->CR1 &= ~TIM_CR1_CEN;
        __HAL_TIM_DISABLE_IT(ax->tim, TIM_IT_UPDATE);
        ax->moving = 0;
        ax->moves_finished++;
        return;
    }

    /* A real pulse has just started. */
    ax->steps_done++;
    ax->position_steps += ax->step_dir;

    load_next_period(ax);
}


/* ======================================================================== */
/* HELPERS (only used inside this file)                                     */
/* ======================================================================== */

/* Set the DIR pin. dir_invert swaps the meaning, for joints whose motor is
 * mounted or wired the other way round. */
static void set_direction(axis_t *ax, uint8_t positive)
{
    uint8_t level = ax->dir_invert ? !positive : positive;
    HAL_GPIO_WritePin(ax->dir_port, ax->dir_pin, level ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* Joint speed in rpm -> motor step rate in steps per second.
 * rpm x (steps per joint turn) / 60 seconds. */
static uint32_t joint_rpm_to_step_hz(const axis_t *ax, float rpm)
{
    return (uint32_t)(rpm * ax->motor_steps_per_rev * ax->gear_ratio / 60.0f);
}

/* Work out the timer period (ARR) for one step of the move.
 *
 *   from_start = how many steps were sent before this one
 *   to_end     = how many steps will still follow this one
 *
 * Constant acceleration obeys  v^2 = v0^2 + 2 a s  (s = distance, here in
 * steps). We work out three speeds and use the smallest:
 *   v_up : the fastest we can be going after speeding up from the start
 *   v_dn : the fastest we can be going and still slow down in time to stop
 *   vmax : the requested cruise speed
 * Taking the smallest gives the trapezoid shape (up, flat, down). A short
 * move never reaches vmax, so v_up and v_dn meet in the middle and the shape
 * becomes a triangle automatically. */
static uint32_t ramp_arr(const axis_t *ax, uint32_t from_start, uint32_t to_end)
{
    float v_up = sqrtf(ax->vstart_sq + ax->two_a * (float)from_start);
    float v_dn = sqrtf(ax->vstart_sq + ax->two_a * (float)to_end);
    float v    = (v_up < v_dn) ? v_up : v_dn;
    if (v > ax->vmax_hz) v = ax->vmax_hz;

    /* Speed (steps/s) -> period in ticks. The timer counts 0..ARR, which is
     * ARR + 1 ticks, hence the "- 1". "+ 0.5" rounds to the nearest tick. */
    float ticks = ax->tick_hz / v + 0.5f;

    /* Too slow for this timer to count? Use the longest period it can hold
     * (the step comes slightly early rather than the count overflowing). */
    if (ticks > (float)ax->arr_max) return ax->arr_max;

    uint32_t arr = (uint32_t)ticks - 1u;

    /* Too fast? Keep the LOW gap between pulses at least as long as the
     * pulse itself, so the driver always sees a clean high-then-low. */
    if (arr < 2u * ax->pulse_ticks) arr = 2u * ax->pulse_ticks;
    return arr;
}

/* Write the waiting (preload) values for the period AFTER the one now
 * running — see "WHY PRELOAD MATTERS" at the top of this file. */
static void load_next_period(axis_t *ax)
{
    if (ax->steps_done >= ax->steps_total)
    {
        /* Every pulse has been launched: make the next period blank
         * (CCR = 0 -> pin stays LOW). When it begins, the interrupt stops the
         * timer. ARR is left as it was. */
        __HAL_TIM_SET_COMPARE(ax->tim, ax->tim_channel, 0);
    }
    else
    {
        uint32_t from_start = ax->steps_done;
        uint32_t to_end     = ax->steps_total - ax->steps_done - 1u;
        __HAL_TIM_SET_AUTORELOAD(ax->tim, ramp_arr(ax, from_start, to_end));
        __HAL_TIM_SET_COMPARE(ax->tim, ax->tim_channel, ax->pulse_ticks);
    }
}

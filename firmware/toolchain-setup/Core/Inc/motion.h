/**
 * motion.h — Step generation and smooth (trapezoidal) moves for stepper axes
 * ==========================================================================
 *
 * WHAT THIS MODULE DOES
 *   Turns "move this joint to 90 degrees at 30 rpm" into an exact train of
 *   STEP pulses, with a gentle speed-up at the start and slow-down at the end.
 *   One hardware timer per motor produces the pulses, so the timing does not
 *   depend on what the main program is busy doing.
 *
 * HOW IT IS ORGANISED
 *   Everything about one motor lives in one bundle of settings and state,
 *   called an `axis_t` (defined below). Every function takes a pointer to the
 *   axis it should act on. With one motor there is one axis_t; with the full
 *   arm there will be six, and this file does not need to change.
 *
 * HOW TO USE IT (example for one joint)
 *   1. In main.c, fill in an axis_t with the timer, pins and units for that
 *      motor (see `joint1` in main.c).
 *   2. After CubeMX has set up the timers and pins, call motion_init(&joint1).
 *   3. Set an acceleration:  motion_set_accel_rpm_s(&joint1, 120.0f);
 *   4. Move:                 motion_move_to_deg(&joint1, 90.0f, 30.0f);
 *   5. Either wait:          motion_wait(&joint1);
 *      or keep working and check later: if (!motion_busy(&joint1)) { ... }
 *
 * UNITS
 *   - Inside this module, position is counted in microsteps (whole numbers,
 *     so they never drift from rounding).
 *   - Outside it, you work in degrees or radians OF THE JOINT, and speeds in
 *     rpm OF THE JOINT. The gear ratio converts between joint and motor.
 *     While gear_ratio = 1.0 (bare motor on the bench), joint = motor shaft.
 *
 * EVERY ACCEPTED MOVE ENDS EXACTLY ONCE
 *   Each move that is accepted (returns MOTION_OK) adds 1 to `moves_finished`
 *   when it ends — whether it ran to the end, was stopped, or was halted, and
 *   even if it was zero steps long. The command module watches this counter
 *   to tell the PC "DONE". A counter is used rather than watching `moving`
 *   change from 1 to 0, because a very short move could start and finish
 *   between two checks and the change would be missed.
 *
 * SAFETY NOTE
 *   The timer keeps producing steps by itself once started. Pausing the
 *   debugger or a crash in the main loop does NOT stop the motor. Stopping
 *   motion safely is the job of the safety stage (Stage F) — not yet built.
 *   motion_stop() and motion_halt() below are normal controls, not a
 *   replacement for that: they rely on the program still running.
 */

#ifndef MOTION_H
#define MOTION_H

#include "main.h"      /* HAL types (TIM_HandleTypeDef, GPIO_TypeDef), pin names */
#include <stdint.h>

/* The most axes the module can look after. The PAROL6 has six joints. */
#define MOTION_MAX_AXES   6

/* Results returned by functions that can refuse a request. */
#define MOTION_OK          0    /* accepted                                  */
#define MOTION_ERR_BUSY   (-1)  /* refused: the axis is still moving         */
#define MOTION_ERR_LIMIT  (-2)  /* refused: speed or acceleration is zero,
                                 * negative, or above the axis's limit       */

/**
 * axis_t — everything about one motor.
 *
 * The fields are in three groups:
 *   1. SETTINGS you fill in before calling motion_init().
 *   2. VALUES WORKED OUT by motion_init() from those settings. Don't set these.
 *   3. LIVE STATE that changes during a move. Read it through the functions
 *      below (motion_get_angle_deg, motion_busy, ...) rather than directly.
 *
 * Live-state fields are marked `volatile`. That keyword tells the compiler
 * "this value can change at any moment behind your back" (here: inside the
 * timer interrupt). Without it, the compiler may keep an old copy in a
 * register, and a loop like motion_wait() could wait forever.
 */
typedef struct
{
    /* ---- 1. SETTINGS (fill these in) ------------------------------------ */

    /* Hardware: which timer output makes the STEP pulses, and which pin sets
     * the direction. */
    TIM_HandleTypeDef *tim;          /* e.g. &htim2                          */
    uint32_t           tim_channel;  /* e.g. TIM_CHANNEL_1 (wired to STEP)   */
    GPIO_TypeDef      *dir_port;     /* e.g. DIR_GPIO_Port                   */
    uint16_t           dir_pin;      /* e.g. DIR_Pin                         */
    uint8_t            dir_invert;   /* 1 = swap which way is "positive".
                                      * Use this if a joint turns the wrong
                                      * way, instead of rewiring the motor.  */

    /* Timer clock: how many times per second the timer counts up by one.
     * Must match the prescaler set in CubeMX:
     *   tick rate = timer input clock / (prescaler + 1).
     * For TIM2 now: 90 MHz / (89 + 1) = 1,000,000 ticks per second.        */
    float tick_hz;

    /* Units */
    float motor_steps_per_rev;  /* microsteps for one turn of the MOTOR shaft
                                 * = 200 full steps x microstep setting
                                 * (256 now -> 51200).                       */
    float gear_ratio;           /* motor turns per one turn of the JOINT.
                                 * 1.0 on the bare bench motor. Set to the
                                 * joint's real reduction once it's built.   */

    /* Pulse shape and the slowest speed used */
    float step_pulse_us;        /* how long each STEP pulse stays high,
                                 * in microseconds (5 now). The TMC5160 needs
                                 * only a fraction of a microsecond, so this
                                 * is generous on purpose.                   */
    float v_start_hz;           /* speed of the very first and very last
                                 * step, in steps per second. Must be above
                                 * zero: a speed of zero would mean an
                                 * infinitely long wait for the first step.  */

    /* Limits. Requests above these are refused, so a typing mistake on the
     * PC (say 600 rpm instead of 60) can't throw the joint around.          */
    float max_rpm;              /* fastest allowed JOINT speed, rpm          */
    float max_accel_rpm_s;      /* largest allowed JOINT acceleration, rpm/s */

    /* ---- 2. WORKED OUT by motion_init() -- do not set ------------------- */

    float    steps_per_deg;     /* microsteps per degree of JOINT rotation   */
    float    steps_per_rad;     /* microsteps per radian of JOINT rotation   */
    uint32_t pulse_ticks;       /* step_pulse_us converted to timer ticks    */
    uint32_t arr_max;           /* largest period the timer can hold
                                 * (TIM2 and TIM5 count to 32 bits; the other
                                 * timers only to 16 bits = 65,535)          */
    float    vstart_sq;         /* v_start_hz squared (used in the ramp sum) */
    float    two_a;             /* 2 x acceleration, in steps per second^2   */

    /* ---- 3. LIVE STATE (changes during moves) --------------------------- */

    volatile int32_t  position_steps;  /* where the motor is, in microsteps,
                                        * counted from where it was at power-
                                        * up (until homing exists, Stage E)  */
    volatile uint32_t steps_total;     /* size of the current move, in steps */
    volatile uint32_t steps_done;      /* steps already sent in this move    */
    volatile int8_t   step_dir;        /* +1 or -1: added to position per step */
    volatile uint8_t  moving;          /* 1 from the start of a move until the
                                        * timer has fully stopped            */
    volatile uint32_t moves_finished;  /* +1 each time a move ends (see
                                        * "EVERY ACCEPTED MOVE ENDS EXACTLY
                                        * ONCE" at the top of this file)     */
    float             vmax_hz;         /* cruise speed of this move, steps/s */
} axis_t;


/* ---- Setup ------------------------------------------------------------- */

/* Prepare one axis. Call once, after CubeMX's MX_TIMx_Init() and
 * MX_GPIO_Init(). Works out the unit conversions, makes the STEP pin sit LOW
 * while idle, and registers the axis so the timer interrupt can find it.
 * Returns 0 if OK, or -1 if the settings are unusable (nothing is changed). */
int motion_init(axis_t *ax);

/* Set how quickly speed builds up and dies away, in JOINT rpm per second.
 * Example: 120 means it takes 0.25 s to go from 0 to 30 rpm.
 * Only takes effect for moves started after this call.
 * Returns MOTION_OK, MOTION_ERR_BUSY (refused while moving, so a move never
 * changes shape halfway through) or MOTION_ERR_LIMIT. */
int motion_set_accel_rpm_s(axis_t *ax, float rpm_per_s);

/* Declare the current position to be 0 degrees. Used until homing exists
 * (Stage E): line the joint up by eye, then call this.
 * Returns MOTION_OK, or MOTION_ERR_BUSY if the axis is moving. */
int motion_set_zero(axis_t *ax);


/* ---- Starting moves ---------------------------------------------------- */
/* All three return straight away; the timer does the rest in the background.
 * Speeds are the cruise (top) speed in JOINT rpm; short moves may never
 * reach it.
 * Each returns MOTION_OK (move started), MOTION_ERR_BUSY (the last move is
 * still running — nothing changed) or MOTION_ERR_LIMIT (speed out of range
 * — nothing changed). */

/* Move by a signed number of microsteps from wherever the axis is now. */
int motion_move_steps(axis_t *ax, int32_t n, float rpm);

/* Move to an absolute JOINT angle in degrees (handy on the bench). */
int motion_move_to_deg(axis_t *ax, float deg, float rpm);

/* Move to an absolute JOINT angle in radians (what the kinematics code,
 * which works in radians, will use). */
int motion_move_to_rad(axis_t *ax, float rad, float rpm);


/* ---- Stopping early ---------------------------------------------------- */

/* Slow down to a stop using the normal deceleration, then stop. Returns
 * straight away; the axis keeps moving briefly while it slows. This is the
 * normal way to stop: no steps are lost, so the position stays correct.
 * Does nothing if the axis is already still. */
void motion_stop(axis_t *ax);

/* Stop sending steps IMMEDIATELY, with no slow-down. For when stopping
 * NOW matters more than accuracy. Warnings:
 *   - A loaded joint can't stop instantly, so it may slip past by some
 *     steps. The position count will then be wrong until re-homed.
 *   - The step pulse in progress may be cut short; the count can be out by
 *     one microstep (0.007 degrees).
 * Does nothing if the axis is already still. */
void motion_halt(axis_t *ax);


/* ---- Checking progress ------------------------------------------------- */

/* Wait here until the move has finished. Simple, but nothing else in the
 * main loop runs while waiting — fine for bench tests, not for the
 * multi-axis or serial-command stages. */
void motion_wait(const axis_t *ax);

/* 1 while a move is running, 0 once it has finished. Returns immediately,
 * so the main loop can keep doing other work in between checks. */
uint8_t motion_busy(const axis_t *ax);

/* How many steps of the current move are still to be sent (0 when idle). */
uint32_t motion_steps_remaining(const axis_t *ax);

/* Current JOINT angle. */
float motion_get_angle_deg(const axis_t *ax);
float motion_get_angle_rad(const axis_t *ax);

#endif /* MOTION_H */

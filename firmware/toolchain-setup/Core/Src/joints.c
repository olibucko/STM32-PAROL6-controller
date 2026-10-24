/**
 * joints.c — The robot's joints: fixed settings for each one
 * ==========================================================
 * See joints.h for why these live in the firmware and not on the PC.
 *
 * Source for the PAROL6 figures: "Robot Specifications - PAROL6 DOCS.pdf"
 * (in the repo's resources folder) — joint limits table and the joint
 * reduction ratios table.
 *
 * UNITS REMINDER
 *   Angles and speeds here are for the JOINT (after the belt or gearbox).
 *   The motor turns gear_ratio times faster than the joint.
 */

#include "joints.h"
#include "main.h"            /* pin names (DIR_Pin, ...) */

extern TIM_HandleTypeDef htim2;   /* created by CubeMX in main.c */


/* ---------------------------------------------------------------------- */
/* JOINT 1 — base rotation                                                */
/* ---------------------------------------------------------------------- */
/* BENCH SETUP (24 Oct 2026): bare motor shaft, no belt. gear_ratio = 1,
 * so every "joint" angle and speed below is simply the motor shaft's.
 *
 * Speed limits: 60 rpm and 600 rpm/s are the fastest settings tested on the
 * bench so far. Raise them only after measuring the step-rate budget
 * (roadmap item 4): at 256 microsteps, 60 rpm is already about 51,000 steps
 * per second.
 *
 * >>> WHEN THE MOTOR GOES INTO THE ASSEMBLED BASE, change together:
 * >>>   gear_ratio      1.0     -> 6.4   (PAROL6 J1 belt reduction)
 * >>>   max_rpm         60      -> 9.375 (= 60 / 6.4, same motor speed)
 * >>>   max_accel       600     -> 93.75 (= 600 / 6.4)
 * >>>   default_rpm     30      -> 4.6875
 * >>>   default_accel   120     -> 18.75
 * >>>   min/max_deg     +/-180  -> +/-123.046875 (see range limits below)
 * >>> Changing the gear ratio alone would make the same rpm numbers drive
 * >>> the motor 6.4 times faster. */
static axis_t joint1 =
{
    /* Hardware */
    .tim                 = &htim2,          /* TIM2 makes the STEP pulses      */
    .tim_channel         = TIM_CHANNEL_1,   /* ...on channel 1 = pin PA0       */
    .dir_port            = DIR_GPIO_Port,   /* DIR on pin PA1                  */
    .dir_pin             = DIR_Pin,
    .dir_invert          = 0,               /* 1 to flip which way is "+".
                                             * Check against the PAROL6 docs'
                                             * positive-rotation diagram once
                                             * the base is assembled.          */

    /* Timer clock: 90 MHz / (prescaler 89 + 1) = 1 MHz. Must match CubeMX.  */
    .tick_hz             = 1000000.0f,

    /* Units */
    .motor_steps_per_rev = 51200.0f,        /* 200 full steps x 256 microsteps */
    .gear_ratio          = 1.0f,            /* bare shaft (base: 6.4, see top) */

    /* Pulse and slowest speed */
    .step_pulse_us       = 5.0f,            /* STEP high time, microseconds    */
    .v_start_hz          = 500.0f,          /* first and last step rate        */

    /* Speed limits (see note above) and the defaults used when a command
     * doesn't give a speed (30 rpm / 120 rpm/s, as used on the bench). */
    .max_rpm             = 60.0f,
    .max_accel_rpm_s     = 600.0f,
    .default_rpm         = 30.0f,
    .default_accel_rpm_s = 120.0f,

    /* Range limits.
     * BENCH VALUES (24 Oct 2026): -180 to +180 degrees of the bare shaft
     * (half a turn each way).
     *
     * >>> BEFORE THE MOTOR GOES INTO THE ASSEMBLED BASE, change these to the
     * >>> PAROL6 spec: -123.046875 to +123.046875 degrees (0 = standby, base
     * >>> facing straight ahead). Beyond that, the base hits its end stop.
     *
     * Only meaningful once 0 is in the right place: until homing exists
     * (Stage E), line the joint up by eye and send ZERO 1. */
    .min_deg             = -180.0f,
    .max_deg             =  180.0f,
};


/* ---------------------------------------------------------------------- */
/* The list of joints, in order. Joint 1 first.                           */
/* For the full arm: { &joint1, &joint2, ..., &joint6 }                   */
/* ---------------------------------------------------------------------- */
axis_t *const g_joints[] = { &joint1 };
const uint8_t g_joint_count = sizeof g_joints / sizeof g_joints[0];

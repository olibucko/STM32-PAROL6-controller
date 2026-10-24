/**
 * joints.h — The robot's joints: fixed settings for each one
 * ==========================================================
 *
 * Every joint's hardware wiring, units, speed limits and range limits are
 * written into joints.c and built into the firmware. They can be READ from
 * the PC (the INFO command) but deliberately can NOT be changed from it:
 * safety limits must only change by editing joints.c and re-flashing, never
 * because of a typing mistake or a bug in a PC script.
 *
 * To add a joint: add its axis_t block to joints.c and add it to
 * g_joints[]. Nothing else needs to change.
 */

#ifndef JOINTS_H
#define JOINTS_H

#include "motion.h"

/* All joints, in order: g_joints[0] is joint 1. */
extern axis_t *const g_joints[];
extern const uint8_t g_joint_count;

#endif /* JOINTS_H */

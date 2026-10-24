/**
 * command.h — Commands typed on (or sent by) the PC, and the replies
 * ==================================================================
 *
 * WHAT THIS MODULE DOES
 *   Takes one line of text (from serial.c), works out which command it is,
 *   checks the numbers, carries it out using motion.c / tmc5160.c, and sends
 *   back a reply. It also tells the PC when each move has finished.
 *
 * THE RULES OF THE CONVERSATION (the "protocol")
 *   - One command per line. Words are separated by spaces. Upper or lower
 *     case both work. Joints are numbered from 1.
 *   - Every command gets exactly ONE reply line, starting with either
 *       OK  ...            the command was accepted
 *       ERR <reason>       it was refused, and nothing changed
 *   - A move's OK says where it starts and ends, so a surprise (like a long
 *     trip back from a far-off position) is visible straight away:
 *       OK 1 from=<degrees> to=<degrees>
 *   - When a move ends (finished, stopped or halted) the controller sends,
 *     by itself:
 *       DONE <joint> pos=<degrees>
 *     Every move that got an OK gets exactly one DONE later.
 *   - Lines starting with "#" are information for a human (start-up
 *     message, HELP text). Programs can ignore them.
 *
 * COMMANDS  (j = joint number, [ ] = optional)
 *   HELP                      list the commands
 *   PING                      check the link: replies OK PONG
 *   STATUS j                  position, moving or not, steps left, settings
 *   INFO j                    the joint's fixed settings (range, gear ratio,
 *                             limits). Read-only: these are set in joints.c.
 *   MOVE j deg [rpm]          go to an ABSOLUTE angle (0 = standby / ZERO)
 *   MOVEREL j deg [rpm]       move by an amount from where it is now
 *   STEPS j n [rpm]           move by n microsteps (for tests)
 *   SPEED j rpm               set the speed used when [rpm] is left out
 *   ACCEL j rpm_per_s         set the acceleration (only while still)
 *   ZERO j                    call the current position 0 degrees
 *   STOP [j]                  slow down and stop (all joints if no j)
 *   HALT [j]                  stop instantly (see motion_halt warnings)
 *   RREG j reg                read a motor-driver register
 *   WREG j reg value          write a motor-driver register (only while still)
 *   Numbers starting "0x" are read as hexadecimal, e.g. RREG 1 0x6C.
 *
 * EXAMPLE
 *   PC sends:   MOVE 1 90 5
 *   Reply:      OK 1 from=0.000 to=90.000
 *   ...about three seconds later...
 *               DONE 1 pos=90.000
 *
 * ANGLES AND SPEEDS ARE FOR THE JOINT, not the motor (see joints.c).
 */

#ifndef COMMAND_H
#define COMMAND_H

#include "motion.h"
#include <stdint.h>

/* Hand the module the list of axes. axes[0] becomes joint 1, axes[1]
 * joint 2, and so on. Each joint starts at its own default speed (set in
 * joints.c). Call once, after motion_init() for every axis. */
void command_init(axis_t *const axes[], uint8_t count);

/* Carry out one command line and send its reply. */
void command_execute(const char *line);

/* Send a "DONE" line for every move that has ended since the last call.
 * Call on every pass of the main loop. */
void command_report_finished_moves(void);

/* Slow every joint to a stop (used by the B1 button). */
void command_stop_all(void);

#endif /* COMMAND_H */

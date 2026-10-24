/**
 * serial.h — Text lines in and out over the USB serial link to the PC
 * ===================================================================
 *
 * WHAT THIS MODULE DOES
 *   Moves characters between the STM32 and the PC, and groups incoming
 *   characters into whole lines (one line = one command). It knows nothing
 *   about what the commands MEAN — that is command.c's job.
 *
 * THE LINK
 *   USART2 (pins PA2/PA3) is wired on the Nucleo board to the ST-LINK chip,
 *   which shows up on the PC as a COM port over the same USB cable used for
 *   flashing. Settings: 115200 baud, 8 data bits, no parity, 1 stop bit.
 *
 * RECEIVING (why an interrupt and a buffer)
 *   The UART hardware holds only ONE received character. If the program does
 *   not collect it before the next one arrives (about 87 microseconds later
 *   at 115200 baud), it is overwritten and lost. So each arriving character
 *   triggers an interrupt that copies it into a 128-character waiting area
 *   (a "ring buffer"). The main loop then reads from that area whenever it
 *   gets round to it.
 *
 * SENDING
 *   serial_print() waits until its text has been sent (about 87 us per
 *   character). Replies are short, so this is acceptable for now.
 */

#ifndef SERIAL_H
#define SERIAL_H

#include "main.h"
#include <stdint.h>

/* Longest command line accepted, in characters. Longer lines are thrown
 * away and reported as an error. */
#define SERIAL_LINE_MAX   80

/* Start receiving. Call once, after MX_USART2_UART_Init(). */
void serial_init(UART_HandleTypeDef *huart);

/* Check for a complete line. Returns immediately.
 *   Returns 1 and copies the line (without the line-ending characters) into
 *   `out` once a whole line has arrived.
 *   Returns 0 if no complete line is waiting yet.
 *   Returns -1 if a line was too long and had to be thrown away.
 * A line ends at either '\r' or '\n', so PC programs that send "\r\n" (most
 * Windows terminals) and those that send just "\n" both work. Empty lines
 * are skipped. */
int serial_read_line(char *out, uint32_t out_size);

/* Send text followed by "\r\n" (a line ending the PC will recognise). */
void serial_println(const char *text);

/* The interrupt handler for USART2 calls this (see stm32f4xx_it.c).
 * Not for use anywhere else. */
void serial_rx_isr(void);

#endif /* SERIAL_H */

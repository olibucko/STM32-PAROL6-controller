/**
 * serial.c — Text lines in and out over the USB serial link to the PC
 * ===================================================================
 * See serial.h for what this module does. This file explains HOW.
 *
 * THE RING BUFFER
 *   A fixed array plus two position counters:
 *     s_head = where the interrupt will put the NEXT arriving character
 *     s_tail = where the main loop will take the NEXT character from
 *   Both wrap round to 0 at the end of the array, hence "ring".
 *     empty when head == tail
 *     full  when head is one place behind tail (one slot is kept unused so
 *           that "full" and "empty" look different)
 *   Only the interrupt changes s_head and only the main loop changes s_tail,
 *   so the two never fight over the same value and no locking is needed.
 *
 * WHY NOT THE HAL's OWN RECEIVE FUNCTION (HAL_UART_Receive_IT)?
 *   It is built for "receive exactly N characters, then stop". For an endless
 *   stream it has to be restarted after every character, and if a receive
 *   error ever happens it stops and stays stopped until restarted — a common
 *   cause of a serial link that silently goes dead. Reading the hardware
 *   register directly, as below, has neither problem.
 */

#include "serial.h"
#include <string.h>

#define RX_BUF_SIZE  128u    /* a power of two, so wrapping is a cheap mask */

static UART_HandleTypeDef *s_huart = 0;

static volatile uint8_t  s_rx_buf[RX_BUF_SIZE];
static volatile uint32_t s_head = 0;   /* written only by the interrupt   */
static volatile uint32_t s_tail = 0;   /* written only by the main loop   */

/* The line being assembled, character by character, in the main loop. */
static char     s_line[SERIAL_LINE_MAX + 1];
static uint32_t s_line_len  = 0;
static uint8_t  s_line_overflow = 0;   /* 1 = current line got too long   */


void serial_init(UART_HandleTypeDef *huart)
{
    s_huart = huart;

    /* Read and discard anything already sitting in the hardware, so we
     * start clean. Reading the status register (SR) and then the data
     * register (DR), in that order, also clears any old error flags. */
    (void)huart->Instance->SR;
    (void)huart->Instance->DR;

    /* Ask the UART to raise an interrupt each time a character arrives
     * ("RXNE" = Receive register Not Empty). The interrupt itself was
     * switched on in CubeMX (NVIC -> USART2 global interrupt). */
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);
}

/* Runs inside the USART2 interrupt, once per arriving character.
 * Kept as short as possible: it only stores the character. */
void serial_rx_isr(void)
{
    if (s_huart == 0) return;

    uint32_t sr = s_huart->Instance->SR;
    if (sr & (USART_SR_RXNE | USART_SR_ORE))
    {
        /* Reading DR collects the character AND clears the flags, so the
         * interrupt doesn't fire again for the same character. (ORE =
         * "overrun": a character was lost because the last one wasn't
         * collected in time. Reading DR also clears that.) */
        uint8_t c = (uint8_t)s_huart->Instance->DR;

        uint32_t next = (s_head + 1u) & (RX_BUF_SIZE - 1u);
        if (next != s_tail)            /* room in the buffer?            */
        {
            s_rx_buf[s_head] = c;
            s_head = next;
        }
        /* else: buffer full, character dropped. The resulting broken
         * command will be rejected by the command parser, and the PC
         * will see an ERR reply. */
    }
}

int serial_read_line(char *out, uint32_t out_size)
{
    /* Take every waiting character out of the ring buffer, one at a time,
     * until a line is complete or the buffer is empty. */
    while (s_tail != s_head)
    {
        char c = (char)s_rx_buf[s_tail];
        s_tail = (s_tail + 1u) & (RX_BUF_SIZE - 1u);

        if (c == '\r' || c == '\n')
        {
            /* End of a line. */
            if (s_line_overflow)
            {
                s_line_overflow = 0;
                s_line_len = 0;
                return -1;                     /* report "too long"      */
            }
            if (s_line_len == 0) continue;     /* empty line: skip it    */

            s_line[s_line_len] = '\0';         /* end the C string       */
            strncpy(out, s_line, out_size - 1u);
            out[out_size - 1u] = '\0';
            s_line_len = 0;
            return 1;
        }

        /* Ordinary character: add it to the line, if it still fits. */
        if (s_line_len < SERIAL_LINE_MAX)
            s_line[s_line_len++] = c;
        else
            s_line_overflow = 1;  /* keep reading until the line ends,
                                   * then throw the whole line away */
    }
    return 0;   /* no complete line yet */
}

void serial_println(const char *text)
{
    if (s_huart == 0) return;
    HAL_UART_Transmit(s_huart, (const uint8_t *)text, (uint16_t)strlen(text), 100);
    HAL_UART_Transmit(s_huart, (const uint8_t *)"\r\n", 2, 100);
    /* 100 ms time limit: if sending ever gets stuck, give up rather than
     * freeze the whole program (HAL_MAX_DELAY would wait forever). */
}

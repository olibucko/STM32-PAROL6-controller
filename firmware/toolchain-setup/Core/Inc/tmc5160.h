#ifndef TMC5160_H
#define TMC5160_H

#include "main.h"
#include "tmc5160_regs.h"
#include <stdint.h>

/* Last status byte returned by the chip (rx[0]) */
extern uint8_t tmc_status;

void     tmc5160_write(uint8_t reg, uint32_t value);
uint32_t tmc5160_read(uint8_t reg);

#endif

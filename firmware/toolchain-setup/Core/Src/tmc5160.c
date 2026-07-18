#include "tmc5160.h"

extern SPI_HandleTypeDef hspi2;

uint8_t tmc_status = 0;

static void cs_low(void)  { HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_SET); }

/* One 40-bit datagram. Returns the 32 data bits; stashes status in tmc_status. */
static uint32_t xfer(uint8_t addr, uint32_t value)
{
    uint8_t tx[5], rx[5];

    tx[0] = addr;
    tx[1] = (value >> 24) & 0xFF;
    tx[2] = (value >> 16) & 0xFF;
    tx[3] = (value >>  8) & 0xFF;
    tx[4] =  value        & 0xFF;

    cs_low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 5, HAL_MAX_DELAY);
    cs_high();

    tmc_status = rx[0];

    return ((uint32_t)rx[1] << 24) |
           ((uint32_t)rx[2] << 16) |
           ((uint32_t)rx[3] <<  8) |
            (uint32_t)rx[4];
}

void tmc5160_write(uint8_t reg, uint32_t value)
{
    xfer(reg | TMC_WRITE_BIT, value);
}

uint32_t tmc5160_read(uint8_t reg)
{
    xfer(reg, 0);          // request — reply is stale, discard
    return xfer(reg, 0);   // collect — this is the real value
}
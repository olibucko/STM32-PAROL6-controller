#include "tmc5160.h"

extern SPI_HandleTypeDef hspi2;

uint8_t tmc_status = 0;

static void cs_low(void)  { HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_RESET); }
static void cs_high(void) { HAL_GPIO_WritePin(TMC_CS_GPIO_Port, TMC_CS_Pin, GPIO_PIN_SET); }

/* One 40-bit datagram. Returns the 32 data bits; stashes status in tmc_status. */
static uint32_t xfer(uint8_t addr, uint32_t value)
{
    uint8_t tx[5], rx[5];

    /* Use bit shifting ">>" to break up transferred value into 4 bytes (8 bits each).
     * Shift the wanted byte down, mask to keep the low 8 bits */
    tx[0] = addr;
    tx[1] = (value >> 24) & 0xFF;
    tx[2] = (value >> 16) & 0xFF;
    tx[3] = (value >>  8) & 0xFF;
    tx[4] =  value        & 0xFF;

    cs_low();
    HAL_SPI_TransmitReceive(&hspi2, tx, rx, 5, HAL_MAX_DELAY);
    cs_high();

    tmc_status = rx[0];

    /* Reassemble the 4 received bytes back into one 32-bit value by shifting each into its slot, using OR ("|") to merge */
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
    /* TMC reads need two MOSI/MISO cycles. The first communicates the address to read, and the second transfers the read value.
     * SPI sends and receives simultaneously, so the requested value only arrives on the following transfer.
     * This is why the read is performed twice, with the function returning the value the second time. */
    xfer(reg, 0);          // request — reply is stale, discard
    return xfer(reg, 0);   // collect — this is the real value
}

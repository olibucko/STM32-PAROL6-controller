#ifndef TMC5160_REGS_H
#define TMC5160_REGS_H

#define TMC_WRITE_BIT       0x80


/* General config */
#define TMC_GCONF           0x00    // RW
#define TMC_GSTAT           0x01    // R+C
#define TMC_IOIN            0x04    // R  (bits 31:24 = version, 0x30)
#define TMC_GLOBALSCALER  	0x0B
#define TMC_TPWMTHRS      	0x13

/* Current control */
#define TMC_IHOLD_IRUN      0x10    // W
#define TMC_TPOWERDOWN      0x11    // W
#define TMC_TSTEP           0x12    // R
#define TMC_TPWMTHRS        0x13    // W

/* Ramp generator */
#define TMC_RAMPMODE        0x20    // RW
#define TMC_XACTUAL         0x21    // RW
#define TMC_VACTUAL         0x22    // R
#define TMC_VSTART          0x23    // W
#define TMC_A1              0x24    // W
#define TMC_V1              0x25    // W
#define TMC_AMAX            0x26    // W
#define TMC_VMAX            0x27    // W
#define TMC_DMAX            0x28    // W
#define TMC_D1              0x2A    // W
#define TMC_VSTOP           0x2B    // W
#define TMC_XTARGET         0x2D    // RW

/* Driver */
#define TMC_CHOPCONF        0x6C    // RW
#define TMC_COOLCONF        0x6D    // W
#define TMC_DRV_STATUS      0x6F    // R
#define TMC_MSCNT        0x6A   // live microstep counter (read-only)

#endif

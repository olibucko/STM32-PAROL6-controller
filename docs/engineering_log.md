# PAROL6 Controller — Engineering Log

Dated lab notebook for `stm32-PAROL6-controller`. Records what was tried, what happened, and **why** — including dead ends. Code changes themselves live in git history; this log holds the reasoning.

**Conventions**
- Newest entry at the top.
- Each entry: Objective → Work done → Results → Findings → Next.
- Status tags: `VERIFIED` (bench-confirmed), `UNTESTED` (written, not run), `THEORY` (hypothesis, not yet proven), `DEAD END` (disproven — kept for the record).
- Commit the log alongside the code it describes.

---

## 2026-10-21 — Stage D2: Smooth acceleration/deceleration (single axis)

**Status:** `VERIFIED` — smooth moves, exact step count, no drift at rest. Bare shaft, no load.

### Goal
Before this, every move started and stopped at full speed instantly. That asks the motor for infinite acceleration, which it can't deliver once there's a load on it — it would skip steps without the firmware knowing. The goal was to make each move speed up gradually, cruise, then slow down gradually ("trapezoidal" speed profile).

### How it works
- **Speed of each step depends on where it sits in the move.** For every step the code works out three speeds and uses the smallest:
  1. how fast it could be going after speeding up from the start: √(v₀² + 2a × steps done)
  2. how fast it can be going and still slow down in time for the end: √(v₀² + 2a × steps left)
  3. the requested cruise speed.
  These come from the standard constant-acceleration equation v² = v₀² + 2as, with distance measured in steps. Short moves never reach cruise speed and become a triangle shape automatically.
- **TIM2 makes the pulses in hardware.** After each step, an interrupt (a function the hardware triggers automatically) counts the step and sets up the timing of the *next* step. Timer "preload" is turned on so a new timing value only takes effect at the start of the next step, never halfway through one.
- **Clean finish:** after the last real step, the timer runs one extra empty period with no pulse, and stops at the end of it. This guarantees exactly the requested number of pulses.
- Settings: first/last step at 500 steps/s, fixed 5 µs pulse, test acceleration 120 rpm/s (0 → 30 rpm in 0.25 s).
- Motion now only runs when the blue button (B1) is pressed — the resting state is "stopped".

### Bug found and fixed — motor creeping when it should be stopped
- **Symptom:** after a move finished, the motor kept slowly ticking forward in the last direction it moved, forever.
- **Evidence:** the step-count test returned +7, +7, +32, +42 extra steps on four runs. A code mistake would give the same error every time; a different error each run meant something random was adding steps.
- **Cause:** at the end of a move, the code switched off the STM32's output on the STEP pin, leaving the wire *floating* (connected to nothing). A floating wire picks up electrical noise from the motor and driver, and the driver counted each noise spike as a step.
- **Fix:** keep the output switched on and held **low** while idle; just pause the timer between moves. Held low from boot as well.
- **Lesson:** never leave a digital input floating — something must always hold it at a definite level. For the PCB: add a 10 kΩ pull-down resistor on STEP (and DIR) so the line stays low even while the STM32 is in reset.

### Results
- `VERIFIED` Step count test (12,345 steps, read back from the driver's own counter MSCNT): returned 0x3C7 on 5 out of 5 resets — exactly 12,345 steps every time (driver counts downward with this wiring).
- `VERIFIED` MSCNT stays constant while idle — no stray steps.
- `VERIFIED` Waypoint cycle (0° → 90° → 180° → 90° → 0°) runs smoothly and stops cleanly.
- Not tested: the earlier theory that the *old* Stage D code sent one extra step per move. The old code is gone, so this is left unresolved.

### Also fixed in this session
- `.ioc` timer prescaler was 90 while `main.c` had 89 — regenerating code would have made every speed ~1% slow. Set to 89, and timer preload enabled in the `.ioc` so regeneration keeps it.
- Build output folder (`Debug/`) removed from git tracking.

### Next
1. At high speeds the timer can only adjust speed in ~5% jumps (at 60 rpm). Fix later by running the timer clock faster (prescaler 0).
2. Assemble joint 1 → Stage E (homing). Tune acceleration under real load.
3. Stage F: power-up safety — the motor driver should stay disabled until the firmware says it's safe.

---

## 2026-10-20 — Stage C resolved + Stage D: Position control (single axis)

**Status:** `VERIFIED` — 0° → 90° → 180° → 90° → 0° waypoints execute with precise stops, no load.

### Stage C blocker — root cause
- Symptom: motor energised and held stiff but would not rotate; XACTUAL advanced in software.
- **Root cause:** TMC5160T `SD_MODE` strapped **high** on the BIGTREETECH board → chip in STEP/DIR mode, ignoring the SPI ramp generator. Confirmed via `IOIN` bit 6 = 1.
- Evidence: `MSCNT` frozen at 0, `stst` set, `CS_ACTUAL` near zero — coils never commutated (no STEP pulses arriving).
- **Cracked by:** UART register dump during a spin attempt (IOIN, MSCNT, VACTUAL, DRV_STATUS). Reading the chip directly beat inferring from PSU current.

### Decision — STEP/DIR architecture (supersedes SPI internal ramp)
- Fixing SD_MODE needs sub-mm resistor rework on the driver — not worth it.
- STEP/DIR is better anyway for 6 axes: STM32 hardware timers own motion timing; six timer-driven step trains coordinate more cleanly than six ramp generators over a shared SPI bus.
- SPI now configuration only (GLOBALSCALER, IHOLD_IRUN, CHOPCONF/microstepping). Ramp registers dropped.

### Wiring added
- PA0 → STEP (TIM2_CH1 PWM); PA1 → DIR (GPIO). All Stage C wiring unchanged.

### Stage D implementation
- TIM2 (PSC=89 → 1 MHz) generates the step train; update ISR counts steps down to target and halts the timer. Absolute position in `g_position_steps`.
- API: `move_steps`, `move_to_deg`, `move_to_rad`, `move_wait` (blocking), `move_busy` (non-blocking), `get_angle_deg/rad`. Units: steps internal, `STEPS_PER_REV` = 51200 (200 × 256 µsteps).
- **Bug fixed:** motor never stopped — NVIC "TIM2 global interrupt" was disabled, so the counting ISR never fired.

### Findings
- `DEAD END` Inferring coil current from PSU current. Supply current ≠ coil current on a chopper driver.
- **Safety:** hardware timers keep stepping independently of the CPU. Halting the IDE did not stop the motor; reset only stopped it while held. → Rest state must be "stopped"; Stage F must actively stop timers and pull EN on any fault/hang.
- `move_wait()` blocking is fine for bench, wrong for multi-axis/serial — switch to `move_busy()` polling at Stage G.

### Next
- Accel/decel ramping (buildable on bare shaft). Then assemble joint 1 → Stage E.

---

## 2026-07-18 — Stage C: First motion attempt (SPI internal ramp)

**Status:** `DEAD END` for the SPI-ramp approach (root cause found 2026-10-20).

### Objective
Spin one NEMA17 (2 A) via TMC5160T internal ramp generator over SPI — no STEP/DIR.

### Setup
- NUCLEO-F446RE, VIN ~11 V from bench PSU (JP5 = E5V; PC USB power was unstable). TMC5160T, R_sense 0.075 Ω (~3.1 A max). VM shares the 11 V rail; 100 µF across VM/GND.
- SPI2: SCK=PB10, MOSI=PC1, MISO=PC2, CS=PB6 (soft GPIO). Mode 3, MSB-first, ÷32 (~1.41 Mbit/s). VIO → 3.3 V, EN → GND.

### Results
- `VERIFIED` SPI comms: status byte 0x39; IOIN version 0x30. Round-trip XACTUAL write/read of 0x12345678 exact.
- `VERIFIED` Driver layer `tmc5160.c/.h/_regs.h` — `tmc5160_write()` / `tmc5160_read()`.
- `VERIFIED` Motor energises and holds; CHOPCONF reads back 0x000100C3; GSTAT = 0; no DRV_STATUS faults.
- Ramp generator ran (VACTUAL = 5000, XACTUAL advancing) but **shaft did not turn**; supply drew only 0.27–0.4 A vs ~1.5 A+ commanded.

### Findings
- `DEAD END` StealthChop lower-current-limit theory (datasheet rev1.17 p61) — plausible fit for "current responds to GLOBALSCALER but capped low", but not the cause. Real cause was SD_MODE (see 2026-10-20).
- Known issue: `TPWMTHRS` defined twice in `tmc5160_regs.h`.

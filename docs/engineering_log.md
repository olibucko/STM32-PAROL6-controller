# PAROL6 Controller — Engineering Log

Dated lab notebook for `stm32-PAROL6-controller`. Records what was tried, what happened, and **why** — including dead ends. Code changes themselves live in git history; this log holds the reasoning.

**Conventions**
- Newest entry at the top.
- Each entry: Objective → Work done → Results → Findings → Next.
- Status tags: `VERIFIED` (bench-confirmed), `UNTESTED` (written, not run), `THEORY` (hypothesis, not yet proven), `DEAD END` (disproven — kept for the record).
- Commit the log alongside the code it describes.

---

## 2026-10-21 — Stage D2: Trapezoidal accel/decel ramping (single axis)

**Status:** `UNTESTED` — written and merged into `main.c`, not yet flashed.

### Objective
Replace instant-on step rate with a trapezoidal speed profile (accelerate → cruise → decelerate). Instant-on demands infinite acceleration torque at t=0 → missed steps under load → silent open-loop position error. Prerequisite for loaded position control.

### Design
- **Per-step speed from position within the move:**
  `speed = min( √(v₀² + 2a·steps_done), √(v₀² + 2a·steps_to_go), vmax )`
  From v² = v₀² + 2as with distance in steps. Stateless (no phase state machine, no accumulated error); short moves become triangular automatically.
- **TIM2** generates pulses in hardware (1 MHz tick, PSC=89). **TIM2 ISR** runs once per step: records the step just begun, computes the *next* step's period (ARR), writes it to preload — always one step ahead.
- **ARR/CCR preload enabled** so changes apply from the next step period, never mid-period.
- **Blank final period** (CCR=0, no pulse); timer stops at the reset after it → exactly N pulses.
- Fixed 5 µs STEP pulse; V_START = 500 Hz; bench accel 120 rpm/s (0→30 rpm in 0.25 s, ~22.5° travel).

### Changes to `main.c`
- `math.h`; `STEPS_PER_*` moved to USER CODE PD.
- New: `TIM_TICK_HZ`, `STEP_PULSE_TICKS`, `V_START_HZ`; state `g_steps_total`, `g_steps_done`, `g_moving`.
- TIM2 `AutoReloadPreload = ENABLE` — **must also be set in .ioc or regen reverts it.**
- New `ramp_arr()`, `load_next_period()`, `set_accel_rpm_s()`. Rewrote `move_steps()` and ISR. `move_wait()` / `move_busy()` now poll `g_moving`.
- Removed `motor_set_step_hz()` / `motor_set_rpm()` — unused, free-running, uncounted (safety hazard).

### Findings
- `THEORY` **Old code sends +1 step per move.** In PWM1 mode the pin rises in hardware at every update, the same instant the ISR runs — so stopping at the Nth update had already launched pulse N+1. The waypoint loop (+,+,−,−) cancels the error, hiding it. Blank-period design fixes it.
- Without ARR preload, writing an ARR below the current count makes 32-bit TIM2 overrun to 2³² (~72 min stall).
- `g_steps_remaining` reaches 0 one period before the timer stops → can't be used as the "move complete" signal; added `g_moving`.
- An ISR is a hardware-triggered function, not inherently a counter — here it counts steps *and* controls the next step's timing.

### Next
1. (Optional baseline) MSCNT test on old code — expect Δ = 58 if +1 bug is real.
2. Add `TMC_MSCNT` (0x6A) to `tmc5160_regs.h` if missing; verify in datasheet MSCNT advances 1 per step at MRES=256.
3. Flash new code — MSCNT test expects Δ = 57 (0x39), or 967 (0x3C7) if polarity counts down.
4. Waypoint loop: smooth start/stop, no clunk.
5. Later: PSC=0 (90 MHz tick) for speed resolution at high rpm (~5% quantisation at 60 rpm currently).
6. Then assemble joint 1 → Stage E (homing); tune accel under real load.

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

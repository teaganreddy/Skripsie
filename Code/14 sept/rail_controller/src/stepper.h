#pragma once
/*
 * stepper.h -- the STEP/DIR pulse engine, on a hardware timer.
 *
 * This layer knows nothing about mm, switches, homing or the stator. It turns
 * a target position and a step rate into an exact pulse train and counts
 * every step it makes. Everything above it (rail.cpp) shapes the rate and
 * decides where to go.
 *
 * Why an ISR and not loop(): the bring-up sketch toggled STEP from loop() with
 * micros(), which was fine with nothing else running. This is a single-core
 * ESP32-C6 and it is about to carry ESP-NOW. The stator's measured worst-case
 * gap with WiFi active is 5.77 ms -- on a 469 us half-period that is a dozen
 * missed toggles, a visible stutter, and then a restart at full rate with no
 * ramp. A hardware timer keeps stepping whatever the CPU is doing.
 *
 * The ISR stops ITSELF at the target. That is the invariant that makes a late
 * control tick harmless: rate shaping from loop() can be a few ms late and the
 * ramp is briefly less smooth, but the carriage cannot overshoot because the
 * decision "have I arrived" is never made by loop().
 */

#include <Arduino.h>

namespace stepper {

void begin();

/* Direction may only be changed while not running -- rail.cpp guarantees it.
 * `sign` is +1 toward IDLE (position increasing), -1 toward HOME. */
void setDirection(int sign);
int direction();

/* Start pulsing toward `target` at `rateStepsPerSec`. The direction must
 * already be set to point at the target; this does not check. */
void start(int32_t target, uint32_t rateStepsPerSec);

/* Change the rate of a move in progress. Picked up by the ISR at its next
 * toggle. Clamped below to 1 step/s so the period never overflows. */
void setRate(uint32_t rateStepsPerSec);
uint32_t rate();

/* Stop pulsing immediately, STEP left low. Position is preserved exactly --
 * the last counted step is the last step the driver received. */
void halt();

bool running();
int32_t position();
void setPosition(int32_t p);  // only while halted
int32_t target();

/* Called from loop(): stops the hardware timer once the ISR has reported
 * arrival. Cheap, safe to call every tick. */
void service();

/* A4988 ENABLE is active LOW. Disabling drops holding torque; position is
 * still counted but the carriage can then be pushed, so rail.cpp treats a
 * disable as "must re-home". */
void enableDriver(bool on);
bool driverEnabled();

}  // namespace stepper

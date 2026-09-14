#pragma once
/*
 * sync.h -- where the arm is pointing, derived locally.
 *
 * ===========================================================================
 *  WHY THIS IS NOT DONE OVER ESP-NOW
 * ===========================================================================
 *
 * At 700 rpm a fresh column is due every 476 us. ESP-NOW latency is
 * milliseconds, with jitter of the same order. Radio cannot beat time for a
 * POV display -- not with retries, not with a tighter protocol, not ever. A
 * single 5 ms hiccup is ten columns, or 20 degrees of smear.
 *
 * So the rotor keeps its own time and the stator never participates in it:
 *
 *      Hall falling edge  ->  angular zero, and one revolution's period
 *      between edges      ->  interpolate the angle from elapsed time
 *
 * The stator's job is to send CONTENT and COMMANDS. It does not tick.
 *
 * The consequence worth being clear about: angular accuracy is set entirely
 * by how steady the rotation is between pulses. The period is measured over
 * the PREVIOUS revolution and applied to the current one, so any acceleration
 * within a revolution shows up as angular error that grows across it. At a
 * steady speed that is negligible; during the stator's 60 rpm/s ramp it is
 * roughly 0.4% of a revolution, or about 1.5 degrees by the far end -- which
 * is why the image is only expected to be crisp once `running`, not while
 * `starting`.
 *
 * The AS5600 inside the motor knows the arm's ABSOLUTE angle (see the note in
 * fan_controller/src/motor_task.cpp), but it is on the wrong side of the radio
 * link to help with per-column timing. Where it earns its keep is multi-unit
 * sync, where two fans need a shared phase reference to paint one image.
 */

#include <Arduino.h>

namespace rotorsync {

void begin();

// True once at least two Hall pulses have been seen, so a period exists.
bool locked();

// True if no pulse has arrived within HALL_TIMEOUT_US.
bool stopped();

/* Current angular sector, 0..ANGLES-1, interpolated from the last Hall pulse.
 * Returns -1 when not locked or stopped. */
int sector();

// Measured revolution period in microseconds, 0 if not locked.
uint32_t periodUs();

// Measured speed in RPM, from the Hall period. Independent of the stator's
// own AS5600 figure, so comparing the two is a useful cross-check.
float rpm();

// Revolutions counted since boot.
uint32_t revolutions();

/* Pulses rejected by the sanity window, and revolutions where the interpolated
 * sector ran past ANGLES-1 before the next pulse arrived (i.e. the arm slowed
 * mid-revolution). Both are bring-up diagnostics. */
uint32_t rejectedPulses();
uint32_t overruns();

// Called by the display loop when it clamps a sector; keeps the count honest.
void noteOverrun();

}  // namespace rotorsync

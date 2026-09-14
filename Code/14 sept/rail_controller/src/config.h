#pragma once
/*
 * config.h -- linear rail constants, all in one place.
 *
 * Pins and the motion figures are taken from linear_rail_test/src/main.cpp,
 * the bring-up sketch that proved the rail runs, not guessed.
 *
 * !! THESE PIN NUMBERS OVERLAP WITH THE STATOR'S. GPIO 8 / 14 / 7 are STEP /
 * !! DIR / ENABLE here and DRV EN / IN3 / nFAULT on fan_controller. Nothing
 * !! conflicts -- they are different boards -- but do not copy a pin table
 * !! between the two projects.
 */

#include <Arduino.h>

// ===========================================================================
//  IDENTITY
// ===========================================================================

/* Which fan unit this rail belongs to. It pairs only with a stator whose
 * beacon carries this unit id (or 0, which means "unset" and is treated as
 * unit 1). Build the second unit's rail with -DUNIT_ID=2. */
#ifndef UNIT_ID
#define UNIT_ID 1
#endif

// ===========================================================================
//  PINS  -- GPIO numbers, not the FireBeetle 2 silkscreen "D" labels
// ===========================================================================

static const uint8_t PIN_STEP = 8;
static const uint8_t PIN_DIR = 14;
static const uint8_t PIN_MS1 = 1;
static const uint8_t PIN_MS2 = 18;
static const uint8_t PIN_MS3 = 6;
static const uint8_t PIN_ENABLE = 7;      // A4988 ENABLE, active LOW
static const uint8_t PIN_LIMIT_HOME = 20; // motor end of the rail (SCL header pin)
static const uint8_t PIN_LIMIT_IDLE = 19; // idler end of the rail (SDA header pin)
static const uint8_t PIN_HEARTBEAT = LED_BUILTIN;  // GPIO15 onboard LED

/* GPIO15 is the onboard LED and must not be a limit input -- the LED loads the
 * weak internal pull-up so the pin never reads a clean HIGH. It is the
 * heartbeat instead, and the blink pattern encodes state (see main.cpp).
 *
 * SLEEP and RESET on the A4988 are hard-wired to VDD, not MCU-controlled.
 *
 * Limit switches are SPDT wired NC + C, NO left unconnected: C -> GND,
 * NC -> GPIO with INPUT_PULLUP. Resting reads LOW; a pressed lever OR a wire
 * that has fallen off reads HIGH. That is deliberate -- a broken switch fails
 * toward "stop", not toward "drive through the end of the rail". */
static const int LIMIT_TRIGGERED = HIGH;

/* Verified on this rig: DIR=LOW carries the carriage toward LIMIT_HOME. Swap
 * these if the wiring changes. Position increases toward IDLE. */
static const int DIR_TOWARD_HOME = LOW;
static const int DIR_TOWARD_IDLE = HIGH;

// ===========================================================================
//  A4988 -- as set on the hardware
// ===========================================================================

/* MS1=H, MS2=H, MS3=L -> 1/8 step. Quiet, and 40 steps/mm is far more
 * resolution than the rail can use, so there is no reason to go finer. */
static const int MICROSTEP_DIVISOR = 8;
static const int MOTOR_STEPS_PER_REV = 200;  // NEMA17, 1.8 deg
static const int STEPS_PER_REV = MOTOR_STEPS_PER_REV * MICROSTEP_DIVISOR;  // 1600

/* Current limit is a POT on the driver board, recorded here so the number is
 * not lost. Vref measured 1.04 V with the confirmed R100 (0.1 ohm) sense
 * resistors:  I = Vref / (8 x Rs) = 1.04 / 0.8 = 1.30 A per phase.
 *
 * 1.3 A is above what a bare A4988 dissipates comfortably (~1 A) -- it needs
 * its heatsink. The driver holds full phase current whenever ENABLE is low,
 * even stationary, so it is warm all the time the rail is powered.
 *
 * VMOT DECOUPLING: the datasheet calls for >= 100 uF at VMOT and the board as
 * built has none fitted. That is not optional on a supply shared with the FOC
 * driver and the AS5600: fit a 100-220 uF, >= 25 V electrolytic across
 * VMOT/GND as close to the driver as physically possible before running both
 * from the one adapter. */
static const float A4988_VREF_V = 1.04f;
static const float A4988_PHASE_CURRENT_A = 1.30f;

// ===========================================================================
//  MECHANICS
// ===========================================================================

static const int PULLEY_TEETH = 20;          // GT2 20T on the motor
static const float BELT_PITCH_MM = 2.0f;     // GT2
static const float MM_PER_REV = PULLEY_TEETH * BELT_PITCH_MM;      // 40 mm
static const float STEPS_PER_MM = STEPS_PER_REV / MM_PER_REV;      // 40

/* Usable travel, HOME switch to IDLE switch.
 *
 * !! PLACEHOLDER -- to be measured on the rig. The rail also measures it for
 * !! itself with RAIL_OP_MEASURE (one seek from HOME to IDLE) and reports the
 * !! result; if the two disagree by more than TRAVEL_MISMATCH_FRACTION the
 * !! measured value is used and a warning is printed. Put the measured number
 * !! here once known, and keep RAIL_TRAVEL_MM in the website's config.js in
 * !! step with it. */
static const float RAIL_TRAVEL_MM_NOMINAL = 500.0f;
static const float TRAVEL_MISMATCH_FRACTION = 0.05f;

/* Soft limits sit this far inside the switches. In normal running the switches
 * are never touched -- they exist to catch lost steps and wiring faults. */
static const float SOFT_MARGIN_MM = 5.0f;

/* Lost-step detection: a limit switch firing while the counter says the
 * carriage is further than this from that end means steps were skipped. Wide
 * enough not to trip on switch hysteresis, tight enough to be useful. */
static const float LIMIT_TOLERANCE_MM = 10.0f;

// ===========================================================================
//  MOTION
// ===========================================================================

/* Speeds in mm/s. CRUISE matches the ~1067 steps/s the bring-up sketch ran at
 * (26.7 mm/s), which is known to move the bare carriage cleanly. MAX is a hard
 * cap on anything the stator asks for. The load is a whole fan unit and an
 * open-loop stepper that skips under it loses position SILENTLY, so start
 * conservative and raise these on the rig, not on paper. */
static const float CRUISE_MM_S = 25.0f;
static const float MAX_MM_S = 60.0f;
static const float MIN_MM_S = 1.0f;  // floor for the ramp so the last steps are not glacial

/* Trapezoidal ramp. The bring-up sketch had none and reversed instantly at the
 * switches, which was fine for a bare carriage and is not fine with a fan on
 * it -- an instant reversal is a torque shock and the first thing to skip. */
static const float ACCEL_MM_S2 = 100.0f;

/* Homing. FAST seeks the switch, SLOW makes the final approach so the zero is
 * repeatable to a step or two rather than to however far the carriage
 * overshoots at speed. BACKOFF is how far to retreat before the slow
 * re-approach, and where the carriage parks after homing so it is not
 * sitting on the switch. */
static const float HOMING_FAST_MM_S = 15.0f;
static const float HOMING_SLOW_MM_S = 3.0f;
static const float HOMING_BACKOFF_MM = 3.0f;

/* If the carriage has travelled this far toward a switch without finding it,
 * something is wrong -- belt off, switch dead, wire fallen off the NO side --
 * and it must stop rather than grind the carriage into the end plate. */
/* Generous on purpose while RAIL_TRAVEL_MM_NOMINAL is still a placeholder: a
 * guard SHORTER than the real rail would fault a perfectly good homing run
 * started from the far end, whereas a guard that is too long only costs
 * seconds in the case where something is already broken. Tighten it to
 * 1.2 x travel once the length is measured. */
static const float HOMING_MAX_TRAVEL_MM =
    (RAIL_TRAVEL_MM_NOMINAL * 1.2f > 1000.0f) ? RAIL_TRAVEL_MM_NOMINAL * 1.2f : 1000.0f;

/* Mechanical snap-action switches bounce for a few ms. From the bring-up
 * sketch, where it worked. */
static const uint32_t SWITCH_DEBOUNCE_MS = 40;

/* Default pause at each end of a sweep. */
static const uint32_t SWEEP_DWELL_MS_DEFAULT = 1000;

// ===========================================================================
//  TIMING
// ===========================================================================

/* The step pulse train is generated by a hardware timer ISR, not by polling in
 * loop(). This is a single-core chip and the moment ESP-NOW is on it, polled
 * stepping stutters whenever the WiFi task runs and then restarts at full rate
 * with no ramp -- the same problem the stator's FOC loop had to design around.
 * The ISR toggles STEP, counts position, and reprograms its own period. */
static const uint32_t STEP_TIMER_HZ = 1000000;  // 1 us resolution

/* The ramp is shaped from loop() at roughly this rate. If WiFi delays it by a
 * few ms the ramp is briefly less smooth; the ISR still stops at the exact
 * target on its own, so a late control tick cannot cause an overshoot. */
static const uint32_t CONTROL_TICK_US = 1000;

// ===========================================================================
//  LINK
// ===========================================================================

/* 5 Hz, matching the rotor. */
static const uint32_t TELEMETRY_INTERVAL_MS = 200;

/* No beacon for this long and the stator is assumed gone. A move in progress
 * is halted -- the failure direction is always "stop". */
static const uint32_t LINK_LOST_MS = 3000;

static const uint32_t HEARTBEAT_IDLE_MS = 500;
static const uint32_t HEARTBEAT_FAULT_MS = 100;

// ===========================================================================
//  DERIVED -- steps
// ===========================================================================

inline int32_t mmToSteps(float mm) { return (int32_t)lroundf(mm * STEPS_PER_MM); }
inline float stepsToMm(int32_t steps) { return steps / STEPS_PER_MM; }
inline uint32_t mmsToRate(float mms) { return (uint32_t)lroundf(mms * STEPS_PER_MM); }

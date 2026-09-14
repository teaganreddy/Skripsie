#pragma once
/*
 * rail.h -- what the rail is doing and where it is.
 *
 * The state machine, the ramp, homing, soft limits, sweeps and faults. Sits
 * on stepper.cpp (which only knows steps) and is driven by rail_link.cpp
 * (commands from the stator) and by the serial keys in main.cpp.
 *
 * Positions are in STEPS from the HOME zero, increasing toward IDLE. mm are
 * for humans; everything here is integer steps so nothing accumulates.
 *
 * The rail NEVER moves on its own. Boot is `unhomed` and motionless whatever
 * the switches say, because the rail board can reset while the fan next to it
 * is spinning, and the only safe thing to do then is nothing.
 */

#include <Arduino.h>

#include "link_protocol.h"

namespace rail {

void begin();

/* Call every CONTROL_TICK_US or so from loop(). Shapes the ramp, runs the
 * homing/measuring/sweep sequences, watches the switches. */
void tick();

// --- commands. Each returns false, with a reason on serial, if refused. ---
bool home();
bool measure();  // seek IDLE once and record the travel; needs homed
bool gotoSteps(int32_t target, uint32_t rateStepsPerSec);  // 0 = cruise
bool jogSteps(int32_t delta, uint32_t rateStepsPerSec);
bool sweep(int32_t minSteps, int32_t maxSteps, uint32_t dwellMs,
           uint32_t rateStepsPerSec);
void stop();        // decelerate to a halt, then idle
void haltNow();     // immediate, no ramp -- E-STOP and lost-link use this
bool clearFault();

/* The stator's interlock. While false, every motion command is refused and a
 * move in progress is halted. Serial bench keys set it true themselves, since
 * pressing a key on the attached board is as good a "the fan is stopped" as
 * any. */
void setMotionPermitted(bool ok);
bool motionPermitted();

void enableDriver(bool on);
bool driverEnabled();

// --- queries, for telemetry and the status line ---
uint8_t state();   // RAIL_STATE_*
uint8_t fault();   // RAIL_FAULT_*
bool homed();
bool travelMeasured();
int32_t position();
int32_t target();
uint32_t travelSteps();
uint32_t rateStepsPerSec();
bool limitHome();  // debounced, true = triggered
bool limitIdle();
int32_t softMin();
int32_t softMax();
const char *stateName();
const char *faultName();

}  // namespace rail

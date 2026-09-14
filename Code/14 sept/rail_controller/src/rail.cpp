#include "rail.h"

#include "config.h"
#include "stepper.h"

namespace {

// ---------------------------------------------------------------------------
//  Limit switches -- debounce carried over unchanged from linear_rail_test
// ---------------------------------------------------------------------------

struct LimitSwitch {
  uint8_t pin;
  int rawLast;
  int stable;
  uint32_t lastEdgeMs;
};

LimitSwitch swHome = {PIN_LIMIT_HOME, LOW, LOW, 0};
LimitSwitch swIdle = {PIN_LIMIT_IDLE, LOW, LOW, 0};

/* Updates the debounced state. Returns true on the exact tick the debounced
 * state goes untriggered -> triggered. */
bool debouncedTrigger(LimitSwitch &sw) {
  const int raw = digitalRead(sw.pin);
  const uint32_t now = millis();
  if (raw != sw.rawLast) {
    sw.rawLast = raw;
    sw.lastEdgeMs = now;
  }
  if ((now - sw.lastEdgeMs) >= SWITCH_DEBOUNCE_MS && raw != sw.stable) {
    const int prev = sw.stable;
    sw.stable = raw;
    return prev != LIMIT_TRIGGERED && sw.stable == LIMIT_TRIGGERED;
  }
  return false;
}

inline bool triggered(const LimitSwitch &sw) { return sw.stable == LIMIT_TRIGGERED; }

// ---------------------------------------------------------------------------
//  State
// ---------------------------------------------------------------------------

uint8_t st = RAIL_STATE_UNHOMED;
uint8_t flt = RAIL_FAULT_NONE;
bool isHomed = false;
bool measured = false;
uint32_t travel = 0;  // steps, HOME zero to IDLE switch
bool permitted = false;

// The move in progress.
int32_t moveTarget = 0;
float cruise = 0;       // steps/s the ramp is heading for
float rateNow = 0;      // steps/s right now
uint32_t lastTickUs = 0;
bool stopping = false;  // stop() asked for a decel-to-halt

// Sub-sequences.
enum class Homing : uint8_t { NONE, SEEK, BACKOFF, APPROACH, PARK };
Homing homing = Homing::NONE;
int backoffTries = 0;

enum class Measuring : uint8_t { NONE, SEEK_IDLE, RETURN };
Measuring measuring = Measuring::NONE;

struct Sweep {
  int32_t lo = 0, hi = 0;
  uint32_t dwellMs = 0;
  uint32_t rate = 0;
  bool toHi = true;
  uint32_t dwellUntil = 0;
  bool dwelling = false;
} sw;

// ---------------------------------------------------------------------------
//  Helpers
// ---------------------------------------------------------------------------

inline int32_t softMinSteps() { return mmToSteps(SOFT_MARGIN_MM); }
inline int32_t softMaxSteps() { return (int32_t)travel - mmToSteps(SOFT_MARGIN_MM); }

const char *faultText(uint8_t f) {
  switch (f) {
    case RAIL_FAULT_NONE: return "none";
    case RAIL_FAULT_BOTH_LIMITS: return "both limit switches read triggered -- wiring";
    case RAIL_FAULT_LOST_STEPS: return "limit switch hit where the counter said it could not -- lost steps, re-home";
    case RAIL_FAULT_HOME_NOT_FOUND: return "travelled the whole rail and never found HOME";
    case RAIL_FAULT_IDLE_NOT_FOUND: return "travelled the whole rail and never found IDLE";
    case RAIL_FAULT_LINK_LOST: return "stator went quiet mid-move -- halted";
    default: return "?";
  }
}

void latch(uint8_t f) {
  stepper::halt();
  rateNow = 0;
  homing = Homing::NONE;
  measuring = Measuring::NONE;
  sw.dwelling = false;
  stopping = false;
  flt = f;
  st = RAIL_STATE_FAULT;
  if (f == RAIL_FAULT_LOST_STEPS || f == RAIL_FAULT_HOME_NOT_FOUND ||
      f == RAIL_FAULT_BOTH_LIMITS) {
    isHomed = false;  // the zero can no longer be trusted
  }
  Serial.printf("[RAIL] FAULT %u: %s (pos %ld steps = %.1f mm)\n", f, faultText(f),
                (long)stepper::position(), stepsToMm(stepper::position()));
}

/* Kick off a move. Sets direction from the sign of the distance, starts the
 * stepper at the ramp floor, and lets tick() shape the rate from there.
 * `raw` bypasses the soft limits -- only homing and measuring use it. */
void beginMove(int32_t target, float cruiseRate, bool raw) {
  if (!raw) {
    if (target < softMinSteps()) target = softMinSteps();
    if (target > softMaxSteps()) target = softMaxSteps();
  }
  moveTarget = target;
  cruise = cruiseRate;
  const int32_t here = stepper::position();
  if (target == here) {
    rateNow = 0;
    return;
  }
  stepper::setDirection(target > here ? 1 : -1);
  rateNow = mmsToRate(MIN_MM_S);
  stopping = false;
  lastTickUs = micros();
  stepper::start(target, (uint32_t)rateNow);
}

/* Trapezoid: accelerate toward cruise, and start decelerating when the
 * distance still to go equals the distance the current speed needs to stop.
 * Called every control tick while the stepper is running. */
void shapeRamp() {
  const uint32_t now = micros();
  float dt = (now - lastTickUs) / 1000000.0f;
  lastTickUs = now;
  if (dt > 0.05f) dt = 0.05f;  // a long stall must not produce a huge jump

  const float accel = ACCEL_MM_S2 * STEPS_PER_MM;
  const float floorRate = mmsToRate(MIN_MM_S);
  const float remaining = (float)labs(moveTarget - stepper::position());
  const float stopDist = (rateNow * rateNow) / (2.0f * accel);

  float goal = stopping ? floorRate : cruise;
  if (remaining <= stopDist) goal = floorRate;

  if (rateNow < goal) {
    rateNow += accel * dt;
    if (rateNow > goal) rateNow = goal;
  } else if (rateNow > goal) {
    rateNow -= accel * dt;
    if (rateNow < goal) rateNow = goal;
  }
  if (rateNow < floorRate) rateNow = floorRate;
  stepper::setRate((uint32_t)rateNow);

  /* A decel-to-halt from stop(): once at the floor, actually stop. Position
   * stays wherever it got to. */
  if (stopping && rateNow <= floorRate) {
    stepper::halt();
    rateNow = 0;
    stopping = false;
  }
}

bool refuse(const char *why) {
  Serial.printf("[RAIL] refused: %s\n", why);
  return false;
}

bool canStart(bool needHomed) {
  if (st == RAIL_STATE_FAULT) return refuse("fault latched -- clear it first");
  if (!permitted) return refuse("motion not permitted (fan not stopped?)");
  if (!stepper::driverEnabled()) return refuse("driver disabled");
  if (stepper::running()) return refuse("already moving -- stop first");
  if (needHomed && !isHomed) return refuse("not homed");
  return true;
}

// ---------------------------------------------------------------------------
//  Sequences, advanced once per tick
// ---------------------------------------------------------------------------

void tickHoming(bool homeHit) {
  const bool arrived = !stepper::running();
  switch (homing) {
    case Homing::SEEK:
      if (homeHit) {
        stepper::halt();
        rateNow = 0;
        backoffTries = 0;
        homing = Homing::BACKOFF;
        beginMove(stepper::position() + mmToSteps(HOMING_BACKOFF_MM),
                  mmsToRate(HOMING_SLOW_MM_S), true);
      } else if (arrived) {
        latch(RAIL_FAULT_HOME_NOT_FOUND);
      }
      break;

    case Homing::BACKOFF:
      if (arrived) {
        if (triggered(swHome)) {
          // Still on the switch -- back off further, but not forever. A switch
          // that never releases is stuck or its NC wire is off.
          if (++backoffTries > 5) {
            Serial.println(F("[RAIL] HOME switch never released while backing off"));
            latch(RAIL_FAULT_HOME_NOT_FOUND);
            return;
          }
          beginMove(stepper::position() + mmToSteps(HOMING_BACKOFF_MM),
                    mmsToRate(HOMING_SLOW_MM_S), true);
        } else {
          homing = Homing::APPROACH;
          beginMove(stepper::position() - mmToSteps(HOMING_BACKOFF_MM * 3.0f),
                    mmsToRate(HOMING_SLOW_MM_S), true);
        }
      }
      break;

    case Homing::APPROACH:
      if (homeHit) {
        stepper::halt();
        rateNow = 0;
        stepper::setPosition(0);  // THE zero: the slow-approach trigger point
        homing = Homing::PARK;
        beginMove(softMinSteps(), mmsToRate(HOMING_SLOW_MM_S), true);
      } else if (arrived) {
        latch(RAIL_FAULT_HOME_NOT_FOUND);
      }
      break;

    case Homing::PARK:
      if (arrived) {
        homing = Homing::NONE;
        isHomed = true;
        st = RAIL_STATE_IDLE;
        Serial.printf("[RAIL] homed. Parked at %.1f mm. Travel %s %.1f mm.\n",
                      stepsToMm(stepper::position()),
                      measured ? "measured" : "nominal", stepsToMm(travel));
      }
      break;

    default:
      break;
  }
}

void tickMeasuring(bool idleHit) {
  const bool arrived = !stepper::running();
  switch (measuring) {
    case Measuring::SEEK_IDLE:
      if (idleHit) {
        stepper::halt();
        rateNow = 0;
        const uint32_t found = (uint32_t)stepper::position();
        const float nominal = mmToSteps(RAIL_TRAVEL_MM_NOMINAL);
        const float diff = fabsf((float)found - nominal) / nominal;
        Serial.printf("[RAIL] travel measured: %lu steps = %.1f mm (nominal %.1f mm, %+.1f%%)\n",
                      (unsigned long)found, stepsToMm(found), RAIL_TRAVEL_MM_NOMINAL,
                      diff * 100.0f);
        if (diff > TRAVEL_MISMATCH_FRACTION) {
          Serial.println(F("[RAIL] !! measured travel disagrees with RAIL_TRAVEL_MM_NOMINAL "
                           "by more than the tolerance. Using the measured value. Put it in "
                           "config.h and the website's config.js."));
        }
        travel = found;
        measured = true;
        measuring = Measuring::RETURN;
        beginMove(softMaxSteps(), mmsToRate(CRUISE_MM_S), false);
      } else if (arrived) {
        latch(RAIL_FAULT_IDLE_NOT_FOUND);
      }
      break;

    case Measuring::RETURN:
      if (arrived) {
        measuring = Measuring::NONE;
        st = RAIL_STATE_IDLE;
      }
      break;

    default:
      break;
  }
}

void tickSweep() {
  if (stepper::running()) return;
  const uint32_t now = millis();
  if (!sw.dwelling) {
    sw.dwelling = true;
    sw.dwellUntil = now + sw.dwellMs;
    return;
  }
  if ((int32_t)(now - sw.dwellUntil) < 0) return;
  sw.dwelling = false;
  sw.toHi = !sw.toHi;
  beginMove(sw.toHi ? sw.hi : sw.lo, (float)sw.rate, false);
}

}  // namespace

// ===========================================================================

namespace rail {

void begin() {
  pinMode(PIN_LIMIT_HOME, INPUT_PULLUP);
  pinMode(PIN_LIMIT_IDLE, INPUT_PULLUP);

  // Seed the debounce from the real levels so a switch already held at boot
  // does not register a fake edge on the first read.
  swHome.rawLast = swHome.stable = digitalRead(PIN_LIMIT_HOME);
  swIdle.rawLast = swIdle.stable = digitalRead(PIN_LIMIT_IDLE);

  travel = (uint32_t)mmToSteps(RAIL_TRAVEL_MM_NOMINAL);
  stepper::begin();

  if (triggered(swHome) && triggered(swIdle)) {
    /* Both ends at once is not a position, it is a wiring fault -- most likely
     * the common ground has come off and both NC lines float high. */
    latch(RAIL_FAULT_BOTH_LIMITS);
  } else {
    st = RAIL_STATE_UNHOMED;
    if (triggered(swHome)) Serial.println(F("[RAIL] HOME switch held at boot -- fine, homing will back off it"));
    if (triggered(swIdle)) Serial.println(F("[RAIL] IDLE switch held at boot -- carriage is at the far end"));
  }

  /* Holding torque from boot, so the carriage cannot be pushed off wherever it
   * was left. Costs driver heat; 'd' toggles it on the bench. */
  stepper::enableDriver(true);

  Serial.printf("[RAIL] unit %d, %.0f steps/mm, nominal travel %.0f mm, soft limits %.0f..%.0f mm\n",
                UNIT_ID, STEPS_PER_MM, RAIL_TRAVEL_MM_NOMINAL, SOFT_MARGIN_MM,
                RAIL_TRAVEL_MM_NOMINAL - SOFT_MARGIN_MM);
  Serial.println(F("[RAIL] boot state: UNHOMED. It will not move until told to."));
}

void tick() {
  stepper::service();

  const bool homeHit = debouncedTrigger(swHome);
  const bool idleHit = debouncedTrigger(swIdle);

  if (st == RAIL_STATE_FAULT) return;

  /* Outside a homing/measuring sequence, a switch firing at all means the
   * counter and the carriage disagree -- the soft limits are supposed to keep
   * the switches untouched. Halt and demand a re-home. This is the only
   * feedback an open-loop stepper has, so it has to count for something. */
  if (homing == Homing::NONE && measuring == Measuring::NONE &&
      (stepper::running() || isHomed)) {
    /* (Unhomed AND stationary is exempt: pressing a switch by hand on the
     * bench should not latch a fault on a rail that was not claiming to know
     * where it was.) */
    if (homeHit || idleHit) {
      Serial.printf("[RAIL] %s switch fired at %.1f mm (travel %.1f)\n",
                    homeHit ? "HOME" : "IDLE", stepsToMm(stepper::position()),
                    stepsToMm(travel));
      latch(RAIL_FAULT_LOST_STEPS);
      return;
    }
  }

  if (stepper::running()) shapeRamp();

  switch (st) {
    case RAIL_STATE_HOMING:
      tickHoming(homeHit);
      break;
    case RAIL_STATE_MEASURING:
      tickMeasuring(idleHit);
      break;
    case RAIL_STATE_SWEEPING:
      tickSweep();
      break;
    case RAIL_STATE_MOVING:
      if (!stepper::running()) st = RAIL_STATE_IDLE;
      break;
    default:
      break;
  }
}

// --- commands ---------------------------------------------------------------

bool home() {
  if (!canStart(false)) return false;
  st = RAIL_STATE_HOMING;
  homing = Homing::SEEK;
  isHomed = false;
  Serial.println(F("[RAIL] homing: seeking HOME"));
  /* If we are already sitting on the switch, SEEK would never see an edge.
   * Start from BACKOFF instead. */
  if (triggered(swHome)) {
    homing = Homing::BACKOFF;
    backoffTries = 0;
    beginMove(stepper::position() + mmToSteps(HOMING_BACKOFF_MM),
              mmsToRate(HOMING_SLOW_MM_S), true);
  } else {
    beginMove(stepper::position() - mmToSteps(HOMING_MAX_TRAVEL_MM),
              mmsToRate(HOMING_FAST_MM_S), true);
  }
  return true;
}

bool measure() {
  if (!canStart(true)) return false;
  st = RAIL_STATE_MEASURING;
  measuring = Measuring::SEEK_IDLE;
  Serial.println(F("[RAIL] measuring: seeking IDLE"));
  beginMove(stepper::position() + mmToSteps(HOMING_MAX_TRAVEL_MM),
            mmsToRate(HOMING_FAST_MM_S), true);
  return true;
}

bool gotoSteps(int32_t target, uint32_t rate) {
  if (!canStart(true)) return false;
  float r = rate ? (float)rate : mmsToRate(CRUISE_MM_S);
  if (r > mmsToRate(MAX_MM_S)) r = mmsToRate(MAX_MM_S);
  st = RAIL_STATE_MOVING;
  beginMove(target, r, false);
  if (!stepper::running()) st = RAIL_STATE_IDLE;  // was already there
  return true;
}

bool jogSteps(int32_t delta, uint32_t rate) {
  return gotoSteps(stepper::position() + delta, rate);
}

bool sweep(int32_t lo, int32_t hi, uint32_t dwellMs, uint32_t rate) {
  if (!canStart(true)) return false;
  if (lo > hi) { const int32_t t = lo; lo = hi; hi = t; }
  if (lo < softMinSteps()) lo = softMinSteps();
  if (hi > softMaxSteps()) hi = softMaxSteps();
  if (hi - lo < mmToSteps(1.0f)) return refuse("sweep span under 1 mm");
  sw.lo = lo;
  sw.hi = hi;
  sw.dwellMs = dwellMs;
  float r = rate ? (float)rate : mmsToRate(CRUISE_MM_S);
  if (r > mmsToRate(MAX_MM_S)) r = mmsToRate(MAX_MM_S);
  sw.rate = (uint32_t)r;
  sw.dwelling = false;
  // Head for whichever end is further, so the first leg is a full one.
  const int32_t here = stepper::position();
  sw.toHi = labs(hi - here) >= labs(here - lo);
  st = RAIL_STATE_SWEEPING;
  beginMove(sw.toHi ? hi : lo, r, false);
  return true;
}

void stop() {
  if (st == RAIL_STATE_FAULT) return;
  homing = Homing::NONE;
  measuring = Measuring::NONE;
  sw.dwelling = false;
  if (stepper::running()) {
    stopping = true;  // shapeRamp() decelerates and halts
    st = RAIL_STATE_MOVING;
  } else {
    st = isHomed ? RAIL_STATE_IDLE : RAIL_STATE_UNHOMED;
  }
  /* A stop during homing/measuring leaves the zero unproven. */
  if (!isHomed) st = RAIL_STATE_UNHOMED;
}

void haltNow() {
  stepper::halt();
  rateNow = 0;
  stopping = false;
  homing = Homing::NONE;
  measuring = Measuring::NONE;
  sw.dwelling = false;
  if (st != RAIL_STATE_FAULT) st = isHomed ? RAIL_STATE_IDLE : RAIL_STATE_UNHOMED;
}

bool clearFault() {
  if (st != RAIL_STATE_FAULT) return true;
  flt = RAIL_FAULT_NONE;
  st = isHomed ? RAIL_STATE_IDLE : RAIL_STATE_UNHOMED;
  Serial.printf("[RAIL] fault cleared -> %s\n", stateName());
  return true;
}

void setMotionPermitted(bool ok) {
  if (permitted && !ok && stepper::running()) {
    Serial.println(F("[RAIL] motion permission withdrawn mid-move -- halting"));
    haltNow();
  }
  permitted = ok;
}

bool motionPermitted() { return permitted; }

void enableDriver(bool on) {
  if (!on && stepper::running()) haltNow();
  stepper::enableDriver(on);
  if (!on && isHomed) {
    /* No holding torque means the carriage can be moved by hand, so the
     * counter can no longer be trusted to match it. */
    isHomed = false;
    if (st != RAIL_STATE_FAULT) st = RAIL_STATE_UNHOMED;
    Serial.println(F("[RAIL] driver disabled -- now UNHOMED"));
  }
}

bool driverEnabled() { return stepper::driverEnabled(); }

// --- queries ----------------------------------------------------------------

uint8_t state() { return st; }
uint8_t fault() { return flt; }
bool homed() { return isHomed; }
bool travelMeasured() { return measured; }
int32_t position() { return stepper::position(); }
int32_t target() { return moveTarget; }
uint32_t travelSteps() { return travel; }
uint32_t rateStepsPerSec() { return stepper::running() ? (uint32_t)rateNow : 0; }
bool limitHome() { return triggered(swHome); }
bool limitIdle() { return triggered(swIdle); }
int32_t softMin() { return softMinSteps(); }
int32_t softMax() { return softMaxSteps(); }

const char *stateName() {
  switch (st) {
    case RAIL_STATE_UNHOMED: return "UNHOMED";
    case RAIL_STATE_HOMING: return "HOMING";
    case RAIL_STATE_IDLE: return "IDLE";
    case RAIL_STATE_MOVING: return "MOVING";
    case RAIL_STATE_SWEEPING: return "SWEEPING";
    case RAIL_STATE_MEASURING: return "MEASURING";
    case RAIL_STATE_FAULT: return "FAULT";
    default: return "?";
  }
}

const char *faultName() { return faultText(flt); }

}  // namespace rail

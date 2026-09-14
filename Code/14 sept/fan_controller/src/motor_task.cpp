#include "motor_task.h"

#include <SimpleFOC.h>
#include <Wire.h>
#include <atomic>

#include "config.h"

/* ---------------------------------------------------------------------------
 * MECHANICAL NOTE, and it matters more later than it does now:
 *
 * This is an outrunner, and the display arm is bolted directly to the motor's
 * rotating face. There is no gearing -- the ratio is 1:1, so motor RPM *is*
 * display RPM.
 *
 * The consequence worth remembering: the AS5600 sits inside the motor reading
 * the same rotating bell the arm is bolted to. It is therefore not merely
 * measuring motor shaft position, it is measuring THE ABSOLUTE ANGULAR
 * POSITION OF THE DISPLAY ARM, in the stator's frame, at ~4096 counts per
 * revolution.
 *
 * When rotor sync arrives in stage 2 that is exactly the quantity needed to
 * tell the S3 which angular column to paint. It means the stator already
 * knows the arm's absolute angle without waiting for the rotor's Hall sensor
 * to come round, and the Hall pulse becomes a once-per-revolution correction
 * rather than the only source of truth.
 * ------------------------------------------------------------------------ */

namespace {

MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);
BLDCMotor motor = BLDCMotor(MOTOR_POLE_PAIRS);
BLDCDriver3PWM driver = BLDCDriver3PWM(PIN_IN1, PIN_IN2, PIN_IN3, PIN_EN);

/* AS5600 registers we read directly, because SimpleFOC exposes none of them.
 *
 * STATUS (0x0B), per the ams datasheet v1-06 section 5.1.2. Getting this wrong
 * cost a bring-up session, so it is spelled out:
 *
 *      bit 5   MD   Magnet Detected      -- must be 1 for the output to be valid
 *      bit 4   ML   Magnet too weak      -- AGC has hit its MAXIMUM gain limit
 *      bit 3   MH   Magnet too strong    -- AGC has hit its MINIMUM gain limit
 *      bits 0-2, 6-7                     -- reserved, and do NOT reliably read 0
 *
 * Only MD is a validity flag. ML and MH are gain-limit warnings: they say the
 * AGC has run out of headroom in one direction, not that the angle is unusable.
 *
 * AGC (0x1A): 0 = gain floored, magnet too close. Maximum = gain railed, magnet
 * too far. The maximum is 255 on a 5 V supply but only 128 on 3.3 V, which is
 * what this board runs -- so AGC 128 here means railed, not mid-range.
 *
 * MAGNITUDE (0x1B): 12-bit internal field magnitude. The honest "is there a
 * magnet" number, independent of the flag bits. */
const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_REG_STATUS = 0x0B;
const uint8_t AS5600_REG_RAW_ANGLE = 0x0C;
const uint8_t AS5600_REG_AGC = 0x1A;
const uint8_t AS5600_REG_MAGNITUDE = 0x1B;
const uint8_t AS5600_REG_CONF_HI = 0x07;  // CONF[13:8]; SF is bits 1:0 here

SemaphoreHandle_t stateMutex = nullptr;
QueueHandle_t commandQueue = nullptr;

// Emergency stop is deliberately NOT a queued command: it must not wait behind
// whatever else a browser has just posted.
std::atomic<bool> estopFlag{false};

enum class CmdType : uint8_t { Power, Speed, ClearFault };
struct Command {
  CmdType type;
  float value;
};

// --- state owned exclusively by the motor task ---------------------------
bool power = false;
FanState state = FanState::Idle;
float rpmTarget = 0.0f;    // user setpoint
float rpmCommand = 0.0f;   // ramp output, what the PID is actually chasing
float rpmActual = 0.0f;  // fast, LPF-filtered -- what the PID chases
bool motorEnabled = false;

/* Latches once a stop has fully finished (ramp at zero, arm stationary). Keeps
 * the driver energised through the whole ramp-down, then off for good until
 * power is requested again -- rather than re-energising if the shaft is nudged.
 * An emergency stop sets it immediately, which is what makes E-STOP coast while
 * a normal stop decelerates under control. */
bool stopComplete = true;

/* Slow, displacement-based speed estimate, and the reason it exists.
 *
 * The AS5600 is 12-bit, so one count is 360/4096 = 0.088 deg. At the measured
 * 1336 Hz loop rate the sample interval is 0.75 ms, so a SINGLE count of
 * dither differentiates to 0.088/0.00075 = 117 deg/s = 19.5 rpm. That is
 * exactly the +/-20 rpm the first bring-up run showed on a shaft that was
 * provably stationary (raw angle rock steady at 2744, spread of one count).
 *
 * It is harmless for control -- at 150 rpm the real signal swamps it, and the
 * LPF smooths it. But it broke two things that test for "stopped":
 * the idle/stopping state machine, and faultCauseActive(), which refuses to
 * clear a motion fault while the arm is still turning. With the fast estimate
 * never settling below RPM_STOPPED, a latched fault could NEVER be cleared.
 *
 * Measuring angular DISPLACEMENT over a long window instead kills the noise
 * dead: over 250 ms, 5 rpm is ~85 counts of travel against +/-1 count of
 * dither. Same sensor, ~500x the discrimination, no filtering needed. */
float rpmSlow = 0.0f;

bool faultLatched = false;
char faultCode[8] = {0};
char faultMessage[112] = {0};

// Sensor health, refreshed periodically rather than every iteration -- the
// status read is an extra I2C transaction and the loop budget is precious.
bool magnetOk = true;      // MD -- gates operation
bool magnetWeak = false;   // ML -- AGC gain railed high. Warning only.
bool magnetStrong = false; // MH -- AGC gain floored. Warning only.
bool i2cOk = true;

uint8_t lastStatusReg = 0;  // last raw AS5600 STATUS byte, for diagnostics
uint8_t sensorFilterSF = 0xFF;  // 0xFF = not yet read/set

// Slow-filter step-response delay, microseconds, indexed by SF. Datasheet.
const uint16_t FILTER_DELAY_US[4] = {2200, 1100, 550, 286};

// Set from another task to request a filter change; applied on the motor task,
// which owns the I2C bus.
std::atomic<bool> filterCycleRequest{false};

// DRV8313 nFAULT, active low. Latched by the driver itself on overcurrent
// until nRESET is pulsed or VM is cycled.
bool driverOk = true;

// Set from any task to ask the motor task to dump sensor diagnostics; the dump
// itself must run on the motor task, since it shares the I2C bus with the
// control loop's angle reads.
std::atomic<bool> diagRequest{false};

// --- instrumentation ------------------------------------------------------
uint32_t iterCount = 0;
uint32_t focLoopHz = 0;
uint32_t lastIterUs = 0;
float maxGapMs = 0.0f;     // worst since boot
float gapMsRecent = 0.0f;  // worst in the last second, so it recovers
float gapMsAccum = 0.0f;

// Velocity ripple over a one-revolution window -- the imbalance proxy.
float rippleMin = 0.0f, rippleMax = 0.0f, rpmRipple = 0.0f;
uint32_t rippleWindowStartMs = 0;

// Fault dwell timers
uint32_t speedErrorSinceMs = 0;
uint32_t saturatedSinceMs = 0;
uint32_t rippleHighSinceMs = 0;
uint32_t stalledSinceMs = 0;

Telemetry published{};

// -------------------------------------------------------------------------

void latchFault(const char *code, const char *message) {
  if (faultLatched) return;  // first cause wins; do not overwrite the story
  faultLatched = true;
  state = FanState::Fault;
  power = false;
  snprintf(faultCode, sizeof(faultCode), "%s", code);
  snprintf(faultMessage, sizeof(faultMessage), "%s", message);

  /* Coast, like E-STOP. Actively braking a high-inertia arm on a voltage-mode
   * FOC with no current sensing pushes regenerated energy back into a mains
   * adapter that has nowhere to put it, raising the DC bus.
   *
   * stopComplete must be set here too: without it, clearing the fault would
   * leave power false and stopComplete false, which the ramp-down logic reads
   * as "still stopping" and would silently re-energise the driver the moment
   * the fault cleared. */
  motor.disable();
  motorEnabled = false;
  stopComplete = true;
  rpmCommand = 0.0f;
  rpmTarget = 0.0f;

  Serial.printf("[FAULT] %s  %s\n", code, message);
}

/* Is the CAUSE of the latched fault still present? /api/fault/clear must
 * refuse while it is. Fills `reason` with something a member of the public
 * can act on. */
bool faultCauseActive(char *reason, size_t reasonLen) {
  if (!faultLatched) {
    reason[0] = '\0';
    return false;
  }

  if (!i2cOk) {
    snprintf(reason, reasonLen,
             "The position sensor still is not responding. Check the motor's "
             "sensor cable, then try again.");
    return true;
  }
  if (!driverOk) {
    /* Still asserted. The clear path pulses nRESET before re-reading, so if we
     * are here the driver is either still over temperature or is re-tripping
     * immediately -- neither of which a button press should override. */
    snprintf(reason, reasonLen,
             "The motor driver is still reporting a problem. Let it cool for a "
             "minute, then try again.");
    return true;
  }
  if (!magnetOk) {
    snprintf(reason, reasonLen,
             "The position sensor still cannot see its magnet. Check the motor "
             "is properly assembled, then try again.");
    return true;
  }

  // Everything else is a motion fault, and the honest test for "has the cause
  // gone away" is that the arm has actually stopped moving. This also stops
  // anyone clearing a fault and restarting while a half-metre arm is still
  // coasting at speed.
  //
  // rpmSlow, not rpmActual: the differentiated estimate never settles below
  // RPM_STOPPED because of encoder quantisation, so testing it here would make
  // every motion fault permanently unclearable.
  if (fabsf(rpmSlow) > RPM_STOPPED) {
    snprintf(reason, reasonLen,
             "The fan is still spinning down. Wait until the arm has stopped, "
             "then try again.");
    return true;
  }

  reason[0] = '\0';
  return false;
}

/* Read n bytes from an AS5600 register. Returns false on any I2C failure,
 * unlike SimpleFOC's own getRawCount(), which ignores requestFrom()'s return
 * value and happily returns whatever wire->read() gives it on a dead bus --
 * which is precisely why a broken sensor shows up as plausible-looking noise
 * rather than as an error. */
bool as5600Read(uint8_t reg, uint8_t *buf, size_t n) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)n) != (int)n) return false;
  for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

bool as5600Write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

/* Sets the AS5600's slow filter (CONF bits 9:8 = bits 1:0 of register 0x07).
 *
 * This is the single biggest lever on commutation quality at speed. The default
 * SF=0 is the 16x filter with a 2.2 ms step response, which at 700 rpm and 7
 * pole pairs is ~65 deg of electrical lag on its own. SF=3 (2x, 0.286 ms) cuts
 * that to ~8 deg.
 *
 * CONF is volatile unless burned, so this is a plain register write that lasts
 * until power-off -- nothing is permanently programmed into the part. */
bool setSensorFilter(uint8_t sf) {
  sf &= 0x03;
  uint8_t hi = 0;
  if (!as5600Read(AS5600_REG_CONF_HI, &hi, 1)) return false;
  const uint8_t updated = (uint8_t)((hi & ~0x03) | sf);
  if (!as5600Write(AS5600_REG_CONF_HI, updated)) return false;

  // Read back -- a write that silently did not take would be worse than none.
  uint8_t check = 0;
  if (!as5600Read(AS5600_REG_CONF_HI, &check, 1)) return false;
  if ((check & 0x03) != sf) {
    Serial.printf("[MOTOR] filter write did not stick: wanted SF=%u, got %u\n",
                  sf, check & 0x03);
    return false;
  }
  sensorFilterSF = sf;
  Serial.printf("[MOTOR] AS5600 slow filter SF=%u (%ux, %u us step response)\n",
                sf, 16 >> sf, FILTER_DELAY_US[sf]);
  return true;
}

/* Dumps everything the AS5600 will tell us about itself. Runs ON THE MOTOR
 * TASK -- never call it from another task, because it shares the I2C bus with
 * loopFOC()'s angle reads. */
void dumpSensorDiag() {
  Serial.println(F("\n---- AS5600 diagnostics ----"));
  Serial.printf("I2C: SDA=%d SCL=%d, clock %lu Hz\n", SDA, SCL,
                (unsigned long)I2C_CLOCK_HZ);

  // Bus scan first: if 0x36 does not answer here, nothing else matters and the
  // problem is wiring or power to the sensor, not configuration.
  Serial.print(F("scan:"));
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", addr);
      found++;
    }
  }
  if (!found) Serial.print(F(" nothing responded"));
  Serial.println();

  if (!found) {
    Serial.println(F("=> No I2C device at all. Check the motor's sensor cable:\n"
                     "   the AS5600 is inside the motor and needs 3V3 and GND\n"
                     "   from the board as well as SDA/SCL."));
    Serial.println(F("----------------------------\n"));
    return;
  }

  uint8_t status = 0, agc = 0, mag[2] = {0, 0}, ang[2] = {0, 0};
  const bool okStatus = as5600Read(AS5600_REG_STATUS, &status, 1);
  const bool okAgc = as5600Read(AS5600_REG_AGC, &agc, 1);
  const bool okMag = as5600Read(AS5600_REG_MAGNITUDE, mag, 2);

  if (!okStatus || !okAgc || !okMag) {
    Serial.println(F("=> The chip answers its address but register reads fail.\n"
                     "   That is the signature of a marginal bus: try a lower\n"
                     "   I2C_CLOCK_HZ, shorter wires, or stronger pull-ups."));
    Serial.println(F("----------------------------\n"));
    return;
  }

  const bool md = (status & 0x20) != 0;  // bit 5
  const bool ml = (status & 0x10) != 0;  // bit 4
  const bool mh = (status & 0x08) != 0;  // bit 3
  Serial.printf("status=0x%02X  MD(detected)=%d ML(too weak)=%d MH(too strong)=%d\n",
                status, md, ml, mh);
  Serial.println(F("  (bits 0-2 and 6-7 are reserved and do not reliably read 0"
                   " -- ignore them)"));
  Serial.printf("agc=%u  (0 = gain floored/magnet too close; max = railed/too far."
                "\n         Max is 128 on this board's 3V3 supply, NOT 255.)\n",
                agc);
  Serial.printf("magnitude=%u  (field strength, independent of the flag bits)\n",
                (unsigned)(((uint16_t)mag[0] << 8) | mag[1]));
  Serial.printf("SimpleFOC currWireError=%u\n", sensor.currWireError);
  if (sensorFilterSF <= 3) {
    Serial.printf("slow filter SF=%u (%ux, %u us) -> %.0f deg electrical lag at "
                  "700 rpm\n",
                  sensorFilterSF, 16 >> sensorFilterSF,
                  FILTER_DELAY_US[sensorFilterSF],
                  360.0f * (700.0f / 60.0f * MOTOR_POLE_PAIRS) *
                      (FILTER_DELAY_US[sensorFilterSF] * 1e-6f));
  }

  if (!md) {
    Serial.println(F("=> MD clear: no magnet detected. The angle output is not"
                     " valid."));
  } else if (ml) {
    Serial.println(F("=> Magnet detected and usable, but the AGC is railed at"
                     " maximum gain\n"
                     "   (field at the weak end). It works, with no gain"
                     " headroom left -- if the\n"
                     "   air gap grows, this is where it would start to fail."
                     " Worth a look at the\n"
                     "   magnet seating, but not a reason to stop."));
  } else if (mh) {
    Serial.println(F("=> Magnet detected but very strong / too close. Works,"
                     " no headroom the other way."));
  } else {
    Serial.println(F("=> Magnet healthy, AGC has headroom in both directions."));
  }

  // Ten raw angle reads. Stable-ish on a stationary shaft means the bus is
  // sound; wild scatter means we are reading noise, which is what produces a
  // stationary motor apparently spinning at 12-37 rpm.
  Serial.print(F("raw angle x10:"));
  uint16_t lo = 0xFFFF, hi = 0;
  int reads = 0;
  for (int i = 0; i < 10; i++) {
    if (as5600Read(AS5600_REG_RAW_ANGLE, ang, 2)) {
      const uint16_t a = (((uint16_t)ang[0] & 0x0F) << 8) | ang[1];
      Serial.printf(" %4u", a);
      lo = min(lo, a);
      hi = max(hi, a);
      reads++;
    } else {
      Serial.print(F("  ERR"));
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
  Serial.println();
  if (reads) {
    Serial.printf("spread over ~200 ms: %u counts (%.1f deg)\n",
                  (unsigned)(hi - lo), (hi - lo) * 360.0f / 4096.0f);
    Serial.println(F("  A stationary shaft should sit within a few counts.\n"
                     "  Tens or hundreds of counts of scatter means the reads\n"
                     "  are unreliable, not that the shaft is moving."));
  }
  Serial.println(F("Turn the shaft slowly by hand and run this again -- the\n"
                   "angle should track smoothly through 0-4095."));
  Serial.println(F("----------------------------\n"));

  /* This dump deliberately sleeps ~200 ms between angle reads, which shows up
   * as one large inter-iteration gap. That is self-inflicted, not a scheduling
   * problem, so do not leave it sitting in the high-water mark pretending to be
   * one. (The first run reported 221 ms from exactly this.) */
  maxGapMs = 0.0f;
  gapMsAccum = 0.0f;
}

void refreshDriverHealth() { driverOk = digitalRead(PIN_DRV_FAULT) == HIGH; }

/* Pulse nRESET low. Per the DRV8313 datasheet this is the only way, short of
 * cycling VM, to bring a channel back after an overcurrent trip has latched
 * it off. Blocking for a few ms here is acceptable: it only ever runs from a
 * fault-clear request, when the motor is already stopped and disabled. */
void resetDriver() {
  Serial.println(F("[MOTOR] pulsing driver nRESET"));
  digitalWrite(PIN_DRV_RESET, LOW);
  vTaskDelay(pdMS_TO_TICKS(DRV_RESET_PULSE_MS));
  digitalWrite(PIN_DRV_RESET, HIGH);
  vTaskDelay(pdMS_TO_TICKS(DRV_RESET_PULSE_MS));
  refreshDriverHealth();
}

void refreshSensorHealth() {
  // SimpleFOC records the Wire.endTransmission() result of its last read.
  i2cOk = (sensor.currWireError == 0);
  if (!i2cOk) return;

  uint8_t status = 0;
  if (!as5600Read(AS5600_REG_STATUS, &status, 1)) {
    i2cOk = false;
    return;
  }
  lastStatusReg = status;

  /* ONLY MD gates operation. ML and MH mean the AGC has run out of gain
   * headroom in one direction or the other -- worth telling the user about,
   * but not a reason to refuse to spin: the datasheet ties output validity to
   * MD, and a railed AGC still produces a usable angle. This board reads
   * STATUS=0x33 (MD=1, ML=1, MH=0) with an angle stable to one count, which is
   * exactly that situation. Latching a fault on it was wrong. */
  magnetOk = (status & 0x20) != 0;              // MD, bit 5
  magnetWeak = (status & 0x10) != 0;            // ML, bit 4
  magnetStrong = (status & 0x08) != 0;          // MH, bit 3
}

void updateRamp(float dtSec) {
  const float goal = power ? rpmTarget : 0.0f;
  const float step = RAMP_RPM_PER_S * dtSec;

  // Rate-limited in BOTH directions and between arbitrary current and target
  // speeds -- so a mid-run speed change is smooth whichever way it goes, not
  // just the initial spin-up from rest.
  if (rpmCommand < goal) {
    rpmCommand = min(goal, rpmCommand + step);
  } else if (rpmCommand > goal) {
    rpmCommand = max(goal, rpmCommand - step);
  }
}

void updateState() {
  if (faultLatched) {
    state = FanState::Fault;
    return;
  }
  if (power) {
    const float band = max(SPEED_ERROR_RPM, rpmTarget * SPEED_ERROR_FRACTION);
    const bool atSpeed = fabsf(rpmCommand - rpmTarget) < 0.5f &&
                         fabsf(rpmActual - rpmTarget) < band;
    state = atSpeed ? FanState::Running : FanState::Starting;
  } else {
    // rpmSlow: the fast estimate's quantisation noise would keep the fan
    // reporting "stopping" forever and never reach idle.
    state = (fabsf(rpmSlow) > RPM_STOPPED) ? FanState::Stopping : FanState::Idle;
  }
}

void updateRipple(uint32_t nowMs) {
  /* Below the speed where imbalance detection is even meaningful, hold at zero.
   *
   * Two reasons. The figure down here is pure encoder quantisation noise, not
   * balance. And the window is one revolution, so at a near-zero speed it
   * stretches to tens of seconds -- which meant any one-off spike (the 221 ms
   * stall from a diagnostic dump, say) froze a nonsense value on the readout
   * for twelve seconds afterwards. */
  if (fabsf(rpmActual) < RIPPLE_MIN_RPM) {
    rpmRipple = 0.0f;
    rippleMin = rippleMax = rpmActual;
    rippleWindowStartMs = nowMs;
    return;
  }

  rippleMin = min(rippleMin, rpmActual);
  rippleMax = max(rippleMax, rpmActual);

  // One window == one revolution, since an imbalance disturbance repeats once
  // per revolution.
  const float rpmForWindow = max(fabsf(rpmActual), RIPPLE_MIN_RPM);
  const uint32_t windowMs = (uint32_t)(60000.0f / rpmForWindow);

  if (nowMs - rippleWindowStartMs >= windowMs) {
    rpmRipple = rippleMax - rippleMin;
    rippleMin = rippleMax = rpmActual;
    rippleWindowStartMs = nowMs;
  }
}

void checkFaults(uint32_t nowMs) {
  if (faultLatched) return;

  /* Driver first: it is the only fault reported by hardware rather than
   * inferred, and it means the driver has ALREADY shut a channel down. */
  if (!driverOk) {
    latchFault("E04",
               "The motor driver has shut down to protect itself, usually from "
               "overheating or drawing too much power. The fan has been "
               "stopped. Let it cool before trying again.");
    return;
  }

  if (!i2cOk) {
    latchFault("E02",
               "Lost contact with the motor's position sensor. The fan has been "
               "stopped. This usually means a loose sensor connection.");
    return;
  }
  if (!magnetOk) {
    latchFault("E01",
               "The motor's position sensor cannot see its magnet, so the fan "
               "cannot tell how fast it is turning. The fan has been stopped.");
    return;
  }

  // Runaway: turning faster than anything we asked for.
  if (fabsf(rpmActual) > RPM_MAX_ALLOWED * OVERSPEED_FACTOR) {
    latchFault("E06",
               "The fan is turning faster than it should be. It has been "
               "stopped as a precaution.");
    return;
  }

  const bool spinningUnderPower = power && motorEnabled;

  // Sustained tracking error. Only meaningful once the ramp has arrived --
  // during the ramp a large error is simply the ramp doing its job.
  if (spinningUnderPower && fabsf(rpmCommand - rpmTarget) < 0.5f) {
    const float band = max(SPEED_ERROR_RPM, rpmTarget * SPEED_ERROR_FRACTION);
    if (fabsf(rpmActual - rpmTarget) > band) {
      if (speedErrorSinceMs == 0) speedErrorSinceMs = nowMs;
      if (nowMs - speedErrorSinceMs > SPEED_ERROR_MS) {
        latchFault("E03",
                   "The fan cannot reach the speed it was asked for. Check that "
                   "nothing is touching or obstructing the arm.");
        return;
      }
    } else {
      speedErrorSinceMs = 0;
    }
  } else {
    speedErrorSinceMs = 0;
  }

  /* Stall. Needed because the tracking band above has an absolute floor, and
   * at a low target that floor can exceed the target itself -- at 60 rpm the
   * band is 40 rpm, so a motor stuck at 25 rpm would sit inside tolerance and
   * E03 would never fire. */
  if (spinningUnderPower && fabsf(rpmCommand - rpmTarget) < 0.5f &&
      fabsf(rpmSlow) < rpmTarget * STALL_FRACTION) {
    if (stalledSinceMs == 0) stalledSinceMs = nowMs;
    if (nowMs - stalledSinceMs > STALL_MS) {
      latchFault("E05",
                 "The fan is not turning even though the motor is running. "
                 "Check that nothing is jamming the arm.");
      return;
    }
  } else {
    stalledSinceMs = 0;
  }

  /* Voltage saturation: the controller has run out of authority. Directly
   * commanded, so unlike a current estimate it needs no model of the motor to
   * be meaningful. See the SATURATION_FRACTION note in config.h for why this
   * replaced the old estimated-current fault. */
  if (spinningUnderPower &&
      fabsf(motor.voltage.q) > MOTOR_VOLTAGE_LIMIT * SATURATION_FRACTION) {
    if (saturatedSinceMs == 0) saturatedSinceMs = nowMs;
    if (nowMs - saturatedSinceMs > SATURATION_MS) {
      latchFault("E09",
                 "The motor is working as hard as it can and still cannot keep "
                 "up. The fan has been stopped to protect the electronics.");
      return;
    }
  } else {
    saturatedSinceMs = 0;
  }

  // Imbalance proxy -- see the RIPPLE_FAULT_RPM note in config.h.
  if (spinningUnderPower && state == FanState::Running &&
      fabsf(rpmActual) > RIPPLE_MIN_RPM && rpmRipple > RIPPLE_FAULT_RPM) {
    if (rippleHighSinceMs == 0) rippleHighSinceMs = nowMs;
    if (nowMs - rippleHighSinceMs > RIPPLE_FAULT_MS) {
      latchFault("E08",
                 "The arm is running unevenly, which usually means it is out of "
                 "balance. The fan has been stopped.");
      return;
    }
  } else {
    rippleHighSinceMs = 0;
  }
}

void drainCommands() {
  Command cmd;
  while (xQueueReceive(commandQueue, &cmd, 0) == pdTRUE) {
    switch (cmd.type) {
      case CmdType::Power:
        if (faultLatched) break;  // refuse while latched; the UI is told why
        power = cmd.value > 0.5f;
        if (power && rpmTarget < RPM_MIN_ALLOWED) rpmTarget = RPM_MIN_ALLOWED;
        break;

      case CmdType::Speed:
        rpmTarget = cmd.value;  // already clamped by clampRpm() at the edge
        break;

      case CmdType::ClearFault: {
        // A latched driver trip stays latched until nRESET is pulsed, so try
        // that BEFORE deciding whether the cause is still present -- otherwise
        // an overcurrent fault could never be cleared from the web page.
        if (!driverOk) resetDriver();

        char reason[112];
        if (!faultCauseActive(reason, sizeof(reason))) {
          faultLatched = false;
          faultCode[0] = '\0';
          faultMessage[0] = '\0';
          speedErrorSinceMs = saturatedSinceMs = 0;
          rippleHighSinceMs = stalledSinceMs = 0;
          state = FanState::Idle;
          Serial.println(F("[FAULT] cleared"));
        }
        break;
      }
    }
  }
}

void publish(float estCurrent) {
  Telemetry t{};
  t.power = power;
  t.state = state;
  t.rpmTarget = rpmTarget;
  t.rpmCommand = rpmCommand;
  t.rpmActual = rpmSlow;   // the honest one -- see the Telemetry comments
  t.rpmFast = rpmActual;   // the noisy control-loop one, diagnostics only
  t.magnetWeak = magnetWeak;
  t.magnetStrong = magnetStrong;
  t.faultLatched = faultLatched;
  snprintf(t.faultCode, sizeof(t.faultCode), "%s", faultCode);
  snprintf(t.faultMessage, sizeof(t.faultMessage), "%s", faultMessage);
  t.faultCauseActive =
      faultCauseActive(t.clearBlockedReason, sizeof(t.clearBlockedReason));
  t.focLoopHz = focLoopHz;
  t.focMaxGapMs = maxGapMs;
  t.focGapMsRecent = gapMsRecent;
  t.rpmRipple = rpmRipple;
  t.voltageQ = motor.voltage.q;
  t.estCurrentA = estCurrent;

  /* A short timeout, not portMAX_DELAY: the control loop must never wait on a
   * reader. Missing one publish is harmless -- the next one is 3 ms away. */
  if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
    published = t;
    xSemaphoreGive(stateMutex);
  }
}

/* Runs once per burst (~333 Hz), not once per FOC iteration. Everything that
 * is not the control loop itself lives here. */
void supervise() {
  static uint32_t lastMs = 0;
  static uint32_t lastHzMs = 0;
  static uint32_t lastSensorCheckMs = 0;

  const uint32_t nowMs = millis();
  if (lastMs == 0) lastMs = nowMs;
  const float dt = (nowMs - lastMs) / 1000.0f;
  lastMs = nowMs;

  rpmActual = radSToRpm(motor.shaft_velocity);

  /* Displacement-based speed over a 250 ms window -- see the rpmSlow comment
   * for why the differentiated estimate above cannot be trusted near zero. */
  {
    static float lastAngle = 0.0f;
    static uint32_t lastSlowMs = 0;
    if (lastSlowMs == 0) {
      lastAngle = motor.shaft_angle;
      lastSlowMs = nowMs;
    } else if (nowMs - lastSlowMs >= 250) {
      const float dt2 = (nowMs - lastSlowMs) / 1000.0f;
      rpmSlow = radSToRpm((motor.shaft_angle - lastAngle) / dt2);
      lastAngle = motor.shaft_angle;
      lastSlowMs = nowMs;
    }
  }

  if (estopFlag.exchange(false)) {
    /* Skip the ramp entirely and cut the drive, exactly as API.md specifies.
     * stopComplete is set here so the ramp-down logic below does NOT keep the
     * driver alive -- that distinction is the whole difference between the two
     * stop buttons:
     *
     *    power off  -> controlled deceleration, motor regulating all the way
     *    E-STOP     -> drive removed instantly, arm coasts
     *
     * It coasts rather than braking because dumping the arm's kinetic energy
     * into a mains adapter in a fraction of a second has nowhere to go, and
     * there is no brake resistor. See the note in the enable logic below.
     *
     * rpmTarget is deliberately left alone, so restarting returns to the speed
     * the slider is still showing rather than silently jumping to the minimum. */
    power = false;
    rpmCommand = 0.0f;
    stopComplete = true;
    motor.disable();
    motorEnabled = false;
    Serial.println(F("[MOTOR] EMERGENCY STOP -- drive removed, arm coasting"));
  }

  drainCommands();

  // Diagnostics run here, on the motor task, because they share the I2C bus
  // with the control loop. Only ever requested while stopped.
  if (diagRequest.exchange(false)) dumpSensorDiag();

  if (filterCycleRequest.exchange(false)) {
    setSensorFilter((uint8_t)((sensorFilterSF + 1) & 0x03));
  }

  // nFAULT is a plain GPIO read, so it is cheap enough to poll every pass --
  // and it is the one fault the hardware has already acted on, so latency here
  // is latency on telling the user why the fan stopped.
  refreshDriverHealth();

  if (nowMs - lastSensorCheckMs >= 250) {
    lastSensorCheckMs = nowMs;
    refreshSensorHealth();
  }

  updateRamp(dt);

  /* Stay driving through the ramp-DOWN, not just the ramp-up.
   *
   * This used to be `wantEnabled = !faultLatched && power`, which disabled the
   * driver the instant power went false. The ramp-down was still being
   * computed, but with the driver off nothing applied it, so every stop was
   * really a coast -- abrupt on a bare shaft, and nothing like the controlled
   * deceleration the ramp was there to provide.
   *
   * Now the motor keeps regulating all the way down and only shuts off once
   * the ramp has arrived at zero AND the arm has actually stopped. stopComplete
   * latches that, so a disabled fan does not re-energise just because someone
   * turns the shaft by hand.
   *
   * On the braking current this implies: decelerating under control means the
   * PID commands negative torque, and that energy goes back into a DC bus fed
   * by a mains adapter. At RAMP_RPM_PER_S = 60 it is a trickle -- roughly
   * 0.35 A and under a watt even with the arm fitted, because the torque needed
   * to shed 60 rpm per second is small and friction does much of the work.
   * That is why a controlled ramp-down is safe while a hard electrical brake
   * is not, and it is why E-STOP still coasts instead. Raise RAMP_RPM_PER_S a
   * long way and the braking current scales with it. */
  if (power) {
    stopComplete = false;
  } else if (!stopComplete && rpmCommand <= 0.5f &&
             fabsf(rpmSlow) <= RPM_STOPPED) {
    stopComplete = true;
    Serial.println(F("[MOTOR] ramp-down complete, driver off"));
  }

  const bool wantEnabled = !faultLatched && (power || !stopComplete);
  if (wantEnabled && !motorEnabled) {
    /* Pick the ramp up from wherever the arm ACTUALLY is, not from zero.
     *
     * This matters when restarting while the arm is still coasting -- which is
     * easy to do, because an outrunner with a half-metre arm on it takes a
     * long time to stop. Starting the ramp at zero would hand the PID a large
     * negative error and it would brake the arm electrically to obey. With no
     * brake resistor and a mains adapter rather than a battery on the DC bus,
     * that regenerated energy has nowhere to go and pushes the rail up.
     *
     * Starting from the measured speed also makes the ramp genuinely "between
     * arbitrary current and target speeds" rather than only from rest. */
    rpmCommand = constrain(fabsf(rpmActual), 0.0f, RPM_MAX_ALLOWED);

    // BLDCMotor::enable() resets PID_velocity itself, so the integrator
    // accumulated before the last stop is already dropped here -- no separate
    // reset needed.
    motor.enable();
    motorEnabled = true;
  } else if (!wantEnabled && motorEnabled) {
    motor.disable();
    motorEnabled = false;
  }

  /* Reported for the diagnostics page only -- this figure is known to be
   * wrong (it goes negative while motoring; see the KV warning in config.h).
   * No fault depends on it. It is kept because it is what motor_test printed,
   * so the two are comparable, and because whichever way the KV question
   * resolves, the shape of this number over speed is a clue. */
  const float backEmf = fabsf(rpmActual) / KV_RATING;
  const float estCurrent = (motor.voltage.q - backEmf) / PHASE_RESISTANCE;

  updateRipple(nowMs);
  updateState();
  checkFaults(nowMs);

  if (nowMs - lastHzMs >= 1000) {
    focLoopHz = iterCount;
    iterCount = 0;
    gapMsRecent = gapMsAccum;
    gapMsAccum = 0.0f;
    lastHzMs = nowMs;
  }

  publish(estCurrent);
}

void focTask(void *) {
  lastIterUs = micros();
  rippleWindowStartMs = millis();

  for (;;) {
    const uint32_t burstStart = micros();
    do {
      const uint32_t now = micros();
      const float gapMs = (now - lastIterUs) / 1000.0f;
      if (gapMs > maxGapMs) maxGapMs = gapMs;
      if (gapMs > gapMsAccum) gapMsAccum = gapMs;
      lastIterUs = now;

      // loopFOC() and move() are called as a pair on every iteration, exactly
      // as motor_test does. Calling move() less often would change the
      // effective control dynamics, and that tuning is tested data.
      motor.loopFOC();
      motor.move(rpmToRadS(rpmCommand));
      iterCount++;

      if (estopFlag.load()) break;
    } while ((uint32_t)(micros() - burstStart) < FOC_BURST_US);

    supervise();

    /* Yield. This is the whole single-core design in one line: the task sleeps
     * a full FreeRTOS tick so AsyncTCP (priority 10, below us) gets to run at
     * all. Busy-spinning here would keep the web server off the CPU and,
     * worse, starve the WiFi driver into dropping the AP. */
    vTaskDelay(1);
  }
}

}  // namespace

const char *fanStateName(FanState s) {
  switch (s) {
    case FanState::Idle: return "idle";
    case FanState::Starting: return "starting";
    case FanState::Running: return "running";
    case FanState::Stopping: return "stopping";
    case FanState::Fault: return "fault";
  }
  return "idle";
}

namespace motorctl {

float clampRpm(float rpm) {
  if (isnan(rpm)) return RPM_MIN_ALLOWED;
  /* Zero passes through instead of being clamped up to RPM_MIN_ALLOWED.
   *
   * The website's slider now starts at 0, and 0 there means STOP -- so
   * clamping it to 50 would answer a stop request with "running at 50 rpm",
   * and the readout would never agree with the setpoint. handleSpeed() turns
   * this 0 into a controlled power-down; everything above it is held inside
   * the band the gains were tuned for. */
  if (rpm <= 0.0f) return 0.0f;
  return constrain(rpm, RPM_MIN_ALLOWED, RPM_MAX_ALLOWED);
}

bool begin() {
  stateMutex = xSemaphoreCreateMutex();
  commandQueue = xQueueCreate(8, sizeof(Command));

  /* Driver fault pins first, before the driver is initialised. nRESET is
   * driven HIGH (inactive) immediately so the DRV8313 is not sitting held in
   * reset, and nFAULT gets an internal pull-up in case the DRI0058 does not
   * provide one -- without a pull-up an open-drain output floats and would
   * read as a permanent fault. */
  pinMode(PIN_DRV_RESET, OUTPUT);
  digitalWrite(PIN_DRV_RESET, HIGH);
  pinMode(PIN_DRV_FAULT, INPUT_PULLUP);

  /* Order matters here, and getting it wrong is invisible.
   *
   * setClock() must come AFTER begin(), because TwoWire::setClock() returns
   * early if the bus lock does not exist yet -- which is the case before
   * begin(). That is exactly how motor_test ended up running at 100 kHz while
   * its comment claimed 400 kHz. See I2C_CLOCK_HZ in config.h.
   *
   * sensor.init() calls Wire.begin() itself; calling it here first makes the
   * ordering explicit rather than incidental. A second begin() inside
   * sensor.init() is a harmless no-op (Arduino-ESP32 logs "Bus already started"
   * and returns), and notably does NOT reset the clock. */
  Wire.begin();
  Wire.setClock(I2C_CLOCK_HZ);
  // Arduino-ESP32's default Wire timeout is 50 ms, which would blow the 5 ms
  // sensor-gap budget an order of magnitude over on a single stuck read. A
  // timeout here surfaces as E02 rather than as a stalled control loop.
  Wire.setTimeOut(10);

  sensor.init();
  Wire.setClock(I2C_CLOCK_HZ);  // belt and braces, in case init() disturbed it
  motor.linkSensor(&sensor);

  driver.voltage_power_supply = SUPPLY_VOLTAGE;
  driver.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  if (!driver.init()) {
    Serial.println(F("[MOTOR] driver init failed"));
    return false;
  }
  motor.linkDriver(&driver);

  motor.controller = MotionControlType::velocity;
  motor.voltage_limit = MOTOR_VOLTAGE_LIMIT;
  motor.velocity_limit = rpmToRadS(RPM_VELOCITY_LIMIT);
  motor.PID_velocity.P = PID_P;
  motor.PID_velocity.I = PID_I;
  motor.PID_velocity.D = PID_D;
  motor.LPF_velocity.Tf = LPF_TF;

  motor.init();
  motor.initFOC();  // the motor twitches during alignment -- expected
  motor.disable();  // ...and then sits idle until asked to spin
  motorEnabled = false;

  /* Sensor health at boot. Report WHICH check failed and dump the full picture
   * -- an earlier version printed only "AS5600 not healthy", which told you
   * something was wrong and nothing about what. */
  Wire.setClock(I2C_CLOCK_HZ);  // sensor.init() may have reset it
  /* Apply the filter BEFORE initFOC() -- alignment measures the sensor, so it
   * should be measuring the sensor we are going to run with. */
  if (!setSensorFilter(AS5600_SLOW_FILTER)) {
    Serial.println(F("[MOTOR] could not set the AS5600 filter -- running at its "
                     "default 16x/2.2 ms, expect poor commutation above ~400 rpm"));
  }

  refreshSensorHealth();
  if (!i2cOk) {
    Serial.println(F("[MOTOR] AS5600: I2C read failed"));
    dumpSensorDiag();
    latchFault("E02",
               "Lost contact with the motor's position sensor. This usually "
               "means a loose sensor connection.");
  } else if (!magnetOk) {
    // MD clear -- the datasheet ties output validity to this bit alone.
    Serial.printf("[MOTOR] AS5600: reachable, but MD clear -- no magnet "
                  "(STATUS=0x%02X)\n",
                  lastStatusReg);
    dumpSensorDiag();
    latchFault("E01",
               "The motor's position sensor cannot see its magnet, so the fan "
               "cannot tell how fast it is turning.");
  } else {
    Serial.printf("[MOTOR] AS5600 OK (STATUS=0x%02X, MD set)\n", lastStatusReg);
    // ML/MH are gain-limit warnings, not validity flags. Report and carry on.
    if (magnetWeak) {
      Serial.println(F("[MOTOR] note: AGC railed at maximum gain (ML) -- magnet "
                       "usable but at the weak end, no headroom left. Press 's' "
                       "for detail."));
    }
    if (magnetStrong) {
      Serial.println(F("[MOTOR] note: AGC floored (MH) -- magnet very close."));
    }
  }

  /* Driver fault state at boot. If nFAULT reads low here, one of two things is
   * true: the driver really has latched a fault (VM was never cycled after an
   * overcurrent trip), or the wire is not connected the way this firmware
   * expects. Both want reporting rather than silently ignoring, but a stale
   * latch is worth one reset attempt first. */
  refreshDriverHealth();
  if (!driverOk) {
    Serial.println(F("[MOTOR] nFAULT asserted at boot -- attempting a reset"));
    resetDriver();
  }
  Serial.printf("[MOTOR] driver nFAULT (D11/GPIO%d) reads %s at boot\n",
                PIN_DRV_FAULT, driverOk ? "OK" : "FAULT");

  // Start at a speed in the middle of the tested range rather than at the
  // bottom of the clamp: 30 rpm is in the friction-dominated regime where the
  // tuning notes say the gains want to be different, which is a poor place for
  // the fan to sit the first time someone presses start. Matches RPM_DEFAULT
  // in the website's config.js.
  rpmTarget = RPM_DEFAULT_TARGET;
  publish(0.0f);

  BaseType_t ok = xTaskCreate(focTask, "foc", FOC_TASK_STACK, nullptr,
                              FOC_TASK_PRIORITY, nullptr);
  if (ok != pdPASS) {
    Serial.println(F("[MOTOR] could not start the FOC task"));
    return false;
  }

  Serial.printf("[MOTOR] ready. clamp %.0f-%.0f rpm, voltage_limit %.1f V\n",
                RPM_MIN_ALLOWED, RPM_MAX_ALLOWED, MOTOR_VOLTAGE_LIMIT);
  return true;
}

Telemetry snapshot() {
  /* Waits for the mutex rather than timing out.
   *
   * That is safe here specifically because the critical section on both sides
   * is a bare struct copy -- no I2C, no flash, no allocation -- so the longest
   * a caller can wait is one memcpy plus whatever pre-empts the holder, and
   * FreeRTOS priority inheritance bounds even that. The alternative, giving up
   * on a timeout and returning an unlocked read, would hand back a TORN struct
   * -- a half-written fault message or a nonsense RPM -- in front of a user.
   * A brief wait is much the lesser evil. */
  Telemetry copy{};
  if (xSemaphoreTake(stateMutex, portMAX_DELAY) == pdTRUE) {
    copy = published;
    xSemaphoreGive(stateMutex);
  }
  return copy;
}

void cmdPower(bool on) {
  Command c{CmdType::Power, on ? 1.0f : 0.0f};
  xQueueSend(commandQueue, &c, 0);
}

void cmdSpeed(float rpm) {
  Command c{CmdType::Speed, clampRpm(rpm)};
  xQueueSend(commandQueue, &c, 0);
}

void cmdStop() { estopFlag.store(true); }

void requestDiagnostics() { diagRequest.store(true); }

void cycleSensorFilter() { filterCycleRequest.store(true); }

void resetGapHighWater() {
  /* Called once boot is complete. Bringing up the WiFi AP and mDNS blocks for
   * tens of milliseconds, and the FOC task is already running by then, so it
   * eats one large gap that would otherwise sit in focMaxGapMs forever and
   * mask any real problem found later. The first bring-up run showed 58.49 ms
   * from exactly this. */
  maxGapMs = 0.0f;
  gapMsAccum = 0.0f;
}

void cmdClearFault() {
  Command c{CmdType::ClearFault, 0.0f};
  xQueueSend(commandQueue, &c, 0);
}

}  // namespace motorctl

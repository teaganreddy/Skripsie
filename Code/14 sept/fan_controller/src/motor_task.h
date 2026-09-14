#pragma once
/*
 * motor_task.h -- the motor half of the firmware, and the only code that is
 * allowed to touch SimpleFOC.
 *
 * Everything here runs on a dedicated FreeRTOS task. Web request handlers run
 * on the AsyncTCP task and must never call into SimpleFOC directly; they post
 * commands through the functions below, all of which are non-blocking and safe
 * to call from any task.
 */

#include <Arduino.h>

enum class FanState : uint8_t { Idle, Starting, Running, Stopping, Fault };

const char *fanStateName(FanState s);

/* A consistent point-in-time copy of everything the web layer needs. Taken
 * under a mutex by the motor task, handed out by value, so a handler never
 * reads a half-updated struct and never holds a lock while writing a
 * response. */
struct Telemetry {
  bool power;
  FanState state;
  float rpmTarget;   // what the user asked for
  float rpmCommand;  // where the ramp currently is -- bring-up visibility

  /* Two speed measurements from the same sensor, for different jobs.
   *
   * rpmActual is displacement over a 250 ms window: low-noise and honest at
   * standstill, so it is what the website shows and what every "has it stopped"
   * test uses.
   *
   * rpmFast is SimpleFOC's LPF-filtered differentiated velocity -- what the PID
   * actually chases. At 12-bit resolution and a ~1.3 kHz loop, one count of
   * encoder dither is ~20 rpm, so near zero this reads +/-20 rpm on a shaft
   * that is provably still. Fine for control, useless for "is it stopped". */
  float rpmActual;
  float rpmFast;

  bool faultLatched;
  bool faultCauseActive;  // is the underlying cause STILL present?

  /* AS5600 AGC gain-limit warnings (STATUS ML/MH). NOT faults -- the angle is
   * still valid -- but they mean the sensor has no gain headroom left in that
   * direction, which is worth surfacing before it becomes a failure. */
  bool magnetWeak;
  bool magnetStrong;
  char faultCode[8];
  char faultMessage[112];
  char clearBlockedReason[112];  // why /api/fault/clear would refuse, if it would

  // --- bring-up instrumentation (see README / the focLoopHz discussion) ---
  uint32_t focLoopHz;   // loopFOC() iterations in the last second
  float focMaxGapMs;    // worst gap between iterations since boot
  float focGapMsRecent; // worst gap in the last second -- recovers, unlike the above
  float rpmRipple;      // peak-to-peak velocity ripple over one revolution
  float voltageQ;       // SimpleFOC's commanded q-axis voltage
  float estCurrentA;    // ROUGH estimate. No current sensing exists. Not protection.
};

namespace motorctl {

// Brings up I2C, the AS5600, the driver and the FOC loop, then starts the
// task. Blocks for the duration of initFOC() -- the motor twitches during
// alignment, which is expected. Returns false if the sensor never appeared.
bool begin();

// Snapshot of current state. Cheap, non-blocking (gives up if the mutex is
// momentarily held rather than waiting on it, returning the previous copy).
Telemetry snapshot();

// Clamp to the firmware's own safe band, ignoring whatever the UI thinks its
// slider range is. Exposed so a handler can report the clamped value back.
float clampRpm(float rpm);

// --- commands. All non-blocking, all safe from the AsyncTCP task. ---

void cmdPower(bool on);
void cmdSpeed(float rpm);

// Emergency stop: skips the ramp entirely. Sets an atomic flag the motor task
// checks inside its inner loop, so it does not queue behind other commands.
void cmdStop();

// Posts a fault-clear request. Check snapshot().faultCauseActive first -- the
// motor task will refuse the clear if the cause is still present, and
// clearBlockedReason says why in words a member of the public can read.
void cmdClearFault();

// Asks the motor task to print a full AS5600 diagnostic dump to serial: bus
// scan, magnet status, AGC, field magnitude, and ten raw angle reads. Bound to
// the 's' key in main.cpp. Runs on the motor task because it shares the I2C bus
// with the control loop.
void requestDiagnostics();

/* Cycles the AS5600 slow filter 16x -> 8x -> 4x -> 2x -> 16x. Bound to 'w'.
 * Lets the commutation-lag hypothesis be A/B tested at a fixed speed without
 * reflashing: watch voltage.q before and after. */
void cycleSensorFilter();

// Clears the worst-gap high-water mark. Called once boot is finished, so the
// unavoidable WiFi/mDNS startup stall does not sit in focMaxGapMs forever.
void resetGapHighWater();

}  // namespace motorctl

/*
 * rail_controller -- linear rail firmware for the hologram fan display.
 * ESP32-C6 driving a NEMA17 through an A4988, carrying the fan unit along an
 * aluminium extrusion between two limit switches.
 *
 * Replaces linear_rail_test, the bring-up sketch that proved the rail runs.
 * What that sketch did not have, and this does:
 *
 *   - stepping on a HARDWARE TIMER, so ESP-NOW on this single-core chip
 *     cannot stutter the pulse train              (stepper.cpp)
 *   - a trapezoidal ramp instead of instant start/reverse -- the load is a
 *     whole fan unit now, not a bare carriage      (rail.cpp)
 *   - homing on command, a counted position, soft limits inside the switches,
 *     and lost-step detection off the switches     (rail.cpp)
 *   - the ESP-NOW link to the stator               (rail_link.cpp)
 *
 * And what it deliberately does NOT do: move on its own. Boot is UNHOMED and
 * motionless whatever the switches read, and every motion command needs the
 * stator's motion-permitted flag -- or a key pressed on this board's serial,
 * which is as good a "someone is standing here and the fan is stopped" as any.
 *
 * Layout mirrors the other two projects: config.h holds every constant,
 * link_protocol.h is the shared wire format and MUST stay byte-identical with
 * the copies in fan_controller/ and rotor_controller/.
 */

#include <Arduino.h>

#include "config.h"
#include "link_protocol.h"
#include "rail.h"
#include "rail_link.h"

namespace {

uint32_t lastControlUs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastBeatMs = 0;
bool beatLevel = false;
bool bannerShown = false;

void printBanner() {
  Serial.println(F("\n=== hologram fan -- linear rail firmware ==="));
  Serial.printf("unit      : %d\n", UNIT_ID);
  Serial.printf("drive     : NEMA17 / A4988, 1/%d step, Vref %.2f V = %.2f A/phase\n",
                MICROSTEP_DIVISOR, A4988_VREF_V, A4988_PHASE_CURRENT_A);
  Serial.printf("mechanics : GT2 %dT, %.0f mm/rev, %.0f steps/mm\n", PULLEY_TEETH,
                MM_PER_REV, STEPS_PER_MM);
  Serial.printf("travel    : %.1f mm %s, soft limits %.1f..%.1f mm\n",
                stepsToMm(rail::travelSteps()),
                rail::travelMeasured() ? "(measured)" : "(nominal -- run 'm' to measure)",
                stepsToMm(rail::softMin()), stepsToMm(rail::softMax()));
  Serial.printf("motion    : cruise %.0f mm/s, max %.0f mm/s, accel %.0f mm/s^2\n",
                CRUISE_MM_S, MAX_MM_S, ACCEL_MM_S2);
  Serial.printf("state     : %s%s%s\n", rail::stateName(),
                rail::homed() ? ", homed" : ", NOT homed",
                rail::motionPermitted() ? ", motion permitted" : ", motion NOT permitted");
  if (rail::fault() != RAIL_FAULT_NONE) Serial.printf("fault     : %s\n", rail::faultName());
  Serial.printf("link      : %s, rx %lu tx %lu fail %lu\n",
                raillink::paired() ? "PAIRED" : "not paired",
                (unsigned long)raillink::received(), (unsigned long)raillink::sent(),
                (unsigned long)raillink::sendFailures());
  Serial.println();
}

void printHelp() {
  Serial.println(F("keys:  h=home  m=measure travel (needs homed)  i=info  ?=help\n"
                   "       , .=jog -10/+10 mm   < >=go to soft min/max   s=sweep full range\n"
                   "       x=stop (ramped)  e=HALT now  c=clear fault  d=toggle driver\n"
                   "       p=toggle motion permission (bench override of the stator's flag)\n"
                   "  Any motion key also grants motion permission -- you are standing\n"
                   "  at the board, so you can see whether the fan is stopped."));
}

void handleKey(int c) {
  auto permit = []() { rail::setMotionPermitted(true); };
  switch (c) {
    case 'h': case 'H': permit(); rail::home(); break;
    case 'm': case 'M': permit(); rail::measure(); break;
    case ',': permit(); rail::jogSteps(-mmToSteps(10.0f), 0); break;
    case '.': permit(); rail::jogSteps(+mmToSteps(10.0f), 0); break;
    case '<': permit(); rail::gotoSteps(rail::softMin(), 0); break;
    case '>': permit(); rail::gotoSteps(rail::softMax(), 0); break;
    case 's': case 'S':
      permit();
      rail::sweep(rail::softMin(), rail::softMax(), SWEEP_DWELL_MS_DEFAULT, 0);
      break;
    case 'x': case 'X': rail::stop(); Serial.println(F("[KEY] stop")); break;
    case 'e': case 'E': rail::haltNow(); Serial.println(F("[KEY] HALT")); break;
    case 'c': case 'C': rail::clearFault(); break;
    case 'd': case 'D':
      rail::enableDriver(!rail::driverEnabled());
      Serial.printf("[KEY] driver %s\n", rail::driverEnabled() ? "ENABLED" : "disabled");
      break;
    case 'p': case 'P':
      rail::setMotionPermitted(!rail::motionPermitted());
      Serial.printf("[KEY] motion %s\n", rail::motionPermitted() ? "permitted" : "NOT permitted");
      break;
    case 'i': case 'I': printBanner(); break;
    case '?': printHelp(); break;
    default: break;
  }
}

/* Heartbeat encodes state so the board says something with no serial attached:
 *   steady slow blink   idle / unhomed
 *   solid on            moving
 *   fast blink          fault */
void heartbeat() {
  const uint32_t now = millis();
  if (rail::rateStepsPerSec() > 0) {
    digitalWrite(PIN_HEARTBEAT, HIGH);
    return;
  }
  const uint32_t period = (rail::state() == RAIL_STATE_FAULT) ? HEARTBEAT_FAULT_MS
                                                              : HEARTBEAT_IDLE_MS;
  if (now - lastBeatMs >= period) {
    lastBeatMs = now;
    beatLevel = !beatLevel;
    digitalWrite(PIN_HEARTBEAT, beatLevel);
  }
}

}  // namespace

void setup() {
  pinMode(PIN_HEARTBEAT, OUTPUT);
  digitalWrite(PIN_HEARTBEAT, LOW);

  Serial.begin(115200);
  /* Native USB CDC: Serial.begin() returns before the host has re-enumerated,
   * so an immediate print is lost. Wait for DTR, capped so the board still
   * comes up with no monitor attached. */
  const uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  delay(100);

  rail::begin();      // also brings up stepper (pins, timer, driver enable)
  raillink::begin();

  lastControlUs = micros();
  printHelp();
}

void loop() {
  // Control tick: ramp shaping, sequences, switches. ~1 kHz; a late tick is
  // harmless because the ISR stops at the target on its own.
  const uint32_t nowUs = micros();
  if (nowUs - lastControlUs >= CONTROL_TICK_US) {
    lastControlUs = nowUs;
    rail::tick();
  }

  raillink::tick();
  heartbeat();

  while (Serial.available()) handleKey(Serial.read());

  const uint32_t nowMs = millis();
  if (!bannerShown && nowMs > 2000) {
    bannerShown = true;
    printBanner();
  }

  if (nowMs - lastStatusMs >= 1000) {
    lastStatusMs = nowMs;
    Serial.printf("%-9s pos %7.1f mm  tgt %7.1f  %5lu st/s  lim H%d I%d  %s%s  link %s\n",
                  rail::stateName(), stepsToMm(rail::position()),
                  stepsToMm(rail::target()), (unsigned long)rail::rateStepsPerSec(),
                  rail::limitHome() ? 1 : 0, rail::limitIdle() ? 1 : 0,
                  rail::homed() ? "homed" : "unhomed",
                  rail::motionPermitted() ? "" : " NOPERMIT",
                  raillink::paired() ? "up" : "down");
  }
}

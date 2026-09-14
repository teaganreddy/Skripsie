/*
 * fan_controller -- stator-side firmware for a 3D hologram-like RGB LED fan
 * display. Final-year Mechatronics project, Stellenbosch University (MNS5).
 *
 * DFRobot FireBeetle 2 ESP32-C6 + SimpleFOCmini (DRV8313) + DFRobot FIT1034
 * 2804 BLDC outrunner with a built-in AS5600 encoder.
 *
 * This board does three jobs at once: closed-loop motor control, a WiFi
 * access point, and the web server the browser talks to. In stage 2 it also
 * relays content to the rotor (ESP32-S3) over ESP-NOW; today that is stubbed
 * (see rotor_stub.h).
 *
 * ===========================================================================
 *  THE SINGLE-CORE PROBLEM, AND HOW THIS FIRMWARE IS ARRANGED AROUND IT
 * ===========================================================================
 *
 * The ESP32-C6 has ONE 160 MHz RISC-V core. There is no second core to pin
 * the FOC loop to, so WiFi, the async web server and motor.loopFOC() all
 * share it. This is the single biggest risk in the design, and the layout
 * below is deliberate rather than incidental.
 *
 * Task priorities (higher number wins):
 *
 *    23  WiFi driver          ESP-IDF. Starve this and the AP drops or the
 *                             watchdog fires. Never pre-empt it.
 *    22  esp_timer
 *    18  lwIP / TCP-IP
 *    15  FOC task             <-- ours. Above the web server, below the radio.
 *    12  telemetry task       <-- ours. 2 Hz status + rotor-progress events.
 *    10  AsyncTCP             pinned in platformio.ini rather than left to
 *                             whatever the library defaults to this release.
 *     1  Arduino loopTask     serial diagnostics only.
 *
 * The FOC task does NOT busy-spin. It runs a ~2 ms burst of loopFOC()/move()
 * and then sleeps one FreeRTOS tick (1 ms, since CONFIG_FREERTOS_HZ is 1000).
 * That sleep is what lets AsyncTCP run at all, and it bounds the worst-case
 * gap between AS5600 reads at roughly 1 ms plus whatever the radio pre-empts.
 *
 * Why that bound matters: at 700 rpm a revolution takes 85.7 ms, so if the
 * gap between sensor reads ever exceeded ~43 ms, SimpleFOC's angle unwrapping
 * could no longer tell how far the shaft had turned and the velocity estimate
 * would become garbage. The design target is under 5 ms -- roughly 8x margin
 * on the failure point, and the actual figure is reported live as focMaxGapMs
 * in /api/status so it is a number to watch rather than a hope.
 *
 * The rule that keeps this true: WEB HANDLERS NEVER TOUCH THE MOTOR. They run
 * on the AsyncTCP task and post commands through motorctl, which queues them
 * for the FOC task. Nothing in web.cpp includes SimpleFOC.
 */

#include <Arduino.h>

#include "config.h"
#include "content.h"
#include "motor_task.h"
#include "rotor_stub.h"
#include "web.h"

void setup() {
  // Straight on, before anything else, so the board shows life the instant it
  // boots regardless of what happens after.
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n=== hologram fan -- stator firmware ==="));

  // Filesystem first: the web server serves out of it, and a failure here is
  // worth reporting before the motor starts moving anything.
  if (!content::begin()) {
    Serial.println(F("[BOOT] filesystem unavailable -- run 'pio run -t uploadfs'"));
  }

  /* rotor::begin() is NOT called here any more. ESP-NOW rides on the WiFi AP
   * interface, so it has to start after WiFi.softAP() -- it now runs from
   * web::begin(). Calling it here initialised ESP-NOW against no interface at
   * all, which fails quietly. */

  // Brings up the driver and runs initFOC(). The motor twitches during
  // electrical alignment; that is expected and is not a fault.
  if (!motorctl::begin()) {
    Serial.println(F("[BOOT] motor did not initialise"));
  }

  if (!web::begin()) {
    Serial.println(F("[BOOT] web server did not start"));
  }
  web::startTelemetryTask();

  /* Bringing up the AP and mDNS blocks for tens of milliseconds while the FOC
   * task is already running, so it banks one large inter-iteration gap. Clear
   * it now: from here on, focMaxGapMs measures the running system, which is
   * the thing actually under question. */
  motorctl::resetGapHighWater();

  Serial.println(F("[BOOT] ready -- IDLE. The motor will not move until it is"
                   " told to start."));
  Serial.println(F("[BOOT] press start on the web page, or press 'g' + Enter"
                   " here. '?' lists the bench keys."));
}

void loop() {
  /* Diagnostics only. Everything that matters runs on the tasks above, so
   * this sits at priority 1 and gets whatever is left over.
   *
   * The print is skipped entirely when the USB TX buffer is not ready, so a
   * host that is not actively reading (mid-reconnect, say) can never stall
   * this task -- the same guard motor_test uses. */
  /* Bench controls, for running the BRINGUP.md sweep without a browser --
   * stepping through five speeds while reading this serial output is awkward
   * with a slider in another window.
   *
   * These post the SAME commands through the SAME queue as the HTTP handlers,
   * so the firmware clamp, the ramp and every fault check apply identically.
   * There is no privileged path here. platformio.ini already sets monitor_echo
   * and send_on_enter. */
  while (Serial.available()) {
    const int c = Serial.read();
    switch (c) {
      case 's': case 'S': motorctl::requestDiagnostics(); break;
      case 'w': case 'W': motorctl::cycleSensorFilter(); break;
      case 'g': case 'G':
        Serial.println(F("[KEY] start"));
        motorctl::cmdPower(true);
        break;
      case 'x': case 'X':
        Serial.println(F("[KEY] stop (ramps down)"));
        motorctl::cmdPower(false);
        break;
      case 'e': case 'E':
        Serial.println(F("[KEY] EMERGENCY STOP -- coasts, does not brake"));
        motorctl::cmdStop();
        break;
      case 'c': case 'C':
        Serial.println(F("[KEY] clear fault"));
        motorctl::cmdClearFault();
        break;
      case '+': case '=': {
        const float rpm = motorctl::clampRpm(motorctl::snapshot().rpmTarget + 10);
        Serial.printf("[KEY] target %d rpm\n", (int)lroundf(rpm));
        motorctl::cmdSpeed(rpm);
        break;
      }
      case '-': case '_': {
        const float rpm = motorctl::clampRpm(motorctl::snapshot().rpmTarget - 10);
        Serial.printf("[KEY] target %d rpm\n", (int)lroundf(rpm));
        motorctl::cmdSpeed(rpm);
        break;
      }
      case '?': case 'h': case 'H':
        Serial.println(F("keys: g=start  x=stop  e=e-stop  +/-=speed  "
                         "c=clear fault  s=sensor diagnostics  w=cycle AS5600 filter"));
        break;
      default: break;  // ignore newlines and anything else
    }
  }

  static uint32_t lastPrint = 0;
  if (millis() - lastPrint >= 1000) {
    lastPrint = millis();
    if (Serial.availableForWrite() >= 128) {
      const Telemetry t = motorctl::snapshot();
      Serial.printf(
          "%-8s target %3d  actual %4d rpm (fast %4d) | Uq %4.2f V | "
          "focLoopHz %5u  gap now %5.2f ms  worst %5.2f ms | ripple %4.1f rpm "
          "| wifi %d client(s)%s\n",
          fanStateName(t.state), (int)lroundf(t.rpmTarget),
          (int)lroundf(t.rpmActual), (int)lroundf(t.rpmFast), t.voltageQ,
          t.focLoopHz, t.focGapMsRecent, t.focMaxGapMs, t.rpmRipple,
          web::stationCount(), t.magnetWeak ? "  [magnet weak]" : "");
    }
  }
  delay(50);
}

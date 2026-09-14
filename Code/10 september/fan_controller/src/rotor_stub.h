#pragma once
/* ===========================================================================
 * The rotor link, as seen from the stator.
 *
 * STAGE 2b IS DONE: telemetry here is REAL ESP-NOW from the ESP32-S3, not the
 * simulation this file used to contain. The function names are unchanged so
 * web.cpp did not need touching.
 *
 * STILL SIMULATED: the content-push progress ramp (startPush /
 * takeProgressEvent). Chunked transfer is stage 2c, and it is marked in
 * rotor_stub.cpp.
 *
 * HONESTLY ABSENT: the battery. No sense divider is wired on the rotor, so
 * batteryPct() and batteryMv() return -1 and the status payload emits null --
 * which API.md explicitly allows. Better an admitted gap than a convincing
 * invention on a public demo.
 * ======================================================================== */

#include <Arduino.h>

namespace rotor {

// Call AFTER WiFi.softAP() -- ESP-NOW rides on the AP interface and needs it up.
void begin();

// Pumps the beacon, the outbound command, and the simulated push. From loop().
void tick(bool fanSpinning);

bool online();

// -1 when unknown (no divider wired); web.cpp turns that into JSON null.
int batteryPct();
int batteryMv();

int rssi();

/* The rotor's own display diagnostics, relayed up. These matter because once
 * the arm is spinning there is no USB on the rotor -- this is the only way to
 * see whether the POV sync is actually holding. */
int rotorRpm();         // measured by the rotor's Hall sensor
int columnsPerSec();    // actual paint rate
int overruns();         // interpolated sector ran past a revolution
int rejectedPulses();   // Hall pulses failing the sanity window
bool syncLocked();
bool hasContent();      // the rotor is holding an image, not the fallback

// --- content push (still simulated, stage 2c) ---
void startPush(const char *id, uint32_t bytes);
bool pushActive();
bool takeProgressEvent(char *idOut, size_t idLen, float &progress, bool &done);

}  // namespace rotor

/* Provided by web.cpp. The command sent to the rotor carries the brightness
 * the website set and the id of the selected content. */
int web_brightness();
const char *selectedContentId();

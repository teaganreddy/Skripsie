#pragma once
/* ===========================================================================
 * The linear rail, as seen from the stator.
 *
 * A third ESP-NOW peer alongside the rotor. The rail C6 hears our beacon and
 * answers with RailTelemetryMsg at 5 Hz; we send it a RailCommandMsg on every
 * beacon (idempotent, seq-numbered, so a lost packet self-heals and a resent
 * HOME does not re-home).
 *
 * TWO MODES, held here so they survive a page reload:
 *
 *   STATIC  "position, then display". The rail moves only while the fan is
 *           stopped, and the fan may not start while the rail is moving. On
 *           entering this mode the fan is sent to the middle of the rail.
 *
 *   SWEEP   the rail oscillates between two bounds while the fan runs, for
 *           content like a car driving side to side. Motion is permitted
 *           while spinning; this is phase 2 and is what the whole interlock
 *           exists to make explicit rather than accidental.
 *
 * E-STOP always halts the rail and drops back to STATIC, so after an
 * emergency the rail cannot move again until the arm has actually stopped.
 *
 * The motion-permitted flag in every command is the belt to this file's
 * braces: the rail refuses to move without it, so even a bug in the handlers
 * here cannot drive a carriage under a spinning arm.
 * ======================================================================== */

#include <Arduino.h>

#include <esp_now.h>

#include "link_protocol.h"

namespace rail {

enum class Mode : uint8_t { Static, Sweep };

/* Called from rotor_stub.cpp's receive callback for MSG_RAIL_TELEMETRY. There
 * is one ESP-NOW receive callback per chip, and the rotor link owns it. */
void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len);

/* From the telemetry task, after rotor::tick(). Sends the command on the
 * same 1 s cadence as the beacon. `fanStopped` is the interlock input. */
void tick(bool fanStopped, bool fanFaulted);

bool online();

// --- commands from the web layer. Each returns nullptr on success or a
//     user-facing reason it was refused. ---
const char *setMode(Mode m, bool fanStopped);
const char *home(bool fanStopped);
const char *gotoMm(float mm, bool fanStopped);
const char *startSweep(float minMm, float maxMm, float speedMms, uint32_t dwellMs);
const char *stopSweep();
void stop();           // ramped
void emergencyStop();  // halt now, drop to STATIC
const char *clearFault();

/* True while the rail is moving in STATIC mode: the fan must not start. In
 * SWEEP mode the fan may start regardless, that is the point of the mode. */
bool blocksFanStart();

// --- state for /api/status ---
Mode mode();
const char *modeName();
uint8_t state();
const char *stateName();
uint8_t fault();
const char *faultName();
bool homed();
bool moving();
bool sweeping();
bool limitHome();
bool limitIdle();
float positionMm();
float targetMm();
float travelMm();
bool travelMeasured();

// The sweep the user last asked for, echoed so the UI can restore its sliders.
float sweepMinMm();
float sweepMaxMm();
float sweepSpeedMms();
uint32_t sweepDwellMs();

}  // namespace rail

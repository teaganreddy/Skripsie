#pragma once
/* ===========================================================================
 * The other fan unit, as seen from this stator.
 *
 * One file, two roles, chosen by UNIT_ID at compile time:
 *
 *   MASTER (unit 1)  tracks the peer stator: records its MAC from the first
 *                    telemetry packet, sends UnitCommandMsg at 5 Hz, and
 *                    exposes the peer's state for /api/status.
 *
 *   PEER   (unit 2)  brings up WiFi in station mode (no AP) and ESP-NOW,
 *                    pairs to the master's beacon, sends UnitTelemetryMsg at
 *                    5 Hz, and applies the master's commands to its own motor
 *                    and rotor exactly as the web handlers would have.
 *
 * The peer's fail-safe is the rail's: nothing from the master for a few
 * seconds while spinning means ramp down. A fan on its own with no one able
 * to stop it is not a state to leave it in.
 * ======================================================================== */

#include <Arduino.h>

#include <esp_now.h>

#include "link_protocol.h"

namespace unitlink {

constexpr bool isMaster() { return UNIT_ID == 1; }

/* The other stator's MAC (master: the peer's; peer: the master's), and
 * whether it is known yet. For file_sync.cpp, which talks to the same board. */
const uint8_t *linkMac();
bool linkKnown();

/* PEER: wake the peer task now (file_sync's receive mailbox). */
void wakePeerTask();

/* Both roles: called from rotor_stub.cpp's receive callback for
 * MSG_UNIT_TELEMETRY (master) and MSG_UNIT_COMMAND / MSG_BEACON (peer). */
void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len);

/* Both roles: from the telemetry task, alongside rotor::tick(). */
void tick();

// ---- PEER only ----------------------------------------------------------

/* WiFi STA + channel + esp_now_init, in place of web::begin(). Must run
 * BEFORE rotor::begin(), which registers the callbacks. */
bool beginPeer();
void startPeerTask();

// ---- MASTER only ---------------------------------------------------------

/* Seeds the outbound command with this board's brightness and boot setpoint.
 * Call after web::begin() has loaded the persisted brightness. */
void beginMaster();

/* The speed the peer is currently being told, i.e. what handleSpeed() last
 * set for the right fan. Distinct from this board's own target. */
int peerRpmCommanded();

/* The interlock's view of the right fan: true if it is online and stopped,
 * or offline but certainly not turning (last seen stopped with no start
 * pending, or silent for longer than its fail-safe ramp-down takes). An
 * offline peer used to count as stopped outright, which let the rail move
 * and the display mode change within ~2 s of losing a fan that might still
 * be at 700 rpm. */
bool peerStoppedOrGone();

// ---- MASTER only: the peer's state, for the web layer -------------------

bool peerOnline();
uint8_t peerState();        // FanState as uint8
bool peerPower();
bool peerFault();
bool peerFaultCauseActive();
const char *peerFaultCode();
const char *peerFaultMessage();
int peerRpmTarget();
int peerRpmActual();
int peerBrightness();
int peerRotorRpm();
int peerRotorRssi();
int peerRotorHallLevel();  // 1 high, 0 low, -1 unknown
bool peerRotorOnline();
bool peerRotorHasContent();
bool peerRotorSyncLocked();
bool peerPushing();  // the peer is transferring content to its rotor right now
const char *peerContentId();
float peerVoltageQ();
int peerFocLoopHz();
float peerFocMaxGapMs();

// ---- MASTER only: commands to the peer ----------------------------------

void peerPower(bool on);
void peerEstop();
void peerClearFault();
void peerSetRpm(float rpm);
void peerSetBrightness(int level);
void peerSelectContent(const char *id);

/* Scene state for the peer's rotor: mirror flag, loop pulse counter and the
 * frame a pulse restarts at. Sent at once; the peer hands all three to its
 * rotor on its next tick. */
void peerSetScene(bool mirror, uint8_t loopSeq, uint8_t startFrame);

/* Unit 2's angle calibration, relayed to its rotor via its stator. */
void peerSetCalibration(int offsetSectors, bool reverse);

}  // namespace unitlink

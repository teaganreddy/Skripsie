#include "rail_link.h"

#include <esp_now.h>

#include "config.h"

namespace {

const uint32_t COMMAND_INTERVAL_MS = 1000;  // same cadence as the beacon
const uint32_t LINK_TIMEOUT_MS = 2000;      // 10 missed packets at 5 Hz

uint8_t railMac[6] = {0};
bool haveRail = false;
volatile uint32_t lastRxMs = 0;
RailTelemetryMsg tele{};
volatile bool haveTele = false;

rail::Mode currentMode = rail::Mode::Static;

/* The command the rail is currently being told. Resent every second; the
 * rail acts on `op` only when `seq` changes, so a resend is just a refresh
 * of the motion-permitted flag. */
RailCommandMsg cmd{};
uint32_t lastCmdMs = 0;
bool lastPermitted = false;

// The last sweep the user configured, echoed back for the UI.
float swMin = 0, swMax = 0, swSpeed = 0;
uint32_t swDwell = 0;
bool sweepRequested = false;

/* Entering Position mode means "send the fan to the middle", but that cannot
 * be a GOTO fired straight after the STOP: two packets a few ms apart land
 * in the rail's single command slot and the second overwrites the first, so
 * the STOP is lost and the GOTO is refused ("already moving"). The rail would
 * have carried on sweeping. Instead this asks tick() to issue the GOTO once
 * telemetry actually shows the rail idle and homed with the fan stopped. */
bool centreWhenIdle = false;

inline int32_t mmToSteps(float mm) { return (int32_t)lroundf(mm * RAIL_STEPS_PER_MM); }
inline float stepsToMm(int32_t s) { return s / RAIL_STEPS_PER_MM; }

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = LINK_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_AP;
  const esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK) Serial.printf("[RAIL] add_peer failed: %s\n", esp_err_to_name(err));
}

void send() {
  if (!haveRail) return;
  cmd.type = MSG_RAIL_COMMAND;
  cmd.version = PROTOCOL_VERSION;
  esp_now_send(railMac, (const uint8_t *)&cmd, sizeof(cmd));
  lastCmdMs = millis();
}

/* Issue a new one-shot op: bump seq so the rail executes it once. */
void issue(uint8_t op, int32_t target = 0, uint32_t speed = 0, int32_t lo = 0,
           int32_t hi = 0, uint32_t dwell = 0) {
  cmd.seq++;
  cmd.op = op;
  cmd.targetSteps = target;
  cmd.speedStepsPerSec = speed;
  cmd.sweepMinSteps = lo;
  cmd.sweepMaxSteps = hi;
  cmd.sweepDwellMs = dwell;
  send();
}

float travelMmNow() {
  const uint32_t t = haveTele ? tele.travelSteps : 0;
  return t ? stepsToMm((int32_t)t) : RAIL_TRAVEL_MM_NOMINAL;
}

float softMinMm() { return RAIL_SOFT_MARGIN_MM; }
float softMaxMm() { return travelMmNow() - RAIL_SOFT_MARGIN_MM; }

const char *stateText(uint8_t s) {
  switch (s) {
    case RAIL_STATE_UNHOMED: return "unhomed";
    case RAIL_STATE_HOMING: return "homing";
    case RAIL_STATE_IDLE: return "idle";
    case RAIL_STATE_MOVING: return "moving";
    case RAIL_STATE_SWEEPING: return "sweeping";
    case RAIL_STATE_MEASURING: return "measuring";
    case RAIL_STATE_FAULT: return "fault";
    default: return "unknown";
  }
}

const char *faultText(uint8_t f) {
  switch (f) {
    case RAIL_FAULT_NONE: return "";
    case RAIL_FAULT_BOTH_LIMITS: return "Both end switches read pressed. Check the rail wiring.";
    case RAIL_FAULT_LOST_STEPS: return "The rail hit an end switch unexpectedly and has lost its position. Home it again.";
    case RAIL_FAULT_HOME_NOT_FOUND: return "The rail could not find its home switch.";
    case RAIL_FAULT_IDLE_NOT_FOUND: return "The rail could not find its far end switch.";
    case RAIL_FAULT_LINK_LOST: return "The rail lost contact with the fan and stopped.";
    default: return "Rail fault.";
  }
}

}  // namespace

namespace rail {

void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < (int)sizeof(RailTelemetryMsg)) return;
  memcpy(&tele, data, sizeof(tele));
  haveTele = true;
  lastRxMs = millis();

  if (!haveRail || memcmp(railMac, info->src_addr, 6) != 0) {
    memcpy(railMac, info->src_addr, 6);
    addPeer(railMac);
    haveRail = true;
    Serial.printf("[RAIL] linked to %02X:%02X:%02X:%02X:%02X:%02X\n", railMac[0],
                  railMac[1], railMac[2], railMac[3], railMac[4], railMac[5]);
    send();  // tell it the current mode/permission straight away
  }
}

bool online() {
  return haveRail && lastRxMs != 0 && (millis() - lastRxMs) < LINK_TIMEOUT_MS;
}

void tick(bool fanStopped, bool fanFaulted) {
  /* THE interlock. Static: the fan must be stopped. Sweep: the fan may run,
   * that is the mode's purpose. A latched fan fault withdraws permission in
   * either mode -- the arm may be coasting. */
  const bool permitted = !fanFaulted &&
                         (currentMode == Mode::Sweep || fanStopped);
  cmd.flags = permitted ? RAIL_FLAG_MOTION_PERMITTED : 0;

  static bool wasOnline = false;
  const bool now = online();
  if (now != wasOnline) {
    Serial.printf("[RAIL] link %s\n", now ? "UP" : "DOWN");
    wasOnline = now;
  }

  /* A permission change goes out at once -- withdrawing it is what halts a
   * carriage under an arm that has just been started, and a second's delay
   * there is a second too long. */
  if (permitted != lastPermitted) {
    lastPermitted = permitted;
    send();
    return;
  }

  /* Once the rail reports it has acted on the current one-shot, stop
   * resending the op itself -- the resend is only there to refresh the flags.
   * A rail that re-pairs later then sees NONE, not a stale HOME. */
  if (haveTele && cmd.op != RAIL_OP_NONE && tele.lastSeq == cmd.seq) {
    cmd.op = RAIL_OP_NONE;
  }

  /* Deferred centring for Position mode: acts only on real feedback. */
  if (centreWhenIdle && now && haveTele && currentMode == Mode::Static &&
      tele.state == RAIL_STATE_IDLE && (tele.flags & RAIL_TFLAG_HOMED) &&
      !(tele.flags & RAIL_TFLAG_MOVING) && permitted) {
    centreWhenIdle = false;
    const int32_t mid = mmToSteps(travelMmNow() / 2.0f);
    if (labs(mid - tele.positionSteps) > mmToSteps(2.0f)) {
      issue(RAIL_OP_GOTO, mid, 0);
      Serial.println(F("[RAIL] position mode: centring"));
    }
    return;
  }

  if (millis() - lastCmdMs >= COMMAND_INTERVAL_MS) send();
}

// --- commands -------------------------------------------------------------

const char *setMode(Mode m, bool fanStopped) {
  if (!online()) return "The rail is not connected.";
  if (m == currentMode) return nullptr;
  currentMode = m;
  sweepRequested = false;

  (void)fanStopped;
  if (m == Mode::Static) {
    // Leaving sweep: stop the carriage. tick() brings it to the middle once
    // the rail reports idle and the fan is stopped (see centreWhenIdle).
    issue(RAIL_OP_STOP);
    centreWhenIdle = true;
  } else {
    // Entering sweep: nothing moves until the user presses Start.
    centreWhenIdle = false;
    issue(RAIL_OP_STOP);
  }
  Serial.printf("[RAIL] mode -> %s\n", modeName());
  return nullptr;
}

const char *home(bool fanStopped) {
  if (!online()) return "The rail is not connected.";
  if (!fanStopped) return "Stop the fan before homing the rail.";
  if (tele.state == RAIL_STATE_FAULT) return "Clear the rail fault first.";
  issue(RAIL_OP_HOME);
  if (currentMode == Mode::Static) centreWhenIdle = true;  // park in the middle after
  return nullptr;
}

const char *gotoMm(float mm, bool fanStopped) {
  if (!online()) return "The rail is not connected.";
  if (currentMode != Mode::Static) return "Switch the rail to Position mode first.";
  if (!fanStopped) return "Stop the fan before moving the rail.";
  if (!(tele.flags & RAIL_TFLAG_HOMED)) return "Home the rail first.";
  if (tele.state == RAIL_STATE_FAULT) return "Clear the rail fault first.";
  if (mm < softMinMm()) mm = softMinMm();
  if (mm > softMaxMm()) mm = softMaxMm();
  centreWhenIdle = false;  // the user chose somewhere else
  issue(RAIL_OP_GOTO, mmToSteps(mm), 0);
  return nullptr;
}

const char *startSweep(float minMm, float maxMm, float speedMms, uint32_t dwellMs) {
  if (!online()) return "The rail is not connected.";
  if (currentMode != Mode::Sweep) return "Switch the rail to Sweep mode first.";
  if (!(tele.flags & RAIL_TFLAG_HOMED)) return "Home the rail first.";
  if (tele.state == RAIL_STATE_FAULT) return "Clear the rail fault first.";
  if (minMm > maxMm) { const float t = minMm; minMm = maxMm; maxMm = t; }
  if (minMm < softMinMm()) minMm = softMinMm();
  if (maxMm > softMaxMm()) maxMm = softMaxMm();
  if (maxMm - minMm < 10.0f) return "The sweep range is too small.";
  if (speedMms <= 0) speedMms = RAIL_CRUISE_MM_S;
  if (speedMms > RAIL_MAX_MM_S) speedMms = RAIL_MAX_MM_S;
  swMin = minMm; swMax = maxMm; swSpeed = speedMms; swDwell = dwellMs;
  sweepRequested = true;
  issue(RAIL_OP_SWEEP, 0, (uint32_t)lroundf(speedMms * RAIL_STEPS_PER_MM),
        mmToSteps(minMm), mmToSteps(maxMm), dwellMs);
  return nullptr;
}

const char *stopSweep() {
  if (!online()) return "The rail is not connected.";
  sweepRequested = false;
  issue(RAIL_OP_STOP);
  return nullptr;
}

void stop() {
  sweepRequested = false;
  centreWhenIdle = false;
  issue(RAIL_OP_STOP);
}

void emergencyStop() {
  /* Halt, and drop to STATIC so permission now depends on the arm having
   * actually stopped. The command goes out immediately with permission
   * withdrawn; the rail halts on that alone even before it sees the op. */
  currentMode = Mode::Static;
  sweepRequested = false;
  centreWhenIdle = false;
  cmd.flags = 0;
  lastPermitted = false;
  issue(RAIL_OP_STOP);
}

const char *clearFault() {
  if (!online()) return "The rail is not connected.";
  issue(RAIL_OP_CLEAR);
  return nullptr;
}

bool blocksFanStart() {
  if (!online() || currentMode != Mode::Static) return false;
  return (tele.flags & RAIL_TFLAG_MOVING) != 0 || tele.state == RAIL_STATE_HOMING ||
         tele.state == RAIL_STATE_MEASURING;
}

// --- state ----------------------------------------------------------------

Mode mode() { return currentMode; }
const char *modeName() { return currentMode == Mode::Static ? "static" : "sweep"; }
uint8_t state() { return online() ? tele.state : RAIL_STATE_UNHOMED; }
const char *stateName() { return online() ? stateText(tele.state) : "offline"; }
uint8_t fault() { return online() ? tele.fault : RAIL_FAULT_NONE; }
const char *faultName() { return faultText(fault()); }
bool homed() { return online() && (tele.flags & RAIL_TFLAG_HOMED); }
bool moving() { return online() && (tele.flags & RAIL_TFLAG_MOVING); }
bool sweeping() { return online() && tele.state == RAIL_STATE_SWEEPING; }
bool limitHome() { return online() && (tele.flags & RAIL_TFLAG_LIMIT_HOME); }
bool limitIdle() { return online() && (tele.flags & RAIL_TFLAG_LIMIT_IDLE); }
float positionMm() { return online() ? stepsToMm(tele.positionSteps) : 0; }
float targetMm() { return online() ? stepsToMm(tele.targetSteps) : 0; }
float travelMm() { return travelMmNow(); }
bool travelMeasured() { return online() && (tele.flags & RAIL_TFLAG_TRAVEL_MEASURED); }
float sweepMinMm() { return swMin; }
float sweepMaxMm() { return swMax; }
float sweepSpeedMms() { return swSpeed; }
uint32_t sweepDwellMs() { return swDwell; }

}  // namespace rail

#include "rail_link.h"

#include <esp_now.h>

#include "config.h"
#include "netlog.h"

namespace {

/* 5 Hz, matching the rail's telemetry. It was 1 Hz, which with the 1 Hz
 * beacon gave the rail only two packets a second to keep its link alive --
 * six losses in a row on a channel shared with rotor telemetry, content and
 * a phone's SSE stream, and the rail's fail-safe halted a perfectly good
 * sweep. At 5 Hz the same 5 s window is thirty. */
const uint32_t COMMAND_INTERVAL_MS = 200;
const uint32_t LINK_TIMEOUT_MS = 2000;      // 10 missed packets at 5 Hz

uint8_t railMac[6] = {0};
bool haveRail = false;
volatile uint32_t lastRxMs = 0;

/* Milliseconds since `stamp`, never negative. The caller passes the stamp
 * (a snapshot of a value the ESP-NOW callback writes) so it is read BEFORE
 * millis(). `millis() - lastRxMs` read the clock first and the stamp second:
 * a packet landing between the two, just after the ms tick, made the stamp
 * newer than the clock, the unsigned subtraction wrapped to ~4e9 ms, and the
 * link looked silent for 49 days. On the rail that latched fault 5 (link
 * lost) mid-sweep every few minutes with the link perfectly healthy. */
inline uint32_t msSince(uint32_t stamp) {
  const uint32_t now = millis();
  return (int32_t)(now - stamp) > 0 ? now - stamp : 0;
}
RailTelemetryMsg tele{};
volatile bool haveTele = false;
/* Guards the copy of a telemetry packet into `tele` (and its time stamp)
 * against legSnapshot(), which needs position, target and age from ONE
 * packet. The other accessors read single fields and do without. */
portMUX_TYPE teleMux = portMUX_INITIALIZER_UNLOCKED;

rail::Mode currentMode = rail::Mode::Static;

/* The command the rail is currently being told. Resent every second; the
 * rail acts on `op` only when `seq` changes, so a resend is just a refresh
 * of the motion-permitted flag. */
RailCommandMsg cmd{};
uint32_t lastCmdMs = 0;

/* Guards cmd. The web handlers (AsyncTCP, priority 10) issue commands while
 * the telemetry task (priority 12) resends and edits the same struct, and on
 * this single core the higher one pre-empts mid-update. issue() used to bump
 * seq and THEN set op: a resend in between went out with the new seq and the
 * old op, the rail executed that and ignored the real packet as a resend.
 * tick() could also see the rail echo the old seq and clear the op the
 * handler had just set. Every edit of cmd, and the copy that is sent, now
 * happens under this lock. */
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
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
  RailCommandMsg out;
  portENTER_CRITICAL(&cmdMux);
  cmd.type = MSG_RAIL_COMMAND;
  cmd.version = PROTOCOL_VERSION;
  out = cmd;
  portEXIT_CRITICAL(&cmdMux);
  esp_now_send(railMac, (const uint8_t *)&out, sizeof(out));
  lastCmdMs = millis();
}

/* Issue a new one-shot op: every field first, seq last, all under the lock,
 * so no copy of cmd can ever pair the new seq with the old op. */
void issue(uint8_t op, int32_t target = 0, uint32_t speed = 0, int32_t lo = 0,
           int32_t hi = 0, uint32_t dwell = 0) {
  portENTER_CRITICAL(&cmdMux);
  cmd.op = op;
  cmd.targetSteps = target;
  cmd.speedStepsPerSec = speed;
  cmd.sweepMinSteps = lo;
  cmd.sweepMaxSteps = hi;
  cmd.sweepDwellMs = dwell;
  cmd.seq++;
  portEXIT_CRITICAL(&cmdMux);
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

const char *refusedText(uint8_t r) {
  switch (r) {
    case RAIL_REFUSED_NONE: return "";
    case RAIL_REFUSED_FAULT: return "The rail has a fault. Clear it first.";
    case RAIL_REFUSED_NOT_PERMITTED: return "The rail is not allowed to move while a fan is turning.";
    case RAIL_REFUSED_DRIVER_OFF: return "The rail's motor driver is switched off.";
    case RAIL_REFUSED_BUSY: return "The rail was still moving. Stop it first.";
    case RAIL_REFUSED_NOT_HOMED: return "Home the rail first.";
    case RAIL_REFUSED_SPAN: return "The sweep range is too small.";
    default: return "The rail refused that command.";
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
  portENTER_CRITICAL(&teleMux);
  memcpy(&tele, data, sizeof(tele));
  lastRxMs = millis();
  portEXIT_CRITICAL(&teleMux);
  haveTele = true;

  if (!haveRail || memcmp(railMac, info->src_addr, 6) != 0) {
    memcpy(railMac, info->src_addr, 6);
    addPeer(railMac);
    haveRail = true;
    netlog::printf("[RAIL] linked to %02X:%02X:%02X:%02X:%02X:%02X", railMac[0],
                   railMac[1], railMac[2], railMac[3], railMac[4], railMac[5]);
    send();  // tell it the current mode/permission straight away
  }
}

bool online() {
  const uint32_t last = lastRxMs;  // snapshot before the clock, see msSince()
  return haveRail && last != 0 && msSince(last) < LINK_TIMEOUT_MS;
}

void tick(bool fanStopped, bool fanFaulted) {
  /* THE interlock. Static: the fan must be stopped. Sweep: the fan may run,
   * that is the mode's purpose. A latched fan fault withdraws permission in
   * either mode -- the arm may be coasting. */
  const bool permitted = !fanFaulted &&
                         (currentMode == Mode::Sweep || fanStopped);
  portENTER_CRITICAL(&cmdMux);
  cmd.flags = permitted ? RAIL_FLAG_MOTION_PERMITTED : 0;
  portEXIT_CRITICAL(&cmdMux);

  static bool wasOnline = false;
  const bool now = online();
  if (now != wasOnline) {
    netlog::printf("[RAIL] link %s", now ? "UP" : "DOWN");
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
  /* A refusal on the rail. The website got 200 when the command was SENT,
   * so this is the only way it hears. Logged once per (seq, reason). */
  if (haveTele) {
    static uint8_t seenSeq = 0, seenRefused = RAIL_REFUSED_NONE;
    const uint8_t s = tele.lastSeq, r = tele.refused;
    if ((s != seenSeq || r != seenRefused) && r != RAIL_REFUSED_NONE) {
      netlog::printf("[RAIL] rail refused command %u: %s", s, refusedText(r));
    }
    seenSeq = s;
    seenRefused = r;
  }

  if (haveTele) {
    const uint8_t echoed = tele.lastSeq;
    portENTER_CRITICAL(&cmdMux);
    if (cmd.op != RAIL_OP_NONE && echoed == cmd.seq) cmd.op = RAIL_OP_NONE;
    portEXIT_CRITICAL(&cmdMux);
  }

  /* Deferred centring for Position mode: acts only on real feedback. */
  if (centreWhenIdle && now && haveTele && currentMode == Mode::Static &&
      tele.state == RAIL_STATE_IDLE && (tele.flags & RAIL_TFLAG_HOMED) &&
      !(tele.flags & RAIL_TFLAG_MOVING) && permitted) {
    centreWhenIdle = false;
    const int32_t mid = mmToSteps(travelMmNow() / 2.0f);
    if (labs(mid - tele.positionSteps) > mmToSteps(2.0f)) {
      issue(RAIL_OP_GOTO, mid, 0);
      netlog::printf("[RAIL] position mode: centring");
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
  portENTER_CRITICAL(&cmdMux);
  cmd.flags = 0;
  portEXIT_CRITICAL(&cmdMux);
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
const char *refusedName() { return online() ? refusedText(tele.refused) : ""; }
bool homed() { return online() && (tele.flags & RAIL_TFLAG_HOMED); }
bool moving() { return online() && (tele.flags & RAIL_TFLAG_MOVING); }
bool sweeping() { return online() && tele.state == RAIL_STATE_SWEEPING; }
bool limitHome() { return online() && (tele.flags & RAIL_TFLAG_LIMIT_HOME); }
bool limitIdle() { return online() && (tele.flags & RAIL_TFLAG_LIMIT_IDLE); }
float positionMm() { return online() ? stepsToMm(tele.positionSteps) : 0; }
int direction() {
  if (!online() || !(tele.flags & RAIL_TFLAG_MOVING)) return 0;
  if (tele.targetSteps > tele.positionSteps) return 1;
  if (tele.targetSteps < tele.positionSteps) return -1;
  return 0;
}
float targetMm() { return online() ? stepsToMm(tele.targetSteps) : 0; }
bool legSnapshot(float &posMm, float &tgtMm, uint32_t &ageMs) {
  if (!online()) return false;
  int32_t p, t;
  uint32_t stamp;
  portENTER_CRITICAL(&teleMux);
  p = tele.positionSteps;
  t = tele.targetSteps;
  stamp = lastRxMs;
  portEXIT_CRITICAL(&teleMux);
  posMm = stepsToMm(p);
  tgtMm = stepsToMm(t);
  ageMs = msSince(stamp);  // stamp read first, see msSince()
  return true;
}
float travelMm() { return travelMmNow(); }
bool travelMeasured() { return online() && (tele.flags & RAIL_TFLAG_TRAVEL_MEASURED); }
float sweepMinMm() { return swMin; }
float sweepMaxMm() { return swMax; }
float sweepSpeedMms() { return swSpeed; }
uint32_t sweepDwellMs() { return swDwell; }

}  // namespace rail

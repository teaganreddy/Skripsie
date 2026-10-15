#include "unit_link.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "config.h"
#include "content.h"
#include "file_sync.h"
#include "motor_task.h"
#include "netlog.h"
#include "rotor_stub.h"
#include "web.h"

namespace {

const uint32_t COMMAND_INTERVAL_MS = 200;    // master -> peer, 5 Hz
const uint32_t TELEMETRY_INTERVAL_MS = 200;  // peer -> master, 5 Hz
const uint32_t LINK_TIMEOUT_MS = 2000;       // 10 missed packets at 5 Hz
const uint32_t PEER_LOST_MS = 5000;          // peer: no master for this long -> ramp down

uint8_t otherMac[6] = {0};
bool haveOther = false;
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
uint32_t lastTxMs = 0;

/* How long the peer must have been silent before the master assumes its arm
 * has stopped. The peer only starts its own ramp-down after PEER_LOST_MS of
 * silence, and 700 rpm at 60 rpm/s takes another 11.7 s: 5 + 11.7 plus
 * margin. Until then an offline peer is "maybe still spinning", not stopped. */
const uint32_t PEER_GONE_SAFE_MS = 30000;

/* Guards `cmd` on the master (web handlers vs the telemetry task, which
 * pre-empts them on this single core) and `lastCmd` on the peer (the WiFi
 * callback vs the peer task). See masterIssue() for what went wrong without
 * it. Critical sections, so they are also safe from the WiFi callback. */
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;

// ---- master side ----
UnitTelemetryMsg tele{};
volatile bool haveTele = false;
UnitCommandMsg cmd{};
/* Whether the last telemetry showed the peer stopped (drive off, idle, arm
 * within RPM_STOPPED). Kept so that an offline peer is judged on what it was
 * last seen doing, not assumed stopped. */
volatile bool lastTeleStopped = false;

// ---- peer side ----
/* Woken by the receive callback as soon as a command lands, instead of
 * finding it on its next 50 ms pass. That pass was the biggest term in the
 * loop-sync latency to unit 2's rotor, and it was random (0 to 50 ms). */
TaskHandle_t peerTaskHandle = nullptr;
UnitCommandMsg lastCmd{};
volatile bool cmdPending = false;
uint8_t actedSeq = 0;
bool haveActed = false;
char appliedContent[24] = {0};
bool lostHandled = false;

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = LINK_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = unitlink::isMaster() ? WIFI_IF_AP : WIFI_IF_STA;
  const esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK) Serial.printf("[UNIT] add_peer failed: %s\n", esp_err_to_name(err));
}

bool linkUp() {
  const uint32_t last = lastRxMs;  // snapshot before the clock, see msSince()
  return haveOther && last != 0 && msSince(last) < LINK_TIMEOUT_MS;
}

// ---------------------------------------------------------------------------
//  MASTER
// ---------------------------------------------------------------------------

void masterSend() {
  if (!haveOther) return;
  UnitCommandMsg out;
  portENTER_CRITICAL(&cmdMux);
  cmd.type = MSG_UNIT_COMMAND;
  cmd.version = PROTOCOL_VERSION;
  out = cmd;
  portEXIT_CRITICAL(&cmdMux);
  esp_now_send(otherMac, (const uint8_t *)&out, sizeof(out));
  lastTxMs = millis();
}

/* Issue a one-shot op: op first, seq last, under the lock.
 *
 * It used to be seq++ then op, unlocked. The telemetry task pre-empts the web
 * handlers, so a resend in between sent the new seq with the OLD op; the peer
 * executed that (usually NONE) and ignored the real packet as a resend. For
 * an E-STOP that means the right fan never got it.
 *
 * An unacknowledged E-STOP is also STICKY: the peer has one command slot
 * (gotcha 32), so any op issued before it echoes the E-STOP's seq would
 * overwrite it. A power-on, power-off or fault-clear in that window is
 * dropped instead -- the fan is being emergency-stopped, and the user can
 * repeat the other action once it is. Returns false if dropped. */
bool masterIssue(uint8_t op) {
  bool issued = true;
  portENTER_CRITICAL(&cmdMux);
  if (cmd.op == UNIT_OP_ESTOP && op != UNIT_OP_ESTOP) {
    issued = false;
  } else {
    cmd.op = op;
    cmd.seq++;
  }
  portEXIT_CRITICAL(&cmdMux);
  if (!issued) {
    netlog::printf("[UNIT] op %u dropped: the right fan's E-STOP is not acknowledged yet", op);
    return false;
  }
  masterSend();
  return true;
}

void masterTick() {
  static bool wasUp = false;
  const bool up = linkUp();
  if (up != wasUp) {
    netlog::printf("[UNIT] peer link %s", up ? "UP" : "DOWN");
    wasUp = up;
  }
  // One-shot acknowledged: stop carrying it, so a peer that re-pairs later
  // does not execute a stale POWER_ON.
  if (haveTele) {
    const uint8_t echoed = tele.lastSeq;
    portENTER_CRITICAL(&cmdMux);
    if (cmd.op != UNIT_OP_NONE && echoed == cmd.seq) cmd.op = UNIT_OP_NONE;
    portEXIT_CRITICAL(&cmdMux);
  }
  if (millis() - lastTxMs >= COMMAND_INTERVAL_MS) masterSend();
}

// ---------------------------------------------------------------------------
//  PEER
// ---------------------------------------------------------------------------

void peerSendTelemetry() {
  if (!haveOther) return;
  const Telemetry t = motorctl::snapshot();

  UnitTelemetryMsg m{};
  m.type = MSG_UNIT_TELEMETRY;
  m.version = PROTOCOL_VERSION;
  m.state = (uint8_t)t.state;
  m.lastSeq = actedSeq;
  m.brightness = (uint8_t)web::brightness();
  m.rpmActual = (int16_t)lroundf(t.rpmActual);
  m.rpmTarget = (uint16_t)lroundf(t.rpmTarget);
  m.rotorRpm = (uint16_t)rotor::rotorRpm();
  m.rotorRssi = (int8_t)constrain(rotor::rssi(), -127, 0);
  m.rotorHallLevel = rotor::hallLevel() < 0 ? 2 : (uint8_t)rotor::hallLevel();
  m.voltageQx100 = (int16_t)lroundf(t.voltageQ * 100.0f);
  m.focLoopHz = (uint16_t)min<uint32_t>(t.focLoopHz, 65535);
  m.focMaxGapMsx100 = (uint16_t)min<long>(lroundf(t.focMaxGapMs * 100.0f), 65535);

  m.flags = 0;
  if (t.power) m.flags |= UNIT_TFLAG_POWER;
  if (t.faultLatched) m.flags |= UNIT_TFLAG_FAULT;
  if (t.faultCauseActive) m.flags |= UNIT_TFLAG_FAULT_CAUSE_ACTIVE;
  if (rotor::online()) m.flags |= UNIT_TFLAG_ROTOR_ONLINE;
  if (rotor::hasContent()) m.flags |= UNIT_TFLAG_ROTOR_HAS_CONTENT;
  if (rotor::syncLocked()) m.flags |= UNIT_TFLAG_ROTOR_SYNC_LOCKED;
  if (rotor::pushActive()) m.flags |= UNIT_TFLAG_PUSHING;

  if (t.faultLatched) {
    snprintf(m.faultCode, sizeof(m.faultCode), "%s", t.faultCode);
    snprintf(m.faultMessage, sizeof(m.faultMessage), "%s", t.faultMessage);
  }
  ContentItem sel{};
  if (content::selected(sel)) snprintf(m.contentId, sizeof(m.contentId), "%s", sel.id);

  esp_now_send(otherMac, (const uint8_t *)&m, sizeof(m));
}

void peerApply() {
  // Copied under the lock: a torn copy (new seq, old op) would be executed
  // and the real packet then ignored as a resend.
  UnitCommandMsg c;
  portENTER_CRITICAL(&cmdMux);
  c = lastCmd;
  portEXIT_CRITICAL(&cmdMux);

  /* Continuous fields, every packet. Same clamps as the web handlers: the
   * master already clamped, but this firmware never trusts a number it did
   * not clamp itself. */
  const float rpm = motorctl::clampRpm((float)c.rpmTarget);
  motorctl::cmdSpeed(rpm);
  web::setBrightnessLevel(c.brightness);
  rotor::setScene((c.flags & UNIT_CFLAG_MIRROR) != 0, c.loopSeq, c.startFrame);
  if (c.calFlags & CAL_FLAG_VALID) {
    rotor::setCalibration(c.sectorOffset, (c.calFlags & CAL_FLAG_REVERSE) != 0);
  }

  if (c.contentId[0] && strcmp(c.contentId, appliedContent) != 0) {
    ContentItem it{};
    if (content::find(c.contentId, it)) {
      snprintf(appliedContent, sizeof(appliedContent), "%s", c.contentId);
      content::select(c.contentId);
      rotor::startPush(it.id, it.bytes);
      netlog::printf("[UNIT] master selected \"%s\"", c.contentId);
    } else {
      /* Not on this flash yet. NOT remembered as applied: file sync may
       * deliver it any moment (the master copies uploads here), and it must
       * then be picked up on the next command. It used to be cached, so a
       * copy that arrived afterwards was never shown. Logged once. */
      static char missingLogged[24] = {0};
      if (strcmp(missingLogged, c.contentId) != 0) {
        snprintf(missingLogged, sizeof(missingLogged), "%s", c.contentId);
        netlog::printf("[UNIT] master selected \"%s\" but it is not on this unit (yet)",
                       c.contentId);
      }
    }
  }

  if (haveActed && c.seq == actedSeq) return;
  const bool firstSincePair = !haveActed;
  haveActed = true;
  actedSeq = c.seq;
  if (firstSincePair) return;  // adopt the stale op, do not execute it

  switch (c.op) {
    case UNIT_OP_NONE: break;
    case UNIT_OP_POWER_ON:
      filesync::peerAbortForStart();  // never drive the motor under a flash write
      motorctl::cmdPower(true);
      break;
    case UNIT_OP_POWER_OFF: motorctl::cmdPower(false); break;
    case UNIT_OP_ESTOP: motorctl::cmdStop(); break;
    case UNIT_OP_CLEAR_FAULT: motorctl::cmdClearFault(); break;
    default: Serial.printf("[UNIT] unknown op %u\n", c.op); break;
  }
}

void peerTick() {
  if (cmdPending) {
    cmdPending = false;
    peerApply();
  }
  if (millis() - lastTxMs >= TELEMETRY_INTERVAL_MS) {
    lastTxMs = millis();
    peerSendTelemetry();
  }
  netlog::tick(otherMac, haveOther);

  /* Master gone quiet. If we are spinning, ramp down -- a fan nobody can stop
   * is not a state to stay in. Announced once per outage. */
  /* This one could RAMP DOWN a spinning right fan, and after four times the
   * window unpair from the master, on a healthy link. See msSince(). */
  const uint32_t lastSeen = lastRxMs;
  const uint32_t since = lastSeen ? msSince(lastSeen) : UINT32_MAX;
  if (haveOther && since > PEER_LOST_MS) {
    if (!lostHandled) {
      lostHandled = true;
      const Telemetry t = motorctl::snapshot();
      if (t.power) {
        netlog::printf("[UNIT] master silent -- ramping down");
        motorctl::cmdPower(false);
      }
    }
    if (since > PEER_LOST_MS * 4) {
      esp_now_del_peer(otherMac);
      haveOther = false;
      haveActed = false;
    }
  } else {
    lostHandled = false;
  }
}

void peerTask(void *) {
  for (;;) {
    const Telemetry t = motorctl::snapshot();
    rotor::tick(fabsf(t.rpmActual) > RPM_STOPPED);
    peerTick();
    filesync::tick();  // copies from the master; writes only while this arm is stopped
    // A selection made while this arm turned is saved once it has stopped.
    if (!t.power && t.state == FanState::Idle && fabsf(t.rpmActual) <= RPM_STOPPED) {
      content::flushPrefs();
    }
    // Sleeps as before, but a command from the master wakes it at once.
    ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(rotor::pushActive() ? 2 : 50));
  }
}

}  // namespace

// ===========================================================================

namespace unitlink {

void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 2) return;

  if (isMaster()) {
    if (data[0] != MSG_UNIT_TELEMETRY || len < (int)sizeof(UnitTelemetryMsg)) return;
    memcpy(&tele, data, sizeof(tele));
    tele.faultCode[sizeof(tele.faultCode) - 1] = '\0';
    tele.faultMessage[sizeof(tele.faultMessage) - 1] = '\0';
    tele.contentId[sizeof(tele.contentId) - 1] = '\0';
    lastTeleStopped = !(tele.flags & UNIT_TFLAG_POWER) &&
                      tele.state == (uint8_t)FanState::Idle &&
                      abs(tele.rpmActual) <= (int)RPM_STOPPED;
    haveTele = true;
    lastRxMs = millis();
    if (!haveOther || memcmp(otherMac, info->src_addr, 6) != 0) {
      memcpy(otherMac, info->src_addr, 6);
      addPeer(otherMac);
      haveOther = true;
      netlog::printf("[UNIT] peer stator linked: %02X:%02X:%02X:%02X:%02X:%02X",
                     otherMac[0], otherMac[1], otherMac[2], otherMac[3], otherMac[4],
                     otherMac[5]);
      masterSend();
    }
    return;
  }

  // ---- peer ----
  if (data[0] == MSG_BEACON) {
    if (len < (int)sizeof(BeaconMsg)) return;
    BeaconMsg b;
    memcpy(&b, data, sizeof(b));
    const uint8_t from = b.unitId ? b.unitId : 1;
    if (from != 1) return;  // only the master's beacon
    lastRxMs = millis();
    if (!haveOther || memcmp(otherMac, info->src_addr, 6) != 0) {
      memcpy(otherMac, info->src_addr, 6);
      addPeer(otherMac);
      haveOther = true;
      haveActed = false;  // the next command's op is stale: adopt, do not run
      netlog::printf("[UNIT] paired with master %02X:%02X:%02X:%02X:%02X:%02X",
                     otherMac[0], otherMac[1], otherMac[2], otherMac[3], otherMac[4],
                     otherMac[5]);
    }
    return;
  }
  if (data[0] == MSG_UNIT_COMMAND && len >= (int)sizeof(UnitCommandMsg) && haveOther &&
      memcmp(otherMac, info->src_addr, 6) == 0) {
    lastRxMs = millis();
    portENTER_CRITICAL(&cmdMux);
    memcpy(&lastCmd, data, sizeof(lastCmd));
    lastCmd.contentId[sizeof(lastCmd.contentId) - 1] = '\0';
    portEXIT_CRITICAL(&cmdMux);
    cmdPending = true;
    if (peerTaskHandle) xTaskNotifyGive(peerTaskHandle);
  }
}

const uint8_t *linkMac() { return otherMac; }
bool linkKnown() { return haveOther; }
void wakePeerTask() {
  if (peerTaskHandle) xTaskNotifyGive(peerTaskHandle);
}

void tick() {
  if (isMaster()) masterTick();
  else peerTick();
}

// ---- PEER -----------------------------------------------------------------

bool beginPeer() {
  /* Station mode, never associated: ESP-NOW rides the interface and the
   * channel has to be pinned by hand. Same as the rotor and the rail. */
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);
  Serial.printf("[UNIT] PEER unit %d: no access point, station on channel %u, MAC %s\n",
                UNIT_ID, LINK_CHANNEL, WiFi.macAddress().c_str());
  return true;
}

void startPeerTask() {
  xTaskCreate(peerTask, "peer", 6144, nullptr, 5, &peerTaskHandle);
}

// ---- MASTER -------------------------------------------------------------------

void beginMaster() {
  /* The command struct is otherwise zero-initialised, and the peer applies
   * rpmTarget and brightness from EVERY packet. Left at zero the right rotor
   * was told brightness 0 (dark) until someone moved the slider in Right or
   * Both mode, and the peer's speed target sat at 0 until a start. Seed both
   * the way this board seeds its own: the persisted brightness and the boot
   * setpoint. */
  portENTER_CRITICAL(&cmdMux);
  cmd.rpmTarget = (uint16_t)lroundf(RPM_DEFAULT_TARGET);
  cmd.brightness = (uint8_t)constrain(web::brightness(), 0, 31);
  portEXIT_CRITICAL(&cmdMux);
}

int peerRpmCommanded() { return cmd.rpmTarget; }  // one aligned 16-bit read

// ---- MASTER: peer state -----------------------------------------------------

bool peerOnline() { return linkUp(); }
uint8_t peerState() { return peerOnline() ? tele.state : 0; }
bool peerPower() { return peerOnline() && (tele.flags & UNIT_TFLAG_POWER); }
bool peerFault() { return peerOnline() && (tele.flags & UNIT_TFLAG_FAULT); }
bool peerFaultCauseActive() { return peerOnline() && (tele.flags & UNIT_TFLAG_FAULT_CAUSE_ACTIVE); }
const char *peerFaultCode() { return tele.faultCode; }
const char *peerFaultMessage() { return tele.faultMessage; }
int peerRpmTarget() { return peerOnline() ? tele.rpmTarget : 0; }
int peerRpmActual() { return peerOnline() ? tele.rpmActual : 0; }
int peerBrightness() { return peerOnline() ? tele.brightness : 0; }
int peerRotorRpm() { return peerOnline() ? tele.rotorRpm : 0; }
int peerRotorRssi() { return peerOnline() ? tele.rotorRssi : 0; }
int peerRotorHallLevel() { return (peerOnline() && tele.rotorHallLevel != 2) ? tele.rotorHallLevel : -1; }
bool peerRotorOnline() { return peerOnline() && (tele.flags & UNIT_TFLAG_ROTOR_ONLINE); }
bool peerRotorHasContent() { return peerOnline() && (tele.flags & UNIT_TFLAG_ROTOR_HAS_CONTENT); }
bool peerRotorSyncLocked() { return peerOnline() && (tele.flags & UNIT_TFLAG_ROTOR_SYNC_LOCKED); }
bool peerPushing() { return peerOnline() && (tele.flags & UNIT_TFLAG_PUSHING); }
const char *peerContentId() { return peerOnline() ? tele.contentId : ""; }
float peerVoltageQ() { return tele.voltageQx100 / 100.0f; }
int peerFocLoopHz() { return tele.focLoopHz; }
float peerFocMaxGapMs() { return tele.focMaxGapMsx100 / 100.0f; }

// ---- MASTER: commands ---------------------------------------------------------

void peerPower(bool on) { masterIssue(on ? UNIT_OP_POWER_ON : UNIT_OP_POWER_OFF); }
void peerEstop() { masterIssue(UNIT_OP_ESTOP); }
void peerClearFault() { masterIssue(UNIT_OP_CLEAR_FAULT); }
void peerSetRpm(float rpm) {
  const uint16_t v = (uint16_t)lroundf(constrain(rpm, 0.0f, 65535.0f));
  portENTER_CRITICAL(&cmdMux);
  cmd.rpmTarget = v;
  portEXIT_CRITICAL(&cmdMux);
  masterSend();
}
void peerSetBrightness(int level) {
  portENTER_CRITICAL(&cmdMux);
  cmd.brightness = (uint8_t)constrain(level, 0, 31);
  portEXIT_CRITICAL(&cmdMux);
  masterSend();
}
void peerSelectContent(const char *id) {
  char buf[sizeof(cmd.contentId)];
  snprintf(buf, sizeof(buf), "%s", id ? id : "");
  portENTER_CRITICAL(&cmdMux);
  memcpy(cmd.contentId, buf, sizeof(cmd.contentId));
  portEXIT_CRITICAL(&cmdMux);
  masterSend();
}
void peerSetCalibration(int offsetSectors, bool reverse) {
  const uint8_t off = (uint8_t)(((offsetSectors % 180) + 180) % 180);
  const uint8_t flags = CAL_FLAG_VALID | (reverse ? CAL_FLAG_REVERSE : 0);
  bool changed = false;
  portENTER_CRITICAL(&cmdMux);
  if (off != cmd.sectorOffset || flags != cmd.calFlags) {
    cmd.sectorOffset = off;
    cmd.calFlags = flags;
    changed = true;
  }
  portEXIT_CRITICAL(&cmdMux);
  if (changed) masterSend();
}

void peerSetScene(bool mirror, uint8_t loopSeq, uint8_t startFrame) {
  const uint8_t flags = mirror ? UNIT_CFLAG_MIRROR : 0;
  bool changed = false;
  portENTER_CRITICAL(&cmdMux);
  if (flags != cmd.flags || loopSeq != cmd.loopSeq || startFrame != cmd.startFrame) {
    cmd.flags = flags;
    cmd.startFrame = startFrame;  // before loopSeq: the restart reads both
    cmd.loopSeq = loopSeq;
    changed = true;
  }
  portEXIT_CRITICAL(&cmdMux);
  if (changed) masterSend();
}

bool peerStoppedOrGone() {
  if (peerOnline()) return lastTeleStopped;
  /* Offline. It cannot be spinning under our command if it was last seen
   * stopped and we have not since told it to start (an unacknowledged
   * POWER_ON may have landed just before the link went). Otherwise it may
   * still be running or ramping down on its own fail-safe: not stopped
   * until it has been silent long enough for that ramp to have finished.
   * Never heard since this board booted counts from boot, because the
   * master can reboot under a peer that is still spinning. */
  bool powerOnPending;
  portENTER_CRITICAL(&cmdMux);
  powerOnPending = cmd.op == UNIT_OP_POWER_ON;
  portEXIT_CRITICAL(&cmdMux);
  if (haveTele && lastTeleStopped && !powerOnPending) return true;
  const uint32_t lastSeen = lastRxMs;
  const uint32_t silentMs = haveTele ? msSince(lastSeen) : millis();
  return silentMs > PEER_GONE_SAFE_MS;
}

}  // namespace unitlink

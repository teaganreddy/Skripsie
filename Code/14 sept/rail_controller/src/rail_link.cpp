#include "rail_link.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "config.h"
#include "link_protocol.h"
#include "rail.h"

namespace {

uint8_t statorMac[6] = {0};
bool havePeer = false;

uint32_t lastRxMs = 0;
uint32_t lastTxMs = 0;
uint32_t txCount = 0, rxCount = 0, txFail = 0;

/* The most recent command, and the seq we last ACTED on. A command is resent
 * every beacon so a lost packet self-heals; acting on it only when seq
 * changes is what stops HOME re-running once a second. */
RailCommandMsg lastCmd{};
bool haveCmd = false;
uint8_t actedSeq = 0;
bool haveActed = false;

/* Set from the receive callback (WiFi task), consumed in tick() (loop). Rail
 * commands are applied from loop so that everything touching the state
 * machine runs in one context. */
volatile bool cmdPending = false;

void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) txFail++;
}

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = LINK_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  const esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK) {
    Serial.printf("[LINK] add_peer failed: %s\n", esp_err_to_name(err));
  }
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 2) return;

  if (data[1] != PROTOCOL_VERSION) {
    static uint32_t lastWarnMs = 0;
    if (millis() - lastWarnMs > 5000) {
      lastWarnMs = millis();
      Serial.printf("[LINK] protocol mismatch: stator v%u, rail v%u -- "
                    "link_protocol.h differs between the projects\n",
                    data[1], PROTOCOL_VERSION);
    }
    return;
  }

  switch (data[0]) {
    case MSG_BEACON: {
      if (len < (int)sizeof(BeaconMsg)) return;
      BeaconMsg b;
      memcpy(&b, data, sizeof(b));
      /* Only OUR unit's stator. 0 is "unset" and means unit 1, so the stator
       * as it stands today (which never fills the field) still pairs with a
       * unit-1 rail. */
      const uint8_t beaconUnit = b.unitId ? b.unitId : 1;
      if (beaconUnit != UNIT_ID) return;

      lastRxMs = millis();
      rxCount++;
      if (!havePeer || memcmp(statorMac, info->src_addr, 6) != 0) {
        memcpy(statorMac, info->src_addr, 6);
        addPeer(statorMac);
        havePeer = true;
        /* Forget what we last acted on. The stator resends its current command
         * every second, so the first packet after a (re)pair carries whatever
         * op was issued BEFORE we rebooted or dropped off -- possibly HOME.
         * Adopting that seq without executing it is what keeps "the rail never
         * moves on its own" true across a rail power-cycle. */
        haveActed = false;
        Serial.printf("[LINK] paired with unit %u stator %02X:%02X:%02X:%02X:%02X:%02X\n",
                      beaconUnit, statorMac[0], statorMac[1], statorMac[2],
                      statorMac[3], statorMac[4], statorMac[5]);
      }
      break;
    }

    case MSG_RAIL_COMMAND:
      if (len >= (int)sizeof(RailCommandMsg) && havePeer &&
          memcmp(statorMac, info->src_addr, 6) == 0) {
        lastRxMs = millis();
        rxCount++;
        memcpy(&lastCmd, data, sizeof(RailCommandMsg));
        haveCmd = true;
        cmdPending = true;
      }
      break;

    default:
      break;  // rotor traffic, or something newer than this firmware
  }
}

void applyCommand() {
  const RailCommandMsg c = lastCmd;  // copy: the callback may overwrite

  // Flags are live on every packet, whether or not seq moved.
  rail::setMotionPermitted((c.flags & RAIL_FLAG_MOTION_PERMITTED) != 0);

  if (haveActed && c.seq == actedSeq) return;  // a resend, already done
  const bool firstSincePair = !haveActed;
  haveActed = true;
  actedSeq = c.seq;
  if (firstSincePair) return;  // see the note in onRecv: adopt, do not execute

  switch (c.op) {
    case RAIL_OP_NONE:
      break;
    case RAIL_OP_STOP:
      rail::stop();
      break;
    case RAIL_OP_HOME:
      rail::home();
      break;
    case RAIL_OP_MEASURE:
      rail::measure();
      break;
    case RAIL_OP_GOTO:
      rail::gotoSteps(c.targetSteps, c.speedStepsPerSec);
      break;
    case RAIL_OP_JOG:
      rail::jogSteps(c.targetSteps, c.speedStepsPerSec);
      break;
    case RAIL_OP_SWEEP:
      rail::sweep(c.sweepMinSteps, c.sweepMaxSteps,
                  c.sweepDwellMs ? c.sweepDwellMs : SWEEP_DWELL_MS_DEFAULT,
                  c.speedStepsPerSec);
      break;
    case RAIL_OP_CLEAR:
      rail::clearFault();
      break;
    default:
      Serial.printf("[LINK] unknown rail op %u\n", c.op);
      break;
  }
}

void sendTelemetry() {
  if (!havePeer) return;

  RailTelemetryMsg m{};
  m.type = MSG_RAIL_TELEMETRY;
  m.version = PROTOCOL_VERSION;
  m.state = rail::state();
  m.fault = rail::fault();
  m.lastSeq = actedSeq;
  m.positionSteps = rail::position();
  m.targetSteps = rail::target();
  m.travelSteps = rail::travelSteps();
  m.rateStepsPerSec = rail::rateStepsPerSec();

  m.flags = 0;
  if (rail::homed()) m.flags |= RAIL_TFLAG_HOMED;
  if (rail::limitHome()) m.flags |= RAIL_TFLAG_LIMIT_HOME;
  if (rail::limitIdle()) m.flags |= RAIL_TFLAG_LIMIT_IDLE;
  if (rail::rateStepsPerSec() > 0) m.flags |= RAIL_TFLAG_MOVING;
  if (rail::travelMeasured()) m.flags |= RAIL_TFLAG_TRAVEL_MEASURED;
  if (rail::driverEnabled()) m.flags |= RAIL_TFLAG_DRIVER_ENABLED;

  const esp_err_t err = esp_now_send(statorMac, (const uint8_t *)&m, sizeof(m));
  if (err != ESP_OK) txFail++;
  else txCount++;
}

}  // namespace

namespace raillink {

void begin() {
  /* STA mode, never associated. ESP-NOW rides the interface; the channel has
   * to be set by hand because nothing else will. Must match the stator's AP
   * channel -- ESP-NOW does not roam and a mismatch is total silence. */
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[LINK] esp_now_init FAILED"));
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  Serial.printf("[LINK] ESP-NOW up on channel %u, my MAC %s, unit %d\n",
                LINK_CHANNEL, WiFi.macAddress().c_str(), UNIT_ID);
  Serial.println(F("[LINK] waiting for the stator's beacon..."));
}

void tick() {
  const uint32_t now = millis();

  if (cmdPending) {
    cmdPending = false;
    applyCommand();
  }

  if (now - lastTxMs >= TELEMETRY_INTERVAL_MS) {
    lastTxMs = now;
    sendTelemetry();
  }

  /* Stator gone quiet. Halt anything in progress -- and withdraw motion
   * permission, so nothing can start until a stator says so again. Announced
   * once per outage; re-armed the moment the link is back. */
  static bool lostHandled = false;
  if (havePeer && sinceLastPacketMs() > LINK_LOST_MS) {
    if (!lostHandled) {
      lostHandled = true;
      Serial.println(F("[LINK] stator silent -- halting, motion no longer permitted"));
      if (rail::rateStepsPerSec() > 0) rail::haltNow();
      rail::setMotionPermitted(false);
    }
    if (sinceLastPacketMs() > LINK_LOST_MS * 4) {
      Serial.println(F("[LINK] dropping pairing"));
      esp_now_del_peer(statorMac);
      havePeer = false;
      haveCmd = false;
    }
  } else {
    lostHandled = false;
  }
}

bool paired() { return havePeer && sinceLastPacketMs() < LINK_LOST_MS; }

uint32_t sinceLastPacketMs() {
  return lastRxMs == 0 ? UINT32_MAX : (millis() - lastRxMs);
}

uint32_t sent() { return txCount; }
uint32_t received() { return rxCount; }
uint32_t sendFailures() { return txFail; }

}  // namespace raillink

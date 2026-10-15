#include "rotor_link.h"

#include <atomic>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "apa102.h"
#include "config.h"
#include "content.h"
#include "netlog.h"
#include "rotor_sync.h"

/* Provided by main.cpp -- the display loop's own counters, reported upward so
 * the stator (and therefore the website) can see inside a spinning rotor that
 * has no USB attached. */
extern volatile uint32_t columnsPainted;
extern volatile uint32_t freeRunRpm;

namespace {

/* 5 Hz. The website only needs 2 Hz (API.md), but this is the ESP-NOW link,
 * not the web feed, and the rotor's antenna is physically spinning past the
 * stator's. Sending more often makes a dropped packet cost far less: at 2 Hz a
 * 3 s timeout was six consecutive losses, which is easy to hit. */
const uint32_t TELEMETRY_INTERVAL_MS = 200;
const uint32_t LINK_LOST_MS = 3000;

uint8_t statorMac[6] = {0};
bool havePeer = false;

volatile uint32_t lastRxMs = 0;  // written by the receive callback

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
uint32_t txCount = 0, rxCount = 0, txFail = 0;

uint32_t lastColumnsSnapshot = 0;
uint32_t lastColumnsMs = 0;
uint16_t columnsPerSec = 0;

CommandMsg lastCommand{};
bool haveCommand = false;
/* lastCommand is written by the receive callback and read by the service
 * task (the content-id check in sendTelemetry). A torn read of the id would
 * report the wrong image and trigger a needless re-push. */
portMUX_TYPE cmdMux = portMUX_INITIALIZER_UNLOCKED;
/* A pending loop restart, set by the callback and taken by the display task:
 * 0 = none, else LOOP_RESTART_PENDING | start frame. One word, so the flag
 * and the frame it goes with can never be seen half-written across cores. */
std::atomic<uint32_t> loopRestart{0};
constexpr uint32_t LOOP_RESTART_PENDING = 1u << 16;
bool mirrorSeen = false;

void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) txFail++;
}

void sendContentAck(uint32_t transferId, uint8_t status);  // defined below

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

  /* Version check before anything is believed. The structs would memcpy
   * happily across a layout change and simply produce wrong numbers, which is
   * a much worse failure than refusing the packet. */
  if (data[1] != PROTOCOL_VERSION) {
    static uint32_t lastWarnMs = 0;
    if (millis() - lastWarnMs > 5000) {
      lastWarnMs = millis();
      Serial.printf("[LINK] protocol mismatch: stator v%u, rotor v%u -- "
                    "link_protocol.h differs between the two projects\n",
                    data[1], PROTOCOL_VERSION);
    }
    return;
  }

  /* Only OUR unit's stator. With two units both stators beacon every
   * second, and without this the rotor re-paired to whichever beaconed
   * last -- flip-flopping at 1 Hz between them. 0 means "unset" (an old
   * stator that never fills the field) and is treated as unit 1.
   *
   * Checked BEFORE the link timer is touched: beacons are broadcast, so the
   * other unit's stator would otherwise keep this rotor's "stator still
   * there" timer alive after its own stator had gone quiet. */
  if (data[0] == MSG_BEACON && len >= (int)sizeof(BeaconMsg)) {
    BeaconMsg b;
    memcpy(&b, data, sizeof(b));
    const uint8_t beaconUnit = b.unitId ? b.unitId : 1;
    if (beaconUnit != UNIT_ID) return;
  }

  lastRxMs = millis();
  rxCount++;

  switch (data[0]) {
    case MSG_BEACON: {
      if (!havePeer || memcmp(statorMac, info->src_addr, 6) != 0) {
        memcpy(statorMac, info->src_addr, 6);
        addPeer(statorMac);
        havePeer = true;
        netlog::printf("[LINK] paired with stator %02X:%02X:%02X:%02X:%02X:%02X",
                       statorMac[0], statorMac[1], statorMac[2], statorMac[3],
                       statorMac[4], statorMac[5]);
      }
      break;
    }

    case MSG_COMMAND:
      if (len >= (int)sizeof(CommandMsg)) {
        const bool had = haveCommand;
        const uint8_t prevLoop = lastCommand.loopSeq;
        portENTER_CRITICAL(&cmdMux);
        memcpy(&lastCommand, data, sizeof(CommandMsg));
        lastCommand.contentId[sizeof(lastCommand.contentId) - 1] = '\0';
        portEXIT_CRITICAL(&cmdMux);
        haveCommand = true;
        /* A changed loopSeq is the master saying "frame startFrame (normally
         * 0), now". The first
         * command after pairing is adopted, not acted on: the number is
         * arbitrary and restarting an animation on every re-pair would look
         * like a glitch. */
        if (had && lastCommand.loopSeq != prevLoop) {
          loopRestart.store(LOOP_RESTART_PENDING | lastCommand.startFrame);
        }
        if (apa102::brightness() != lastCommand.brightness) {
          apa102::setBrightness(lastCommand.brightness);
        }
        /* Calibration comes from the stator, which holds it per unit and
         * lets the website adjust it while the arm spins. */
        if (lastCommand.calFlags & CAL_FLAG_VALID) {
          const bool rev = (lastCommand.calFlags & CAL_FLAG_REVERSE) != 0;
          if (lastCommand.sectorOffset != content::offsetSectors() || rev != content::reversed()) {
            content::setOffsetSectors(lastCommand.sectorOffset);
            content::setReversed(rev);
            netlog::printf("[CAL] offset %d sectors (%d deg), rotation %s",
                           content::offsetSectors(), content::offsetSectors() * 360 / ANGLES,
                           rev ? "reversed" : "normal");
          }
        }
        const bool mir = (lastCommand.flags & CMD_FLAG_MIRROR) != 0;
        if (had && mir != mirrorSeen) netlog::printf("[SCENE] mirror %s", mir ? "ON" : "off");
        mirrorSeen = mir;
      }
      break;

    case MSG_CONTENT_BEGIN:
      if (len >= (int)sizeof(ContentBeginMsg)) {
        ContentBeginMsg m;
        memcpy(&m, data, sizeof(m));
        sendContentAck(m.transferId, content::onBegin(m));
      }
      break;

    case MSG_CONTENT_CHUNK:
      if (len > (int)sizeof(ContentChunkMsg)) {
        ContentChunkMsg h;
        memcpy(&h, data, sizeof(h));
        const size_t payload = (size_t)len - sizeof(ContentChunkMsg);
        if (h.dataLen == payload) {
          content::onChunk(h.transferId, h.offset,
                           data + sizeof(ContentChunkMsg), payload);
        }
        /* No per-chunk ack on purpose. ESP-NOW already acknowledges at the MAC
         * layer and the sender retries on failure, so an application ack here
         * would double the round trips for no extra guarantee. */
      }
      break;

    case MSG_CONTENT_END:
      if (len >= (int)sizeof(ContentEndMsg)) {
        ContentEndMsg m;
        memcpy(&m, data, sizeof(m));
        sendContentAck(m.transferId, content::onEnd(m));
      }
      break;

    default:
      break;
  }
}

/* Acked only at BEGIN and END -- the two points where the stator needs to
 * know something it cannot infer from the MAC-layer acks. */
void sendContentAck(uint32_t transferId, uint8_t status) {
  if (!havePeer) return;
  ContentAckMsg a{};
  a.type = MSG_CONTENT_ACK;
  a.version = PROTOCOL_VERSION;
  a.status = status;
  a.transferId = transferId;
  a.bytesReceived = content::bytesReceived();
  esp_now_send(statorMac, (const uint8_t *)&a, sizeof(a));
}

void sendTelemetry() {
  if (!havePeer) return;

  TelemetryMsg m{};
  m.type = MSG_TELEMETRY;
  m.version = PROTOCOL_VERSION;

  /* ===================================================================
   * STAGE 2: no battery sense divider is wired on the rotor yet, and the
   * FireBeetle 2 S3 variant exposes no VBAT pin. Rather than inventing a
   * plausible number the way the old stub did, say so honestly -- API.md
   * allows batteryPct to be null, and the stator forwards that null. A
   * fake battery reading on a public demo is worse than an absent one.
   *
   * When a divider goes in: set batteryValid, read the ADC, and this is
   * the only place that changes.
   * =================================================================== */
  m.batteryValid = 0;
  m.batteryPct = 0;
  m.batteryMv = 0;

  m.rpm = (uint16_t)lroundf(rotorsync::rpm());
  m.columnsPerSec = columnsPerSec;
  m.overruns = (uint16_t)min<uint32_t>(rotorsync::overruns(), 65535);
  m.rejectedPulses = (uint16_t)min<uint32_t>(rotorsync::rejectedPulses(), 65535);
  m.revolutions = rotorsync::revolutions();
  m.brightness = apa102::brightness();
  m.hallLevel = digitalRead(PIN_HALL) ? 1 : 0;

  m.flags = 0;
  if (rotorsync::locked()) m.flags |= LINK_FLAG_SYNC_LOCKED;
  if (!rotorsync::stopped()) m.flags |= LINK_FLAG_SPINNING;
  if (freeRunRpm > 0) m.flags |= LINK_FLAG_FREE_RUN;
  if (content::available()) m.flags |= LINK_FLAG_HAS_CONTENT;

  /* Holding an image, but not the one the stator asked for. The stator only
   * used to re-push when the rotor held NOTHING, so a failed push (or one
   * refused while this rotor was out of range) left the old picture up for
   * good while the website named the new one. Not while a transfer is under
   * way, and not for the sensor test, which is a mode rather than a file. */
  if (haveCommand && content::available() && !content::transferActive()) {
    char want[sizeof(lastCommand.contentId)];
    uint8_t cmdFlags;
    portENTER_CRITICAL(&cmdMux);
    memcpy(want, lastCommand.contentId, sizeof(want));
    cmdFlags = lastCommand.flags;
    portEXIT_CRITICAL(&cmdMux);
    if (want[0] && !(cmdFlags & CMD_FLAG_SENSOR_TEST) && strcmp(want, content::id()) != 0) {
      m.flags |= LINK_FLAG_WRONG_CONTENT;
    }
  }

  const esp_err_t err = esp_now_send(statorMac, (const uint8_t *)&m, sizeof(m));
  if (err != ESP_OK) txFail++;
  else txCount++;
}

}  // namespace

namespace rotorlink {

void begin() {
  /* STA mode, never connected to anything -- ESP-NOW rides on the interface
   * without an association. The channel must then be set by hand, because
   * nothing else is going to set it. */
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(LINK_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[LINK] esp_now_init FAILED"));
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);

  Serial.printf("[LINK] ESP-NOW up on channel %u, my MAC %s\n", LINK_CHANNEL,
                WiFi.macAddress().c_str());
  Serial.println(F("[LINK] waiting for the stator's beacon..."));
}

void tick() {
  /* Forward queued log lines, one per tick, so a burst cannot crowd out
   * telemetry. Nothing to send while unpaired: the lines wait in the queue. */
  {
    LogMsg m{};
    if (netlog::take(m.text, sizeof(m.text))) {
      Serial.println(m.text);  // the serial copy, printed from here, never from the callback
      if (havePeer) {
        m.type = MSG_LOG;
        m.version = PROTOCOL_VERSION;
        m.unitId = UNIT_ID;
        m.source = LOG_SRC_ROTOR;
        esp_now_send(statorMac, (const uint8_t *)&m, sizeof(m));
      }
    }
  }
  const uint32_t now = millis();

  /* Finish any transfer whose END arrived. The checksum is deliberately run
   * here, on the service task, rather than in the receive callback -- see
   * finishTransfer() in content.cpp. The rotor's verdict is acked only once it
   * is actually known. */
  {
    uint8_t status = 0;
    uint32_t transferId = 0;
    if (content::verifyPending(status, transferId)) {
      sendContentAck(transferId, status);
    }
  }

  // Column rate, measured over the same window telemetry is sent on.
  if (now - lastColumnsMs >= 1000) {
    const uint32_t c = columnsPainted;
    columnsPerSec = (uint16_t)min<uint32_t>(c - lastColumnsSnapshot, 65535);
    lastColumnsSnapshot = c;
    lastColumnsMs = now;
  }

  if (now - lastTxMs >= TELEMETRY_INTERVAL_MS) {
    lastTxMs = now;
    sendTelemetry();
  }

  // Forget a stator that has gone quiet, so a reboot re-pairs cleanly.
  if (havePeer && sinceLastPacketMs() > LINK_LOST_MS * 4) {
    Serial.println(F("[LINK] stator silent, dropping pairing"));
    esp_now_del_peer(statorMac);
    havePeer = false;
    haveCommand = false;
  }
}

bool paired() { return havePeer && sinceLastPacketMs() < LINK_LOST_MS; }

uint32_t sinceLastPacketMs() {
  const uint32_t last = lastRxMs;  // snapshot before the clock, see msSince()
  return last == 0 ? UINT32_MAX : msSince(last);
}

uint8_t commandedBrightness() {
  return haveCommand ? lastCommand.brightness : DEFAULT_BRIGHTNESS;
}

bool displayEnabled() {
  // Default to ON when unpaired, so a bench rotor still shows the test pattern.
  return haveCommand ? (lastCommand.flags & CMD_FLAG_DISPLAY_ON) != 0 : true;
}

bool sensorTest() {
  return haveCommand && (lastCommand.flags & CMD_FLAG_SENSOR_TEST) != 0;
}

bool mirror() {
  return haveCommand && (lastCommand.flags & CMD_FLAG_MIRROR) != 0;
}

bool takeLoopRestart(uint16_t &startFrame) {
  const uint32_t v = loopRestart.exchange(0);
  if (!v) return false;
  startFrame = (uint16_t)(v & 0xFFFFu);
  return true;
}

const char *commandedContentId() {
  return haveCommand ? lastCommand.contentId : "";
}

uint32_t sent() { return txCount; }
uint32_t received() { return rxCount; }
uint32_t sendFailures() { return txFail; }

}  // namespace rotorlink

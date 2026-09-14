#include "rotor_link.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include "apa102.h"
#include "config.h"
#include "content.h"
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

uint32_t lastRxMs = 0;
uint32_t lastTxMs = 0;
uint32_t txCount = 0, rxCount = 0, txFail = 0;

uint32_t lastColumnsSnapshot = 0;
uint32_t lastColumnsMs = 0;
uint16_t columnsPerSec = 0;

CommandMsg lastCommand{};
bool haveCommand = false;

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

  lastRxMs = millis();
  rxCount++;

  switch (data[0]) {
    case MSG_BEACON:
      if (!havePeer || memcmp(statorMac, info->src_addr, 6) != 0) {
        memcpy(statorMac, info->src_addr, 6);
        addPeer(statorMac);
        havePeer = true;
        Serial.printf("[LINK] paired with stator %02X:%02X:%02X:%02X:%02X:%02X\n",
                      statorMac[0], statorMac[1], statorMac[2], statorMac[3],
                      statorMac[4], statorMac[5]);
      }
      break;

    case MSG_COMMAND:
      if (len >= (int)sizeof(CommandMsg)) {
        memcpy(&lastCommand, data, sizeof(CommandMsg));
        lastCommand.contentId[sizeof(lastCommand.contentId) - 1] = '\0';
        haveCommand = true;
        if (apa102::brightness() != lastCommand.brightness) {
          apa102::setBrightness(lastCommand.brightness);
        }
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

  m.flags = 0;
  if (rotorsync::locked()) m.flags |= LINK_FLAG_SYNC_LOCKED;
  if (!rotorsync::stopped()) m.flags |= LINK_FLAG_SPINNING;
  if (freeRunRpm > 0) m.flags |= LINK_FLAG_FREE_RUN;
  if (content::available()) m.flags |= LINK_FLAG_HAS_CONTENT;

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
  return lastRxMs == 0 ? UINT32_MAX : (millis() - lastRxMs);
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

const char *commandedContentId() {
  return haveCommand ? lastCommand.contentId : "";
}

uint32_t sent() { return txCount; }
uint32_t received() { return rxCount; }
uint32_t sendFailures() { return txFail; }

}  // namespace rotorlink

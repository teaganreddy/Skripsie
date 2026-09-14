/* ===========================================================================
 *  STAGE 2b: REAL ESP-NOW TELEMETRY -- the simulation is gone
 * ===========================================================================
 *
 * This file used to invent rotor telemetry so the website could be exercised
 * before the rotor existed. It now talks to the real ESP32-S3.
 *
 * The interface in rotor_stub.h is unchanged on purpose, so web.cpp did not
 * have to be touched: online()/batteryPct()/rssi() mean the same things, they
 * are just no longer fiction.
 *
 * What is REAL now:
 *      online()      an actual link, timing out if the rotor goes quiet
 *      rssi()        measured by the WiFi driver on receipt
 *      rpm/columns/overruns/rejected   the rotor's own display diagnostics
 *
 * What is STILL SIMULATED, and clearly marked below:
 *      the rotor-push progress ramp -- content transfer is stage 2c
 *
 * What is HONESTLY ABSENT rather than faked:
 *      the battery. No sense divider is wired on the rotor, so batteryPct()
 *      returns -1 and the status payload emits null. API.md allows exactly
 *      that. The old code drained an invented battery from 87% and it looked
 *      convincing, which on a public demo is worse than showing nothing.
 *
 * The channel matters more than anything else here. ESP-NOW does not scan or
 * roam: both ends must already be on the AP's channel (AP_CHANNEL = 1, and
 * LINK_CHANNEL in link_protocol.h). If they disagree, absolutely nothing is
 * received and nothing reports an error -- the packets simply go out where no
 * one is listening.
 * ======================================================================== */

#include "rotor_stub.h"

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <LittleFS.h>

#include "config.h"
#include "content.h"
#include "link_protocol.h"

namespace {

const uint32_t BEACON_INTERVAL_MS = 1000;
const uint32_t LINK_TIMEOUT_MS = 2000;  // 10 missed packets at 5 Hz

const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

uint8_t rotorMac[6] = {0};
bool haveRotor = false;

volatile uint32_t lastRxMs = 0;
volatile int lastRssi = 0;
TelemetryMsg lastTelemetry{};
volatile bool haveTelemetry = false;

uint32_t lastBeaconMs = 0;
uint32_t rxCount = 0, txCount = 0;

/* --- STAGE 2c: real chunked content push --------------------------------
 *
 * The simulated 45 KB/s ramp is gone. This reads the .povf straight off
 * LittleFS and sends it to the rotor's PSRAM, and the progress the website
 * shows is now actual bytes acknowledged rather than a timer.
 *
 * Flow control is stop-and-wait on ESP-NOW's own MAC-layer acknowledgement:
 * send one chunk, wait for the send callback, retry on failure, advance on
 * success. One round trip per chunk, no protocol of our own to get wrong, and
 * ordered delivery for free. It is not the fastest possible scheme -- a sliding
 * window would beat it -- but correctness first, and the measured throughput
 * printed at the end is what tells us whether more is needed.
 */
const uint32_t PROGRESS_EVENT_MS = 200;
const int MAX_CHUNK_RETRIES = 8;
const uint32_t CHUNK_TIMEOUT_MS = 200;

/* Payload bytes per packet, decided at runtime from the negotiated ESP-NOW
 * version. v2 carries 1470, v1 only 250 -- a 6x difference in throughput, so
 * it is worth asking rather than assuming the conservative number. */
size_t chunkBytes = 200;

char pushId[28] = {0};
uint32_t pushBytes = 0;      // payload length, header excluded
uint32_t pushSentBytes = 0;  // payload bytes the rotor has ACKNOWLEDGED
/* Length of the chunk currently in flight, so an acknowledgement can advance
 * pushSentBytes by exactly what it acknowledged.
 *
 * This used to be `pushSentBytes = pushFile.position()`, which was the bug that
 * made every content push fail. position() counts from the START OF THE FILE
 * and the payload starts 16 bytes in, so each acknowledged chunk advanced the
 * payload offset by dataLen + POVF_HEADER_BYTES. The drift accumulated 16 bytes
 * per chunk: offsets skipped forward, the rotor received fewer bytes than the
 * header promised, and every transfer ended in CONTENT_ERR_INCOMPLETE. The
 * stator happily printed "content push complete" while the rotor discarded the
 * lot, so the website showed 100% and the arm kept showing the fallback. */
uint32_t pushChunkBytes = 0;
uint32_t pushTransferId = 0;
uint32_t pushChecksum = 0;
bool pushing = false;
bool pushFinalSent = true;
bool pushFailed = false;
File pushFile;
uint32_t lastProgressMs = 0;
uint32_t lastTickMs = 0;
uint32_t pushStartMs = 0;

/* Nothing re-sends content after a rotor reboot: the stator only pushed when
 * the website selected something, so powering the rotor on second -- or its
 * browning out mid-demo -- left it showing the fallback pattern for good, with
 * a website insisting an image was selected. tick() re-pushes when the rotor
 * reports holding nothing, rate-limited by this so a transfer that keeps
 * failing retries occasionally instead of becoming a loop. */
const uint32_t AUTO_PUSH_RETRY_MS = 8000;
uint32_t lastPushAttemptMs = 0;

/* How many times the auto re-push may retry the SAME id before giving up.
 *
 * Without this the retry is unbounded: the condition is only "the rotor reports
 * holding no content", which is exactly what a rotor that keeps REJECTING the
 * content also reports. A file the rotor will never accept -- too big, or
 * corrupt -- therefore re-transferred itself every 8 seconds for as long as the
 * fan was powered, each attempt reading LittleFS (which stalls the FOC loop)
 * and flooding the link the telemetry has to share. That is a plausible way to
 * make a healthy link look like it is randomly dropping. */
const int MAX_AUTO_PUSH_FAILURES = 3;
int autoPushFailures = 0;
char autoPushFailedId[28] = {0};

/* The BEGIN packet is the one packet whose loss cannot be recovered from: the
 * rotor drops every chunk of a transfer it was never told about, so the whole
 * push is then guaranteed to fail at END. It used to be fire-and-forget --
 * startPush() sent it and immediately began chunking. Now the chunk loop waits
 * for the rotor's CONTENT_IN_PROGRESS ack and resends BEGIN if it does not
 * come. */
const uint32_t BEGIN_ACK_TIMEOUT_MS = 300;
const int MAX_BEGIN_RETRIES = 6;
bool awaitingBegin = false;
uint32_t beginSentMs = 0;
int beginRetries = 0;
ContentBeginMsg beginMsg{};

// Set by the send callback; the sender waits on these.
volatile bool sendPending = false;
volatile bool sendOk = false;
uint32_t sendStartedMs = 0;
int chunkRetries = 0;

uint8_t packet[16 + 1470];  // header + the largest payload v2 allows

void abortPush(const char *why) {
  if (pushFile) pushFile.close();
  pushing = false;
  awaitingBegin = false;
  pushFailed = true;
  pushFinalSent = false;  // still emit a final progress event, so the UI unsticks
  Serial.printf("[ROTOR] content push FAILED: %s\n", why);
}

void addPeer(const uint8_t *mac) {
  if (esp_now_is_peer_exist(mac)) return;
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = LINK_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_AP;  // the C6 is an access point, not a station
  const esp_err_t err = esp_now_add_peer(&peer);
  if (err != ESP_OK) {
    Serial.printf("[ROTOR] add_peer failed: %s\n", esp_err_to_name(err));
  }
}

void onSent(const esp_now_send_info_t *, esp_now_send_status_t status) {
  sendOk = (status == ESP_NOW_SEND_SUCCESS);
  sendPending = false;
}

void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 2) return;

  /* Content acks arrive on this path too, and carry the rotor's verdict on a
   * transfer -- the one thing the MAC-layer acks cannot tell us. */
  if (data[0] == MSG_CONTENT_ACK && len >= (int)sizeof(ContentAckMsg)) {
    ContentAckMsg a;
    memcpy(&a, data, sizeof(a));
    if (a.transferId == pushTransferId) {
      // The rotor has the BEGIN and has a buffer set up; chunks may now flow.
      if (awaitingBegin && a.status == CONTENT_IN_PROGRESS) awaitingBegin = false;

      if (a.status == CONTENT_OK) {
        autoPushFailures = 0;
        autoPushFailedId[0] = '\0';
        const uint32_t ms = millis() - pushStartMs;
        Serial.printf("[ROTOR] content \"%s\" ACCEPTED: %lu bytes in %lu ms = "
                      "%.1f KB/s  <-- MEASURED ESP-NOW THROUGHPUT\n",
                      pushId, (unsigned long)pushBytes, (unsigned long)ms,
                      ms ? pushBytes / 1024.0f / (ms / 1000.0f) : 0.0f);
      } else if (a.status != CONTENT_IN_PROGRESS) {
        if (strcmp(autoPushFailedId, pushId) != 0) {
          snprintf(autoPushFailedId, sizeof(autoPushFailedId), "%s", pushId);
          autoPushFailures = 0;
        }
        autoPushFailures++;
        Serial.printf("[ROTOR] rotor REJECTED content, status %u (failure %d of %d)\n",
                      a.status, autoPushFailures, MAX_AUTO_PUSH_FAILURES);
        if (autoPushFailures >= MAX_AUTO_PUSH_FAILURES) {
          Serial.printf("[ROTOR] giving up on \"%s\" -- it will not be retried "
                        "again until a different image is selected\n", pushId);
        }
      }
    }
    return;
  }

  if (data[0] != MSG_TELEMETRY) return;

  if (data[1] != PROTOCOL_VERSION) {
    static uint32_t lastWarnMs = 0;
    if (millis() - lastWarnMs > 5000) {
      lastWarnMs = millis();
      Serial.printf("[ROTOR] protocol mismatch: rotor v%u, stator v%u -- "
                    "link_protocol.h differs between the two projects\n",
                    data[1], PROTOCOL_VERSION);
    }
    return;
  }
  if (len < (int)sizeof(TelemetryMsg)) return;

  memcpy((void *)&lastTelemetry, data, sizeof(TelemetryMsg));
  haveTelemetry = true;
  lastRxMs = millis();
  lastRssi = info->rx_ctrl ? info->rx_ctrl->rssi : 0;
  rxCount++;

  if (!haveRotor || memcmp(rotorMac, info->src_addr, 6) != 0) {
    memcpy(rotorMac, info->src_addr, 6);
    addPeer(rotorMac);
    haveRotor = true;
    Serial.printf("[ROTOR] linked to %02X:%02X:%02X:%02X:%02X:%02X\n",
                  rotorMac[0], rotorMac[1], rotorMac[2], rotorMac[3],
                  rotorMac[4], rotorMac[5]);
  }
}


/* Log every up/down transition with how long the previous state lasted and how
 * many packets arrived in it. "Rotor link down" on the website is otherwise a
 * bare fact with no way to tell a brief RF dropout from a rotor that rebooted
 * or browned out. */
void logLinkTransitions() {
  static bool wasOnline = false;
  static uint32_t sinceMs = 0;
  static uint32_t rxAtChange = 0;

  const bool now = rotor::online();
  if (now == wasOnline) return;

  const uint32_t heldMs = millis() - sinceMs;
  if (now) {
    Serial.printf("[ROTOR] link UP after %lu ms down\n", (unsigned long)heldMs);
  } else {
    Serial.printf("[ROTOR] link DOWN -- was up %lu ms, %lu packets in that time "
                  "(%.1f/s expected 5.0)\n",
                  (unsigned long)heldMs, (unsigned long)(rxCount - rxAtChange),
                  heldMs ? (rxCount - rxAtChange) * 1000.0f / heldMs : 0.0f);
  }
  wasOnline = now;
  sinceMs = millis();
  rxAtChange = rxCount;
}
}  // namespace

namespace rotor {

// Defined below; tick() drives it and comes first in this file.
void pumpPush();

void begin() {
  /* Called AFTER WiFi.softAP() so the AP interface already exists and is on
   * its channel. ESP-NOW then rides on that same interface. */
  if (esp_now_init() != ESP_OK) {
    Serial.println(F("[ROTOR] esp_now_init FAILED -- rotor link unavailable"));
    return;
  }
  esp_now_register_recv_cb(onRecv);
  esp_now_register_send_cb(onSent);
  addPeer(BROADCAST);

  /* Ask which ESP-NOW version we negotiated rather than assuming. v2 carries
   * 1470 bytes per packet against v1's 250, and with stop-and-wait flow control
   * the packet size sets the throughput almost single-handedly. */
  uint32_t espnowVersion = 1;
  esp_now_get_version(&espnowVersion);
  chunkBytes = (espnowVersion >= 2) ? 1400 : 200;
  Serial.printf("[ROTOR] ESP-NOW v%lu, %u byte chunks\n",
                (unsigned long)espnowVersion, (unsigned)chunkBytes);

  uint8_t mac[6];
  esp_wifi_get_mac(WIFI_IF_AP, mac);
  Serial.printf("[ROTOR] ESP-NOW up on channel %u, AP MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                LINK_CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  Serial.println(F("[ROTOR] beaconing for the rotor; battery is NOT wired so it "
                   "reports null"));
}

void tick(bool fanSpinning) {
  const uint32_t now = millis();
  if (lastTickMs == 0) lastTickMs = now;
  const float dt = (now - lastTickMs) / 1000.0f;
  lastTickMs = now;
  (void)fanSpinning;

  /* Beacon on broadcast so the rotor can find us without either MAC being
   * hardcoded, and piggyback the current command -- resending it every second
   * means a dropped packet self-heals with no ack machinery. */
  logLinkTransitions();

  if (now - lastBeaconMs >= BEACON_INTERVAL_MS) {
    lastBeaconMs = now;

    BeaconMsg b{};
    b.type = MSG_BEACON;
    b.version = PROTOCOL_VERSION;
    b.channel = LINK_CHANNEL;
    esp_now_send(BROADCAST, (const uint8_t *)&b, sizeof(b));
    txCount++;

    if (haveRotor) {
      CommandMsg c{};
      c.type = MSG_COMMAND;
      c.version = PROTOCOL_VERSION;
      c.brightness = (uint8_t)constrain(web_brightness(), 0, 31);
      c.flags = CMD_FLAG_DISPLAY_ON;
      snprintf(c.contentId, sizeof(c.contentId), "%s", selectedContentId());
      /* Resent every second rather than once on selection, so the rotor picks
       * the mode up even if it booted after the choice was made. */
      if (strcmp(c.contentId, SENSOR_TEST_ID) == 0) c.flags |= CMD_FLAG_SENSOR_TEST;
      esp_now_send(rotorMac, (const uint8_t *)&c, sizeof(c));
    }
  }

  /* Re-push to a rotor that is up but holding nothing. Only while it reports
   * no content, so one success ends it; the sensor test is a mode, not a file,
   * and has nothing to send. */
  const char *want = selectedContentId();
  if (online() && !pushing && haveTelemetry && want && *want &&
      strcmp(want, SENSOR_TEST_ID) != 0 &&
      !(lastTelemetry.flags & LINK_FLAG_HAS_CONTENT) &&
      !(strcmp(autoPushFailedId, want) == 0 &&
        autoPushFailures >= MAX_AUTO_PUSH_FAILURES) &&
      now - lastPushAttemptMs >= AUTO_PUSH_RETRY_MS) {
    Serial.printf("[ROTOR] rotor holds no content -- re-pushing \"%s\"\n", want);
    startPush(want, 0);
  }

  pumpPush();
  (void)dt;
}

/* Sends one chunk per call, waiting on the previous one's MAC acknowledgement.
 * Runs from the telemetry task, NOT from an interrupt or the FOC task.
 *
 * Reading the .povf from LittleFS here does touch flash, which stalls the
 * instruction cache and therefore the FOC loop -- the same mechanism flagged
 * for uploads in web.cpp. Reads are far cheaper than the erases an upload
 * causes, but focMaxGapMs is still the number to watch during a push. */
void pumpPush() {
  if (!pushing) return;

  /* Nothing may be sent until the rotor has acknowledged the BEGIN. Chunks sent
   * before it are silently discarded (content::onChunk drops anything whose
   * transferId it has not been told about), so pushing on regardless meant
   * transferring the entire file into a void and only discovering it at END. */
  if (awaitingBegin) {
    if (millis() - beginSentMs < BEGIN_ACK_TIMEOUT_MS) return;
    if (++beginRetries > MAX_BEGIN_RETRIES) {
      abortPush("rotor never acknowledged the transfer header");
      return;
    }
    beginSentMs = millis();
    esp_now_send(rotorMac, (const uint8_t *)&beginMsg, sizeof(beginMsg));
    Serial.printf("[ROTOR] BEGIN unacknowledged, resend %d of %d\n", beginRetries,
                  MAX_BEGIN_RETRIES);
    return;
  }

  if (sendPending) {
    if (millis() - sendStartedMs < CHUNK_TIMEOUT_MS) return;  // still in flight
    sendPending = false;
    sendOk = false;  // treat a silent callback as a failure and retry
  }

  if (!sendOk && sendStartedMs != 0) {
    if (++chunkRetries > MAX_CHUNK_RETRIES) {
      abortPush("too many retries -- rotor not acknowledging");
      return;
    }
    // Fall through and resend the same offset.
  } else {
    chunkRetries = 0;
    pushSentBytes += pushChunkBytes;  // exactly what was just acknowledged
    pushChunkBytes = 0;
  }

  if (pushSentBytes >= pushBytes) {
    ContentEndMsg e{};
    e.type = MSG_CONTENT_END;
    e.version = PROTOCOL_VERSION;
    e.transferId = pushTransferId;
    e.payloadBytes = pushBytes;
    e.checksum = pushChecksum;
    esp_now_send(rotorMac, (const uint8_t *)&e, sizeof(e));

    pushFile.close();
    pushing = false;
    pushFinalSent = false;
    Serial.printf("[ROTOR] content push complete, %lu bytes sent\n",
                  (unsigned long)pushBytes);
    return;
  }

  const size_t want = min((size_t)(pushBytes - pushSentBytes), chunkBytes);
  ContentChunkMsg h{};
  h.type = MSG_CONTENT_CHUNK;
  h.version = PROTOCOL_VERSION;
  h.dataLen = (uint16_t)want;
  h.offset = pushSentBytes;
  h.transferId = pushTransferId;
  memcpy(packet, &h, sizeof(h));

  pushFile.seek(POVF_HEADER_BYTES + pushSentBytes);
  const int got = pushFile.read(packet + sizeof(h), want);
  if (got != (int)want) {
    abortPush("short read from LittleFS");
    return;
  }
  // Checksum only on the first send of each offset, never on a retry.
  if (chunkRetries == 0) {
    for (int i = 0; i < got; i++) pushChecksum += packet[sizeof(h) + i];
  }

  pushChunkBytes = (uint32_t)want;
  sendPending = true;
  sendOk = false;
  sendStartedMs = millis();
  if (esp_now_send(rotorMac, packet, sizeof(h) + want) != ESP_OK) {
    sendPending = false;
    if (++chunkRetries > MAX_CHUNK_RETRIES) abortPush("esp_now_send kept failing");
  }
}

bool online() {
  return haveRotor && lastRxMs != 0 && (millis() - lastRxMs) < LINK_TIMEOUT_MS;
}


/* -1 means "unknown", and web.cpp emits JSON null for it. There is no battery
 * sense divider on the rotor; inventing a number here is exactly the kind of
 * plausible fiction that survives into a demo unnoticed. */
int batteryPct() {
  if (!online() || !haveTelemetry || !lastTelemetry.batteryValid) return -1;
  return lastTelemetry.batteryPct;
}

int batteryMv() {
  if (!online() || !haveTelemetry || !lastTelemetry.batteryValid) return -1;
  return lastTelemetry.batteryMv;
}

int rssi() { return online() ? lastRssi : 0; }

int rotorRpm() {
  return (online() && haveTelemetry) ? lastTelemetry.rpm : 0;
}
int columnsPerSec() {
  return (online() && haveTelemetry) ? lastTelemetry.columnsPerSec : 0;
}
int overruns() {
  return (online() && haveTelemetry) ? lastTelemetry.overruns : 0;
}
int rejectedPulses() {
  return (online() && haveTelemetry) ? lastTelemetry.rejectedPulses : 0;
}
bool syncLocked() {
  return online() && haveTelemetry &&
         (lastTelemetry.flags & LINK_FLAG_SYNC_LOCKED);
}

/* Whether the rotor is actually holding an image. Worth reporting even though
 * the website does not draw it: "the rotor rejected the transfer" and "the
 * website thinks it sent one" looked identical from outside, which is how a
 * broken content push went unnoticed. Now /api/status says so. */
bool hasContent() {
  return online() && haveTelemetry &&
         (lastTelemetry.flags & LINK_FLAG_HAS_CONTENT);
}

void startPush(const char *id, uint32_t bytes) {
  (void)bytes;  // the real length comes from the file's own header

  if (pushing) {
    // Abandon the old transfer. Chunks still in flight carry the old
    // transferId, and the rotor discards those.
    if (pushFile) pushFile.close();
    pushing = false;
  }

  snprintf(pushId, sizeof(pushId), "%s", id ? id : "");
  pushFailed = false;
  lastPushAttemptMs = millis();

  /* The sensor test is not a file. It is a mode the rotor enters, carried by a
   * flag on the command that is resent every beacon, so there is nothing to
   * transfer and nothing to wait for. */
  if (strcmp(pushId, SENSOR_TEST_ID) == 0) {
    Serial.println(F("[ROTOR] sensor test selected -- rotor switches to the "
                     "per-revolution R/G/B diagnostic, no transfer needed"));
    return;
  }

  ContentItem item{};
  if (!content::find(pushId, item)) {
    Serial.printf("[ROTOR] cannot push \"%s\": not in the library\n", pushId);
    return;
  }
  if (!haveRotor) {
    Serial.println(F("[ROTOR] cannot push: rotor not linked"));
    return;
  }

  char path[80];
  snprintf(path, sizeof(path), "%s/%s.povf",
           item.preset ? PRESET_DIR : USER_DIR, pushId);
  pushFile = LittleFS.open(path, "r");
  if (!pushFile) {
    Serial.printf("[ROTOR] cannot open %s\n", path);
    return;
  }

  pushBytes = (uint32_t)item.frames * item.angles * item.radial * 3;
  pushSentBytes = 0;
  pushChunkBytes = 0;
  pushChecksum = 0;
  pushTransferId++;
  chunkRetries = 0;
  sendPending = false;
  sendOk = true;
  sendStartedMs = 0;
  pushStartMs = millis();
  pushing = true;
  pushFinalSent = true;
  lastProgressMs = 0;

  beginMsg = ContentBeginMsg{};
  beginMsg.type = MSG_CONTENT_BEGIN;
  beginMsg.version = PROTOCOL_VERSION;
  beginMsg.frames = item.frames;
  beginMsg.angles = item.angles;
  beginMsg.radial = item.radial;
  beginMsg.fps = item.fps;
  beginMsg.payloadBytes = pushBytes;
  beginMsg.transferId = pushTransferId;
  snprintf(beginMsg.id, sizeof(beginMsg.id), "%s", pushId);

  // Kept, so it can be resent if the rotor does not acknowledge it.
  awaitingBegin = true;
  beginSentMs = millis();
  beginRetries = 0;
  esp_now_send(rotorMac, (const uint8_t *)&beginMsg, sizeof(beginMsg));

  Serial.printf("[ROTOR] pushing \"%s\": %lu bytes in %u-byte chunks\n", pushId,
                (unsigned long)pushBytes, (unsigned)chunkBytes);
}

bool pushActive() { return pushing; }

bool takeProgressEvent(char *idOut, size_t idLen, float &progress, bool &done) {
  const uint32_t now = millis();
  if (!pushing && !pushFinalSent) {
    pushFinalSent = true;
    snprintf(idOut, idLen, "%s", pushId);
    progress = 1.0f;
    done = true;
    return true;
  }
  if (!pushing) return false;
  if (now - lastProgressMs < PROGRESS_EVENT_MS) return false;
  lastProgressMs = now;
  snprintf(idOut, idLen, "%s", pushId);
  // Real bytes acknowledged, not a timer.
  progress = pushBytes ? ((float)pushSentBytes / (float)pushBytes) : 1.0f;
  done = false;
  return true;
}

}  // namespace rotor

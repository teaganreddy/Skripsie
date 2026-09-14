#include "web.h"

#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ESPmDNS.h>
#include <LittleFS.h>
#include <Preferences.h>
#include <WiFi.h>
#include <esp_wifi.h>

#include "config.h"
#include "content.h"
#include "motor_task.h"
#include "rotor_stub.h"

namespace {

AsyncWebServer server(80);
AsyncEventSource events("/api/events");
Preferences webPrefs;

// Stored and reported only. Brightness is the rotor's business -- the C6 never
// touches the LED strip. It goes out in the status payload so the website's
// slider has something to reflect, and in stage 2 it gets relayed over ESP-NOW.
int brightnessLevel = 24;

/* 802.11 / ESP-IDF disconnect reason codes, named. Reason 15 is the one that
 * matters here -- it is the four-way handshake timing out, which is what a
 * wrong passphrase and a broken handshake both look like from the AP side. */
const char *disconnectReason(uint8_t reason) {
  switch (reason) {
    case 1: return "unspecified";
    case 2: return "previous auth no longer valid";
    case 3: return "station is leaving";
    case 4: return "inactivity timeout";
    case 5: return "AP is out of resources";
    case 6: return "class-2 frame from non-authenticated station";
    case 7: return "class-3 frame from non-associated station";
    case 8: return "station left the BSS";
    case 15: return "4-WAY HANDSHAKE TIMEOUT -- wrong passphrase, or the "
                    "handshake is failing";
    case 16: return "group key update timeout";
    case 23: return "802.1X auth failed";
    case 204: return "handshake timeout (esp)";
    case 205: return "connection failed (esp)";
    default: return "see esp_wifi_types.h wifi_err_reason_t";
  }
}

// ---------------------------------------------------------------------------
//  JSON, written by hand
//
//  The payloads here are small and fixed in shape, so a JSON library would be
//  a dependency and a few KB of flash to save perhaps thirty lines. Escaping
//  matters though: display names come from whatever the user typed into the
//  browser, so they can legitimately contain quotes and backslashes.
// ---------------------------------------------------------------------------

void appendEscaped(String &out, const char *s) {
  for (const char *p = s; *p; ++p) {
    switch (*p) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if ((uint8_t)*p < 0x20) {
          char buf[7];
          snprintf(buf, sizeof(buf), "\\u%04x", *p);
          out += buf;
        } else {
          out += *p;
        }
    }
  }
}

void appendItem(String &out, const ContentItem &it) {
  out += "{\"id\":\"";
  appendEscaped(out, it.id);
  out += "\",\"name\":\"";
  appendEscaped(out, it.name);
  out += "\",\"kind\":\"";
  out += it.preset ? "preset" : "user";
  out += "\",\"frames\":";
  out += it.frames;
  out += ",\"fps\":";
  out += it.fps;
  out += ",\"radial\":";
  out += it.radial;
  out += ",\"angles\":";
  out += it.angles;
  out += ",\"bytes\":";
  out += it.bytes;
  out += "}";
}

String statusJson() {
  const Telemetry t = motorctl::snapshot();

  String out;
  out.reserve(640);
  out += "{\"unit\":{\"id\":\"primary\",\"power\":";
  out += t.power ? "true" : "false";
  out += ",\"state\":\"";
  out += fanStateName(t.state);
  out += "\",\"rpmTarget\":";
  out += (int)lroundf(t.rpmTarget);
  out += ",\"rpmActual\":";
  out += (int)lroundf(t.rpmActual);
  out += ",\"brightness\":";
  out += brightnessLevel;

  out += ",\"fault\":";
  if (t.faultLatched) {
    out += "{\"code\":\"";
    appendEscaped(out, t.faultCode);
    out += "\",\"message\":\"";
    appendEscaped(out, t.faultMessage);
    out += "\"}";
  } else {
    out += "null";
  }

  // --- STAGE 2: replace with real ESP-NOW telemetry ---
  out += ",\"rotor\":{\"online\":";
  out += rotor::online() ? "true" : "false";
  /* -1 means no battery sense hardware. API.md allows null for exactly this,
   * and null is honest where an invented percentage is not. */
  out += ",\"batteryPct\":";
  if (rotor::batteryPct() < 0) out += "null"; else out += rotor::batteryPct();
  out += ",\"batteryMv\":";
  if (rotor::batteryMv() < 0) out += "null"; else out += rotor::batteryMv();
  out += ",\"rssi\":";
  out += rotor::rssi();
  out += "}";
  // --- end stage 2 stub ---

  ContentItem sel{};
  out += ",\"content\":";
  if (content::selected(sel)) {
    out += "{\"id\":\"";
    appendEscaped(out, sel.id);
    out += "\",\"name\":\"";
    appendEscaped(out, sel.name);
    out += "\"}";
  } else {
    out += "null";
  }

  /* Bring-up instrumentation. Not part of the original API.md contract, but
   * the website ignores fields it does not know about, so this is free to
   * carry. focLoopHz is the answer to "is WiFi wrecking my motor control?"
   * and focMaxGapMs is the one that actually matters -- an average rate can
   * look healthy while a single 20 ms stall ruins the velocity estimate. It
   * is a since-boot high-water mark, so it survives even if telemetry itself
   * gets starved during an upload. */
  out += ",\"focLoopHz\":";
  out += t.focLoopHz;
  out += ",\"focMaxGapMs\":";
  out += String(t.focMaxGapMs, 2);
  out += ",\"focGapMsRecent\":";
  out += String(t.focGapMsRecent, 2);
  out += ",\"rpmFast\":";
  out += (int)lroundf(t.rpmFast);
  out += ",\"magnetWeak\":";
  out += t.magnetWeak ? "true" : "false";
  out += ",\"rpmRipple\":";
  out += String(t.rpmRipple, 1);
  out += ",\"voltageQ\":";
  out += String(t.voltageQ, 2);
  out += ",\"estCurrentA\":";
  out += String(t.estCurrentA, 2);
  out += ",\"rotorRpm\":";
  out += rotor::rotorRpm();
  out += ",\"rotorColumnsPerSec\":";
  out += rotor::columnsPerSec();
  out += ",\"rotorOverruns\":";
  out += rotor::overruns();
  out += ",\"rotorRejected\":";
  out += rotor::rejectedPulses();
  out += ",\"rotorSyncLocked\":";
  out += rotor::syncLocked() ? "true" : "false";
  /* False while the fan claims to be showing an image means the transfer to
   * the rotor did not land -- the difference between "selected" and "actually
   * on the arm", which was previously invisible from outside. */
  out += ",\"rotorHasContent\":";
  out += rotor::hasContent() ? "true" : "false";
  out += ",\"freeHeap\":";
  out += (uint32_t)ESP.getFreeHeap();
  out += ",\"fsFreeBytes\":";
  out += (uint32_t)content::freeBytes();

  out += "}}";
  return out;
}

// ---------------------------------------------------------------------------
//  Minimal JSON field extraction
//
//  The command bodies are all one field of a known type ({"on":true},
//  {"rpm":700}, {"level":24}, {"id":"spiral"}), so a scanner for exactly that
//  shape is enough and avoids parsing arbitrary JSON on the device.
// ---------------------------------------------------------------------------

const char *findValue(const char *body, const char *key) {
  char needle[24];
  snprintf(needle, sizeof(needle), "\"%s\"", key);
  const char *p = strstr(body, needle);
  if (!p) return nullptr;
  p = strchr(p + strlen(needle), ':');
  if (!p) return nullptr;
  p++;
  while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') p++;
  return p;
}

bool jsonBool(const char *body, const char *key, bool &out) {
  const char *v = findValue(body, key);
  if (!v) return false;
  if (strncmp(v, "true", 4) == 0) { out = true; return true; }
  if (strncmp(v, "false", 5) == 0) { out = false; return true; }
  if (*v == '1') { out = true; return true; }
  if (*v == '0') { out = false; return true; }
  return false;
}

bool jsonNumber(const char *body, const char *key, float &out) {
  const char *v = findValue(body, key);
  if (!v) return false;
  char *end = nullptr;
  const float parsed = strtof(v, &end);
  if (end == v) return false;  // "rpm": "fast" -- not a number
  out = parsed;
  return true;
}

bool jsonString(const char *body, const char *key, char *out, size_t len) {
  const char *v = findValue(body, key);
  if (!v || *v != '"') return false;
  v++;
  size_t n = 0;
  while (*v && *v != '"' && n < len - 1) {
    if (*v == '\\' && v[1]) v++;  // keep it simple: unescape one level
    out[n++] = *v++;
  }
  out[n] = '\0';
  return true;
}

// ---------------------------------------------------------------------------
//  Request/response helpers
// ---------------------------------------------------------------------------

void sendJson(AsyncWebServerRequest *req, int code, const String &body) {
  AsyncWebServerResponse *res = req->beginResponse(code, "application/json", body);
  res->addHeader("Cache-Control", "no-store");
  req->send(res);
}

void sendOk(AsyncWebServerRequest *req) { sendJson(req, 200, "{\"ok\":true}"); }

/* Errors carry a string the website shows verbatim to whoever is standing in
 * front of the fan, so they are written as sentences, not log lines. */
void sendError(AsyncWebServerRequest *req, int code, const char *message) {
  String body = "{\"error\":\"";
  appendEscaped(body, message);
  body += "\"}";
  sendJson(req, code, body);
}

/* The JSON command bodies arrive through the body handler, which may be
 * called more than once. Accumulate into a malloc'd buffer hung off the
 * request -- ESPAsyncWebServer free()s _tempObject when the request is
 * destroyed, so this cleans itself up even if the client disappears. */
const size_t MAX_BODY = 192;

void collectBody(AsyncWebServerRequest *req, uint8_t *data, size_t len,
                 size_t index, size_t total) {
  if (index == 0) {
    if (req->_tempObject) {
      free(req->_tempObject);
      req->_tempObject = nullptr;
    }
    req->_tempObject = malloc(MAX_BODY);
    if (req->_tempObject) ((char *)req->_tempObject)[0] = '\0';
  }
  if (!req->_tempObject) return;

  char *buf = (char *)req->_tempObject;
  const size_t have = strlen(buf);
  const size_t take = min(len, MAX_BODY - 1 - have);
  memcpy(buf + have, data, take);
  buf[have + take] = '\0';
  (void)total;
}

const char *bodyOf(AsyncWebServerRequest *req) {
  return req->_tempObject ? (const char *)req->_tempObject : "";
}

// ---------------------------------------------------------------------------
//  Handlers
// ---------------------------------------------------------------------------

/* Note the `first` flag rather than testing the loop index: at() can legitimately
 * fail for an entry (a file deleted between the count and the read), and keying
 * the comma off `i` would then emit a leading comma and produce invalid JSON
 * that the whole page fails to parse. */
void appendLibraryArray(String &body) {
  body += '[';
  ContentItem it{};
  bool first = true;
  const size_t n = content::count();
  for (size_t i = 0; i < n; i++) {
    if (!content::at(i, it)) continue;
    if (!first) body += ',';
    first = false;
    appendItem(body, it);
  }
  body += ']';
}

void handleStatus(AsyncWebServerRequest *req) {
  // Same payload as the status event, plus the library (API.md section 1).
  String body = statusJson();
  body.remove(body.length() - 1);  // drop the outer '}' to splice in "library"
  body += ",\"library\":";
  appendLibraryArray(body);
  body += "}";
  sendJson(req, 200, body);
}

void handleLibrary(AsyncWebServerRequest *req) {
  String body;
  appendLibraryArray(body);
  sendJson(req, 200, body);
}

void handlePower(AsyncWebServerRequest *req) {
  const Telemetry t = motorctl::snapshot();
  if (t.faultLatched) {
    sendError(req, 409,
              "The fan has stopped because of a fault. Clear the fault before "
              "starting it again.");
    return;
  }
  bool on = false;
  if (!jsonBool(bodyOf(req), "on", on)) {
    sendError(req, 400, "That request did not say whether to start or stop.");
    return;
  }
  /* The other half of the upload guard below. Refusing uploads while the arm
   * turns is no use if the arm can start turning DURING one -- the flash
   * erases would then land exactly where they must not. Stopping is always
   * allowed. A client that vanishes mid-upload is cleaned up by
   * content::uploadWatchdog() after 15 s, so this cannot wedge the fan off. */
  if (on && content::uploadActive()) {
    sendError(req, 409,
              "The fan is still receiving an image. Wait for that to finish "
              "before starting it.");
    return;
  }
  motorctl::cmdPower(on);
  sendOk(req);
}

void handleSpeed(AsyncWebServerRequest *req) {
  float rpm = 0.0f;
  if (!jsonNumber(bodyOf(req), "rpm", rpm)) {
    sendError(req, 400, "That request did not include a valid speed.");
    return;
  }
  /* Clamped in firmware, always. The website's slider range is a UI
   * affordance, and a hand-written POST is not bound by it at all. */
  const float clamped = motorctl::clampRpm(rpm);

  /* Zero is the slider's bottom stop and means STOP.
   *
   * It is a CONTROLLED power-down -- the same 60 rpm/s ramp as the power
   * button -- and deliberately NOT /api/stop, which is the E-STOP and cuts
   * drive so the arm coasts. Dragging a slider to its end must not be an
   * emergency stop.
   *
   * Note what this does NOT do: moving the slider back up sets the target but
   * does not restart the motor. Speed commands have never started the fan, and
   * an arm that spins up because someone brushed a slider would be a genuinely
   * bad property for a machine with a half-metre blade on it. */
  if (clamped <= 0.0f) {
    motorctl::cmdSpeed(0.0f);
    motorctl::cmdPower(false);
  } else {
    motorctl::cmdSpeed(clamped);
  }

  String body = "{\"ok\":true,\"rpm\":";
  body += (int)lroundf(clamped);
  body += "}";
  sendJson(req, 200, body);
}

void handleBrightness(AsyncWebServerRequest *req) {
  float level = 0.0f;
  if (!jsonNumber(bodyOf(req), "level", level)) {
    sendError(req, 400, "That request did not include a valid brightness.");
    return;
  }
  brightnessLevel = constrain((int)lroundf(level), BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  webPrefs.putInt("bright", brightnessLevel);

  String body = "{\"ok\":true,\"level\":";
  body += brightnessLevel;
  body += "}";
  sendJson(req, 200, body);
}

void handleStop(AsyncWebServerRequest *req) {
  motorctl::cmdStop();
  sendOk(req);
}

void handleFaultClear(AsyncWebServerRequest *req) {
  const Telemetry t = motorctl::snapshot();
  if (!t.faultLatched) {
    sendOk(req);  // nothing to clear; idempotent
    return;
  }
  if (t.faultCauseActive) {
    // API.md: refuse while the cause is still present, and say why.
    sendError(req, 409, t.clearBlockedReason);
    return;
  }
  motorctl::cmdClearFault();
  sendOk(req);
}

void handleSelect(AsyncWebServerRequest *req) {
  char id[28] = {0};
  if (!jsonString(bodyOf(req), "id", id, sizeof(id))) {
    sendError(req, 400, "That request did not say which image to load.");
    return;
  }
  ContentItem it{};
  if (!content::find(id, it)) {
    sendError(req, 404, "That image is not on the fan.");
    return;
  }
  content::select(id);
  // STAGE 2: this is where the real ESP-NOW push would start.
  rotor::startPush(it.id, it.bytes);
  sendOk(req);
}

void handleDelete(AsyncWebServerRequest *req) {
  String path = req->url();
  const String prefix = "/api/library/";
  if (!path.startsWith(prefix)) {
    sendError(req, 404, "That image is not on the fan.");
    return;
  }
  // The website sends encodeURIComponent(id), so decode before matching.
  String id = req->urlDecode(path.substring(prefix.length()));

  ContentItem it{};
  const bool wasPreset = content::find(id.c_str(), it) && it.preset;
  const char *err = content::remove(id.c_str());
  if (err) {
    sendError(req, wasPreset ? 403 : 404, err);
    return;
  }
  sendOk(req);
}

// --- upload -----------------------------------------------------------------

/* POST /api/upload?name=... with a raw application/octet-stream body -- not
 * multipart, so this is the body handler rather than the upload handler.
 *
 * The payload is written to LittleFS as it arrives and is never buffered
 * whole. The 16-byte .povf header is validated the moment it is complete, so
 * a bad file is rejected after 16 bytes rather than after a megabyte, and any
 * partial file is deleted on every failure path.
 *
 * ---------------------------------------------------------------------------
 * WATCH focMaxGapMs DURING AN UPLOAD. This is the one place in the firmware
 * where task priority does not protect the control loop.
 *
 * Writing to LittleFS goes through the SPI flash driver, which disables the
 * instruction cache while a page program or sector erase is in flight. Code
 * executing from flash -- which includes the FOC task -- cannot run at all
 * during that window, no matter that it sits at priority 15 and this handler
 * sits at 10. A page program is a few hundred microseconds; a 4 KB sector
 * erase can be tens of milliseconds.
 *
 * So an upload is the realistic worst case for the sensor-read gap, and a
 * ~1.1 MB payload does a lot of erasing. If focMaxGapMs climbs past the 5 ms
 * budget during an upload, the fix is not more tuning here -- it is to refuse
 * uploads while the motor is spinning.
 * ------------------------------------------------------------------------ */

/* Upload bookkeeping is PER REQUEST, not global. A single shared "failed" flag
 * would be a bug the moment two clients upload at once: the second request is
 * correctly refused, but its refusal would also abort the first one's
 * perfectly good transfer. The token guards the mirror case -- if our upload
 * was abandoned and a different one has since started, our late-arriving
 * chunks must not be written into someone else's file. */
struct UploadCtx {
  uint32_t token;
  bool failed;
};

void handleUploadBody(AsyncWebServerRequest *req, uint8_t *data, size_t len,
                      size_t index, size_t total) {
  if (index == 0) {
    if (req->_tempObject) {
      free(req->_tempObject);
      req->_tempObject = nullptr;
    }
    req->_tempObject = malloc(sizeof(UploadCtx));
    if (!req->_tempObject) {
      sendError(req, 507, "The fan is out of memory. Try again in a moment.");
      return;
    }
    UploadCtx *ctx = (UploadCtx *)req->_tempObject;
    ctx->failed = false;
    ctx->token = 0;

    /* ---------------------------------------------------------------------
     * NO UPLOADS WHILE THE ARM IS TURNING.
     *
     * Writing to LittleFS disables the instruction cache while a page program
     * or sector erase is in flight, and the FOC task executes from flash -- so
     * it stops dead for the duration regardless of sitting at priority 15. A
     * page program is a few hundred microseconds; a 4 KB sector erase is tens
     * of milliseconds, and a 600 KB upload does a lot of erasing.
     *
     * That was survivable at the old 250 rpm ceiling. It is not at 700:
     *
     *      250 rpm   240 ms per revolution   unwrapping breaks above ~120 ms
     *      700 rpm  85.7 ms per revolution   unwrapping breaks above ~43 ms
     *
     * A 40 ms erase stall is 280 degrees of rotation at 700 rpm. Past 180 the
     * encoder unwrapping infers the wrong direction, the velocity estimate
     * inverts, and the PID acts on the inverted figure -- an E06 overspeed
     * latch at best, a torque transient into a spinning half-metre arm at
     * worst. This is the mitigation web.cpp has recommended in the comment
     * above since bring-up; the speed increase is what made it necessary.
     *
     * rpmActual is the displacement-over-250 ms figure, not SimpleFOC's
     * differentiated one. That matters: the differentiated estimate carries
     * ~20 rpm of quantisation noise at standstill, so a threshold test against
     * it would never pass and uploads would be refused forever. See the
     * Telemetry comments in motor_task.h. */
    const Telemetry mt = motorctl::snapshot();
    if (mt.power || fabsf(mt.rpmActual) > RPM_STOPPED) {
      ctx->failed = true;
      sendError(req, 409,
                "The fan has to be stopped before you can send it a new image. "
                "Turn it off, wait for the arm to come to a complete stop, then "
                "try again.");
      return;
    }

    char name[32] = "Untitled";
    if (req->hasParam("name")) {
      snprintf(name, sizeof(name), "%s", req->getParam("name")->value().c_str());
    }

    const char *err = content::uploadBegin(name, total);
    if (err) {
      ctx->failed = true;
      sendError(req, 507, err);
      return;
    }
    ctx->token = content::uploadToken();
  }

  UploadCtx *ctx = (UploadCtx *)req->_tempObject;
  if (!ctx || ctx->failed) return;  // already answered; drain the rest silently
  if (ctx->token != content::uploadToken()) {
    // Our upload was abandoned (watchdog) and another has taken its place.
    ctx->failed = true;
    sendError(req, 409, "That upload timed out and was cancelled.");
    return;
  }

  const char *err = content::uploadChunk(data, len);
  if (err) {
    ctx->failed = true;
    sendError(req, 400, err);
    return;
  }

  if (index + len == total) {
    ContentItem item{};
    err = content::uploadFinish(item);
    if (err) {
      ctx->failed = true;
      sendError(req, 400, err);
      return;
    }
    // STAGE 2: the real ESP-NOW push would start here.
    rotor::startPush(item.id, item.bytes);

    String body;
    appendItem(body, item);  // API.md: return the new library entry
    sendJson(req, 200, body);
  }
}

void handleUploadRequest(AsyncWebServerRequest *req) {
  // Reached only when the body handler did not already respond -- e.g. a POST
  // with no body at all.
  if (!req->getResponse()) {
    sendError(req, 400, "That upload did not contain any image data.");
  }
}

// ---------------------------------------------------------------------------
//  Telemetry task
// ---------------------------------------------------------------------------

void telemetryTask(void *) {
  uint32_t lastStatusMs = 0;
  for (;;) {
    const uint32_t now = millis();

    const Telemetry t = motorctl::snapshot();
    rotor::tick(fabsf(t.rpmActual) > RPM_STOPPED);
    content::uploadWatchdog();

    if (events.count() > 0) {
      char id[28];
      float progress = 0.0f;
      bool done = false;
      if (rotor::takeProgressEvent(id, sizeof(id), progress, done)) {
        String body = "{\"id\":\"";
        appendEscaped(body, id);
        body += "\",\"progress\":";
        body += String(progress, 3);
        body += ",\"done\":";
        body += done ? "true" : "false";
        body += "}";
        events.send(body.c_str(), "rotor-progress");
      }

      if (now - lastStatusMs >= STATUS_EVENT_MS) {
        lastStatusMs = now;
        events.send(statusJson().c_str(), "status");
      }
    }

    /* Fast while a content push is running, idle otherwise.
     *
     * pumpPush() sends one chunk per pass (stop-and-wait on the MAC-layer ack),
     * so this delay IS the transfer rate: at 50 ms it caps out at 20 chunks/s,
     * or about 28 KB/s with 1400-byte chunks. That would have made the
     * "measured ESP-NOW throughput" figure a measurement of this sleep rather
     * than of the radio. 2 ms lets the link be the limit, which is the whole
     * point of measuring it. */
    vTaskDelay(pdMS_TO_TICKS(rotor::pushActive() ? 2 : 50));
  }
}

}  // namespace

namespace web {

int brightness() { return brightnessLevel; }

int stationCount() { return WiFi.softAPgetStationNum(); }

bool begin() {
  webPrefs.begin("web", false);
  brightnessLevel = webPrefs.getInt("bright", 24);

  /* AP event logging, registered BEFORE the AP starts so nothing is missed.
   *
   * This exists because "I cannot connect" covers four completely different
   * failures and the fix for each is different. The log turns it into a
   * decision tree:
   *
   *   no PROBEREQ at all   -> the client never sees us. Beacons are not getting
   *                           out: radio init, channel, or the FOC task
   *                           starving the WiFi stack.
   *   PROBEREQ, no STACONNECTED  -> seen but association/auth fails.
   *   STACONNECTED, no STAIPASSIGNED -> associated but DHCP is not answering.
   *   STAIPASSIGNED but no page -> network is fine, it is the HTTP server. */
  WiFi.onEvent([](arduino_event_id_t event, arduino_event_info_t info) {
    switch (event) {
      case ARDUINO_EVENT_WIFI_AP_START:
        Serial.println(F("[WIFI] AP started"));
        break;
      case ARDUINO_EVENT_WIFI_AP_STOP:
        Serial.println(F("[WIFI] AP stopped"));
        break;
      case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
        // Rate-limited even when enabled: a room full of phones scanning
        // produces a flood of these.
        {
          static uint32_t lastProbeMs = 0;
          if (WIFI_LOG_PROBES && millis() - lastProbeMs > 2000) {
            lastProbeMs = millis();
            Serial.printf("[WIFI] probe request seen (rssi %d) -- a client can "
                          "see this AP\n",
                          info.wifi_ap_probereqrecved.rssi);
          }
        }
        break;
      /* IMPORTANT: this event does NOT mean "associated". ESP-IDF puts a
       * station into the AP's station list as soon as 802.11 association
       * completes -- which is what softAPgetStationNum() counts -- but only
       * raises STACONNECTED once the station is fully AUTHORISED, i.e. after
       * the WPA2 four-way handshake succeeds.
       *
       * So "station count goes 0 -> 1 -> 0 with no STACONNECTED" is the
       * signature of a handshake that never completed. That is a much more
       * precise reading than "it connected then dropped". */
      case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
        Serial.printf("[WIFI] station AUTHORISED (handshake ok), now %u\n",
                      WiFi.softAPgetStationNum());
        break;
      case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
        Serial.printf("[WIFI] station left: reason %u (%s), now %u\n",
                      info.wifi_ap_stadisconnected.reason,
                      disconnectReason(info.wifi_ap_stadisconnected.reason),
                      WiFi.softAPgetStationNum());
        break;
      case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
        Serial.printf("[WIFI] DHCP gave out %s -- open http://%s/ now\n",
                      IPAddress(info.wifi_ap_staipassigned.ip.addr).toString().c_str(),
                      WiFi.softAPIP().toString().c_str());
        break;
      default:
        break;
    }
  });

  WiFi.mode(WIFI_AP);
  if (!WiFi.softAP(AP_SSID, AP_OPEN ? nullptr : AP_PASSWORD, AP_CHANNEL, 0,
                   AP_MAX_CLIENTS)) {
    Serial.println(F("[WIFI] softAP failed"));
    return false;
  }

  /* Restrict the AP to 802.11 b/g/n, 20 MHz. This is the fix for clients that
   * associate and are then thrown off a second later.
   *
   * The ESP32-C6 is a WiFi 6 part, and this Arduino core's default protocol
   * mask (WIFI_PROTOCOL_DEFAULT in WiFiGeneric.cpp) includes
   * WIFI_PROTOCOL_11AX. Advertising HE capability from a SoftAP is a known
   * interop problem: the client associates at the 802.11 layer, the WPA2
   * four-way handshake then fails, and the client deauthenticates. iOS reports
   * that as "incorrect password" -- which is misleading, and cost a bring-up
   * session. The symptom in the log was a station appearing for about four
   * seconds and then leaving.
   *
   * Nothing here needs 11ax. The largest thing that ever crosses this link is a
   * ~1 MB upload over a few seconds, and b/g/n is what every phone and laptop
   * in the room supports without argument. */
  /* Probe-request events are masked off by ESP-IDF by default, which is why no
   * "probe request seen" line ever appeared even while clients were clearly
   * scanning. Unmask everything -- this is a bench tool, the extra events cost
   * nothing here. */
  esp_wifi_set_event_mask(0);

  esp_err_t protoErr = esp_wifi_set_protocol(
      WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  esp_err_t bwErr = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  Serial.printf("[WIFI] AP radio: 11b/g/n %s, HT20 %s\n",
                protoErr == ESP_OK ? "ok" : esp_err_to_name(protoErr),
                bwErr == ESP_OK ? "ok" : esp_err_to_name(bwErr));
  // Modem sleep parks the radio between beacons, which shows up as latency
  // spikes on every request and, worse, as jitter the FOC task has to share
  // the CPU with at unpredictable moments.
  WiFi.setSleep(false);

  if (MDNS.begin(MDNS_HOSTNAME)) {
    MDNS.addService("http", "tcp", 80);
    Serial.printf("[WIFI] http://%s.local\n", MDNS_HOSTNAME);
  } else {
    Serial.println(F("[WIFI] mDNS failed to start"));
  }
  Serial.printf("[WIFI] AP \"%s\" (%s) at %s\n", AP_SSID,
                AP_OPEN ? "open, no password" : "WPA2",
                WiFi.softAPIP().toString().c_str());

  /* CORS. The device serves the page itself in normal use, so this is not
   * needed for an open-day visitor -- but it IS needed for the development
   * workflow in the website's README:
   *
   *     http://localhost:8080/?api=http://192.168.4.1
   *
   * That page is a different origin from the device, so without these headers
   * the browser blocks every request and the JSON POSTs never even leave, they
   * fail at the OPTIONS preflight. Wide-open is acceptable here: it is an
   * isolated access point with no internet route and nothing worth stealing. */
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Origin", "*");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Methods",
                                       "GET, POST, DELETE, OPTIONS");
  DefaultHeaders::Instance().addHeader("Access-Control-Allow-Headers",
                                       "Content-Type");

  /* ESP-NOW starts HERE, not in setup() before WiFi -- it rides on the AP
   * interface and that has to exist and be on its channel first. */
  rotor::begin();

  // SSE. Each client holds its own message queue in RAM, and an open day is a
  // crowd -- cap it rather than letting the tenth phone exhaust the heap.
  events.onConnect([](AsyncEventSourceClient *client) {
    if (events.count() > MAX_SSE_CLIENTS) {
      client->close();
      return;
    }
    client->send(statusJson().c_str(), "status");  // prime the UI immediately
  });
  server.addHandler(&events);

  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/library", HTTP_GET, handleLibrary);

  server.on("/api/power", HTTP_POST, handlePower, nullptr, collectBody);
  server.on("/api/speed", HTTP_POST, handleSpeed, nullptr, collectBody);
  server.on("/api/brightness", HTTP_POST, handleBrightness, nullptr, collectBody);
  server.on("/api/stop", HTTP_POST, handleStop);
  server.on("/api/fault/clear", HTTP_POST, handleFaultClear);
  server.on("/api/library/select", HTTP_POST, handleSelect, nullptr, collectBody);

  server.on("/api/upload", HTTP_POST, handleUploadRequest, nullptr,
            handleUploadBody);

  /* DELETE /api/library/{id}. A prefix matcher rather than a regex: regex
   * routing in this library needs ASYNCWEBSERVER_REGEX defined at build time,
   * and without it the pattern would be treated as a literal URI and silently
   * never match. The handler pulls the id back out of the URL itself. */
  server.on(AsyncURIMatcher::prefix("/api/library/"), HTTP_DELETE, handleDelete);

  // CORS preflight for the cross-origin development workflow above.
  server.on(AsyncURIMatcher::prefix("/api/"), HTTP_OPTIONS,
            [](AsyncWebServerRequest *req) { req->send(204); });

  /* Static files from LittleFS. The bundle ships pre-gzipped, and
   * AsyncStaticWebHandler handles that correctly for us: it looks for
   * "<path>.gz" first (_tryGzipFirst defaults true), and AsyncFileResponse
   * then sets Content-Encoding: gzip while deriving the MIME type from the
   * INNER extension -- so /js/app.js resolves to app.js.gz, is served with
   * Content-Type: text/javascript and Content-Encoding: gzip, and the browser
   * inflates it. Getting that wrong is what shows up as a blank page.
   *
   * Worth knowing when testing by hand: the handler does not check the
   * request's Accept-Encoding, it just serves the .gz. Every browser sends
   * the header so this is fine in practice, but plain `curl` will hand you
   * gzip bytes unless you pass --compressed. */
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.onNotFound([](AsyncWebServerRequest *req) {
    if (req->url().startsWith("/api/")) {
      sendError(req, 404, "The fan does not know that command.");
    } else {
      sendError(req, 404,
                "That page is not on the fan. Try http://fan.local/ instead.");
    }
  });

  server.begin();
  Serial.println(F("[WEB] server started"));
  return true;
}

void startTelemetryTask() {
  /* Its own task at priority 12: above AsyncTCP (10) so a long upload cannot
   * starve telemetry -- which would be exactly the wrong moment to go blind,
   * since an upload is the worst case this instrumentation exists to measure
   * -- and below the FOC task (15) so it can never delay the control loop. */
  xTaskCreate(telemetryTask, "telemetry", 4096, nullptr, 12, nullptr);
}

}  // namespace web

/* Bridges for rotor_stub.cpp, which builds the command sent to the rotor.
 * Declared in rotor_stub.h rather than web.h to keep the dependency one-way. */
int web_brightness() { return web::brightness(); }

const char *selectedContentId() {
  static ContentItem sel;
  return content::selected(sel) ? sel.id : "";
}

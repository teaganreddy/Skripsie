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
#include "file_sync.h"
#include "motor_task.h"
#include "netlog.h"
#include "rail_link.h"
#include "rotor_stub.h"
#include "scene.h"
#include "unit_link.h"

namespace {

AsyncWebServer server(80);
AsyncEventSource events("/api/events");
Preferences webPrefs;

// Unit 1's brightness. The C6 never touches the LED strip: this is reported
// in the status payload and sent to the rotor in every CommandMsg.
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
  /* Uploads only: whether the right fan has its own copy yet (file_sync). */
  if (!it.preset) {
    out += ",\"onRight\":";
    out += filesync::state(it.id) == filesync::State::Synced ? "true" : "false";
  }
  out += "}";
}

/* ---------------------------------------------------------------------------
 *  Which fans the controls drive: LEFT (unit 1, this board), RIGHT (unit 2,
 *  the peer stator) or BOTH. Held here and persisted so a page reload does
 *  not lose it. Changing it is REFUSED unless every fan is stopped: a pre-run
 *  decision, never a live switch that stops a spinning arm by surprise.
 * ------------------------------------------------------------------------- */
enum class DisplayMode : uint8_t { Left = 0, Right = 1, Both = 2 };
DisplayMode displayMode = DisplayMode::Left;

/* Angle calibration per unit (gotcha 27), the master's copy. Held here so
 * it survives a reboot and can be adjusted from the website while the arms
 * spin, since no spinning board can have a serial cable. Unit 1's goes to
 * our rotor directly; unit 2's rides the peer link. */
struct Cal { int offset; bool reverse; };
Cal cal[2] = {{0, false}, {0, false}};

void applyCalibration() {
  rotor::setCalibration(cal[0].offset, cal[0].reverse);
  unitlink::peerSetCalibration(cal[1].offset, cal[1].reverse);
}

void saveCalibration(int unit) {
  char k[8];
  snprintf(k, sizeof(k), "c%doff", unit);
  webPrefs.putInt(k, cal[unit - 1].offset);
  snprintf(k, sizeof(k), "c%drev", unit);
  webPrefs.putBool(k, cal[unit - 1].reverse);
}

void appendCalibration(String &out, int unit) {
  out += ",\"calibration\":{\"offset\":";
  out += cal[unit - 1].offset;
  out += ",\"reverse\":";
  out += cal[unit - 1].reverse ? "true" : "false";
  out += "}";
}

const char *displayModeName(DisplayMode m) {
  return m == DisplayMode::Right ? "right" : m == DisplayMode::Both ? "both" : "left";
}
bool leftActive() { return displayMode != DisplayMode::Right; }
bool rightActive() { return displayMode != DisplayMode::Left; }

/* "Stopped": drive off AND the arm not turning. rpmActual is the 250 ms
 * displacement figure (gotcha 4), so it reads a true zero on a stationary
 * shaft rather than +-20 rpm of encoder dither.
 *
 * An offline peer is NOT simply "stopped" any more. It used to be, but the
 * master calls it offline after 2 s while the peer only starts ramping down
 * after 5 s of silence and then needs up to ~12 s more -- so the rail could
 * be moved, and the display mode changed, under an arm that might still be
 * at speed. See unitlink::peerStoppedOrGone(). */
bool leftStopped(const Telemetry &t) {
  return !t.power && t.state == FanState::Idle && fabsf(t.rpmActual) <= RPM_STOPPED;
}
bool rightStopped() { return unitlink::peerStoppedOrGone(); }

/* Flash writes stall this board's FOC loop (gotcha 6), so settings changed
 * while the arm turns are held in RAM and written to NVS once it has
 * stopped. Only THIS board's arm matters: NVS is on this board's flash. */
volatile bool brightnessDirty = false;
volatile bool calDirty[2] = {false, false};
bool allStopped(const Telemetry &t) { return leftStopped(t) && rightStopped(); }

/* The rail interlock's notion of "stopped" is EVERY fan: both ride the one
 * carriage. */
bool fanStoppedForRail(const Telemetry &t) { return allStopped(t); }

void appendFault(String &out, bool latched, const char *code, const char *message) {
  if (!latched) { out += "null"; return; }
  out += "{\"code\":\"";
  appendEscaped(out, code);
  out += "\",\"message\":\"";
  appendEscaped(out, message);
  out += "\"}";
}

void appendUnit1(String &out, const Telemetry &t) {
  out += "{\"id\":1,\"label\":\"Left\",\"online\":true,\"power\":";
  out += t.power ? "true" : "false";
  out += ",\"state\":\"";
  out += fanStateName(t.state);
  out += "\",\"rpmTarget\":";
  out += (int)lroundf(t.rpmTarget);
  out += ",\"rpmActual\":";
  out += (int)lroundf(t.rpmActual);
  /* The ramp's output: the setpoint the speed loop is actually chasing at
   * this instant (rpmTarget is where the ramp is heading). Unit 1 only; the
   * peer's telemetry packet does not carry it. For the website's Record
   * panel, so a step in rpmTarget can be plotted against the real reference. */
  out += ",\"rpmCommand\":";
  out += (int)lroundf(t.rpmCommand);
  out += ",\"brightness\":";
  out += brightnessLevel;
  out += ",\"fault\":";
  appendFault(out, t.faultLatched, t.faultCode, t.faultMessage);

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

  /* Bring-up instrumentation. focLoopHz answers "is WiFi wrecking my motor
   * control?" and focMaxGapMs is the one that matters -- a since-boot
   * high-water mark that survives even if telemetry gets starved. */
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
  out += ",\"rotorHall\":";
  if (rotor::hallLevel() < 0) out += "null"; else out += rotor::hallLevel();
  /* False while the fan claims to be showing an image means the transfer to
   * the rotor did not land -- the difference between "selected" and "actually
   * on the arm", which was previously invisible from outside. */
  out += ",\"rotorHasContent\":";
  out += rotor::hasContent() ? "true" : "false";
  out += ",\"pushing\":";
  out += rotor::pushActive() ? "true" : "false";
  appendCalibration(out, 1);
  out += "}";
}

/* Unit 2 as relayed by the peer stator. Everything here came over ESP-NOW;
 * when the peer is silent the unit is reported offline and nothing is
 * invented for it. */
void appendUnit2(String &out) {
  const bool on = unitlink::peerOnline();
  out += "{\"id\":2,\"label\":\"Right\",\"online\":";
  out += on ? "true" : "false";
  out += ",\"power\":";
  out += unitlink::peerPower() ? "true" : "false";
  out += ",\"state\":\"";
  out += on ? fanStateName((FanState)unitlink::peerState()) : "offline";
  out += "\",\"rpmTarget\":";
  out += unitlink::peerRpmTarget();
  out += ",\"rpmActual\":";
  out += unitlink::peerRpmActual();
  out += ",\"brightness\":";
  out += unitlink::peerBrightness();
  out += ",\"fault\":";
  appendFault(out, unitlink::peerFault(), unitlink::peerFaultCode(), unitlink::peerFaultMessage());
  out += ",\"rotor\":{\"online\":";
  out += unitlink::peerRotorOnline() ? "true" : "false";
  out += ",\"batteryPct\":null,\"batteryMv\":null,\"rssi\":";
  out += unitlink::peerRotorRssi();
  out += "}";

  /* The peer reports the content id; the name comes from OUR library, which
   * holds the same presets. An id we do not know is shown as itself. */
  out += ",\"content\":";
  const char *cid = unitlink::peerContentId();
  if (on && cid[0]) {
    ContentItem it{};
    const bool known = content::find(cid, it);
    out += "{\"id\":\"";
    appendEscaped(out, cid);
    out += "\",\"name\":\"";
    appendEscaped(out, known ? it.name : cid);
    out += "\"}";
  } else {
    out += "null";
  }
  out += ",\"focLoopHz\":";
  out += unitlink::peerFocLoopHz();
  out += ",\"focMaxGapMs\":";
  out += String(unitlink::peerFocMaxGapMs(), 2);
  out += ",\"voltageQ\":";
  out += String(unitlink::peerVoltageQ(), 2);
  out += ",\"rotorRpm\":";
  out += unitlink::peerRotorRpm();
  out += ",\"rotorSyncLocked\":";
  out += unitlink::peerRotorSyncLocked() ? "true" : "false";
  out += ",\"rotorHall\":";
  if (unitlink::peerRotorHallLevel() < 0) out += "null"; else out += unitlink::peerRotorHallLevel();
  out += ",\"rotorHasContent\":";
  out += unitlink::peerRotorHasContent() ? "true" : "false";
  /* The peer pushes content to its rotor off its own flash, so the master
   * has no byte count to report -- only whether a transfer is running. The
   * website's progress bar for a right-fan selection watches this. */
  out += ",\"pushing\":";
  out += unitlink::peerPushing() ? "true" : "false";
  appendCalibration(out, 2);
  out += "}";
}

String statusJson() {
  const Telemetry t = motorctl::snapshot();

  String out;
  out.reserve(1400);
  out += "{\"display\":{\"mode\":\"";
  out += displayModeName(displayMode);
  out += "\",\"canChangeMode\":";
  out += allStopped(t) ? "true" : "false";
  out += "},\"units\":[";
  appendUnit1(out, t);
  out += ",";
  appendUnit2(out);
  out += "]";

  // --- linear rail ---
  out += ",\"rail\":{\"online\":";
  out += rail::online() ? "true" : "false";
  out += ",\"mode\":\"";
  out += rail::modeName();
  out += "\",\"state\":\"";
  out += rail::stateName();
  out += "\",\"homed\":";
  out += rail::homed() ? "true" : "false";
  out += ",\"moving\":";
  out += rail::moving() ? "true" : "false";
  out += ",\"sweeping\":";
  out += rail::sweeping() ? "true" : "false";
  out += ",\"positionMm\":";
  out += String(rail::positionMm(), 1);
  out += ",\"targetMm\":";
  out += String(rail::targetMm(), 1);
  out += ",\"travelMm\":";
  out += String(rail::travelMm(), 1);
  out += ",\"travelMeasured\":";
  out += rail::travelMeasured() ? "true" : "false";
  out += ",\"limitHome\":";
  out += rail::limitHome() ? "true" : "false";
  out += ",\"limitIdle\":";
  out += rail::limitIdle() ? "true" : "false";
  out += ",\"direction\":";
  out += rail::direction();
  out += ",\"canMove\":";
  out += (rail::mode() == rail::Mode::Sweep || fanStoppedForRail(t)) ? "true" : "false";
  out += ",\"fault\":";
  if (rail::fault() != RAIL_FAULT_NONE) {
    out += "{\"code\":";
    out += rail::fault();
    out += ",\"message\":\"";
    appendEscaped(out, rail::faultName());
    out += "\"}";
  } else {
    out += "null";
  }
  /* The rail's verdict on the last command, which the 200 from /api/rail/*
   * cannot carry (it is sent before the rail has seen the command). */
  out += ",\"refused\":";
  if (rail::refusedName()[0]) {
    out += "\"";
    appendEscaped(out, rail::refusedName());
    out += "\"";
  } else {
    out += "null";
  }
  out += ",\"sweep\":{\"minMm\":";
  out += String(rail::sweepMinMm(), 1);
  out += ",\"maxMm\":";
  out += String(rail::sweepMaxMm(), 1);
  out += ",\"speedMms\":";
  out += String(rail::sweepSpeedMms(), 1);
  out += ",\"dwellMs\":";
  out += rail::sweepDwellMs();
  out += "}}";

  /* What the master is doing for the two-fan scenes: sending loop pulses
   * (pong, penalty) or rail-cycle restarts (flappy), and whether the content
   * is currently played mirrored (fish while the rail moves left). */
  out += ",\"scene\":{\"sync\":";
  out += scene::syncing() ? "true" : "false";
  out += ",\"mirror\":";
  out += scene::mirrored() ? "true" : "false";
  out += "}";

  out += ",\"freeHeap\":";
  out += (uint32_t)ESP.getFreeHeap();
  out += ",\"fsFreeBytes\":";
  out += (uint32_t)content::freeBytes();
  out += "}";
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
  bool on = false;
  if (!jsonBool(bodyOf(req), "on", on)) {
    sendError(req, 400, "That request did not say whether to start or stop.");
    return;
  }
  if (on) {
    if (leftActive() && t.faultLatched) {
      sendError(req, 409, "The left fan has stopped because of a fault. Clear it before starting.");
      return;
    }
    if (rightActive() && !unitlink::peerOnline()) {
      sendError(req, 409, "The right fan is not connected.");
      return;
    }
    if (rightActive() && unitlink::peerFault()) {
      sendError(req, 409, "The right fan has stopped because of a fault. Clear it before starting.");
      return;
    }
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
  /* Position mode: the carriage must not be moving when the arm starts. In
   * sweep mode it is allowed to, that is the mode's purpose. */
  if (on && rightActive() && filesync::busy()) {
    sendError(req, 409,
              "An image is still being copied to the right fan. Try again in a few seconds.");
    return;
  }
  if (on && rail::blocksFanStart()) {
    sendError(req, 409, "The rail is still moving. Wait for it to stop before starting the fan.");
    return;
  }
  /* Both mode: both fans get THIS board's setpoint first, so they spin up to
   * the same speed together -- that is the "synchronous" part.
   *
   * Right mode: the right fan keeps the speed handleSpeed() gave it. It used
   * to be sent t.rpmTarget here too, which in Right mode is the LEFT motor's
   * target (handleSpeed never touches it in that mode), so the right fan
   * started at whatever the left last ran at and the slider snapped to it.
   * A target of 0 (slider dragged to the stop, then Start pressed) is lifted
   * to the floor, mirroring what the motor task does for this board. */
  if (leftActive()) motorctl::cmdPower(on);
  if (rightActive()) {
    if (on) {
      const float want = displayMode == DisplayMode::Both ? t.rpmTarget
                                                           : (float)unitlink::peerRpmCommanded();
      unitlink::peerSetRpm(fmaxf(want, RPM_MIN_ALLOWED));
    }
    unitlink::peerPower(on);
  }
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
  if (leftActive()) {
    if (clamped <= 0.0f) {
      motorctl::cmdSpeed(0.0f);
      motorctl::cmdPower(false);
    } else {
      motorctl::cmdSpeed(clamped);
    }
  }
  if (rightActive()) {
    unitlink::peerSetRpm(clamped);
    if (clamped <= 0.0f) unitlink::peerPower(false);
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
  const int lvl = constrain((int)lroundf(level), BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  if (leftActive()) {
    brightnessLevel = lvl;
    brightnessDirty = true;  // saved by the telemetry task once the arm stops
  }
  if (rightActive()) unitlink::peerSetBrightness(lvl);

  String body = "{\"ok\":true,\"level\":";
  body += lvl;
  body += "}";
  sendJson(req, 200, body);
}

void handleStop(AsyncWebServerRequest *req) {
  // Never scoped by the display mode: both fans and the rail, always.
  motorctl::cmdStop();
  unitlink::peerEstop();
  rail::emergencyStop();  // halts the carriage and drops to position mode
  sendOk(req);
}

void handleDisplayMode(AsyncWebServerRequest *req) {
  char mode[12] = {0};
  if (!jsonString(bodyOf(req), "mode", mode, sizeof(mode))) {
    sendError(req, 400, "That request did not say which fans to use.");
    return;
  }
  DisplayMode want;
  if (strcmp(mode, "left") == 0) want = DisplayMode::Left;
  else if (strcmp(mode, "right") == 0) want = DisplayMode::Right;
  else if (strcmp(mode, "both") == 0) want = DisplayMode::Both;
  else { sendError(req, 400, "Unknown display mode."); return; }

  if (want == displayMode) { sendOk(req); return; }
  const Telemetry t = motorctl::snapshot();
  if (!leftStopped(t) || (unitlink::peerOnline() && !rightStopped())) {
    sendError(req, 409, "Stop the fans before changing which ones are used.");
    return;
  }
  if (!rightStopped()) {
    // The right fan is offline and not yet known to have stopped.
    sendError(req, 409,
              "The right fan has lost contact and may still be spinning down. "
              "Wait half a minute, then try again.");
    return;
  }
  if (want != DisplayMode::Left && !unitlink::peerOnline()) {
    sendError(req, 409, "The right fan is not connected.");
    return;
  }
  displayMode = want;
  webPrefs.putUChar("dmode", (uint8_t)displayMode);
  netlog::printf("[WEB] display mode -> %s", displayModeName(displayMode));
  sendOk(req);
}

/* POST /api/calibrate {"unit":1|2, "delta":+-n} or {"offset":n} and/or
 * {"reverse":bool}. Applied at once (the rotor turns the image on the next
 * command) and saved to NVS, so the number shown is always the live one. */
void handleCalibrate(AsyncWebServerRequest *req) {
  const char *b = bodyOf(req);
  float which = 1;
  jsonNumber(b, "unit", which);
  const int unit = (which == 2) ? 2 : 1;
  Cal &c = cal[unit - 1];

  float v = 0;
  if (jsonNumber(b, "offset", v)) c.offset = (int)lroundf(v);
  if (jsonNumber(b, "delta", v)) c.offset += (int)lroundf(v);
  bool rev = false;
  if (jsonBool(b, "reverse", rev)) c.reverse = rev;
  c.offset = ((c.offset % 180) + 180) % 180;

  /* Applied at once, SAVED later: this page is meant to be used with the arms
   * turning, and an NVS write here could stall the FOC loop (gotcha 6). */
  calDirty[unit - 1] = true;
  applyCalibration();
  netlog::printf("[CAL] unit %d: offset %d sectors (%d deg), rotation %s", unit, c.offset,
                 c.offset * 2, c.reverse ? "reversed" : "normal");

  String body = "{\"ok\":true,\"unit\":";
  body += unit;
  body += ",\"offset\":";
  body += c.offset;
  body += ",\"reverse\":";
  body += c.reverse ? "true" : "false";
  body += "}";
  sendJson(req, 200, body);
}

/* GET /api/log?since=N -- the network log, newest lines last. */
void handleLog(AsyncWebServerRequest *req) {
  uint32_t since = 0;
  if (req->hasParam("since")) since = (uint32_t)req->getParam("since")->value().toInt();
  String body;
  body.reserve(4096);
  netlog::toJson(body, since);
  sendJson(req, 200, body);
}

// ---------------------------------------------------------------------------
//  Linear rail
// ---------------------------------------------------------------------------

void railReply(AsyncWebServerRequest *req, const char *refused) {
  if (refused) sendError(req, 409, refused);
  else sendOk(req);
}

void handleRailMode(AsyncWebServerRequest *req) {
  char mode[16] = {0};
  if (!jsonString(bodyOf(req), "mode", mode, sizeof(mode))) {
    sendError(req, 400, "That request did not say which rail mode to use.");
    return;
  }
  const Telemetry t = motorctl::snapshot();
  if (strcmp(mode, "static") == 0 || strcmp(mode, "position") == 0) {
    railReply(req, rail::setMode(rail::Mode::Static, fanStoppedForRail(t)));
  } else if (strcmp(mode, "sweep") == 0) {
    railReply(req, rail::setMode(rail::Mode::Sweep, fanStoppedForRail(t)));
  } else {
    sendError(req, 400, "Unknown rail mode.");
  }
}

void handleRailHome(AsyncWebServerRequest *req) {
  const Telemetry t = motorctl::snapshot();
  railReply(req, rail::home(fanStoppedForRail(t)));
}

void handleRailGoto(AsyncWebServerRequest *req) {
  float mm = 0;
  if (!jsonNumber(bodyOf(req), "mm", mm)) {
    sendError(req, 400, "That request did not include a position.");
    return;
  }
  const Telemetry t = motorctl::snapshot();
  railReply(req, rail::gotoMm(mm, fanStoppedForRail(t)));
}

void handleRailSweep(AsyncWebServerRequest *req) {
  bool on = false;
  if (!jsonBool(bodyOf(req), "on", on)) {
    sendError(req, 400, "That request did not say whether to start or stop the sweep.");
    return;
  }
  if (!on) {
    railReply(req, rail::stopSweep());
    return;
  }
  float minMm = 0, maxMm = 0, speed = 0, dwell = 0;
  const char *b = bodyOf(req);
  if (!jsonNumber(b, "minMm", minMm) || !jsonNumber(b, "maxMm", maxMm)) {
    sendError(req, 400, "A sweep needs a start and end position.");
    return;
  }
  jsonNumber(b, "speedMms", speed);
  jsonNumber(b, "dwellMs", dwell);
  railReply(req, rail::startSweep(minMm, maxMm, speed, (uint32_t)fmaxf(0, dwell)));
}

void handleRailStop(AsyncWebServerRequest *req) {
  rail::stop();
  sendOk(req);
}

void handleRailClear(AsyncWebServerRequest *req) {
  railReply(req, rail::clearFault());
}

void handleFaultClear(AsyncWebServerRequest *req) {
  /* {"unit":1|2} clears one; no unit clears whichever are latched. A refusal
   * on either side wins, so the user sees why. */
  float which = 0;
  jsonNumber(bodyOf(req), "unit", which);
  const bool doLeft = which == 0 || which == 1;
  const bool doRight = which == 0 || which == 2;

  const Telemetry t = motorctl::snapshot();
  if (doLeft && t.faultLatched) {
    if (t.faultCauseActive) {
      // API.md: refuse while the cause is still present, and say why.
      sendError(req, 409, t.clearBlockedReason);
      return;
    }
    motorctl::cmdClearFault();
  }
  if (doRight && unitlink::peerFault()) {
    if (unitlink::peerFaultCauseActive()) {
      sendError(req, 409, "The right fan is still spinning down. Wait until the arm has stopped, then try again.");
      return;
    }
    unitlink::peerClearFault();
  }
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
  /* target: "left" | "right" | "both"; absent = every active fan. */
  char target[8] = {0};
  jsonString(bodyOf(req), "target", target, sizeof(target));
  bool toLeft = leftActive(), toRight = rightActive();
  if (strcmp(target, "left") == 0) { toLeft = true; toRight = false; }
  else if (strcmp(target, "right") == 0) { toLeft = false; toRight = true; }
  else if (strcmp(target, "both") == 0) { toLeft = true; toRight = true; }

  if (toRight && !unitlink::peerOnline()) {
    sendError(req, 409, "The right fan is not connected.");
    return;
  }
  /* Uploads exist on the right fan only once file_sync has copied them to
   * the peer's flash. Until then refuse with the reason, and move this file
   * to the front of the copy queue. */
  if (toRight && !it.preset) {
    const filesync::State st = filesync::state(it.id);
    if (st != filesync::State::Synced) {
      filesync::prioritise(it.id);
      const char *why = filesync::lastError(it.id);
      if (st == filesync::State::Pending && strcmp(why, "the right fan is turning") == 0) {
        sendError(req, 409,
                  "That image has not been copied to the right fan yet, and it can only be "
                  "copied while the right fan is stopped. Stop it, then try again.");
      } else if (st == filesync::State::Failed) {
        String msg = "Copying that image to the right fan failed (";
        msg += why;
        msg += "). It will try again by itself in a few seconds.";
        sendError(req, 409, msg.c_str());
      } else {
        sendError(req, 409,
                  "That image is still being copied to the right fan. Try again in a few seconds.");
      }
      return;
    }
  }
  if (toLeft) {
    content::select(id);
    rotor::startPush(it.id, it.bytes);
  }
  if (toRight) unitlink::peerSelectContent(id);
  sendOk(req);
}

void handleDelete(AsyncWebServerRequest *req) {
  /* Deleting rewrites LittleFS metadata -- a flash erase -- so it gets the
   * upload's guard: this board's arm must be stopped (gotcha 6). */
  const Telemetry mt = motorctl::snapshot();
  if (mt.power || fabsf(mt.rpmActual) > RPM_STOPPED) {
    sendError(req, 409,
              "The fan has to be stopped before an image can be deleted. Turn it "
              "off, wait for the arm to stop, then try again.");
    return;
  }
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
  filesync::onDeleted(id.c_str());  // and the right fan's copy
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
    rotor::startPush(item.id, item.bytes);
    filesync::onUploaded(item.id);  // copy it to the right fan's flash too

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
    unitlink::tick();
    rail::tick(fanStoppedForRail(t), t.faultLatched || unitlink::peerFault());
    scene::tick();
    content::uploadWatchdog();
    filesync::tick();

    /* Settings changed while the arm turned are written now it has stopped. */
    if (leftStopped(t)) {
      content::flushPrefs();
      if (brightnessDirty) {
        brightnessDirty = false;
        webPrefs.putInt("bright", brightnessLevel);
      }
      for (int u = 1; u <= 2; u++) {
        if (calDirty[u - 1]) {
          calDirty[u - 1] = false;
          saveCalibration(u);
        }
      }
    }

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
     * point of measuring it.
     *
     * And never past a scene timing point: loop pulses leave on time, not on
     * the next 50 ms pass. Minimum 1 tick so this task always yields. */
    uint32_t sleepMs = rotor::pushActive() ? 2 : 50;
    const uint32_t sceneDue = scene::msUntilDue();
    if (sceneDue < sleepMs) sleepMs = sceneDue;
    vTaskDelay(max<TickType_t>(1, pdMS_TO_TICKS(sleepMs)));
  }
}

}  // namespace

namespace web {

int brightness() { return brightnessLevel; }

void setBrightnessLevel(int level) {
  const int clamped = constrain(level, BRIGHTNESS_MIN, BRIGHTNESS_MAX);
  if (clamped == brightnessLevel) return;
  brightnessLevel = clamped;
  // The peer never opens the preferences namespace (web::begin() is master
  // only); the master resends brightness every packet anyway. Saved by the
  // telemetry task once the arm has stopped, never while it turns.
  if (unitlink::isMaster()) brightnessDirty = true;
}

int stationCount() { return WiFi.softAPgetStationNum(); }

bool begin() {
  webPrefs.begin("web", false);
  brightnessLevel = webPrefs.getInt("bright", 24);
  displayMode = (DisplayMode)constrain(webPrefs.getUChar("dmode", 0), 0, 2);
  cal[0] = {webPrefs.getInt("c1off", 0), webPrefs.getBool("c1rev", false)};
  cal[1] = {webPrefs.getInt("c2off", 0), webPrefs.getBool("c2rev", false)};
  Serial.printf("[WEB] calibration: unit 1 offset %d%s, unit 2 offset %d%s\n",
                cal[0].offset, cal[0].reverse ? " reversed" : "",
                cal[1].offset, cal[1].reverse ? " reversed" : "");

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
  applyCalibration();

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
  server.on("/api/fault/clear", HTTP_POST, handleFaultClear, nullptr, collectBody);
  server.on("/api/display/mode", HTTP_POST, handleDisplayMode, nullptr, collectBody);
  server.on("/api/calibrate", HTTP_POST, handleCalibrate, nullptr, collectBody);
  server.on("/api/log", HTTP_GET, handleLog);

  server.on("/api/rail/mode", HTTP_POST, handleRailMode, nullptr, collectBody);
  server.on("/api/rail/home", HTTP_POST, handleRailHome);
  server.on("/api/rail/goto", HTTP_POST, handleRailGoto, nullptr, collectBody);
  server.on("/api/rail/sweep", HTTP_POST, handleRailSweep, nullptr, collectBody);
  server.on("/api/rail/stop", HTTP_POST, handleRailStop);
  server.on("/api/rail/clear", HTTP_POST, handleRailClear);
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

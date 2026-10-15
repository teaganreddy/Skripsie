#include "netlog.h"

#include <stdarg.h>

#include "unit_link.h"

namespace {

const int RING = 48;  // lines kept on the master; ~8 KB

struct Line {
  uint32_t seq;
  uint32_t ms;
  char tag[10];
  char text[sizeof(LogMsg::text)];
};

Line ring[RING];
int head = 0, count = 0;
uint32_t nextSeq = 1;
/* Random per boot. The website restarts its view of the log whenever this
 * changes, because sequence numbers start again from 1 after a restart. */
uint32_t bootId = 0;
SemaphoreHandle_t mutex = nullptr;

// Peer: outbound queue to the master.
const int QUEUE = 8;
LogMsg queued[QUEUE];
int qHead = 0, qCount = 0;

void appendEscaped(String &out, const char *s) {
  for (const char *p = s; *p; ++p) {
    if (*p == '"' || *p == '\\') { out += '\\'; out += *p; }
    else if ((uint8_t)*p < 0x20) out += ' ';
    else out += *p;
  }
}

/* Master: file a line. Peer: queue it for the master. Neither blocks. */
void file(const char *tag, const char *text, uint8_t unitId, uint8_t source) {
  if (!mutex || xSemaphoreTake(mutex, 0) != pdTRUE) return;
  if (unitlink::isMaster()) {
    if (count == RING) { head = (head + 1) % RING; count--; }
    Line &l = ring[(head + count) % RING];
    l.seq = nextSeq++;
    l.ms = millis();
    snprintf(l.tag, sizeof(l.tag), "%s", tag);
    snprintf(l.text, sizeof(l.text), "%s", text);
    count++;
  } else {
    if (qCount == QUEUE) { qHead = (qHead + 1) % QUEUE; qCount--; }
    LogMsg &m = queued[(qHead + qCount) % QUEUE];
    m.type = MSG_LOG;
    m.version = PROTOCOL_VERSION;
    m.unitId = unitId;
    m.source = source;
    snprintf(m.text, sizeof(m.text), "%s", text);
    qCount++;
  }
  xSemaphoreGive(mutex);
}

}  // namespace

namespace netlog {

void begin() { mutex = xSemaphoreCreateMutex(); }

void printf(const char *fmt, ...) {
  char buf[sizeof(LogMsg::text)];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  /* Only if it will not block. This is called from the ESP-NOW receive
   * callback (pairing, content verdicts), which must return promptly
   * (gotcha 29), and a USB host that is attached but not reading makes a
   * plain println wait out its timeout. The line still reaches the website
   * log below; only the serial echo is skipped. Same guard as onPacket(). */
  if (Serial.availableForWrite() >= (int)strlen(buf) + 2) Serial.println(buf);
  file(unitlink::isMaster() ? "U1" : "U2", buf, UNIT_ID, LOG_SRC_STATOR);
}

void onPacket(const esp_now_recv_info_t *, const uint8_t *data, int len) {
  if (len < (int)sizeof(LogMsg)) return;
  LogMsg m;
  memcpy(&m, data, sizeof(m));
  m.text[sizeof(m.text) - 1] = '\0';
  char tag[10];
  snprintf(tag, sizeof(tag), "U%u%s", m.unitId, m.source == LOG_SRC_ROTOR ? " rotor" : "");
  /* Echo to this board's serial too, so a bench session still sees it -- but
   * only if it will not block: this runs in the ESP-NOW receive callback,
   * which must return promptly (CLAUDE.md gotcha 29). */
  if (Serial.availableForWrite() >= (int)strlen(m.text) + 16) {
    Serial.printf("[%s] %s\n", tag, m.text);
  }
  file(tag, m.text, m.unitId, m.source);
}

void tick(const uint8_t *masterMac, bool haveMaster) {
  if (unitlink::isMaster() || !haveMaster || !mutex) return;
  LogMsg m;
  bool have = false;
  if (xSemaphoreTake(mutex, 0) == pdTRUE) {
    if (qCount > 0) {
      m = queued[qHead];
      qHead = (qHead + 1) % QUEUE;
      qCount--;
      have = true;
    }
    xSemaphoreGive(mutex);
  }
  if (have) esp_now_send(masterMac, (const uint8_t *)&m, sizeof(m));
}

void toJson(String &out, uint32_t since) {
  /* Chosen on first use, after WiFi is up: the hardware RNG only has real
   * entropy once the radio runs, and this must differ between boots. */
  if (!bootId) bootId = esp_random() | 1;
  out += "{\"boot\":";
  out += bootId;
  out += ",\"next\":";
  out += nextSeq;
  out += ",\"lines\":[";
  if (mutex && xSemaphoreTake(mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    bool first = true;
    for (int i = 0; i < count; i++) {
      const Line &l = ring[(head + i) % RING];
      if (l.seq < since) continue;
      if (!first) out += ',';
      first = false;
      out += "{\"seq\":";
      out += l.seq;
      out += ",\"t\":";
      out += l.ms;
      out += ",\"tag\":\"";
      appendEscaped(out, l.tag);
      out += "\",\"text\":\"";
      appendEscaped(out, l.text);
      out += "\"}";
    }
    xSemaphoreGive(mutex);
  }
  out += "]}";
}

}  // namespace netlog

#include "file_sync.h"

#include <LittleFS.h>
#include <esp_now.h>

#include "config.h"
#include "content.h"
#include "link_protocol.h"
#include "motor_task.h"
#include "netlog.h"
#include "unit_link.h"

namespace {

using filesync::State;

const uint32_t ACK_TIMEOUT_MS = 600;   // a chunk is a radio hop plus a flash write
const int MAX_RESENDS = 6;
const uint32_t RETRY_AFTER_FAIL_MS = 10000;
const uint32_t RETRY_WAIT_STOPPED_MS = 2000;
const size_t MAX_ENTRIES = 40;         // content.cpp's library size
const int MAX_DELETES = 8;

size_t chunkBytes = 0;  // decided from the ESP-NOW version on first use

void userPath(const char *id, char *out, size_t len) {
  snprintf(out, len, "%s/%s.povf", USER_DIR, id);
}

/* Sum of every byte of a file, the checksum both ends agree on. Read in small
 * blocks: this runs on tasks with 4-6 KB of stack. */
bool fileSum(const char *path, uint32_t &size, uint32_t &sum) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  static uint8_t buf[512];
  size = f.size();
  sum = 0;
  for (;;) {
    const int n = f.read(buf, sizeof(buf));
    if (n <= 0) break;
    for (int i = 0; i < n; i++) sum += buf[i];
  }
  f.close();
  return true;
}

bool armStopped() {
  const Telemetry t = motorctl::snapshot();
  return !t.power && t.state == FanState::Idle && fabsf(t.rpmActual) <= RPM_STOPPED;
}

// ===========================================================================
//  MASTER
// ===========================================================================

struct Entry {
  char id[24];
  State st;
  bool waitStopped;  // the peer said it is spinning; retry once it stops
  uint8_t failures;
  uint32_t retryAtMs;
  char err[64];
};
Entry table[MAX_ENTRIES];
size_t tableCount = 0;
char priorityId[24] = {0};

struct PendingDelete {
  char id[24];
  bool waitStopped;
  uint32_t retryAtMs;
};
PendingDelete deletes[MAX_DELETES];
int deleteCount = 0;

/* table/deletes are read and changed by the web handlers (AsyncTCP) and by
 * tick() (telemetry task). Short critical sections only: no flash or radio
 * work is ever done while holding this. */
portMUX_TYPE tableMux = portMUX_INITIALIZER_UNLOCKED;

enum class Phase : uint8_t { None, Begin, Chunk, End, Delete };
struct Job {
  Phase phase = Phase::None;
  bool isDelete = false;
  char id[24] = {0};
  uint32_t transferId = 0;
  File file;
  uint32_t total = 0;
  uint32_t checksum = 0;
  uint32_t offset = 0;      // of the chunk last sent
  uint32_t sentAtMs = 0;
  uint32_t startedMs = 0;
  int resends = 0;
  bool wroteAny = false;    // the peer did not already have it
} job;
uint32_t nextTransferId = 0;

uint8_t txBuf[sizeof(FileChunkMsg) + 1470];

// Ack mailbox: written by the receive callback, taken by tick().
FileAckMsg ackMail;
volatile bool ackFull = false;
portMUX_TYPE ackMux = portMUX_INITIALIZER_UNLOCKED;

int findEntry(const char *id) {  // caller holds tableMux
  for (size_t i = 0; i < tableCount; i++) {
    if (strcmp(table[i].id, id) == 0) return (int)i;
  }
  return -1;
}

void setEntry(const char *id, State st, const char *err = nullptr, bool waitStopped = false) {
  portENTER_CRITICAL(&tableMux);
  int i = findEntry(id);
  if (i < 0 && tableCount < MAX_ENTRIES) {
    i = (int)tableCount++;
    memset(&table[i], 0, sizeof(Entry));
    snprintf(table[i].id, sizeof(table[i].id), "%s", id);
  }
  if (i >= 0) {
    Entry &e = table[i];
    e.st = st;
    e.waitStopped = waitStopped;
    if (st == State::Failed) {
      e.failures++;
      e.retryAtMs = millis() + RETRY_AFTER_FAIL_MS * (e.failures > 5 ? 6 : 1);
    } else if (st == State::Pending && waitStopped) {
      e.retryAtMs = millis() + RETRY_WAIT_STOPPED_MS;
    } else {
      e.retryAtMs = 0;
    }
    if (st == State::Synced) e.failures = 0;
    snprintf(e.err, sizeof(e.err), "%s", err ? err : "");
  }
  portEXIT_CRITICAL(&tableMux);
}

void removeEntry(const char *id) {
  portENTER_CRITICAL(&tableMux);
  const int i = findEntry(id);
  if (i >= 0) {
    table[i] = table[tableCount - 1];
    tableCount--;
  }
  portEXIT_CRITICAL(&tableMux);
}

bool sendToPeer(const void *data, size_t len) {
  if (!unitlink::linkKnown()) return false;
  return esp_now_send(unitlink::linkMac(), (const uint8_t *)data, len) == ESP_OK;
}

/* (Re)send whatever the job is waiting on. Used for the first send and for
 * every resend after a timeout, so a lost packet in either direction heals. */
void sendCurrent() {
  job.sentAtMs = millis();
  switch (job.phase) {
    case Phase::Begin: {
      FileBeginMsg m{};
      m.type = MSG_FILE_BEGIN;
      m.version = PROTOCOL_VERSION;
      m.transferId = job.transferId;
      m.totalBytes = job.total;
      m.checksum = job.checksum;
      snprintf(m.id, sizeof(m.id), "%s", job.id);
      ContentItem it{};
      if (content::find(job.id, it)) snprintf(m.name, sizeof(m.name), "%s", it.name);
      sendToPeer(&m, sizeof(m));
      break;
    }
    case Phase::Chunk: {
      const size_t want = min((size_t)(job.total - job.offset), chunkBytes);
      FileChunkMsg h{};
      h.type = MSG_FILE_CHUNK;
      h.version = PROTOCOL_VERSION;
      h.dataLen = (uint16_t)want;
      h.transferId = job.transferId;
      h.offset = job.offset;
      memcpy(txBuf, &h, sizeof(h));
      job.file.seek(job.offset);
      if (job.file.read(txBuf + sizeof(h), want) != (int)want) {
        netlog::printf("[SYNC] \"%s\": could not read it back from flash", job.id);
        job.phase = Phase::None;  // finishJob() below reports it
        return;
      }
      sendToPeer(txBuf, sizeof(h) + want);
      break;
    }
    case Phase::End: {
      FileEndMsg m{};
      m.type = MSG_FILE_END;
      m.version = PROTOCOL_VERSION;
      m.transferId = job.transferId;
      m.totalBytes = job.total;
      m.checksum = job.checksum;
      sendToPeer(&m, sizeof(m));
      break;
    }
    case Phase::Delete: {
      FileDeleteMsg m{};
      m.type = MSG_FILE_DELETE;
      m.version = PROTOCOL_VERSION;
      m.transferId = job.transferId;
      snprintf(m.id, sizeof(m.id), "%s", job.id);
      sendToPeer(&m, sizeof(m));
      break;
    }
    case Phase::None:
      break;
  }
}

void endJob() {
  if (job.file) job.file.close();
  job.phase = Phase::None;
  job.isDelete = false;
}

void popDelete(const char *id) {
  portENTER_CRITICAL(&tableMux);
  for (int i = 0; i < deleteCount; i++) {
    if (strcmp(deletes[i].id, id) == 0) {
      deletes[i] = deletes[deleteCount - 1];
      deleteCount--;
      break;
    }
  }
  portEXIT_CRITICAL(&tableMux);
}

void deleteWaitStopped(const char *id) {
  portENTER_CRITICAL(&tableMux);
  for (int i = 0; i < deleteCount; i++) {
    if (strcmp(deletes[i].id, id) == 0) {
      deletes[i].waitStopped = true;
      deletes[i].retryAtMs = millis() + RETRY_WAIT_STOPPED_MS;
    }
  }
  portEXIT_CRITICAL(&tableMux);
}

const char *statusText(uint8_t s) {
  switch (s) {
    case FILE_ERR_SPINNING: return "the right fan is turning";
    case FILE_ERR_SPACE: return "the right fan has no room for it";
    case FILE_ERR_INVALID: return "the right fan rejected the file";
    case FILE_ERR_CHECKSUM: return "it arrived damaged";
    case FILE_ERR_IO: return "the right fan could not save it";
    default: return "unknown error";
  }
}

/* Abandon the job without a verdict (link lost): back to Pending so it is
 * picked up again once the peer is back. */
void abortJob() {
  if (job.phase == Phase::None) return;
  if (!job.isDelete) setEntry(job.id, State::Pending);
  endJob();
}

bool startCopy(const char *id) {
  ContentItem it{};
  if (!content::find(id, it) || it.preset) {  // deleted meanwhile, or not ours to copy
    removeEntry(id);
    return false;
  }
  char path[64];
  userPath(id, path, sizeof(path));
  uint32_t size = 0, sum = 0;
  if (!fileSum(path, size, sum)) {
    setEntry(id, State::Failed, "it could not be read from the left fan's flash");
    return false;
  }
  job = Job{};
  job.file = LittleFS.open(path, "r");
  if (!job.file) {
    setEntry(id, State::Failed, "it could not be read from the left fan's flash");
    return false;
  }
  snprintf(job.id, sizeof(job.id), "%s", id);
  job.transferId = ++nextTransferId;
  job.total = size;
  job.checksum = sum;
  job.startedMs = millis();
  job.phase = Phase::Begin;
  setEntry(id, State::Copying);
  sendCurrent();
  return true;
}

void startDelete(const char *id) {
  job = Job{};
  job.isDelete = true;
  snprintf(job.id, sizeof(job.id), "%s", id);
  job.transferId = ++nextTransferId;
  job.phase = Phase::Delete;
  sendCurrent();
}

/* Pick the next thing to do: deletes first (they free room), then the file
 * the user just asked for, then anything unknown or pending. */
void startNextJob() {
  const uint32_t now = millis();
  const bool peerStopped = unitlink::peerStoppedOrGone();

  char id[24] = {0};
  bool isDelete = false;
  portENTER_CRITICAL(&tableMux);
  for (int i = 0; i < deleteCount && !id[0]; i++) {
    if ((int32_t)(now - deletes[i].retryAtMs) < 0) continue;
    if (deletes[i].waitStopped && !peerStopped) continue;
    snprintf(id, sizeof(id), "%s", deletes[i].id);
    isDelete = true;
  }
  if (!id[0] && priorityId[0]) {
    const int i = findEntry(priorityId);
    if (i >= 0 && table[i].st != State::Synced && table[i].st != State::Copying &&
        (!table[i].waitStopped || peerStopped)) {
      snprintf(id, sizeof(id), "%s", priorityId);
    }
    priorityId[0] = '\0';
  }
  for (size_t i = 0; i < tableCount && !id[0]; i++) {
    const Entry &e = table[i];
    if (e.st != State::Unknown && e.st != State::Pending && e.st != State::Failed) continue;
    if ((int32_t)(now - e.retryAtMs) < 0) continue;
    if (e.waitStopped && !peerStopped) continue;
    snprintf(id, sizeof(id), "%s", e.id);
  }
  portEXIT_CRITICAL(&tableMux);

  if (!id[0]) return;
  if (isDelete) startDelete(id);
  else startCopy(id);
}

void pumpJob(const FileAckMsg *a) {
  if (a && a->transferId == job.transferId) {
    job.resends = 0;
    switch (a->status) {
      case FILE_OK:
        if (!job.isDelete) {
          setEntry(job.id, State::Synced);
          if (job.wroteAny) {
            netlog::printf("[SYNC] \"%s\" copied to the right fan: %lu bytes in %lu ms", job.id,
                           (unsigned long)job.total, (unsigned long)(millis() - job.startedMs));
          }
        }
        endJob();
        return;
      case FILE_DELETED:
        if (job.isDelete) {
          netlog::printf("[SYNC] \"%s\" deleted from the right fan", job.id);
          popDelete(job.id);
        }
        endJob();
        return;
      case FILE_IN_PROGRESS: {
        if (job.isDelete) return;
        job.wroteAny = true;
        if (job.phase == Phase::Begin) {
          netlog::printf("[SYNC] copying \"%s\" to the right fan (%lu bytes)", job.id,
                         (unsigned long)job.total);
        }
        job.offset = a->bytesReceived;
        job.phase = (job.offset >= job.total) ? Phase::End : Phase::Chunk;
        sendCurrent();
        if (job.phase == Phase::None) {  // read failure inside sendCurrent()
          setEntry(job.id, State::Failed, "it could not be read from the left fan's flash");
          endJob();
        }
        return;
      }
      case FILE_ERR_SPINNING:
        if (job.isDelete) {
          deleteWaitStopped(job.id);
        } else {
          setEntry(job.id, State::Pending, "the right fan is turning", true);
          netlog::printf("[SYNC] \"%s\": the right fan is turning, will copy when it stops",
                         job.id);
        }
        endJob();
        return;
      default:
        if (job.isDelete) {
          deleteWaitStopped(job.id);  // try again later; nothing better to do
        } else {
          setEntry(job.id, State::Failed, statusText(a->status));
          netlog::printf("[SYNC] \"%s\" could not be copied: %s", job.id, statusText(a->status));
        }
        endJob();
        return;
    }
  }

  if (millis() - job.sentAtMs < ACK_TIMEOUT_MS) return;
  if (++job.resends > MAX_RESENDS) {
    if (job.isDelete) {
      deleteWaitStopped(job.id);
    } else {
      setEntry(job.id, State::Failed, "the right fan stopped answering");
      netlog::printf("[SYNC] \"%s\": the right fan stopped answering", job.id);
    }
    endJob();
    return;
  }
  sendCurrent();
}

void masterTick() {
  if (!chunkBytes) {
    uint32_t v = 1;
    esp_now_get_version(&v);
    chunkBytes = (v >= 2) ? 1400 : 200;
  }

  /* Re-check everything whenever the peer link comes back: the peer may have
   * been reflashed (uploadfs wipes its copies) or rebooted mid-copy. A file it
   * still holds costs one BEGIN and a FILE_OK, no transfer. */
  static bool wasOnline = false;
  const bool online = unitlink::peerOnline();
  if (online && !wasOnline) {
    portENTER_CRITICAL(&tableMux);
    for (size_t i = 0; i < tableCount; i++) {
      if (table[i].st == State::Synced || table[i].st == State::Failed) {
        table[i].st = State::Unknown;
        table[i].failures = 0;
        table[i].retryAtMs = 0;
      }
    }
    portEXIT_CRITICAL(&tableMux);
  }
  wasOnline = online;
  if (!online) {
    abortJob();
    return;
  }

  FileAckMsg a;
  bool have = false;
  portENTER_CRITICAL(&ackMux);
  if (ackFull) {
    a = ackMail;
    ackFull = false;
    have = true;
  }
  portEXIT_CRITICAL(&ackMux);

  if (job.phase != Phase::None) {
    pumpJob(have ? &a : nullptr);
    return;
  }
  startNextJob();
}

// ===========================================================================
//  PEER
// ===========================================================================

uint8_t mail[sizeof(FileChunkMsg) + 1470];
int mailLen = 0;
volatile bool mailFull = false;
portMUX_TYPE mailMux = portMUX_INITIALIZER_UNLOCKED;

struct {
  bool active = false;
  uint32_t transferId = 0;
  char id[24] = {0};
  uint32_t total = 0;
  uint32_t received = 0;
  uint32_t sum = 0;
  uint32_t expectSum = 0;
  uint32_t lastDoneTransferId = 0;  // to re-ack an END whose OK was lost
} rx;

void ack(uint8_t status, uint32_t transferId, uint32_t bytes, const char *id) {
  FileAckMsg m{};
  m.type = MSG_FILE_ACK;
  m.version = PROTOCOL_VERSION;
  m.status = status;
  m.transferId = transferId;
  m.bytesReceived = bytes;
  snprintf(m.id, sizeof(m.id), "%s", id ? id : "");
  sendToPeer(&m, sizeof(m));  // on the peer, the "other" end is the master
}

void abandonRx() {
  if (rx.active) content::uploadAbort();
  rx.active = false;
}

void peerBegin(const FileBeginMsg &m) {
  char id[sizeof(m.id) + 1];
  memcpy(id, m.id, sizeof(m.id));
  id[sizeof(m.id)] = '\0';

  if (rx.active && m.transferId == rx.transferId) {  // a resent BEGIN
    ack(FILE_IN_PROGRESS, m.transferId, rx.received, id);
    return;
  }
  abandonRx();  // a new transfer replaces any stale one

  // Already here, identical? Then nothing to write, which is fine mid-spin.
  ContentItem it{};
  if (content::find(id, it) && !it.preset) {
    char path[64];
    userPath(id, path, sizeof(path));
    uint32_t size = 0, sum = 0;
    if (fileSum(path, size, sum) && size == m.totalBytes && sum == m.checksum) {
      ack(FILE_OK, m.transferId, size, id);
      return;
    }
  }
  if (!armStopped()) {
    ack(FILE_ERR_SPINNING, m.transferId, 0, id);
    return;
  }
  if (content::exists(id)) content::remove(id);  // a stale copy under this id

  char name[sizeof(m.name) + 1];
  memcpy(name, m.name, sizeof(m.name));
  name[sizeof(m.name)] = '\0';
  const char *err = content::uploadBegin(name, m.totalBytes, id);
  if (err) {
    netlog::printf("[SYNC] cannot take \"%s\": %s", id, err);
    ack(FILE_ERR_SPACE, m.transferId, 0, id);
    return;
  }
  rx.active = true;
  rx.transferId = m.transferId;
  snprintf(rx.id, sizeof(rx.id), "%s", id);
  rx.total = m.totalBytes;
  rx.received = 0;
  rx.sum = 0;
  rx.expectSum = m.checksum;
  ack(FILE_IN_PROGRESS, m.transferId, 0, id);
}

void peerChunk(const FileChunkMsg &h, const uint8_t *data, size_t len) {
  if (!rx.active || h.transferId != rx.transferId) return;
  if (h.offset != rx.received) {  // a resend or a gap: say where we are
    ack(FILE_IN_PROGRESS, rx.transferId, rx.received, rx.id);
    return;
  }
  if (!armStopped()) {
    abandonRx();
    ack(FILE_ERR_SPINNING, h.transferId, 0, rx.id);
    return;
  }
  const char *err = content::uploadChunk(data, len);
  if (err) {
    netlog::printf("[SYNC] \"%s\" rejected: %s", rx.id, err);
    rx.active = false;  // uploadChunk has already discarded the partial file
    ack(FILE_ERR_INVALID, h.transferId, 0, rx.id);
    return;
  }
  for (size_t i = 0; i < len; i++) rx.sum += data[i];
  rx.received += len;
  ack(FILE_IN_PROGRESS, rx.transferId, rx.received, rx.id);
}

void peerEnd(const FileEndMsg &m) {
  if (!rx.active || m.transferId != rx.transferId) {
    if (m.transferId == rx.lastDoneTransferId) ack(FILE_OK, m.transferId, m.totalBytes, rx.id);
    return;
  }
  if (rx.received != rx.total) {
    ack(FILE_IN_PROGRESS, rx.transferId, rx.received, rx.id);
    return;
  }
  if (rx.sum != rx.expectSum) {
    abandonRx();
    netlog::printf("[SYNC] \"%s\" arrived damaged and was discarded", rx.id);
    ack(FILE_ERR_CHECKSUM, m.transferId, 0, rx.id);
    return;
  }
  ContentItem item{};
  const char *err = content::uploadFinish(item, false);  // never changes our selection
  rx.active = false;
  if (err) {
    netlog::printf("[SYNC] \"%s\" could not be saved: %s", rx.id, err);
    ack(FILE_ERR_IO, m.transferId, 0, rx.id);
    return;
  }
  rx.lastDoneTransferId = m.transferId;
  netlog::printf("[SYNC] received \"%s\" from the left fan, %lu bytes", rx.id,
                 (unsigned long)rx.total);
  ack(FILE_OK, m.transferId, rx.total, rx.id);
}

void peerDelete(const FileDeleteMsg &m) {
  char id[sizeof(m.id) + 1];
  memcpy(id, m.id, sizeof(m.id));
  id[sizeof(m.id)] = '\0';
  ContentItem it{};
  if (!content::find(id, it) || it.preset) {  // gone already, or never ours to delete
    ack(FILE_DELETED, m.transferId, 0, id);
    return;
  }
  if (!armStopped()) {
    ack(FILE_ERR_SPINNING, m.transferId, 0, id);
    return;
  }
  content::remove(id);
  netlog::printf("[SYNC] deleted \"%s\" (deleted on the left fan)", id);
  ack(FILE_DELETED, m.transferId, 0, id);
}

void peerTick() {
  content::uploadWatchdog();  // a master that vanished mid-copy

  static uint8_t pkt[sizeof(mail)];
  int len = 0;
  portENTER_CRITICAL(&mailMux);
  if (mailFull) {
    len = mailLen;
    memcpy(pkt, mail, len);
    mailFull = false;
  }
  portEXIT_CRITICAL(&mailMux);
  if (len < 2) return;

  switch (pkt[0]) {
    case MSG_FILE_BEGIN:
      if (len >= (int)sizeof(FileBeginMsg)) {
        FileBeginMsg m;
        memcpy(&m, pkt, sizeof(m));
        peerBegin(m);
      }
      break;
    case MSG_FILE_CHUNK:
      if (len > (int)sizeof(FileChunkMsg)) {
        FileChunkMsg h;
        memcpy(&h, pkt, sizeof(h));
        const size_t payload = (size_t)len - sizeof(h);
        if (h.dataLen == payload) peerChunk(h, pkt + sizeof(h), payload);
      }
      break;
    case MSG_FILE_END:
      if (len >= (int)sizeof(FileEndMsg)) {
        FileEndMsg m;
        memcpy(&m, pkt, sizeof(m));
        peerEnd(m);
      }
      break;
    case MSG_FILE_DELETE:
      if (len >= (int)sizeof(FileDeleteMsg)) {
        FileDeleteMsg m;
        memcpy(&m, pkt, sizeof(m));
        peerDelete(m);
      }
      break;
    default:
      break;
  }
}

}  // namespace

namespace filesync {

void begin() {
  if (!unitlink::isMaster()) return;
  nextTransferId = esp_random();  // never reuse a peer's remembered id after our reboot
  ContentItem it{};
  const size_t n = content::count();
  for (size_t i = 0; i < n; i++) {
    if (content::at(i, it) && !it.preset) setEntry(it.id, State::Unknown);
  }
}

void tick() {
  if (unitlink::isMaster()) masterTick();
  else peerTick();
}

void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len < 2 || data[1] != PROTOCOL_VERSION) return;
  // Only from the other stator: the master's acks, or the master's files.
  if (!unitlink::linkKnown() || memcmp(info->src_addr, unitlink::linkMac(), 6) != 0) return;

  if (unitlink::isMaster()) {
    if (data[0] != MSG_FILE_ACK || len < (int)sizeof(FileAckMsg)) return;
    portENTER_CRITICAL(&ackMux);
    memcpy(&ackMail, data, sizeof(FileAckMsg));
    ackFull = true;
    portEXIT_CRITICAL(&ackMux);
    return;
  }
  // Peer: one packet at a time. The master waits for each ack before sending
  // the next, so a full mailbox only ever means a resend; dropping it is fine.
  if (len > (int)sizeof(mail)) return;
  portENTER_CRITICAL(&mailMux);
  if (!mailFull) {
    memcpy(mail, data, len);
    mailLen = len;
    mailFull = true;
  }
  portEXIT_CRITICAL(&mailMux);
  unitlink::wakePeerTask();
}

void onUploaded(const char *id) {
  if (unitlink::isMaster() && id && *id) setEntry(id, State::Unknown);
}

void onDeleted(const char *id) {
  if (!unitlink::isMaster() || !id || !*id) return;
  removeEntry(id);
  portENTER_CRITICAL(&tableMux);
  if (deleteCount < MAX_DELETES) {
    PendingDelete &d = deletes[deleteCount++];
    snprintf(d.id, sizeof(d.id), "%s", id);
    d.waitStopped = false;
    d.retryAtMs = 0;
  }
  portEXIT_CRITICAL(&tableMux);
}

State state(const char *id) {
  State st = State::None;
  portENTER_CRITICAL(&tableMux);
  const int i = findEntry(id);
  if (i >= 0) st = table[i].st;
  portEXIT_CRITICAL(&tableMux);
  return st;
}

const char *lastError(const char *id) {
  static char err[64];
  err[0] = '\0';
  portENTER_CRITICAL(&tableMux);
  const int i = findEntry(id);
  if (i >= 0) snprintf(err, sizeof(err), "%s", table[i].err);
  portEXIT_CRITICAL(&tableMux);
  return err;
}

void prioritise(const char *id) {
  portENTER_CRITICAL(&tableMux);
  const int i = findEntry(id);
  if (i >= 0) {
    if (table[i].st == State::Failed) table[i].retryAtMs = 0;
    snprintf(priorityId, sizeof(priorityId), "%s", id);
  }
  portEXIT_CRITICAL(&tableMux);
}

bool busy() { return unitlink::isMaster() && job.phase != Phase::None && !job.isDelete; }

void peerAbortForStart() {
  if (unitlink::isMaster() || !rx.active) return;
  abandonRx();
  netlog::printf("[SYNC] copy of \"%s\" abandoned: the fan was started", rx.id);
}

}  // namespace filesync

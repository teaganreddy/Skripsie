#include "content.h"

#include <LittleFS.h>
#include <Preferences.h>

#include "config.h"
#include "link_protocol.h"

namespace {

const size_t MAX_ITEMS = 24;

ContentItem items[MAX_ITEMS];
size_t itemCount = 0;
char selectedId[28] = {0};

Preferences prefs;

/* The library array is mutated from the AsyncTCP task (upload, delete) and
 * read from the telemetry task (which names the current content in every
 * status event), so it needs a lock. Recursive because remove() legitimately
 * calls rescan() and find() beneath itself. */
SemaphoreHandle_t libMutex = nullptr;

struct Lock {
  Lock() {
    if (libMutex) xSemaphoreTakeRecursive(libMutex, portMAX_DELAY);
  }
  ~Lock() {
    if (libMutex) xSemaphoreGiveRecursive(libMutex);
  }
};

// The presets that ship in the web bundle. Their display names live here
// rather than being parsed out of presets/manifest.json, which is gzipped on
// flash -- decompressing JSON just to recover a handful of fixed strings would
// be a lot of machinery for no benefit.
//
// MUST BE KEPT IN STEP with the PRESETS array in the website's
// tools/make-presets.js. A preset missing from this table still lists and
// still plays, but under its raw id ("colour-wheel" rather than
// "Colour wheel"), so the symptom is cosmetic and easy to miss.
struct PresetName {
  const char *id;
  const char *name;
};
const PresetName PRESET_NAMES[] = {
    {"circle", "Circle"}, {"line", "Line"},     {"colour-wheel", "Colour wheel"},
    {"rings", "Rings"},   {"spiral", "Spiral"}, {"pulse", "Pulse"},
};

const char *prettyPresetName(const char *id) {
  for (const auto &p : PRESET_NAMES) {
    if (strcmp(p.id, id) == 0) return p.name;
  }
  return id;  // an unrecognised preset still lists, under its own id
}

uint32_t be32(const uint8_t *p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}
uint16_t be16(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }

/* Validates the 16-byte .povf header (see API.md section 5). Returns nullptr
 * if it is good, otherwise a message for the user. */
const char *parseHeader(const uint8_t *h, ContentItem &out) {
  if (be32(h) != POVF_MAGIC) {
    return "That file is not in the format the fan expects. Please upload an "
           "image through the fan's own upload page.";
  }
  if (h[4] != POVF_VERSION) {
    return "That file was made by a different version of the fan software.";
  }
  out.radial = h[5];
  out.angles = be16(h + 6);
  out.frames = be16(h + 8);
  out.fps = h[10];

  if (out.radial == 0 || out.radial > POVF_MAX_RADIAL || out.angles == 0 ||
      out.angles > POVF_MAX_ANGLES || out.frames == 0 ||
      out.frames > POVF_MAX_FRAMES) {
    return "That file's contents do not look right and it has not been saved.";
  }

  // The declared payload length must match the geometry it claims, or the
  // rotor would read off the end of the buffer in stage 2.
  const uint32_t declared = be32(h + 12);
  const uint32_t expected =
      (uint32_t)out.frames * out.angles * out.radial * 3;
  if (declared != expected) {
    return "That file appears to be incomplete or damaged.";
  }

  /* The rotor has to hold the whole payload in one PSRAM buffer. Its limit is
   * LINK_MAX_CONTENT_BYTES; the POVF_MAX_* bounds above are far looser
   * (64 x 360 x 64 x 3 = 4 423 680), so without this check a file could pass
   * validation, be written to LittleFS, be listed in the library, and then be
   * refused by the rotor with CONTENT_ERR_SIZE every time it was selected --
   * while the auto re-push in rotor_stub.cpp retried it every 8 seconds
   * forever. Better to refuse it at the door with a message that says why. */
  if (declared > LINK_MAX_CONTENT_BYTES) {
    static char msg[160];
    snprintf(msg, sizeof(msg),
             "That image is too large for the display to hold (%u KB, limit "
             "%u KB). Use fewer frames.",
             (unsigned)(declared / 1024),
             (unsigned)(LINK_MAX_CONTENT_BYTES / 1024));
    return msg;
  }
  return nullptr;
}

bool readItemFromFile(const char *path, const char *id, bool preset,
                      ContentItem &out) {
  File f = LittleFS.open(path, "r");
  if (!f) return false;
  if (f.size() < POVF_HEADER_BYTES) {
    f.close();
    return false;
  }
  uint8_t header[POVF_HEADER_BYTES];
  if (f.read(header, POVF_HEADER_BYTES) != POVF_HEADER_BYTES) {
    f.close();
    return false;
  }
  out.bytes = f.size();
  f.close();

  memset(out.id, 0, sizeof(out.id));
  snprintf(out.id, sizeof(out.id), "%s", id);
  out.preset = preset;
  return parseHeader(header, out) == nullptr;
}

/* What to fall back to when the remembered selection has gone. Never the
 * sensor test: it sits at index 0 and is a diagnostic, so a fan that had its
 * selected image deleted -- or that has never been told what to show -- would
 * otherwise come up flashing R/G/B at an open day. Unlocked; callers hold the
 * library lock. */
int firstDisplayableIndex() {
  for (size_t i = 0; i < itemCount; i++) {
    if (strcmp(items[i].id, SENSOR_TEST_ID) != 0) return (int)i;
  }
  return itemCount > 0 ? 0 : -1;
}

// Unlocked -- callers already hold the library lock.
int findIndex(const char *id) {
  if (!id || !*id) return -1;
  for (size_t i = 0; i < itemCount; i++) {
    if (strcmp(items[i].id, id) == 0) return (int)i;
  }
  return -1;
}

void userNamePath(const char *id, char *buf, size_t len) {
  snprintf(buf, len, "%s/%s.nam", USER_DIR, id);
}
void userDataPath(const char *id, char *buf, size_t len) {
  snprintf(buf, len, "%s/%s.povf", USER_DIR, id);
}

void loadUserName(const char *id, char *out, size_t len) {
  char path[64];
  userNamePath(id, path, sizeof(path));
  File f = LittleFS.open(path, "r");
  if (!f) {
    snprintf(out, len, "%s", id);
    return;
  }
  size_t n = f.read((uint8_t *)out, len - 1);
  out[n] = '\0';
  f.close();
  if (out[0] == '\0') snprintf(out, len, "%s", id);
}

void scanDir(const char *dir, bool preset) {
  File root = LittleFS.open(dir);
  if (!root || !root.isDirectory()) return;

  for (File f = root.openNextFile(); f; f = root.openNextFile()) {
    if (itemCount >= MAX_ITEMS) break;
    if (f.isDirectory()) continue;

    // f.name() is the bare filename on this core; build the full path back up.
    String fname = f.name();
    int slash = fname.lastIndexOf('/');
    if (slash >= 0) fname = fname.substring(slash + 1);
    if (!fname.endsWith(".povf")) continue;

    String id = fname.substring(0, fname.length() - 5);
    char path[80];
    snprintf(path, sizeof(path), "%s/%s", dir, fname.c_str());

    ContentItem item{};
    if (!readItemFromFile(path, id.c_str(), preset, item)) continue;

    if (preset) {
      snprintf(item.name, sizeof(item.name), "%s", prettyPresetName(id.c_str()));
    } else {
      loadUserName(id.c_str(), item.name, sizeof(item.name));
    }
    items[itemCount++] = item;
  }
  root.close();
}

// --- upload state. One at a time; a second concurrent upload is refused. ---
struct {
  bool active = false;
  File file;
  char id[28] = {0};
  char name[32] = {0};
  char path[64] = {0};
  size_t written = 0;
  size_t expected = 0;
  uint8_t header[POVF_HEADER_BYTES];
  size_t headerSeen = 0;
  ContentItem parsed{};
  uint32_t token = 0;
  uint32_t lastChunkMs = 0;
} up;

uint32_t uploadTokenCounter = 0;

// How long an upload may go without a chunk before it is presumed abandoned.
// Generous: a 1 MB payload over a microcontroller AP is genuinely slow, and
// this only has to catch a client that has gone away entirely.
const uint32_t UPLOAD_STALL_MS = 15000;

void discardPartial() {
  if (up.file) up.file.close();
  if (up.path[0]) LittleFS.remove(up.path);
  up.active = false;
  up.written = 0;
  up.headerSeen = 0;
  up.path[0] = '\0';
}

/* Content ids without a wall clock. The mock uses a millisecond timestamp,
 * which the ESP32 does not have -- millis() restarts at zero every boot and
 * would collide. A counter in NVS survives reboots instead. */
void nextId(char *out, size_t len) {
  uint32_t n = prefs.getUInt("nextid", 1);
  prefs.putUInt("nextid", n + 1);
  snprintf(out, len, "u%lx", (unsigned long)n);
}

}  // namespace

namespace content {

bool begin() {
  libMutex = xSemaphoreCreateRecursiveMutex();

  /* The partition label MUST be passed explicitly. Arduino's LittleFS
   * defaults to partitionLabel = "spiffs" (see LittleFS.h), and esp_littlefs
   * matches partitions by label -- so with our partition named "littlefs",
   * a plain LittleFS.begin() looks for a label that does not exist and the
   * filesystem silently never mounts. That presents as a blank web page with
   * no obvious cause. */
  if (!LittleFS.begin(false, "/littlefs", 10, "littlefs")) {
    Serial.println(F("[FS] mount failed -- has the filesystem been uploaded? "
                     "(pio run -t uploadfs)"));
    return false;
  }
  if (!LittleFS.exists(USER_DIR)) LittleFS.mkdir(USER_DIR);

  prefs.begin("fan", false);
  String saved = prefs.getString("content", "");
  snprintf(selectedId, sizeof(selectedId), "%s", saved.c_str());

  rescan();

  // If nothing was remembered (or what was remembered has since been deleted)
  // fall back to whatever is first in the library.
  {
    Lock lock;
    const int fallback = firstDisplayableIndex();
    if (findIndex(selectedId) < 0 && fallback >= 0) {
      snprintf(selectedId, sizeof(selectedId), "%s", items[fallback].id);
    }
  }

  Serial.printf("[FS] %u item(s), %u KB total, %u KB free for uploads\n",
                (unsigned)itemCount, (unsigned)(LittleFS.totalBytes() / 1024),
                (unsigned)(freeBytes() / 1024));
  return true;
}

/* The sensor test, which has no file. Listed first so it is easy to find when
 * something is wrong, and marked preset so it refuses deletion like the rest.
 *
 * Its geometry is reported as zero rather than as a plausible 1 x 180 x 18,
 * because there is no image data behind it -- and the website reads `bytes` to
 * decide whether to show the "loading onto rotor" bar, which must not appear
 * for a mode that transfers nothing. */
void addSensorTest() {
  ContentItem &it = items[itemCount++];
  it = ContentItem{};
  snprintf(it.id, sizeof(it.id), "%s", SENSOR_TEST_ID);
  snprintf(it.name, sizeof(it.name), "%s", SENSOR_TEST_NAME);
  it.preset = true;
  it.frames = 1;
  it.fps = 1;
  it.radial = 0;
  it.angles = 0;
  it.bytes = 0;
}

void rescan() {
  Lock lock;
  itemCount = 0;
  addSensorTest();
  scanDir(PRESET_DIR, true);
  scanDir(USER_DIR, false);
}

size_t count() {
  Lock lock;
  return itemCount;
}

bool at(size_t i, ContentItem &out) {
  Lock lock;
  if (i >= itemCount) return false;
  out = items[i];
  return true;
}

bool find(const char *id, ContentItem &out) {
  Lock lock;
  const int idx = findIndex(id);
  if (idx < 0) return false;
  out = items[idx];
  return true;
}

bool exists(const char *id) {
  Lock lock;
  return findIndex(id) >= 0;
}

size_t freeBytes() {
  const size_t total = LittleFS.totalBytes();
  const size_t used = LittleFS.usedBytes();
  if (total <= used + FS_RESERVE_BYTES) return 0;
  return total - used - FS_RESERVE_BYTES;
}

const char *remove(const char *id) {
  Lock lock;
  const int idx = findIndex(id);
  if (idx < 0) return "That image is not on the fan.";
  if (items[idx].preset) return "Presets are built in and cannot be deleted.";

  char data[64], name[64];
  userDataPath(id, data, sizeof(data));
  userNamePath(id, name, sizeof(name));
  LittleFS.remove(data);
  LittleFS.remove(name);

  const bool wasSelected = strcmp(selectedId, id) == 0;
  rescan();
  if (wasSelected) {
    selectedId[0] = '\0';
    const int fallback = firstDisplayableIndex();
    if (fallback >= 0) {
      snprintf(selectedId, sizeof(selectedId), "%s", items[fallback].id);
    }
    prefs.putString("content", selectedId);
  }
  return nullptr;
}

bool select(const char *id) {
  Lock lock;
  if (findIndex(id) < 0) return false;
  snprintf(selectedId, sizeof(selectedId), "%s", id);
  prefs.putString("content", selectedId);
  return true;
}

bool selected(ContentItem &out) {
  Lock lock;
  const int idx = findIndex(selectedId);
  if (idx < 0) return false;
  out = items[idx];
  return true;
}

// --- upload -------------------------------------------------------------

bool uploadActive() { return up.active; }

const char *uploadBegin(const char *displayName, size_t totalBytes) {
  if (up.active) {
    return "The fan is already receiving an image. Please wait for that one to "
           "finish.";
  }
  if (totalBytes < POVF_HEADER_BYTES) {
    return "That upload was empty.";
  }
  if (totalBytes > freeBytes()) {
    static char msg[128];
    snprintf(msg, sizeof(msg),
             "There is not enough space left on the fan for that image (it "
             "needs %u KB, %u KB free). Delete something first.",
             (unsigned)(totalBytes / 1024), (unsigned)(freeBytes() / 1024));
    return msg;
  }

  up = {};
  nextId(up.id, sizeof(up.id));
  snprintf(up.name, sizeof(up.name), "%s",
           (displayName && *displayName) ? displayName : "Untitled");
  userDataPath(up.id, up.path, sizeof(up.path));

  up.file = LittleFS.open(up.path, "w");
  if (!up.file) {
    up.path[0] = '\0';
    return "The fan could not save that image. Its storage may be full.";
  }
  up.expected = totalBytes;
  up.active = true;
  up.token = ++uploadTokenCounter;
  up.lastChunkMs = millis();
  return nullptr;
}

uint32_t uploadToken() { return up.token; }

void uploadWatchdog() {
  if (!up.active) return;
  if (millis() - up.lastChunkMs < UPLOAD_STALL_MS) return;
  Serial.println(F("[FS] upload stalled -- client gone, discarding partial file"));
  discardPartial();
}

const char *uploadChunk(const uint8_t *data, size_t len) {
  if (!up.active) return "That upload is no longer in progress.";
  up.lastChunkMs = millis();

  // Validate the header the moment we have all 16 bytes, so a bad payload is
  // rejected after 16 bytes rather than after a megabyte.
  if (up.headerSeen < POVF_HEADER_BYTES) {
    const size_t take = min(len, POVF_HEADER_BYTES - up.headerSeen);
    memcpy(up.header + up.headerSeen, data, take);
    up.headerSeen += take;

    if (up.headerSeen == POVF_HEADER_BYTES) {
      const char *err = parseHeader(up.header, up.parsed);
      if (err) {
        discardPartial();
        return err;
      }
      const uint32_t expectedTotal =
          POVF_HEADER_BYTES +
          (uint32_t)up.parsed.frames * up.parsed.angles * up.parsed.radial * 3;
      if (up.expected != expectedTotal) {
        discardPartial();
        return "That file's size does not match what its header describes.";
      }
    }
  }

  if (up.file.write(data, len) != len) {
    discardPartial();
    return "The fan ran out of space while saving that image.";
  }
  up.written += len;

  if (up.written > up.expected) {
    discardPartial();
    return "That upload sent more data than it said it would.";
  }
  return nullptr;
}

const char *uploadFinish(ContentItem &out) {
  if (!up.active) return "That upload is no longer in progress.";

  if (up.written != up.expected || up.headerSeen < POVF_HEADER_BYTES) {
    discardPartial();
    return "That upload did not finish. Please try again.";
  }

  up.file.close();

  char namePath[64];
  userNamePath(up.id, namePath, sizeof(namePath));
  File nf = LittleFS.open(namePath, "w");
  if (nf) {
    nf.print(up.name);
    nf.close();
  }

  out = up.parsed;
  snprintf(out.id, sizeof(out.id), "%s", up.id);
  snprintf(out.name, sizeof(out.name), "%s", up.name);
  out.preset = false;
  out.bytes = up.written;

  up.active = false;
  rescan();
  select(out.id);

  Serial.printf("[FS] uploaded \"%s\" (%s) -- %u frame(s), %u KB\n", out.name,
                out.id, out.frames, (unsigned)(out.bytes / 1024));
  return nullptr;
}

void uploadAbort() {
  if (up.active) {
    Serial.println(F("[FS] upload aborted, discarding partial file"));
    discardPartial();
  }
}

}  // namespace content

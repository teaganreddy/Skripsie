#include "content.h"

#include <atomic>

#include "config.h"
#include "netlog.h"
#include "rotor_link.h"

namespace {

/* Worst case the website can produce: MAX_FRAMES (64) at the current geometry.
 * Allocated up front so a transfer never allocates while running.
 *
 * Taken from link_protocol.h so the stator validates uploads against the SAME
 * number this allocates -- they used to be independent and disagreed by 4x. */
const size_t MAX_CONTENT_BYTES = LINK_MAX_CONTENT_BYTES;

struct Buffer {
  uint8_t *data = nullptr;
  uint16_t frames = 0;
  uint16_t angles = 0;
  uint8_t radial = 0;
  uint8_t fps = 1;
  uint32_t bytes = 0;
  char id[24] = {0};
};

Buffer bufA, bufB;

/* Where an RLE transfer lands before it is decoded. Same size as a play
 * buffer: a run-length stream is never larger than 4/3 of its output and the
 * stator only sends RLE when it is actually smaller, so this is ample. The
 * transfer path itself does not know or care about the encoding -- bytes are
 * placed by offset exactly as a raw transfer's are, so retransmitted chunks
 * stay harmless -- and the decode happens once, after END has verified. */
uint8_t *stage = nullptr;
uint8_t incomingEncoding = CONTENT_ENC_RAW;
uint32_t wireBytes = 0;   // bytes expected on the wire this transfer
uint8_t *wireBuf = nullptr;

/* Physical-to-content angle mapping, applied in column().
 *
 * Angle 0 of a frame is wherever the Hall magnet happens to be glued, which
 * differs per unit; with two fans side by side a mismatch is glaring, and
 * anything directional (the ball crossing from one disc to the other, fish
 * swimming with the rail) needs it right. sectorOffset rotates content to the
 * physical frame; sectorReverse flips the direction of rotation if the arm
 * turns the other way from what the website assumes. Both are calibrated on
 * the website's Diagnostics tab with the "Calibrate" preset; the master
 * stator holds them and sends them in every CommandMsg. */
volatile int sectorOffset = SECTOR_OFFSET;
volatile bool sectorReverse = SECTOR_REVERSE != 0;

/* The display task on core 1 reads `active` while the WiFi task on core 0
 * writes `incoming`. Only the pointer is shared, and it is only ever swapped
 * between two buffers that are both permanently allocated -- so a reader can
 * never follow it into freed memory. */
std::atomic<Buffer *> active{nullptr};
Buffer *incoming = &bufB;

uint32_t currentTransfer = 0;
uint32_t received = 0;
uint32_t runningSum = 0;
bool inProgress = false;

/* An END has arrived and been range-checked, but the buffer has not been summed
 * yet. serviceTask picks this up; see finishTransfer(). */
ContentEndMsg pendingEnd{};
volatile bool endPending = false;

/* Bumped every time what is on screen changes. displayTask watches it to reset
 * its frame cursor: frameIdx is a function-static that survived across content
 * changes, so selecting a 1-frame image after a 24-frame animation carried the
 * old index in. content::column() bounds it with (frame % frames) so it was
 * never unsafe, but an animation would resume from an arbitrary frame instead
 * of starting at the beginning. */
std::atomic<uint32_t> generationCounter{0};

/* Scratch for resampling when the content's radial != the strip's. TWO of
 * them, one per arm: the display fetches both arms' columns before writing
 * either, so a single shared buffer would have arm B quietly overwrite arm A
 * and paint the same column on both. */
uint8_t resampled[2][LEDS_PER_ARM * 3];

}  // namespace

namespace content {

bool begin() {
  bufA.data = (uint8_t *)ps_malloc(MAX_CONTENT_BYTES);
  bufB.data = (uint8_t *)ps_malloc(MAX_CONTENT_BYTES);
  if (!bufA.data || !bufB.data) {
    Serial.println(F("[CONTENT] PSRAM alloc FAILED -- no content playback"));
    return false;
  }
  stage = (uint8_t *)ps_malloc(MAX_CONTENT_BYTES);
  if (!stage) Serial.println(F("[CONTENT] no PSRAM for the RLE staging buffer"));
  active.store(nullptr);
  incoming = &bufB;
  Serial.printf("[CONTENT] two %u KB buffers in PSRAM (double buffered)%s\n",
                (unsigned)(MAX_CONTENT_BYTES / 1024), stage ? " + RLE staging" : "");
  return true;
}

uint8_t onBegin(const ContentBeginMsg &m) {
  if (!bufA.data || !bufB.data) return CONTENT_ERR_NO_MEMORY;

  if (m.payloadBytes == 0 || m.payloadBytes > MAX_CONTENT_BYTES ||
      m.radial == 0 || m.angles == 0 || m.frames == 0) {
    netlog::printf("[CONTENT] rejecting: %u frames, %u angles, %u radial, %lu bytes",
                  m.frames, m.angles, m.radial, (unsigned long)m.payloadBytes);
    return CONTENT_ERR_SIZE;
  }
  const uint32_t expect = (uint32_t)m.frames * m.angles * m.radial * 3;
  if (expect > MAX_CONTENT_BYTES) return CONTENT_ERR_SIZE;
  if (m.encoding == CONTENT_ENC_RAW) {
    // The declared geometry must actually describe the declared length.
    if (expect != m.payloadBytes) {
      Serial.printf("[CONTENT] geometry/length mismatch: expect %lu, told %lu\n",
                    (unsigned long)expect, (unsigned long)m.payloadBytes);
      return CONTENT_ERR_SIZE;
    }
  } else if (m.encoding == CONTENT_ENC_RLE) {
    if (!stage) return CONTENT_ERR_NO_MEMORY;
  } else {
    Serial.printf("[CONTENT] unknown encoding %u\n", m.encoding);
    return CONTENT_ERR_SIZE;
  }

  /* Receive into whichever buffer is NOT on screen. */
  Buffer *act = active.load();
  incoming = (act == &bufA) ? &bufB : &bufA;

  incoming->frames = m.frames;
  incoming->angles = m.angles;
  incoming->radial = m.radial;
  incoming->fps = m.fps ? m.fps : 1;
  incoming->bytes = expect;  // decoded size; what column() indexes
  memcpy(incoming->id, m.id, sizeof(incoming->id) - 1);
  incoming->id[sizeof(incoming->id) - 1] = '\0';

  incomingEncoding = m.encoding;
  wireBytes = m.payloadBytes;
  wireBuf = (m.encoding == CONTENT_ENC_RLE) ? stage : incoming->data;

  currentTransfer = m.transferId;
  received = 0;
  runningSum = 0;
  inProgress = true;

  netlog::printf("[CONTENT] receiving \"%s\": %u frames @ %u fps, %ux%u, %lu bytes%s",
                 incoming->id, m.frames, incoming->fps, m.radial, m.angles,
                 (unsigned long)m.payloadBytes,
                 m.encoding == CONTENT_ENC_RLE ? " (RLE)" : "");
  return CONTENT_IN_PROGRESS;
}

void onChunk(uint32_t transferId, uint32_t offset, const uint8_t *data,
             size_t len) {
  // Stale packet from an abandoned transfer -- must not land in the new buffer.
  if (!inProgress || transferId != currentTransfer) return;
  if (offset + len > wireBytes) return;  // would run past the end

  memcpy(wireBuf + offset, data, len);

  /* Coverage, not an arrival count -- and the checksum is deliberately NOT
   * accumulated here any more.
   *
   * The stator's flow control is stop-and-wait on the MAC-layer ack, so it
   * resends a chunk whose ACK was lost. The chunk itself may well have arrived:
   * a duplicate would then have been added to the byte count and summed a
   * second time, failing the checksum on a transfer that was in fact perfect.
   * Offsets arrive in order, so the high-water mark is the honest byte count,
   * and the checksum is taken over the assembled buffer at END -- which is
   * immune to duplicates and to reordering alike. */
  const uint32_t end = offset + (uint32_t)len;
  if (end > received) received = end;
}

/* Verifies and swaps in a completed transfer.
 *
 * DELIBERATELY NOT CALLED FROM THE ESP-NOW RECEIVE CALLBACK. It sums the whole
 * assembled buffer -- up to 1.05 MB, a byte at a time, out of PSRAM -- and that
 * used to run inside onRecv(), i.e. inside the WiFi task. ESP-IDF requires
 * those callbacks to return promptly: while this ran, nothing else was
 * received, no ack could be sent, and the 5 Hz telemetry the stator times out
 * on at 2 s was competing with a blocked WiFi task. At 233 KB (the spiral
 * preset) that is milliseconds; at the 1.05 MB cap it is tens of them.
 *
 * onEnd() now only records that an END arrived, and serviceTask on core 0 calls
 * this. */
uint8_t finishTransfer() {
  if (received != pendingEnd.payloadBytes || received != wireBytes) {
    netlog::printf("[CONTENT] INCOMPLETE: got %lu of %lu bytes",
                  (unsigned long)received, (unsigned long)wireBytes);
    return CONTENT_ERR_INCOMPLETE;
  }

  // Over the assembled buffer, so retransmitted chunks cannot double-count.
  runningSum = 0;
  for (uint32_t i = 0; i < wireBytes; i++) runningSum += wireBuf[i];

  if (runningSum != pendingEnd.checksum) {
    netlog::printf("[CONTENT] CHECKSUM mismatch: %lu vs %lu",
                  (unsigned long)runningSum, (unsigned long)pendingEnd.checksum);
    return CONTENT_ERR_CHECKSUM;
  }

  if (incomingEncoding == CONTENT_ENC_RLE) {
    /* Decode [count][r][g][b] runs from the staging buffer into the play
     * buffer. The output must come out at exactly the declared geometry; a
     * stream that runs short or long is corrupt, whatever the checksum said. */
    const uint32_t t0 = millis();
    uint32_t out = 0;
    uint32_t in = 0;
    while (in + 4 <= wireBytes) {
      const uint32_t n = stage[in];
      const uint8_t r = stage[in + 1], g = stage[in + 2], b = stage[in + 3];
      in += 4;
      if (n == 0 || out + n * 3 > incoming->bytes) break;
      uint8_t *d = incoming->data + out;
      for (uint32_t i = 0; i < n; i++) { *d++ = r; *d++ = g; *d++ = b; }
      out += n * 3;
    }
    if (out != incoming->bytes || in != wireBytes) {
      netlog::printf("[CONTENT] RLE decode mismatch: %lu of %lu bytes out, %lu of %lu in",
                    (unsigned long)out, (unsigned long)incoming->bytes,
                    (unsigned long)in, (unsigned long)wireBytes);
      return CONTENT_ERR_INCOMPLETE;
    }
    netlog::printf("[CONTENT] RLE %lu -> %lu bytes in %lu ms", (unsigned long)wireBytes,
                  (unsigned long)out, (unsigned long)(millis() - t0));
  }

  // Verified -- only now does it become what the display paints.
  active.store(incoming);
  generationCounter.fetch_add(1);
  netlog::printf("[CONTENT] \"%s\" accepted and now displaying", incoming->id);
  return CONTENT_OK;
}

uint8_t onEnd(const ContentEndMsg &m) {
  if (!inProgress || m.transferId != currentTransfer) return CONTENT_ERR_INCOMPLETE;
  inProgress = false;

  // Cheap checks only -- this runs in the WiFi task. The expensive sum happens
  // in finishTransfer(), driven from serviceTask.
  if (received != wireBytes) {
    Serial.printf("[CONTENT] INCOMPLETE: got %lu of %lu bytes\n",
                  (unsigned long)received, (unsigned long)wireBytes);
    return CONTENT_ERR_INCOMPLETE;
  }

  pendingEnd = m;
  endPending = true;
  return CONTENT_IN_PROGRESS;  // verdict follows once the checksum has run
}

bool verifyPending(uint8_t &statusOut, uint32_t &transferOut) {
  if (!endPending) return false;
  endPending = false;
  transferOut = pendingEnd.transferId;
  statusOut = finishTransfer();
  return true;
}

uint32_t bytesReceived() { return received; }
uint32_t activeTransferId() { return currentTransfer; }
bool receiving() { return inProgress; }
bool transferActive() { return inProgress || endPending; }

bool available() { return active.load() != nullptr; }

uint16_t frames() { Buffer *b = active.load(); return b ? b->frames : 0; }
uint8_t fps() { Buffer *b = active.load(); return b ? b->fps : 1; }
uint16_t angles() { Buffer *b = active.load(); return b ? b->angles : 0; }
uint8_t radial() { Buffer *b = active.load(); return b ? b->radial : 0; }
const char *id() { Buffer *b = active.load(); return b ? b->id : ""; }

bool needsResample() {
  Buffer *b = active.load();
  return b && b->radial != LEDS_PER_ARM;
}

int mapSector(int sector) {
  /* Physical sector (Hall zero, direction of rotation) -> content angle.
   * Reverse first, then rotate: the offset is measured AFTER the direction
   * is right, which is the order the calibration keys are used in. */
  int a = sectorReverse ? (ANGLES - sector) % ANGLES : sector;
  a = (a + sectorOffset) % ANGLES;
  if (a < 0) a += ANGLES;
  /* Mirror about the content's vertical axis: theta -> 180 - theta. Content
   * angle 0 is +x (to the viewer's right) and increases anticlockwise, as the
   * website's polar.js defines it. */
  if (rotorlink::mirror()) a = (ANGLES / 2 - a + ANGLES) % ANGLES;
  return a;
}

const uint8_t *column(uint16_t frame, int sector, int arm) {
  Buffer *b = active.load();
  if (!b || sector < 0) return nullptr;

  const int phys = mapSector(sector);

  // Remap if the content's angular resolution differs from ours.
  uint32_t a = (b->angles == ANGLES)
                   ? (uint32_t)phys
                   : ((uint32_t)phys * b->angles) / ANGLES;
  if (a >= b->angles) a = b->angles - 1;

  const uint16_t f = b->frames ? (frame % b->frames) : 0;
  const uint8_t *col = b->data + ((size_t)(f * b->angles + a) * b->radial) * 3;

  if (b->radial == LEDS_PER_ARM) return col;  // the common case: no copy at all

  /* Nearest-neighbour resample onto the strip. Only runs when the content was
   * generated for a different arm length -- the website now generates 18 to
   * match, so this is a fallback for older files rather than the normal path.
   * Nearest-neighbour rather than interpolation on purpose: the browser has
   * already applied gamma, and blending gamma-encoded values would shift the
   * colours. */
  uint8_t *dst = resampled[arm & 1];
  for (int r = 0; r < LEDS_PER_ARM; r++) {
    int src = (int)((uint32_t)r * b->radial / LEDS_PER_ARM);
    if (src >= b->radial) src = b->radial - 1;
    dst[r * 3 + 0] = col[src * 3 + 0];
    dst[r * 3 + 1] = col[src * 3 + 1];
    dst[r * 3 + 2] = col[src * 3 + 2];
  }
  return dst;
}

void clear() {
  active.store(nullptr);
  inProgress = false;
  endPending = false;
  generationCounter.fetch_add(1);
}

uint32_t generation() { return generationCounter.load(); }

int offsetSectors() { return sectorOffset; }
bool reversed() { return sectorReverse; }
void setOffsetSectors(int s) {
  s %= ANGLES;
  if (s < 0) s += ANGLES;
  sectorOffset = s;
}
void setReversed(bool r) { sectorReverse = r; }

}  // namespace content

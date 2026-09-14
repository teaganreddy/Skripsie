#pragma once
/*
 * content.h -- the .povf library on LittleFS.
 *
 * Presets are flashed with the web bundle and live in /presets. User uploads
 * live in /user as <id>.povf plus a tiny <id>.nam sidecar holding the display
 * name, which avoids needing a JSON parser on the device just to remember what
 * the user called their image.
 *
 * Uploads are genuinely written to flash and genuinely listed here. Only the
 * onward push to the rotor is faked -- see rotor_stub.h.
 */

#include <Arduino.h>

struct ContentItem {
  char id[28];
  char name[32];
  bool preset;  // presets are built in and refuse deletion
  uint16_t frames;
  uint8_t fps;
  uint8_t radial;
  uint16_t angles;
  uint32_t bytes;
};

namespace content {

bool begin();
void rescan();

/* These copy the item out rather than handing back a pointer into the library
 * array: the array is rebuilt on the AsyncTCP task whenever something is
 * uploaded or deleted, so a pointer handed to another task could be left
 * pointing at a rebuilt entry mid-iteration. */
size_t count();
bool at(size_t i, ContentItem &out);
bool find(const char *id, ContentItem &out);
bool exists(const char *id);

// Bytes available for a new upload, after leaving LittleFS its working slack.
size_t freeBytes();

// Returns nullptr on success, or a message written for a member of the public.
const char *remove(const char *id);

bool select(const char *id);
bool selected(ContentItem &out);

/* --- streamed upload ---------------------------------------------------
 * Called from the async body handler. The payload is written to flash as it
 * arrives and is NEVER buffered whole: a 64-frame animation is ~1.1 MB and
 * the C6 has 512 KB of SRAM in total.
 *
 * Each function returns nullptr on success or a human-readable error. Once
 * one fails, the partial file has already been deleted and uploadActive()
 * goes false.
 */
const char *uploadBegin(const char *displayName, size_t totalBytes);
const char *uploadChunk(const uint8_t *data, size_t len);
const char *uploadFinish(ContentItem &out);
void uploadAbort();
bool uploadActive();

/* Identifies the upload in progress. A body handler grabs this at its first
 * chunk and re-checks it on every later chunk, so that if its upload was
 * abandoned and a different one has since started, its chunks cannot be
 * written into the new one's file. */
uint32_t uploadToken();

/* A client that vanishes mid-upload never runs its handler again, so nothing
 * would otherwise close the file or delete the partial -- and uploadActive()
 * would stay true forever, refusing every future upload. Call periodically.
 */
void uploadWatchdog();

}  // namespace content

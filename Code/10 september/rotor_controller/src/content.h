#pragma once
/*
 * content.h -- received .povf frame data, held in PSRAM and played locally.
 *
 * DOUBLE BUFFERED, and that is not a luxury. A transfer takes seconds; the
 * display is painting columns the whole time. Receiving straight into the
 * buffer being displayed would tear the image for the entire push and show
 * half of the old picture and half of the new. Chunks land in `incoming`, and
 * only a complete, verified transfer swaps it in.
 *
 * Both buffers are allocated once at boot, at the maximum content size, so a
 * transfer never has to allocate under time pressure and can never fail
 * halfway through for want of memory.
 *
 * Layout matches .povf exactly (API.md section 5), angle-major:
 *      offset(f, a, r, c) = ((f * angles + a) * radial + r) * 3 + c
 * so a column is contiguous and can be handed straight to the SPI driver with
 * no copying or rearranging -- which is the whole reason the browser writes it
 * in this order.
 */

#include <Arduino.h>

#include "link_protocol.h"

namespace content {

bool begin();

// --- receive side, driven by rotor_link.cpp from the ESP-NOW callback ---
uint8_t onBegin(const ContentBeginMsg &m);
void onChunk(uint32_t transferId, uint32_t offset, const uint8_t *data, size_t len);
uint8_t onEnd(const ContentEndMsg &m);

/* Runs the expensive END verification (a full-buffer checksum) OUTSIDE the
 * ESP-NOW receive callback. Call from a normal task; returns true when a
 * transfer was pending, and then statusOut is the CONTENT_* verdict to ack. */
bool verifyPending(uint8_t &statusOut, uint32_t &transferOut);

uint32_t bytesReceived();
uint32_t activeTransferId();
bool receiving();

// --- display side, read from the display task on core 1 ---
bool available();

/* Changes whenever what is displayed changes. The display task resets its frame
 * cursor on a change; see the note in content.cpp. */
uint32_t generation();
uint16_t frames();
uint8_t fps();
uint16_t angles();
uint8_t radial();
const char *id();

/* The contiguous RGB run for one column, or nullptr if there is no content.
 * `sector` is in ANGLES units and is remapped if the content disagrees.
 *
 * The returned pointer is into the active buffer, which only changes on a
 * completed transfer, so it stays valid for the caller's immediate use. */
/* `arm` is 0 or 1 and selects which resample scratch buffer is used. Both
 * arms are fetched before either is painted, so they cannot share one. */
const uint8_t *column(uint16_t frame, int sector, int arm);

/* True when the content's radial resolution does not match the strip, so
 * column() needs resampling. Reported rather than hidden, because it costs
 * image quality and the honest fix is to regenerate the content. */
bool needsResample();

void clear();

}  // namespace content

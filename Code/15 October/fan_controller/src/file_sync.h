#pragma once
/* ===========================================================================
 * File sync: user uploads copied from the master's flash to the peer's.
 *
 * Uploads land only on unit 1's LittleFS. The right fan's stator pushes images
 * to its rotor from ITS OWN flash, so an upload has to be copied there first.
 * Once copied it has the same id on both stators and behaves exactly like a
 * preset: the master sends the id, the peer pushes it with the normal path,
 * and it survives a rotor reboot.
 *
 *   MASTER  keeps a sync state per user file. After an upload (and for every
 *           user file at boot, and again whenever the peer link comes back)
 *           it sends FILE_BEGIN; the peer answers FILE_OK if it already has
 *           the file, otherwise the master streams it chunk by chunk.
 *           Deleting an upload deletes the peer's copy too.
 *
 *   PEER    writes to flash only while ITS arm is stopped (gotcha 6) and
 *           answers FILE_ERR_SPINNING otherwise; the master then waits for
 *           the right fan to stop and tries again. A start command abandons
 *           any half-written copy before the motor is driven.
 *
 * Wire format: FileBeginMsg / FileChunkMsg / FileEndMsg / FileDeleteMsg /
 * FileAckMsg in link_protocol.h.
 * ======================================================================== */

#include <Arduino.h>
#include <esp_now.h>

namespace filesync {

enum class State : uint8_t {
  None,     // not a user file the master knows about
  Unknown,  // not yet checked against the peer
  Pending,  // needs copying; waiting for the link or for the right fan to stop
  Copying,  // transfer in progress
  Synced,   // the peer holds an identical copy
  Failed,   // last attempt failed; retried automatically
};

/* MASTER: seed the table with every user file on flash. After content::begin()
 * and after ESP-NOW is up. */
void begin();

/* Both roles. Master: from the telemetry task. Peer: from the peer task. */
void tick();

/* Both roles, from rotor_stub.cpp's receive callback for MSG_FILE_*. Copies
 * the packet into a mailbox and returns; tick() does the work. */
void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len);

// ---- MASTER ----
void onUploaded(const char *id);
void onDeleted(const char *id);
State state(const char *id);
/* Why the last attempt failed, for the 409 the website shows. */
const char *lastError(const char *id);
/* Move this file to the front of the queue (the user just asked for it). */
void prioritise(const char *id);
/* A copy is being streamed: the right fan must not be started under it. */
bool busy();

// ---- PEER ----
/* Abandon a half-written copy. Called before the motor is started. */
void peerAbortForStart();

}  // namespace filesync

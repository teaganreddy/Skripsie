#pragma once
/* ===========================================================================
 * netlog -- the log you can read without a serial cable.
 *
 * Nothing that spins can have USB attached, and both stators sit under the
 * arms. So the lines that explain a problem are printed to Serial as before
 * AND:
 *
 *   MASTER  appended to a ring buffer the website reads at /api/log, tagged
 *           with the unit and board they came from.
 *   PEER    queued and sent to the master as MSG_LOG, one per tick.
 *
 * Both roles also receive MSG_LOG from their own rotor: the master files it,
 * the peer forwards it. Keep the 1 Hz status chatter on plain Serial; the
 * website shows those numbers already and the link is shared with telemetry.
 * ======================================================================== */

#include <Arduino.h>

#include <esp_now.h>

#include "link_protocol.h"

namespace netlog {

void begin();

/* Print + file/forward. Safe from any task, including the ESP-NOW callback
 * (it never blocks: a line that cannot get the lock is dropped). */
void printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* MSG_LOG from the rotor (either role) or from the peer stator (master). */
void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len);

/* Peer: sends one queued line to the master, if any. Master: no-op. */
void tick(const uint8_t *masterMac, bool haveMaster);

/* Master: JSON {"boot":B,"next":N,"lines":[{"seq":n,"t":ms,"tag":"U1","text":"..."}]}
 * B is random per boot, so a reader can tell the log restarted.
 * for lines with seq >= since. */
void toJson(String &out, uint32_t since);

}  // namespace netlog

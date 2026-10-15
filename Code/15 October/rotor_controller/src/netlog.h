#pragma once
/* netlog -- serial lines that also travel to the stator over ESP-NOW.
 *
 * The rotor spins; nobody can watch its serial port while it matters. The
 * lines that explain a problem (pairing, content accepted or rejected, the
 * RLE decode, calibration) go through here: printed to Serial exactly as
 * before AND queued for rotorlink to send as MSG_LOG. The stator forwards
 * them to the website's Diagnostics tab. Keep the 1 Hz status chatter on
 * plain Serial: the queue is small and the link is shared with telemetry. */

#include <Arduino.h>

namespace netlog {

void printf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/* Pops the oldest queued line into `out` (LogMsg::text sized). False if none. */
bool take(char *out, size_t len);

}  // namespace netlog

#pragma once
/*
 * blackbox.h -- a flight recorder for the spinning rotor.
 *
 * The rotor has no USB while it is turning and the radio link is exactly what
 * is unreliable, so the only trustworthy way to see what happened is to write
 * it down locally and read it back afterwards.
 *
 * Two rings, both in PSRAM (which is now confirmed at 8 MB, so the space is
 * free):
 *
 *   HALL RING   one entry per Hall edge, written from the ISR. This is the
 *               decisive record for the sync question -- it shows the actual
 *               interval between every edge, and whether each was accepted or
 *               rejected. Missed pulses, contact bounce and a mismeasured
 *               period all look completely different here, and identical from
 *               the outside.
 *
 *   STATE RING  a periodic snapshot of speed, sector, paint rate and link
 *               health, so a link dropout can be lined up in time against
 *               whatever the display was doing when it happened.
 *
 * Dump with 'd' over USB once the arm has stopped. Output is CSV.
 */

#include <Arduino.h>

namespace blackbox {

void begin();

/* Called from the Hall ISR -- must stay IRAM-safe and short. `intervalUs` is
 * the gap since the previous edge; `accepted` is false when the sanity window
 * rejected it. */
void IRAM_ATTR recordHallEdge(uint32_t intervalUs, bool accepted);

// Periodic state snapshot. From loop().
void tick();

// Print both rings as CSV, oldest first.
void dump();

void clear();
bool ready();

}  // namespace blackbox

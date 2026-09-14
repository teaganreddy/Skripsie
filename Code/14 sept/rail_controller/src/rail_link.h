#pragma once
/*
 * rail_link.h -- ESP-NOW to the stator.
 *
 * Mirrors rotor_controller/src/rotor_link.cpp, which is the pattern proven on
 * this IDF: STA mode with no association, channel pinned by hand, pair with
 * whichever stator beacons OUR unit id, send telemetry at 5 Hz, obey commands.
 *
 * The rail takes commands only from its paired stator and never moves without
 * one. If the stator goes quiet mid-move the rail halts -- the failure
 * direction is always "stop".
 */

#include <Arduino.h>

namespace raillink {

void begin();
void tick();  // from loop()

bool paired();
uint32_t sinceLastPacketMs();

uint32_t sent();
uint32_t received();
uint32_t sendFailures();

}  // namespace raillink

#pragma once
/*
 * rotor_link.h -- rotor side of the ESP-NOW link.
 *
 * Runs on core 0 alongside WiFi. The display loop owns core 1 and never waits
 * on anything here, which is the point of putting the display on a dual-core
 * part: radio work cannot disturb column timing.
 *
 * Discovery rather than hardcoded MACs: the stator broadcasts a beacon, this
 * end learns its address from the packet and unicasts telemetry back. Either
 * board can be swapped or reflashed without the other caring.
 *
 * The channel is the trap worth knowing about. ESP-NOW does not scan or roam,
 * so both ends must already be on the same channel. The stator's AP fixes
 * channel 1 (AP_CHANNEL in fan_controller/src/config.h) and this end locks to
 * the same via LINK_CHANNEL. If they ever disagree, nothing is received and
 * nothing reports an error -- packets simply go out on a channel no one is
 * listening to.
 */

#include <Arduino.h>

#include "link_protocol.h"

namespace rotorlink {

void begin();

// Pumps discovery and the 2 Hz telemetry send. Call from loop().
void tick();

// True once a beacon has been heard and the stator's MAC is known.
bool paired();

// Milliseconds since the last packet from the stator, or UINT32_MAX if never.
uint32_t sinceLastPacketMs();

// Latest command from the stator. Valid only while paired().
uint8_t commandedBrightness();
bool displayEnabled();
const char *commandedContentId();

/* The website has selected the "Sensor test" preset: show the per-revolution
 * R/G/B diagnostic instead of any content that is loaded. False when unpaired,
 * so a bench rotor behaves as it always did. */
bool sensorTest();

// Counters, for the serial diagnostics.
uint32_t sent();
uint32_t received();
uint32_t sendFailures();

}  // namespace rotorlink

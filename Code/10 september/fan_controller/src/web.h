#pragma once
/*
 * web.h -- WiFi access point, mDNS, static file serving and the whole API.md
 * surface.
 *
 * Everything in here runs on the AsyncTCP task. The rules that follow from
 * that, and which the implementation sticks to:
 *
 *   - no handler ever calls into SimpleFOC; commands go through motorctl,
 *     which queues them for the motor task
 *   - no handler blocks, sleeps, or waits on a mutex it might not get
 *   - no handler buffers an upload in RAM
 */

#include <Arduino.h>

namespace web {

bool begin();

// Pumps 2 Hz status events and rotor-progress events. Runs on its own task
// (see web.cpp) so telemetry survives a long upload.
void startTelemetryTask();

int brightness();

// Number of stations currently associated with the AP. Printed on the serial
// diagnostic line, so "did my phone actually join?" is visible from the bench.
int stationCount();

}  // namespace web

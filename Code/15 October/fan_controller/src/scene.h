#pragma once
/* ===========================================================================
 * Scenes -- the two-fan shows on the website's Scenes tab. MASTER only.
 *
 * The website does the sequencing (Both mode, rail mode, one file per fan,
 * start). What the site cannot do is the two things that need sub-second
 * timing between the boards, and they live here:
 *
 *   LOOP SYNC     pong and penalty have one ball crossing two discs, so the
 *                 two rotors must be on the same frame. Once per animation
 *                 loop the master bumps a counter that every rotor treats as
 *                 "frame 0, now". Unit 2's copy goes via its stator, so
 *                 unit 1's is sent SCENE_SYNC_LEAD_MS later to land level.
 *
 *   FOLLOW RAIL   fish are drawn facing right. While the rail carries the
 *                 fans toward the viewer's left the content is played
 *                 mirrored, so the fish swim the way they travel.
 *
 *   RAIL CYCLE    flappy's file is one whole sweep cycle (out, turn, back,
 *                 turn), with the bird on whichever fan is trailing. Each
 *                 time the carriage sets off, both rotors are restarted at
 *                 the frame that matches where it is, worked out from the
 *                 rail's position and its trapezoid move, so the turn in
 *                 the picture lands on the real stop at the end.
 *
 * Which content wants which is the PRESET_* flags in content.cpp, keyed on
 * the LEFT fan's selection.
 * ======================================================================== */

#include <Arduino.h>

namespace scene {

// From the telemetry task, after rotor::tick() / unitlink::tick() / rail::tick().
void tick();

/* Milliseconds until tick() next has something timed to do (a loop pulse,
 * or unit 1's delayed copy of one), or UINT32_MAX. The telemetry task
 * sleeps no longer than this, so pulses leave on time rather than on the
 * next 50 ms pass. */
uint32_t msUntilDue();

bool syncing();   // loop pulses or rail-cycle restarts are being sent
bool mirrored();  // content is currently played mirrored
}  // namespace scene

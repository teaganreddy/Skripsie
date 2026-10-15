#include "scene.h"

#include "config.h"
#include "content.h"
#include "netlog.h"
#include "rail_link.h"
#include "rotor_stub.h"
#include "unit_link.h"

namespace {

uint8_t loopSeq = 0;
uint32_t loopStartMs = 0;
uint32_t loopMs = 0;
char syncedId[28] = {0};
bool mirror = false;
bool pendingOwnRotor = false;
uint32_t ownRotorDueMs = 0;
uint8_t startFrame = 0;  // sent with every scene update: where a loopSeq restart starts

/* Rail-cycle scenes (flappy). The file is one whole sweep cycle, drawn
 * against the rail's own trapezoid: the first half is the leg toward the
 * viewer's right and then the dwell at that end, the second half the leg
 * back and the dwell at HOME. Each half ENDS with the dwell. Every time the
 * carriage sets off, both rotors are restarted at the frame that matches
 * where the carriage is, so the picture stays locked to the rail however
 * long the real legs take. */
bool cycleOn = false;       // a PRESET_RAIL_CYCLE file is selected and the rail is sweeping
int legDir = 0;             // direction of the leg already restarted for; 0 = none yet
float restMm = 0;           // where the carriage last stood still: the start of the next leg
bool cyclePending = false;  // a restart is scheduled
uint32_t cycleSendMs = 0;   // when it goes to the peer (unit 1 follows SCENE_SYNC_LEAD_MS later)
uint8_t cycleFrame = 0;     // the frame it restarts at

/* The rail's moves are trapezoids: accelerate at RAIL_ACCEL_MM_S2 up to v,
 * cruise, decelerate the same way (a triangle if the move is too short to
 * reach v). Seconds for a whole move of `mm`. */
float moveSeconds(float mm, float v) {
  const float a = RAIL_ACCEL_MM_S2, ta = v / a, da = v * ta / 2;
  return mm >= 2 * da ? 2 * ta + (mm - 2 * da) / v : 2 * sqrtf(mm / a);
}

// Seconds since departure when the carriage is `d` mm into a move of `mm`.
float secondsAt(float d, float mm, float v) {
  const float a = RAIL_ACCEL_MM_S2, da = v * v / (2 * a);
  if (2 * da > mm) {  // triangle: accelerate to half way, then brake
    return d <= mm / 2 ? sqrtf(2 * d / a) : moveSeconds(mm, v) - sqrtf(2 * (mm - d) / a);
  }
  if (d <= da) return sqrtf(2 * d / a);
  if (d <= mm - da) return v / a + (d - da) / v;
  return moveSeconds(mm, v) - sqrtf(2 * (mm - d) / a);
}

/* The carriage has just set off. Work out how far into its half of the file
 * the content should be right now, and schedule a restart on the next frame
 * boundary. Lined up on the ARRIVAL: the content's dwell starts the moment
 * the real carriage stops, which is what the pipes and the bird swap are
 * drawn around, even if the real leg is a little longer or shorter than the
 * one the file was drawn for. */
void scheduleLegRestart(const ContentItem &sel, int dir, uint32_t now) {
  float pos, tgt;
  uint32_t ageMs;
  if (!rail::legSnapshot(pos, tgt, ageMs)) return;

  float v = rail::sweepSpeedMms();
  if (v <= 0) v = RAIL_CRUISE_MM_S;  // what the rail uses for 0
  if (v > RAIL_MAX_MM_S) v = RAIL_MAX_MM_S;
  uint32_t dwellMs = rail::sweepDwellMs();
  if (dwellMs == 0) dwellMs = 1000;  // the rail's SWEEP_DWELL_MS_DEFAULT

  const float legMm = fabsf(tgt - restMm);
  float d = fabsf(pos - restMm);
  if (d > legMm) d = legMm;
  // Seconds the carriage still needs to arrive, as of now (the packet is ageMs old).
  const float toGoS = moveSeconds(legMm, v) - secondsAt(d, legMm, v) - ageMs / 1000.0f;

  const int32_t frameMs = (int32_t)(1000u / sel.fps);  // the rotor's own integer frame time
  const int32_t half = sel.frames / 2;
  const int32_t arriveMs = half * frameMs - (int32_t)dwellMs;  // content time of the arrival
  const int32_t contentMs = arriveMs - (int32_t)lroundf(toGoS * 1000.0f);  // content time now

  // Restart on a frame boundary, far enough ahead for the peer's copy to lead.
  int32_t k = contentMs > 0 ? (contentMs + frameMs - 1) / frameMs : 0;
  int32_t waitMs = k * frameMs - contentMs;
  if (waitMs < (int32_t)SCENE_SYNC_LEAD_MS) {
    k++;
    waitMs += frameMs;
  }
  if (k > half - 1) {  // only with a stale or nonsense packet: best effort
    k = half - 1;
    waitMs = (int32_t)SCENE_SYNC_LEAD_MS;
  }

  const bool towardRight = RAIL_HOME_IS_LEFT ? dir > 0 : dir < 0;
  cycleFrame = (uint8_t)((towardRight ? 0 : half) + k);
  cycleSendMs = now + (uint32_t)waitMs - SCENE_SYNC_LEAD_MS;
  cyclePending = true;
}

}  // namespace

namespace scene {

void tick() {
  const uint32_t now = millis();

  ContentItem sel{};
  const bool have = content::selected(sel);
  const uint8_t flags = have ? content::presetFlags(sel.id) : 0;

  // ---- follow the rail: mirror while the carriage moves toward the viewer's left
  bool wantMirror = false;
  if (flags & PRESET_FOLLOW_RAIL) {
    const int dir = rail::direction();  // +1 toward the far end, -1 toward HOME
    if (dir != 0) {
      const bool towardHome = dir < 0;
      wantMirror = RAIL_HOME_IS_LEFT ? towardHome : !towardHome;
    } else {
      wantMirror = mirror;  // dwelling at an end: keep facing the way we came
    }
  }
  if (wantMirror != mirror) {
    mirror = wantMirror;
    netlog::printf("[SCENE] mirror %s (rail direction %d)", mirror ? "ON" : "off", rail::direction());
    unitlink::peerSetScene(mirror, loopSeq, startFrame);
    rotor::setScene(mirror, loopSeq, startFrame);
  }

  // ---- loop sync: one pulse per animation loop
  if ((flags & PRESET_SYNC) && sel.frames > 1 && sel.fps > 0) {
    if (strcmp(syncedId, sel.id) != 0) {
      snprintf(syncedId, sizeof(syncedId), "%s", sel.id);
      // Same integer arithmetic as the rotor's frame timer, so a loop here is
      // exactly a loop there and the pulse lands on the wrap, not near it.
      loopMs = (uint32_t)sel.frames * (1000u / sel.fps);
      loopStartMs = now;  // first pulse one loop from now; the rotors start at 0 on receipt
      netlog::printf("[SCENE] loop sync on for \"%s\": pulse every %lu ms", sel.id,
                     (unsigned long)loopMs);
    }
    if (loopMs && now - loopStartMs >= loopMs) {
      loopStartMs += loopMs;  // keep the cadence, do not accumulate tick jitter
      loopSeq++;
      startFrame = 0;
      unitlink::peerSetScene(mirror, loopSeq, startFrame);
      pendingOwnRotor = true;
      ownRotorDueMs = now + SCENE_SYNC_LEAD_MS;
    }
  } else {
    syncedId[0] = '\0';
    loopMs = 0;
  }

  // ---- rail cycle: restart both rotors at the matching frame each time the carriage sets off
  // Where it stands still is where the next leg starts (the far end after a
  // dwell, or wherever it was when the sweep began).
  if (rail::online() && !rail::moving()) restMm = rail::positionMm();
  const bool wantCycle = (flags & PRESET_RAIL_CYCLE) && sel.frames >= 2 && sel.frames <= 255 &&
                         sel.fps > 0 && rail::sweeping();
  if (wantCycle) {
    if (!cycleOn) {
      cycleOn = true;
      legDir = 0;
      netlog::printf("[SCENE] rail cycle on for \"%s\": %u frames, restarted every leg", sel.id,
                     (unsigned)sel.frames);
    }
    const int dir = rail::direction();
    if (dir == 0) {
      legDir = 0;  // dwelling: the next departure, either way, is a new leg
    } else if (dir != legDir) {
      legDir = dir;
      scheduleLegRestart(sel, dir, now);
    }
  } else if (cycleOn) {
    cycleOn = false;
    cyclePending = false;
    legDir = 0;
    netlog::printf("[SCENE] rail cycle off");
  }

  if (cyclePending && (int32_t)(now - cycleSendMs) >= 0) {
    cyclePending = false;
    loopSeq++;
    startFrame = cycleFrame;
    unitlink::peerSetScene(mirror, loopSeq, startFrame);
    pendingOwnRotor = true;
    ownRotorDueMs = now + SCENE_SYNC_LEAD_MS;
  }

  if (pendingOwnRotor && (int32_t)(now - ownRotorDueMs) >= 0) {
    pendingOwnRotor = false;
    rotor::setScene(mirror, loopSeq, startFrame);
  }
}

uint32_t msUntilDue() {
  const uint32_t now = millis();
  uint32_t due = UINT32_MAX;
  if (pendingOwnRotor) {
    const int32_t d = (int32_t)(ownRotorDueMs - now);
    due = d > 0 ? (uint32_t)d : 0;
  }
  if (loopMs) {  // the next pulse itself
    const int32_t d = (int32_t)(loopStartMs + loopMs - now);
    const uint32_t next = d > 0 ? (uint32_t)d : 0;
    if (next < due) due = next;
  }
  if (cyclePending) {  // a scheduled rail-cycle restart
    const int32_t d = (int32_t)(cycleSendMs - now);
    const uint32_t next = d > 0 ? (uint32_t)d : 0;
    if (next < due) due = next;
  }
  return due;
}

bool syncing() { return loopMs != 0 || cycleOn; }
bool mirrored() { return mirror; }

}  // namespace scene

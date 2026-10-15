#include "rotor_sync.h"

#include "blackbox.h"
#include "config.h"

namespace {

/* Shared with the ISR. 32-bit aligned scalars, so reads and writes are atomic
 * on this core -- but period and timestamp are a PAIR, and a torn read of the
 * two would put the angle badly wrong. seq guards them: the reader retries
 * while the ISR is mid-update, the classic seqlock. */
volatile uint32_t isrSeq = 0;
volatile uint32_t lastEdgeUs = 0;
volatile uint32_t revPeriodUs = 0;
volatile uint32_t revCount = 0;
volatile uint32_t rejected = 0;
volatile uint32_t overrunCount = 0;
/* Which revolution the current overrun was already counted against. sector()
 * is a pure query that gets polled many times a second, so counting on every
 * call measured the POLL RATE, not the number of overruns -- one slow
 * deceleration showed up as eighteen. */
volatile uint32_t overrunNotedRev = UINT32_MAX;
volatile uint32_t lastAcceptedUs = 0;

/* IRAM_ATTR because this must still run while flash is busy. The rotor writes
 * nothing to flash today, but ESP-NOW content transfer in stage 2c will, and
 * a cache stall during a Hall edge would corrupt the angular zero for a whole
 * revolution. Cheaper to be right now than to debug later. */
void IRAM_ATTR hallISR() {
  const uint32_t now = micros();
  const uint32_t since = now - lastAcceptedUs;

  if (since < HALL_DEBOUNCE_US) return;  // contact glitch, not a revolution

  /* Reject implausible periods rather than acting on them. A missed pulse
   * would otherwise read as half speed and rotate the image; electrical noise
   * would read as absurd speed and scramble it. */
  if (lastAcceptedUs != 0 && (since < PERIOD_MIN_US || since > PERIOD_MAX_US)) {
    rejected = rejected + 1;
    blackbox::recordHallEdge(since, false);
    lastAcceptedUs = now;  // resync anyway; the next interval may be sane
    return;
  }

  isrSeq = isrSeq + 1;  // odd: update in progress
  if (lastAcceptedUs != 0) revPeriodUs = since;
  lastEdgeUs = now;
  isrSeq = isrSeq + 1;  // even: consistent again

  blackbox::recordHallEdge(lastAcceptedUs ? since : 0, true);
  lastAcceptedUs = now;
  revCount = revCount + 1;
}

/* How long without a pulse before the arm counts as stopped, scaled to the
 * speed it was last turning at. See the HALL_TIMEOUT note in config.h. */
inline uint32_t timeoutFor(uint32_t period) {
  if (period == 0) return HALL_TIMEOUT_MAX_US;  // one edge seen, no period yet
  const uint32_t t = period * 2 + period / 2;   // 2.5 revolutions
  return constrain(t, HALL_TIMEOUT_MIN_US, HALL_TIMEOUT_MAX_US);
}

// Consistent snapshot of the (edge timestamp, period) pair.
inline bool snapshot(uint32_t &edgeUs, uint32_t &periodOut) {
  for (int attempt = 0; attempt < 4; attempt++) {
    const uint32_t s1 = isrSeq;
    if (s1 & 1) continue;  // ISR mid-update
    edgeUs = lastEdgeUs;
    periodOut = revPeriodUs;
    if (isrSeq == s1) return true;
  }
  return false;
}

}  // namespace

namespace rotorsync {

void begin() {
  pinMode(PIN_HALL, INPUT_PULLUP);  // open-collector Hall, LOW on magnet
  attachInterrupt(digitalPinToInterrupt(PIN_HALL), hallISR, FALLING);
  Serial.printf("[SYNC] Hall on GPIO%d, falling edge, %d magnet(s) per rev\n",
                PIN_HALL, MAGNETS_PER_REV);
}

bool locked() { return revPeriodUs != 0; }

bool stopped() {
  uint32_t edgeUs, period;
  if (!snapshot(edgeUs, period)) return true;
  if (edgeUs == 0) return true;
  return (uint32_t)(micros() - edgeUs) > timeoutFor(period);
}

int sector() {
  uint32_t edgeUs, period;
  if (!snapshot(edgeUs, period)) return -1;
  if (period == 0 || edgeUs == 0) return -1;

  const uint32_t elapsed = micros() - edgeUs;
  if (elapsed > timeoutFor(period)) return -1;  // arm has stopped

  /* 64-bit intermediate: elapsed can reach ~1e6 us at the slow end and ANGLES
   * is 180, so the product overflows 32 bits well inside the working range. */
  uint32_t s = (uint32_t)(((uint64_t)elapsed * ANGLES) / period);

  /* Ran past a full revolution before the next pulse -- the arm slowed after
   * the period was measured. Hold at the last sector rather than wrapping,
   * which would paint the image backwards over itself. */
  if (s >= (uint32_t)ANGLES) {
    // Count at most once per revolution -- see overrunNotedRev.
    if (overrunNotedRev != revCount) {
      overrunNotedRev = revCount;
      overrunCount = overrunCount + 1;
    }
    s = ANGLES - 1;
  }
  return (int)s;
}

uint32_t periodUs() { return revPeriodUs; }

float rpm() {
  const uint32_t p = revPeriodUs;
  if (p == 0 || stopped()) return 0.0f;
  return 60000000.0f / (float)p / (float)MAGNETS_PER_REV;
}

uint32_t revolutions() { return revCount; }
uint32_t rejectedPulses() { return rejected; }
uint32_t overruns() { return overrunCount; }
void noteOverrun() { overrunCount = overrunCount + 1; }

}  // namespace rotorsync

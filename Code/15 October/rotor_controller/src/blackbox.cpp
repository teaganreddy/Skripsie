#include "blackbox.h"

#include "apa102.h"
#include "config.h"
#include "rotor_link.h"
#include "rotor_sync.h"

extern volatile uint32_t columnsPainted;
extern volatile uint32_t freeRunRpm;

namespace {

/* Sized for a couple of minutes of running, which is far more than any single
 * test needs. In PSRAM, so this costs nothing that matters. */
const size_t HALL_ENTRIES = 4096;
const size_t STATE_ENTRIES = 4096;
const uint32_t STATE_INTERVAL_MS = 50;  // 20 Hz

struct __attribute__((packed)) HallEntry {
  uint32_t atMs;
  uint32_t intervalUs;
  uint8_t accepted;
  uint8_t pad[3];
};

struct __attribute__((packed)) StateEntry {
  uint32_t atMs;
  uint32_t periodUs;
  uint32_t revolutions;
  uint32_t columns;
  int16_t sector;
  uint16_t rpm;
  uint16_t rejected;
  uint16_t overruns;
  uint32_t linkRx;
  uint8_t flags;  // bit0 paired, bit1 spinning, bit2 locked
  uint8_t pad[3];
};

HallEntry *hallRing = nullptr;
StateEntry *stateRing = nullptr;
volatile size_t hallHead = 0;
size_t stateHead = 0;
volatile uint32_t hallTotal = 0;
uint32_t stateTotal = 0;
uint32_t lastStateMs = 0;
bool allocated = false;

}  // namespace

namespace blackbox {

void begin() {
  hallRing = (HallEntry *)ps_malloc(sizeof(HallEntry) * HALL_ENTRIES);
  stateRing = (StateEntry *)ps_malloc(sizeof(StateEntry) * STATE_ENTRIES);

  if (!hallRing || !stateRing) {
    // Fall back to internal RAM at a much smaller size rather than silently
    // recording nothing.
    Serial.println(F("[BLACKBOX] PSRAM alloc failed -- recorder disabled"));
    allocated = false;
    return;
  }
  memset(hallRing, 0, sizeof(HallEntry) * HALL_ENTRIES);
  memset(stateRing, 0, sizeof(StateEntry) * STATE_ENTRIES);
  allocated = true;
  Serial.printf("[BLACKBOX] recording to PSRAM: %u hall + %u state entries "
                "(%u KB). Press 'd' to dump, 'z' to clear.\n",
                (unsigned)HALL_ENTRIES, (unsigned)STATE_ENTRIES,
                (unsigned)((sizeof(HallEntry) * HALL_ENTRIES +
                            sizeof(StateEntry) * STATE_ENTRIES) /
                           1024));
}

void recordHallEdge(uint32_t intervalUs, bool accepted) {
  if (!allocated) return;
  const size_t i = hallHead;
  hallRing[i].atMs = millis();
  hallRing[i].intervalUs = intervalUs;
  hallRing[i].accepted = accepted ? 1 : 0;
  hallHead = (i + 1) % HALL_ENTRIES;
  hallTotal = hallTotal + 1;
}

void tick() {
  if (!allocated) return;
  const uint32_t now = millis();
  if (now - lastStateMs < STATE_INTERVAL_MS) return;
  lastStateMs = now;

  StateEntry &e = stateRing[stateHead];
  e.atMs = now;
  e.periodUs = rotorsync::periodUs();
  e.revolutions = rotorsync::revolutions();
  e.columns = columnsPainted;
  e.sector = (int16_t)rotorsync::sector();
  e.rpm = (uint16_t)lroundf(rotorsync::rpm());
  e.rejected = (uint16_t)rotorsync::rejectedPulses();
  e.overruns = (uint16_t)rotorsync::overruns();
  e.linkRx = rotorlink::received();
  e.flags = 0;
  if (rotorlink::paired()) e.flags |= 1;
  if (!rotorsync::stopped()) e.flags |= 2;
  if (rotorsync::locked()) e.flags |= 4;

  stateHead = (stateHead + 1) % STATE_ENTRIES;
  stateTotal++;
}

void dump() {
  if (!allocated) {
    Serial.println(F("[BLACKBOX] not allocated, nothing recorded"));
    return;
  }

  const uint32_t ht = hallTotal;
  const size_t hCount = ht < HALL_ENTRIES ? ht : HALL_ENTRIES;
  const size_t hStart = ht < HALL_ENTRIES ? 0 : hallHead;

  Serial.printf("\n#### BLACKBOX HALL EDGES (%u entries, newest last) ####\n",
                (unsigned)hCount);
  Serial.println(F("# interval_us is the gap since the previous edge."));
  Serial.println(F("# Steady rotation should show a near-constant interval."));
  Serial.println(F("#   intervals ~2x the rest  -> a pulse was MISSED"));
  Serial.println(F("#   clusters of tiny gaps   -> contact bounce"));
  Serial.println(F("#   accepted=0              -> outside the sanity window"));
  Serial.println(F("idx,at_ms,interval_us,implied_rpm,accepted"));
  for (size_t i = 0; i < hCount; i++) {
    const HallEntry &e = hallRing[(hStart + i) % HALL_ENTRIES];
    const float rpm = e.intervalUs ? 60000000.0f / (float)e.intervalUs : 0.0f;
    Serial.printf("%u,%lu,%lu,%.1f,%u\n", (unsigned)i, (unsigned long)e.atMs,
                  (unsigned long)e.intervalUs, rpm, e.accepted);
  }

  const uint32_t st = stateTotal;
  const size_t sCount = st < STATE_ENTRIES ? st : STATE_ENTRIES;
  const size_t sStart = st < STATE_ENTRIES ? 0 : stateHead;

  Serial.printf("\n#### BLACKBOX STATE (%u entries, newest last) ####\n",
                (unsigned)sCount);
  Serial.println(F("# flags: 1=link paired, 2=spinning, 4=sync locked"));
  Serial.println(F("idx,at_ms,rpm,period_us,sector,revs,columns,rejected,"
                   "overruns,link_rx,flags"));
  for (size_t i = 0; i < sCount; i++) {
    const StateEntry &e = stateRing[(sStart + i) % STATE_ENTRIES];
    Serial.printf("%u,%lu,%u,%lu,%d,%lu,%lu,%u,%u,%lu,%u\n", (unsigned)i,
                  (unsigned long)e.atMs, e.rpm, (unsigned long)e.periodUs,
                  e.sector, (unsigned long)e.revolutions,
                  (unsigned long)e.columns, e.rejected, e.overruns,
                  (unsigned long)e.linkRx, e.flags);
  }
  Serial.println(F("#### END BLACKBOX ####\n"));
}

void clear() {
  hallHead = 0;
  stateHead = 0;
  hallTotal = 0;
  stateTotal = 0;
  Serial.println(F("[BLACKBOX] cleared"));
}

bool ready() { return allocated; }

}  // namespace blackbox

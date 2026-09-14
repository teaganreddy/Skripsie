/*
 * rotor_controller -- rotor-side firmware for the hologram fan display.
 * ESP32-S3 on the rotating arm. Final-year Mechatronics project (MNS5).
 *
 * STAGE 2a, which is what this is: prove the display timing works.
 *   - hand-rolled APA102 driver           (apa102.cpp)
 *   - angular position from the Hall pulse (rotor_sync.cpp)
 *   - a test pattern generated in flash, painted at the interpolated angle
 *   - NO radio at all
 *
 * The point of doing it in this order is that column timing is the highest
 * technical risk in the project and it needs nothing from the stator. If the
 * image will not stand still, no amount of ESP-NOW work matters.
 *
 * Still to come: ESP-NOW telemetry (2b) and content transfer (2c), which
 * replace the STAGE 2 stub in fan_controller/src/rotor_stub.cpp.
 *
 * ===========================================================================
 *  THE ONE-CORE PROBLEM DOES NOT APPLY HERE
 * ===========================================================================
 * The C6 on the stator has a single core, and its whole design is arranged
 * around sharing it (see fan_controller/src/main.cpp). The S3 has two. Core 1
 * is given entirely to the display loop; WiFi and ESP-NOW live on core 0 and
 * never touch column timing. That is a genuine architectural advantage of
 * putting the display on this chip rather than the stator.
 *
 * "Given entirely" is literal, and it is a constraint as much as a luxury.
 * displayTask polls the interpolated sector and never blocks, at priority 20,
 * so NOTHING else can live on core 1 -- anything of lower priority there is
 * starved outright, with no watchdog to complain (the idle-task watchdog is
 * not enabled for CPU1). Arduino's own loop() runs on core 1 at priority 1,
 * which is why loop() on this firmware is empty and everything that is not the
 * display runs in serviceTask() on core 0. See the comment there: telemetry
 * sat in loop() until 2026-08-19 and silently stopped the moment an image
 * started playing.
 */

#include <Arduino.h>
#include <WiFi.h>

#include "apa102.h"
#include "blackbox.h"
#include "config.h"
#include "content.h"
#include "rotor_link.h"
#include "rotor_sync.h"

/* Shared with rotor_link.cpp, which reports them to the stator -- so these
 * deliberately sit outside the anonymous namespace below. Once the arm is
 * spinning there is no USB on the rotor, and these counters travelling over
 * ESP-NOW are the only way to see inside it. */
volatile uint32_t columnsPainted = 0;
volatile uint32_t freeRunRpm = 0;

namespace {

/* Test pattern, angle-major, matching the .povf layout so the playback code
 * in stage 2c can drop straight in:
 *      offset(a, r, c) = (a * LEDS_PER_ARM + r) * 3 + c
 *
 * 180 x 18 x 3 = 9720 bytes -- comfortably in SRAM, no PSRAM needed. */
uint8_t pattern[ANGLES * LEDS_PER_ARM * 3];

inline uint8_t *columnAt(int a) { return &pattern[(size_t)a * LEDS_PER_ARM * 3]; }

void hsvToRgb(float h, float s, float v, uint8_t &r, uint8_t &g, uint8_t &b) {
  const float c = v * s;
  const float x = c * (1.0f - fabsf(fmodf(h / 60.0f, 2.0f) - 1.0f));
  const float m = v - c;
  float rf = 0, gf = 0, bf = 0;
  if (h < 60)       { rf = c; gf = x; }
  else if (h < 120) { rf = x; gf = c; }
  else if (h < 180) { gf = c; bf = x; }
  else if (h < 240) { gf = x; bf = c; }
  else if (h < 300) { rf = x; bf = c; }
  else              { rf = c; bf = x; }
  r = (uint8_t)((rf + m) * 255);
  g = (uint8_t)((gf + m) * 255);
  b = (uint8_t)((bf + m) * 255);
}

/* A pattern chosen to make sync errors legible rather than to look pretty:
 *
 *   - hue sweeps with ANGLE, so the disc is a colour wheel. If the wheel
 *     rotates slowly, the period estimate is wrong. If it shimmers, there is
 *     jitter in the Hall timing.
 *   - a WHITE radial spoke at sector 0, the Hall zero. It should stand
 *     perfectly still and be one sector wide. Smearing = timing jitter;
 *     drifting = period error; two spokes = the arms are being painted with
 *     the wrong 180 degree offset.
 *   - brightness ramps hub-to-tip, so a radially mirrored arm is obvious
 *     (bright end at the hub means ARM_A_TIP_AT_INDEX_ZERO is wrong).
 */
void buildPattern() {
  for (int a = 0; a < ANGLES; a++) {
    uint8_t *col = columnAt(a);
    const bool marker = (a == 0);
    for (int r = 0; r < LEDS_PER_ARM; r++) {
      const float radial = (float)r / (float)(LEDS_PER_ARM - 1);  // 0 hub, 1 tip
      uint8_t cr, cg, cb;
      if (marker) {
        cr = cg = cb = 255;  // white spoke at the Hall zero
      } else {
        hsvToRgb((float)a * 360.0f / (float)ANGLES, 1.0f,
                 0.15f + 0.85f * radial, cr, cg, cb);
      }
      col[r * 3 + 0] = cr;
      col[r * 3 + 1] = cg;
      col[r * 3 + 2] = cb;
    }
  }
}

/* Free-run: a virtual rotation, so the whole render path can be checked on a
 * bench with USB attached and the arm stationary.
 *
 * Sync can only honestly be judged under STEADY rotation -- the period is
 * measured over the previous revolution and applied to the current one, so a
 * hand spin (long pause, quick turn) gives a period that describes neither.
 * That is not a fault to debug, it is an input the maths cannot use.
 *
 * This separates the two questions. Free-run exercises pattern -> column ->
 * SPI -> strip with a perfectly regular clock, so if the colours do not sweep
 * smoothly here the problem is the render path, not the Hall sensor. 0 = off,
 * use the real sensor. Defined above, outside this namespace. */

/* Display mode.
 *
 * REV_COLOUR is the DEFAULT and is a pure Hall-sensor diagnostic: the whole
 * strip goes one flat colour, cycling red -> green -> blue, advancing once per
 * accepted revolution. It takes the angle interpolation completely out of the
 * picture, so what you see is the sensor and nothing else:
 *
 *      clean R,G,B,R,G,B... one change per revolution  -> Hall is good
 *      colours skipping (R,B,G,R...)                   -> pulses being MISSED
 *      several changes in one revolution               -> bounce / multi-trigger
 *      no change at all while spinning                 -> no pulses accepted
 *
 * It also paints ONCE per revolution instead of 180 times, so the strip's duty
 * cycle and the SPI traffic both drop by a large factor. If the radio link
 * survives in this mode but not in POV mode, that points at power draw rather
 * than RF.
 *
 * POV_PATTERN is the colour-wheel-and-spoke pattern, which is the real display
 * test but only means anything once the sensor is trusted. Toggle with 'm'. */
enum class Mode : uint8_t { REV_COLOUR, POV_PATTERN };
volatile Mode displayMode = Mode::REV_COLOUR;

// --- display loop instrumentation ---
volatile uint32_t columnsLate = 0;
volatile uint32_t worstLateUs = 0;

/* Runs alone on core 1. Repaints whenever the interpolated sector changes,
 * rather than on a fixed clock -- the sector IS the clock, and deriving the
 * cadence from it means the display follows the arm through acceleration
 * without any rate tracking of its own. */
void displayTask(void *) {
  int lastSector = -1;
  uint32_t lastDeadline = 0;

  for (;;) {
    /* --- revolution-colour diagnostic ------------------------------------
     * Advances only on an ACCEPTED revolution, so rejected pulses show up as
     * the colour failing to change rather than as a silent statistic. */
    /* Received content ALWAYS wins over the diagnostic patterns. Selecting an
     * image on the website is an explicit instruction to show it, and it would
     * be absurd for that to depend on which bench mode happened to be set.
     *
     * This guard is the bug that made stage 2c look broken on first test: the
     * content rendering lived only in the POV branch, while REV_COLOUR was the
     * default and returned before ever reaching it. Images transferred
     * perfectly and were then thrown away. displayMode now only chooses the
     * FALLBACK shown when no content is loaded.
     *
     * The one thing that DOES override content is the website asking for this
     * diagnostic by name -- the "Sensor test" preset. That is not a bench mode
     * being allowed to win by accident, it is an explicit instruction of the
     * same kind as selecting an image, and content stays loaded underneath so
     * switching back to it is instant. */
    if (rotorlink::sensorTest() ||
        (displayMode == Mode::REV_COLOUR && !content::available())) {
      static uint32_t lastRevs = 0;
      static uint8_t colourIndex = 0;
      static bool blanked = false;

      if (rotorsync::stopped()) {
        if (!blanked) {
          apa102::clear();
          blanked = true;
        }
        lastRevs = rotorsync::revolutions();
        vTaskDelay(pdMS_TO_TICKS(20));
        continue;
      }

      const uint32_t revs = rotorsync::revolutions();
      if (revs != lastRevs || blanked) {
        lastRevs = revs;
        blanked = false;
        colourIndex = (uint8_t)((colourIndex + 1) % 3);
        apa102::fill(colourIndex == 0 ? 255 : 0, colourIndex == 1 ? 255 : 0,
                     colourIndex == 2 ? 255 : 0);
        columnsPainted = columnsPainted + 1;
      }
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    const uint32_t sim = freeRunRpm;
    if (sim > 0) {
      // Virtual clock -- Hall ignored entirely.
      const uint32_t periodUs = 60000000UL / sim;
      const int s = (int)(((uint64_t)(micros() % periodUs) * ANGLES) / periodUs);
      if (s != lastSector) {
        apa102::writeColumn(columnAt(s), columnAt((s + ANGLES / 2) % ANGLES));
        lastSector = s;
        columnsPainted = columnsPainted + 1;
      }
      taskYIELD();
      continue;
    }

    if (rotorsync::stopped() || !rotorsync::locked()) {
      if (lastSector != -1) {
        apa102::clear();
        lastSector = -1;
      }
      vTaskDelay(pdMS_TO_TICKS(20));  // nothing to paint; yield properly
      continue;
    }

    const int s = rotorsync::sector();
    if (s < 0 || s == lastSector) {
      taskYIELD();
      continue;
    }

    /* Both arms paint at once, half a revolution apart. The .povf layout puts
     * both arms' data in the same buffer already, so this is two lookups and
     * no mirroring -- exactly as API.md section 5 describes. */
    const int sectorB = (s + ANGLES / 2) % ANGLES;

    if (content::available()) {
      /* Received content wins over the test pattern. Frame advance is timed
       * from the file's own fps, independent of rotation speed -- an animation
       * should play at the rate it was authored at, not faster because the fan
       * is spinning quicker. */
      static uint32_t lastFrameMs = 0;
      static uint16_t frameIdx = 0;
      static uint32_t seenGeneration = 0;

      /* Start a newly selected image at frame 0. These are function-statics and
       * used to survive a content change, so picking a 24-frame animation and
       * then a different one resumed from wherever the last one happened to
       * be. column() bounds the index with (frame % frames) so it was never
       * unsafe -- just wrong. */
      const uint32_t gen = content::generation();
      if (gen != seenGeneration) {
        seenGeneration = gen;
        frameIdx = 0;
        lastFrameMs = millis();
      }

      const uint32_t frameMs = 1000u / max<uint8_t>(content::fps(), 1);
      const uint32_t nowMs = millis();
      if (content::frames() > 1 && nowMs - lastFrameMs >= frameMs) {
        lastFrameMs = nowMs;
        frameIdx = (uint16_t)((frameIdx + 1) % content::frames());
      }

      // One scratch buffer per arm, so fetching B cannot clobber A.
      apa102::writeColumn(content::column(frameIdx, s, 0),
                          content::column(frameIdx, sectorB, 1));
    } else {
      apa102::writeColumn(columnAt(s), columnAt(sectorB));
    }

    const uint32_t now = micros();
    if (lastDeadline != 0 && lastSector >= 0) {
      const int skipped = (s - lastSector + ANGLES) % ANGLES;
      if (skipped > 1) {
        columnsLate = columnsLate + 1;
        const uint32_t late = now - lastDeadline;
        if (late > worstLateUs) worstLateUs = late;
      }
    }
    lastDeadline = now;
    lastSector = s;
    columnsPainted = columnsPainted + 1;
  }
}

void printBootBanner() {
  Serial.println(F("\n=== hologram fan -- rotor firmware (stage 2a) ==="));
  Serial.printf("chip      : %s rev %d, %d core(s) @ %lu MHz\n",
                ESP.getChipModel(), ESP.getChipRevision(), ESP.getChipCores(),
                (unsigned long)getCpuFrequencyMhz());
  Serial.printf("flash     : %u bytes (%u MB)\n", ESP.getFlashChipSize(),
                ESP.getFlashChipSize() / (1024 * 1024));
  Serial.printf("heap free : %u bytes\n", ESP.getFreeHeap());

  /* THE question this build exists to answer. A 64-frame animation is 1.1 MB;
   * the S3 has 512 KB of SRAM. Without PSRAM, "push once to the rotor and play
   * locally" is off the table and animations have to be capped or streamed. */
  const size_t psram = ESP.getPsramSize();
  Serial.printf("PSRAM     : %u bytes", psram);
  if (psram == 0) {
    Serial.println(F("  <-- NONE FOUND.\n"
                     "            A 1.1 MB animation cannot live on the rotor.\n"
                     "            Either this is the N4 board, or PSRAM is not\n"
                     "            enabled. Frame counts will have to be capped."));
  } else {
    Serial.printf(" (%u MB free %u)  <-- enough for a 1.1 MB animation\n",
                  psram / (1024 * 1024), ESP.getFreePsram());
  }

  Serial.printf("geometry  : %d LEDs, %d per arm, %d arms, %d sectors\n",
                LEDS_TOTAL, LEDS_PER_ARM, ARMS, ANGLES);
  Serial.printf("SPI       : %lu Hz, data GPIO%d, clock GPIO%d\n",
                (unsigned long)LED_SPI_HZ, PIN_LED_DATA, PIN_LED_CLOCK);
  Serial.printf("content   : %s%s\n",
                content::available() ? content::id() : "none -- showing test pattern",
                content::needsResample() ? "  (RESAMPLED, radial mismatch)" : "");
  Serial.printf("fallback  : %s\n",
                displayMode == Mode::REV_COLOUR
                    ? "REV_COLOUR -- whole strip R/G/B, one step per revolution"
                    : "POV_PATTERN -- colour wheel + white spoke");
  Serial.printf("link      : ESP-NOW ch %u, my MAC %s, stator %s\n",
                LINK_CHANNEL, WiFi.macAddress().c_str(),
                rotorlink::paired() ? "PAIRED" : "not found yet");
  Serial.println(F("content   : website now generates radial=18 to match this\n"
                   "            strip, so no resampling is needed. Presets were\n"
                   "            regenerated; a 64-frame animation is 608 KB."));
  Serial.println();
}

/* ===========================================================================
 *  SERVICE TASK -- everything that is not the display, on the OTHER core
 * ===========================================================================
 * This was the body of loop(), and having it there was a latent bug that only
 * became reachable once the content push started working.
 *
 * Arduino's loopTask runs on CORE 1 at PRIORITY 1 (CONFIG_ARDUINO_RUNNING_CORE
 * is 1). displayTask is pinned to core 1 at priority 20, and on the content
 * path it never blocks -- it polls rotorsync::sector() and calls taskYIELD(),
 * which reschedules only among tasks of EQUAL OR HIGHER priority. So the
 * moment an image was playing on a spinning arm, loopTask stopped being
 * scheduled at all, and with it the 5 Hz telemetry send in rotorlink::tick().
 * The stator heard nothing for LINK_TIMEOUT_MS and reported the rotor link
 * down, while ESP-NOW RECEIVE kept working perfectly -- those callbacks run on
 * the WiFi task on core 0. Hence the odd signature: the stator could still
 * command a rotor it could no longer hear, and selecting the sensor test
 * "fixed" the link, because that branch calls vTaskDelay() and lets loopTask
 * run again.
 *
 * Before the content push was fixed, content::available() was never true, the
 * display loop always took the REV_COLOUR branch with its vTaskDelay(1), and
 * loopTask always ran. The bug was unreachable rather than absent, which is
 * why telemetry had always looked fine.
 *
 * So this now lives where this file's own header comment always claimed it
 * did: core 0, with WiFi and ESP-NOW, never touching column timing. The
 * serial diagnostics come along for the ride, which matters -- 'd', 'c', 'i'
 * and the 1 Hz status line were dead exactly when an image was playing, which
 * is exactly when you want them.
 * ======================================================================== */
void serviceTask(void *) {
  uint32_t lastPrint = 0;
  uint32_t lastColumns = 0;
  uint32_t lastLinkPrint = 0;
  bool bannerShown = false;

  for (;;) {
    rotorlink::tick();
    blackbox::tick();

    // Late enough that the serial monitor is attached and will actually see it.
    if (!bannerShown && millis() > 2000) {
      bannerShown = true;
      printBootBanner();
    }

    while (Serial.available()) {
      const int c = Serial.read();
      if (c == 'i' || c == 'I') printBootBanner();
      else if (c == 'm' || c == 'M') {
        displayMode = (displayMode == Mode::REV_COLOUR) ? Mode::POV_PATTERN
                                                        : Mode::REV_COLOUR;
        Serial.printf("[KEY] mode = %s\n",
                      displayMode == Mode::REV_COLOUR
                          ? "REV_COLOUR (whole strip, R/G/B once per revolution)"
                          : "POV_PATTERN (colour wheel + white spoke)");
      }
      else if (c == 'c' || c == 'C') {
        content::clear();
        Serial.println(F("[KEY] content cleared -- back to the test pattern"));
      }
      else if (c == 'd' || c == 'D') blackbox::dump();
      else if (c == 'z' || c == 'Z') blackbox::clear();
      else if (c == 'f' || c == 'F') {
        freeRunRpm = freeRunRpm ? 0 : 120;
        Serial.printf("[KEY] free-run %s\n",
                      freeRunRpm ? "ON at 120 rpm (Hall ignored)" : "OFF");
      } else if (c == '<') {
        if (freeRunRpm > 20) freeRunRpm -= 20;
        Serial.printf("[KEY] free-run %lu rpm\n", (unsigned long)freeRunRpm);
      } else if (c == '>') {
        freeRunRpm += 20;
        Serial.printf("[KEY] free-run %lu rpm\n", (unsigned long)freeRunRpm);
      } else if (c == 'b' || c == 'B') {
        apa102::setBrightness(apa102::brightness() >= 31 ? 1
                                                         : apa102::brightness() * 2);
        Serial.printf("[KEY] brightness %u/31\n", apa102::brightness());
      } else if (c == '?') {
        Serial.println(F("keys: i=info  b=brightness  m=fallback pattern  f=free-run\n"
                         "      < >=free-run rpm  c=clear content  d=dump  z=clear log"));
      }
    }

    if (millis() - lastPrint >= 1000) {
      lastPrint = millis();
      const uint32_t cols = columnsPainted;
      const uint32_t rate = cols - lastColumns;
      lastColumns = cols;

      if (freeRunRpm > 0) {
        Serial.printf("FREE-RUN %lu rpm (Hall ignored) | %4lu col/s (want %4lu) | "
                      "wire %3lu us | brightness %u/31\n",
                      (unsigned long)freeRunRpm, (unsigned long)rate,
                      (unsigned long)(freeRunRpm * ANGLES / 60),
                      (unsigned long)apa102::lastWriteUs(), apa102::brightness());
      } else if (rotorsync::stopped()) {
        /* Report the LAST measured figures, not just zeros. A few seconds of
         * hand-spinning is over long before the next print, and a bare
         * "stopped" line threw away the only evidence that it worked. */
        Serial.printf("stopped  | revs %lu  rejected %lu  overrun %lu | "
                      "last %5.1f rpm (period %lu us) | columns painted %lu\n",
                      (unsigned long)rotorsync::revolutions(),
                      (unsigned long)rotorsync::rejectedPulses(),
                      (unsigned long)rotorsync::overruns(),
                      rotorsync::periodUs() ? 60000000.0f / rotorsync::periodUs() : 0.0f,
                      (unsigned long)rotorsync::periodUs(),
                      (unsigned long)cols);
      } else {
        Serial.printf(
            "%6.1f rpm | period %6lu us | %4lu col/s (want %4.0f) | "
            "wire %3lu us | late %lu (worst %lu us) | overrun %lu | rejected %lu\n",
            rotorsync::rpm(), (unsigned long)rotorsync::periodUs(), (unsigned long)rate,
            rotorsync::rpm() / 60.0f * ANGLES, (unsigned long)apa102::lastWriteUs(),
            (unsigned long)columnsLate, (unsigned long)worstLateUs,
            (unsigned long)rotorsync::overruns(),
            (unsigned long)rotorsync::rejectedPulses());
      }
    }
    /* Link state on its own cadence. begin()'s own prints land in the window
     * before the serial monitor attaches and are lost, so this is the line that
     * actually tells you whether the two boards found each other. */
    if (millis() - lastLinkPrint >= 5000) {
      lastLinkPrint = millis();
      if (rotorlink::paired()) {
        Serial.printf("[LINK] paired | content: %s | sent %lu  recv %lu  failed %lu | "
                      "brightness %u  display %s\n",
                      rotorlink::sensorTest()
                          ? "SENSOR TEST (R/G/B per revolution)"
                          : content::available()
                          ? content::id()
                          : (content::receiving() ? "<receiving>" : "none (test pattern)"),
                      (unsigned long)rotorlink::sent(),
                      (unsigned long)rotorlink::received(),
                      (unsigned long)rotorlink::sendFailures(),
                      rotorlink::commandedBrightness(),
                      rotorlink::displayEnabled() ? "on" : "off");
      } else {
        Serial.printf("[LINK] NOT paired -- no beacon heard on channel %u "
                      "(recv %lu)\n",
                      LINK_CHANNEL, (unsigned long)rotorlink::received());
      }
    }

    vTaskDelay(pdMS_TO_TICKS(SERVICE_TICK_MS));
  }
}

}  // namespace

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  Serial.begin(115200);
  const uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(200);

  /* The banner is NOT printed here. `pio run -t upload -t monitor` resets the
   * board and only then opens the terminal, so anything setup() prints in its
   * first moments goes into the void -- which is exactly what happened on the
   * first run: the whole banner, including the PSRAM line the build existed to
   * produce, was lost. It is printed from loop() at t=2 s instead, and can be
   * reprinted any time with 'i'. */
  buildPattern();
  Serial.printf("[PATTERN] built, %u bytes\n", (unsigned)sizeof(pattern));

  apa102::begin();
  apa102::setBrightness(DEFAULT_BRIGHTNESS);
  Serial.printf("[LED] driver up, brightness %u/31, %lu us per column on the wire\n",
                apa102::brightness(), (unsigned long)apa102::lastWriteUs());

  rotorsync::begin();

  // ESP-NOW to the stator. Runs on core 0; the display owns core 1.
  content::begin();
  blackbox::begin();
  rotorlink::begin();

  xTaskCreatePinnedToCore(displayTask, "display", DISPLAY_TASK_STACK, nullptr,
                          DISPLAY_TASK_PRIORITY, nullptr, DISPLAY_TASK_CORE);
  Serial.printf("[DISPLAY] task pinned to core %d at priority %u\n",
                DISPLAY_TASK_CORE, (unsigned)DISPLAY_TASK_PRIORITY);

  /* Telemetry, blackbox and serial, on the OTHER core. Must not be core 1:
   * displayTask never blocks and would starve this outright, which is exactly
   * the bug that made the rotor link drop whenever an image was playing.
   * config.h static_asserts the two cores differ. */
  xTaskCreatePinnedToCore(serviceTask, "service", SERVICE_TASK_STACK, nullptr,
                          SERVICE_TASK_PRIORITY, nullptr, SERVICE_TASK_CORE);
  Serial.printf("[SERVICE] task pinned to core %d at priority %u\n",
                SERVICE_TASK_CORE, (unsigned)SERVICE_TASK_PRIORITY);

  Serial.println(F("\nDEFAULT MODE is the Hall diagnostic: the whole strip should\n"
                   "step cleanly RED -> GREEN -> BLUE, exactly one change per\n"
                   "revolution. Skipped colours mean missed pulses; several\n"
                   "changes per revolution mean the sensor is multi-triggering.\n"
                   "Press 'm' for the POV pattern once the sensor is trusted.\n"));
  Serial.println(F("POV pattern: a colour wheel with ONE white spoke\n"
                   "standing still at the Hall magnet. Reading it:\n"
                   "  spoke drifts round   -> period estimate wrong\n"
                   "  spoke smears         -> Hall timing jitter\n"
                   "  two spokes           -> arm offset wrong (not 180 deg)\n"
                   "  bright end at hub    -> flip ARM_A_TIP_AT_INDEX_ZERO\n"));
}

void loop() {
  /* Deliberately empty -- see serviceTask(). loopTask shares core 1 with
   * displayTask, which owns that core outright and will starve anything put
   * here the moment content is playing. Nothing may live in loop() on this
   * firmware; sleeping keeps it out of the way. */
  vTaskDelay(pdMS_TO_TICKS(1000));
}

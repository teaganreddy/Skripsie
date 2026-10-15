#include "apa102.h"

#include <SPI.h>

namespace {

// 4 start + 4 per LED + end frame. ceil(n/16) clock-stretch bytes, min 4.
const size_t END_BYTES = max(4, (LEDS_TOTAL + 15) / 16);
const size_t FRAME_BYTES = 4 + (size_t)LEDS_TOTAL * 4 + END_BYTES;

uint8_t frame[FRAME_BYTES];
uint8_t brightnessLevel = DEFAULT_BRIGHTNESS;
uint32_t lastUs = 0;

// Byte offset of the LED frame for a given strip index.
inline size_t slot(int stripIndex) { return 4 + (size_t)stripIndex * 4; }

/* Maps a radial position on an arm to its index on the physical strip.
 *
 * The strip runs continuously through the hub, so the two arms are indexed in
 * opposite directions:  arm A is tip-to-hub over 0..17, arm B is hub-to-tip
 * over 18..35. Radial r is always hub-first (0 = hub) to match .povf. */
inline int stripIndexFor(int arm, int r) {
  if (arm == 0) {
    return ARM_A_TIP_AT_INDEX_ZERO ? (LEDS_PER_ARM - 1 - r) : r;
  }
  return ARM_A_TIP_AT_INDEX_ZERO ? (LEDS_PER_ARM + r)
                                 : (LEDS_TOTAL - 1 - r);
}

void writeArm(int arm, const uint8_t *rgb) {
  for (int r = 0; r < LEDS_PER_ARM; r++) {
    uint8_t *p = &frame[slot(stripIndexFor(arm, r))];
    if (rgb) {
      // .povf stores R,G,B; the strip wants B,G,R after the header byte.
      p[1] = rgb[r * 3 + 2];  // blue
      p[2] = rgb[r * 3 + 1];  // green
      p[3] = rgb[r * 3 + 0];  // red
    } else {
      p[1] = p[2] = p[3] = 0;
    }
  }
}

}  // namespace

namespace apa102 {

void begin() {
  memset(frame, 0, 4);                                  // start frame
  memset(frame + FRAME_BYTES - END_BYTES, 0xFF, END_BYTES);  // end frame
  for (int i = 0; i < LEDS_TOTAL; i++) {
    uint8_t *p = &frame[slot(i)];
    p[0] = 0xE0 | (brightnessLevel & 0x1F);
    p[1] = p[2] = p[3] = 0;
  }

  /* POWER SEQUENCING. Hold data and clock high-impedance for a moment before
   * the SPI peripheral takes them over.
   *
   * The strip runs at 5 V and the S3's outputs are 3.3 V, so if the MCU is
   * already driving these lines while the strip's 5 V rail is absent or still
   * rising, current flows through the first LED's input protection diodes
   * into its unpowered supply. That can bring the chip up latched -- the
   * symptom is a strip that stays dark until its supply is ramped up slowly
   * from zero -- and it is the same mechanism that destroyed unit 2's
   * original first LED (gotcha 37).
   *
   * High-Z costs nothing and covers the normal case where the MCU and the
   * strip come off the same battery and the 3.3 V rail wins the race. It does
   * NOT cover applying strip power long after boot: for that, and to protect
   * the GPIOs from 5 V backfeed, fit ~330 ohm series resistors in both lines. */
  pinMode(PIN_LED_DATA, INPUT);
  pinMode(PIN_LED_CLOCK, INPUT);
  delay(LED_POWER_SETTLE_MS);

  // (sck, miso, mosi, ss). MISO unused -- the strip is write-only.
  SPI.begin(PIN_LED_CLOCK, -1, PIN_LED_DATA, -1);

  clear();
}

/* The SPI clock, adjustable at runtime.
 *
 * The strip is 5 V driven straight from 3.3 V logic with the level shifter
 * bypassed, which config.h already calls marginal: 3.3 V barely clears an
 * APA102's logic-high threshold, and the margin gets worse as edges get
 * faster and the wiring run gets longer. Two units with different wiring can
 * therefore disagree at the same clock. Being able to step it down without
 * reflashing turns "is this a level-margin problem?" into one key press. */
uint32_t clockHz = LED_SPI_HZ;

/* Lowest the 'k' key walks down to. 50 kHz is 160x slower than the default:
 * at that speed rise times, wire length and ringing cannot plausibly be the
 * problem, so a strip that is still dark is not failing on edge quality. It
 * says nothing about the DC logic THRESHOLD, which is what the 3.3 V drive
 * into a 5 V strip is marginal on. */

void setClockHz(uint32_t hz) { clockHz = hz ? hz : LED_SPI_HZ; }
uint32_t clockHzNow() { return clockHz; }

void setBrightness(uint8_t level) {
  brightnessLevel = min<uint8_t>(level, 31);
  for (int i = 0; i < LEDS_TOTAL; i++) {
    frame[slot(i)] = 0xE0 | (brightnessLevel & 0x1F);
  }
}

uint8_t brightness() { return brightnessLevel; }

void writeColumn(const uint8_t *armA, const uint8_t *armB) {
  writeArm(0, armA);
  writeArm(1, armB);

  const uint32_t t0 = micros();
  SPI.beginTransaction(SPISettings(clockHz, MSBFIRST, SPI_MODE0));
  SPI.writeBytes(frame, FRAME_BYTES);
  SPI.endTransaction();
  lastUs = micros() - t0;
}

void fill(uint8_t r, uint8_t g, uint8_t b) {
  for (int i = 0; i < LEDS_TOTAL; i++) {
    uint8_t *p = &frame[slot(i)];
    p[1] = b;  // strip order after the header byte is B, G, R
    p[2] = g;
    p[3] = r;
  }
  const uint32_t t0 = micros();
  SPI.beginTransaction(SPISettings(clockHz, MSBFIRST, SPI_MODE0));
  SPI.writeBytes(frame, FRAME_BYTES);
  SPI.endTransaction();
  lastUs = micros() - t0;
}

void clear() { writeColumn(nullptr, nullptr); }

uint32_t lastWriteUs() { return lastUs; }

}  // namespace apa102

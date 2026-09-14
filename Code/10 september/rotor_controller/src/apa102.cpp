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

  // (sck, miso, mosi, ss). MISO unused -- the strip is write-only.
  SPI.begin(PIN_LED_CLOCK, -1, PIN_LED_DATA, -1);

  clear();
}

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
  SPI.beginTransaction(SPISettings(LED_SPI_HZ, MSBFIRST, SPI_MODE0));
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
  SPI.beginTransaction(SPISettings(LED_SPI_HZ, MSBFIRST, SPI_MODE0));
  SPI.writeBytes(frame, FRAME_BYTES);
  SPI.endTransaction();
  lastUs = micros() - t0;
}

void clear() { writeColumn(nullptr, nullptr); }

uint32_t lastWriteUs() { return lastUs; }

}  // namespace apa102

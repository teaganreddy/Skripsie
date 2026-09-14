#pragma once
/*
 * apa102.h -- hand-rolled APA102 / DotStar driver.
 *
 * Written from the protocol rather than pulled in as a library, for two
 * independent reasons:
 *
 *   1. FR12 requires the display driving to be custom-developed.
 *   2. Adafruit_DotStar could not do this job anyway. Its show() blocks for
 *      the whole frame, applies brightness by scaling RGB in software (losing
 *      colour resolution), and gives no way to prepare the next column while
 *      the current one is going out. A POV display needs a fixed cadence with
 *      a column ready at every deadline.
 *
 * Wire format, per the APA102 datasheet:
 *
 *      start frame   4 bytes of 0x00
 *      LED frame     1 byte  111BBBBB   (3 marker bits, 5-bit global current)
 *                    1 byte  blue
 *                    1 byte  green
 *                    1 byte  red        <- BGR order, as LED_Test found
 *      end frame     enough clock cycles to shift data to the far end
 *
 * The end frame is the part people get wrong. Each LED delays the clock by
 * half a cycle, so the last LED needs n/2 extra clocks before it latches --
 * ceil(n/16) bytes. For 36 LEDs that is 3, and 4 is sent for margin. This
 * matches what Adafruit_DotStar emits, which is known to work on this strip.
 *
 * Brightness uses the hardware 5-bit field rather than scaling RGB, so the
 * colour data goes out untouched. That matters here: the browser has already
 * applied gamma 2.2 before writing the .povf, and API.md is explicit that the
 * firmware must not touch those bytes again.
 */

#include <Arduino.h>

#include "config.h"

namespace apa102 {

// Sets up SPI on PIN_LED_CLOCK / PIN_LED_DATA and blanks the strip.
void begin();

// 0-31, the APA102's native global-current field.
void setBrightness(uint8_t level);
uint8_t brightness();

/* Paints one angular column.
 *
 * `armA` and `armB` each point at LEDS_PER_ARM RGB triples ordered hub-first
 * (index 0 = hub, LEDS_PER_ARM-1 = tip), which is the same convention the
 * .povf format uses for its radial axis. Passing nullptr blanks that arm.
 *
 * Blocking, and deliberately so: it runs on a dedicated core where 152 us out
 * of a 476 us budget costs nothing, and blocking keeps the timing honest and
 * the code simple. DMA is an optimisation available later if the budget ever
 * tightens.
 */
void writeColumn(const uint8_t *armA, const uint8_t *armB);

/* Every LED the same colour, in one transfer. Used by the revolution-colour
 * diagnostic mode, which paints once per revolution rather than 180 times --
 * so it also drops the strip's duty cycle and SPI traffic by a large factor. */
void fill(uint8_t r, uint8_t g, uint8_t b);

// All LEDs off, brightness field left alone.
void clear();

// Microseconds the last writeColumn() spent on the wire. Bring-up visibility.
uint32_t lastWriteUs();

}  // namespace apa102

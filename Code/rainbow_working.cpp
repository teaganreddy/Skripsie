// APA102 / DotStar on ESP32-S3 via HARDWARE SPI at full speed (8 MHz).
// Rainbow animation scrolling across the strip.
//
// Wiring: strip DATA (DI) -> IO15/MOSI, strip CLOCK (CI) -> IO17/SCK,
// driven directly at 3.3V (BSS138 level shifter bypassed).
//
// Pin remap note: this board's default hardware-SPI pins are not IO15/IO17,
// so we call SPI.begin(SCK, MISO, MOSI, SS) with our pins BEFORE strip.begin().
// The ESP32 core's SPI.begin() returns early if the bus is already started,
// so Adafruit_DotStar's internal no-arg SPI.begin() won't clobber our mapping.

#include <Adafruit_DotStar.h>
#include <SPI.h>

#define NUMPIXELS 36
#define DATAPIN   15   // MOSI
#define CLOCKPIN  17   // SCK

// Hardware-SPI constructor (uses the default 'SPI' object at 8 MHz).
Adafruit_DotStar strip(NUMPIXELS, DOTSTAR_BGR);

const uint16_t HUE_STEP  = 256;   // hue advance per frame (higher = faster scroll)
const uint8_t  RAINBOWS  = 1;     // how many full rainbows span the strip at once
const uint16_t FRAME_MS  = 20;    // ~50 fps

const uint8_t  BRIGHT_75 = 191;   // 75% target brightness (191/255)
const uint16_t FADEIN_MS = 1500;  // startup ramp to soften LED turn-on inrush

void setup() {
  Serial.begin(115200);
  uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 3000) delay(10);
  delay(200);
  Serial.println("checkpoint: Serial up");

  pinMode(LED_BUILTIN, OUTPUT);

  // Map hardware SPI onto the pins the strip is wired to, BEFORE strip.begin().
  SPI.begin(CLOCKPIN, -1, DATAPIN, -1);   // (sck, miso, mosi, ss)

  strip.begin();               // hardware SPI @ 8 MHz
  strip.setBrightness(0);      // start dark; loop() ramps up to BRIGHT_75
  strip.clear();
  strip.show();
  Serial.println("checkpoint: strip initialized (HW SPI 8 MHz), fading in rainbow");
}

void loop() {
  static uint16_t firstHue = 0;
  static uint32_t bootMs = millis();

  // Startup fade-in: ramp brightness 0 -> BRIGHT_75 over FADEIN_MS so the
  // strip's current rises gradually instead of stepping on all at once.
  uint32_t elapsed = millis() - bootMs;
  uint8_t b = (elapsed >= FADEIN_MS)
                ? BRIGHT_75
                : (uint32_t)BRIGHT_75 * elapsed / FADEIN_MS;
  strip.setBrightness(b);

  // rainbow(firstHue, reps, saturation, brightness, gammify)
  strip.rainbow(firstHue, RAINBOWS, 255, 255, true);
  strip.show();

  firstHue += HUE_STEP;        // wraps naturally at 65536 -> seamless scroll
  digitalWrite(LED_BUILTIN, (firstHue >> 13) & 1);   // slow heartbeat
  delay(FRAME_MS);
}

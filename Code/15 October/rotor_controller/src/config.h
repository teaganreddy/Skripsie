#pragma once
/*
 * config.h -- rotor-side constants, all in one place.
 *
 * Pins are taken from the working LED_Test and hall_effect_sensor projects,
 * not guessed.
 */

#include <Arduino.h>

/* Which fan unit this rotor belongs to. It pairs only with the stator whose
 * beacon carries this unit id. Build unit 2's rotor with -DUNIT_ID=2 (the
 * `unit2` environment in platformio.ini). */
#ifndef UNIT_ID
#define UNIT_ID 1
#endif

// ===========================================================================
//  PINS  -- as actually wired and tested
// ===========================================================================

/* Strip DI / CI. GPIO15 (MOSI) and GPIO17 (SCK) on the boards built so far;
 * overridable per environment with -D PIN_LED_DATA_GPIO / -D PIN_LED_CLOCK_GPIO
 * in case a board revision maps the header differently. */
#ifndef PIN_LED_DATA_GPIO
#define PIN_LED_DATA_GPIO 15
#endif
#ifndef PIN_LED_CLOCK_GPIO
#define PIN_LED_CLOCK_GPIO 17
#endif
/* How long the strip's data and clock are left high-impedance at boot, so the
 * 5 V rail can come up before the 3.3 V outputs start driving into it. See
 * the power-sequencing note in apa102.cpp. */
static const uint32_t LED_POWER_SETTLE_MS = 300;

static const int PIN_LED_DATA = PIN_LED_DATA_GPIO;    // strip DI
static const int PIN_LED_CLOCK = PIN_LED_CLOCK_GPIO;  // strip CI
/* D10 = GPIO14 on both units unless overridden. Override per environment with
 * -D PIN_HALL_GPIO=<n> in platformio.ini if a board's GPIO14 is damaged. */
#ifndef PIN_HALL_GPIO
#define PIN_HALL_GPIO 14
#endif
static const int PIN_HALL = PIN_HALL_GPIO;  // INPUT_PULLUP; pulls LOW on magnet.

/* SPI clock for the strip.
 *
 * Starting at 8 MHz because that is what LED_Test runs at today and is known
 * to work on this wiring. There is room to go much faster -- see the budget
 * below -- but the strip is powered from 5 V and driven straight from 3.3 V
 * GPIO with the BSS138 level shifter bypassed, which is marginal by design.
 * 3.3 V clears an APA102's logic-high threshold with little to spare, and the
 * margin gets worse as edges get faster and the run gets longer.
 *
 * So: raise this only while watching for glitches (wrong colours, flicker,
 * the last LEDs misbehaving). If it breaks above 8 MHz, that is the level
 * shifter asking to be put back, not a firmware problem. */
static const uint32_t LED_SPI_HZ = 8000000;

// ===========================================================================
//  DISPLAY GEOMETRY
// ===========================================================================

/* 36 LEDs on one continuous strip laid across the full diameter, so the middle
 * of the strip is the hub and each half is an arm of 18.
 *
 * IN STEP with the website as of 2026-09-09: web/js/config.js is LEDS_TOTAL 36
 * / LEDS_PER_ARM 18 and every regenerated .povf preset is radial=18, so the
 * resample path in content.cpp is a fallback for older files, not the normal
 * one. (This block previously warned that the website said 64/32 and every
 * preset was radial=32. That was true before the presets were regenerated and
 * is now false in every particular -- it was left here long enough to be
 * actively misleading.) */
/* PER UNIT. Unit 1 has the full 36. Unit 2's strip lost its first LED to a
 * shorted input during the build (2026-09-17); rather than run one arm a
 * pixel short, one LED was taken off the far end too and the strip re-centred
 * on the hub, so it is 34: still symmetric, 17 per arm. The website keeps
 * generating 18-radial content and content.cpp resamples it onto the shorter
 * arm (nearest-neighbour); unit 2 simply lacks the outermost ring.
 *
 * Set from platformio.ini (-D LEDS_TOTAL=34 in the unit2 environment). Must
 * be even: the hub sits exactly between the two middle LEDs. */
#ifndef LEDS_TOTAL
#define LEDS_TOTAL 36
#endif
static_assert(LEDS_TOTAL % 2 == 0, "LEDS_TOTAL must be even: the hub is between the two middle LEDs");
static const int LEDS_PER_ARM = LEDS_TOTAL / 2;  // 18 on unit 1, 17 on unit 2
static const int ARMS = 2;

/* Angular sectors per full revolution. Matches ANGULAR_STEPS in the website's
 * config.js and the `angles` field in every .povf header. */
static const int ANGLES = 180;

/* Strip index layout. The strip runs continuously through the hub, so one arm
 * is indexed outward-to-inward and the other inward-to-outward:
 *
 *      index 0 .............. 17 | 18 .............. 35
 *      arm A tip ....... arm A hub | arm B hub ...... arm B tip
 *
 * If the image comes out radially mirrored on one arm, this is the assumption
 * to flip. */
static const bool ARM_A_TIP_AT_INDEX_ZERO = true;

/* Physical-to-content angle calibration, PER UNIT. Angle 0 of a frame is
 * wherever this unit's Hall magnet is glued, and the arm may turn the other
 * way from what the website assumes. Neither is knowable from software.
 *
 * The live value is held by the STATOR (its NVS, per unit) and arrives in
 * every command; it is set from the website's Diagnostics tab with the
 * "Calibrate" preset showing (an arrow that must point to the viewer's right
 * with a dot above it). These are only the defaults for an unpaired rotor on
 * the bench. */
#ifndef SECTOR_OFFSET
#define SECTOR_OFFSET 0     // sectors, 0..179, 2 deg each
#endif
#ifndef SECTOR_REVERSE
#define SECTOR_REVERSE 0    // 1 if the arm turns the other way
#endif

// ===========================================================================
//  HALL ANGULAR REFERENCE
// ===========================================================================

/* One magnet on the stator, so one falling edge per revolution. If a second
 * magnet is ever added this must change, and so must the period maths --
 * the ISR would then be timing half-revolutions. */
static const int MAGNETS_PER_REV = 1;

/* Glitch rejection in the Hall ISR, microseconds. 2 ms is what LED_Test uses
 * and it is nowhere near limiting: at 700 rpm a revolution is 85.7 ms, so this
 * blanks 2.3% of it and would only bite above ~30000 rpm. */
static const uint32_t HALL_DEBOUNCE_US = 2000;

/* "Stopped" is judged against 2.5 revolutions of the LAST measured period,
 * not a fixed time -- clamped to this range.
 *
 * A fixed 1.5 s was the first attempt and it was wrong: hand-spinning the arm
 * takes seconds per revolution, so anything below ~40 rpm permanently read as
 * stopped and the display never painted. Scaling with the measured period
 * makes the same rule work at 10 rpm on a bench and at 700 rpm in service --
 * at 250 rpm it comes out at 0.6 s, hand-spinning at 10 rpm gives 15 s. */
static const uint32_t HALL_TIMEOUT_MIN_US = 500000;    // 0.5 s
static const uint32_t HALL_TIMEOUT_MAX_US = 15000000;  // 15 s

/* Sanity bounds on a measured revolution period, microseconds. This is a NOISE
 * filter, not a speed limit -- deliberately wide.
 *
 *      20 ms -> 3000 rpm        12 s -> 5 rpm
 *
 * It was originally 30 ms - 2 s (2000 - 30 rpm), which rejected essentially
 * every hand-spun revolution. Being permissive is the safer error here: a
 * wrong period costs ONE bad revolution and self-corrects at the next pulse,
 * whereas rejecting a good pulse leaves the angular zero stale and the whole
 * image drifts until the next accepted one. */
static const uint32_t PERIOD_MIN_US = 20000;
static const uint32_t PERIOD_MAX_US = 12000000;

// ===========================================================================
//  DISPLAY TASK
// ===========================================================================

/* The S3 is DUAL core, which is the luxury the C6 did not have. Core 1 is
 * given over to the display loop so column timing never competes with WiFi or
 * ESP-NOW (which live on core 0). No burst-and-yield gymnastics needed here.
 *
 * Timing budget, 36 LEDs at 8 MHz -> 152 us per column:
 *      150 rpm  ->  2222 us per column   (7%  duty)
 *      250 rpm  ->  1333 us per column   (11% duty)
 *      700 rpm  ->   476 us per column   (32% duty)
 * Comfortable even at the design speed, and 16 MHz would halve it again. */
static const int DISPLAY_TASK_CORE = 1;
static const UBaseType_t DISPLAY_TASK_PRIORITY = 20;
static const uint32_t DISPLAY_TASK_STACK = 4096;

/* Everything that is not the display: telemetry, the blackbox, serial.
 *
 * MUST NOT share a core with the display task. displayTask polls the
 * interpolated sector and calls taskYIELD(), which only reschedules among
 * tasks of EQUAL OR HIGHER priority -- so it never blocks and starves
 * everything below it on its core outright. That is fine, and is what "core 1
 * is given entirely to the display" means, but it does mean core 1 has room
 * for exactly one task. The static_assert below makes that a build error
 * rather than a silent 2 s telemetry timeout.
 *
 * Priority 3: above idle, far below the WiFi stack it shares core 0 with. */
static const int SERVICE_TASK_CORE = 0;
static const UBaseType_t SERVICE_TASK_PRIORITY = 3;
static const uint32_t SERVICE_TASK_STACK = 8192;  // as Arduino gives loopTask
static const uint32_t SERVICE_TICK_MS = 20;

static_assert(SERVICE_TASK_CORE != DISPLAY_TASK_CORE,
              "The service task cannot share a core with the display task: "
              "displayTask spins without ever blocking and would starve it, "
              "which stops rotor telemetry and reads as 'rotor link down'.");

// Global brightness, 0-31. The APA102's native 5-bit field, matching the
// `brightness` value in API.md. Relayed from the stator in stage 2b.
static const uint8_t DEFAULT_BRIGHTNESS = 8;

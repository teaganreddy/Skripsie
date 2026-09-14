#pragma once
/*
 * config.h -- every tunable in the firmware, in one place.
 *
 * Grouped so that the things you are likely to change during bring-up are
 * near the top and the things that encode tested, hard-won behaviour are
 * marked as such.
 */

#include <Arduino.h>

// ===========================================================================
//  WIFI ACCESS POINT  -- change these two lines to rename the fan's network
// ===========================================================================

/* Which fan unit this stator is. Carried in every beacon so a rotor or rail
 * pairs only with its own stator. Build the second unit with -DUNIT_ID=2. */
#ifndef UNIT_ID
#define UNIT_ID 1
#endif

static const char *AP_SSID = "HologramFan";
static const char *AP_PASSWORD = "spinme123";  // >= 8 chars, or the AP starts open
static const char *MDNS_HOSTNAME = "fan";      // -> http://fan.local

/* Set true to run the access point with NO password.
 *
 * Kept as a first-class option rather than a debug hack, because for an open
 * day it is arguably the better choice: nothing to print on a sign, nothing to
 * mistype, and there is nothing on this device worth protecting -- it serves
 * one page and spins a fan on an isolated network with no internet route.
 *
 * It is also the cleanest test for a WPA2 handshake problem. If an open AP
 * joins fine and a passworded one does not, the fault is in the handshake, not
 * in the radio or the DHCP server. */
static const bool AP_OPEN = true;

// Channel 1. ESP-NOW in stage 2 has to share this radio, and ESP-NOW peers
// must sit on the same channel as the AP, so fixing it now avoids a surprise
// later when the rotor cannot be reached because the AP picked a channel.
static const int AP_CHANNEL = 1;
static const int AP_MAX_CLIENTS = 4;

/* Log every probe request. Invaluable while the AP was refusing connections --
 * it is how you tell "the client cannot see us" from "the client sees us and
 * the handshake fails" -- but in a busy room it fires every couple of seconds
 * forever and buries the telemetry. Off now that the AP works; the useful
 * low-rate events (authorised / DHCP / disconnect-with-reason) stay on. */
static const bool WIFI_LOG_PROBES = false;

// ===========================================================================
//  MOTOR + DRIVER  -- from motor_test, which is tested and known good.
//  Do not change P/I/D or Tf without re-running the bench tests.
// ===========================================================================

static const int MOTOR_POLE_PAIRS = 7;  // 2804 outrunner: 12 slots / 14 poles
static const int PIN_IN1 = 1;           // D6
static const int PIN_IN2 = 18;          // D7
static const int PIN_IN3 = 14;          // D3
static const int PIN_EN = 8;            // D2

/* DRV8313 fault plumbing. Both active low, both broken out on the DRI0058.
 *
 * nFAULT is an open-drain output asserted for overcurrent AND overtemperature.
 * Without it an OCP trip is completely invisible to firmware -- the datasheet
 * says the affected channel stays disabled "until either assertion of nRESET
 * or the cycling of VM power", so a phase simply dies mid-spin with nothing
 * shown on the website. Read with an internal pull-up in case the board has
 * none of its own.
 *
 * nRESET must be HIGH for normal operation, and pulsing it LOW is the only way
 * to clear a latched OCP trip short of unplugging the barrel jack. It floats
 * to enabled on the DRV8313's internal pull-up, so the driver still works
 * correctly in the window before setup() drives it.
 */
static const int PIN_DRV_FAULT = 7;  // D11, input, active low
static const int PIN_DRV_RESET = 6;  // D12, output, active low

// How long nRESET is held low to clear a latched driver fault.
static const uint32_t DRV_RESET_PULSE_MS = 5;

static const float SUPPLY_VOLTAGE = 12.0f;  // 12 V 5 A mains adapter

/* AS5600 I2C clock.
 *
 * 100 kHz, and this is deliberate -- it is what the working motor_test has
 * ACTUALLY been running at, despite its comment claiming 400 kHz.
 *
 * motor_test calls Wire.setClock(400000) before Wire.begin() (which happens
 * inside sensor.init()). Arduino-ESP32's TwoWire::setClock() bails out at its
 * first line if the bus lock has not been created yet, i.e. before begin() --
 * so that call silently did nothing and the bus stayed at the 100 kHz default.
 *
 * The first version of this firmware called Wire.begin() explicitly first, so
 * the 400 kHz took effect for real. That was the only substantive change made
 * to a proven sensor setup, and the first bring-up run came back with garbage
 * angle data. 400 kHz into the AS5600 buried in the motor, down whatever cable
 * and pull-ups the FIT1034 provides, is the obvious suspect.
 *
 * Cost of 100 kHz: the 2-byte angle read goes from ~125 us to ~500 us, so
 * expect focLoopHz nearer 1.0-1.5 kHz than 3.3 kHz. That is still far inside
 * the 5 ms sensor-gap budget, so it buys reliability for something we have in
 * abundance. Try 400000 again only once the sensor is known good at 100 kHz,
 * and watch the raw angle for noise when you do. */
static const uint32_t I2C_CLOCK_HZ = 100000;

/* AS5600 slow-filter setting -- CONF bits 9:8. This is the dominant source of
 * commutation lag at speed and the default is the slowest option:
 *
 *      0 = 16x, 2.2 ms   <- AS5600 DEFAULT, what we were running
 *      1 =  8x, 1.1 ms
 *      2 =  4x, 0.55 ms
 *      3 =  2x, 0.286 ms
 *
 * The sensor delay becomes electrical lag proportional to speed: at 700 rpm and
 * 7 pole pairs the electrical frequency is 81.7 Hz, so 2.2 ms is 65 deg of lag
 * before the ~1 ms of loop sampling is even counted. Past 90 deg total the
 * current vector pushes sideways to the rotor and produces almost no torque,
 * which is why the motor could hold 250 rpm easily and died around 600.
 *
 * Toggle at runtime with 'w' to A/B it against the old behaviour. */
static const uint8_t AS5600_SLOW_FILTER = 3;  // 2x, 0.286 ms

/* Flight recorder, for sweeps you cannot watch live.
 *
 * The stator cannot have USB attached while the fan runs, so the useful numbers
 * (voltage.q against speed) were previously unobservable exactly when they
 * mattered. This records them to RAM and dumps CSV over serial afterwards --
 * the same trick the rotor's blackbox uses, and the same reason.
 *
 * 2500 samples at 2 Hz is ~20 minutes, which covers a full 50 rpm sweep with
 * room to spare. 22 bytes each = ~55 KB of the C6's 327 KB. */
static const size_t MOTOR_LOG_ENTRIES = 2500;
static const uint32_t MOTOR_LOG_INTERVAL_MS = 500;

/* Auto filter cycling. With no keyboard access while spinning, the 16x/8x/4x/2x
 * A/B cannot be driven by hand -- so the firmware can walk it instead, changing
 * SF every few seconds while the log records which setting was live. One
 * hands-off run at a fixed speed then contains all four. Armed with 'a'. */
static const uint32_t AUTO_FILTER_PERIOD_MS = 8000;

/* ---------------------------------------------------------------------------
 * VOLTAGE LIMIT -- READ THIS BEFORE RAISING IT.
 *
 * motor_test ran at 8 V on a bare shaft. That is NOT carried over here,
 * because the arithmetic against the driver's actual rating does not survive
 * a real load. The SimpleFOCmini (DRI0058) is a DRV8313, and per TI's
 * datasheet (SLVSBA5B) each half-bridge is rated:
 *
 *      2.5 A peak  /  1.75 A RMS continuous
 *          ...and the 1.75 A figure is quoted "with proper PCB heatsinking
 *          at 24 V and 25 C", which a bare SimpleFOCmini does not have.
 *      OCP trip level: 3 A min, 5 A max, 5 us deglitch
 *      Thermal shutdown: 150-180 C die
 *
 * With voltage_limit = 8 V and a 2.3 ohm phase resistance, using the MEASURED
 * KV of 333 rpm/V (the original sum here used the datasheet's 220, which put
 * back-EMF at 700 rpm at 3.2 V instead of 2.1 V -- the conclusion survives the
 * correction, but only just, so the corrected figures are the ones to quote):
 *      at 700 rpm  back-EMF 2.1 V  ->  (8-2.1)/2.3 ~ 2.6 A   (over PEAK)
 *      near stall  back-EMF ~0 V   ->   8/2.3      ~ 3.5 A   (over PEAK, and
 *                                                             inside the OCP
 *                                                             3-5 A window)
 *
 * Being inside the OCP window is the worst place to sit: a part at the low end
 * of the distribution trips and latches off, a part at the high end just
 * cooks. And per the datasheet an OCP trip disables the channel "until either
 * assertion of nRESET or the cycling of VM power" -- with nRESET floating and
 * nFAULT unwired, the firmware can neither detect nor clear it. It presents as
 * a phase dying mid-spin with no fault shown on the website.
 *
 * 5.0 V is chosen so that even the worst case (zero speed, PID saturated)
 * stays at 5/2.3 = 2.2 A: under the 2.5 A peak rating and under the 3 A OCP
 * minimum.
 *
 * RE-EXAMINED 2026-08-18 when the speed ceiling went to 700 rpm, and
 * DELIBERATELY LEFT AT 5.0 V. Two things fell out of that:
 *
 *   1. The worst case is unchanged. It sits at zero speed, where back-EMF is
 *      zero, and it is set by THIS constant -- not by the speed ceiling. A
 *      higher top speed does not push more current; back-EMF rising with speed
 *      means the same 5 V passes LESS current the faster the motor turns
 *      (1.26 A at 700 rpm against 2.17 A at standstill).
 *
 *   2. There is almost no room above 5 V anyway. 6 V would give 2.6 A at
 *      standstill, already over the 2.5 A peak rating, before any of the
 *      benefit shows up. With no current sensing anywhere in the system, the
 *      voltage limit IS the current limit, and it is doing the whole job.
 *
 * So if the arm turns out not to reach 700 rpm, raising this is the WRONG
 * lever -- the honest fixes are to reduce the arm's drag or accept a lower top
 * speed. Do not trade the driver for 50 rpm.
 *
 * A meter reading across two motor leads came back 6.6-6.7 ohm. Taken at face
 * value that is 3.3 ohm per phase for a wye winding -- HIGHER than the
 * datasheet's 2.3, and well clear of the 1.15 ohm worst case that would have
 * doubled every current above. The meter is not trusted at these values, but
 * it points the safe way, so the arithmetic above stands as the conservative
 * case.
 *
 * ---------------------------------------------------------------------------
 * MEASURED, 2026-07-30, bare shaft. This settles the question.
 *
 *      200 rpm steady state:  voltage.q = 0.61 V   (of 8 V available -- 7.6%)
 *       50 rpm steady state:  voltage.q = 0.08-0.26 V
 *
 * Even on the pessimistic reading -- all of Uq across the winding, no back-EMF
 * at all -- 0.61 V / 2.3 ohm is 0.27 A. The tested envelope is nowhere near
 * this driver's limits, and 5.0 V is ample headroom over 0.61 V.
 *
 * So the concern is NOT the tested regime. It is entirely about (a) what the
 * arm's drag adds at speed, and (b) the ramp-up transient, where back-EMF is
 * near zero and only the ramp rate limits current. Both are addressed by
 * RAMP_RPM_PER_S below and the procedure in BRINGUP.md.
 *
 * This also retires motor_test's "bumped from 7 -- 60rpm target was
 * voltage-saturating" note: at 50 rpm the steady-state demand is under 0.3 V,
 * so whatever was saturating was a transient during a step change, not a
 * standing requirement. The 60 rpm/s ramp here avoids that class of transient.
 * ------------------------------------------------------------------------ */
static const float MOTOR_VOLTAGE_LIMIT = 5.0f;

// Tested gains. See motor_test/src/main.cpp for the full tuning notes:
// friction and cogging dominate below ~10 rpm and want I=2.0, but I=2.0
// causes a slow sustained oscillation above ~60 rpm, so 0.4 it is.
static const float PID_P = 0.2f;
static const float PID_I = 0.4f;
static const float PID_D = 0.0f;
static const float LPF_TF = 0.02f;

static const float PHASE_RESISTANCE = 2.3f;  // ohms, from the FIT1034 datasheet

/* ---------------------------------------------------------------------------
 * KV IS MEASURED, NOT FROM THE DATASHEET. The datasheet's 220 rpm/V does not
 * describe this motor as SimpleFOC drives it, and using it produced a current
 * estimate that read NEGATIVE while the motor was plainly motoring.
 *
 * Measured 2026-07-30, bare shaft, steady state, via the web UI:
 *
 *      rpm   voltage.q
 *      130     0.42 V
 *      150     0.48 V
 *      190     0.60 V
 *      250     0.78 V
 *
 * Least-squares fit:   Uq = 0.00300 * rpm + 0.030 V
 *
 * Dead straight, so 1/slope gives an effective KV of 333 rpm/V -- 1.52x the
 * datasheet figure. Whether that is the motor genuinely differing from spec (a
 * ~50% KV error is not unusual on a cheap BLDC) or a scaling convention inside
 * SimpleFOC's voltage.q, the fit is what the firmware actually sees, so it is
 * what the firmware should use.
 *
 * The check that it is right: with KV = 333 the residual current
 * (Uq - backEMF)/R comes out at 13 mA at EVERY one of those four speeds. A
 * constant, positive, plausible no-load current is exactly what a bare shaft
 * with only bearing friction should draw -- and it is the constant-friction
 * load model rather than an aerodynamic one, which is the distinction that
 * matters for predicting behaviour with the arm fitted. With KV = 220 the same
 * arithmetic gave -88 mA at 150 rpm and -155 mA at 250.
 *
 * estCurrentA is therefore now physically meaningful. It is still an estimate
 * from a voltage and a datasheet resistance with NO current sensing anywhere in
 * the system, so the E09 fault deliberately still watches voltage saturation
 * instead -- that needs no model at all.
 * ------------------------------------------------------------------------ */
static const float KV_RATING = 333.0f;  // rpm per volt, MEASURED (datasheet: 220)

// ===========================================================================
//  SPEED ENVELOPE
// ===========================================================================

/* The website's slider range is a UI affordance, not a safety mechanism, so
 * every requested RPM is clamped to this band in firmware regardless of what
 * arrives over HTTP.
 *
 * ---------------------------------------------------------------------------
 * RAISED 250 -> 700 on 2026-08-18, to reach the POV design target. The
 * electrical case for it, worked from the MEASURED constants above:
 *
 *   back-EMF at 700 rpm      700 / 333 KV            = 2.10 V
 *   measured Uq at 700 rpm   0.00300 * 700 + 0.030   = 2.13 V  (BARE SHAFT)
 *   headroom over back-EMF   5.00 - 2.10             = 2.90 V
 *   => most current the driver can pass at 700 rpm
 *                            2.90 / 2.3 ohm          = 1.26 A
 *
 * 1.26 A is under the DRV8313's 1.75 A RMS continuous rating and a long way
 * under its 2.5 A peak and 3 A OCP floor. The reason is that back-EMF rises
 * with speed and eats the available voltage, so the FASTER the motor turns the
 * LESS current the same voltage limit can force through it.
 *
 * The consequence is the important one, and it is the opposite of the intuition
 * that "faster is more dangerous" electrically:
 *
 *      RAISING THE SPEED CEILING DOES NOT RAISE THE WORST-CASE CURRENT.
 *
 * The worst case has always been near ZERO speed with the PID saturated --
 * 5.0 / 2.3 = 2.17 A -- and that number is set by MOTOR_VOLTAGE_LIMIT, not by
 * this constant. It is unchanged by this edit. See the voltage-limit block
 * above for why 5.0 V must stay where it is: at 8 V the same sum gives 2.57 A
 * at 700 rpm, over the peak rating.
 *
 * WHAT IS NOT PROVEN: whether 2.90 V of headroom is enough TORQUE to overcome
 * the arm's drag at 700 rpm. That works out to about 0.036 N.m, or ~2.6 W of
 * drag power at 700 rpm, and the bare-shaft measurements say nothing about it
 * -- they found a friction-dominated load with no arm fitted. If the arm needs
 * more than that, the motor simply will not reach 700 and E09 (voltage
 * saturation) fires after 3 s. That is a safe and informative failure, not a
 * dangerous one, which is why this ceiling can be raised before the drag is
 * measured while MOTOR_VOLTAGE_LIMIT cannot.
 *
 * WHAT IS STILL A REAL RISK, and firmware cannot help with: MECHANICAL
 * BALANCE. Out-of-balance force goes as omega^2, so 700 rpm is 7.8x the force
 * at 250. E08 watches velocity ripple as a proxy but sits on a ~20 rpm encoder
 * noise floor and only catches gross imbalance. Balance the arm first, and go
 * up in 50 rpm steps watching rpmRipple, focMaxGapMs and voltageQ -- see
 * BRINGUP.md.
 * ------------------------------------------------------------------------ */
static const float RPM_MAX_ALLOWED = 700.0f;

/* 50 rpm, lowered from 60 on 2026-08-18 so the website's 50 rpm slider stop is
 * a speed the firmware will actually hold rather than one it silently clamps
 * up. It is the bottom stop, not a recommended operating point.
 *
 * The 2026-07-30 data says what to expect there: at a 50 rpm target the
 * measured speed wandered 48.4-56.4 rpm (8 rpm peak-to-peak) and sat about
 * 2.4 rpm above target. That matches motor_test's own tuning note -- friction
 * and cogging dominate down there and want I=2.0, while these gains (I=0.4)
 * are the "~60 rpm and above" set. So 50 rpm is sloppy, not dangerous: it will
 * turn, it will not hold the number precisely, and nothing about it stresses
 * the driver (steady-state demand there is 0.08-0.26 V of a 5 V limit).
 *
 * ZERO is handled separately and is NOT clamped up into this band -- a speed
 * request of 0 is the slider's bottom stop and means "stop", which clampRpm()
 * passes through and handleSpeed() turns into a controlled power-down. Without
 * that, asking for 0 would come back as 50 and the readout would never agree
 * with the setpoint. */
static const float RPM_MIN_ALLOWED = 50.0f;

// Where the setpoint sits at boot, before anyone touches the slider. Matches
// RPM_DEFAULT in the website's web/js/config.js.
static const float RPM_DEFAULT_TARGET = 150.0f;

/* velocity_limit for the PID itself. Kept a little above the clamp so the
 * controller has authority to correct at the top of the range rather than
 * saturating right where we spend most of our time.
 *
 * MUST BE RAISED WITH RPM_MAX_ALLOWED. Left at 300 while the clamp went to
 * 700, SimpleFOC would have limited the loop's own velocity target to 300 rpm
 * and the fan would have stuck there while the website showed 700 -- which
 * looks exactly like a motor too weak to reach the speed, and would have sent
 * you hunting for drag that was not there. 1.2x the clamp, as before. */
static const float RPM_VELOCITY_LIMIT = 840.0f;

/* Ramp rate, rpm per second, applied in BOTH directions and between
 * ARBITRARY current and target speeds -- so a mid-run speed change is smooth
 * either way, not just the initial spin-up.
 *
 * This is also the only current limiter available. Accelerating torque is
 * what pushes current past the steady-state figure, so a gentler ramp is
 * directly a lower peak current. motor_test's one-shot ramp was 200 rpm in
 * 3 s = ~67 rpm/s; 60 is a touch gentler than that. */
static const float RAMP_RPM_PER_S = 60.0f;

// Below this the arm is treated as stopped, for the idle/stopping distinction.
static const float RPM_STOPPED = 5.0f;

// ===========================================================================
//  FAULT THRESHOLDS
// ===========================================================================

// Sustained tracking error: the motor is being asked for a speed it is not
// delivering. Tolerated briefly (that is just the ramp), faulted if sustained.
static const float SPEED_ERROR_RPM = 40.0f;      // absolute floor of the band
static const float SPEED_ERROR_FRACTION = 0.2f;  // ...or 20% of target, whichever bigger
static const uint32_t SPEED_ERROR_MS = 2500;

// Runaway: actual speed well above anything we asked for.
static const float OVERSPEED_FACTOR = 1.3f;

/* Stall: the arm is barely turning despite being under power with the ramp
 * arrived. This exists because the tracking-error band below has a floor of
 * SPEED_ERROR_RPM, and at a low target that floor can exceed the target
 * itself -- at a 60 rpm target the band is 40 rpm, so a motor stuck at 25 rpm
 * would be inside tolerance and E03 would never fire. This catches it. */
static const float STALL_FRACTION = 0.35f;  // of target
static const uint32_t STALL_MS = 3000;

/* Voltage saturation -- the controller has run out of authority.
 *
 * This REPLACED an estimated-current fault. The current estimate is
 * demonstrably wrong (see the KV warning above: it reads negative while
 * motoring), and a fault built on a broken model is worse than no fault -- it
 * would stop the fan for no reason and teach you to distrust the fault system.
 *
 * voltage.q sitting pinned at the limit needs no model at all. It is directly
 * commanded, and it is the condition that precedes both "cannot reach speed"
 * and "drawing as much current as the driver will pass". Measured steady-state
 * demand is 0.61 V at 200 rpm against a 5 V limit, so anything near the limit
 * means something has changed a great deal. */
static const float SATURATION_FRACTION = 0.9f;  // of MOTOR_VOLTAGE_LIMIT
static const uint32_t SATURATION_MS = 3000;

/* Imbalance. Firmware cannot measure vibration without an accelerometer, but
 * an out-of-balance arm applies a torque disturbance once per revolution,
 * which shows up as velocity ripple the PID cannot flatten. Peak-to-peak
 * ripple over a one-revolution window is therefore a usable proxy.
 *
 * THERE IS A HARD NOISE FLOOR, and it is not small. Ripple is computed from
 * SimpleFOC's differentiated velocity, and at 12-bit resolution with a ~1.3 kHz
 * loop a single count of encoder dither is ~20 rpm. The first bring-up run
 * measured 19-28 rpm of "ripple" on a shaft that was provably stationary.
 *
 * So an earlier 25 rpm threshold sat INSIDE the noise and would have false-
 * tripped constantly. 60 rpm clears the floor with margin. The consequence is
 * worth being honest about: at a 150 rpm target, 60 rpm peak-to-peak is a 20%
 * speed swing, so E08 can only catch GROSS imbalance. It is not a substitute
 * for balancing the arm mechanically, and a subtle imbalance will pass it.
 *
 * Still uncalibrated against a real unbalanced arm -- run a balanced one and
 * then a deliberately weighted one and compare, per BRINGUP.md section 3.
 *
 * RIPPLE_MIN_RPM exists because below that speed the control loop itself
 * dominates the roughness (motor_test wandered 8 rpm at a 50 rpm target), and
 * calling that imbalance would be a false accusation. */
static const float RIPPLE_FAULT_RPM = 60.0f;
static const float RIPPLE_MIN_RPM = 100.0f;
static const uint32_t RIPPLE_FAULT_MS = 3000;

// ===========================================================================
//  TASK SCHEDULING  -- the single-core design. See main.cpp for the rationale.
// ===========================================================================

// Above AsyncTCP (10, pinned in platformio.ini), below lwIP (18) and the
// WiFi driver (23). Starving those two drops the AP or trips the watchdog.
static const UBaseType_t FOC_TASK_PRIORITY = 15;
static const uint32_t FOC_TASK_STACK = 4096;

/* The FOC task runs a burst of loopFOC() then sleeps one FreeRTOS tick.
 * CONFIG_FREERTOS_HZ is 1000 on Arduino-ESP32, so vTaskDelay(1) is 1 ms.
 *
 * 1 ms of work per 1 ms of sleep, so the loop takes about HALF the CPU and
 * leaves the other half to the radio and the web server.
 *
 * This was 2000 (67% duty), which measured focLoopHz 1330 and a 2.14 ms worst
 * gap -- both fine. It came down because the brief's stated failure mode is
 * "starving the WiFi stack will drop the access point", and the first run with
 * the motor spinning could not be joined from a phone or a laptop. Halving the
 * FOC share is cheap: measured control margin is enormous (a 5 ms design budget
 * against a 43 ms failure point) and loop rate is the thing we have to spare.
 *
 * Expect roughly focLoopHz 650-700 and a similar gap. If the AP is reliable at
 * this setting and you want the loop rate back, raise it in steps and watch
 * both focMaxGapMs AND whether clients still associate. */
static const uint32_t FOC_BURST_US = 1000;

// ===========================================================================
//  WEB / TELEMETRY
// ===========================================================================

static const uint32_t STATUS_EVENT_MS = 500;  // 2 Hz, per API.md
static const size_t MAX_SSE_CLIENTS = 3;      // each client holds a queue in
                                              // RAM; an open day is a crowd

static const int BRIGHTNESS_MIN = 0;
static const int BRIGHTNESS_MAX = 31;  // SK9822's native 5-bit global field

// ===========================================================================
//  CONTENT
// ===========================================================================

static const char *PRESET_DIR = "/presets";
static const char *USER_DIR = "/user";

/* A preset with no file behind it. Selecting it puts the ROTOR into its
 * per-revolution R/G/B diagnostic: the whole strip goes one flat colour and
 * steps red -> green -> blue once per accepted Hall pulse.
 *
 * It cannot be a .povf, and that is the point. A .povf is indexed by ANGLE, so
 * anything stored in one is a function of where the arm is pointing, not of how
 * many times it has been round -- a "flashing colours" image file would flash
 * identically whether or not the Hall sensor worked at all, which is precisely
 * the question being asked. The diagnostic has to live in the rotor's display
 * loop, where the revolution count is, so the website selects a MODE here
 * rather than a file. */
// ===========================================================================
//  LINEAR RAIL  -- must stay in step with rail_controller/src/config.h and
//  RAIL_* in the website's web/js/config.js
// ===========================================================================

/* GT2 20T pulley (40 mm/rev), 200-step NEMA17 at 1/8 microstep = 1600
 * steps/rev. The rail reports positions in STEPS; this converts them. */
static const float RAIL_STEPS_PER_MM = 40.0f;

/* Fallback only: the rail reports its own travel (measured, or its configured
 * nominal) in every telemetry packet, and that is what is used when the link
 * is up. This is the number shown before the rail has ever been heard from. */
static const float RAIL_TRAVEL_MM_NOMINAL = 500.0f;

/* Soft limits, matching the rail's own. Targets are clamped to these. */
static const float RAIL_SOFT_MARGIN_MM = 5.0f;

static const float RAIL_CRUISE_MM_S = 25.0f;
static const float RAIL_MAX_MM_S = 60.0f;

static const char *SENSOR_TEST_ID = "sensor-test";
static const char *SENSOR_TEST_NAME = "Sensor test";

static const uint32_t POVF_MAGIC = 0x504F5646UL;  // 'POVF', big-endian
static const size_t POVF_HEADER_BYTES = 16;

// Sanity bounds for a .povf header, so a corrupt or hostile file is rejected
// at byte 16 rather than after a megabyte has been written.
static const uint8_t POVF_VERSION = 1;
static const uint16_t POVF_MAX_FRAMES = 64;   // matches config.js MAX_FRAMES
static const uint8_t POVF_MAX_RADIAL = 64;
static const uint16_t POVF_MAX_ANGLES = 360;

// Keep some slack so the filesystem never runs completely dry -- LittleFS
// needs free blocks to do its own bookkeeping.
static const size_t FS_RESERVE_BYTES = 32 * 1024;

inline float rpmToRadS(float rpm) { return rpm * 2.0f * PI / 60.0f; }
inline float radSToRpm(float rads) { return rads * 60.0f / (2.0f * PI); }

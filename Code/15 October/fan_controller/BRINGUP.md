# Bring-up procedure

How to get from "the firmware compiles" to "the arm spins at the design speed"
without finding the ceiling by releasing smoke.

Read §0 before applying power with the arm attached.

---

## 0. Before the first spin-up with a load — the driver problem

**Short version: `voltage_limit = 8` is not safe with a real load, and this
firmware ships at 5.0 V instead. Measured data (2026-07-30) shows 5 V is ample
for the tested envelope. One modelling question remains open and it affects
every speed number in the project — see "the back-EMF model" below.**

The SimpleFOCmini (DRI0058) is built around a TI DRV8313. From the datasheet
(SLVSBA5B), per half-bridge:

| Parameter | Value | Conditions |
|---|---|---|
| Peak output current | **2.5 A** | 24 V, 25 °C |
| Continuous RMS output | **1.75 A** | *"with proper PCB heatsinking"*, 24 V, 25 °C |
| OCP trip level | **3 A min / 5 A max** | 5 µs deglitch |
| Thermal shutdown | 150–180 °C die | auto-recovers |
| RDS(on) low-side | 0.29 Ω typ / 0.39 Ω max | at TJ = 85 °C |

With `voltage_limit = 8 V` and 2.3 Ω:

- **700 rpm**, back-EMF ≈ 3.2 V → (8 − 3.2)/2.3 ≈ **2.1 A** — over the
  continuous rating, and that rating assumes heatsinking this board does not
  have.
- **Near stall during ramp-up**, back-EMF ≈ 0 → 8/2.3 ≈ **3.5 A** — over the
  *peak* rating, and inside the 3–5 A OCP window.

Being inside the OCP window is the worst place to be. A part at the low end of
the distribution trips and latches off; a part at the high end just cooks.

**The failure mode is worse than heat.** Per the datasheet, an OCP trip
disables the channel *"until either assertion of nRESET or the cycling of VM
power."* `nRESET` is currently floating and `nFAULT` is unwired, so the
firmware can neither detect nor clear it. It presents as one phase dying
mid-spin — violent cogging on a half-metre arm — with no fault on the website.

### Two wires that fix the blind spot — NOW WIRED

| Signal | Board pin | GPIO | Direction |
|---|---|---|---|
| `nFAULT` | **D11** | 7 | input, `INPUT_PULLUP`, active low |
| `nRESET` | **D12** | 6 | output, driven HIGH, pulsed LOW to clear |

This is the single most valuable wire in the build. `nFAULT` covers both
overcurrent and overtemperature, and it is the *only* way this hardware can
tell you the driver has shut a channel down — there is no current sensing to
infer it from. It now latches **E04** with a message on the page.

`nRESET` matters because an OCP trip stays latched until it is pulsed;
`/api/fault/clear` now does that automatically before re-checking, so an
overcurrent fault is recoverable from the web page instead of needing the barrel
jack pulled.

At boot the serial log prints which state `nFAULT` reads. **If it says `FAULT`
with a cold, idle driver, suspect the wiring rather than the driver** — an
open-drain output with no pull-up floats, which is why the firmware enables the
ESP32's internal pull-up.

### The resistance measurement, and the back-EMF model

**Resolved enough to proceed.** A meter reading across two motor leads came
back **6.6–6.7 Ω**. Taken at face value that is 3.3 Ω per phase for a wye
winding — *higher* than the datasheet's 2.3, and well clear of the 1.15 Ω case
that would have doubled every current figure. The meter is not trusted at these
values, but it points the safe way, so the table above stands as the
conservative case.

**What is NOT resolved is the back-EMF model**, and it matters more:

    measured at 200 rpm:   voltage.q = 0.61 V
    predicted back-EMF:    200/220 = 0.909 V

A motor cannot hold 200 rpm on less applied voltage than its own back-EMF, so
the estimate `(Uq − back-EMF)/R` comes out **negative while the motor is
plainly motoring**. One of these is wrong: the real KV is well above 220
(the data implies >320 rpm/V), or SimpleFOC's `voltage.q` carries a √3-ish
scaling factor, or the reported RPM is itself scaled wrong.

**A strobe or optical tachometer against the real shaft speed would settle the
third possibility in five minutes, and it is the one with the widest
consequences** — if reported RPM is wrong, every speed number in the project is
wrong, including the 700 rpm design target and the refresh-rate arithmetic.

Because of this, the firmware reports the current estimate but **no fault
depends on it**. The E09 fault now watches `voltage.q` against the limit
instead, which needs no model at all.

### What the measured data does and does not settle

**Settled: the tested envelope is nowhere near the driver's limits.** At 200 rpm
the controller uses 0.61 V of the 8 V it had — 7.6 %. Even on the pessimistic
reading (all of `Uq` across the winding, no back-EMF at all) that is 0.27 A. The
`voltage_limit = 5.0 V` this firmware ships with is ample headroom over 0.61 V,
so the earlier worry that 5 V might not spin the motor was unfounded.

This also retires `motor_test`'s *"bumped from 7 — 60rpm target was
voltage-saturating"* note. At 50 rpm the steady-state demand is under 0.3 V, so
whatever was saturating was a transient during a step change, not a standing
requirement — and the 60 rpm/s ramp avoids that class of transient entirely.

**Not settled: what the arm adds.** That is the whole of §2, and the honest
answer is that it has to be measured, not extrapolated.

---

## 1. First flash, no arm attached

Build and flash both the firmware and the filesystem:

```bash
cd ~/Documents/PlatformIO/Projects/fan_controller
~/.platformio/penv/bin/pio run --target upload
```

The `data/` directory is a copy of the website's `dist/`. Refresh it whenever
the UI changes:

```bash
cd ~/Desktop/Skripsie/Skripsie_Site && npm run build && rm -rf ~/Documents/PlatformIO/Projects/fan_controller/data && cp -R dist ~/Documents/PlatformIO/Projects/fan_controller/data
```

Then write it to LittleFS:

```bash
cd ~/Documents/PlatformIO/Projects/fan_controller && ~/.platformio/penv/bin/pio run --target uploadfs
```

Check on the serial monitor:

- `[FS] 5 item(s), ... KB free for uploads` — the filesystem mounted and the
  presets were found. If item count is 0, `uploadfs` has not been run.
- `[MOTOR] ready. clamp 60-250 rpm, voltage_limit 5.0 V`
- `[MOTOR] driver nFAULT (D11/GPIO7) reads OK at boot` — if this says
  `FAULT` on a cold driver, check the D11 wiring before anything else
- `[WIFI] AP "HologramFan" at 192.168.4.1` and `http://fan.local`

Join the `HologramFan` network (password `spinme123`, both in
`src/config.h`) and open <http://fan.local>. If mDNS does not resolve — some
Android builds refuse a network with no internet route — use
<http://192.168.4.1> directly.

### Reading the station count correctly

`wifi N client(s)` on the serial line comes from `softAPgetStationNum()`, and it
does **not** mean a client has connected successfully.

ESP-IDF adds a station to the AP's list as soon as **802.11 association**
completes, but only raises `AP_STACONNECTED` once the station is fully
**authorised** — after the WPA2 four-way handshake succeeds. So:

| What you see | What it means |
|---|---|
| count 0 → 1 → 0, **no** `station AUTHORISED` line | Associated, then the four-way handshake failed. Look at the disconnect reason |
| `station AUTHORISED (handshake ok)` | Genuinely connected. Next thing to want is `DHCP gave out` |

The disconnect line now names the 802.11 reason code. **Reason 15 is the
four-way handshake timing out**, which is what both a wrong passphrase and a
broken handshake look like from the AP side.

### If a client says "incorrect password"

It almost certainly is not the password. The ESP32-C6 is a WiFi 6 part and this
Arduino core's default AP protocol mask includes `WIFI_PROTOCOL_11AX`;
advertising HE capability from a SoftAP breaks the WPA2 four-way handshake for
some clients, Apple's especially. The client associates, the handshake fails, it
deauthenticates, and iOS reports the generic "incorrect password".

The firmware now pins the AP to 802.11 b/g/n at HT20, which fixes it. Boot prints
`[WIFI] AP radio: 11b/g/n ok, HT20 ok`.

The AP event log makes this diagnosable rather than guessable:

| What the log shows | Where it is failing |
|---|---|
| No `probe request seen` | Client never sees the AP — radio, channel, or CPU starvation |
| Probes but no `station associated` | Seen, but association fails |
| `station associated`, then a disconnect, no `DHCP gave out` | **The handshake — this is the 11ax case** |
| `DHCP gave out 192.168.4.x` | Network is fine; look at the browser or the HTTP server |

If it still refuses, set `AP_OPEN = true` in `src/config.h` to run the AP with no
password at all. If an open AP joins cleanly, the fault is definitely in the
handshake. That is also a perfectly reasonable way to *ship* this for an open
day: nothing to print on a sign, nothing to mistype, and there is nothing on the
device worth protecting.

### The motor will not move on its own

`motor_test` had a hardcoded `TARGET_RPM` and began ramping in `setup()`. **This
firmware does not.** It boots to `idle` with the driver disabled and waits to be
told to start, because speed is now API-driven. A healthy board therefore looks
exactly like a dead one on the serial monitor:

```
idle     target 150  actual    0 rpm (fast   -3) | Uq 0.00 V | ...
```

`state = idle` together with `Uq = 0.00 V` means "deliberately disabled", not
"broken". Press start on the web page, or use the bench keys below.

### Bench keys (serial monitor, then Enter)

For running the §2 sweep without juggling a browser window. These post the same
commands through the same queue as the HTTP handlers, so the firmware clamp, the
ramp and every fault check apply identically — there is no privileged path.

| Key | Action |
|---|---|
| `g` | start |
| `x` | stop (ramps down) |
| `e` | emergency stop — coasts, does not brake |
| `+` / `-` | target speed ±10 rpm |
| `c` | clear fault |
| `s` | AS5600 diagnostics |
| `?` | list the keys |

**MEASURED, 2026-07-30 — the single-core question is answered.**

| I²C clock | `focLoopHz` | steady gap | worst gap |
|---|---|---|---|
| 400 kHz | 3330 | 1.2–1.5 ms | boot transient only |
| 100 kHz (shipping) | **1336** | 1.3–1.6 ms | 1.85 ms |

Against a 5 ms design budget and a 43 ms failure point, with the AP up and the
web server running. **The architecture works.** The loop is bounded by the AS5600
read over I²C rather than by the CPU, which is why dropping to 100 kHz costs loop
rate but barely moves the gap — and the gap is the number that matters.

Two large gaps are expected and are *not* scheduling problems. Both clear
themselves now:

- **~58 ms at boot**, from `WiFi.softAP()` and mDNS blocking while the FOC task
  is already running.
- **~221 ms after an `s` diagnostic dump**, because the dump deliberately sleeps
  between angle reads.

### Reading the AS5600 dump (`s` + Enter)

| What you see | Means |
|---|---|
| `scan: nothing responded` | Sensor has no power, or SDA/SCL not connected |
| `0x36` present, register reads fail | Bus is marginal — clock, pull-ups, cable |
| `MD=0` | **The only real magnet fault.** No magnet, angle invalid |
| `MD=1, ML=1`, AGC railed at max | Usable but field is weak, no gain headroom. Works |
| `MD=1, MH=1`, AGC 0 | Magnet very close. Works, no headroom the other way |
| Angle stable to a few counts, tracks smoothly by hand | Healthy, whatever the flags say |

**AGC maxes at 128 on this board, not 255** — it is a 3.3 V supply, and the
datasheet's 0–255 range is the 5 V figure. AGC 128 here means railed.

**Measured on this hardware:** `STATUS=0x33` → MD=1, ML=1, MH=0, AGC 128,
magnitude 1731, angle stable to one count. That is a working sensor with a
slightly weak field, not a fault. An earlier version of this firmware had MD and
MH transposed and latched E01 on it, which is why this table spells out that only
MD gates validity.

The ML flag is worth one look at the magnet's seating, though — it means the AGC
has no gain left, so if the air gap grows this is where it starts to fail. On an
outrunner with the display arm bolted to the rotating face, added axial load is a
plausible way for that gap to change.

### Why the RPM readout jitters at standstill

The serial line prints two speeds: `actual` and `(fast N)`. On a stationary shaft
`fast` reads ±20 rpm while `actual` reads ~0. That is not a bug in either.

The AS5600 is 12-bit — 0.088° per count — and at 1336 Hz the sample interval is
0.75 ms, so **one count of dither differentiates to ~20 rpm.** `fast` is
SimpleFOC's differentiated, filtered velocity and carries that noise; the PID
needs it and is unbothered. `actual` measures angular *displacement* over 250 ms
instead, where 5 rpm is ~85 counts of travel against ±1 count of dither.

Everything that asks "has it stopped" uses `actual`, and it has to: with the fast
estimate never settling below `RPM_STOPPED`, the state machine would never reach
`idle` and **a latched motion fault could never be cleared.**

---

## 2. Finding the drag ceiling — do this before ever asking for 700 rpm

**Correction to an earlier draft of this document.** It claimed you could
predict 700 rpm behaviour by scaling current as ω². That is specific to
aerodynamic and propeller loads, not a general motor law, and asserting it here
was wrong. This setup — an LED strip and a Hall sensor on an arm, with no real
air-moving surface — plausibly sits much closer to constant bearing friction
plus back-EMF, which does not scale that way at all. **Do not use a ×7.84
extrapolation.**

What the load actually is, is unknown until measured with the arm fitted. The
general form has both terms:

```
Uq(ω)  =  Ke·ω   +   R·(T_friction + k·ω²)/Kt
          ↑           ↑              ↑
      back-EMF    constant friction  aerodynamic drag
      (linear)    (flat in ω)        (quadratic, may be negligible)
```

The bare-shaft data already shows the linear term dominating: `Uq` went 0.18 V
at 50 rpm to 0.61 V at 200 rpm, a 3.4× rise for a 4× speed increase. That is
back-EMF, essentially — the load contribution is small at both points. It tells
us nothing about the arm, because the arm was not on.

So the method is not extrapolation. **It is to walk up in steps and watch the
shape of the curve**, which distinguishes the two cases without assuming
either:

- **`Uq` rises in a straight line with speed** → back-EMF and constant friction
  dominate. Drag is not a factor and there is no cliff ahead.
- **`Uq` curves upward, increasingly steeply** → the ω² term is taking over.
  That is the signature of real aerodynamic drag, and the point at which
  extrapolation becomes both possible and necessary. Only then fit `k`.

Plot it as you go. The bend, or its absence, is the answer.

### Procedure

Attach the real arm, LED strip and all — the mass and the drag are the whole
point. Then, with the serial monitor open:

The website's speed slider now moves in 50 rpm detents from 0 to 700, so the
steps below and the positions on the slider are the same thing — walk it up one
click at a time.

| Step | Target | What to record |
|---|---|---|
| 1 | 50 rpm | `Uq`, `est A`, `ripple` |
| 2 | 100 rpm | same |
| 3 | 150 rpm | same |
| 4 | 200 rpm | same |
| 5 | 250 rpm | same — **end of the previously tested envelope** |
| 6 | 300 rpm | same, and from here also watch `focMaxGapMs` |
| 7–14 | 350 … 700 rpm | same, one 50 rpm click at a time |

**Steps 6 onward are new ground.** The clamp was raised to 700 rpm on
2026-08-18, so above 250 rpm nothing here has been run before. Stop at the
first step that misbehaves rather than pressing on to the next one.

The serial line prints all of it once a second:

```
running  target 150  actual 149 rpm | Uq 1.87 V  est 0.52 A | focLoopHz  3210  gap now  1.18 ms  worst  1.44 ms | ripple  2.3 rpm
```

**Let each step settle for 30 s before recording**, and watch the driver
temperature with a finger or an IR thermometer between steps.

### Reading the result

Compare the arm-fitted `Uq` at each speed against the bare-shaft baseline
(0.18 V at 50 rpm, 0.61 V at 200 rpm):

| What you see with the arm on | Means |
|---|---|
| `Uq` barely above baseline, straight line | The arm costs almost nothing. Proceed. |
| `Uq` offset above baseline by a roughly constant amount | Added bearing/friction load. Flat in speed, so it does not get worse as you go up. |
| `Uq` pulling away from the line, gap widening with speed | Aerodynamic drag is real. **This is the case that has a ceiling.** Fit `k` and predict before going further. |

Two things bound the top speed, and it is worth knowing which one you hit
first:

- **Voltage.** The E09 fault fires when `Uq` sits above 90 % of the limit, i.e.
  4.5 V of 5.0 V. With the measured KV of 333 rpm/V, back-EMF at 700 rpm is
  2.10 V, so there is 2.4 V of that budget left for load and acceleration
  combined. This is the constraint you are most likely to meet.
- **Current, and therefore heat.** Not directly measurable on this hardware —
  which is why the `nFAULT` wire matters. It is the only thing that will tell
  you the driver has overheated. Note that the ceiling here does *not* get
  worse with speed: back-EMF rises with rpm and eats the available voltage, so
  the most the driver can pass at 700 rpm is (5.0 − 2.10) / 2.3 = **1.26 A**,
  against **2.17 A** at standstill. The worst case is at the bottom of the
  range, not the top.

**If E09 fires while still climbing rather than at steady speed, the lever is
`RAMP_RPM_PER_S`, not `MOTOR_VOLTAGE_LIMIT`.** Accelerating torque is on top of
everything else, and it disappears once the ramp arrives. Very roughly — and
this is an *estimate from a guessed arm inertia*, not a measurement — 60 rpm/s
costs something like 1.2 V of headroom near the top of the ramp. Halving the
ramp rate halves that. Raising the voltage limit instead buys 50 rpm and costs
the driver: 6 V already puts standstill current at 2.6 A, over the 2.5 A peak
rating.

### Then raise the ceiling one step at a time

Only once the extrapolation says there is room:

**As of 2026-08-18 the ceiling is already at the 700 rpm target**, so this is
no longer about editing constants — it is about walking the slider up and being
willing to stop. `RPM_MAX_ALLOWED` in `src/config.h` and `RPM_MAX` in the
website's `web/js/config.js` are both at 700 and are commented to point at each
other; if you need to come back down, lower both.

Expect to re-tune. The PID notes in `motor_test` stop at 250 rpm and `I = 0.4`
was chosen for the 60–250 rpm regime, so above 250 the gains are extrapolated,
not tested. Two things to watch that are specific to the top of the range:

- **Commutation resolution.** At 700 rpm with 7 pole pairs the electrical
  frequency is 81.7 Hz, and a ~1 kHz FOC loop gives only ~12 updates per
  electrical cycle — about 30 electrical degrees per update, against ~30 Hz and
  ~120 electrical degrees of margin at 250 rpm. Expect more torque ripple. A
  single 5.77 ms stall (the measured worst case with a browser attached) is
  170 *electrical* degrees at 700 rpm — the torque vector is briefly wrong, not
  merely stale. Watch `focMaxGapMs` and prefer to run the arm with no browser
  actively driving the UI when taking measurements.
- **Balance.** Out-of-balance force goes as ω², so 700 rpm is **7.8×** the
  force at 250 rpm. See §3 — and do that section before this one.

**Signs you have gone too far:** `Uq` pinned at the voltage limit (the
controller has run out of authority), `E03` latching, or rising ripple.

---

## 3. Balance and vibration

Firmware cannot measure vibration without an accelerometer, so it measures the
symptom instead. An out-of-balance arm applies a torque disturbance **once per
revolution**, which the velocity loop cannot flatten, so it shows up as
periodic velocity ripple. `rpmRipple` is the peak-to-peak spread of measured
RPM over a one-revolution window, and **E08** latches when it stays high.

`RIPPLE_FAULT_RPM` is currently **25 rpm, and that is a guess.** Calibrate it
before trusting it:

1. Run the balanced arm through the §2 table and note `ripple` at each speed.
2. Tape a coin near the tip of one arm and repeat at a *low* speed only.
3. Set the threshold roughly midway between the two, with margin for the fact
   that ripple grows with speed.

Note the honest limit: this catches a *gross* imbalance, and only while
running. It is not a substitute for balancing the arm mechanically, and by the
time a badly-balanced arm is at 700 rpm the bearing loads are a mechanical
problem, not one firmware can fix.

---

## 3b. RESULTS — bare shaft, 2026-07-30

Driven from the web UI over the AP, with a browser connected and actively
moving the slider.

| Target rpm | Actual | `voltage.q` | ripple |
|---|---|---|---|
| 60 | 60–61 | 0.17–0.32 V | — (below `RIPPLE_MIN_RPM`) |
| 130 | 130–131 | 0.37–0.46 V | 7.3–9.3 rpm |
| 150 | 149–150 | 0.45–0.51 V | 4.5–7.7 rpm |
| 190 | 190–191 | 0.57–0.63 V | 3.1–9.1 rpm |
| 250 | 248–250 | 0.72–0.82 V | 3.1–15.5 rpm |

Tracking is within 1 rpm at every setpoint above 130. The 60 rpm point is
visibly twitchier, which is the friction-dominated regime the tuning notes warn
about — `RPM_MIN_ALLOWED` sits right at its edge deliberately.

### The load is friction, not drag — and the KV is wrong

Least-squares over the four clean points:

```
Uq = 0.00300 × rpm + 0.030 V        (dead straight, tiny intercept)
```

Two things fall out of that slope.

**The effective KV is 333 rpm/V, not the datasheet's 220** — a factor of 1.52.
`KV_RATING` in `config.h` now uses the measured figure.

**With the corrected KV, the residual current is 13 mA at every one of the four
speeds.** Constant, positive, and a plausible no-load figure. That constancy is
the point: a constant current means a **constant friction torque**, exactly as
predicted in §2, and *not* an aerodynamic load. With the old KV the same
arithmetic gave −88 mA at 150 rpm and −155 mA at 250, which is what flagged the
model as broken in the first place.

This is the bare-shaft baseline. **Repeat this table with the arm fitted and
compare**: if the line stays straight and merely shifts up, the arm has added
friction and there is no ceiling ahead. If it starts bending upward, that is the
aerodynamic term appearing, and only then does §2's quadratic fit apply.

### Headroom at 250 rpm

`Uq` 0.78 V against a 5.0 V limit — **16 % used**. Even taking all of `Uq` across
the winding with no back-EMF at all, that is 0.34 A against a driver rated 1.75 A
continuous. The tested envelope is nowhere near any electrical limit; whatever
constrains the top speed will be introduced by the arm, not by the electronics.

## 4. Measuring whether the single-core design works

This is the question the whole architecture rests on, and it is answered by two
numbers in `/api/status`:

- **`focLoopHz`** — `loopFOC()` iterations in the last second.
- **`focMaxGapMs`** — worst gap between iterations *since boot*. This is the
  one that matters. An average rate can look perfectly healthy while a single
  20 ms stall ruins the velocity estimate. It is a high-water mark, so it
  survives even if telemetry itself gets starved.

The budget: at 700 rpm a revolution takes 85.7 ms, so a gap over ~43 ms means
SimpleFOC's angle unwrapping can no longer tell how far the shaft turned.
**Design target is under 5 ms** — about 8× margin.

### RESULTS, 2026-07-30 — the architecture works

Measured on hardware, motor spinning, AP up, a browser connected and actively
driving the UI:

| Condition | `focLoopHz` | typical gap | worst gap |
|---|---|---|---|
| Idle, no client | 1000 | 1.5 ms | 2.4 ms |
| Client associated, browsing | 994–1000 | 2.0–2.4 ms | **5.77 ms** |
| Motor at 250 rpm + browser | 991–999 | 2.0–3.0 ms | 5.77 ms |

**Loop rate is essentially unaffected by WiFi** — 1000 Hz idle versus 994–1000 Hz
with a browser hammering it. That is the answer to the question the whole
architecture rested on.

**The worst gap did exceed the 5 ms design target, at 5.77 ms.** Stating that
plainly rather than rounding it away. In context it is still comfortable: at
250 rpm the arm turns 8.7° in 5.77 ms, and even at the 700 rpm design target it
would be 24° — against the ~180° at which SimpleFOC's angle unwrapping actually
breaks down. The 5 ms figure was a deliberately conservative target with ~8×
margin built in, and exceeding it by 15 % consumes some of that margin without
threatening the mechanism it was protecting.

The spikes coincide with WiFi housekeeping, not with anything the motor does.

Record all four of these:

```bash
# 1. idle, nothing connected
curl -s http://192.168.4.1/api/status | python3 -m json.tool | grep foc

# 2. with a browser open on the control page (SSE streaming at 2 Hz)
# 3. while dragging the speed slider around
# 4. while uploading a large .povf
```

**The upload case is the one to watch, and it is the one place task priority
does not protect the control loop.** Writing to LittleFS goes through the SPI
flash driver, which disables the instruction cache while a page program or
sector erase is in flight — and code running from flash, including the FOC
task, cannot execute at all during that window, regardless of its priority. A
page program is a few hundred microseconds; a 4 KB sector erase can be tens of
milliseconds.

If `focMaxGapMs` stays under 5 ms through all four, the architecture is sound.
If it only breaks during uploads, the fix is to refuse uploads while the motor
is spinning rather than to tune anything. If it breaks with a browser merely
connected, the honest options are the ones you listed: drop the AS5600 read
rate relative to the PWM update, move the web server to the S3, or accept a
lower top speed.

Reset the high-water mark by rebooting the board.

---

## 5. Function checklist

| Check | Expected |
|---|---|
| Page loads over the AP | Full UI, no blank page. A blank page means the `.gz` handling or the LittleFS mount — check `[FS]` on serial |
| Status panel tracks the slider | `rpmActual` follows `rpmTarget` within a few rpm once `state` is `running` |
| Start / stop | Ramps at 60 rpm/s in both directions; `state` goes `starting` → `running` → `stopping` → `idle` |
| Emergency stop | Motor disables immediately, `state` goes `stopping`. **The arm coasts — that is deliberate**, see below |
| Fault appears | Unplug the AS5600 while idle → **E01/E02** latches with a readable message |
| Fault clears | Refuses while the arm is still turning or the sensor is still unhappy; succeeds once stopped |
| Upload | Lands on LittleFS, appears in the library, survives a reboot |
| Delete | Works on uploads, 403 on presets |
| `focLoopHz` | Healthy throughout all of the above |

### On the emergency stop coasting

`/api/stop` calls `motor.disable()`. It does **not** brake, and on a
high-inertia outrunner the arm will keep turning for a long time.

That is deliberate. Braking a spinning motor electrically pushes the energy
back into the DC bus, and the bus here is a mains adapter, not a battery — it
cannot absorb it, so the rail voltage rises and the driver sees more than 12 V.
With no brake resistor and no current sensing, coasting is the safe choice.

**The consequence is mechanical, not electrical: "emergency stop" does not mean
"stops quickly".** For a public demo, the physical guard around the arm is the
actual safety mechanism, and the button is a convenience. Worth stating
explicitly in the safety report.

---

## 6. Fault codes

| Code | Meaning | Clears when |
|---|---|---|
| `E01` | AS5600 cannot see its magnet | Sensor reports a healthy magnet |
| `E02` | I²C failure talking to the AS5600 | Sensor responds again |
| `E03` | Sustained tracking error — cannot reach the requested speed | Arm has stopped |
| `E04` | Driver `nFAULT` asserted — overcurrent or overtemperature | `nRESET` pulsed and `nFAULT` released |
| `E05` | Stalled — under power but barely turning | Arm has stopped |
| `E06` | Overspeed / runaway | Arm has stopped |
| `E08` | Excessive velocity ripple above 100 rpm — probable imbalance | Arm has stopped |
| `E09` | `voltage.q` pinned near the limit — controller out of authority | Arm has stopped |

**E04 is the only fault the hardware reports rather than firmware inferring**,
and it means the driver has *already* shut a channel down. Everything else is
inference.

**E09 used to be an estimated-current fault and no longer is.** The current
estimate is demonstrably broken — it reads negative while the motor is motoring
(see §0) — and a fault built on a wrong model is worse than no fault, because it
stops the fan for no reason and teaches you to distrust the fault system.
Watching `voltage.q` against the limit needs no model at all, is directly
commanded, and is the condition that precedes both "cannot reach speed" and
"drawing as much as the driver will pass".

**E05 exists to close a hole in E03.** The tracking band has an absolute floor
of 40 rpm, so at a 60 rpm target a motor stuck at 25 rpm would sit *inside*
tolerance and E03 would never fire.

Every motion fault requires the arm to have actually stopped before it can be
cleared. That is deliberate — it stops someone clearing a fault and restarting
while a half-metre arm is still coasting at speed.

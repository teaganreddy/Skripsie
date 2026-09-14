# CLAUDE.md — Hologram fan display (Stellenbosch skripsie, MNS5)

Shared context for **any** Claude session working on this project — firmware or
website. Read this first; it is the distilled result of a long bring-up session
and most of it was learned the hard way on real hardware.

**Student:** Teagan Reddy, Mechatronics, Stellenbosch University.
**Project:** a 3D hologram-like RGB LED fan display (persistence of vision). A
spinning arm of RGB LEDs is strobed in sync with rotation angle to paint an
image that appears to float in a disc of space. A browser served off the device
lets anyone drive it and upload their own content.

---

## 1. Where everything lives

| Path | What | Who edits |
|---|---|---|
| `~/Desktop/Skripsie/Skripsie_Site` | Website + mock device. Has its own `CLAUDE.md` for frontend detail | **Website chat** |
| `~/Documents/PlatformIO/Projects/fan_controller` | **ESP32-C6 stator**: motor + WiFi AP + web server + ESP-NOW | Firmware chat |
| `~/Documents/PlatformIO/Projects/rotor_controller` | **ESP32-S3 rotor**: APA102 strip + Hall sync + ESP-NOW | Firmware chat |
| `~/Documents/PlatformIO/Projects/motor_test` | **Known-good motor reference. DO NOT MODIFY.** Its PID tuning is tested bench data | nobody |
| `~/Documents/PlatformIO/Projects/LED_Test`, `hall_effect_sensor` | S3 test sketches, kept as references | nobody |
| `~/Desktop/Skripsie/Datasheets/` | Component datasheets. **No DRV8313 datasheet here** — it was fetched from TI | — |
| `Code_with_Site/REPORT/` | Thesis write-up. **Not code — leave it alone.** | separate chat |

`Code_with_Site` (this folder) is a scratch PlatformIO project, not part of the
build. Both chats run from here.

PlatformIO CLI is **not on PATH**. Use:
```bash
~/.platformio/penv/bin/pio run --target upload
```

---

## 2. Keeping the three code bases in step

Firmware (`fan_controller/`, `rotor_controller/`) and the website
(`Skripsie_Site/`) are worked on together — a change on one side usually needs
the matching change on the other. `REPORT/` belongs to a separate chat and is
not code; leave it alone.

**Nothing is under git. There is no undo.** Check file timestamps before
overwriting anything you did not just write yourself.

### Things that MUST stay in step

| A | B | Why |
|---|---|---|
| `web/js/config.js` `RPM_MIN`/`RPM_MAX` | `fan_controller/src/config.h` `RPM_MIN_ALLOWED`/`RPM_MAX_ALLOWED` | A slider offering speeds the firmware clamps means the readout never reaches its setpoint |

> **Currently asymmetric and UNVERIFIED:** the slider is `RPM_MIN: 0` while the
> firmware clamps to `RPM_MIN_ALLOWED = 50`. Dragging to zero therefore sends
> `rpm=0`, the firmware clamps it to 50, and the readout disagrees with the
> slider. That may be deliberate (0 = "use the power button to stop"), but it
> has not been checked on hardware. Worth confirming before a demo.

| `web/js/config.js` `LEDS_TOTAL`/`LEDS_PER_ARM` | `rotor_controller/src/config.h` `LEDS_TOTAL`/`LEDS_PER_ARM` | Content geometry. Mismatch forces resampling on the rotor |
| `fan_controller/src/link_protocol.h` | `rotor_controller/src/link_protocol.h` | **Byte-identical.** Copy one over the other and bump `PROTOCOL_VERSION` |
| `mock/server.js` | the real firmware behaviour | The mock exists to make the UI meet realistic behaviour before hardware |

After changing website content: `npm run presets && npm run build`, then copy
`dist/` to `fan_controller/data/` and `pio run --target uploadfs`.

---

## 3. Hardware, as actually wired and measured

### Stator — DFRobot FireBeetle 2 **ESP32-C6**, single core, 4 MB flash
```
Motor    DFRobot FIT1034, 2804 BLDC outrunner, 7 pole pairs
Encoder  AS5600 inside the motor, I2C, SDA=GPIO19 SCL=GPIO20, 100 kHz
Driver   SimpleFOCmini (DRI0058, DRV8313)
           IN1=GPIO1(D6)  IN2=GPIO18(D7)  IN3=GPIO14(D3)  EN=GPIO8(D2)
           nFAULT=GPIO7(D11, INPUT_PULLUP)   nRESET=GPIO6(D12, output)
Supply   12 V 5 A mains adapter, barrel jack
```

**The arm is bolted directly to the outrunner's rotating face — 1:1, no
gearing.** So motor RPM *is* display RPM, and the AS5600 is measuring the
**absolute angular position of the display arm**. That becomes important for
multi-unit sync later.

### Rotor — DFRobot FireBeetle 2 **ESP32-S3 N16R8**, dual core
```
Flash    16 MB          PSRAM  8 MB (octal)
Strip    36 APA102 across the full diameter -> 18 per arm, hub in the middle
           data=GPIO15  clock=GPIO17  SPI 8 MHz
           5 V supply, 3.3 V logic (BSS138 level shifter bypassed)
Hall     GPIO14 (D10), INPUT_PULLUP, FALLING, 1 magnet per revolution
```

**The board definition is wrong for this unit** — it declares 4 MB flash and no
PSRAM (the base N4 variant). `platformio.ini` overrides all of it. See §6.

### Display geometry
```
36 LEDs, 18 per arm, 2 arms 180 deg apart, 180 angular sectors
refresh = 2 x RPM / 60      ->  23.3 Hz at 700 rpm, but only 8.3 Hz at 250
frame   = 18 x 180 x 3      =  9 720 bytes
```

---

## 4. Measured data — use these, not the datasheet

All bare shaft, 2026-07-30, via the web UI.

| rpm | `voltage.q` |
|---|---|
| 130 | 0.42 V |
| 150 | 0.48 V |
| 190 | 0.60 V |
| 250 | 0.78 V |

Least squares: **`Uq = 0.00300 x rpm + 0.030 V`** — dead straight.

- **Effective KV = 333 rpm/V, not the datasheet's 220** (1.52x). `KV_RATING` in
  `fan_controller/src/config.h` uses the measured figure. With 220 the current
  estimate came out *negative* while motoring, which is what exposed it.
- With KV=333 the residual current is **13 mA at every one of those speeds** —
  constant, so the load is **friction, not aerodynamic drag**.
- At 250 rpm, `Uq` is 0.78 V of a 5 V limit = **16 %**. The tested envelope is
  nowhere near any electrical limit.

### Single-core timing (the biggest design risk, now answered)

| Condition | `focLoopHz` | typical gap | worst |
|---|---|---|---|
| Idle | 1000 | 1.5 ms | 2.4 ms |
| Browser connected, driving the UI | 994–1000 | 2.0–2.4 ms | **5.77 ms** |

Design budget was 5 ms; angle unwrapping actually breaks around 43 ms. **The
architecture works** — WiFi costs the control loop under 1 %. The 5.77 ms
overshoot is honest but harmless (24° of rotation at 700 rpm vs the ~180° that
matters).

### AS5600 health on this unit
`STATUS = 0x33` → MD=1 (detected), ML=1 (**AGC railed, magnet at the weak
end**), MH=0. AGC 128, magnitude ~1731, angle stable to 1 count. Works, but with
no gain headroom — worth checking the magnet seating before high speed.

---

## 5. Safety constraints — do not change these casually

### `MOTOR_VOLTAGE_LIMIT = 5.0 V` (not `motor_test`'s 8 V)

The SimpleFOCmini is a **DRV8313**: 2.5 A peak, 1.75 A RMS continuous *"with
proper PCB heatsinking"* (which this bare board lacks), OCP trips 3–5 A, thermal
shutdown 150–180 °C.

At 8 V with 2.3 Ω, ramp-up would draw ~3.5 A — over the *peak* rating and inside
the OCP window, which is the worst place to sit (a part at the low end latches
off, one at the high end just cooks). **An OCP trip disables the channel until
nRESET is pulsed or VM is cycled** — hence the nFAULT/nRESET wires.

Measured demand at 250 rpm is 0.78 V, so 5 V is ample. Raising it needs a
reason.

### `RPM_MAX_ALLOWED = 700` / `RPM_MIN_ALLOWED = 50`

**Raised from 250/60 on 2026-08-18** so the display can reach the POV range.
The website slider matches: 0–700 in 50 rpm detents.

The electrical case, from the measured constants: back-EMF at 700 rpm is
700/333 = 2.10 V, leaving 2.90 V of the 5 V limit, so the driver can pass at
most 1.26 A there — **less** than the 2.17 A it can pass at standstill, because
back-EMF rises with speed and eats the available voltage. So raising the speed
ceiling did **not** move the worst-case current, which sits at zero speed and
is set by `MOTOR_VOLTAGE_LIMIT`, not by this constant.

**A speed ceiling that looks like a load problem is probably sensor lag** —
see gotcha 26 before touching `MOTOR_VOLTAGE_LIMIT`. Raising the voltage limit
to push through E09 would have been the wrong fix and would have driven far more
current for no extra torque.

**700 rpm has now been run successfully (2026-09-09)** — the display is clear
and animations are legible, so the gains tuned for 60–250 hold at 700 without
retuning. The walk-up procedure in `BRINGUP.md` §2 remains the right way to
approach any *new* speed, but the design target is reached.

**The remaining risk is mechanical, not electrical.** Out-of-balance force goes
as ω², so 700 rpm is 7.8× the force at 250. E08 (velocity ripple) only catches
gross imbalance — it sits on a ~20 rpm encoder noise floor. Balance the arm
first; `BRINGUP.md` §3.

### `/api/stop` coasts, it does not brake

Braking a high-inertia arm electrically pushes energy into a mains adapter with
nowhere to put it. Power-off does a controlled 60 rpm/s ramp-down (motor stays
energised); E-STOP removes drive entirely and the arm coasts. **The physical
guard is the real safety mechanism**, not the button.

---

## 6. Gotchas — every one of these cost real debugging time

### Motor / sensor
1. **`Wire.setClock()` before `Wire.begin()` silently does nothing** — it
   returns early if the bus lock does not exist. `motor_test` has run at 100 kHz
   for its whole life despite a comment claiming 400 kHz.
2. **AS5600 `STATUS` (0x0B) bits: 5 = MD, 4 = ML, 3 = MH.** MD and MH are easy
   to transpose, and doing so inverts the health check into requiring *no*
   magnet. Only MD gates validity; ML/MH are gain warnings.
3. **AGC maxes at 128 on a 3.3 V supply, not 255.**
4. **12-bit encoder + fast loop = ~20 rpm of quantisation noise.** One count of
   dither at 1.3 kHz differentiates to 20 rpm, so a stationary shaft reads ±20.
   Any `abs(rpm) < threshold` test for "stopped" therefore never passes — which
   once made latched faults permanently unclearable. Measure **displacement over
   250 ms** instead (`rpmSlow` in `motor_task.cpp`).
5. **SimpleFOC's `getRawCount()` ignores `requestFrom()`'s return value** — a
   failing I2C read returns plausible garbage rather than an error.
6. **LittleFS writes stall the instruction cache**, which stalls the FOC task
   regardless of its priority. Watch `focMaxGapMs` during uploads.

### WiFi / ESP-NOW
7. **ESP32-C6 SoftAP advertises 802.11ax by default** (`WIFI_PROTOCOL_DEFAULT`
   includes `WIFI_PROTOCOL_11AX`). Clients associate, the WPA2 handshake fails,
   and **iOS reports "incorrect password"**. Pin the AP to b/g/n + HT20.
   *(Note: on this hardware that was not the actual cause — an open AP was.
   `AP_OPEN = true` currently, which is also a fine way to ship for an open day.)*
8. **`softAPgetStationNum()` counts 802.11 association, not authorisation.**
   Count 0→1→0 with no `AP_STACONNECTED` event = the four-way handshake failed.
9. **Probe-request events are masked off by default** — `esp_wifi_set_event_mask(0)`.
10. **ESP-NOW does not scan or roam.** Both ends must already be on the AP's
    channel (1). Mismatch = total silence, no error anywhere.
11. **`rotor::begin()` must run after `WiFi.softAP()`** — ESP-NOW rides on the
    AP interface.
12. **This IDF's send callback is `esp_now_send_info_t*`**, not `uint8_t*`.
13. **ESP-NOW v2 carries 1470 bytes**, v1 only 250. Query
    `esp_now_get_version()` rather than assuming.

### Build / platform
14. **`memory_type` selects a directory by name; only `dio_opi`, `dio_qspi`,
    `opi_opi` exist.** There is no `qio_opi` — setting one silently falls back
    to a no-PSRAM build. Use **`dio_opi`** for the S3R8.
15. **`-DBOARD_HAS_PSRAM` is required** or `ESP.getPsramSize()` returns 0 even
    with PSRAM present.
16. **`LittleFS.begin()` defaults to partition label `"spiffs"`.** Our partition
    is named `littlefs`, so the label must be passed explicitly or it never
    mounts (presents as a blank web page).
17. **`sync` and `link` collide with POSIX `sync()`/`link()`** from `<unistd.h>`.
    Namespaces are `rotorsync` and `rotorlink`.
18. **The C6 may be USB-powered.** Unplugging it to move a cable kills the
    stator and drops the ESP-NOW link — this masqueraded as an RF problem.

### Content transfer
20. **`File::position()` counts from the start of the FILE, and the .povf
    payload starts 16 bytes in.** `pushSentBytes = pushFile.position()` in
    `rotor_stub.cpp` therefore drifted the payload offset by
    `POVF_HEADER_BYTES` per chunk. Offsets skipped, the rotor ended every
    transfer `CONTENT_ERR_INCOMPLETE`, and the stator still printed "content
    push complete" — so the website showed 100 % while the arm kept showing
    the fallback pattern. Fixed 2026-08-12 by counting acknowledged bytes
    explicitly. **Any file over one chunk (1400 B) failed; every preset is.**
21. **A retransmitted chunk is not necessarily a lost chunk.** Stop-and-wait
    retries when the *ack* is lost, so the rotor can legitimately receive the
    same chunk twice. Accumulating the checksum per arrival therefore failed
    perfectly good transfers. The rotor now sums the assembled buffer at END
    and counts bytes by high-water offset, both duplicate-proof.

### Rotor tasks / FreeRTOS
22. **`taskYIELD()` only reschedules among tasks of EQUAL OR HIGHER priority.**
    It is not a yield to *everything*. A task that spins on `taskYIELD()`
    starves every lower-priority task on its core, permanently.
23. **Arduino's `loop()` runs on CORE 1 at PRIORITY 1** (`CONFIG_ARDUINO_
    RUNNING_CORE=1`). `displayTask` is core 1, priority 20, and on the content
    path it never blocks — so `loop()` stopped running entirely whenever an
    image was playing, and the 5 Hz telemetry send that lived there stopped
    with it. The stator reported **"rotor link down" while the image kept
    painting perfectly**. ESP-NOW *receive* was unaffected (callbacks run on
    the WiFi task on core 0), so the stator could still command a rotor it
    could not hear — that asymmetry is the signature. Fixed 2026-08-19 by
    moving everything that is not the display into `serviceTask()` on core 0;
    `loop()` is now empty and must stay that way. `config.h` static_asserts the
    two cores differ.
24. **The idle-task watchdog is NOT enabled for CPU1**
    (`CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1` is unset), so starving core 1
    produces silence rather than a reboot. Nothing will tell you.
25. **This bug was latent from the start and our own fix exposed it.** Before
    the content push worked, `content::available()` was never true, the display
    loop always took the `REV_COLOUR` branch with its `vTaskDelay(1)`, and
    `loop()` always ran. Telemetry "had always worked" because the broken
    branch was unreachable.

### Motor, part 2 — the speed ceiling
26. **The AS5600's default output filter is the slow 16x setting, ~2.2 ms step
    response** (CONF bits 9:8, `SF`). That delay becomes electrical lag
    proportional to speed, and at 7 pole pairs it is brutal:

    | rpm | f_elec | lag (filter + 1 ms sampling) | torque/amp | current needed |
    |---|---|---|---|---|
    | 250 | 29 Hz | 34° | 0.83 | 1.2x |
    | 450 | 53 Hz | 61° | 0.49 | 2.0x |
    | 600 | 70 Hz | 81° | 0.16 | 6x |
    | 700 | 82 Hz | 94° | ~0 | no useful torque |

    Past 90° the current vector pushes sideways to the rotor and produces
    essentially nothing. **This is why the motor could hold 250 rpm easily,
    struggled and made a grinding noise around 600, and stalled out at 450 with
    the arm fitted** — it was not fighting a load, it was fighting its own
    misaligned field. It presents as E09 (voltage saturation) because the PID
    demands more and more voltage to compensate.

    Fixed 2026-09-10: `AS5600_SLOW_FILTER = 3` (2x, 0.286 ms) written to CONF at
    boot, before `initFOC()` so alignment measures the same sensor. Lag at
    700 rpm drops 94° -> 38°. **Confirmed on hardware: smooth to 650, 700 fine
    with slight noise, where before it could not pass 600 unloaded.**

    `CONF` is volatile unless burned, so this is a plain register write that
    reverts on power-off — nothing is permanently programmed into the part.
    Press **`w`** to cycle 16x/8x/4x/2x at a fixed speed and watch `Uq` move;
    that is the single-variable proof.

    **Consequence for the old numbers:** the §4 `Uq` baseline was measured with
    the slow filter, so it is inflated ~20 % at 250 rpm by lag. The linear fit
    and the KV derived from it are still broadly right, but re-measure before
    quoting them as precise.

### Process
19. **Do not batch many edits into one script that aborts on a failed
    assertion** — Python writes at the end, so one failure discards them all.
    This silently left ESP-NOW never being called despite a clean build. Verify
    each edit landed.

---

## 7. Current state

### Working end to end (website → hardware)
- Motor start/stop, controlled ramp both directions, speed slider, E-STOP
- Fault detection, latching, and refusal to clear while the cause persists
- Rotor telemetry over ESP-NOW: online, RSSI, rpm, columns/s, overruns
- Brightness relay to the APA102 global-current field
- Upload `.povf` to LittleFS, library listing, delete
- **Content transfer C6 → rotor PSRAM** (stage 2c) with real progress.
  **CONFIRMED WORKING on hardware 2026-09-09** — presets transfer and play.
  (Was broken until 2026-08-12; see gotchas 20/21 for what it was.)
- **"Sensor test" preset** — a fileless library entry (`SENSOR_TEST_ID`) that
  puts the rotor into its per-revolution R/G/B Hall diagnostic, selectable from
  the website. It is a *mode*, not a `.povf`: a `.povf` is indexed by angle and
  would flash identically whether or not the Hall sensor worked.
- APA102 driver, Hall angle interpolation (**verified to 0.3 %**), frame playback
- **THE DISPLAY WORKS AT ITS DESIGN SPEED.** Run at **700 rpm on 2026-09-09**:
  images are clear and animations are legible. This closes the project's
  central risk — POV needs ~600 rpm and the whole chain (motor, sync, transfer,
  playback) holds together there.
- **Rotor telemetry while content is playing** — was broken by task starvation
  until 2026-08-19, see gotchas 22–25. The website's rotor tiles now show
  "Rotor speed" (the rotor's own Hall figure) instead of the never-wired
  battery, which makes the stator encoder and the rotor Hall a live
  cross-check.

### Not done
- **Custom image upload end-to-end** — believed complete as of 2026-08-18 but
  NOT yet run on hardware. Every stage existed already; it was only ever
  blocked by the content-push bug (gotcha 20), because a user file and a preset
  take the same path below `/api/library/select`. **Uploads are now refused
  while the arm turns** — a flash erase stalls the FOC task for tens of ms, and
  at 700 rpm that exceeds the ~43 ms angle-unwrapping limit. Starting the fan
  during an upload is refused too.
- **Battery sensing.** No divider wired; `batteryPct` reports `null` on purpose.
  API.md allows it. **Do not fake it.**
- **Measured ESP-NOW throughput** — the code prints it; the number has not been
  captured yet. `API.md` open question 3 and `mock/server.js`
  `ESPNOW_BYTES_PER_SEC` both still assume 45 KB/s.
- Linear rail (third board) — not built.

### Display quality — where the limit actually is (analysed 2026-09-09)

Now that it runs at speed, the binding constraint on image quality is
**radial resolution, not timing.**

- Radial spacing is 222 mm / 18 = **12.3 mm** per LED.
- Angular arc at the rim is 2·π·222/180 = **7.7 mm**.

So pixels are 60 % taller than they are wide — the display is **radially
under-sampled relative to its angular resolution**. Going to 32 LEDs per arm
gives 6.9 mm radial against 7.7 mm tangential, near enough square, and is the
single biggest available image-quality win.

Timing headroom, from the measured 187 µs wire time (36 LEDs, 8 MHz, which
implies ~35 µs fixed per-transfer overhead):

| LEDs | SPI | wire | duty at 700 rpm, 180 sectors |
|---|---|---|---|
| 36 (now) | 8 MHz | 187 µs | **39 %** |
| 64 | 8 MHz | 299 µs | 63 % |
| 64 | 8 MHz | — | 89 % at 256 sectors — no margin |
| 64 | 16 MHz | 167 µs | 35 % (256 sectors: 50 %) |

So 64 LEDs fits at 8 MHz today. Going beyond 180 sectors *with* 64 LEDs needs
16 MHz — **and that forces the level-shifter question**: the strip is on 5 V
driven from 3.3 V logic with the BSS138 bypassed, and that margin shrinks as
edges get faster. Wrong colours or a misbehaving far end at higher clock is the
level shifter asking to be put back, not a firmware bug.

Refresh is 2 × 700/60 = **23.3 Hz**, right at the edge of flicker fusion. Bright
small sources stay visible as flicker above that, especially in peripheral
vision. Only more arms would fix it; speed is capped by the driver.

### On the "floating 3D hologram" effect

Worth recording because it is a common misconception and it shapes what is
worth building. **The commercial fans are not volumetric either.** A single
planar arm sweeps a *disc* — there is no depth information anywhere in the
system. The floating impression comes from three things:

1. the mechanism is invisible (arm is a blur, frame is dark),
2. black is genuinely black (LEDs off emit nothing, so contrast is effectively
   infinite against a dark room), and
3. the **content** carries the depth cues — perspective, shading, and above all
   rotation.

So the effect is a *content and presentation* problem, not a hardware one. Pure
black backgrounds, a rotating subject, margin around the object, a dark room and
a masked hub get most of the way there with zero code changes. True volumetric
would need emitters displaced along a third axis (helical blade, stacked discs,
tilted arm) — a different machine. The linear rail translates the whole fan and
buys **wider composite images** (FR16), not depth.

### Diagnostics available
**Rotor serial keys:** `i` info · `b` brightness · `m` fallback pattern ·
`f` free-run (virtual rotation, no Hall) · `<` `>` free-run rpm ·
`c` clear content · `d` dump blackbox · `z` clear log

The **blackbox** records every Hall edge and 20 Hz state snapshots to PSRAM, so
a spinning rotor with no USB can be examined afterwards. It is the tool that
proved the sync works.

**Stator serial keys:** `g` start · `x` stop · `e` E-STOP · `+`/`-` speed ·
`c` clear fault · `s` AS5600 diagnostics · `?` help

---

## 8. Working style Teagan expects

- **He flashes and tests; Claude writes and compiles.** Always build before
  handing over, and say which boards need reflashing and whether the filesystem
  changed.
- **Say plainly when something is your bug.** Several rounds here were lost to
  firmware errors; naming them quickly was more useful than hedging.
- **Do not fake data.** The battery reports null rather than a plausible
  invention, precisely because a convincing fake survives into a demo unnoticed.
- **He pushes back on shaky reasoning and is often right** — e.g. correctly
  rejecting an ω² drag extrapolation as specific to aerodynamic loads. Engage
  with the argument.
- Requirement **FR12** says firmware must be custom-developed without pre-built
  libraries. SimpleFOC is used anyway (carried over from `motor_test`); the
  APA102 driver is hand-rolled. Worth a sentence to his supervisor.

---

## 9. Key documents

- `fan_controller/BRINGUP.md` — bring-up procedure, driver current analysis,
  the drag-measurement method, fault code table. **Read before raising speed.**
- `Skripsie_Site/API.md` — the device API contract, now implemented
- `Skripsie_Site/CLAUDE.md` — frontend detail and the project's formal requirements
- `Skripsie_Site/README.md` — `.povf` format, geometry, the browser-does-the-
  image-processing decision
- `FIRMWARE_PROMPT.md` (this folder) — onboarding brief for a new session

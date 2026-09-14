# FIRMWARE_PROMPT.md — onboarding brief for a new session

*Replaces the original July 2026 brief, which described building the stator
firmware from nothing. That work is done. This is a handover, not a spec.*

**Scope: code and lab work** — both firmware projects and the website. The
thesis write-up in `REPORT/` is a different chat's job; do not touch it.

---

## Read this first

**`CLAUDE.md` in this folder is the authority.** It carries the hardware as
actually wired, the measured constants, the safety reasoning, and 25 numbered
gotchas that each cost real debugging time. This file tells you where the
project stands and how to work on it; `CLAUDE.md` tells you how it works.

Do not re-derive things it already records. In particular: do not trust the
motor datasheet over the measured constants, and read §6 before touching I²C,
WiFi, ESP-NOW, FreeRTOS tasks or the build configuration.

---

## What this is

A 3D hologram-like RGB LED fan display — a final-year Mechatronics thesis
project (Stellenbosch, MNS5) for **Teagan Reddy**. A spinning arm of RGB LEDs is
strobed in sync with rotation angle to paint an image that appears to float in a
disc of air. A browser served off the device itself lets anyone drive it and
upload their own content.

Three processors, two of them built:

| Board | Role | Status |
|---|---|---|
| **ESP32-C6** stator | Motor control + WiFi AP + web server + ESP-NOW | Working |
| **ESP32-S3** rotor | APA102 strip + Hall sync + content playback | Working |
| Linear rail | Lateral translation for multi-unit composites | Not built |

## Where the project actually is

**The core function is complete.** As of 2026-09-09 it runs at **700 rpm — its
design speed — with clear images and legible animations.** The whole chain
holds together: motor control, angular sync, wireless content transfer, and
playback. That closes the project's central technical risk.

So your job is most likely **verification, polish and optional improvement**,
not construction. Read `CLAUDE.md` §7 for the current working/not-done split
before assuming anything is missing.

### Outstanding, roughly in order of value

1. **Capture the ESP-NOW throughput figure.** The code measures and prints it on
   every content push; nobody has written the number down. It answers `API.md`
   open question 3, and both `mock/server.js` (`ESPNOW_BYTES_PER_SEC`) and the
   stator still assume a guessed 45 KB/s.
2. **Verify custom image upload end to end on hardware.** Believed complete —
   a user file takes the same path as a preset below `/api/library/select` —
   but never actually run. Note uploads are deliberately refused while the arm
   turns (a flash erase stalls the FOC task past the angle-unwrapping limit).
3. **The slider/clamp asymmetry.** The website offers 0–700; firmware clamps to
   50–700. Dragging to zero sends `rpm=0` and gets 50 back. May be deliberate,
   has not been checked. See the callout in `CLAUDE.md` §2.
4. **Image quality: 18 → 32 LEDs per arm.** The display is radially
   under-sampled relative to its angular resolution — pixels are 60 % taller
   than wide. This is the biggest available quality win and it *fits in the
   existing timing budget at 8 MHz*. See `CLAUDE.md` §7 for the bandwidth table
   and the level-shifter caveat that comes with going faster.
5. **Battery sensing.** No divider wired. `batteryPct` reports `null` on
   purpose and `API.md` allows it. **Do not fake it.**
6. Linear rail — a whole third subsystem, only if there is time.

---

## What you own

Three code bases, worked on together because a change to one usually needs the
matching change in another:

- **`fan_controller/`** — ESP32-C6 stator
- **`rotor_controller/`** — ESP32-S3 rotor
- **`~/Desktop/Skripsie/Skripsie_Site/`** — website, mock device, `API.md`,
  preset generator

`REPORT/` is a separate chat's thesis write-up. Not code, not yours.

**Nothing is under git. There is no undo.** Read `CLAUDE.md` §2 for the things
that must stay in step across those three code bases — particularly
`link_protocol.h`, which exists in **both** firmware projects and must be
byte-identical, and the RPM and LED-geometry constants, which appear in both
the website config and the firmware config and silently disagree if only one
is changed.

`motor_test/`, `LED_Test/` and `hall_effect_sensor/` are **read-only
references**. `motor_test`'s PID tuning is hard-won bench data and the fallback
if the motor ever misbehaves.

---

## Build and flash

PlatformIO is **not on `PATH`**:

```bash
~/.platformio/penv/bin/pio run --target upload
```

- Stator firmware: `cd ~/Documents/PlatformIO/Projects/fan_controller`
- Rotor firmware: `cd ~/Documents/PlatformIO/Projects/rotor_controller`
- Website content changed? `npm run presets && npm run build` in the site repo,
  copy `dist/` to `fan_controller/data/`, then `pio run --target uploadfs`.

**Always build before handing anything over**, and say explicitly which boards
need reflashing and whether the filesystem changed. Teagan flashes and tests;
you write and compile.

---

## How to work with Teagan

- **He flashes and tests.** Give him something that compiles and a clear
  statement of what to look for. Serial output and the on-board diagnostics
  (`CLAUDE.md` §7) are how you see what happened.
- **Name your own bugs immediately and plainly.** Several rounds of this project
  were lost to firmware errors; saying "that was my bug, here is the mechanism"
  moved things faster than hedging every time.
- **Never fabricate data.** The battery reports `null` rather than a plausible
  invention precisely because a convincing fake survives into a demo unnoticed.
  Same for anything you have not actually verified — say so.
- **He pushes back on shaky reasoning and is frequently right.** He correctly
  rejected an ω² drag extrapolation as specific to aerodynamic loads, and the
  measured data later proved him right. Engage with the argument rather than
  defending the first answer.
- **Verify every edit landed.** Gotcha 19: a multi-edit script that aborts
  partway discards everything, and a clean build will happily hide the fact that
  a whole subsystem is never called. Check individually.

---

## Traps most likely to bite you

The full list is `CLAUDE.md` §6. The ones that have cost the most time:

- **`taskYIELD()` only yields to equal-or-higher priority.** A spinning task on
  the rotor starved `loop()` permanently and telemetry died while the image kept
  painting perfectly. Core 1 also has no idle watchdog, so starvation is silent.
- **`Wire.setClock()` before `Wire.begin()` silently does nothing.**
- **AS5600 STATUS bits: 5 = MD, 4 = ML, 3 = MH.** Transposing MD and MH inverts
  the health check into requiring *no* magnet.
- **A retransmitted chunk is not a lost chunk** — stop-and-wait retries when the
  *ack* is lost, so the receiver can legitimately get the same chunk twice.
- **ESP-NOW does not scan or roam.** Both ends must already be on channel 1;
  a mismatch is total silence with no error reported anywhere.
- **Flash writes stall the instruction cache**, which stalls the FOC loop
  regardless of task priority.

---

## Safety — do not change these without a reason

`MOTOR_VOLTAGE_LIMIT = 5.0 V` and the speed clamp are **safety settings**, not
tuning knobs. The driver is a DRV8313 with no heatsinking; the reasoning and the
arithmetic are in `CLAUDE.md` §5 and `fan_controller/BRINGUP.md`.

Emergency stop **coasts rather than brakes**, deliberately — braking a
high-inertia arm pushes energy into a mains adapter that cannot absorb it. The
physical guard, not the button, is what protects a person from a spinning arm.

The remaining risk at 700 rpm is **mechanical, not electrical**: out-of-balance
force goes as ω². The E08 velocity-ripple fault only catches gross imbalance
because it sits on a ~20 rpm encoder noise floor.

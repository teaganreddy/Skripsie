#pragma once
/*
 * link_protocol.h -- the ESP-NOW wire format between stator (C6) and rotor (S3).
 *
 * >>> THIS FILE IS DUPLICATED IN THREE PROJECTS AND MUST STAY IDENTICAL. <<<
 *     fan_controller/src/link_protocol.h      (stator, the master)
 *     rotor_controller/src/link_protocol.h    (rotor)
 *     rail_controller/src/link_protocol.h     (linear rail)
 * Change one, copy it to the other, bump PROTOCOL_VERSION if the layout moves.
 * A mismatch here is silent: the structs still memcpy, the numbers are just
 * wrong. The version check below is what turns that into a visible error.
 *
 * Design notes:
 *
 *  - Fixed-size PODs, no serialisation. ESP-NOW carries up to 250 bytes and
 *    these are far smaller, so the simplicity is free.
 *  - Explicit widths and a static_assert on the size, because the two ends are
 *    different architectures (RISC-V on the C6, Xtensa on the S3) and struct
 *    padding is the classic way for that to go quietly wrong.
 *  - No timing information anywhere. The rotor derives its own angle from the
 *    Hall sensor; radio is far too slow and jittery to carry a 476 us column
 *    cadence. See rotor_controller/src/rotor_sync.h.
 */

#include <stdint.h>

static const uint8_t PROTOCOL_VERSION = 2;  // 2 adds content transfer

// Both ends must sit on the AP's channel. ESP-NOW does not roam.
static const uint8_t LINK_CHANNEL = 1;

enum : uint8_t {
  MSG_BEACON = 0xB0,     // stator -> broadcast, "I am here, this is my MAC"
  MSG_TELEMETRY = 0x71,  // rotor  -> stator, 2 Hz
  MSG_COMMAND = 0xC0,    // stator -> rotor

  // --- content transfer, stage 2c ---
  MSG_CONTENT_BEGIN = 0xD0,  // stator -> rotor, announces a transfer
  MSG_CONTENT_CHUNK = 0xD1,  // stator -> rotor, payload
  MSG_CONTENT_END = 0xD2,    // stator -> rotor, finish + checksum
  MSG_CONTENT_ACK = 0xD3,    // rotor  -> stator, progress and result

  // --- linear rail, a third peer on the same link ---
  MSG_RAIL_TELEMETRY = 0xE0,  // rail   -> stator, 5 Hz
  MSG_RAIL_COMMAND = 0xE1,    // stator -> rail, on change and resent per beacon
};

/* ===========================================================================
 *  CONTENT TRANSFER
 * ===========================================================================
 * A .povf payload moves from the stator's LittleFS into the rotor's PSRAM
 * once, and is then played back locally -- never streamed per frame. At 700 rpm
 * a column is due every 476 us and ESP-NOW latency is milliseconds, so
 * per-frame streaming is not slow, it is impossible. API.md section 5 assumes
 * this model.
 *
 * Reliability rides on ESP-NOW's own link-layer acknowledgement rather than an
 * application-level ack per chunk. The send callback reports whether the peer
 * MAC acked, so the stator retries a chunk that failed and only advances when
 * one succeeded. That gives ordered, reliable delivery with one round trip per
 * chunk and no protocol of our own to get wrong.
 *
 * transferId lets the rotor discard chunks from an abandoned transfer -- select
 * a different image mid-push and the stale packets still in flight must not be
 * written into the new buffer.
 */
struct __attribute__((packed)) ContentBeginMsg {
  uint8_t type;
  uint8_t version;
  uint16_t frames;
  uint16_t angles;
  uint8_t radial;
  uint8_t fps;
  uint32_t payloadBytes;  // frames * angles * radial * 3, header excluded
  uint32_t transferId;
  char id[24];
};
static_assert(sizeof(ContentBeginMsg) == 40, "ContentBeginMsg size changed");

/* Followed immediately by the payload bytes, so the whole packet is
 * sizeof(ContentChunkMsg) + dataLen. Kept small deliberately: every byte of
 * header is a byte of payload that does not fit. */
struct __attribute__((packed)) ContentChunkMsg {
  uint8_t type;
  uint8_t version;
  uint16_t dataLen;
  uint32_t offset;
  uint32_t transferId;
};
static_assert(sizeof(ContentChunkMsg) == 12, "ContentChunkMsg size changed");

struct __attribute__((packed)) ContentEndMsg {
  uint8_t type;
  uint8_t version;
  uint16_t reserved;
  uint32_t transferId;
  uint32_t payloadBytes;
  uint32_t checksum;  // plain 32-bit sum of payload bytes; catches gross loss
};
static_assert(sizeof(ContentEndMsg) == 16, "ContentEndMsg size changed");

struct __attribute__((packed)) ContentAckMsg {
  uint8_t type;
  uint8_t version;
  uint8_t status;  // CONTENT_*
  uint8_t reserved;
  uint32_t transferId;
  uint32_t bytesReceived;
};
static_assert(sizeof(ContentAckMsg) == 12, "ContentAckMsg size changed");

enum : uint8_t {
  CONTENT_OK = 0,        // transfer complete and accepted, now displaying
  CONTENT_IN_PROGRESS = 1,
  CONTENT_ERR_SIZE = 2,     // geometry or length the rotor cannot hold
  CONTENT_ERR_INCOMPLETE = 3,  // bytes missing at END
  CONTENT_ERR_CHECKSUM = 4,
  CONTENT_ERR_NO_MEMORY = 5,
};

/* The largest payload the ROTOR can hold, and therefore the largest the stator
 * may accept at upload. It is the size of each of the rotor's two PSRAM
 * buffers.
 *
 * This lives here, in the shared header, because the two ends had independent
 * limits and they did not agree. The stator validated an upload against
 * POVF_MAX_FRAMES(64) x POVF_MAX_ANGLES(360) x POVF_MAX_RADIAL(64) x 3 =
 * 4 423 680 bytes and against free flash; the rotor could only ever hold
 * 1 105 920. Anything between the two was accepted, written to LittleFS,
 * listed in the library, and then rejected by the rotor with
 * CONTENT_ERR_SIZE every single time it was selected -- which in turn fed the
 * 8-second auto re-push retry in rotor_stub.cpp forever. One number, shared,
 * removes the whole class of failure.
 *
 * 64 frames x 180 angles x 32 radial x 3. The radial 32 is headroom: the strip
 * is 18 per arm today, so the website's worst case is 622 080 bytes. */
static const uint32_t LINK_MAX_CONTENT_BYTES = 64u * 180u * 32u * 3u;  // 1 105 920

/* Stator -> broadcast. Discovery, so neither MAC has to be hardcoded and a
 * board can be swapped without reflashing the other end. */
struct __attribute__((packed)) BeaconMsg {
  uint8_t type;     // MSG_BEACON
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t channel;
  /* Which fan unit this stator is. 0 means "unset" and is treated as unit 1,
   * so a stator that never fills the field keeps working. This byte was
   * `reserved` and is what stops two units' rotors flip-flopping between two
   * beaconing stators: a peer only pairs with the beacon carrying its own
   * unit id. Same size, so PROTOCOL_VERSION stays at 2. */
  uint8_t unitId;
};
static_assert(sizeof(BeaconMsg) == 4, "BeaconMsg size changed");

/* Rotor -> stator, 2 Hz. Everything the website's rotor panel shows, plus the
 * display diagnostics -- which matter because once the arm is spinning there
 * is no USB on the rotor and this is the only way to see inside it. */
struct __attribute__((packed)) TelemetryMsg {
  uint8_t type;     // MSG_TELEMETRY
  uint8_t version;  // PROTOCOL_VERSION

  /* Battery. batteryValid is false when no sense divider is wired, and the
   * stator then reports null rather than inventing a number -- API.md allows
   * batteryPct to be null precisely for this. */
  uint8_t batteryValid;
  uint8_t batteryPct;  // 0-100
  uint16_t batteryMv;

  uint16_t rpm;            // measured from the Hall sensor, rotor's own figure
  uint16_t columnsPerSec;  // actual paint rate
  uint16_t overruns;       // sector ran past the end of a revolution
  uint16_t rejectedPulses; // Hall pulses failing the sanity window
  uint32_t revolutions;

  uint8_t flags;  // see LINK_FLAG_* below
  uint8_t brightness;
  uint8_t reserved[2];
};
static_assert(sizeof(TelemetryMsg) == 22, "TelemetryMsg size changed");

enum : uint8_t {
  LINK_FLAG_SYNC_LOCKED = 1 << 0,  // a revolution period has been measured
  LINK_FLAG_SPINNING = 1 << 1,     // Hall seen within the adaptive timeout
  LINK_FLAG_FREE_RUN = 1 << 2,     // virtual clock, Hall ignored (bench only)
  LINK_FLAG_HAS_CONTENT = 1 << 3,  // real content loaded, not the test pattern
};

/* Stator -> rotor. Small, idempotent, and resent every beacon so a dropped
 * packet self-heals rather than needing an ack. */
struct __attribute__((packed)) CommandMsg {
  uint8_t type;     // MSG_COMMAND
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t brightness;
  uint8_t flags;     // see CMD_FLAG_*
  char contentId[24];
};
static_assert(sizeof(CommandMsg) == 28, "CommandMsg size changed");

enum : uint8_t {
  CMD_FLAG_DISPLAY_ON = 1 << 0,  // paint; clear the strip when unset

  /* Show the per-revolution R/G/B Hall diagnostic instead of content. A flag
   * rather than a magic contentId the rotor has to recognise: the stator owns
   * the id (SENSOR_TEST_ID in its config.h) and the rotor just obeys.
   *
   * Additive -- no struct layout moved, so PROTOCOL_VERSION stays at 2. An
   * older rotor would ignore the bit and keep showing content rather than
   * misreading anything. */
  CMD_FLAG_SENSOR_TEST = 1 << 1,
};

/* ===========================================================================
 *  LINEAR RAIL
 * ===========================================================================
 * A third C6 drives a NEMA17 on a belt and carries the whole fan unit along an
 * aluminium extrusion between two limit switches. It pairs the same way the
 * rotor does -- hears the stator's beacon, answers with telemetry -- and takes
 * commands from the stator only. It never moves on its own.
 *
 * Commands carry a sequence number so that resending the current command every
 * beacon (the same self-healing trick CommandMsg uses) does not re-execute a
 * one-shot op like HOME on every resend. The rail acts on a command once, when
 * seq changes, and otherwise just re-reads the flags.
 */
enum : uint8_t {
  RAIL_OP_NONE = 0,
  RAIL_OP_STOP = 1,      // decelerate and hold
  RAIL_OP_HOME = 2,      // seek the HOME switch and zero
  RAIL_OP_MEASURE = 3,   // after homing: seek IDLE once and record the travel
  RAIL_OP_GOTO = 4,      // move to targetSteps at speedStepsPerSec
  RAIL_OP_JOG = 5,       // move by targetSteps (signed, relative)
  RAIL_OP_SWEEP = 6,     // oscillate between sweepMin/sweepMax with a dwell
  RAIL_OP_CLEAR = 7,     // clear a latched fault (rail stays unhomed)
};

enum : uint8_t {
  /* The stator asserts this only while the fan is stopped (phase 1) or while
   * sweeping is explicitly allowed (phase 2). The rail refuses to start a move
   * without it and HALTS a move in progress if a command arrives without it --
   * so a bug in the stator's interlock cannot leave a carriage moving under a
   * spinning arm. Defence in depth, not the primary gate. */
  RAIL_FLAG_MOTION_PERMITTED = 1 << 0,
};

struct __attribute__((packed)) RailCommandMsg {
  uint8_t type;     // MSG_RAIL_COMMAND
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t seq;      // bump to execute `op`; unchanged = just refresh flags
  uint8_t op;       // RAIL_OP_*
  uint8_t flags;    // RAIL_FLAG_*
  uint8_t reserved[3];
  int32_t targetSteps;        // GOTO: absolute; JOG: relative
  uint32_t speedStepsPerSec;  // 0 = rail's configured cruise speed
  int32_t sweepMinSteps;
  int32_t sweepMaxSteps;
  uint32_t sweepDwellMs;
};
static_assert(sizeof(RailCommandMsg) == 28, "RailCommandMsg size changed");

enum : uint8_t {
  RAIL_STATE_UNHOMED = 0,
  RAIL_STATE_HOMING = 1,
  RAIL_STATE_IDLE = 2,
  RAIL_STATE_MOVING = 3,
  RAIL_STATE_SWEEPING = 4,
  RAIL_STATE_MEASURING = 5,
  RAIL_STATE_FAULT = 6,
};

enum : uint8_t {
  RAIL_FAULT_NONE = 0,
  RAIL_FAULT_BOTH_LIMITS = 1,   // both switches read triggered -- wiring
  RAIL_FAULT_LOST_STEPS = 2,    // a switch fired where the counter said it could not
  RAIL_FAULT_HOME_NOT_FOUND = 3,  // travelled past the whole rail and never hit HOME
  RAIL_FAULT_IDLE_NOT_FOUND = 4,  // same, for the IDLE end while measuring
  RAIL_FAULT_LINK_LOST = 5,     // stator went quiet mid-move; halted
};

enum : uint8_t {
  RAIL_TFLAG_HOMED = 1 << 0,
  RAIL_TFLAG_LIMIT_HOME = 1 << 1,  // switch currently triggered
  RAIL_TFLAG_LIMIT_IDLE = 1 << 2,
  RAIL_TFLAG_MOVING = 1 << 3,      // pulses are being generated right now
  RAIL_TFLAG_DRIVER_ENABLED = 1 << 4,
  RAIL_TFLAG_TRAVEL_MEASURED = 1 << 5,
};

struct __attribute__((packed)) RailTelemetryMsg {
  uint8_t type;     // MSG_RAIL_TELEMETRY
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t state;    // RAIL_STATE_*
  uint8_t fault;    // RAIL_FAULT_*
  uint8_t flags;    // RAIL_TFLAG_*
  uint8_t lastSeq;  // the command seq most recently acted on
  uint8_t reserved[2];
  int32_t positionSteps;   // from the HOME zero; meaningless until homed
  int32_t targetSteps;
  uint32_t travelSteps;    // measured if TRAVEL_MEASURED, else configured
  uint32_t rateStepsPerSec;
};
static_assert(sizeof(RailTelemetryMsg) == 24, "RailTelemetryMsg size changed");

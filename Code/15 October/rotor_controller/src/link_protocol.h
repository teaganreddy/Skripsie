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

static const uint8_t PROTOCOL_VERSION = 4;  // 4: network log, calibration in the command

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

  // --- second fan unit: its stator is a peer of the master's ---
  MSG_UNIT_TELEMETRY = 0xF0,  // peer stator -> master, 5 Hz
  MSG_UNIT_COMMAND = 0xF1,    // master -> peer stator, 5 Hz

  // --- network log: serial lines from boards that cannot have USB attached ---
  MSG_LOG = 0xA0,  // rotor -> its stator; peer stator -> master (own + its rotor's)

  // --- file sync: the master copies user uploads to the peer stator's flash ---
  MSG_FILE_BEGIN = 0xF2,   // master -> peer: announce a file (or ask "have you got it?")
  MSG_FILE_CHUNK = 0xF3,   // master -> peer: file bytes
  MSG_FILE_END = 0xF4,     // master -> peer: finish + checksum
  MSG_FILE_ACK = 0xF5,     // peer -> master: progress and verdict
  MSG_FILE_DELETE = 0xF6,  // master -> peer: remove a user file
};

/* ===========================================================================
 *  NETWORK LOG
 * ===========================================================================
 * No spinning board can have a serial monitor attached, and the stators sit
 * under the arms too. The lines that matter (pairing, content accepted or
 * rejected, faults, link loss, scene events) are therefore also sent here:
 * a rotor sends its lines to its stator, a peer stator forwards its own and
 * its rotor's to the master, and the master keeps a ring buffer that the
 * website reads at /api/log. Rate limited at the source; the chatty 1 Hz
 * status lines never travel. */
enum : uint8_t {
  LOG_SRC_STATOR = 0,
  LOG_SRC_ROTOR = 1,
};

struct __attribute__((packed)) LogMsg {
  uint8_t type;     // MSG_LOG
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t unitId;   // which fan unit the line came from
  uint8_t source;   // LOG_SRC_*
  char text[160];   // NUL-terminated
};
static_assert(sizeof(LogMsg) == 164, "LogMsg size changed");

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
  uint32_t payloadBytes;  // bytes ON THE WIRE, header excluded (compressed if RLE)
  uint32_t transferId;
  char id[24];
  uint8_t encoding;       // CONTENT_ENC_*
  uint8_t reserved[3];
};
static_assert(sizeof(ContentBeginMsg) == 44, "ContentBeginMsg size changed");

/* How the payload is encoded on the wire and in the file.
 *
 * RAW is the .povf body as the browser writes it. RLE is a run-length coding
 * of the same pixel stream: [count 1..255][r][g][b] repeated. The scene
 * presets (pong, fish, flappy) are 95 % flat background, so RLE shrinks them
 * ten to fifty times, which is what makes six animated scenes fit in a
 * 2 MB filesystem and appear in under a second instead of ~14 s. The rotor
 * receives the compressed bytes into a staging buffer exactly as it would a
 * raw transfer (byte-addressed, idempotent, checksummed at END) and decodes
 * into the play buffer once the transfer has verified. Uploads stay RAW. */
enum : uint8_t {
  CONTENT_ENC_RAW = 0,
  CONTENT_ENC_RLE = 1,
};

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
  /* The raw level on the Hall input, 1 = high (no magnet), 0 = low. Lets the
   * website's Diagnostics tab show whether the sensor is switching at all
   * when the arm is turned by hand: the difference between "no pulses" and
   * "no sensor". Was a reserved byte; PROTOCOL_VERSION unchanged. */
  uint8_t hallLevel;
  uint8_t reserved;
};
static_assert(sizeof(TelemetryMsg) == 22, "TelemetryMsg size changed");

enum : uint8_t {
  LINK_FLAG_SYNC_LOCKED = 1 << 0,  // a revolution period has been measured
  LINK_FLAG_SPINNING = 1 << 1,     // Hall seen within the adaptive timeout
  LINK_FLAG_FREE_RUN = 1 << 2,     // virtual clock, Hall ignored (bench only)
  LINK_FLAG_HAS_CONTENT = 1 << 3,  // real content loaded, not the test pattern
  /* Content is loaded but it is NOT what CommandMsg.contentId asks for (a
   * push failed, or was refused while this rotor was offline, and the old
   * image stayed up). The stator re-pushes on it. Clear while a transfer is
   * being received. Additive, so PROTOCOL_VERSION is unchanged: an older
   * rotor never sets it and the stator behaves as it always did. */
  LINK_FLAG_WRONG_CONTENT = 1 << 4,
};

/* Stator -> rotor. Small, idempotent, and resent every beacon so a dropped
 * packet self-heals rather than needing an ack. */
struct __attribute__((packed)) CommandMsg {
  uint8_t type;     // MSG_COMMAND
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t brightness;
  uint8_t flags;     // see CMD_FLAG_*
  char contentId[24];
  /* Loop sync. Bumped by the master once per animation loop for content that
   * has to play in step on both fans (pong: the ball leaves one disc and
   * enters the other). A rotor restarts its animation at frame `startFrame`
   * (normally 0) whenever this changes. Unit 2's rotor gets it relayed
   * through its own stator. */
  uint8_t loopSeq;
  /* Angle calibration, held by the STATOR (per unit, in its NVS, set from
   * the website's Diagnostics tab) and applied by the rotor to every column:
   * content angle = (reverse ? -sector : sector) + sectorOffset. The rotor
   * uses its own build defaults only until a command carrying CAL_VALID
   * arrives. Sectors, 0..179, 2 deg each. */
  uint8_t sectorOffset;
  uint8_t calFlags;  // CAL_FLAG_*
  /* The frame a loopSeq restart starts at, instead of frame 0. Used by the
   * rail-cycle scenes (flappy), whose file is one whole sweep cycle: the
   * master restarts both rotors at the frame that matches where the carriage
   * is each time it sets off from an end. 0 for everything else. Read only
   * when loopSeq changes. Was `reserved` (always sent as 0), so an older
   * rotor ignores it and PROTOCOL_VERSION is unchanged. */
  uint8_t startFrame;
};
static_assert(sizeof(CommandMsg) == 32, "CommandMsg size changed");

enum : uint8_t {
  CAL_FLAG_VALID = 1 << 0,    // the offset/reverse fields are meaningful
  CAL_FLAG_REVERSE = 1 << 1,  // the arm turns the other way from the content's convention
};

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

  /* Play the content mirrored about its vertical axis. Set by the master
   * while the rail carries the fans toward the viewer's left, for content
   * authored facing right (fish): the same file swims both ways. */
  CMD_FLAG_MIRROR = 1 << 2,
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

enum : uint8_t {
  RAIL_REFUSED_NONE = 0,
  RAIL_REFUSED_FAULT = 1,          // a fault is latched
  RAIL_REFUSED_NOT_PERMITTED = 2,  // no motion permission (a fan not stopped)
  RAIL_REFUSED_DRIVER_OFF = 3,     // A4988 disabled ('d' on the rail's serial)
  RAIL_REFUSED_BUSY = 4,           // already moving
  RAIL_REFUSED_NOT_HOMED = 5,
  RAIL_REFUSED_SPAN = 6,           // sweep range under 1 mm after clamping
};

struct __attribute__((packed)) RailTelemetryMsg {
  uint8_t type;     // MSG_RAIL_TELEMETRY
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t state;    // RAIL_STATE_*
  uint8_t fault;    // RAIL_FAULT_*
  uint8_t flags;    // RAIL_TFLAG_*
  uint8_t lastSeq;  // the command seq most recently acted on
  /* Why the op at lastSeq was refused (RAIL_REFUSED_*), 0 if it was
   * accepted. The stator answers the website 200 as soon as it has SENT a
   * command, so a refusal on the rail ("already moving") was otherwise
   * visible only on the rail's serial. Was reserved[0]; layout unchanged,
   * PROTOCOL_VERSION unchanged. */
  uint8_t refused;
  uint8_t reserved;
  int32_t positionSteps;   // from the HOME zero; meaningless until homed
  int32_t targetSteps;
  uint32_t travelSteps;    // measured if TRAVEL_MEASURED, else configured
  uint32_t rateStepsPerSec;
};
static_assert(sizeof(RailTelemetryMsg) == 24, "RailTelemetryMsg size changed");

/* ===========================================================================
 *  SECOND FAN UNIT
 * ===========================================================================
 * Two identical fan units, one website. Unit 1's stator is the MASTER: it
 * runs the access point, the web server and the content library. Unit 2's
 * stator is a PEER: same firmware built with -DUNIT_ID=2, no AP, its own FOC
 * loop and its own rotor link, taking commands from the master over ESP-NOW
 * exactly the way the rail does.
 *
 * Discovery is the rail's pattern in reverse: the peer hears the master's
 * beacon (unitId 1), pairs to it and starts sending telemetry; the master
 * records the peer's MAC from the first telemetry packet.
 *
 * Content: every stator's LittleFS is flashed with the same image, so the
 * PRESETS already exist on the peer. The master therefore sends only the
 * content id and the peer pushes it to its own rotor with the existing,
 * tested push code. User uploads exist only on the master's flash until a
 * file-sync path exists, and the master refuses to send one to the peer.
 *
 * Commands use the rail's seq/op scheme: continuous fields (rpm, brightness,
 * contentId) are applied on every packet, one-shot ops only when seq changes.
 */
enum : uint8_t {
  UNIT_OP_NONE = 0,
  UNIT_OP_POWER_ON = 1,
  UNIT_OP_POWER_OFF = 2,   // controlled ramp-down
  UNIT_OP_ESTOP = 3,       // remove drive, coast
  UNIT_OP_CLEAR_FAULT = 4,
};

struct __attribute__((packed)) UnitCommandMsg {
  uint8_t type;     // MSG_UNIT_COMMAND
  uint8_t version;  // PROTOCOL_VERSION
  uint8_t seq;      // bump to execute `op`
  uint8_t op;       // UNIT_OP_*
  uint16_t rpmTarget;   // applied every packet; the peer clamps as it would a web request
  uint8_t brightness;   // 0-31, applied every packet
  uint8_t loopSeq;      // relayed to the peer's rotor as CommandMsg.loopSeq
  char contentId[24];   // "" = leave alone; else select and push to the peer's rotor
  uint8_t flags;        // UNIT_CFLAG_*
  uint8_t sectorOffset; // relayed to the peer's rotor as CommandMsg.sectorOffset
  uint8_t calFlags;     // ... and CommandMsg.calFlags
  uint8_t startFrame;   // ... and CommandMsg.startFrame (was reserved, sent as 0)
};
static_assert(sizeof(UnitCommandMsg) == 36, "UnitCommandMsg size changed");

enum : uint8_t {
  UNIT_CFLAG_MIRROR = 1 << 0,  // relayed to the peer's rotor as CMD_FLAG_MIRROR
};

enum : uint8_t {
  UNIT_TFLAG_POWER = 1 << 0,
  UNIT_TFLAG_FAULT = 1 << 1,
  UNIT_TFLAG_ROTOR_ONLINE = 1 << 2,
  UNIT_TFLAG_ROTOR_HAS_CONTENT = 1 << 3,
  UNIT_TFLAG_ROTOR_SYNC_LOCKED = 1 << 4,
  UNIT_TFLAG_FAULT_CAUSE_ACTIVE = 1 << 5,  // clear would be refused right now
  UNIT_TFLAG_PUSHING = 1 << 6,             // content transfer to its rotor in progress
};

struct __attribute__((packed)) UnitTelemetryMsg {
  uint8_t type;      // MSG_UNIT_TELEMETRY
  uint8_t version;   // PROTOCOL_VERSION
  uint8_t state;     // FanState as uint8: 0 idle 1 starting 2 running 3 stopping 4 fault
  uint8_t flags;     // UNIT_TFLAG_*
  uint8_t lastSeq;   // command seq most recently acted on
  uint8_t brightness;
  int16_t rpmActual;
  uint16_t rpmTarget;
  uint16_t rotorRpm;
  int8_t rotorRssi;
  uint8_t rotorHallLevel;  // relayed TelemetryMsg.hallLevel; 2 = rotor offline
  int16_t voltageQx100;    // voltage.q * 100
  uint16_t focLoopHz;
  uint16_t focMaxGapMsx100;
  char faultCode[4];       // "E09" etc, "" when none
  char contentId[24];
  char faultMessage[64];   // truncated user-facing text
};
static_assert(sizeof(UnitTelemetryMsg) == 112, "UnitTelemetryMsg size changed");

/* ===========================================================================
 *  FILE SYNC (master -> peer stator)
 * ===========================================================================
 * User uploads land only on the master's LittleFS. To show one on the right
 * fan, the master copies the whole .povf file (header included) to the peer
 * stator's LittleFS under the SAME id; from then on the peer treats it like
 * a preset and pushes it to its rotor with the normal content path.
 *
 * Flow control is application-level stop-and-wait: the peer writes each chunk
 * to flash and only then acks bytesReceived, because a MAC-layer ack says the
 * radio got it, not that the flash has it. Writes stall the peer's FOC loop
 * (gotcha 6), so the peer refuses (FILE_ERR_SPINNING) unless its arm is
 * stopped. A BEGIN for a file the peer already holds with the same size and
 * checksum is answered FILE_OK at once, with no write, so the master can check
 * "have you got it?" even while the arm turns. */
struct __attribute__((packed)) FileBeginMsg {
  uint8_t type;       // MSG_FILE_BEGIN
  uint8_t version;    // PROTOCOL_VERSION
  uint8_t reserved[2];
  uint32_t transferId;
  uint32_t totalBytes;  // whole file, 16-byte .povf header included
  uint32_t checksum;    // 32-bit sum of every byte of the file
  char id[24];
  char name[32];        // display name, for the peer's .nam sidecar
};
static_assert(sizeof(FileBeginMsg) == 72, "FileBeginMsg size changed");

/* Followed by dataLen bytes of file. */
struct __attribute__((packed)) FileChunkMsg {
  uint8_t type;      // MSG_FILE_CHUNK
  uint8_t version;
  uint16_t dataLen;
  uint32_t transferId;
  uint32_t offset;   // from the start of the FILE
};
static_assert(sizeof(FileChunkMsg) == 12, "FileChunkMsg size changed");

struct __attribute__((packed)) FileEndMsg {
  uint8_t type;      // MSG_FILE_END
  uint8_t version;
  uint8_t reserved[2];
  uint32_t transferId;
  uint32_t totalBytes;
  uint32_t checksum;
};
static_assert(sizeof(FileEndMsg) == 16, "FileEndMsg size changed");

struct __attribute__((packed)) FileDeleteMsg {
  uint8_t type;      // MSG_FILE_DELETE
  uint8_t version;
  uint8_t reserved[2];
  uint32_t transferId;
  char id[24];
};
static_assert(sizeof(FileDeleteMsg) == 32, "FileDeleteMsg size changed");

struct __attribute__((packed)) FileAckMsg {
  uint8_t type;      // MSG_FILE_ACK
  uint8_t version;
  uint8_t status;    // FILE_*
  uint8_t reserved;
  uint32_t transferId;
  uint32_t bytesReceived;  // written to flash so far
  char id[24];
};
static_assert(sizeof(FileAckMsg) == 36, "FileAckMsg size changed");

enum : uint8_t {
  FILE_OK = 0,             // the peer holds this file, complete and verified
  FILE_IN_PROGRESS = 1,    // bytesReceived written; send the next chunk
  FILE_DELETED = 2,        // the file is gone (or never existed)
  FILE_ERR_SPINNING = 3,   // the peer's arm is turning: flash writes refused
  FILE_ERR_SPACE = 4,      // no room, or the peer's library is full
  FILE_ERR_INVALID = 5,    // header or size rejected
  FILE_ERR_CHECKSUM = 6,   // bytes arrived but did not add up; file discarded
  FILE_ERR_IO = 7,         // flash write failed
};

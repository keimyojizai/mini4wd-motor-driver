// ============================================================
// Mini4AI MG24 Rules Realtime Firmware
// v3.58-r10-soft-start: ramp startup duty toward NORMAL_DUTY; never exceed target during launch.
// v3.58-safety-review: IMU start gate, START reentry guard, lap debounce fix, control slip clamp.
// v3.38-vbat-log: stopped log rows now include event-time Battery VBAT.
// v3.37-reliable-ack-dump: stopped log dump is now UI-pulled by sequence number.
//      CMD_LOG_REQUEST sends a META packet; UI requests each row with CMD_LOG_GET.
//      Lost BLE notifications are recovered by requesting the same seq again.
// v3.36-log-capacity: LOG_CAP 384->768 so wave-heavy runs (which emit more rows
//      since v3.35's reversal detector) don't overflow the on-device log mid-run.
// v3.35-corner-reversal: detect left<->right reversals that occur while still
//      inCorner (yaw never dipped below STRAIGHT_TH for EXIT_DEBOUNCE_MS). The
//      old detector merged these and dropped the new segment, which skipped
//      segments at speed and missed waves. Now each reversal closes the current
//      corner (emits its PEAK) and begins a new LEFT/RIGHT, bypassing the
//      per-segment cooldown so fast wave lobes are not suppressed. >=2 quick
//      reversals within WAVE_WINDOW_MS also emit SEG_WAVE (amplitude-independent
//      wave path that complements the legacy +/- spike-pair detector).
// v3.31-wave-legacy-restore: rollback v3.30 edge/reversal WAVE detector.
//      Keep the older continuously-refreshed +/- spike-pair detector because
//      real course tests detected WAVE more often with it, although it may still miss some.
// v3.28-live-seg-queue: live debug segment notify now uses a small FIFO and sends
//      [seg, elapsedMs u32, durMs u16] so realtime lap timing does not lose
//      repeated/fast segment events before the stopped log is downloaded.
// v3.32-live-seg-drain: DEBUG live segment notify no longer emits only one event every 150ms.
//      BLE loop drains the live segment FIFO every poll so realtime lap display sees the same
//      segment order as the stopped log as much as BLE allows. FIFO enlarged 12->32.
// v3.27-repeat-action-pattern: terminology update for the Web UI. EVERY_N + triggerPhase is generic:
//      it executes the configured action sequence on the selected item in an N-cycle,
//      not specifically LC and not specifically deceleration. Protocol unchanged from v3.26.
// v3.25-highrate-telemetry: BLE telemetry notify is raised from 2Hz to ~20Hz.
//      VBAT uses the existing 10ms control-side sample so voltage/sag charts
//      show finer transients without slowing the control loop. Board temp is cached.
// v3.24-scroll-sag-split: firmware unchanged from v3.23; UI splits sag chart and adds timeline review
//      for every segment/action row so load-heavy course sections can be visualized.
// v3.22-sag-log: stopped log includes final action duration plus voltage-correction
//      delta and load-sag delta for WAIT/BRAKE rows. Sag remains optional/gated.
// v3.21-sag-mode: adaptive WAIT/BRAKE separates stopped/unloaded VBAT correction
//      from running load-sag correction. Sag can be OFF / always / duty-thresholded
//      so intentionally low-duty sections do not look like speed/load sag.
// v3.20-running-vbat-mix: adaptive WAIT/BRAKE can blend stopped/unloaded VBAT with
//      recent running/loaded VBAT. Running VBAT acts as a speed/load proxy.
// v3.19-voltage-coeff-compatible: no protocol break. The Web UI can set a
//      per-rule reference voltage and signed WAIT/BRAKE coefficients, then
//      converts them to the existing low/high endpoint map before writing this
//      adaptive-brake characteristic. Firmware continues to execute the endpoint
//      map for standalone reliability and backward compatibility.
// v3.17-explicit-save: BLE writes only apply to runtime/RAM.
//      EEPROM-backed flash is updated only by CMD_SAVE_CONFIG from the UI.
// v3.16-standalone-boot: upside-down power-on arms standalone mode;
//      face-up stable -> 5s countdown -> auto start using the saved config.
// v3.15-selective-unloaded-adaptive-brake: voltage-adaptive WAIT/BRAKE can target
//      first WAIT/BRAKE pair, all WAIT/BRAKE actions, or selected action indices.
// v3.14-unloaded-adaptive-brake: use unloaded VBAT estimate from stop/short-brake windows
//      for voltage-adaptive WAIT/BRAKE interpolation; telemetry adds unloaded VBAT and sag
// v3.13-adaptive-brake: voltage-based wait/brake interpolation per rule
// v3.12-telemetry: added low-rate board temp / VBAT / MID / VBAT-MID BLE telemetry
// v3.33-device-lap-log: firmware-side lap boundary matcher/log.
//      UI sends LapPattern as a compact trailer on rulesChar; firmware stores
//      confirmed lap records separately and appends them to stopped log dump as SEG_LAP.
//      BLE/live/log overflows no longer destroy final lap times.
// v3.34-compile-fix: add forward declarations for Arduino's auto-prototype pass.
//      Some Arduino builders emit function prototypes before struct definitions;
//      helpers using EventLog/LapRecord/PersistentConfig by reference/value then fail to compile.
// v3.11.1: BLE invisibility fix — bulkChar max-len was 20B but v3.11
//          enlarged dump packets to 25B, which silently broke advertise
//          on some boards. Bumped bulkChar to 32B and trimmed LOG_CAP
//          384→256 to keep RAM in check after EventLog grew.
// v3.11: log entries enriched with curve peak (peakDps10) and segment
//        duration (durMs). Lets the host show "this curve peaked at
//        500dps and lasted 230ms" rather than just gz at the moment.
// v3.10: PEAK detection: relative %-drop from cornerPeakAbs
//        (CURVE_PEAK_DROP_PCT, default 95%). Was absolute dps drop which
//        fired too late — effectively at corner exit.
// v3.9: time-only wildcard pattern element (SEG_WILDCARD)
// v3.8: per-rule independent timing for curveLong / peakAfter / straightHold
// XIAO MG24 Sense + LSM6DS3 + BLE
//
// Production policy:
//   - No realtime gyro/features notify
//   - No Web-side decision making during run
//   - 200 Hz local IMU -> rule -> PWM control
//   - BLE is only config/start/stop + stopped log dump
//   - Optional low-rate segment event notify for debugging only
//
// BLE service/major UUIDs are kept compatible with the previous app
// where practical, so the Web UI can be swapped without changing the
// board identity.
// ============================================================

// Arduino IDE may auto-generate function prototypes before later struct definitions.
// Forward declarations keep those generated prototypes valid.
struct EventLog;
struct LapRecord;
struct PersistentConfig;

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoBLE.h>
#include <EEPROM.h>
#include <LSM6DS3.h>
#include <math.h>
#include <string.h>

// ============================================================
// Hardware
// ============================================================
#if defined(__has_include)
  #if __has_include("firmware_config.h")
    #include "firmware_config.h"
  #endif
#endif
#ifndef MINI4AI_DEFAULT_DEVICE_NAME
#define MINI4AI_DEFAULT_DEVICE_NAME "Mini4AI"
#endif
#define DEFAULT_DEVICE_NAME MINI4AI_DEFAULT_DEVICE_NAME
#define FW_VERSION "v3.58-r10"
static const uint8_t DEVICE_NAME_MAX_LEN = 20;
static char deviceName[DEVICE_NAME_MAX_LEN + 1] = {0};

static const uint8_t PWM_PIN    = PC2;
static const uint8_t IMU_EN_PIN = PD5;

// Board telemetry pins. README wiring: PC0(D0)=VBAT, PC1(D1)=NTC, PC3(D3)=MID.
static const uint8_t VBAT_SENSE_PIN = PC0;
static const uint8_t NTC_TEMP_PIN    = PC1;
static const uint8_t MID_SENSE_PIN   = PC3;

// Analog scaling. Adjust VBAT/MID dividers here if the PCB resistor ratio differs.
static const float ADC_REF_V        = 3.300f;
static const float ADC_COUNTS       = 4095.0f;
static const float VBAT_DIV_SCALE   = 3.200f;  // 22k:10k divider: pin voltage × 3.2 -> battery voltage
static const float MID_DIV_SCALE    = 3.200f;  // 22k:10k divider: pin voltage × 3.2 -> M- / MID voltage
static const float NTC_PULLUP_OHM   = 10000.0f;
static const float NTC_NOMINAL_OHM  = 10000.0f;
static const float NTC_BETA         = 4100.0f;
static const float NTC_NOMINAL_C    = 25.0f;
static const bool  NTC_TO_GND       = true;    // true: 3V3--Rpullup--ADC--NTC--GND

#ifndef LED_BUILTIN
#define LED_BUILTIN LED_RED
#endif

LSM6DS3 myIMU(I2C_MODE, 0x6A);
static bool imuReady = false;

// ============================================================
// Control constants
// ============================================================
static const uint32_t CONTROL_HZ      = 200;
static const uint32_t CONTROL_DT_MS   = 1000 / CONTROL_HZ;  // 5 ms
static const uint32_t CONTROL_SLIP_CLAMP_MS = 50;           // after a stall, do not burst-catch-up control ticks
static const uint32_t BLE_POLL_MS     = 20;                 // 50 Hz max when stopped/configuring
static const uint32_t BLE_POLL_RUNNING_MS = 50;              // control-priority run mode: poll only for STOP/config writes
static const uint32_t TELEMETRY_MS    = 50;                 // stopped/config telemetry notify (~20 Hz)
static const uint32_t RUNNING_TELEMETRY_MS = 1000;            // v3.54: header-only running status telemetry (~1 Hz)
// v3.55: separated high-density stopped voltage timeline from event log.
// v3.54: keep graphs stopped during run, but notify a small status telemetry packet
// about once per second so the header can show VBAT / board temperature.
static const bool     RUNNING_HEADER_TELEMETRY_BLE = true;
static const uint32_t RUNNING_VOLTAGE_LOG_MS = 25;              // v3.55: stopped-log voltage timeline (~40 Hz, memory-only during run)
static const uint32_t VBAT_SAMPLE_MS   = 10;                 // local ADC sample for telemetry/adaptive brake (100 Hz)
static const uint32_t UNLOADED_VBAT_SETTLE_MS = 40;            // ignore first noisy ms after entering brake/stop
static const uint32_t LIVE_EVENT_MS   = 150;                // optional debug only
// v3.39: Live/debug segment notify must not steal unbounded time from the 200 Hz control loop.
// Send only a few live packets per BLE poll and never BLE.poll() per packet.
static const uint8_t  LIVE_SEG_SEND_BUDGET_PER_BLE_POLL = 3;
// v3.51: during a run, never emit an unbounded burst of BLE packets.
// Each BLE service pass may write at most one application payload: either
// a pending lap event or one sparse voltage telemetry packet. STOP/config
// reception still happens through BLE.poll().
static const uint8_t  RUNNING_BLE_PAYLOAD_BUDGET_PER_POLL = 1;
// v3.43: Production-control safe. Do not notify full live segments while running;
// stopped ACK log is used for complete post-run analysis.
// v3.44: SEG_LAP is still notified once per confirmed lap for realtime lap display.
// v3.45: SEG_LAP live notify includes event-time Battery VBAT (9-byte packet).
// v3.47: SEG_LAP live notify/stopped lap log also include max WAIT-load sag
//      observed during the lap. This is telemetry only; no correction is applied.
// v3.57: live timing patch allows inserting one SPEED action for brake replacement tuning.
// v3.48: WAIT-sag tracking no longer depends only on activeActionIdx.
//      A dedicated WAIT measurement window is opened when ACTION_WAIT starts,
//      so the sag value is captured reliably and sent in realtime lap/log rows.
// v3.49: SEG_LAP live notify also includes minimum Battery VBAT observed during
//      ACTION_BRAKE in the lap. This is logged only for analysis/correlation;
//      it is not used for compensation or auto-tune decisions.
// v3.51: control-priority scheduler: realtime control remains non-blocking,
//      BLE writes are sparse/deferred, and stopped logs use a ring buffer so
//      the newest diagnostic rows are retained if the run is log-heavy.
//      Running voltage timeline is recorded to the stopped log with timestamps;
//      BLE graph telemetry during run is disabled by default.
static const bool     LIVE_SEG_NOTIFY_DURING_RUN = false;
// v3.39: in-corner reversal must require a strong opposite swing.
// Do not tie it directly to CORNER_TH; lowering CORNER_TH to catch gentle 180s
// otherwise splits one long curve into LEFT->RIGHT->LEFT and breaks EVEN/ODD rules.
static const float    REVERSAL_MIN_ABS_DPS = 300.0f;
static const uint16_t GZ_BIAS_SAMPLES = 80;
static const uint8_t  GZ_FILT_N       = 3;

// Segment IDs. Kept intentionally simple.
static const uint8_t SEG_NONE     = 0;
static const uint8_t SEG_STRAIGHT = 1;
static const uint8_t SEG_RIGHT    = 2;  // gz negative in this firmware
static const uint8_t SEG_LEFT     = 4;  // gz positive in this firmware
static const uint8_t SEG_WAVE     = 6;
static const uint8_t SEG_LEFT_PEAK       = 7;  // emitted after LEFT curve peak passed
static const uint8_t SEG_RIGHT_PEAK      = 8;  // emitted after RIGHT curve peak passed
static const uint8_t SEG_STRAIGHT_HOLD   = 9;  // emitted after straight continues for configured time
static const uint8_t SEG_LEFT_HOLD        = 10; // emitted when a LEFT curve continues for configured time
static const uint8_t SEG_RIGHT_HOLD       = 11; // emitted when a RIGHT curve continues for configured time
// v3.9 wildcard: a pattern-only segment ID that never appears as a real
// detected segment. While the rule's matchIdx points at SEG_WILDCARD, every
// incoming segment is ignored — no advance, no reset. After wildcardMs has
// elapsed, the matcher auto-advances past the wildcard and resumes normal
// matching on the next pattern element. Use case: "ignore everything for
// 2 seconds, then fire on the next LEFT_PEAK".
static const uint8_t SEG_WILDCARD         = 14;
static const uint8_t SEG_LAP              = 15;  // synthetic stopped-log row: firmware-confirmed lap
static const uint8_t SEG_RUN_START        = 16;  // diagnostic stopped-log row: run started
static const uint8_t SEG_RUN_STOP         = 17;  // diagnostic stopped-log row: run stopped
static const uint8_t SEG_VOLTAGE          = 18;  // diagnostic stopped-log row: timestamped voltage sample

// Actions
static const uint8_t ACTION_NONE  = 0;
static const uint8_t ACTION_BRAKE = 1;
static const uint8_t ACTION_SPEED = 2;
static const uint8_t ACTION_STOP  = 4;
static const uint8_t ACTION_WAIT  = 5;
// Global action lockout. While active, no new rule can preempt motor control;
// existing motor duty is held. Useful for "ignore everything for N ms after
// landing from a jump" or "skip any braking right after the start line".
static const uint8_t ACTION_LOCK  = 6;
// Pseudo action used only in stopped logs when a pending rule is canceled during its confirmation window.
static const uint8_t ACTION_CANCEL = 7;

// Voltage-adaptive brake target selection.
// FIRST_PAIR preserves the original behavior: first WAIT and the first BRAKE after it.
// ALL applies to every WAIT/BRAKE action in the rule.
// SELECTED applies only to action indices set in adaptiveTargetMask.
static const uint8_t ADAPT_TARGET_FIRST_PAIR = 0;
static const uint8_t ADAPT_TARGET_ALL        = 1;
static const uint8_t ADAPT_TARGET_SELECTED   = 2;

// Load-sag correction mode. Sag is unloaded/rest VBAT minus recent running VBAT.
// DUTY_MIN prevents intentional low-duty sections from being interpreted as sag.
static const uint8_t ADAPT_SAG_OFF       = 0;
static const uint8_t ADAPT_SAG_ALWAYS    = 1;
static const uint8_t ADAPT_SAG_DUTY_MIN  = 2;

// Rule trigger modes. The match counter is per rule and increments every time
// the pattern is recognized, regardless of whether actions fire.
static const uint8_t TRIG_EVERY   = 0;
static const uint8_t TRIG_ODD     = 1;
static const uint8_t TRIG_EVEN    = 2;
static const uint8_t TRIG_EVERY_N = 3;
static const uint8_t TRIG_ONLY_N  = 4;

// Commands
static const uint8_t CMD_STOP           = 0;
static const uint8_t CMD_ARM            = 1;  // legacy: reset/ready only. UI v3.1 does not expose this.
static const uint8_t CMD_START          = 2;
static const uint8_t CMD_LOG_REQUEST    = 4;
static const uint8_t CMD_CLEAR_LOG      = 5;
static const uint8_t CMD_SAVE_CONFIG    = 6;  // explicit EEPROM-backed flash save; BLE writes alone are temporary.
static const uint8_t CMD_LOG_GET        = 7;  // v3.37: request one stopped-log row by sequence [cmd, seqLo, seqHi].
static const uint8_t CMD_LOG_ABORT      = 8;  // v3.37: cancel stopped-log transfer.

// Status
static const uint8_t STATUS_IDLE    = 0;
static const uint8_t STATUS_READY   = 1;   // legacy ARM value, UI no longer exposes ARM
static const uint8_t STATUS_RUNNING = 2;
static const uint8_t STATUS_STOPPED = 3;
static const uint8_t STATUS_ERROR   = 4;

// Rule packet limits
static const uint8_t MAX_PATTERN_LEN = 8;
static const uint8_t MAX_RULES       = 8;
static const uint8_t MAX_ACTIONS     = 5;
// v3.41: Rule packet expanded to 47 bytes. Layout:
//   q[0]                  patternLen
//   q[1..8]               pattern[8]
//   q[9]                  actionCount
//   q[10..14]             action types[5]
//   q[15..24]             action durations u16 LE [5]
//   q[25..29]             action strengths[5]
//   q[30]                 loopMode
//   q[31]                 triggerMode
//   q[32]                 triggerN
//   q[33]                 priority (0=legacy auto)
//   q[46]                 triggerPhase: in EVERY_N, which item in the repeated set fires (1..triggerN)
//   q[34..35]             peakAfterOverrideMs    (0xFFFF=use global)
//   q[36..37]             straightHoldOverrideMs (0xFFFF=use global)
//   q[38..39]             curveLongOverrideMs    (0xFFFF=use global)
//   q[40..41]             wildcardMs
//   q[42..43]             confirmMs (0 = disabled)
//   q[44..45]             confirmCancelMask (bit[segment id], e.g. RIGHT/WAVE)
// Header byte buf[0] = protocol version: 2 (36B) | 3 (42B) | 4 (46B) | 5 (47B).
static const uint8_t RULE_BYTES      = 47;
static const uint8_t RULE_BYTES_V4   = 46;
static const uint8_t RULE_BYTES_V3   = 42;
static const uint8_t RULE_BYTES_V2   = 36;  // legacy v36 packet, still accepted

// ============================================================
// Tunable parameters, writable from params characteristic
// Packet: 30 bytes, LE
// [0..1]   cornerTh dps
// [2..3]   straightTh dps
// [4..5]   wavePosTh dps
// [6..7]   waveNegTh dps
// [8..9]   wavePairWindow ms
// [10..11] waveLatch ms
// [12..13] waveCooldown ms
// [14..15] exitDebounce ms
// [16]     normalDuty
// [17]     recoverDuty
// [18]     startDuty
// [19]     flags: bit0 enable low-rate live segment notify
// [20..21] curvePeakAfterMs: peak must be older than this
// [22..23] curvePeakDropDps: emit peak after abs(gz) drops this much from peak
// [24..25] straightHoldMs: emit STRAIGHT_HOLD after this duration below STRAIGHT_TH
// [26..27] segmentCooldownMs: suppress duplicate same segment events inside this window
// [28..29] curveLongMs: curve must continue this long to emit LEFT/RIGHT_HOLD
// ============================================================
static float    CORNER_TH       = 260.0f;
static float    STRAIGHT_TH     = 220.0f;
static float    WAVE_POS_TH     = 200.0f;
static float    WAVE_NEG_TH     = 200.0f;
static uint32_t WAVE_WINDOW_MS  = 200;
static uint32_t WAVE_LATCH_MS   = 1200;
static uint32_t WAVE_COOLDOWN_MS= 400;
static uint32_t EXIT_DEBOUNCE_MS= 15;
static uint8_t  NORMAL_DUTY     = 255;
static uint8_t  RECOVER_DUTY    = 100;
static uint8_t  START_DUTY      = 64;
static bool     DEBUG_LIVE_SEG  = false;
static uint32_t CURVE_PEAK_AFTER_MS = 30;
// v3.10: PEAK 発火条件は「ピーク値からの絶対 dps ドロップ」ではなく
// 「ピーク値の何 % まで角速度が落ちたか」の相対判定に変更。
// ピーク値の (CURVE_PEAK_DROP_PCT)% を一度でも下回ったら PEAK 発火。
// デフォルト 95 → ピーク 500 dps なら 475 dps を割った瞬間に発火 (5% 落下)。
// この値だと PEAK は実質 peakAfter (= 30ms 既定) 後にすぐ立つ → 「ピーク直後」。
// 値を下げる (80 など) と "出口寄り" の発火に近づく。
// 旧 CURVE_PEAK_DROP_DPS (絶対 dps) は廃止。BLE プロトコルでは同じ
// バイト位置 p[22..23] を再利用。
static uint8_t  CURVE_PEAK_DROP_PCT = 95;
static uint32_t STRAIGHT_HOLD_MS    = 250;
static uint32_t SEGMENT_COOLDOWN_MS = 80;
static uint32_t CURVE_LONG_MS        = 180;

// Startup ramp. START_DUTY is the initial duty and is clamped so it never
// exceeds NORMAL_DUTY. This prevents a 100% launch when the user selected a
// lower running duty. Set START_DUTY=NORMAL_DUTY for instant start.
static const uint32_t SOFT_START_MS = 300;

// Standalone boot mode.
// Power on while the car is upside down -> arm standalone mode.
// Turn it face up, place it on the course, and keep it still/face-up; after
// STANDALONE_START_DELAY_MS the saved BLE configuration starts without Web/BLE.
// If the board mounting makes normal face-up read accel Z negative, swap the
// signs of the two thresholds below.
static const bool     STANDALONE_BOOT_ENABLE        = true;
static const float    STANDALONE_UPSIDE_DOWN_Z_G    = -0.55f;
static const float    STANDALONE_FACE_UP_Z_G        =  0.55f;
static const uint32_t STANDALONE_FACE_UP_STABLE_MS  = 500;
static const uint32_t STANDALONE_START_DELAY_MS     = 3000;


// ============================================================
// Rule model
// ============================================================
struct RuleAction {
  uint8_t  type;
  uint16_t durationMs;
  uint8_t  strength;    // for SPEED: duty 0..255. If 1..4, maps to 25/50/75/100%.
};

struct Rule {
  uint8_t pattern[MAX_PATTERN_LEN];
  uint8_t patternLen;
  RuleAction actions[MAX_ACTIONS];
  uint8_t actionCount;
  bool loopMode;
  uint8_t matchIdx;
  uint8_t triggerMode;
  uint8_t triggerN;
  uint8_t triggerPhase;   // v3.41: EVERY_N fires on this item within the N-cycle, 1..triggerN
  uint8_t priority;     // 1..255. Higher value wins. 0 in packet means legacy auto-priority.
  uint16_t matchCount;
  bool enabled;
  // v37 per-rule timing overrides. 0xFFFF = inherit global.
  uint16_t peakAfterOverrideMs;
  uint16_t straightHoldOverrideMs;
  uint16_t curveLongOverrideMs;
  // v38: Per-rule segment emission flags. Reset at curve/straight start.
  // Each rule independently tracks whether its own override-time threshold
  // has fired for the current curve/straight. Rules without an override
  // (0xFFFF) are driven by the global firing instead — see global flags.
  bool perRuleCurveHoldEmitted;
  bool perRulePeakEmitted;
  bool perRuleStraightHoldEmitted;
  // v3.9 wildcard timing. wildcardMs is the configured wait time. While the
  // matcher's matchIdx points at a SEG_WILDCARD position, all incoming
  // segments are ignored. wildcardEntryMs is the timestamp at which matchIdx
  // landed on the wildcard (0 = not yet entered). The periodic
  // tickRuleWildcards() advances past the wildcard after wildcardMs.
  uint16_t wildcardMs;
  uint32_t wildcardEntryMs;

  // v3.21: voltage-adaptive WAIT/BRAKE.
  // Base correction uses stopped/unloaded VBAT. Load-sag correction is separate:
  // sag = stopped/unloaded VBAT - recent running VBAT. This avoids coupling
  // low-battery compensation and load/speed compensation.
  bool adaptiveBrakeEnabled;
  uint16_t adaptiveVLowMv;
  uint16_t adaptiveVHighMv;
  uint16_t adaptiveWaitLowMs;
  uint16_t adaptiveWaitHighMs;
  uint16_t adaptiveBrakeLowMs;
  uint16_t adaptiveBrakeHighMs;
  uint16_t adaptiveSampleMs;
  uint8_t adaptiveRunVbatMixPct; // legacy v3.20 only; v3.21 keeps this at 0
  uint8_t adaptiveSagMode;       // 0=off, 1=always, 2=only when duty >= adaptiveSagDutyMinPct
  uint8_t adaptiveSagDutyMinPct; // current PWM duty threshold in % of 255
  int16_t adaptiveSagWaitMsPerV;
  int16_t adaptiveSagBrakeMsPerV;
  uint8_t adaptiveTargetMode; // 0:first WAIT/BRAKE, 1:all WAIT/BRAKE, 2:selected indices
  uint8_t adaptiveTargetMask; // bit i = action index i is adaptive target when mode=selected

  // v39: pre-action confirmation gate.
  // When a pattern completes, actions are not fired immediately if confirmMs>0
  // and confirmCancelMask is non-zero. The rule enters a pending state. During
  // confirmMs, any segment whose bit is set in confirmCancelMask cancels this
  // candidate and restores the prefix before the final pattern element. If no
  // cancel segment appears, actions fire after confirmMs. This separates
  // "is this candidate real?" from the subsequent WAIT used to place braking.
  uint16_t confirmMs;
  uint16_t confirmCancelMask;
  bool confirmPending;
  uint32_t confirmStartMs;
  uint8_t confirmPrefixIdx;
  float confirmGz;
};

static Rule rules[MAX_RULES];
static uint8_t rulesCount = 0;

// ============================================================
// Runtime state
// ============================================================
static bool bleReady = false;
static bool bleConnected = false;
static bool isRunning = false;
static uint8_t statusValue = STATUS_IDLE;

static uint32_t runStartMs = 0;
static uint32_t lastControlMs = 0;
static uint32_t lastBlePollMs = 0;
static uint32_t lastTelemetryMs = 0;
static uint32_t lastLiveEventMs = 0;
static uint32_t lastVbatSampleMs = 0;
static uint32_t lastVoltageLogMs = 0;

static const uint8_t VBAT_RING_N = 64;
static uint32_t vbatRingMs[VBAT_RING_N] = {0};
static uint16_t vbatRingMv[VBAT_RING_N] = {0};
static uint8_t vbatRingPos = 0;
static uint8_t vbatRingCount = 0;
static uint16_t lastVbatMv = 0xFFFF;
static uint16_t unloadedVbatMv = 0xFFFF;       // estimated no-motor-load battery voltage [mV]
static uint32_t unloadedVbatLastUpdateMs = 0;  // last update time for telemetry/debug
static int16_t cachedBoardTemp10 = -32768;
static uint32_t lastBoardTempSampleMs = 0;
static const uint32_t BOARD_TEMP_SAMPLE_MS = 1000;   // temp is slow; do not read NTC at telemetry rate
static uint32_t unloadedBrakeStartMs = 0;      // time when PWM entered 0% brake/stop
static bool unloadedBrakeWindowActive = false;

// Standalone boot/autostart state.
static bool standaloneBootArmed = false;
static bool standaloneCountdownActive = false;
static uint32_t standaloneFaceUpSinceMs = 0;
static uint32_t standaloneStartAtMs = 0;
static uint32_t standaloneLastAccelCheckMs = 0;

static float gzBias = 0.0f;
static float gzBuf[GZ_FILT_N] = {0};
static uint8_t gzIdx = 0;

static bool inCorner = false;
static int8_t cornerSign = 0;     // +1 left, -1 right
static uint32_t cornerStartMs = 0;
static uint32_t straightCandMs = 0;
static uint32_t straightHoldStartMs = 0;
static bool straightHoldEmitted = false;
static float cornerPeakAbs = 0.0f;
static uint32_t cornerPeakMs = 0;
static bool cornerPeakEmitted = false;
static bool cornerHoldEmitted = false;
static uint32_t lastSegEventMs[16] = {0};
static uint32_t lastPosSpikeMs = 0;
static uint32_t lastNegSpikeMs = 0;
static uint32_t waveLatchUntilMs = 0;
static uint32_t waveCooldownUntilMs = 0;
// v3.35 corner-reversal: track left<->right reversals that happen while still
// inCorner (i.e. the yaw never dipped below STRAIGHT_TH long enough to exit).
// These are the events the old detector silently merged, causing skipped
// segments at speed and missed waves.
static uint32_t lastReversalMs = 0;     // time of the most recent in-corner reversal
static uint8_t  reversalRunCount = 0;   // consecutive quick reversals (wave-by-structure)
static const uint32_t REVERSAL_MIN_AGE_MS = 10;  // ignore reversals immediately after entry (noise guard)
static uint8_t lastSegment = SEG_NONE;

// Action queue execution
static RuleAction activeActions[MAX_ACTIONS];
static int16_t activeVoltageDeltaMs[MAX_ACTIONS] = {0};
static int16_t activeSagDeltaMs[MAX_ACTIONS] = {0};
static uint8_t activeActionOriginalIdx[MAX_ACTIONS] = {0};
static uint8_t activeActionCount = 0;
static uint8_t activeActionIdx = 0;
static bool actionActive = false;
static uint32_t actionEndMs = 0;
static uint8_t activeRuleIdx = 0xFF;
static uint8_t activePriority = 0;
static float activeTriggerGz = 0.0f;
static uint8_t currentMotorDuty = 0;

// Global action lockout: while now < actionLockUntilMs, no new rule can
// trigger motor changes (BRAKE/SPEED/STOP). Segment detection and rule
// matching continue normally; only the motor-side dispatch is suppressed.
static uint32_t actionLockUntilMs = 0;

// Optional low-rate live event notify queue.
// v3.28: keep a small FIFO instead of one overwritten byte. The Web UI can use
// elapsedMs for lap timing; if an old UI is used it still reads byte0 as seg.
struct LiveSegPacket {
  uint8_t seg;
  uint32_t elapsedMs;
  uint16_t durMs;
  int16_t vbatMv;       // v3.45: event-time Battery VBAT [mV], -32768 if invalid
  int16_t waitSagMv;    // v3.47: max WAIT-load sag since previous lap [mV], -32768 if invalid
  int16_t brakeVbatMv;  // v3.49: min Battery VBAT observed during ACTION_BRAKE [mV], -32768 if invalid
};
static const uint8_t LIVE_SEG_QUEUE_N = 32;
static LiveSegPacket liveSegQueue[LIVE_SEG_QUEUE_N];
static uint8_t liveSegHead = 0;
static uint8_t liveSegTail = 0;
static uint8_t liveSegCount = 0;

static void pushLiveSegPacketRaw(uint8_t seg, uint32_t now, uint32_t durMs, int16_t vbatMv = -32768, int16_t waitSagMv = -32768, int16_t brakeVbatMv = -32768) {
  LiveSegPacket &p = liveSegQueue[liveSegHead];
  p.seg = seg;
  p.elapsedMs = (runStartMs != 0) ? (uint32_t)(now - runStartMs) : 0UL;
  p.durMs = (uint16_t)constrain(durMs, 0UL, 65535UL);
  p.vbatMv = vbatMv;
  p.waitSagMv = waitSagMv;
  p.brakeVbatMv = brakeVbatMv;
  liveSegHead = (uint8_t)((liveSegHead + 1) % LIVE_SEG_QUEUE_N);
  if (liveSegCount < LIVE_SEG_QUEUE_N) {
    liveSegCount++;
  } else {
    // Queue full: keep FIFO order and drop the newest event.
    liveSegHead = (uint8_t)((liveSegHead + LIVE_SEG_QUEUE_N - 1) % LIVE_SEG_QUEUE_N);
  }
}

static void queueRealtimeLapNotify(uint32_t now, uint32_t lapMs, int16_t waitSagMv = -32768, int16_t brakeVbatMv = -32768) {
  // Production-safe realtime notification: one compact packet per completed lap.
  // v3.49 packet: [seg, elapsedMs u32, lapMs u16, Battery VBAT i16, WAIT-sag i16, Brake VBAT i16].
  // This is not the old live segment debug stream and should not disturb control timing.
  uint16_t mv = lastVbatMv;
  int16_t lapVbatMv = (mv == 0xFFFF) ? -32768 : (int16_t)mv;
  pushLiveSegPacketRaw(SEG_LAP, now, lapMs, lapVbatMv, waitSagMv, brakeVbatMv);
}

// ============================================================
// Firmware-side lap logging.
// UI realtime lap is only display; BLE notification/log overflow must not be
// the source of truth. UI sends this config as a 16-byte trailer on rulesChar:
//   "LAP1", patternLen, pattern[8], passesPerLap, occurrenceN, occurrencePhase.
// The firmware appends confirmed lap records to stopped-log dump as SEG_LAP.
// ============================================================
struct LapConfig {
  bool enabled;
  uint8_t patternLen;
  uint8_t pattern[MAX_PATTERN_LEN];
  uint8_t passesPerLap;
  uint8_t occurrenceN;
  uint8_t occurrencePhase;
};
struct LapRecord {
  uint16_t endMs;
  uint16_t lapMs;
  uint8_t lapNo;
  uint8_t sectorCount;
  int16_t vbatMv;       // event-time Battery VBAT [mV], -32768 if invalid
  int16_t waitSagMv;    // max WAIT-load sag during this lap [mV], -32768 if invalid
  int16_t brakeVbatMv;  // minimum Battery VBAT during ACTION_BRAKE [mV], -32768 if invalid
};
static LapConfig lapConfig = {false, 0, {0}, 1, 1, 1};
static uint8_t lapMatchIdx = 0;
static uint16_t lapCandidateCount = 0;
static bool lapHaveBoundary = false;
static uint32_t lapLastBoundaryMs = 0;
static uint32_t lapAccumMs = 0;
static uint8_t lapSectorCount = 0;
static const uint8_t LAP_LOG_CAP = 64;
static LapRecord lapBuf[LAP_LOG_CAP];
static uint8_t lapCount = 0;
static uint16_t lapOverflowCount = 0;

// v3.47: max sag observed while ACTION_WAIT is active inside the current lap.
// This is only monitored/logged for analysis; it is not used for correction.
static int16_t waitSagMaxMvSinceLap = -32768;

// v3.49: minimum VBAT observed while ACTION_BRAKE is active inside the current lap.
// This is only monitored/logged for analysis; it is not used for correction.
static int16_t brakeVbatMinMvSinceLap = -32768;
static bool brakeVbatWindowActive = false;
static uint32_t brakeVbatWindowEndMs = 0;

// v3.48: dedicated WAIT measurement window.
// Opened exactly when ACTION_WAIT starts, then sampled from the control loop
// until the scheduled WAIT end. This avoids reporting WAIT Sag as invalid when
// the action index advances around a control-tick boundary.
static bool waitSagWindowActive = false;
static uint32_t waitSagWindowEndMs = 0;

static void resetLapRecords() {
  lapCount = 0;
  lapOverflowCount = 0;
  waitSagMaxMvSinceLap = -32768;
  waitSagWindowActive = false;
  waitSagWindowEndMs = 0;
  brakeVbatMinMvSinceLap = -32768;
  brakeVbatWindowActive = false;
  brakeVbatWindowEndMs = 0;
}

static void resetLapMatcher() {
  lapMatchIdx = 0;
  lapCandidateCount = 0;
  lapHaveBoundary = false;
  lapLastBoundaryMs = 0;
  lapAccumMs = 0;
  lapSectorCount = 0;
}

static bool lapSegmentIsAux(uint8_t seg) {
  return seg == SEG_WAVE || seg == SEG_LEFT_PEAK || seg == SEG_RIGHT_PEAK ||
         seg == SEG_STRAIGHT_HOLD || seg == SEG_LEFT_HOLD || seg == SEG_RIGHT_HOLD;
}

static void appendLapRecord(uint32_t nowMs, uint32_t lapMs) {
  if (lapMs == 0) return;
  const int16_t lapWaitSagMv = waitSagMaxMvSinceLap;
  const int16_t lapBrakeVbatMv = brakeVbatMinMvSinceLap;
  queueRealtimeLapNotify(nowMs, lapMs, lapWaitSagMv, lapBrakeVbatMv);
  waitSagMaxMvSinceLap = -32768;
  brakeVbatMinMvSinceLap = -32768;
  if (lapCount >= LAP_LOG_CAP) {
    if (lapOverflowCount < 0xFFFF) lapOverflowCount++;
    return;
  }
  uint32_t endMs = (runStartMs != 0) ? (uint32_t)(nowMs - runStartMs) : 0UL;
  if (endMs > 65535UL) endMs = 65535UL;
  if (lapMs > 65535UL) lapMs = 65535UL;
  LapRecord &r = lapBuf[lapCount];
  r.endMs = (uint16_t)endMs;
  r.lapMs = (uint16_t)lapMs;
  r.lapNo = (uint8_t)constrain((int)lapCount + 1, 1, 255);
  r.sectorCount = lapSectorCount;
  uint16_t mv = lastVbatMv;
  r.vbatMv = (mv == 0xFFFF) ? -32768 : (int16_t)mv;
  r.waitSagMv = lapWaitSagMv;
  r.brakeVbatMv = lapBrakeVbatMv;
  lapCount++;
}

static void acceptLapBoundary(uint32_t now) {
  if (!lapHaveBoundary) {
    lapHaveBoundary = true;
    lapLastBoundaryMs = now;
    lapAccumMs = 0;
    lapSectorCount = 0;
    return;
  }
  uint32_t sectorMs = now - lapLastBoundaryMs;
  // Reject debounce hits without moving the boundary reference. Updating
  // lapLastBoundaryMs before returning would silently shorten the measured lap.
  if (sectorMs < 80UL) return;
  lapLastBoundaryMs = now;
  lapAccumMs += sectorMs;
  lapSectorCount++;
  uint8_t passes = lapConfig.passesPerLap ? lapConfig.passesPerLap : 1;
  if (lapSectorCount >= passes) {
    if (lapAccumMs >= 800UL) appendLapRecord(now, lapAccumMs);
    lapAccumMs = 0;
    lapSectorCount = 0;
  }
}

static void processLapSegment(uint8_t seg, uint32_t now) {
  if (!lapConfig.enabled || lapConfig.patternLen == 0) return;
  const uint8_t expected = lapConfig.pattern[lapMatchIdx];
  const uint8_t prev = (lapMatchIdx > 0) ? lapConfig.pattern[lapMatchIdx - 1] : 0;

  if (seg == expected) {
    lapMatchIdx++;
  } else if (lapMatchIdx > 0 && seg == prev) {
    return;
  } else if (lapMatchIdx > 0 && lapSegmentIsAux(seg) && seg != expected) {
    return;
  } else if (seg == lapConfig.pattern[0]) {
    lapMatchIdx = 1;
  } else {
    lapMatchIdx = 0;
    return;
  }

  if (lapMatchIdx >= lapConfig.patternLen) {
    lapMatchIdx = 0;
    lapCandidateCount++;
    uint8_t n = lapConfig.occurrenceN ? lapConfig.occurrenceN : 1;
    uint8_t ph = lapConfig.occurrencePhase ? lapConfig.occurrencePhase : 1;
    if (ph > n) ph = n;
    if (n <= 1 || (((lapCandidateCount - 1) % n) + 1) == ph) {
      acceptLapBoundary(now);
    }
  }
}

// ============================================================
// Event log. Only section/action events are logged during run.
// No 200 Hz BLE transfer during run.
// ============================================================
struct EventLog {
  uint16_t tMs;       // elapsed from runStartMs, saturated
  uint8_t  seg;
  uint8_t  ruleIdx;   // 0xFF if none
  uint8_t  action;
  int16_t  gz10;
  // v3.11: curve / straight context. peak10 = absolute peak gz × 10 (signed
  // for symmetry with gz10, always >= 0 in practice). durMs = elapsed time
  // since the relevant segment started. Both 0 when not applicable
  // (e.g. SEG_WAVE has no peak/duration concept).
  int16_t  peak10;
  uint16_t durMs;
  // v3.22: action timing debug. These are meaningful for WAIT/BRAKE
  // action-start rows only; segment rows keep them zero/0xFF.
  uint8_t  actionIdx;
  uint16_t actionDurMs;      // final duration actually used after correction
  int16_t  voltageDeltaMs;   // stopped/unloaded VBAT correction relative to configured duration
  int16_t  sagDeltaMs;       // additional load-sag correction
  int16_t  loadSagMv;        // unloaded/rest VBAT - event-time VBAT [mV], -32768 if invalid
  int16_t  vbatMv;           // event-time Battery VBAT [mV], -32768 if invalid
  // v3.52: extra stopped-log telemetry for time-series reconstruction.
  int16_t  boardTemp10;      // board temperature ×10 [degC], -32768 if invalid
  int16_t  midMv;            // MID voltage [mV], -32768 if invalid
  int16_t  vbatMinusMidMv;   // VBAT - MID [mV], -32768 if invalid
};

// v3.22: EventLog includes correction-debug fields.
// v3.36: LOG_CAP 384->768.
// v3.51: ring buffer.
// v3.54: during run, send sparse 1Hz telemetry for header VBAT / board temp only.
// Graph time-series remains stopped-log based.
// v3.52: stopped-log voltage rows also carry board temp / MID / VBAT-MID for time-series graphs. When the log becomes full, keep the newest LOG_CAP rows
// and increment logOverflowCount. This is safer for post-run diagnosis than
// keeping only the beginning of a long run. Dump sequences are chronological
// from the oldest retained row to the newest retained row.
static const uint16_t LOG_CAP = 768;
static EventLog logBuf[LOG_CAP];
static uint16_t logStart = 0;   // index of oldest retained event/action row
static uint16_t logCount = 0;   // retained event/action row count, <= LOG_CAP
static uint16_t logOverflowCount = 0;

// v3.55: Dense voltage/time-series samples are kept in a separate compact
// ring buffer so they do not overwrite section/action events. They are still
// exported through the same stopped-log protocol as synthetic SEG_VOLTAGE rows.
struct VoltageLog {
  uint16_t tMs;
  int16_t  loadSagMv;
  int16_t  vbatMv;
  int16_t  boardTemp10;
  int16_t  midMv;
  int16_t  vbatMinusMidMv;
};
static const uint16_t VOLT_LOG_CAP = 1536;  // 40 Hz -> about 38 seconds retained
static VoltageLog voltBuf[VOLT_LOG_CAP];
static uint16_t voltStart = 0;
static uint16_t voltCount = 0;
static uint16_t voltOverflowCount = 0;
static bool dumpActive = false;
// v3.37 reliable dump is pull-based. The UI asks for a row sequence and the
// firmware sends exactly that row. These legacy cursors are kept only so older
// helper code can reset them; they are not advanced by the new protocol.
static uint16_t dumpIndex = 0;
static uint16_t dumpSeq = 0;
static uint8_t dumpLapIndex = 0;

// ============================================================
// BLE service and characteristics
// ============================================================
BLEService configService("12345678-1234-5678-1234-56789abcdef0");

BLECharacteristic commandChar(
  "12345678-1234-5678-1234-56789abcdef4",
  BLERead | BLEWrite,
  8
);

BLEByteCharacteristic statusChar(
  "12345678-1234-5678-1234-56789abcdef5",
  BLERead | BLENotify
);

BLECharacteristic currentSegChar(
  "12345678-1234-5678-1234-56789abcdef6",
  BLERead | BLENotify,
  13
);

BLEByteCharacteristic speedChar(
  "12345678-1234-5678-1234-56789abcdefa",
  BLERead | BLEWrite
);

// Reuses the old thresholds UUID as production params.
BLECharacteristic paramsChar(
  "12345678-1234-5678-1234-56789abcdf00",
  BLERead | BLEWrite,
  30
);

BLECharacteristic rulesChar(
  "12345678-1234-5678-1234-56789abcdefd",
  BLERead | BLEWrite,
  400
);

// Stopped log dump. Packet: [seqLo seqHi count events...]
// v3.11 event = [tLo tHi seg rule action gzLo gzHi peakLo peakHi durLo durHi].
// v3.23 event = v3.22 18-byte event + int16 loadSagMv at [18..19].
// terminator: seq=0xFFFF, count=0.
// v3.11: packet grew from 17B (2×7+3) to 25B (2×11+3). bulkChar max value
// length must be ≥ packet size or ArduinoBLE may silently truncate / fail
// to advertise. v3.23 uses 3 hdr + 1×20 bytes = 23 bytes; 32 gives headroom.
BLECharacteristic bulkChar(
  "12345678-1234-5678-1234-56789abcdefc",
  BLERead | BLENotify,
  32
);

// Low-rate board telemetry. Packet, 12 bytes LE:
//   [0..1]   int16  board temperature ×10 [degC], -32768 if invalid
//   [2..3]   uint16 current VBAT [mV], 0xFFFF if invalid
//   [4..5]   uint16 MID  [mV], 0xFFFF if invalid
//   [6..7]   int16  VBAT - MID [mV]
//   [8..9]   uint16 unloaded VBAT estimate [mV], 0xFFFF if invalid
//   [10..11] int16  unloaded VBAT - current VBAT [mV], -32768 if invalid
BLECharacteristic telemetryChar(
  "12345678-1234-5678-1234-56789abcdf01",
  BLERead | BLENotify,
  12
);

// Adaptive WAIT/BRAKE settings.
// Packet v1/v2 remain readable for compatibility. v3.21 writes packet version 3,
// 2 + N*23 bytes LE. Base voltage correction still uses low/high endpoints;
// sag correction is an independent signed coefficient.
//   [0] version=1/2/3, [1] rule count
//   v3 per rule:
//     [0] flags bit0=enabled, bit2=sag correction enabled
//     [1..2]   low voltage [mV]
//     [3..4]   high voltage [mV]
//     [5..6]   wait at low voltage [ms]
//     [7..8]   wait at high voltage [ms]
//     [9..10]  brake at low voltage [ms]
//     [11..12] brake at high voltage [ms]
//     [13]     recent running VBAT averaging window [ms]
//     [14]     sag mode: 0=off, 1=always, 2=only when current duty >= threshold
//     [15]     target mode: 0=first wait/brake pair, 1=all wait/brake, 2=selected
//     [16]     target action mask when mode=selected
//     [17]     sag duty threshold [% of 255]
//     [18..19] int16 wait sag coefficient [ms/V]
//     [20..21] int16 brake sag coefficient [ms/V]
//     [22]     reserved
BLECharacteristic adaptiveBrakeChar(
  "12345678-1234-5678-1234-56789abcdf02",
  BLERead | BLEWrite,
  256
);

// Firmware information for the Web app.
// Plain ASCII string, e.g. "v3.58". Optional for older Web apps.
BLECharacteristic firmwareInfoChar(
  "12345678-1234-5678-1234-56789abcdf03",
  BLERead,
  32
);

// User-visible BLE device name.
// Plain ASCII string, max 20 bytes. Written from the Web app or generated at flash time.
// The saved name is used from the next reboot/advertise cycle.
BLECharacteristic deviceNameChar(
  "12345678-1234-5678-1234-56789abcdf04",
  BLERead | BLEWrite,
  DEVICE_NAME_MAX_LEN + 1
);

// ============================================================
// Persistent BLE configuration storage
// ============================================================
static const uint32_t PERSIST_MAGIC = 0x3449414DUL;  // "MAI4" little-endian marker
static const uint16_t PERSIST_VERSION = 3;
static const uint16_t PERSIST_ADDR = 0;
static const uint16_t PARAMS_PACKET_MAX = 30;
static const uint16_t RULES_PACKET_MAX = 400;
static const uint16_t ADAPTIVE_PACKET_MAX = 2 + MAX_RULES * 23;

struct PersistentConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint16_t paramsLen;
  uint16_t rulesLen;
  uint16_t adaptiveLen;
  uint16_t crc;
  uint8_t params[PARAMS_PACKET_MAX];
  uint8_t rules[RULES_PACKET_MAX];
  uint8_t adaptive[ADAPTIVE_PACKET_MAX];
};

static uint8_t lastParamsPacket[PARAMS_PACKET_MAX] = {0};
static uint16_t lastParamsPacketLen = 0;
static uint8_t lastRulesPacket[RULES_PACKET_MAX] = {1, 0};
static uint16_t lastRulesPacketLen = 2;
static uint8_t lastAdaptivePacket[ADAPTIVE_PACKET_MAX] = {3, 0};
static uint16_t lastAdaptivePacketLen = 2;

static bool persistentLoadAttempted = false;
static bool persistentConfigValid = false;

// Device name is intentionally stored separately from the main running config so
// renaming the machine does not invalidate saved rules or timing parameters.
static const uint32_t DEVICE_NAME_MAGIC = 0x454D414EUL;  // "NAME" little-endian marker
static const uint16_t DEVICE_NAME_VERSION = 1;
static const uint16_t DEVICE_NAME_ADDR = PERSIST_ADDR + sizeof(PersistentConfig);

struct PersistentDeviceName {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  char name[DEVICE_NAME_MAX_LEN + 1];
  uint16_t crc;
};

// ============================================================
// Small utilities
// ============================================================
static inline bool reached(uint32_t now, uint32_t t) {
  return (int32_t)(now - t) >= 0;
}

static uint16_t readU16LE(const uint8_t *p) {
  return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}

static void writeU16LE(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)(v >> 8);
}

static void writeU32LE(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFF);
  p[1] = (uint8_t)((v >> 8) & 0xFF);
  p[2] = (uint8_t)((v >> 16) & 0xFF);
  p[3] = (uint8_t)((v >> 24) & 0xFF);
}

static int16_t readI16LE(const uint8_t *p) {
  return (int16_t)readU16LE(p);
}

static void writeI16LE(uint8_t *p, int16_t v) {
  writeU16LE(p, (uint16_t)v);
}

static uint16_t clampMsFromSigned(int32_t v) {
  if (v < 0) return 0;
  if (v > 60000) return 60000;
  return (uint16_t)v;
}

static int16_t clampI16FromInt32(int32_t v) {
  if (v > 32767L) return 32767;
  if (v < -32768L) return -32768;
  return (int16_t)v;
}

static void setAdaptiveBrakeDefaults(Rule &r) {
  r.adaptiveBrakeEnabled = false;
  r.adaptiveVLowMv = 2700;
  r.adaptiveVHighMv = 3000;
  r.adaptiveWaitLowMs = 230;
  r.adaptiveWaitHighMs = 140;
  r.adaptiveBrakeLowMs = 520;
  r.adaptiveBrakeHighMs = 760;
  r.adaptiveSampleMs = 150;
  r.adaptiveRunVbatMixPct = 0;
  r.adaptiveSagMode = ADAPT_SAG_OFF;
  r.adaptiveSagDutyMinPct = 80;
  r.adaptiveSagWaitMsPerV = 100;
  r.adaptiveSagBrakeMsPerV = -100;
  r.adaptiveTargetMode = ADAPT_TARGET_FIRST_PAIR;
  r.adaptiveTargetMask = 0;
}

static int16_t clampI16(float v) {
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static uint8_t clampU8(int v) {
  if (v < 0) return 0;
  if (v > 255) return 255;
  return (uint8_t)v;
}

static uint16_t clampU16FromFloat(float v) {
  if (isnan(v) || isinf(v) || v < 0.0f) return 0xFFFF;
  if (v > 65535.0f) return 65535;
  return (uint16_t)lroundf(v);
}

static int16_t clampI16FromFloat(float v) {
  if (isnan(v) || isinf(v)) return -32768;
  if (v > 32767.0f) return 32767;
  if (v < -32768.0f) return -32768;
  return (int16_t)lroundf(v);
}

static uint16_t readAdcAvg(uint8_t pin, uint8_t n = 8) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < n; i++) {
    sum += (uint16_t)analogRead(pin);
  }
  return (uint16_t)(sum / n);
}

static float adcRawToPinV(uint16_t raw) {
  return ((float)raw * ADC_REF_V) / ADC_COUNTS;
}

static float readScaledVoltage(uint8_t pin, float scale) {
  uint16_t raw = readAdcAvg(pin);
  return adcRawToPinV(raw) * scale;
}

static uint16_t readScaledVoltageMvFast(uint8_t pin, float scale) {
  uint16_t raw = readAdcAvg(pin, 2);
  return clampU16FromFloat(adcRawToPinV(raw) * scale * 1000.0f);
}

static uint16_t averageRecentVbatMv(uint32_t now, uint16_t sampleMs);

static void updateUnloadedVbatEstimate(uint32_t now, uint16_t currentVbatMv) {
  if (currentVbatMv == 0xFFFF) return;

  // PWM duty 0% on this board means short brake. When stopped, the loop also
  // keeps duty at 0%. In both states the battery is not supplying motor drive
  // current, so VBAT is a practical estimate of the pre-race adjusted voltage.
  const bool brakeOrStopWindow = (currentMotorDuty == 0);
  if (!brakeOrStopWindow) {
    unloadedBrakeWindowActive = false;
    return;
  }

  if (!unloadedBrakeWindowActive) {
    unloadedBrakeWindowActive = true;
    unloadedBrakeStartMs = now;
    return;
  }

  if ((uint32_t)(now - unloadedBrakeStartMs) < UNLOADED_VBAT_SETTLE_MS) {
    return;
  }

  if (unloadedVbatMv == 0xFFFF) {
    unloadedVbatMv = currentVbatMv;
  } else {
    // Light smoothing: fast enough to follow battery changes across brakes,
    // slow enough to reject switching/contact spikes.
    unloadedVbatMv = (uint16_t)(((uint32_t)unloadedVbatMv * 85u + (uint32_t)currentVbatMv * 15u + 50u) / 100u);
  }
  unloadedVbatLastUpdateMs = now;
}

static void primeUnloadedVbatEstimate(uint32_t now) {
  uint16_t mv = readScaledVoltageMvFast(VBAT_SENSE_PIN, VBAT_DIV_SCALE);
  if (mv == 0xFFFF) return;
  unloadedVbatMv = mv;
  unloadedVbatLastUpdateMs = now;
  unloadedBrakeWindowActive = true;
  unloadedBrakeStartMs = now;
}

static uint16_t adaptiveBrakeInputVbatMv(uint32_t now, uint16_t fallbackSampleMs) {
  if (unloadedVbatMv != 0xFFFF) return unloadedVbatMv;
  // Fallback for the very first moment after boot if no stop/brake sample
  // has been captured yet. Normal operation should use unloadedVbatMv.
  return averageRecentVbatMv(now, fallbackSampleMs);
}

static uint16_t adaptiveEffectiveVbatMv(uint32_t now, uint16_t sampleMs, uint8_t runMixPct) {
  uint16_t unloadedMv = adaptiveBrakeInputVbatMv(now, sampleMs);
  if (unloadedMv == 0xFFFF) return 0xFFFF;
  if (runMixPct == 0) return unloadedMv;

  uint16_t runningMv = averageRecentVbatMv(now, sampleMs);
  if (runningMv == 0xFFFF) return unloadedMv;

  // mix = 0   -> stopped/unloaded VBAT only (legacy behavior)
  // mix = 100 -> recent running/loaded VBAT
  // mix > 100 -> exaggerate sag if the running VBAT is a strong speed proxy
  int32_t eff = (int32_t)unloadedMv + (((int32_t)runningMv - (int32_t)unloadedMv) * (int32_t)runMixPct) / 100;
  if (eff < 0) eff = 0;
  if (eff > 65535) eff = 65535;
  return (uint16_t)eff;
}

static void sampleControlVbat(uint32_t now) {
  if ((uint32_t)(now - lastVbatSampleMs) < VBAT_SAMPLE_MS) return;
  lastVbatSampleMs = now;
  uint16_t mv = readScaledVoltageMvFast(VBAT_SENSE_PIN, VBAT_DIV_SCALE);
  lastVbatMv = mv;
  vbatRingMs[vbatRingPos] = now;
  vbatRingMv[vbatRingPos] = mv;
  vbatRingPos = (uint8_t)((vbatRingPos + 1) % VBAT_RING_N);
  if (vbatRingCount < VBAT_RING_N) vbatRingCount++;
  updateUnloadedVbatEstimate(now, mv);
}

static uint16_t averageRecentVbatMv(uint32_t now, uint16_t sampleMs) {
  if (vbatRingCount == 0) return lastVbatMv;
  if (sampleMs < 20) sampleMs = 20;
  if (sampleMs > 600) sampleMs = 600;
  uint32_t sum = 0;
  uint16_t count = 0;
  for (uint8_t i = 0; i < vbatRingCount; i++) {
    uint8_t idx = (uint8_t)((VBAT_RING_N + vbatRingPos - 1 - i) % VBAT_RING_N);
    uint32_t t = vbatRingMs[idx];
    if ((uint32_t)(now - t) > (uint32_t)sampleMs) break;
    uint16_t mv = vbatRingMv[idx];
    if (mv == 0xFFFF) continue;
    sum += mv;
    count++;
  }
  if (count == 0) return lastVbatMv;
  return (uint16_t)(sum / count);
}


static int16_t currentLoadSagMv(uint32_t now) {
  uint16_t baseMv = unloadedVbatMv;
  if (baseMv == 0xFFFF) baseMv = adaptiveBrakeInputVbatMv(now, 100);
  if (baseMv == 0xFFFF) return -32768;

  // Use a short window for WAIT-sag telemetry. A long 100ms average can hide
  // a short WAIT action or mix pre-WAIT samples into the value.
  uint16_t runningMv = averageRecentVbatMv(now, 40);
  if (runningMv == 0xFFFF) runningMv = lastVbatMv;
  if (runningMv == 0xFFFF) return -32768;

  int32_t sag = (int32_t)baseMv - (int32_t)runningMv;
  if (sag < 0) sag = 0;
  if (sag > 32767) sag = 32767;
  return (int16_t)sag;
}

static void sampleWaitSagPoint(uint32_t now) {
  int16_t sag = currentLoadSagMv(now);
  if (sag == -32768) return;
  if (sag < 0) sag = 0;
  if (waitSagMaxMvSinceLap == -32768 || sag > waitSagMaxMvSinceLap) {
    waitSagMaxMvSinceLap = sag;
  }
}

static void beginWaitSagWindow(uint32_t now, uint16_t durationMs) {
  if (!isRunning) return;
  if (durationMs == 0) durationMs = 1;
  waitSagWindowActive = true;
  waitSagWindowEndMs = now + (uint32_t)durationMs;
  sampleWaitSagPoint(now);
}

static void endWaitSagWindow(uint32_t now) {
  if (!waitSagWindowActive) return;
  sampleWaitSagPoint(now);
  waitSagWindowActive = false;
  waitSagWindowEndMs = 0;
}

static void trackWaitSagDuringActiveWait(uint32_t now) {
  if (!isRunning) {
    waitSagWindowActive = false;
    return;
  }

  bool inWaitAction = false;
  if (actionActive && activeActionIdx < activeActionCount && activeActions[activeActionIdx].type == ACTION_WAIT) {
    inWaitAction = true;
  }

  bool inWaitWindow = waitSagWindowActive && !reached(now, waitSagWindowEndMs);
  if (!inWaitAction && !inWaitWindow) {
    if (waitSagWindowActive && reached(now, waitSagWindowEndMs)) endWaitSagWindow(now);
    return;
  }

  sampleWaitSagPoint(now);
}

static int16_t recentBrakeVbatMv(uint32_t now) {
  uint16_t mv = averageRecentVbatMv(now, 40);
  if (mv == 0xFFFF) mv = lastVbatMv;
  if (mv == 0xFFFF) return -32768;
  return (int16_t)mv;
}

static void sampleBrakeVbatPoint(uint32_t now) {
  int16_t mv = recentBrakeVbatMv(now);
  if (mv == -32768) return;
  if (brakeVbatMinMvSinceLap == -32768 || mv < brakeVbatMinMvSinceLap) {
    brakeVbatMinMvSinceLap = mv;
  }
}

static void beginBrakeVbatWindow(uint32_t now, uint16_t durationMs) {
  if (!isRunning) return;
  if (durationMs == 0) durationMs = 1;
  brakeVbatWindowActive = true;
  brakeVbatWindowEndMs = now + (uint32_t)durationMs;
  sampleBrakeVbatPoint(now);
}

static void endBrakeVbatWindow(uint32_t now) {
  if (!brakeVbatWindowActive) return;
  sampleBrakeVbatPoint(now);
  brakeVbatWindowActive = false;
  brakeVbatWindowEndMs = 0;
}

static void trackBrakeVbatDuringActiveBrake(uint32_t now) {
  if (!isRunning) {
    brakeVbatWindowActive = false;
    return;
  }

  bool inBrakeAction = false;
  if (actionActive && activeActionIdx < activeActionCount && activeActions[activeActionIdx].type == ACTION_BRAKE) {
    inBrakeAction = true;
  }

  bool inBrakeWindow = brakeVbatWindowActive && !reached(now, brakeVbatWindowEndMs);
  if (!inBrakeAction && !inBrakeWindow) {
    if (brakeVbatWindowActive && reached(now, brakeVbatWindowEndMs)) endBrakeVbatWindow(now);
    return;
  }

  sampleBrakeVbatPoint(now);
}

static uint16_t interpByVoltageMv(uint16_t vMv, uint16_t vLow, uint16_t vHigh, uint16_t yLow, uint16_t yHigh) {
  if (vHigh == vLow) return yLow;
  if (vLow > vHigh) {
    uint16_t tv = vLow; vLow = vHigh; vHigh = tv;
    uint16_t ty = yLow; yLow = yHigh; yHigh = ty;
  }
  if (vMv <= vLow) return yLow;
  if (vMv >= vHigh) return yHigh;
  float t = (float)((int32_t)vMv - (int32_t)vLow) / (float)((int32_t)vHigh - (int32_t)vLow);
  float y = (float)yLow + t * ((float)yHigh - (float)yLow);
  return (uint16_t)constrain((int)lroundf(y), 0, 60000);
}

static float readBoardTempC() {
  uint16_t raw = readAdcAvg(NTC_TEMP_PIN);
  float v = adcRawToPinV(raw);
  if (v <= 0.001f || v >= (ADC_REF_V - 0.001f)) return NAN;

  float rNtc;
  if (NTC_TO_GND) {
    // 3V3 -- pullup -- ADC -- NTC -- GND
    rNtc = NTC_PULLUP_OHM * v / (ADC_REF_V - v);
  } else {
    // 3V3 -- NTC -- ADC -- pulldown -- GND
    rNtc = NTC_PULLUP_OHM * (ADC_REF_V - v) / v;
  }
  if (rNtc <= 0.0f) return NAN;

  float invT = (1.0f / (NTC_NOMINAL_C + 273.15f)) + (logf(rNtc / NTC_NOMINAL_OHM) / NTC_BETA);
  return (1.0f / invT) - 273.15f;
}

static void writeTelemetryToChar(bool force = false, uint32_t minIntervalMs = TELEMETRY_MS) {
  if (!bleReady) return;
  uint32_t now = millis();
  if (!force && (uint32_t)(now - lastTelemetryMs) < minIntervalMs) return;
  lastTelemetryMs = now;

  // v3.25: keep telemetry frequent without extra slow averaging.
  // VBAT is already sampled by the control loop every 10ms; publish the latest sample.
  if (cachedBoardTemp10 == -32768 || (uint32_t)(now - lastBoardTempSampleMs) >= BOARD_TEMP_SAMPLE_MS) {
    lastBoardTempSampleMs = now;
    float tempC = readBoardTempC();
    cachedBoardTemp10 = clampI16FromFloat(tempC * 10.0f);
  }

  uint16_t vbatMv = lastVbatMv;
  if (vbatMv == 0xFFFF) vbatMv = readScaledVoltageMvFast(VBAT_SENSE_PIN, VBAT_DIV_SCALE);
  uint16_t midMv = readScaledVoltageMvFast(MID_SENSE_PIN, MID_DIV_SCALE);
  int16_t diffMv = -32768;
  if (vbatMv != 0xFFFF && midMv != 0xFFFF) {
    diffMv = clampI16FromFloat((float)((int32_t)vbatMv - (int32_t)midMv));
  }
  int16_t sagMv = -32768;
  if (unloadedVbatMv != 0xFFFF && vbatMv != 0xFFFF) {
    sagMv = clampI16FromFloat((float)((int32_t)unloadedVbatMv - (int32_t)vbatMv));
  }

  uint8_t pkt[12] = {0};
  writeU16LE(&pkt[0], (uint16_t)cachedBoardTemp10);
  writeU16LE(&pkt[2], vbatMv);
  writeU16LE(&pkt[4], midMv);
  writeU16LE(&pkt[6], (uint16_t)diffMv);
  writeU16LE(&pkt[8], unloadedVbatMv);
  writeU16LE(&pkt[10], (uint16_t)sagMv);
  telemetryChar.writeValue(pkt, sizeof(pkt));
}

static void writeAdaptiveBrakeToChar() {
  if (!bleReady) return;
  const uint8_t REC_BYTES = 23;
  uint8_t count = rulesCount;
  if (count > MAX_RULES) count = MAX_RULES;
  uint8_t pkt[2 + MAX_RULES * REC_BYTES] = {0};
  pkt[0] = 3;
  pkt[1] = count;
  for (uint8_t r = 0; r < count; r++) {
    Rule &rule = rules[r];
    uint8_t *q = &pkt[2 + r * REC_BYTES];
    uint8_t sagMode = constrain(rule.adaptiveSagMode, (uint8_t)0, (uint8_t)2);
    uint8_t flags = rule.adaptiveBrakeEnabled ? 1 : 0;
    if (sagMode != ADAPT_SAG_OFF) flags |= 0x04;
    q[0] = flags;
    writeU16LE(&q[1], rule.adaptiveVLowMv);
    writeU16LE(&q[3], rule.adaptiveVHighMv);
    writeU16LE(&q[5], rule.adaptiveWaitLowMs);
    writeU16LE(&q[7], rule.adaptiveWaitHighMs);
    writeU16LE(&q[9], rule.adaptiveBrakeLowMs);
    writeU16LE(&q[11], rule.adaptiveBrakeHighMs);
    q[13] = (uint8_t)constrain(rule.adaptiveSampleMs, (uint16_t)20, (uint16_t)250);
    q[14] = sagMode;
    q[15] = constrain(rule.adaptiveTargetMode, (uint8_t)0, (uint8_t)2);
    q[16] = rule.adaptiveTargetMask & ((1u << MAX_ACTIONS) - 1u);
    q[17] = (uint8_t)constrain(rule.adaptiveSagDutyMinPct, (uint8_t)0, (uint8_t)100);
    writeI16LE(&q[18], (int16_t)constrain(rule.adaptiveSagWaitMsPerV, (int16_t)-5000, (int16_t)5000));
    writeI16LE(&q[20], (int16_t)constrain(rule.adaptiveSagBrakeMsPerV, (int16_t)-5000, (int16_t)5000));
    q[22] = 0;
  }
  adaptiveBrakeChar.writeValue(pkt, 2 + count * REC_BYTES);
}


static uint8_t dutyFromStrength(uint8_t s) {
  if (s == 0) return 0;
  if (s == 1) return 64;
  if (s == 2) return 128;
  if (s == 3) return 192;
  if (s == 4) return 255;
  return s;
}

static void motorDuty(uint8_t d) {
  currentMotorDuty = d;
  analogWrite(PWM_PIN, d);
}

static void motorBrake() {
  motorDuty(0);
}

static uint8_t startupDutyFor(uint32_t now) {
  const uint8_t target = NORMAL_DUTY;
  uint8_t start = START_DUTY;
  if (start > target) start = target;
  if (target == 0) return 0;
  if (SOFT_START_MS == 0 || start >= target) return target;

  const uint32_t elapsed = (uint32_t)(now - runStartMs);
  if (elapsed >= SOFT_START_MS) return target;

  const uint16_t span = (uint16_t)target - (uint16_t)start;
  const uint16_t add = (uint16_t)((span * elapsed + (SOFT_START_MS / 2)) / SOFT_START_MS);
  return (uint8_t)((uint16_t)start + add);
}

static void ledSet(bool on) {
  digitalWrite(LED_BUILTIN, on ? LOW : HIGH);  // many XIAO LEDs are active-low
}

static void setStatus(uint8_t s) {
  if (statusValue == s) return;
  statusValue = s;
  if (bleReady) statusChar.writeValue(statusValue);
}

static void writeParamsToChar() {
  uint8_t p[30] = {0};
  writeU16LE(&p[0],  (uint16_t)lroundf(CORNER_TH));
  writeU16LE(&p[2],  (uint16_t)lroundf(STRAIGHT_TH));
  writeU16LE(&p[4],  (uint16_t)lroundf(WAVE_POS_TH));
  writeU16LE(&p[6],  (uint16_t)lroundf(WAVE_NEG_TH));
  writeU16LE(&p[8],  (uint16_t)WAVE_WINDOW_MS);
  writeU16LE(&p[10], (uint16_t)WAVE_LATCH_MS);
  writeU16LE(&p[12], (uint16_t)WAVE_COOLDOWN_MS);
  writeU16LE(&p[14], (uint16_t)EXIT_DEBOUNCE_MS);
  p[16] = NORMAL_DUTY;
  p[17] = RECOVER_DUTY;
  p[18] = START_DUTY;
  p[19] = 0;  // live/debug segment notify disabled in production-control-safe mode
  writeU16LE(&p[20], (uint16_t)CURVE_PEAK_AFTER_MS);
  // v3.10: p[22..23] は相対判定 % に変更。バイト位置は維持。
  writeU16LE(&p[22], (uint16_t)CURVE_PEAK_DROP_PCT);
  writeU16LE(&p[24], (uint16_t)STRAIGHT_HOLD_MS);
  writeU16LE(&p[26], (uint16_t)SEGMENT_COOLDOWN_MS);
  writeU16LE(&p[28], (uint16_t)CURVE_LONG_MS);
  paramsChar.writeValue(p, sizeof(p));
  speedChar.writeValue(NORMAL_DUTY);
}

static void logReset() {
  logStart = 0;
  logCount = 0;
  logOverflowCount = 0;
  voltStart = 0;
  voltCount = 0;
  voltOverflowCount = 0;
  dumpActive = false;
  dumpIndex = 0;
  dumpSeq = 0;
  dumpLapIndex = 0;
  resetLapRecords();
  resetLapMatcher();
}

// v3.11: rich log writer with curve-context fields. Use this for segments
// that have a meaningful peak gz / duration (curve enter / PEAK / HOLD,
// straight HOLD). Plain logEvent() stays as a thin wrapper that fills 0
// for non-applicable contexts.
static void logEventEx(uint8_t seg, uint8_t ruleIdx, uint8_t action, float gz,
                       float peakDps, uint32_t durMs,
                       uint8_t actionIdx = 0xFF,
                       uint16_t actionDurMs = 0,
                       int16_t voltageDeltaMs = 0,
                       int16_t sagDeltaMs = 0) {
  uint16_t idx;
  if (logCount < LOG_CAP) {
    idx = (uint16_t)((logStart + logCount) % LOG_CAP);
    logCount++;
  } else {
    // Ring-buffer behavior: overwrite the oldest retained row, then advance
    // the chronological start pointer. This keeps the end of the run, where
    // failures and auto-tune decisions usually matter most.
    idx = logStart;
    logStart = (uint16_t)((logStart + 1) % LOG_CAP);
    if (logOverflowCount < 0xFFFF) logOverflowCount++;
  }
  EventLog &row = logBuf[idx];
  uint32_t nowMs = millis();
  uint32_t dt = nowMs - runStartMs;
  if (dt > 65535UL) dt = 65535UL;
  if (durMs > 65535UL) durMs = 65535UL;
  row.tMs = (uint16_t)dt;
  row.seg = seg;
  row.ruleIdx = ruleIdx;
  row.action = action;
  row.gz10 = clampI16(gz * 10.0f);
  row.peak10 = clampI16(peakDps * 10.0f);
  row.durMs = (uint16_t)durMs;
  row.actionIdx = actionIdx;
  row.actionDurMs = actionDurMs;
  row.voltageDeltaMs = voltageDeltaMs;
  row.sagDeltaMs = sagDeltaMs;
  row.loadSagMv = currentLoadSagMv(nowMs);
  uint16_t eventVbatMv = lastVbatMv;
  if (eventVbatMv == 0xFFFF) eventVbatMv = readScaledVoltageMvFast(VBAT_SENSE_PIN, VBAT_DIV_SCALE);
  row.vbatMv = (eventVbatMv == 0xFFFF) ? -32768 : (int16_t)eventVbatMv;

  // v3.52: keep the diagnostic voltage timeline self-contained.
  // These fields are primarily for SEG_VOLTAGE rows, but filling them for
  // other retained rows also helps sparse fallback graphs. No BLE notify is
  // called here; values are only copied into the RAM ring buffer.
  if (cachedBoardTemp10 == -32768 || (uint32_t)(nowMs - lastBoardTempSampleMs) >= BOARD_TEMP_SAMPLE_MS) {
    lastBoardTempSampleMs = nowMs;
    float tempC = readBoardTempC();
    cachedBoardTemp10 = clampI16FromFloat(tempC * 10.0f);
  }
  row.boardTemp10 = cachedBoardTemp10;
  uint16_t eventMidMv = readScaledVoltageMvFast(MID_SENSE_PIN, MID_DIV_SCALE);
  row.midMv = (eventMidMv == 0xFFFF) ? -32768 : (int16_t)eventMidMv;
  if (eventVbatMv != 0xFFFF && eventMidMv != 0xFFFF) {
    row.vbatMinusMidMv = clampI16FromFloat((float)((int32_t)eventVbatMv - (int32_t)eventMidMv));
  } else {
    row.vbatMinusMidMv = -32768;
  }
}

static void logEvent(uint8_t seg, uint8_t ruleIdx, uint8_t action, float gz) {
  logEventEx(seg, ruleIdx, action, gz, 0.0f, 0UL);
}

static void appendVoltageLogSample(uint32_t now) {
  uint16_t idx;
  if (voltCount < VOLT_LOG_CAP) {
    idx = (uint16_t)((voltStart + voltCount) % VOLT_LOG_CAP);
    voltCount++;
  } else {
    idx = voltStart;
    voltStart = (uint16_t)((voltStart + 1) % VOLT_LOG_CAP);
    if (voltOverflowCount < 0xFFFF) voltOverflowCount++;
  }

  VoltageLog &row = voltBuf[idx];
  uint32_t dt = now - runStartMs;
  if (dt > 65535UL) dt = 65535UL;
  row.tMs = (uint16_t)dt;

  int16_t sagMv = currentLoadSagMv(now);
  row.loadSagMv = sagMv;

  uint16_t vbatMv = lastVbatMv;
  if (vbatMv == 0xFFFF) vbatMv = readScaledVoltageMvFast(VBAT_SENSE_PIN, VBAT_DIV_SCALE);
  row.vbatMv = (vbatMv == 0xFFFF) ? -32768 : (int16_t)vbatMv;

  if (cachedBoardTemp10 == -32768 || (uint32_t)(now - lastBoardTempSampleMs) >= BOARD_TEMP_SAMPLE_MS) {
    lastBoardTempSampleMs = now;
    float tempC = readBoardTempC();
    cachedBoardTemp10 = clampI16FromFloat(tempC * 10.0f);
  }
  row.boardTemp10 = cachedBoardTemp10;

  uint16_t midMv = readScaledVoltageMvFast(MID_SENSE_PIN, MID_DIV_SCALE);
  row.midMv = (midMv == 0xFFFF) ? -32768 : (int16_t)midMv;
  if (vbatMv != 0xFFFF && midMv != 0xFFFF) {
    row.vbatMinusMidMv = clampI16FromFloat((float)((int32_t)vbatMv - (int32_t)midMv));
  } else {
    row.vbatMinusMidMv = -32768;
  }
}

static void logVoltageSample(uint32_t now) {
  if (!isRunning) return;
  if ((uint32_t)(now - lastVoltageLogMs) < RUNNING_VOLTAGE_LOG_MS) return;
  lastVoltageLogMs = now;
  // Timestamped voltage monitor row for stopped-log analysis.
  // This is memory-only during the run; it never calls BLE notify and it does
  // not consume the section/action event log ring buffer.
  appendVoltageLogSample(now);
}

static void logActionEvent(uint8_t seg, uint8_t ruleIdx, uint8_t action, float gz,
                           uint8_t actionIdx, uint16_t actionDurMs,
                           int16_t voltageDeltaMs, int16_t sagDeltaMs) {
  logEventEx(seg, ruleIdx, action, gz, 0.0f, 0UL,
             actionIdx, actionDurMs, voltageDeltaMs, sagDeltaMs);
}

static uint16_t dumpTotalRows() {
  uint32_t total = (uint32_t)logCount + (uint32_t)voltCount + (uint32_t)lapCount;
  return (total > 0xFFFFUL) ? 0xFFFF : (uint16_t)total;
}

static uint16_t dumpTotalOverflow() {
  uint32_t total = (uint32_t)logOverflowCount + (uint32_t)voltOverflowCount + (uint32_t)lapOverflowCount;
  return (total > 0xFFFFUL) ? 0xFFFF : (uint16_t)total;
}

static void dumpAbort() {
  dumpActive = false;
  dumpIndex = 0;
  dumpSeq = 0;
  dumpLapIndex = 0;
}

static void writeEventLogToPacket(uint8_t *pkt, uint8_t off, const EventLog &e) {
  writeU16LE(&pkt[off + 0], e.tMs);
  pkt[off + 2] = e.seg;
  pkt[off + 3] = e.ruleIdx;
  pkt[off + 4] = e.action;
  writeI16LE(&pkt[off + 5], e.gz10);
  writeI16LE(&pkt[off + 7], e.peak10);
  writeU16LE(&pkt[off + 9], e.durMs);
  pkt[off + 11] = e.actionIdx;
  writeU16LE(&pkt[off + 12], e.actionDurMs);
  writeI16LE(&pkt[off + 14], e.voltageDeltaMs);
  writeI16LE(&pkt[off + 16], e.sagDeltaMs);
  writeI16LE(&pkt[off + 18], e.loadSagMv);
  writeI16LE(&pkt[off + 20], e.vbatMv);
  writeI16LE(&pkt[off + 22], e.boardTemp10);
  writeI16LE(&pkt[off + 24], e.midMv);
  writeI16LE(&pkt[off + 26], e.vbatMinusMidMv);
}

static EventLog makeLapEventLog(const LapRecord &lr) {
  EventLog e;
  e.tMs = lr.endMs;
  e.seg = SEG_LAP;
  e.ruleIdx = 0xFF;
  e.action = ACTION_NONE;
  e.gz10 = 0;
  e.peak10 = (int16_t)lr.lapNo * 10;  // Web UI reads peak/10 as lap number.
  e.durMs = lr.lapMs;                 // Web UI uses durMs as lap duration.
  e.actionIdx = 0xFF;
  e.actionDurMs = 0;
  e.voltageDeltaMs = 0;
  // For SEG_LAP only, carry minimum Brake VBAT in sagDeltaMs so old packet length stays unchanged.
  e.sagDeltaMs = lr.brakeVbatMv;
  e.loadSagMv = lr.waitSagMv;
  e.vbatMv = lr.vbatMv;
  e.boardTemp10 = cachedBoardTemp10;
  uint16_t eventMidMv = readScaledVoltageMvFast(MID_SENSE_PIN, MID_DIV_SCALE);
  e.midMv = (eventMidMv == 0xFFFF) ? -32768 : (int16_t)eventMidMv;
  if (lr.vbatMv != -32768 && eventMidMv != 0xFFFF) {
    e.vbatMinusMidMv = clampI16FromFloat((float)((int32_t)lr.vbatMv - (int32_t)eventMidMv));
  } else {
    e.vbatMinusMidMv = -32768;
  }
  return e;
}


static bool getDumpEventBySeq(uint16_t seq, EventLog &out) {
  if (seq < logCount) {
    uint16_t idx = (uint16_t)((logStart + seq) % LOG_CAP);
    out = logBuf[idx];
    return true;
  }
  seq = (uint16_t)(seq - logCount);
  if (seq < voltCount) {
    uint16_t idx = (uint16_t)((voltStart + seq) % VOLT_LOG_CAP);
    const VoltageLog &vr = voltBuf[idx];
    out.tMs = vr.tMs;
    out.seg = SEG_VOLTAGE;
    out.ruleIdx = 0xFF;
    out.action = ACTION_NONE;
    out.gz10 = 0;
    out.peak10 = 0;
    out.durMs = 0;
    out.actionIdx = 0xFF;
    out.actionDurMs = 0;
    out.voltageDeltaMs = 0;
    out.sagDeltaMs = 0;
    out.loadSagMv = vr.loadSagMv;
    out.vbatMv = vr.vbatMv;
    out.boardTemp10 = vr.boardTemp10;
    out.midMv = vr.midMv;
    out.vbatMinusMidMv = vr.vbatMinusMidMv;
    return true;
  }
  uint16_t lapSeq = (uint16_t)(seq - voltCount);
  if (lapSeq < lapCount) {
    out = makeLapEventLog(lapBuf[lapSeq]);
    return true;
  }
  return false;
}

static void sendDumpMeta() {
  if (!bleReady || isRunning) return;
  uint8_t pkt[32] = {0};
  writeU16LE(&pkt[0], 0xFFFE);  // META packet. Not a data sequence.
  pkt[2] = 0;
  writeU16LE(&pkt[3], dumpTotalRows());
  writeU16LE(&pkt[5], dumpTotalOverflow());
  pkt[7] = 2;     // reliable dump protocol version
  pkt[8] = 28;    // EventLog serialized bytes per row
  writeU16LE(&pkt[9], logCount);      // section/action diagnostic rows
  writeU16LE(&pkt[11], lapCount);      // firmware-confirmed lap rows
  writeU16LE(&pkt[13], LOG_CAP);
  writeU16LE(&pkt[15], LAP_LOG_CAP);
  pkt[17] = 1;    // rows per data packet
  writeU16LE(&pkt[18], voltCount);     // v3.55: dedicated voltage timeline rows
  writeU16LE(&pkt[20], VOLT_LOG_CAP);
  bulkChar.writeValue(pkt, 22);
}

static void sendDumpTerminator() {
  if (!bleReady) return;
  uint8_t pkt[32] = {0};
  writeU16LE(&pkt[0], 0xFFFF);
  pkt[2] = 0;
  writeU16LE(&pkt[3], dumpTotalRows());
  writeU16LE(&pkt[5], dumpTotalOverflow());
  bulkChar.writeValue(pkt, 7);
  dumpAbort();
}

static void sendDumpSeq(uint16_t seq) {
  if (!dumpActive || !bleReady || isRunning) return;
  const uint16_t total = dumpTotalRows();
  if (seq >= total) {
    sendDumpTerminator();
    return;
  }
  EventLog e;
  if (!getDumpEventBySeq(seq, e)) {
    sendDumpTerminator();
    return;
  }
  uint8_t pkt[32] = {0};
  writeU16LE(&pkt[0], seq);
  pkt[2] = 1;
  writeEventLogToPacket(pkt, 3, e);
  bulkChar.writeValue(pkt, 31);  // 3-byte header + 28-byte EventLog row
}

static void dumpStart() {
  if (!bleReady || isRunning) return;
  dumpActive = true;
  dumpIndex = 0;
  dumpSeq = 0;
  dumpLapIndex = 0;
  sendDumpMeta();
}

// v3.37: legacy timed streaming is intentionally disabled. The UI now pulls
// rows one by one using CMD_LOG_GET so a dropped BLE notification can be
// recovered by asking for the same sequence again.
static void dumpStep() {
  return;
}

// ============================================================
// IMU
// ============================================================
static float calibrateGzBias() {
  float sum = 0.0f;
  for (uint16_t i = 0; i < GZ_BIAS_SAMPLES; i++) {
    sum += myIMU.readFloatGyroZ();
    delay(2);
  }
  return sum / (float)GZ_BIAS_SAMPLES;
}

static float updateGzFilter(float gz) {
  gzBuf[gzIdx] = gz;
  gzIdx = (uint8_t)((gzIdx + 1) % GZ_FILT_N);
  float sum = 0.0f;
  for (uint8_t i = 0; i < GZ_FILT_N; i++) sum += gzBuf[i];
  return sum / (float)GZ_FILT_N;
}

// ============================================================
// Rule/action execution
// ============================================================
static void startActionsFromRule(uint8_t ruleIdx, float gz, uint32_t now);
static void emitSegmentLog(uint8_t seg, uint8_t ruleIdx, float gz, uint32_t now, float peakDps = 0.0f, uint32_t durMs = 0UL);  // v3.9 used by tickRuleWildcards; v3.11 added peak/dur
static void completeRulePattern(uint8_t ruleIdx, uint8_t seg, float gz, uint32_t now);
static void tickRuleConfirmations(float gz, uint32_t now);

static uint8_t priorityForRule(uint8_t ruleIdx) {
  if (ruleIdx >= rulesCount) return 0;
  uint8_t p = rules[ruleIdx].priority;
  if (p == 0) {
    // Legacy fallback: earlier rules are slightly higher priority.
    return (uint8_t)(100 - ruleIdx);
  }
  return p;
}

static bool segmentMatches(uint8_t expected, uint8_t actual) {
  if (expected == SEG_NONE) return false;
  // v3.9: wildcard is a time-only pattern element, never matched directly
  // against real segments. The matcher / wildcard-tick handles it explicitly.
  if (expected == SEG_WILDCARD) return false;
  return expected == actual;
}

static bool ruleContainsSegment(const Rule &r, uint8_t seg) {
  for (uint8_t i = 0; i < r.patternLen; i++) {
    if (r.pattern[i] == seg) return true;
  }
  return false;
}

static bool isAuxSegment(uint8_t seg) {
  return seg == SEG_WAVE || seg == SEG_LEFT_PEAK || seg == SEG_RIGHT_PEAK || seg == SEG_STRAIGHT_HOLD || seg == SEG_LEFT_HOLD || seg == SEG_RIGHT_HOLD;
}

// ---- v37 per-rule override helpers ----
//
// v38: Per-rule emission timings are tracked independently per rule.
// Global firing uses the configured global parameters and reaches rules whose
// override is 0xFFFF (inherit). Rules with explicit overrides receive their
// own per-rule firings calculated from cornerStartMs / cornerPeakMs /
// straightHoldStartMs at the per-rule override interval.
static bool segWantsPeakAfter(uint8_t seg) {
  return seg == SEG_LEFT_PEAK || seg == SEG_RIGHT_PEAK;
}
static bool segWantsStraightHold(uint8_t seg) {
  return seg == SEG_STRAIGHT_HOLD;
}
static bool segWantsCurveLong(uint8_t seg) {
  return seg == SEG_LEFT_HOLD || seg == SEG_RIGHT_HOLD;
}

// Smallest override value among rules whose pattern contains 'seg'. Returns
// 0xFFFF if no rule has an override for this segment kind.
// v38: Global emission timings always use the configured global values.
// Per-rule overrides are handled independently via per-rule timers in
// updateSegmentDetection(), no longer through min-of-overrides aggregation.
static uint16_t effectivePeakAfterMs() { return (uint16_t)CURVE_PEAK_AFTER_MS; }
static uint16_t effectiveStraightHoldMs() { return (uint16_t)STRAIGHT_HOLD_MS; }
static uint16_t effectiveCurveLongMs() { return (uint16_t)CURVE_LONG_MS; }

static bool ruleTriggerAllowsFire(Rule &r) {
  r.matchCount++;
  uint8_t n = r.triggerN == 0 ? 1 : r.triggerN;
  uint8_t phase = r.triggerPhase == 0 ? 1 : r.triggerPhase;
  if (phase > n) phase = n;
  switch (r.triggerMode) {
    case TRIG_ODD:     return (r.matchCount % 2) == 1;
    case TRIG_EVEN:    return (r.matchCount % 2) == 0;
    case TRIG_EVERY_N: return (r.matchCount >= phase) && (((r.matchCount - phase) % n) == 0);
    case TRIG_ONLY_N:  return r.matchCount == n;
    case TRIG_EVERY:
    default:           return true;
  }
}

static uint16_t segmentBit(uint8_t seg) {
  if (seg >= 16) return 0;
  return (uint16_t)(1UL << seg);
}

static void cancelRuleConfirmation(Rule &r, uint8_t ruleIdx, uint8_t seg, float gz) {
  r.confirmPending = false;
  // Keep the already matched prefix so a false final candidate can be discarded
  // without losing the prior structural flag (e.g. WAVE -> wait for next LEFT_HOLD).
  r.matchIdx = r.confirmPrefixIdx;
  if (r.matchIdx > r.patternLen) r.matchIdx = 0;
  r.wildcardEntryMs = 0;
  logEvent(seg, ruleIdx, ACTION_CANCEL, gz);
}

static void fireRuleActionsAfterConfirm(uint8_t ruleIdx, float gz, uint32_t now) {
  if (ruleIdx >= rulesCount) return;
  Rule &r = rules[ruleIdx];
  if (ruleTriggerAllowsFire(r)) {
    startActionsFromRule(ruleIdx, gz, now);
    if (!r.loopMode) r.enabled = false;
  } else {
    logEvent(lastSegment, ruleIdx, ACTION_NONE, gz);
  }
}

static void completeRulePattern(uint8_t ruleIdx, uint8_t seg, float gz, uint32_t now) {
  if (ruleIdx >= rulesCount) return;
  Rule &r = rules[ruleIdx];
  r.matchIdx = 0;
  r.wildcardEntryMs = 0;

  if (r.confirmMs > 0 && r.confirmCancelMask != 0) {
    r.confirmPending = true;
    r.confirmStartMs = now;
    r.confirmPrefixIdx = (r.patternLen > 0) ? (uint8_t)(r.patternLen - 1) : 0;
    r.confirmGz = gz;
    // Log the candidate match as ACTION_NONE; a later CANCEL or action log shows outcome.
    logEvent(seg, ruleIdx, ACTION_NONE, gz);
    return;
  }

  fireRuleActionsAfterConfirm(ruleIdx, gz, now);
}

static void tickRuleConfirmations(float gz, uint32_t now) {
  (void)gz;
  for (uint8_t i = 0; i < rulesCount; i++) {
    Rule &r = rules[i];
    if (!r.enabled || !r.confirmPending) continue;
    if ((uint32_t)(now - r.confirmStartMs) < (uint32_t)r.confirmMs) continue;
    r.confirmPending = false;
    r.matchIdx = 0;
    r.wildcardEntryMs = 0;
    fireRuleActionsAfterConfirm(i, r.confirmGz, now);
  }
}

// Production matcher policy:
//   - Exact order is kept.
//   - Duplicate events of the last matched segment are ignored.
//   - WAVE is ignored while the rule is waiting for LEFT/RIGHT.
// This is intentional: on a real car, wave oscillation and corner vibration can
// insert WAVE/RIGHT/LEFT noise between structural course sections. The motor
// action must not depend on a fragile exact event stream.
static void advanceRuleWithSegment(Rule &r, uint8_t seg, uint8_t ruleIdx, float gz, uint32_t now) {
  if (!r.enabled || r.patternLen == 0) return;

  // v39: pending confirmation. During the short confirmation window, only
  // configured cancel segments are meaningful. Other segments are ignored so
  // the action-placement WAIT remains separate from candidate validation.
  if (r.confirmPending) {
    // If the verification window has already elapsed, confirm before this new
    // segment can cancel the candidate. This matters because segment detection
    // runs before the periodic confirmation tick in the control loop.
    if ((uint32_t)(now - r.confirmStartMs) >= (uint32_t)r.confirmMs) {
      r.confirmPending = false;
      r.matchIdx = 0;
      r.wildcardEntryMs = 0;
      fireRuleActionsAfterConfirm(ruleIdx, r.confirmGz, now);
      return;
    }
    if ((r.confirmCancelMask & segmentBit(seg)) != 0) {
      cancelRuleConfirmation(r, ruleIdx, seg, gz);
    }
    return;
  }

  // v3.9 wildcard: while the rule sits at a wildcard slot, ignore EVERY
  // incoming segment. The wildcard timer is advanced from
  // tickRuleWildcards() which runs every control tick (5ms).
  if (r.matchIdx < r.patternLen && r.pattern[r.matchIdx] == SEG_WILDCARD) {
    if (r.wildcardEntryMs == 0) r.wildcardEntryMs = now;
    return;
  }

  uint8_t expected = r.pattern[r.matchIdx];

  if (segmentMatches(expected, seg)) {
    r.matchIdx++;
    // v3.9: if the next slot is a wildcard, start its timer immediately so
    // the configured wait begins from this segment match (not from a
    // ~5ms-later tick).
    if (r.matchIdx < r.patternLen && r.pattern[r.matchIdx] == SEG_WILDCARD) {
      r.wildcardEntryMs = now;
    } else {
      r.wildcardEntryMs = 0;
    }
    if (r.matchIdx >= r.patternLen) {
      // v39: complete match may enter pending confirmation instead of firing now.
      completeRulePattern(ruleIdx, seg, gz, now);
    }
    return;
  }

  if (r.matchIdx > 0 && segmentMatches(r.pattern[r.matchIdx - 1], seg)) {
    return;
  }
  if (isAuxSegment(seg) && expected != seg) {
    return;
  }
  r.matchIdx = segmentMatches(r.pattern[0], seg) ? 1 : 0;
  // v3.9: matchIdx just changed — re-arm wildcard timer if we land on one,
  // otherwise clear it.
  if (r.matchIdx < r.patternLen && r.pattern[r.matchIdx] == SEG_WILDCARD) {
    r.wildcardEntryMs = now;
  } else {
    r.wildcardEntryMs = 0;
  }
}

static void resetRuleProgress() {
  for (uint8_t i = 0; i < MAX_RULES; i++) {
    rules[i].matchIdx = 0;
    rules[i].matchCount = 0;
    rules[i].enabled = true;
    rules[i].perRuleCurveHoldEmitted = false;
    rules[i].perRulePeakEmitted = false;
    rules[i].perRuleStraightHoldEmitted = false;
    // v3.9: wildcard timers always reset at run start. If pattern[0] is
    // a wildcard, tickRuleWildcards() will set the entry timestamp on the
    // first control tick (effectively measuring from run start).
    rules[i].wildcardEntryMs = 0;
    rules[i].confirmPending = false;
    rules[i].confirmStartMs = 0;
    rules[i].confirmPrefixIdx = 0;
    rules[i].confirmGz = 0.0f;
  }
  activeRuleIdx = 0xFF;
  activePriority = 0;
}

// v3.9 wildcard tick. Called every control cycle (200Hz). Advances any rule
// whose matchIdx currently points at a SEG_WILDCARD whose configured
// wildcardMs has elapsed. If the wildcard happens to be the LAST element of
// the pattern, the rule fires here (no real segment is required to terminate
// a trailing wildcard).
static void tickRuleWildcards(float gz, uint32_t now) {
  for (uint8_t i = 0; i < rulesCount; i++) {
    Rule &r = rules[i];
    if (!r.enabled || r.patternLen == 0) continue;
    if (r.matchIdx >= r.patternLen) continue;
    if (r.pattern[r.matchIdx] != SEG_WILDCARD) continue;
    if (r.wildcardEntryMs == 0) r.wildcardEntryMs = now;
    if ((uint32_t)(now - r.wildcardEntryMs) < (uint32_t)r.wildcardMs) continue;

    // wildcard time elapsed: advance past the wildcard slot
    r.matchIdx++;
    r.wildcardEntryMs = 0;
    // log a synthetic wildcard event so the trace dump shows the auto-advance
    emitSegmentLog(SEG_WILDCARD, i, gz, now);

    // chain: if the next slot is ALSO a wildcard, prime its entry now so
    // its wait starts immediately rather than 5ms later.
    if (r.matchIdx < r.patternLen && r.pattern[r.matchIdx] == SEG_WILDCARD) {
      r.wildcardEntryMs = now;
    }

    if (r.matchIdx >= r.patternLen) {
      // wildcard was the last element of the pattern — complete the rule now.
      completeRulePattern(i, SEG_WILDCARD, gz, now);
    }
  }
}

static void startNextAction(uint32_t now) {
  while (activeActionIdx < activeActionCount) {
    RuleAction &a = activeActions[activeActionIdx];

    if (a.type == ACTION_NONE) {
      activeActionIdx++;
      continue;
    }

    // Log every action start. v3.22 also records the final duration and
    // correction deltas so the stopped log shows what voltage compensation did.
    logActionEvent(lastSegment, activeRuleIdx, a.type, activeTriggerGz,
                   activeActionOriginalIdx[activeActionIdx], a.durationMs,
                   activeVoltageDeltaMs[activeActionIdx], activeSagDeltaMs[activeActionIdx]);

    if (a.type == ACTION_STOP) {
      motorBrake();
      isRunning = false;
      actionActive = false;
      setStatus(STATUS_STOPPED);
      dumpStart();
      return;
    }

    if (a.type == ACTION_BRAKE) {
      motorBrake();
      beginBrakeVbatWindow(now, a.durationMs);
    } else if (a.type == ACTION_SPEED) {
      motorDuty(dutyFromStrength(a.strength));
    } else if (a.type == ACTION_WAIT) {
      motorDuty(NORMAL_DUTY);
      beginWaitSagWindow(now, a.durationMs);
    } else if (a.type == ACTION_LOCK) {
      // Hold the current motor output as-is. Only update the global lock
      // window so that subsequent rule firings are suppressed for the same
      // duration. The action itself is time-based like WAIT.
      uint16_t lockDur = a.durationMs;
      if (lockDur == 0) lockDur = 1;
      actionLockUntilMs = now + lockDur;
    }

    uint16_t dur = a.durationMs;
    if (dur == 0) dur = 1;
    actionEndMs = now + dur;
    actionActive = true;
    return;
  }

  actionActive = false;
  activeActionCount = 0;
  activeActionIdx = 0;
  activeRuleIdx = 0xFF;
  activePriority = 0;
  activeTriggerGz = 0.0f;
  if (isRunning) motorDuty(NORMAL_DUTY);
}

static bool adaptiveSagAllowedByDuty(const Rule &r) {
  uint8_t sagMode = constrain(r.adaptiveSagMode, (uint8_t)0, (uint8_t)2);
  if (sagMode == ADAPT_SAG_ALWAYS) return true;
  if (sagMode == ADAPT_SAG_DUTY_MIN) {
    uint8_t minPct = constrain(r.adaptiveSagDutyMinPct, (uint8_t)0, (uint8_t)100);
    uint8_t minDuty = (uint8_t)(((uint16_t)minPct * 255u + 50u) / 100u);
    return currentMotorDuty >= minDuty;
  }
  return false;
}

static void applyAdaptiveDurationToAction(uint8_t i, uint16_t waitBaseMs, uint16_t brakeBaseMs,
                                          int16_t waitSagDeltaMs, int16_t brakeSagDeltaMs) {
  if (i >= activeActionCount) return;
  RuleAction &a = activeActions[i];
  if (a.type != ACTION_WAIT && a.type != ACTION_BRAKE) return;

  uint16_t originalMs = a.durationMs;
  int32_t baseMs = (a.type == ACTION_WAIT) ? (int32_t)waitBaseMs : (int32_t)brakeBaseMs;
  int32_t sagDelta = (a.type == ACTION_WAIT) ? (int32_t)waitSagDeltaMs : (int32_t)brakeSagDeltaMs;
  uint16_t finalMs = clampMsFromSigned(baseMs + sagDelta);

  a.durationMs = finalMs;
  activeVoltageDeltaMs[i] = clampI16FromInt32(baseMs - (int32_t)originalMs);
  activeSagDeltaMs[i] = clampI16FromInt32((int32_t)finalMs - baseMs);
}

static void applyAdaptiveBrakeToActiveActions(const Rule &r, uint32_t now) {
  if (!r.adaptiveBrakeEnabled || activeActionCount == 0) return;

  // v3.21: Base voltage correction uses stopped/unloaded VBAT only.
  // Load-sag correction is a separate term so low-battery compensation and
  // load/speed compensation can have independent signs.
  uint16_t baseVbatMv = adaptiveBrakeInputVbatMv(now, r.adaptiveSampleMs);
  if (baseVbatMv == 0xFFFF) return;

  uint16_t waitBaseMs = interpByVoltageMv(baseVbatMv, r.adaptiveVLowMv, r.adaptiveVHighMv,
                                          r.adaptiveWaitLowMs, r.adaptiveWaitHighMs);
  uint16_t brakeBaseMs = interpByVoltageMv(baseVbatMv, r.adaptiveVLowMv, r.adaptiveVHighMv,
                                           r.adaptiveBrakeLowMs, r.adaptiveBrakeHighMs);

  int16_t waitSagDeltaMs = 0;
  int16_t brakeSagDeltaMs = 0;
  if (adaptiveSagAllowedByDuty(r)) {
    uint16_t runningMv = averageRecentVbatMv(now, r.adaptiveSampleMs);
    if (runningMv != 0xFFFF && baseVbatMv > runningMv) {
      uint16_t sagMv = (uint16_t)(baseVbatMv - runningMv);
      waitSagDeltaMs = clampI16FromInt32(((int32_t)r.adaptiveSagWaitMsPerV * (int32_t)sagMv) / 1000);
      brakeSagDeltaMs = clampI16FromInt32(((int32_t)r.adaptiveSagBrakeMsPerV * (int32_t)sagMv) / 1000);
    }
  }

  uint8_t mode = r.adaptiveTargetMode;
  if (mode > ADAPT_TARGET_SELECTED) mode = ADAPT_TARGET_FIRST_PAIR;

  if (mode == ADAPT_TARGET_ALL) {
    for (uint8_t i = 0; i < activeActionCount; i++) {
      applyAdaptiveDurationToAction(i, waitBaseMs, brakeBaseMs, waitSagDeltaMs, brakeSagDeltaMs);
    }
    return;
  }

  if (mode == ADAPT_TARGET_SELECTED) {
    uint8_t mask = r.adaptiveTargetMask & ((1u << MAX_ACTIONS) - 1u);
    if (mask == 0) return;
    for (uint8_t i = 0; i < activeActionCount && i < MAX_ACTIONS; i++) {
      if ((mask & (1u << i)) == 0) continue;
      applyAdaptiveDurationToAction(i, waitBaseMs, brakeBaseMs, waitSagDeltaMs, brakeSagDeltaMs);
    }
    return;
  }

  // Default / legacy behavior: adjust the first WAIT and the first BRAKE after it.
  // If there is no WAIT, adjust the first BRAKE only.
  int8_t waitIdx = -1;
  int8_t brakeIdx = -1;
  for (uint8_t i = 0; i < activeActionCount; i++) {
    if (waitIdx < 0 && activeActions[i].type == ACTION_WAIT) {
      waitIdx = (int8_t)i;
      continue;
    }
    if (activeActions[i].type == ACTION_BRAKE && (waitIdx < 0 || i > (uint8_t)waitIdx)) {
      brakeIdx = (int8_t)i;
      break;
    }
  }
  if (brakeIdx < 0) {
    for (uint8_t i = 0; i < activeActionCount; i++) {
      if (activeActions[i].type == ACTION_BRAKE) {
        brakeIdx = (int8_t)i;
        break;
      }
    }
  }
  if (waitIdx >= 0) applyAdaptiveDurationToAction((uint8_t)waitIdx, waitBaseMs, brakeBaseMs, waitSagDeltaMs, brakeSagDeltaMs);
  if (brakeIdx >= 0) applyAdaptiveDurationToAction((uint8_t)brakeIdx, waitBaseMs, brakeBaseMs, waitSagDeltaMs, brakeSagDeltaMs);
}

static void startActionsFromRule(uint8_t ruleIdx, float gz, uint32_t now) {
  if (ruleIdx >= rulesCount) return;
  Rule &r = rules[ruleIdx];
  if (r.actionCount == 0) return;

  // Global action lockout: while in the lock window, suppress new rule
  // firings. We still allow rules whose *first* action is ACTION_LOCK,
  // because the user explicitly intends to extend / re-arm the lockout
  // from another segment context (e.g. re-lock after a jump landing).
  if (!reached(now, actionLockUntilMs) && r.actions[0].type != ACTION_LOCK) {
    return;
  }

  uint8_t prio = priorityForRule(ruleIdx);
  // Higher priority can preempt lower-priority actions.
  // If priority is equal, the earlier rule index wins.
  if (actionActive) {
    if (prio < activePriority) return;
    if (prio == activePriority && activeRuleIdx != 0xFF && ruleIdx >= activeRuleIdx) return;
  }

  for (uint8_t i = 0; i < MAX_ACTIONS; i++) {
    activeActions[i] = r.actions[i];
    activeVoltageDeltaMs[i] = 0;
    activeSagDeltaMs[i] = 0;
    activeActionOriginalIdx[i] = i;
  }
  activeActionCount = r.actionCount;
  applyAdaptiveBrakeToActiveActions(r, now);
  activeActionIdx = 0;
  activeRuleIdx = ruleIdx;
  activePriority = prio;
  activeTriggerGz = gz;
  startNextAction(now);
}

static void updateActionEngine(uint32_t now) {
  if (!actionActive) return;
  if (reached(now, actionEndMs)) {
    if (activeActionIdx < activeActionCount && activeActions[activeActionIdx].type == ACTION_WAIT) {
      endWaitSagWindow(now);
    }
    if (activeActionIdx < activeActionCount && activeActions[activeActionIdx].type == ACTION_BRAKE) {
      endBrakeVbatWindow(now);
    }
    activeActionIdx++;
    startNextAction(now);
  }
}

static void queueLiveSegment(uint8_t seg, uint32_t now, uint32_t durMs) {
  if (!DEBUG_LIVE_SEG) return;
  if (isRunning && !LIVE_SEG_NOTIFY_DURING_RUN) return;
  pushLiveSegPacketRaw(seg, now, durMs);
}

static bool popLiveSegment(LiveSegPacket &out) {
  if (liveSegCount == 0) return false;
  out = liveSegQueue[liveSegTail];
  liveSegTail = (uint8_t)((liveSegTail + 1) % LIVE_SEG_QUEUE_N);
  liveSegCount--;
  return true;
}

static void clearLiveSegmentQueue() {
  liveSegHead = 0;
  liveSegTail = 0;
  liveSegCount = 0;
}

// Logs the segment event and (optionally) updates live debug. Used by both
// global broadcast and per-rule targeted dispatch.
// v3.11: optional peakDps + durMs are written to the log entry for richer
// host-side display (curve enter / PEAK / HOLD / straight-hold).
static void emitSegmentLog(uint8_t seg, uint8_t ruleIdx, float gz, uint32_t now, float peakDps, uint32_t durMs) {
  lastSegment = seg;
  logEventEx(seg, ruleIdx, ACTION_NONE, gz, peakDps, durMs);
  queueLiveSegment(seg, now, durMs);
}

// Dispatch a segment event to all rules in priority order (the "global" path,
// for rules whose override for this segment kind is 0xFFFF=inherit).
// SEGMENT_COOLDOWN_MS guards repeated emits of the same segment ID.
// v3.11: peakDps / durMs are passed through to the log entry. Defaults of 0
// preserve existing call sites that have nothing meaningful to report
// (SEG_WAVE, SEG_STRAIGHT enter, etc).
static void onSegmentDetected(uint8_t seg, float gz, uint32_t now, float peakDps = 0.0f, uint32_t durMs = 0UL) {
  if (seg < 16 && lastSegEventMs[seg] != 0 && (uint32_t)(now - lastSegEventMs[seg]) < SEGMENT_COOLDOWN_MS) {
    return;
  }
  if (seg < 16) lastSegEventMs[seg] = now;

  emitSegmentLog(seg, 0xFF, gz, now, peakDps, durMs);
  processLapSegment(seg, now);

  // Evaluate rules by explicit priority, not by visual list order alone.
  // This prevents a broad low-priority rule from consuming a segment before
  // a narrow high-priority safety rule can fire. Same priority falls back to
  // the list order: earlier rule wins.
  // v38: skip rules that have a per-rule override for this segment kind —
  // those receive their own targeted events from the per-rule timers.
  bool used[MAX_RULES] = {false};
  for (uint8_t pass = 0; pass < rulesCount; pass++) {
    int best = -1;
    uint8_t bestPrio = 0;
    for (uint8_t i = 0; i < rulesCount; i++) {
      if (used[i]) continue;
      uint8_t p = priorityForRule(i);
      if (best < 0 || p > bestPrio || (p == bestPrio && i < (uint8_t)best)) {
        best = i;
        bestPrio = p;
      }
    }
    if (best < 0) break;
    used[best] = true;
    Rule &r = rules[best];
    // Skip rules that own a per-rule override for this segment kind:
    // they get a separate targeted dispatch via onSegmentDetectedForRule.
    if (segWantsPeakAfter(seg)        && r.peakAfterOverrideMs    != 0xFFFF) continue;
    if (segWantsStraightHold(seg)     && r.straightHoldOverrideMs != 0xFFFF) continue;
    if (segWantsCurveLong(seg)        && r.curveLongOverrideMs    != 0xFFFF) continue;
    advanceRuleWithSegment(r, seg, (uint8_t)best, gz, now);
  }
}

// v38: Targeted segment dispatch for a single rule.
// Used by per-rule timers when a rule's own override interval has been reached.
// Logs the event with the rule index so traces clearly show which rule received
// it, then advances only that rule's pattern matcher. No global cooldown — each
// rule's own per-rule emitted-flag prevents re-fire within the same curve.
// v3.11: peak/dur context forwarded to log.
static void onSegmentDetectedForRule(uint8_t ruleIdx, uint8_t seg, float gz, uint32_t now, float peakDps = 0.0f, uint32_t durMs = 0UL) {
  if (ruleIdx >= rulesCount) return;
  emitSegmentLog(seg, ruleIdx, gz, now, peakDps, durMs);
  advanceRuleWithSegment(rules[ruleIdx], seg, ruleIdx, gz, now);
}

// ============================================================
// Segment detection. Emits curve-start events, not curve-end events,
// so rules can fire quickly like the M5StampS3 version.
// ============================================================
static void resetDetection(uint32_t now) {
  inCorner = false;
  cornerSign = 0;
  cornerStartMs = now;
  straightCandMs = 0;
  straightHoldStartMs = 0;
  straightHoldEmitted = false;
  cornerPeakAbs = 0.0f;
  cornerPeakMs = now;
  cornerPeakEmitted = false;
  cornerHoldEmitted = false;
  // v38: clear per-rule emission flags too.
  for (uint8_t i = 0; i < MAX_RULES; i++) {
    rules[i].perRuleCurveHoldEmitted = false;
    rules[i].perRulePeakEmitted = false;
    rules[i].perRuleStraightHoldEmitted = false;
    rules[i].confirmPending = false;
  }
  for (uint8_t i = 0; i < 16; i++) lastSegEventMs[i] = 0;
  lastPosSpikeMs = 0;
  lastNegSpikeMs = 0;
  waveLatchUntilMs = 0;
  waveCooldownUntilMs = 0;
  lastReversalMs = 0;
  reversalRunCount = 0;
  lastSegment = SEG_NONE;
  for (uint8_t i = 0; i < GZ_FILT_N; i++) gzBuf[i] = 0.0f;
  gzIdx = 0;
}

// v3.35: Start (or restart) a corner with the given sign. Factored out of the
// old !inCorner branch so the in-corner reversal path can reuse exactly the
// same setup. When `forceEmit` is set (reversal case), the per-segment cooldown
// is cleared first so a fast wave's alternating lobes are NOT suppressed by
// SEGMENT_COOLDOWN_MS.
static void beginCorner(int8_t sign, float gz, uint32_t now, bool forceEmit) {
  inCorner = true;
  cornerStartMs = now;
  straightCandMs = 0;
  straightHoldStartMs = 0;
  straightHoldEmitted = false;
  cornerSign = sign;
  cornerPeakAbs = fabsf(gz);
  cornerPeakMs = now;
  cornerPeakEmitted = false;
  cornerHoldEmitted = false;
  for (uint8_t i = 0; i < rulesCount; i++) {
    rules[i].perRuleCurveHoldEmitted = false;
    rules[i].perRulePeakEmitted = false;
  }
  uint8_t seg = (sign > 0) ? SEG_LEFT : SEG_RIGHT;
  if (forceEmit && seg < 16) lastSegEventMs[seg] = 0;  // bypass cooldown for this structural event
  onSegmentDetected(seg, gz, now);
}

static void updateSegmentDetection(float gz, uint32_t now) {
  float a = fabsf(gz);

  // Wave: M5-like positive/negative spike pair within a time window.
  // If the car naturally reports wave as LEFT->RIGHT->LEFT, rules can simply
  // use those structural segments and ignore SEG_WAVE.
  if (gz > WAVE_POS_TH) lastPosSpikeMs = now;
  if (gz < -WAVE_NEG_TH) lastNegSpikeMs = now;

  if (reached(now, waveCooldownUntilMs) && lastPosSpikeMs > 0 && lastNegSpikeMs > 0) {
    uint32_t gap = (lastPosSpikeMs > lastNegSpikeMs)
                 ? (lastPosSpikeMs - lastNegSpikeMs)
                 : (lastNegSpikeMs - lastPosSpikeMs);
    if (gap > 0 && gap <= WAVE_WINDOW_MS) {
      waveLatchUntilMs = now + WAVE_LATCH_MS;
      waveCooldownUntilMs = now + WAVE_COOLDOWN_MS;
      lastPosSpikeMs = 0;
      lastNegSpikeMs = 0;
      onSegmentDetected(SEG_WAVE, gz, now);
      // Do not return. A strong wave sample can also be the start of a curve.
    }
  }

  // Corner start: emit immediately for fast rule matching.
  if (!inCorner && a > CORNER_TH) {
    reversalRunCount = 0;   // fresh corner; reversal chain restarts
    beginCorner((gz >= 0.0f) ? +1 : -1, gz, now, false);
    return;
  }

  if (inCorner) {
    // v3.35: In-corner reversal. If the yaw swings to the OPPOSITE direction
    // and re-crosses CORNER_TH before a clean straight exit was ever seen,
    // the old detector merged it into the current corner and dropped the new
    // segment entirely. This is the main cause of skipped segments at speed
    // and of missed waves (which are just rapid L/R/L/R reversals). Treat it
    // as: close the current corner (emit its PEAK if not already emitted) and
    // immediately begin a new corner with the new sign.
    int8_t liveSign = (gz >= 0.0f) ? +1 : -1;
    float reversalTh = fmaxf(CORNER_TH, REVERSAL_MIN_ABS_DPS);
    if (liveSign != cornerSign && a > reversalTh
        && (uint32_t)(now - cornerStartMs) >= REVERSAL_MIN_AGE_MS) {
      // Close the outgoing corner with a PEAK marker so rules / lap patterns
      // keyed on LEFT_PEAK / RIGHT_PEAK still see the exit of the old lobe.
      if (!cornerPeakEmitted) {
        uint8_t oldPeak = (cornerSign > 0) ? SEG_LEFT_PEAK : SEG_RIGHT_PEAK;
        onSegmentDetected(oldPeak, gz, now, cornerPeakAbs, (uint32_t)(now - cornerStartMs));
        cornerPeakEmitted = true;
      }
      // Wave-by-structure: count quick consecutive reversals. Two or more
      // reversals inside WAVE_WINDOW_MS is a robust, speed-tolerant wave signal
      // that does not depend on the +/- spike-pair amplitudes lining up.
      if (lastReversalMs != 0 && (uint32_t)(now - lastReversalMs) <= WAVE_WINDOW_MS) {
        reversalRunCount++;
      } else {
        reversalRunCount = 1;
      }
      lastReversalMs = now;
      if (reversalRunCount >= 2 && reached(now, waveCooldownUntilMs)) {
        waveLatchUntilMs    = now + WAVE_LATCH_MS;
        waveCooldownUntilMs = now + WAVE_COOLDOWN_MS;
        lastPosSpikeMs = 0;
        lastNegSpikeMs = 0;
        onSegmentDetected(SEG_WAVE, gz, now);
      }
      beginCorner(liveSign, gz, now, true);  // force-emit past the cooldown
      return;
    }

    // Track the maximum absolute yaw rate in the current curve. Once the value
    // has clearly dropped from that peak, emit LEFT_PEAK/RIGHT_PEAK. A rule can
    // then use: LEFT_PEAK -> WAIT 120ms -> BRAKE, which means "after the curve
    // peak, wait 120ms, then act".
    if (a > cornerPeakAbs) {
      cornerPeakAbs = a;
      cornerPeakMs = now;
    }

    uint32_t curveAgeMs = now - cornerStartMs;

    // Curve-hold event: emitted only when the same LEFT/RIGHT curve remains
    // active for CURVE_LONG_MS. This separates a long 180-degree curve from
    // a short 90-degree curve followed by the opposite 90-degree curve.
    // (LEFT_HOLD/RIGHT_HOLD is now the sole long-curve discriminator; the
    //  former *_LONG_PEAK events were removed because PEAK already fires at
    //  curve exit and HOLD already filters by curve length.)
    uint8_t holdSeg = (cornerSign > 0) ? SEG_LEFT_HOLD : SEG_RIGHT_HOLD;
    if (!cornerHoldEmitted && curveAgeMs >= effectiveCurveLongMs()) {
      cornerHoldEmitted = true;
      // v3.11: include cornerPeakAbs (so far) and curveAgeMs in log.
      onSegmentDetected(holdSeg, gz, now, cornerPeakAbs, curveAgeMs);
    }
    // v38: per-rule curve-hold timers. A rule with curveLongOverrideMs set
    // gets its own targeted dispatch when its override interval is reached.
    // The flag perRuleCurveHoldEmitted prevents re-fire within the same curve.
    for (uint8_t i = 0; i < rulesCount; i++) {
      Rule &r = rules[i];
      if (!r.enabled || r.curveLongOverrideMs == 0xFFFF) continue;
      if (r.perRuleCurveHoldEmitted) continue;
      if (curveAgeMs >= (uint32_t)r.curveLongOverrideMs) {
        r.perRuleCurveHoldEmitted = true;
        onSegmentDetectedForRule(i, holdSeg, gz, now, cornerPeakAbs, curveAgeMs);
      }
    }

    // v3.10: 「ピーク値からの相対 % 落下」で発火判定。
    //   peakPassed = a が peakAbs * (CURVE_PEAK_DROP_PCT/100) を一度でも下回ったか
    // (整数演算のため 100*a < peakAbs * pct で比較)
    uint32_t pct = (uint32_t)CURVE_PEAK_DROP_PCT;
    bool dropOk = (uint32_t)(100.0f * a) < (uint32_t)(cornerPeakAbs * pct);
    bool peakPassed = cornerPeakAbs >= CORNER_TH
                   && (uint32_t)(now - cornerPeakMs) >= effectivePeakAfterMs()
                   && dropOk;

    uint8_t peakSeg = (cornerSign > 0) ? SEG_LEFT_PEAK : SEG_RIGHT_PEAK;
    if (!cornerPeakEmitted && peakPassed) {
      cornerPeakEmitted = true;
      // v3.11: PEAK gets cornerPeakAbs + curveAgeMs.
      onSegmentDetected(peakSeg, gz, now, cornerPeakAbs, curveAgeMs);
    }
    // v38: per-rule peak-after timers. PEAK requires the corner to have peaked
    // already (cornerPeakAbs >= CORNER_TH) AND a drop from peak — but the per-
    // rule timer governs the "after how long since the peak" portion only.
    // We still gate on the physical drop condition so an early override doesn't
    // fire mid-corner before any real peak has occurred.
    if (cornerPeakAbs >= CORNER_TH && dropOk) {
      for (uint8_t i = 0; i < rulesCount; i++) {
        Rule &r = rules[i];
        if (!r.enabled || r.peakAfterOverrideMs == 0xFFFF) continue;
        if (r.perRulePeakEmitted) continue;
        if ((uint32_t)(now - cornerPeakMs) >= (uint32_t)r.peakAfterOverrideMs) {
          r.perRulePeakEmitted = true;
          onSegmentDetectedForRule(i, peakSeg, gz, now, cornerPeakAbs, curveAgeMs);
        }
      }
    }

    // Corner exit debounce only prevents duplicate curve-start events.
    if (a < STRAIGHT_TH) {
      if (straightCandMs == 0) straightCandMs = now;
      if ((now - straightCandMs) >= EXIT_DEBOUNCE_MS) {
        inCorner = false;
        cornerSign = 0;
        straightCandMs = 0;
        straightHoldStartMs = now;
        straightHoldEmitted = false;
      }
    } else {
      straightCandMs = 0;
    }
    return;
  }

  // Straight hold: emit once after a straight has continued below STRAIGHT_TH
  // for STRAIGHT_HOLD_MS. This is deliberately a one-shot per straight section.
  if (a < STRAIGHT_TH) {
    if (straightHoldStartMs == 0) straightHoldStartMs = now;
    uint32_t straightAgeMs = now - straightHoldStartMs;
    if (!straightHoldEmitted && straightAgeMs >= effectiveStraightHoldMs()) {
      straightHoldEmitted = true;
      // v3.11: peak=0 (no curve), dur=straightAgeMs.
      onSegmentDetected(SEG_STRAIGHT_HOLD, gz, now, 0.0f, straightAgeMs);
    }
    // v38: per-rule straight-hold timers.
    for (uint8_t i = 0; i < rulesCount; i++) {
      Rule &r = rules[i];
      if (!r.enabled || r.straightHoldOverrideMs == 0xFFFF) continue;
      if (r.perRuleStraightHoldEmitted) continue;
      if (straightAgeMs >= (uint32_t)r.straightHoldOverrideMs) {
        r.perRuleStraightHoldEmitted = true;
        onSegmentDetectedForRule(i, SEG_STRAIGHT_HOLD, gz, now, 0.0f, straightAgeMs);
      }
    }
  } else {
    straightHoldStartMs = 0;
    straightHoldEmitted = false;
    // Reset per-rule straight-hold flags when the straight ends.
    for (uint8_t i = 0; i < rulesCount; i++) {
      rules[i].perRuleStraightHoldEmitted = false;
    }
  }
}


// ============================================================
// Persistent config helpers
// ============================================================
static uint16_t persistCrc16(const uint8_t *data, uint16_t len) {
  uint16_t crc = 0xFFFF;
  for (uint16_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      if (crc & 1) crc = (uint16_t)((crc >> 1) ^ 0xA001);
      else crc >>= 1;
    }
  }
  return crc;
}

static bool sanitizeDeviceName(const char *src, char *dst, uint8_t dstLen) {
  if (!dst || dstLen == 0) return false;
  dst[0] = '\0';
  if (!src) src = DEFAULT_DEVICE_NAME;

  uint8_t n = 0;
  for (uint8_t i = 0; src[i] != '\0' && n < (uint8_t)(dstLen - 1); i++) {
    char c = src[i];
    const bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                    (c >= '0' && c <= '9') || c == '_' || c == '-';
    if (ok) dst[n++] = c;
  }
  dst[n] = '\0';

  if (n == 0) {
    n = 0;
    const char *fallback = "Mini4AI";
    for (uint8_t i = 0; fallback[i] != '\0' && n < (uint8_t)(dstLen - 1); i++) {
      dst[n++] = fallback[i];
    }
    dst[n] = '\0';
  }
  return n > 0;
}

static void setDeviceNameRuntime(const char *name) {
  sanitizeDeviceName(name, deviceName, sizeof(deviceName));
}

static void writeDeviceNameToChar() {
  if (!bleReady) return;
  deviceNameChar.writeValue((const uint8_t *)deviceName, strlen(deviceName));
}

static void persistDeviceName() {
  PersistentDeviceName cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = DEVICE_NAME_MAGIC;
  cfg.version = DEVICE_NAME_VERSION;
  cfg.size = sizeof(PersistentDeviceName);
  sanitizeDeviceName(deviceName, cfg.name, sizeof(cfg.name));
  cfg.crc = 0;
  cfg.crc = persistCrc16((const uint8_t *)&cfg, sizeof(cfg));
  if ((uint32_t)EEPROM.length() >= (uint32_t)(DEVICE_NAME_ADDR + sizeof(PersistentDeviceName))) {
    EEPROM.put(DEVICE_NAME_ADDR, cfg);
  }
}

static bool loadDeviceName() {
  setDeviceNameRuntime(DEFAULT_DEVICE_NAME);
  if ((uint32_t)EEPROM.length() < (uint32_t)(DEVICE_NAME_ADDR + sizeof(PersistentDeviceName))) return false;

  PersistentDeviceName cfg;
  EEPROM.get(DEVICE_NAME_ADDR, cfg);
  if (cfg.magic != DEVICE_NAME_MAGIC) return false;
  if (cfg.version != DEVICE_NAME_VERSION) return false;
  if (cfg.size != sizeof(PersistentDeviceName)) return false;

  PersistentDeviceName tmp = cfg;
  uint16_t stored = tmp.crc;
  tmp.crc = 0;
  if (stored != persistCrc16((const uint8_t *)&tmp, sizeof(tmp))) return false;

  setDeviceNameRuntime(cfg.name);
  return true;
}

static void refreshStoredPacketCopiesFromRuntime() {
  uint8_t p[PARAMS_PACKET_MAX] = {0};
  writeU16LE(&p[0],  (uint16_t)lroundf(CORNER_TH));
  writeU16LE(&p[2],  (uint16_t)lroundf(STRAIGHT_TH));
  writeU16LE(&p[4],  (uint16_t)lroundf(WAVE_POS_TH));
  writeU16LE(&p[6],  (uint16_t)lroundf(WAVE_NEG_TH));
  writeU16LE(&p[8],  (uint16_t)WAVE_WINDOW_MS);
  writeU16LE(&p[10], (uint16_t)WAVE_LATCH_MS);
  writeU16LE(&p[12], (uint16_t)WAVE_COOLDOWN_MS);
  writeU16LE(&p[14], (uint16_t)EXIT_DEBOUNCE_MS);
  p[16] = NORMAL_DUTY;
  p[17] = RECOVER_DUTY;
  p[18] = START_DUTY;
  p[19] = 0;  // live/debug segment notify disabled in production-control-safe mode
  writeU16LE(&p[20], (uint16_t)CURVE_PEAK_AFTER_MS);
  writeU16LE(&p[22], (uint16_t)CURVE_PEAK_DROP_PCT);
  writeU16LE(&p[24], (uint16_t)STRAIGHT_HOLD_MS);
  writeU16LE(&p[26], (uint16_t)SEGMENT_COOLDOWN_MS);
  writeU16LE(&p[28], (uint16_t)CURVE_LONG_MS);
  memcpy(lastParamsPacket, p, PARAMS_PACKET_MAX);
  lastParamsPacketLen = PARAMS_PACKET_MAX;

  uint8_t ab[ADAPTIVE_PACKET_MAX] = {0};
  const uint8_t REC_BYTES = 23;
  uint8_t count = rulesCount > MAX_RULES ? MAX_RULES : rulesCount;
  ab[0] = 3;
  ab[1] = count;
  for (uint8_t r = 0; r < count; r++) {
    uint8_t *q = &ab[2 + r * REC_BYTES];
    const Rule &rule = rules[r];
    uint8_t sagMode = constrain(rule.adaptiveSagMode, (uint8_t)0, (uint8_t)2);
    uint8_t flags = rule.adaptiveBrakeEnabled ? 1 : 0;
    if (sagMode != ADAPT_SAG_OFF) flags |= 0x04;
    q[0] = flags;
    writeU16LE(&q[1],  rule.adaptiveVLowMv);
    writeU16LE(&q[3],  rule.adaptiveVHighMv);
    writeU16LE(&q[5],  rule.adaptiveWaitLowMs);
    writeU16LE(&q[7],  rule.adaptiveWaitHighMs);
    writeU16LE(&q[9],  rule.adaptiveBrakeLowMs);
    writeU16LE(&q[11], rule.adaptiveBrakeHighMs);
    q[13] = (uint8_t)constrain(rule.adaptiveSampleMs, (uint16_t)20, (uint16_t)250);
    q[14] = sagMode;
    q[15] = constrain(rule.adaptiveTargetMode, (uint8_t)0, (uint8_t)2);
    q[16] = rule.adaptiveTargetMask & ((1u << MAX_ACTIONS) - 1u);
    q[17] = (uint8_t)constrain(rule.adaptiveSagDutyMinPct, (uint8_t)0, (uint8_t)100);
    writeI16LE(&q[18], (int16_t)constrain(rule.adaptiveSagWaitMsPerV, (int16_t)-5000, (int16_t)5000));
    writeI16LE(&q[20], (int16_t)constrain(rule.adaptiveSagBrakeMsPerV, (int16_t)-5000, (int16_t)5000));
    q[22] = 0;
  }
  memcpy(lastAdaptivePacket, ab, 2 + count * REC_BYTES);
  lastAdaptivePacketLen = 2 + count * REC_BYTES;
}


static void persistConfig() {
  PersistentConfig cfg;
  memset(&cfg, 0xFF, sizeof(cfg));
  cfg.magic = PERSIST_MAGIC;
  cfg.version = PERSIST_VERSION;
  cfg.size = sizeof(PersistentConfig);
  cfg.paramsLen = lastParamsPacketLen;
  cfg.rulesLen = lastRulesPacketLen;
  cfg.adaptiveLen = lastAdaptivePacketLen;
  if (cfg.paramsLen > PARAMS_PACKET_MAX) cfg.paramsLen = PARAMS_PACKET_MAX;
  if (cfg.rulesLen > RULES_PACKET_MAX) cfg.rulesLen = RULES_PACKET_MAX;
  if (cfg.adaptiveLen > ADAPTIVE_PACKET_MAX) cfg.adaptiveLen = ADAPTIVE_PACKET_MAX;
  memset(cfg.params, 0, sizeof(cfg.params));
  memset(cfg.rules, 0, sizeof(cfg.rules));
  memset(cfg.adaptive, 0, sizeof(cfg.adaptive));
  memcpy(cfg.params, lastParamsPacket, cfg.paramsLen);
  memcpy(cfg.rules, lastRulesPacket, cfg.rulesLen);
  memcpy(cfg.adaptive, lastAdaptivePacket, cfg.adaptiveLen);
  cfg.crc = 0;
  cfg.crc = persistCrc16((const uint8_t *)&cfg, sizeof(cfg));
  if ((uint32_t)EEPROM.length() >= (uint32_t)(PERSIST_ADDR + sizeof(PersistentConfig))) {
    EEPROM.put(PERSIST_ADDR, cfg);  // put() uses update semantics in the Silabs EEPROM library.
  }
}

static bool validateLoadedConfig(const PersistentConfig &cfg) {
  if (cfg.magic != PERSIST_MAGIC) return false;
  if (cfg.version != PERSIST_VERSION) return false;
  if (cfg.size != sizeof(PersistentConfig)) return false;
  if (cfg.paramsLen > PARAMS_PACKET_MAX) return false;
  if (cfg.rulesLen > RULES_PACKET_MAX || cfg.rulesLen < 2) return false;
  if (cfg.adaptiveLen > ADAPTIVE_PACKET_MAX || cfg.adaptiveLen < 2) return false;
  PersistentConfig tmp = cfg;
  uint16_t stored = tmp.crc;
  tmp.crc = 0;
  return stored == persistCrc16((const uint8_t *)&tmp, sizeof(tmp));
}


static bool applyParamsPacket(const uint8_t *pIn, int len, bool updateChars) {
  if (len < 20) return false;
  uint8_t p[PARAMS_PACKET_MAX] = {0};
  int nread = len > (int)PARAMS_PACKET_MAX ? (int)PARAMS_PACKET_MAX : len;
  memcpy(p, pIn, nread);

  CORNER_TH        = constrain((float)readU16LE(&p[0]),   50.0f, 800.0f);
  STRAIGHT_TH      = constrain((float)readU16LE(&p[2]),   10.0f, 600.0f);
  WAVE_POS_TH      = constrain((float)readU16LE(&p[4]),   50.0f, 1000.0f);
  WAVE_NEG_TH      = constrain((float)readU16LE(&p[6]),   50.0f, 1000.0f);
  WAVE_WINDOW_MS   = constrain((uint32_t)readU16LE(&p[8]),  40UL, 600UL);
  WAVE_LATCH_MS    = constrain((uint32_t)readU16LE(&p[10]), 100UL, 3000UL);
  WAVE_COOLDOWN_MS = constrain((uint32_t)readU16LE(&p[12]),  50UL, 1500UL);
  EXIT_DEBOUNCE_MS = constrain((uint32_t)readU16LE(&p[14]),   5UL, 100UL);
  NORMAL_DUTY      = p[16];
  RECOVER_DUTY     = p[17];
  START_DUTY       = p[18];
  // v3.43: ignore Web/debug flag. Running BLE notifications can disturb the control loop.
  DEBUG_LIVE_SEG   = false;
  if (len >= 28) {
    CURVE_PEAK_AFTER_MS = constrain((uint32_t)readU16LE(&p[20]),   5UL, 300UL);
    uint16_t rawDrop = readU16LE(&p[22]);
    if (rawDrop >= 50 && rawDrop <= 95) {
      CURVE_PEAK_DROP_PCT = (uint8_t)rawDrop;
    } else {
      CURVE_PEAK_DROP_PCT = 95;
    }
    STRAIGHT_HOLD_MS    = constrain((uint32_t)readU16LE(&p[24]),  20UL, 2000UL);
    SEGMENT_COOLDOWN_MS = constrain((uint32_t)readU16LE(&p[26]),   0UL, 500UL);
  }
  if (len >= 30) {
    CURVE_LONG_MS       = constrain((uint32_t)readU16LE(&p[28]),  30UL, 1200UL);
  }

  refreshStoredPacketCopiesFromRuntime();
  if (updateChars && bleReady) writeParamsToChar();
  return true;
}

static bool applyRulesPacket(const uint8_t *bufIn, int len, bool updateChars) {
  if (len < 2) return false;
  uint8_t buf[RULES_PACKET_MAX] = {0};
  if (len > (int)sizeof(buf)) len = sizeof(buf);
  memcpy(buf, bufIn, len);

  uint8_t count = buf[1];
  if (count > MAX_RULES) count = MAX_RULES;

  rulesCount = count;
  for (uint8_t r = 0; r < MAX_RULES; r++) {
    memset(&rules[r], 0, sizeof(Rule));
    setAdaptiveBrakeDefaults(rules[r]);
    rules[r].peakAfterOverrideMs    = 0xFFFF;
    rules[r].straightHoldOverrideMs = 0xFFFF;
    rules[r].curveLongOverrideMs    = 0xFFFF;
    rules[r].wildcardMs             = 0;
    rules[r].wildcardEntryMs        = 0;
    rules[r].confirmMs              = 0;
    rules[r].confirmCancelMask      = 0;
    rules[r].confirmPending         = false;
    rules[r].confirmStartMs         = 0;
    rules[r].confirmPrefixIdx       = 0;
    rules[r].confirmGz              = 0.0f;
  }

  uint8_t proto = buf[0];
  uint8_t recBytes = (proto >= 5) ? RULE_BYTES : ((proto >= 4) ? RULE_BYTES_V4 : ((proto >= 3) ? RULE_BYTES_V3 : RULE_BYTES_V2));
  if (len < 2 + count * recBytes) {
    count = (uint8_t)((len - 2) / recBytes);
    rulesCount = count;
    buf[1] = count;
    len = 2 + count * recBytes;
  }

  for (uint8_t r = 0; r < count; r++) {
    const uint8_t *q = &buf[2 + r * recBytes];
    Rule &rule = rules[r];
    rule.patternLen = q[0];
    if (rule.patternLen > MAX_PATTERN_LEN) rule.patternLen = MAX_PATTERN_LEN;
    for (uint8_t i = 0; i < MAX_PATTERN_LEN; i++) rule.pattern[i] = q[1 + i];

    rule.actionCount = q[9];
    if (rule.actionCount > MAX_ACTIONS) rule.actionCount = MAX_ACTIONS;
    for (uint8_t i = 0; i < MAX_ACTIONS; i++) {
      rule.actions[i].type = q[10 + i];
      rule.actions[i].durationMs = readU16LE(&q[15 + 2 * i]);
      rule.actions[i].strength = q[25 + i];
    }
    rule.loopMode = (q[30] != 0);
    rule.triggerMode = q[31];
    if (rule.triggerMode > TRIG_ONLY_N) rule.triggerMode = TRIG_EVERY;
    rule.triggerN = q[32] == 0 ? 1 : q[32];
    // v3.41: New UI sends q[46]. Older EVERY_N packets fired on N,2N,3N..., so preserve that as phase=N.
    rule.triggerPhase = (rule.triggerMode == TRIG_EVERY_N) ? rule.triggerN : 1;
    rule.priority = q[33] == 0 ? (uint8_t)(100 - r) : q[33];
    if (proto >= 3 && recBytes >= 42) {
      rule.peakAfterOverrideMs    = readU16LE(&q[34]);
      rule.straightHoldOverrideMs = readU16LE(&q[36]);
      rule.curveLongOverrideMs    = readU16LE(&q[38]);
      rule.wildcardMs             = readU16LE(&q[40]);
    }
    if (proto >= 4 && recBytes >= 46) {
      rule.confirmMs              = readU16LE(&q[42]);
      rule.confirmCancelMask      = readU16LE(&q[44]);
    }
    if (proto >= 5 && recBytes >= 47) {
      rule.triggerPhase = q[46] == 0 ? 1 : q[46];
      if (rule.triggerPhase > rule.triggerN) rule.triggerPhase = rule.triggerN;
    }
    rule.matchIdx = 0;
    rule.matchCount = 0;
    rule.enabled = true;
    rule.perRuleCurveHoldEmitted = false;
    rule.perRulePeakEmitted = false;
    rule.perRuleStraightHoldEmitted = false;
    rule.wildcardEntryMs = 0;
    rule.confirmPending = false;
    rule.confirmStartMs = 0;
    rule.confirmPrefixIdx = 0;
    rule.confirmGz = 0.0f;
  }

  {
    const int lapOff = 2 + (int)count * recBytes;
    if (len >= lapOff + 16 && buf[lapOff + 0] == 'L' && buf[lapOff + 1] == 'A' &&
        buf[lapOff + 2] == 'P' && buf[lapOff + 3] == '1') {
      uint8_t plen = buf[lapOff + 4];
      if (plen > MAX_PATTERN_LEN) plen = MAX_PATTERN_LEN;
      lapConfig.patternLen = plen;
      for (uint8_t i = 0; i < MAX_PATTERN_LEN; i++) lapConfig.pattern[i] = buf[lapOff + 5 + i];
      lapConfig.passesPerLap = buf[lapOff + 13] == 0 ? 1 : buf[lapOff + 13];
      lapConfig.occurrenceN = buf[lapOff + 14] == 0 ? 1 : buf[lapOff + 14];
      lapConfig.occurrencePhase = buf[lapOff + 15] == 0 ? 1 : buf[lapOff + 15];
      if (lapConfig.occurrencePhase > lapConfig.occurrenceN) lapConfig.occurrencePhase = lapConfig.occurrenceN;
      lapConfig.enabled = (lapConfig.patternLen > 0);
    } else {
      lapConfig.enabled = false;
      lapConfig.patternLen = 0;
      lapConfig.passesPerLap = 1;
      lapConfig.occurrenceN = 1;
      lapConfig.occurrencePhase = 1;
    }
    resetLapMatcher();
  }

  resetRuleProgress();
  memcpy(lastRulesPacket, buf, len);
  lastRulesPacketLen = len;
  refreshStoredPacketCopiesFromRuntime();
  if (updateChars && bleReady) {
    rulesChar.writeValue(buf, len);
    writeAdaptiveBrakeToChar();
    firmwareInfoChar.writeValue((const uint8_t*)FW_VERSION, strlen(FW_VERSION));
  }
  return true;
}


// v3.46: During a run, accept only safe live timing patches from the normal
// rules packet. Full rule replacement still remains blocked while running,
// because resetting rule/lap progress during a run is unsafe. This function
// updates only action durations/strengths when the action layout is unchanged.
static bool applyRuleTimingPatchDuringRun(const uint8_t *bufIn, int len) {
  if (len < 2) return false;
  uint8_t buf[RULES_PACKET_MAX] = {0};
  if (len > (int)sizeof(buf)) len = sizeof(buf);
  memcpy(buf, bufIn, len);

  uint8_t proto = buf[0];
  uint8_t recBytes = (proto >= 5) ? RULE_BYTES : ((proto >= 4) ? RULE_BYTES_V4 : ((proto >= 3) ? RULE_BYTES_V3 : RULE_BYTES_V2));
  uint8_t count = buf[1];
  if (count > rulesCount) count = rulesCount;
  if (count > MAX_RULES) count = MAX_RULES;
  if (len < 2 + count * recBytes) count = (uint8_t)((len - 2) / recBytes);

  bool any = false;
  for (uint8_t r = 0; r < count; r++) {
    const uint8_t *q = &buf[2 + r * recBytes];
    Rule &rule = rules[r];
    uint8_t newActionCount = q[9];
    if (newActionCount > MAX_ACTIONS) newActionCount = MAX_ACTIONS;

    // Do not generally rewrite rule matching state while the car is running.
    // Safe live patch policy:
    //   1) If the action layout is unchanged, patch durations/strengths.
    //   2) v3.57: allow adding exactly one ACTION_SPEED into the action list.
    //      This supports the Web "brake replacement" tuner, where the next
    //      firing should become BRAKE shorter -> LOW POWER for longer.
    //      The currently executing activeActions[] copy is not mutated.
    bool sameLayout = (newActionCount == rule.actionCount);
    if (sameLayout) {
      for (uint8_t i = 0; i < newActionCount; i++) {
        if (rule.actions[i].type != q[10 + i]) {
          sameLayout = false;
          break;
        }
      }
    }

    bool allowInsertedSpeed = false;
    if (!sameLayout && newActionCount == rule.actionCount + 1 && newActionCount <= MAX_ACTIONS) {
      uint8_t oi = 0;
      uint8_t insertCount = 0;
      bool ok = true;
      for (uint8_t ni = 0; ni < newActionCount; ni++) {
        uint8_t nt = q[10 + ni];
        if (oi < rule.actionCount && rule.actions[oi].type == nt) {
          oi++;
          continue;
        }
        if (nt == ACTION_SPEED && insertCount == 0) {
          insertCount++;
          continue;
        }
        ok = false;
        break;
      }
      allowInsertedSpeed = ok && oi == rule.actionCount && insertCount == 1;
    }

    if (!sameLayout && !allowInsertedSpeed) continue;

    rule.actionCount = newActionCount;
    for (uint8_t i = 0; i < MAX_ACTIONS; i++) {
      if (i < newActionCount) {
        rule.actions[i].type = q[10 + i];
        rule.actions[i].durationMs = readU16LE(&q[15 + 2 * i]);
        rule.actions[i].strength = q[25 + i];
      } else {
        rule.actions[i].type = ACTION_NONE;
        rule.actions[i].durationMs = 0;
        rule.actions[i].strength = 0;
      }
    }
    any = true;
  }

  if (any) {
    memcpy(lastRulesPacket, buf, len);
    lastRulesPacketLen = len;
    refreshStoredPacketCopiesFromRuntime();
  }
  return any;
}

static bool applyAdaptiveBrakePacket(const uint8_t *bufIn, int len, bool updateChars) {
  if (len < 2) return false;
  uint8_t buf[ADAPTIVE_PACKET_MAX] = {0};
  if (len > (int)sizeof(buf)) len = sizeof(buf);
  memcpy(buf, bufIn, len);
  if (buf[0] != 1 && buf[0] != 2 && buf[0] != 3) return false;
  const uint8_t REC_BYTES = (buf[0] >= 3) ? 23 : ((buf[0] >= 2) ? 17 : 15);
  uint8_t count = buf[1];
  if (count > rulesCount) count = rulesCount;
  if (count > MAX_RULES) count = MAX_RULES;
  if (len < 2 + count * REC_BYTES) count = (uint8_t)((len - 2) / REC_BYTES);
  buf[1] = count;
  len = 2 + count * REC_BYTES;
  for (uint8_t r = 0; r < count; r++) {
    const uint8_t *q = &buf[2 + r * REC_BYTES];
    Rule &rule = rules[r];
    uint8_t flags = q[0];
    rule.adaptiveBrakeEnabled = (flags & 0x01) != 0;
    rule.adaptiveVLowMv = constrain((uint16_t)readU16LE(&q[1]), 1800, 4500);
    rule.adaptiveVHighMv = constrain((uint16_t)readU16LE(&q[3]), 1800, 4500);
    rule.adaptiveWaitLowMs = constrain((uint16_t)readU16LE(&q[5]), 0, 60000);
    rule.adaptiveWaitHighMs = constrain((uint16_t)readU16LE(&q[7]), 0, 60000);
    rule.adaptiveBrakeLowMs = constrain((uint16_t)readU16LE(&q[9]), 0, 60000);
    rule.adaptiveBrakeHighMs = constrain((uint16_t)readU16LE(&q[11]), 0, 60000);

    if (buf[0] >= 3) {
      rule.adaptiveSampleMs = constrain((uint16_t)q[13], 20, 250);
      rule.adaptiveRunVbatMixPct = 0;
      rule.adaptiveSagMode = constrain(q[14], (uint8_t)0, (uint8_t)2);
      rule.adaptiveTargetMode = constrain(q[15], (uint8_t)0, (uint8_t)2);
      rule.adaptiveTargetMask = q[16] & ((1u << MAX_ACTIONS) - 1u);
      rule.adaptiveSagDutyMinPct = constrain(q[17], (uint8_t)0, (uint8_t)100);
      rule.adaptiveSagWaitMsPerV = constrain(readI16LE(&q[18]), (int16_t)-5000, (int16_t)5000);
      rule.adaptiveSagBrakeMsPerV = constrain(readI16LE(&q[20]), (int16_t)-5000, (int16_t)5000);
    } else {
      // Legacy v1/v2 compatibility. v2 may have blended running VBAT into the
      // voltage map. Keep it readable, but v3 UI should send explicit sag terms.
      if ((flags & 0x02) != 0 && REC_BYTES >= 17) {
        rule.adaptiveSampleMs = constrain((uint16_t)q[13], 20, 250);
        rule.adaptiveRunVbatMixPct = constrain(q[14], (uint8_t)0, (uint8_t)200);
      } else {
        rule.adaptiveSampleMs = constrain((uint16_t)readU16LE(&q[13]), 20, 600);
        rule.adaptiveRunVbatMixPct = 0;
      }
      if (REC_BYTES >= 17) {
        rule.adaptiveTargetMode = constrain(q[15], (uint8_t)0, (uint8_t)2);
        rule.adaptiveTargetMask = q[16] & ((1u << MAX_ACTIONS) - 1u);
      } else {
        rule.adaptiveTargetMode = ADAPT_TARGET_FIRST_PAIR;
        rule.adaptiveTargetMask = 0;
      }
      rule.adaptiveSagMode = ADAPT_SAG_OFF;
      rule.adaptiveSagDutyMinPct = 80;
      rule.adaptiveSagWaitMsPerV = 100;
      rule.adaptiveSagBrakeMsPerV = -100;
    }
  }
  memcpy(lastAdaptivePacket, buf, len);
  lastAdaptivePacketLen = len;
  if (updateChars && bleReady) writeAdaptiveBrakeToChar();
  return true;
}


static bool loadPersistentConfig(bool updateChars) {
  persistentLoadAttempted = true;
  persistentConfigValid = false;
  if ((uint32_t)EEPROM.length() < (uint32_t)(PERSIST_ADDR + sizeof(PersistentConfig))) return false;

  PersistentConfig cfg;
  EEPROM.get(PERSIST_ADDR, cfg);
  if (!validateLoadedConfig(cfg)) return false;

  bool okParams = applyParamsPacket(cfg.params, cfg.paramsLen, updateChars);
  bool okRules = applyRulesPacket(cfg.rules, cfg.rulesLen, updateChars);
  bool okAdaptive = applyAdaptiveBrakePacket(cfg.adaptive, cfg.adaptiveLen, updateChars);
  persistentConfigValid = okParams && okRules && okAdaptive;
  return persistentConfigValid;
}

// ============================================================
// BLE callbacks
// ============================================================
static void onConnected(BLEDevice central) {
  (void)central;
  bleConnected = true;
  statusChar.writeValue(statusValue);
  { uint8_t p[13] = {0}; p[0] = lastSegment; currentSegChar.writeValue(p, 13); }
  writeTelemetryToChar(true);
  writeAdaptiveBrakeToChar();
  firmwareInfoChar.writeValue((const uint8_t*)FW_VERSION, strlen(FW_VERSION));
  writeDeviceNameToChar();
}

static void onDisconnected(BLEDevice central) {
  (void)central;
  bleConnected = false;
  // Do not stop the car on BLE disconnect. Production control is local.
}

static void onParamsWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  if (isRunning) return;  // never mutate control parameters during run

  uint8_t p[PARAMS_PACKET_MAX] = {0};
  int len = paramsChar.valueLength();
  if (len < 20) return;
  if (len > (int)sizeof(p)) len = sizeof(p);
  paramsChar.readValue(p, len);

  // BLE writes are temporary. Use CMD_SAVE_CONFIG to persist intentionally.
  applyParamsPacket(p, len, true);
}

static void onSpeedWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  if (isRunning) return;
  NORMAL_DUTY = speedChar.value();
  refreshStoredPacketCopiesFromRuntime();
  writeParamsToChar();
}

static void onRulesWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;

  int len = rulesChar.valueLength();
  if (len < 2) return;
  uint8_t buf[RULES_PACKET_MAX] = {0};
  if (len > (int)sizeof(buf)) len = sizeof(buf);
  rulesChar.readValue(buf, len);

  if (isRunning) {
    // v3.46: keep full rule replacement blocked during a run, but allow the
    // Web auto-tune loop to patch WAIT/BRAKE durations for the next firing.
    applyRuleTimingPatchDuringRun(buf, len);
    return;
  }

  // BLE writes are temporary. Use CMD_SAVE_CONFIG to persist intentionally.
  applyRulesPacket(buf, len, true);
}

static void onAdaptiveBrakeWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  int len = adaptiveBrakeChar.valueLength();
  if (len < 2) return;
  uint8_t buf[ADAPTIVE_PACKET_MAX] = {0};
  if (len > (int)sizeof(buf)) len = sizeof(buf);
  adaptiveBrakeChar.readValue(buf, len);

  if (isRunning) {
    // v3.46: adaptive timing endpoints contain only numeric correction values;
    // applying them does not reset rule/lap progress, so it is safe enough for
    // lap-by-lap auto-tune. Avoid writing back to the characteristic while run.
    applyAdaptiveBrakePacket(buf, len, false);
    return;
  }

  // BLE writes are temporary. Use CMD_SAVE_CONFIG to persist intentionally.
  applyAdaptiveBrakePacket(buf, len, true);
}



// ============================================================
// Standalone boot mode helpers
// ============================================================
static bool readAccelZAverageG(float &azOut, uint8_t samples = 12) {
  if (!imuReady) return false;
  float sum = 0.0f;
  uint8_t ok = 0;
  for (uint8_t i = 0; i < samples; i++) {
    float az = myIMU.readFloatAccelZ();
    if (!isnan(az) && isfinite(az)) {
      sum += az;
      ok++;
    }
    delay(3);
  }
  if (ok == 0) return false;
  azOut = sum / (float)ok;
  return true;
}

static bool detectUpsideDownAtBoot() {
  if (!STANDALONE_BOOT_ENABLE) return false;
  float az = 0.0f;
  if (!readAccelZAverageG(az, 20)) return false;
  return az <= STANDALONE_UPSIDE_DOWN_Z_G;
}

static void armStandaloneBootMode(uint32_t now) {
  standaloneBootArmed = true;
  standaloneCountdownActive = false;
  standaloneFaceUpSinceMs = 0;
  standaloneStartAtMs = 0;
  standaloneLastAccelCheckMs = now;
  motorBrake();
  isRunning = false;
  resetRuleProgress();
  resetDetection(now);
  setStatus(STATUS_READY);
}

static void armRun(uint32_t now) {
  if (!imuReady) {
    motorBrake();
    setStatus(STATUS_ERROR);
    return;
  }
  motorBrake();
  primeUnloadedVbatEstimate(now);
  isRunning = false;
  resetRuleProgress();
  resetDetection(now);
  setStatus(STATUS_READY);
}

static void startRun(uint32_t now) {
  if (isRunning) return;
  if (!imuReady) {
    motorBrake();
    setStatus(STATUS_ERROR);
    return;
  }
  motorBrake();
  primeUnloadedVbatEstimate(now);

  // Recalibrate at start while motor is still off.
  gzBias = calibrateGzBias();
  for (uint8_t i = 0; i < GZ_FILT_N; i++) gzBuf[i] = 0.0f;
  gzIdx = 0;

  logReset();
  resetRuleProgress();
  resetDetection(millis());
  runStartMs = millis();
  lastVoltageLogMs = runStartMs;
  logEventEx(SEG_RUN_START, 0xFF, ACTION_NONE, 0.0f, 0.0f, 0UL);
  // Initial timestamped voltage row so the stopped graph has a clear t=0 reference.
  appendVoltageLogSample(runStartMs);
  clearLiveSegmentQueue();
  lastControlMs = runStartMs;
  activeActionCount = 0;
  activeActionIdx = 0;
  actionActive = false;
  activeRuleIdx = 0xFF;
  activePriority = 0;
  activeTriggerGz = 0.0f;
  actionLockUntilMs = 0;
  waitSagWindowActive = false;
  waitSagWindowEndMs = 0;
  brakeVbatWindowActive = false;
  brakeVbatWindowEndMs = 0;
  brakeVbatMinMvSinceLap = -32768;

  isRunning = true;
  setStatus(STATUS_RUNNING);
  motorDuty(startupDutyFor(runStartMs));
}


static void updateStandaloneAutostart(uint32_t now) {
  if (!standaloneBootArmed || isRunning) return;
  if ((uint32_t)(now - standaloneLastAccelCheckMs) < 50UL) return;
  standaloneLastAccelCheckMs = now;

  float az = 0.0f;
  if (!readAccelZAverageG(az, 3)) return;

  const bool faceUp = az >= STANDALONE_FACE_UP_Z_G;
  if (!faceUp) {
    standaloneFaceUpSinceMs = 0;
    standaloneCountdownActive = false;
    standaloneStartAtMs = 0;
    motorBrake();
    return;
  }

  if (standaloneFaceUpSinceMs == 0) standaloneFaceUpSinceMs = now;
  if (!standaloneCountdownActive && (uint32_t)(now - standaloneFaceUpSinceMs) >= STANDALONE_FACE_UP_STABLE_MS) {
    standaloneCountdownActive = true;
    standaloneStartAtMs = now + STANDALONE_START_DELAY_MS;
    setStatus(STATUS_READY);
  }

  if (standaloneCountdownActive && reached(now, standaloneStartAtMs)) {
    if (!imuReady) {
      standaloneBootArmed = false;
      standaloneCountdownActive = false;
      motorBrake();
      setStatus(STATUS_ERROR);
      return;
    }
    if (rulesCount == 0) {
      // Safety: do not launch a car with no saved rules. BLE can still connect
      // and write a valid configuration, then the board can be power-cycled.
      standaloneBootArmed = false;
      standaloneCountdownActive = false;
      motorBrake();
      setStatus(STATUS_ERROR);
      return;
    }
    standaloneBootArmed = false;
    standaloneCountdownActive = false;
    startRun(now);
  }
}

static void stopRun(uint8_t stopStatus) {
  isRunning = false;
  actionActive = false;
  activeActionCount = 0;
  activeActionIdx = 0;
  activeTriggerGz = 0.0f;
  actionLockUntilMs = 0;
  waitSagWindowActive = false;
  waitSagWindowEndMs = 0;
  brakeVbatWindowActive = false;
  brakeVbatWindowEndMs = 0;
  motorBrake();
  primeUnloadedVbatEstimate(millis());
  logEventEx(SEG_RUN_STOP, 0xFF, ACTION_NONE, 0.0f, 0.0f, 0UL);
  setStatus(stopStatus);
  dumpStart();
}

static void onDeviceNameWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  if (isRunning) {
    writeDeviceNameToChar();
    return;
  }

  uint8_t buf[DEVICE_NAME_MAX_LEN + 1] = {0};
  int len = deviceNameChar.valueLength();
  if (len < 1) {
    writeDeviceNameToChar();
    return;
  }
  if (len > (int)(sizeof(buf) - 1)) len = sizeof(buf) - 1;
  deviceNameChar.readValue(buf, len);
  buf[len] = 0;

  setDeviceNameRuntime((const char *)buf);
  BLE.setLocalName(deviceName);
  BLE.setDeviceName(deviceName);
  writeDeviceNameToChar();
  persistDeviceName();
}

static void onCommandWritten(BLEDevice central, BLECharacteristic characteristic) {
  (void)central; (void)characteristic;
  uint8_t buf[8] = {0};
  int len = commandChar.valueLength();
  if (len < 1) return;
  if (len > (int)sizeof(buf)) len = sizeof(buf);
  commandChar.readValue(buf, len);
  uint8_t cmd = buf[0];
  uint32_t now = millis();

  if (cmd == CMD_STOP) {
    stopRun(STATUS_STOPPED);
  } else if (cmd == CMD_ARM) {
    armRun(now);
  } else if (cmd == CMD_START) {
    if (!isRunning) startRun(now);
  } else if (cmd == CMD_LOG_REQUEST) {
    if (!isRunning) dumpStart();
  } else if (cmd == CMD_LOG_GET) {
    if (!isRunning && len >= 3) {
      uint16_t seq = readU16LE(&buf[1]);
      sendDumpSeq(seq);
    }
  } else if (cmd == CMD_LOG_ABORT) {
    if (!isRunning) dumpAbort();
  } else if (cmd == CMD_CLEAR_LOG) {
    if (!isRunning) logReset();
  } else if (cmd == CMD_SAVE_CONFIG) {
    if (!isRunning) {
      // Persist the current runtime packets only when the user explicitly requests it.
      refreshStoredPacketCopiesFromRuntime();
      persistConfig();
      persistentConfigValid = true;
    }
  }
}

// ============================================================
// Setup / loop
// ============================================================
void setup() {
  pinMode(PWM_PIN, OUTPUT);
  digitalWrite(PWM_PIN, LOW);
  analogWrite(PWM_PIN, 0);

  analogReadResolution(12);
  pinMode(VBAT_SENSE_PIN, INPUT);
  pinMode(NTC_TEMP_PIN, INPUT);
  pinMode(MID_SENSE_PIN, INPUT);

  pinMode(LED_BUILTIN, OUTPUT);
  ledSet(false);

  pinMode(IMU_EN_PIN, OUTPUT);
  digitalWrite(IMU_EN_PIN, HIGH);
  delay(30);

  Wire.begin();
  Serial.begin(115200);

  imuReady = (myIMU.begin() == 0);
  if (imuReady) {
    delay(80);
    gzBias = calibrateGzBias();
  } else {
    gzBias = 0.0f;
    motorBrake();
  }

  refreshStoredPacketCopiesFromRuntime();
  loadPersistentConfig(false);
  loadDeviceName();

  // v3.11: BLE.begin() can fail on first call if the radio init races with
  // power-up. Retry once after a short delay before giving up. If it still
  // fails (e.g. RAM exhausted by static buffers), the device will be
  // invisible to scanners — Serial logs help diagnose at production time.
  bool bleOk = BLE.begin();
  if (!bleOk) {
    delay(100);
    bleOk = BLE.begin();
  }
  if (bleOk) {
    bleReady = true;
    BLE.setLocalName(deviceName);
    BLE.setDeviceName(deviceName);

    configService.addCharacteristic(commandChar);
    configService.addCharacteristic(statusChar);
    configService.addCharacteristic(currentSegChar);
    configService.addCharacteristic(speedChar);
    configService.addCharacteristic(paramsChar);
    configService.addCharacteristic(rulesChar);
    configService.addCharacteristic(bulkChar);
    configService.addCharacteristic(telemetryChar);
    configService.addCharacteristic(adaptiveBrakeChar);
    configService.addCharacteristic(firmwareInfoChar);
    configService.addCharacteristic(deviceNameChar);
    BLE.addService(configService);
    BLE.setAdvertisedService(configService);

    commandChar.setEventHandler(BLEWritten, onCommandWritten);
    speedChar.setEventHandler(BLEWritten, onSpeedWritten);
    paramsChar.setEventHandler(BLEWritten, onParamsWritten);
    rulesChar.setEventHandler(BLEWritten, onRulesWritten);
    adaptiveBrakeChar.setEventHandler(BLEWritten, onAdaptiveBrakeWritten);
    deviceNameChar.setEventHandler(BLEWritten, onDeviceNameWritten);
    BLE.setEventHandler(BLEConnected, onConnected);
    BLE.setEventHandler(BLEDisconnected, onDisconnected);

    commandChar.writeValue((uint8_t)0);
    statusChar.writeValue(statusValue);
    { uint8_t p[13] = {0}; p[0] = SEG_NONE; currentSegChar.writeValue(p, 13); }
    speedChar.writeValue(NORMAL_DUTY);
    writeParamsToChar();
    rulesChar.writeValue(lastRulesPacket, lastRulesPacketLen);
    uint8_t emptyBulk[3] = {0xFF, 0xFF, 0};
    bulkChar.writeValue(emptyBulk, 3);
    writeTelemetryToChar(true);
    writeAdaptiveBrakeToChar();
    firmwareInfoChar.writeValue((const uint8_t*)FW_VERSION, strlen(FW_VERSION));
    writeDeviceNameToChar();

    BLE.advertise();
  }

  uint32_t now = millis();
  lastControlMs = now;
  lastBlePollMs = now;
  lastTelemetryMs = now;
  if (imuReady) {
    setStatus(STATUS_IDLE);
    if (detectUpsideDownAtBoot()) {
      armStandaloneBootMode(now);
    }
  } else {
    motorBrake();
    setStatus(STATUS_ERROR);
  }
}

void loop() {
  uint32_t now = millis();

  // v3.42: absolute priority for time-based actions.
  // This runs before BLE.poll(), before IMU reads, and even outside the 5 ms
  // control tick. Therefore a WAIT that expires at 2691 ms can start BRAKE
  // immediately even if BLE or IMU work delayed the next control sample.
  if (isRunning) {
    updateActionEngine(now);
  }

  // 1. Local realtime control first.
  if ((uint32_t)(now - lastControlMs) >= CONTROL_DT_MS) {
    const uint32_t controlLagMs = (uint32_t)(now - lastControlMs);
    if (controlLagMs > CONTROL_SLIP_CLAMP_MS) lastControlMs = now;
    else lastControlMs += CONTROL_DT_MS;
    sampleControlVbat(now);
    logVoltageSample(now);
    trackWaitSagDuringActiveWait(now);
    trackBrakeVbatDuringActiveBrake(now);

    if (isRunning) {
      if (!actionActive && isRunning) {
        // Soft start at run launch: ramp from START_DUTY to NORMAL_DUTY.
        // START_DUTY is clamped to NORMAL_DUTY, so lowering normal duty no longer
        // produces a momentary 100% launch.
        motorDuty(startupDutyFor(now));
      }

      if (!imuReady) {
        stopRun(STATUS_ERROR);
        return;
      }
      float gzRaw = myIMU.readFloatGyroZ();
      if (!isnan(gzRaw)) {
        float gz = updateGzFilter(gzRaw - gzBias);

        updateSegmentDetection(gz, now);
        // v3.9: drive any rule sitting on a wildcard slot whose timer has
        // elapsed. Cheap: O(rulesCount) per tick.
        tickRuleWildcards(gz, now);
        // v39: finish pending rule confirmations whose verification window elapsed.
        tickRuleConfirmations(gz, now);
      }
    } else {
      motorBrake();
      updateStandaloneAutostart(now);
    }
  }

  // v3.42: If the control tick took time and an action boundary passed during
  // it, advance once more before any BLE operation can process a STOP command.
  now = millis();
  if (isRunning) {
    updateActionEngine(now);
  }

  // 2. BLE is lower priority and never required for control.
  // During RUNNING, BLE is polled only to receive STOP/config writes, sparse
  // voltage telemetry, and tiny SEG_LAP packets. Full live segment/debug logs
  // are still disabled; the stopped ACK dump remains the detailed record.
  now = millis();
  const uint32_t bleInterval = isRunning ? BLE_POLL_RUNNING_MS : BLE_POLL_MS;
  if (bleReady && (uint32_t)(now - lastBlePollMs) >= bleInterval) {
    lastBlePollMs = now;
    BLE.poll();

    if (isRunning) {
      // v3.51: BLE is deferred and budgeted. A running BLE service pass may
      // send at most one small application payload. Prioritize completed lap
      // events over voltage telemetry; telemetry will be sent on a later pass.
      uint8_t payloadsSent = 0;
      if (liveSegCount > 0 && payloadsSent < RUNNING_BLE_PAYLOAD_BUDGET_PER_POLL) {
        const bool allowFullLiveDebug = LIVE_SEG_NOTIFY_DURING_RUN && DEBUG_LIVE_SEG;
        LiveSegPacket ev;
        while (popLiveSegment(ev)) {
          if (!allowFullLiveDebug && ev.seg != SEG_LAP) continue;
          uint8_t p[13] = {0};
          p[0] = ev.seg;
          writeU32LE(&p[1], ev.elapsedMs);
          writeU16LE(&p[5], ev.durMs);
          writeI16LE(&p[7], ev.vbatMv);
          writeI16LE(&p[9], ev.waitSagMv);
          writeI16LE(&p[11], ev.brakeVbatMv);
          currentSegChar.writeValue(p, 13);
          payloadsSent++;
          lastLiveEventMs = now;
          break;
        }
      }
      if (RUNNING_HEADER_TELEMETRY_BLE && payloadsSent < RUNNING_BLE_PAYLOAD_BUDGET_PER_POLL) {
        // v3.54: header status only. Web updates VBAT / board temperature, but does
        // not append these sparse running packets to the graph timeline.
        writeTelemetryToChar(false, RUNNING_TELEMETRY_MS);
      }
    } else {
      if (!dumpActive) writeTelemetryToChar(false);
      dumpStep();
    }
  }

  // Small LED heartbeat. Avoid Serial logging in production.
  if (isRunning) {
    ledSet((millis() / 100) % 2 == 0);
  } else if (bleConnected) {
    ledSet((millis() / 500) % 2 == 0);
  } else {
    ledSet(false);
  }
}

#include "stepper.h"

#include <driver/gpio.h>
#include <driver/gptimer.h>

#include "config.h"

namespace {

gptimer_handle_t timer = nullptr;
bool timerRunning = false;  // hardware timer started; loop() side only
bool enabled = false;

/* Shared with the ISR. All 32-bit, so each read and write is atomic on this
 * core; none of them is a pair that has to be consistent with another. */
volatile int32_t pos = 0;
volatile int32_t tgt = 0;
volatile int32_t dirSign = 1;
volatile uint32_t stepRate = 0;   // steps per second
volatile bool active = false;     // ISR is producing pulses
volatile bool stepLevel = false;  // current STEP pin level

inline uint32_t halfPeriodUs(uint32_t r) {
  if (r < 1) r = 1;
  return STEP_TIMER_HZ / (2u * r);
}

/* One alarm = one toggle of STEP. A step is counted on the rising edge, which
 * is the edge the A4988 acts on. The alarm auto-reloads to zero, so setting a
 * new alarm_count here sets the NEXT half-period -- that is how the rate is
 * changed mid-move without stopping. */
bool IRAM_ATTR onAlarm(gptimer_handle_t t, const gptimer_alarm_event_data_t *,
                       void *) {
  if (!active) return false;

  if (!stepLevel) {
    // Rising edge: the driver takes a step now.
    gpio_set_level((gpio_num_t)PIN_STEP, 1);
    stepLevel = true;
    pos = pos + dirSign;
    if (pos == tgt) {
      /* Arrived. Bring STEP back low on the very next alarm and go quiet;
       * service() will stop the timer from task context. */
      active = false;
      gpio_set_level((gpio_num_t)PIN_STEP, 0);
      stepLevel = false;
      return false;
    }
  } else {
    gpio_set_level((gpio_num_t)PIN_STEP, 0);
    stepLevel = false;
  }

  gptimer_alarm_config_t a = {};
  a.alarm_count = halfPeriodUs(stepRate);
  a.reload_count = 0;
  a.flags.auto_reload_on_alarm = true;
  gptimer_set_alarm_action(t, &a);
  return false;
}

}  // namespace

namespace stepper {

void begin() {
  pinMode(PIN_STEP, OUTPUT);
  pinMode(PIN_DIR, OUTPUT);
  pinMode(PIN_MS1, OUTPUT);
  pinMode(PIN_MS2, OUTPUT);
  pinMode(PIN_MS3, OUTPUT);
  pinMode(PIN_ENABLE, OUTPUT);

  // 1/8 step: MS1=H, MS2=H, MS3=L (A4988 datasheet table). Set before enable.
  digitalWrite(PIN_MS1, HIGH);
  digitalWrite(PIN_MS2, HIGH);
  digitalWrite(PIN_MS3, LOW);
  digitalWrite(PIN_STEP, LOW);
  digitalWrite(PIN_DIR, DIR_TOWARD_HOME);
  dirSign = -1;

  enableDriver(false);  // rail.cpp enables once it has decided the boot state

  gptimer_config_t cfg = {};
  cfg.clk_src = GPTIMER_CLK_SRC_DEFAULT;
  cfg.direction = GPTIMER_COUNT_UP;
  cfg.resolution_hz = STEP_TIMER_HZ;
  ESP_ERROR_CHECK(gptimer_new_timer(&cfg, &timer));

  gptimer_event_callbacks_t cbs = {};
  cbs.on_alarm = onAlarm;
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(timer, &cbs, nullptr));
  ESP_ERROR_CHECK(gptimer_enable(timer));

  Serial.printf("[STEP] timer up at %lu Hz, 1/%d step, %.0f steps/mm\n",
                (unsigned long)STEP_TIMER_HZ, MICROSTEP_DIVISOR, STEPS_PER_MM);
}

void setDirection(int sign) {
  if (active) return;  // never mid-move; rail.cpp does not ask
  dirSign = (sign < 0) ? -1 : 1;
  digitalWrite(PIN_DIR, dirSign < 0 ? DIR_TOWARD_HOME : DIR_TOWARD_IDLE);
}

int direction() { return dirSign; }

void start(int32_t target, uint32_t rateStepsPerSec) {
  if (active) return;
  if (target == pos) return;  // already there; nothing to pulse

  tgt = target;
  stepRate = rateStepsPerSec < 1 ? 1 : rateStepsPerSec;
  stepLevel = false;
  digitalWrite(PIN_STEP, LOW);

  gptimer_alarm_config_t a = {};
  a.alarm_count = halfPeriodUs(stepRate);
  a.reload_count = 0;
  a.flags.auto_reload_on_alarm = true;
  gptimer_set_raw_count(timer, 0);
  gptimer_set_alarm_action(timer, &a);

  active = true;
  if (!timerRunning) {
    gptimer_start(timer);
    timerRunning = true;
  }
}

void setRate(uint32_t rateStepsPerSec) {
  stepRate = rateStepsPerSec < 1 ? 1 : rateStepsPerSec;
}

uint32_t rate() { return stepRate; }

void halt() {
  active = false;
  if (timerRunning) {
    gptimer_stop(timer);
    timerRunning = false;
  }
  digitalWrite(PIN_STEP, LOW);
  stepLevel = false;
}

bool running() { return active; }
int32_t position() { return pos; }

void setPosition(int32_t p) {
  if (active) return;
  pos = p;
}

int32_t target() { return tgt; }

void service() {
  if (!active && timerRunning) {
    gptimer_stop(timer);
    timerRunning = false;
  }
}

void enableDriver(bool on) {
  enabled = on;
  digitalWrite(PIN_ENABLE, on ? LOW : HIGH);  // active LOW
}

bool driverEnabled() { return enabled; }

}  // namespace stepper

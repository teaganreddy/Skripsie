#include "netlog.h"

#include <stdarg.h>

#include "link_protocol.h"

namespace {

const int QUEUE = 8;
char lines[QUEUE][sizeof(LogMsg::text)];
volatile int head = 0, count = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

}  // namespace

namespace netlog {

void printf(const char *fmt, ...) {
  char buf[sizeof(LogMsg::text)];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  /* Not printed here. This is called from the ESP-NOW receive callback, and
   * a USB-CDC write with no host attached can block for its timeout; the
   * service task prints each line as it takes it from the queue. */

  portENTER_CRITICAL(&mux);
  if (count == QUEUE) {  // overflow: drop the oldest, the newest is the useful one
    head = (head + 1) % QUEUE;
    count--;
  }
  const int slot = (head + count) % QUEUE;
  strncpy(lines[slot], buf, sizeof(lines[slot]) - 1);
  lines[slot][sizeof(lines[slot]) - 1] = '\0';
  count++;
  portEXIT_CRITICAL(&mux);
}

bool take(char *out, size_t len) {
  portENTER_CRITICAL(&mux);
  if (count == 0) {
    portEXIT_CRITICAL(&mux);
    return false;
  }
  strncpy(out, lines[head], len - 1);
  out[len - 1] = '\0';
  head = (head + 1) % QUEUE;
  count--;
  portEXIT_CRITICAL(&mux);
  return true;
}

}  // namespace netlog

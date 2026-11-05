// Dolby Atmos Renderer head-tracker plugin for the Audeze Maxwell 2 ANC.
//
// The Maxwell 2 ANC fuses its IMU on the headset and streams a unit quaternion in
// HID input report 0x0A (4 little-endian floats) once sensors are enabled with
// vendor output report 0x06. This plugin exposes the C API the Renderer loads
// from ~/Library/Application Support/Dolby/Dolby Atmos Renderer/Head Tracking.

#include <CoreFoundation/CoreFoundation.h>
#include <IOKit/hid/IOHIDManager.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define AUDEZE_VID      0x3329
#define VENDOR_PAGE     0xFF13
#define REPORT_ENABLE   0x06
#define REPORT_QUAT     0x0A
#define SENSOR_RATE     0x3C

#define PID_HEADSET     0x4BAA   // Maxwell 2 ANC connected by USB cable
#define PID_DONGLE      0x4BA9   // Maxwell USB dongle (wireless)

static const int kProductIds[] = { PID_HEADSET, PID_DONGLE };

enum { ERR_OK = 0, ERR_NO_DEVICE = 1, ERR_NO_DATA = 2, ERR_INTERNAL = 3 };

typedef struct { const char *name; const char *developer; } headtracker_description;

// Maps a vector in the headset's sensor frame to the Renderer's frame
// (x forward, y left, z up). Calibration showed the headset frame already matches.
static const float kAxisMap[3][3] = { {1, 0, 0}, {0, 1, 0}, {0, 0, 1} };

typedef struct {
  atomic_bool stop;
  bool running;          // headtracker_run is executing; guarded by lock
  pthread_cond_t stopped;
  pthread_mutex_t lock;
  IOHIDDeviceRef device;
  uint8_t report_buf[64];
  float q[4];            // latest (w, x, y, z) in sensor frame
  float q_zero[4];       // zero reference, conjugated
  bool have_sample, have_zero, zero_requested;
  double last_sample;
  atomic_int error;
  atomic_int samples, rot_calls, rot_ok, zero_calls;  // diagnostics
} tracker;

static double now_s(void) { return CFAbsoluteTimeGetCurrent(); }

// Diagnostic log at ~/Library/Logs/Maxwell2HT.log.
static void dbg(const void *inst, const char *fmt, ...) {
  static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;
  const char *home = getenv("HOME");
  if (!home) return;
  char path[1024];
  snprintf(path, sizeof path, "%s/Library/Logs/Maxwell2HT.log", home);
  pthread_mutex_lock(&log_lock);
  FILE *f = fopen(path, "a");
  if (f) {
    CFAbsoluteTime t = CFAbsoluteTimeGetCurrent();
    fprintf(f, "%.3f [%p] ", t, inst);
    va_list ap; va_start(ap, fmt); vfprintf(f, fmt, ap); va_end(ap);
    fputc('\n', f);
    fclose(f);
  }
  pthread_mutex_unlock(&log_lock);
}

static void quat_mul(const float a[4], const float b[4], float r[4]) {
  r[0] = a[0]*b[0] - a[1]*b[1] - a[2]*b[2] - a[3]*b[3];
  r[1] = a[0]*b[1] + a[1]*b[0] + a[2]*b[3] - a[3]*b[2];
  r[2] = a[0]*b[2] - a[1]*b[3] + a[2]*b[0] + a[3]*b[1];
  r[3] = a[0]*b[3] + a[1]*b[2] - a[2]*b[1] + a[3]*b[0];
}

// Commands sent through the USB dongle need the 0x80 routing flag to reach the headset;
// without it the dongle acknowledges them but the headset's sensor stream is unaffected.
static uint8_t route_flag(IOHIDDeviceRef dev) {
  int pid = 0;
  CFNumberRef n = IOHIDDeviceGetProperty(dev, CFSTR(kIOHIDProductIDKey));
  if (n) CFNumberGetValue(n, kCFNumberIntType, &pid);
  return pid == PID_DONGLE ? 0x80 : 0x00;
}

static void send_enable(IOHIDDeviceRef dev, bool on) {
  uint8_t cmd[62] = { REPORT_ENABLE, 0x09, route_flag(dev), 0x05, 0x5A, 0x05, 0x00, 0x00, 0x09, 0x2E, 0x00,
                      (uint8_t)(SENSOR_RATE | (on ? 0x80 : 0x00)) };
  IOHIDDeviceSetReport(dev, kIOHIDReportTypeOutput, REPORT_ENABLE, cmd, sizeof cmd);
}

static void on_report(void *ctx, IOReturn res, void *sender, IOHIDReportType type,
                      uint32_t id, uint8_t *report, CFIndex len) {
  tracker *t = ctx;
  if (res != kIOReturnSuccess || id != REPORT_QUAT || len < 17 || report[0] != REPORT_QUAT) return;
  float q[4];
  memcpy(q, report + 1, sizeof q);
  float n = sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
  if (!(n > 0.5f && n < 1.5f)) return;
  for (int i = 0; i < 4; i++) q[i] /= n;

  pthread_mutex_lock(&t->lock);
  memcpy(t->q, q, sizeof q);
  t->have_sample = true;
  t->last_sample = now_s();
  if (!t->have_zero || t->zero_requested) {
    t->q_zero[0] = q[0]; t->q_zero[1] = -q[1]; t->q_zero[2] = -q[2]; t->q_zero[3] = -q[3];
    t->have_zero = true;
    t->zero_requested = false;
  }
  pthread_mutex_unlock(&t->lock);
  atomic_store(&t->error, ERR_OK);
  atomic_fetch_add(&t->samples, 1);
}

static void attach(tracker *t, IOHIDDeviceRef dev) {
  if (t->device) return;
  IOReturn r = IOHIDDeviceOpen(dev, kIOHIDOptionsTypeNone);
  dbg(t, "device matched (route flag 0x%02x), IOHIDDeviceOpen -> 0x%x", route_flag(dev), r);
  if (r != kIOReturnSuccess) return;
  CFRetain(dev);
  t->device = dev;
  IOHIDDeviceRegisterInputReportCallback(dev, t->report_buf, sizeof t->report_buf, on_report, t);
  send_enable(dev, true);
}

static void on_match(void *ctx, IOReturn res, void *sender, IOHIDDeviceRef dev) { attach(ctx, dev); }

static void on_remove(void *ctx, IOReturn res, void *sender, IOHIDDeviceRef dev) {
  tracker *t = ctx;
  if (t->device != dev) return;
  dbg(t, "device removed");
  CFRelease(t->device);
  t->device = NULL;
  pthread_mutex_lock(&t->lock);
  t->have_sample = false;
  pthread_mutex_unlock(&t->lock);
  atomic_store(&t->error, ERR_NO_DEVICE);
}

static CFDictionaryRef matching_dict(int pid) {
  int vals[] = { AUDEZE_VID, pid, VENDOR_PAGE, 1 };
  CFStringRef keys[] = { CFSTR(kIOHIDVendorIDKey), CFSTR(kIOHIDProductIDKey),
                         CFSTR(kIOHIDDeviceUsagePageKey), CFSTR(kIOHIDDeviceUsageKey) };
  CFNumberRef nums[4];
  for (int i = 0; i < 4; i++) nums[i] = CFNumberCreate(NULL, kCFNumberIntType, &vals[i]);
  CFDictionaryRef d = CFDictionaryCreate(NULL, (const void **)keys, (const void **)nums, 4,
                                         &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);
  for (int i = 0; i < 4; i++) CFRelease(nums[i]);
  return d;
}

// Runs the HID session on the calling thread until headtracker_cancel sets `stop`.
static void track(tracker *t) {
  CFRunLoopRef runloop = CFRunLoopGetCurrent();
  IOHIDManagerRef mgr = IOHIDManagerCreate(NULL, kIOHIDOptionsTypeNone);
  int count = sizeof kProductIds / sizeof *kProductIds;
  CFMutableArrayRef matches = CFArrayCreateMutable(NULL, count, &kCFTypeArrayCallBacks);
  for (int i = 0; i < count; i++) {
    CFDictionaryRef d = matching_dict(kProductIds[i]);
    CFArrayAppendValue(matches, d);
    CFRelease(d);
  }
  IOHIDManagerSetDeviceMatchingMultiple(mgr, matches);
  CFRelease(matches);
  IOHIDManagerRegisterDeviceMatchingCallback(mgr, on_match, t);
  IOHIDManagerRegisterDeviceRemovalCallback(mgr, on_remove, t);
  IOHIDManagerScheduleWithRunLoop(mgr, runloop, kCFRunLoopDefaultMode);
  dbg(t, "IOHIDManagerOpen -> 0x%x", IOHIDManagerOpen(mgr, kIOHIDOptionsTypeNone));

  double last_kick = 0, last_stats = now_s();
  while (!atomic_load(&t->stop)) {
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.25, false);
    if (now_s() - last_stats >= 5) {
      last_stats = now_s();
      pthread_mutex_lock(&t->lock);
      float q[4]; memcpy(q, t->q, sizeof q);
      pthread_mutex_unlock(&t->lock);
      dbg(t, "stats: device=%s samples=%d rot_calls=%d rot_ok=%d zero_calls=%d q=(%.3f %.3f %.3f %.3f)",
          t->device ? "yes" : "no", atomic_exchange(&t->samples, 0), atomic_exchange(&t->rot_calls, 0),
          atomic_exchange(&t->rot_ok, 0), atomic_load(&t->zero_calls), q[0], q[1], q[2], q[3]);
    }
    if (!t->device) continue;
    // The headset stops streaming after power cycles or dongle re-pairing; re-enable when quiet.
    pthread_mutex_lock(&t->lock);
    double quiet = now_s() - t->last_sample;
    pthread_mutex_unlock(&t->lock);
    if (quiet > 1.0 && now_s() - last_kick > 1.0) {
      send_enable(t->device, true);
      last_kick = now_s();
      atomic_store(&t->error, ERR_NO_DATA);
    }
  }

  if (t->device) {
    send_enable(t->device, false);
    IOHIDDeviceRegisterInputReportCallback(t->device, t->report_buf, sizeof t->report_buf, NULL, NULL);
    CFRelease(t->device);
    t->device = NULL;
  }
  IOHIDManagerUnscheduleFromRunLoop(mgr, runloop, kCFRunLoopDefaultMode);
  IOHIDManagerClose(mgr, kIOHIDOptionsTypeNone);
  CFRelease(mgr);
}

__attribute__((visibility("default")))
void headtracker_describe(headtracker_description *d) {
  dbg(NULL, "describe");
  d->name = "Audeze Maxwell 2 head tracker";
  d->developer = "Community plugin (Maxwell 2 HID report 0x0A)";
}

__attribute__((visibility("default")))
void *headtracker_create(void) {
  tracker *t = calloc(1, sizeof *t);
  if (!t) return NULL;
  pthread_mutex_init(&t->lock, NULL);
  pthread_cond_init(&t->stopped, NULL);
  t->q[0] = 1;
  atomic_store(&t->error, ERR_NO_DEVICE);
  dbg(t, "create");
  return t;
}

// Like Dolby's OSC tracker, run blocks on the Renderer's tracking thread until cancel.
__attribute__((visibility("default")))
void headtracker_run(void *h) {
  tracker *t = h;
  if (!t) return;
  pthread_mutex_lock(&t->lock);
  bool busy = t->running;
  t->running = true;
  pthread_mutex_unlock(&t->lock);
  if (busy) return;

  dbg(t, "run: enter");
  atomic_store(&t->stop, false);
  track(t);
  dbg(t, "run: exit");

  pthread_mutex_lock(&t->lock);
  t->running = false;
  pthread_cond_broadcast(&t->stopped);
  pthread_mutex_unlock(&t->lock);
}

__attribute__((visibility("default")))
void headtracker_cancel(void *h) {
  tracker *t = h;
  if (!t) return;
  dbg(t, "cancel");
  atomic_store(&t->stop, true);
  // Wait (bounded) for run to finish so destroy never frees a tracker that is still in use.
  struct timespec deadline;
  clock_gettime(CLOCK_REALTIME, &deadline);
  deadline.tv_sec += 3;
  pthread_mutex_lock(&t->lock);
  while (t->running && pthread_cond_timedwait(&t->stopped, &t->lock, &deadline) == 0) {}
  pthread_mutex_unlock(&t->lock);
}

__attribute__((visibility("default")))
void headtracker_destroy(void *h) {
  tracker *t = h;
  if (!t) return;
  dbg(t, "destroy");
  headtracker_cancel(t);
  pthread_mutex_lock(&t->lock);
  bool still_running = t->running;
  pthread_mutex_unlock(&t->lock);
  if (still_running) return;  // leak rather than free memory run() still uses
  pthread_cond_destroy(&t->stopped);
  pthread_mutex_destroy(&t->lock);
  free(t);
}

__attribute__((visibility("default")))
void headtracker_set_zero_reference(void *h) {
  tracker *t = h;
  if (!t) return;
  if (atomic_fetch_add(&t->zero_calls, 1) < 5) dbg(t, "set_zero_reference");
  pthread_mutex_lock(&t->lock);
  t->zero_requested = true;
  pthread_mutex_unlock(&t->lock);
}

__attribute__((visibility("default")))
int headtracker_get_rotation_matrix(void *h, float *m) {
  tracker *t = h;
  if (!t) return 0;
  if (atomic_fetch_add(&t->rot_calls, 1) == 0) dbg(t, "get_rotation_matrix: first call");
  float q[4], qz[4];
  pthread_mutex_lock(&t->lock);
  bool ok = t->have_sample && t->have_zero;
  memcpy(q, t->q, sizeof q);
  memcpy(qz, t->q_zero, sizeof qz);
  pthread_mutex_unlock(&t->lock);
  if (!ok) {
    static const float identity[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
    memcpy(m, identity, sizeof identity);
    return 0;
  }

  // Rotation relative to the zero reference, in the headset frame.
  float r[4];
  quat_mul(qz, q, r);
  float w = r[0], x = r[1], y = r[2], z = r[3];
  float s[3][3] = {
    { 1 - 2*(y*y + z*z), 2*(x*y - w*z),     2*(x*z + w*y) },
    { 2*(x*y + w*z),     1 - 2*(x*x + z*z), 2*(y*z - w*x) },
    { 2*(x*z - w*y),     2*(y*z + w*x),     1 - 2*(x*x + y*y) },
  };
  // Change of basis into the Renderer frame: R = A * S * A^T.
  float as[3][3] = {{0}};
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) for (int k = 0; k < 3; k++)
    as[i][j] += kAxisMap[i][k] * s[k][j];
  for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) {
    float v = 0;
    for (int k = 0; k < 3; k++) v += as[i][k] * kAxisMap[j][k];
    m[i * 3 + j] = v;
  }
  atomic_fetch_add(&t->rot_ok, 1);
  return 1;
}

__attribute__((visibility("default")))
int headtracker_get_error(void *h) {
  tracker *t = h;
  // The Renderer does not call run() for a tracker whose error is non-zero right after create,
  // and the headset is only opened inside run(). So connection state is kept internal (see the
  // diagnostic log) and only a missing tracker is reported, like Audeze's original plugin.
  static atomic_int calls;
  if (atomic_fetch_add(&calls, 1) < 20)
    dbg(t, "get_error (internal status %d)", t ? atomic_load(&t->error) : -1);
  return t ? ERR_OK : ERR_INTERNAL;
}

__attribute__((visibility("default")))
const char *headtracker_get_error_string(void *h, int code) {
  dbg(h, "get_error_string(%d)", code);
  switch (code) {
    case ERR_OK: return "ok";
    case ERR_NO_DEVICE: return "Audeze Maxwell 2 not connected";
    case ERR_NO_DATA: return "no head-tracking data from headset";
    case ERR_INTERNAL: return "internal error";
    default: return "unknown error code";
  }
}

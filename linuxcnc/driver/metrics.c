// metrics.c — sliding-window statistics and histograms for electrophorus
// This file is included directly by electrophorus.c (halcompile single TU).

#include <limits.h>
#include <math.h>
#include <stdint.h>
#include <string.h>
#include <time.h>

// forward decls from electrophorus.c
// - state_t structure is defined before including this file
// - pin_err helper is declared before include
extern int comp_id;
extern const char *modname;
extern const char *prefix;
static int metrics_export_pins(int comp_id, const char *prefix, state_t *state);

// Window bounds
#define METRIC_MIN_WINDOW 16
#define METRIC_MAX_WINDOW 1024

// Servo timing (ns) histogram centered around requested period
#define SERVO_RANGE_NS 250000                                              // +/- 250 us around period
#define SERVO_BIN_WIDTH_NS 5000                                            // 5 us bins
#define SERVO_BIN_COUNT (((SERVO_RANGE_NS * 2) / SERVO_BIN_WIDTH_NS) + 2)  // +under/+over

static int servo_win_size = 256;
static int servo_count = 0;
static int servo_head = 0;  // next insert index
static int32_t servo_ring[METRIC_MAX_WINDOW];
static int64_t servo_sum = 0;        // sum of samples
static uint64_t servo_sumsq = 0;     // sum of squares
static int64_t servo_center_ns = 0;  // last l_period_ns used to center histogram
static uint32_t servo_hist[SERVO_BIN_COUNT];
static uint64_t servo_prev_ts_ns = 0;  // previous tick timestamp

static inline int servo_bin_index(const int32_t sample_ns) {
  if (servo_center_ns == 0) return 0;  // until initialized, dump into underflow
  int64_t delta = (int64_t)sample_ns - servo_center_ns;
  if (delta < -(int64_t)SERVO_RANGE_NS) return 0;                   // underflow
  if (delta > (int64_t)SERVO_RANGE_NS) return SERVO_BIN_COUNT - 1;  // overflow
  int64_t shifted = delta + SERVO_RANGE_NS;                         // [0 .. 2*RANGE]
  int idx = (int)(shifted / SERVO_BIN_WIDTH_NS) + 1;                // +1 to reserve 0 for underflow
  if (idx < 1) idx = 1;
  if (idx > SERVO_BIN_COUNT - 2) idx = SERVO_BIN_COUNT - 2;
  return idx;
}

static inline void servo_reset_all_internal(void) {
  servo_count = 0;
  servo_head = 0;
  servo_sum = 0;
  servo_sumsq = 0;
  servo_prev_ts_ns = 0;
  memset(servo_ring, 0, sizeof(servo_ring));
  memset(servo_hist, 0, sizeof(servo_hist));
}

static inline void servo_set_window_internal(int n) {
  if (n < METRIC_MIN_WINDOW) n = METRIC_MIN_WINDOW;
  if (n > METRIC_MAX_WINDOW) n = METRIC_MAX_WINDOW;
  if (n != servo_win_size) {
    servo_win_size = n;
    servo_reset_all_internal();
  }
}

static inline void servo_push_internal(const int32_t sample_ns, const long l_period_ns) {
  if (servo_center_ns != (int64_t)l_period_ns) {
    servo_center_ns = l_period_ns;
    servo_reset_all_internal();
  }
  // eviction
  if (servo_count == servo_win_size) {
    int32_t old = servo_ring[servo_head];
    servo_sum -= old;
    servo_sumsq -= (uint64_t)((int64_t)old * (int64_t)old);
    int old_bin = servo_bin_index(old);
    if (servo_hist[old_bin] > 0) servo_hist[old_bin]--;
  } else {
    servo_count++;
  }
  // insert
  servo_ring[servo_head] = sample_ns;
  servo_sum += sample_ns;
  servo_sumsq += (uint64_t)((int64_t)sample_ns * (int64_t)sample_ns);
  servo_hist[servo_bin_index(sample_ns)]++;
  servo_head = (servo_head + 1) % servo_win_size;
}

static inline void servo_compute_minmax(int32_t *out_min, int32_t *out_max) {
  int32_t mn = INT32_MAX, mx = 0;
  for (int i = 0; i < servo_count; i++) {
    int idx = (servo_head - 1 - i);
    if (idx < 0) idx += servo_win_size;
    int32_t v = servo_ring[idx];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  *out_min = (servo_count ? mn : 0);
  *out_max = (servo_count ? mx : 0);
}

static inline int32_t servo_percentile(double p) {
  if (servo_count == 0) return 0;
  uint32_t target = (uint32_t)ceil(p * servo_count);
  if (target < 1) target = 1;
  uint32_t acc = 0;
  for (int b = 0; b < SERVO_BIN_COUNT; ++b) {
    acc += servo_hist[b];
    if (acc >= target) {
      if (b == 0) return (int32_t)(servo_center_ns - SERVO_RANGE_NS);
      if (b == SERVO_BIN_COUNT - 1) return (int32_t)(servo_center_ns + SERVO_RANGE_NS);
      int64_t lower = servo_center_ns - SERVO_RANGE_NS + (int64_t)(b - 1) * SERVO_BIN_WIDTH_NS;
      return (int32_t)lower;  // lower edge of bin
    }
  }
  return (int32_t)servo_center_ns;
}

// UART RTT (us) histogram: 0..2000us with 10us bins
#define RTT_MAX_US 2000
#define RTT_BIN_WIDTH_US 10
#define RTT_BIN_COUNT ((RTT_MAX_US / RTT_BIN_WIDTH_US) + 2)  // +under/+over

static int rtt_win_size = 256;
static int rtt_count = 0;
static int rtt_head = 0;
static int32_t rtt_ring[METRIC_MAX_WINDOW];
static int64_t rtt_sum = 0;
static uint64_t rtt_sumsq = 0;
static uint32_t rtt_hist[RTT_BIN_COUNT];

static inline int rtt_bin_index(const int32_t us) {
  if (us < 0) return 0;
  if (us > RTT_MAX_US) return RTT_BIN_COUNT - 1;
  int idx = (us / RTT_BIN_WIDTH_US) + 1;
  if (idx < 1) idx = 1;
  if (idx > RTT_BIN_COUNT - 2) idx = RTT_BIN_COUNT - 2;
  return idx;
}

static inline void rtt_reset_all_internal(void) {
  rtt_count = 0;
  rtt_head = 0;
  rtt_sum = 0;
  rtt_sumsq = 0;
  memset(rtt_ring, 0, sizeof(rtt_ring));
  memset(rtt_hist, 0, sizeof(rtt_hist));
}

static inline void rtt_set_window_internal(int n) {
  if (n < METRIC_MIN_WINDOW) n = METRIC_MIN_WINDOW;
  if (n > METRIC_MAX_WINDOW) n = METRIC_MAX_WINDOW;
  if (n != rtt_win_size) {
    rtt_win_size = n;
    rtt_reset_all_internal();
  }
}

static inline void rtt_push_internal(const int32_t us) {
  if (rtt_count == rtt_win_size) {
    int32_t old = rtt_ring[rtt_head];
    rtt_sum -= old;
    rtt_sumsq -= (uint64_t)((int64_t)old * (int64_t)old);
    int ob = rtt_bin_index(old);
    if (rtt_hist[ob] > 0) rtt_hist[ob]--;
  } else {
    rtt_count++;
  }
  rtt_ring[rtt_head] = us;
  rtt_sum += us;
  rtt_sumsq += (uint64_t)((int64_t)us * (int64_t)us);
  rtt_hist[rtt_bin_index(us)]++;
  rtt_head = (rtt_head + 1) % rtt_win_size;
}

static inline void rtt_compute_minmax(int32_t *out_min, int32_t *out_max) {
  int32_t mn = INT32_MAX, mx = 0;
  for (int i = 0; i < rtt_count; i++) {
    int idx = (rtt_head - 1 - i);
    if (idx < 0) idx += rtt_win_size;
    int32_t v = rtt_ring[idx];
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  *out_min = (rtt_count ? mn : 0);
  *out_max = (rtt_count ? mx : 0);
}

static inline int32_t rtt_percentile(double p) {
  if (rtt_count == 0) return 0;
  uint32_t target = (uint32_t)ceil(p * rtt_count);
  if (target < 1) target = 1;
  uint32_t acc = 0;
  for (int b = 0; b < RTT_BIN_COUNT; ++b) {
    acc += rtt_hist[b];
    if (acc >= target) {
      if (b == 0) return 0;
      if (b == RTT_BIN_COUNT - 1) return RTT_MAX_US;
      int32_t lower = (int32_t)((b - 1) * RTT_BIN_WIDTH_US);
      return lower;
    }
  }
  return 0;
}

static inline void metrics_apply_window_size(state_t *state) {
  int ws = *state->metrics_window_size;
  if (ws <= 0) ws = 256;
  if (ws < METRIC_MIN_WINDOW) ws = METRIC_MIN_WINDOW;
  if (ws > METRIC_MAX_WINDOW) ws = METRIC_MAX_WINDOW;
  servo_set_window_internal(ws);
  rtt_set_window_internal(ws);
}

static inline void metrics_update_servo_outputs(state_t *state) {
  if (servo_count <= 0) {
    *state->servo_ns_min = 0;
    *state->servo_ns_max = 0;
    *state->servo_ns_mean = 0;
    *state->servo_ns_stddev = 0;
    *state->servo_ns_p50 = 0;
    *state->servo_ns_p95 = 0;
    *state->servo_ns_p99 = 0;
    *state->servo_ns_p99_9 = 0;
    return;
  }
  int32_t mn, mx;
  servo_compute_minmax(&mn, &mx);
  *state->servo_ns_min = mn;
  *state->servo_ns_max = mx;
  int32_t mean = (int32_t)(servo_sum / (servo_count ? servo_count : 1));
  *state->servo_ns_mean = mean;
  double ex2 = (double)servo_sumsq / (double)(servo_count ? servo_count : 1);
  double var = ex2 - ((double)mean * (double)mean);
  if (var < 0.0) var = 0.0;
  *state->servo_ns_stddev = (int32_t)(sqrt(var) + 0.5);
  *state->servo_ns_p50 = servo_percentile(0.50);
  *state->servo_ns_p95 = servo_percentile(0.95);
  *state->servo_ns_p99 = servo_percentile(0.99);
  *state->servo_ns_p99_9 = servo_percentile(0.999);
}

static inline void metrics_update_rtt_outputs(state_t *state) {
  if (rtt_count <= 0) {
    *state->rtt_us_min = 0;
    *state->rtt_us_max = 0;
    *state->rtt_us_mean = 0;
    *state->rtt_us_stddev = 0;
    *state->rtt_us_p50 = 0;
    *state->rtt_us_p95 = 0;
    *state->rtt_us_p99 = 0;
    *state->rtt_us_p99_9 = 0;
    return;
  }
  int32_t mn, mx;
  rtt_compute_minmax(&mn, &mx);
  *state->rtt_us_min = mn;
  *state->rtt_us_max = mx;
  int32_t mean = (int32_t)(rtt_sum / (rtt_count ? rtt_count : 1));
  *state->rtt_us_mean = mean;
  double ex2 = (double)rtt_sumsq / (double)(rtt_count ? rtt_count : 1);
  double var = ex2 - ((double)mean * (double)mean);
  if (var < 0.0) var = 0.0;
  *state->rtt_us_stddev = (int32_t)(sqrt(var) + 0.5);
  *state->rtt_us_p50 = rtt_percentile(0.50);
  *state->rtt_us_p95 = rtt_percentile(0.95);
  *state->rtt_us_p99 = rtt_percentile(0.99);
  *state->rtt_us_p99_9 = rtt_percentile(0.999);
}

void metrics_reset_all(state_t *state) {
  servo_reset_all_internal();
  rtt_reset_all_internal();
  servo_center_ns = 0;
  servo_prev_ts_ns = 0;
  // zero outputs
  metrics_update_servo_outputs(state);
  metrics_update_rtt_outputs(state);
  *state->metrics_reset = 0;
}

void metrics_servo_tick(state_t *state, long l_period_ns) {
  metrics_apply_window_size(state);
  if (*state->metrics_reset) {
    metrics_reset_all(state);
    // do not compute a sample this tick; initialize prev time and return
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    servo_prev_ts_ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
    return;
  }
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
  uint64_t now_ns = (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
  if (servo_prev_ts_ns != 0) {
    uint64_t dt64 = now_ns - servo_prev_ts_ns;
    int32_t dt32 = (dt64 > (uint64_t)INT32_MAX) ? INT32_MAX : (int32_t)dt64;
    servo_push_internal(dt32, l_period_ns);
    metrics_update_servo_outputs(state);
  }
  servo_prev_ts_ns = now_ns;
}
static int metrics_export_pins(int comp_id, const char *prefix, state_t *state) {
  // control
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->metrics_reset, comp_id, "%s.metrics.reset", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_IN, &state->metrics_window_size, comp_id, "%s.metrics.window-size", prefix)))
    return -1;

  // servo timing stats (ns)
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_min, comp_id, "%s.metrics.servo-ns.min", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_max, comp_id, "%s.metrics.servo-ns.max", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_mean, comp_id, "%s.metrics.servo-ns.mean", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_stddev, comp_id, "%s.metrics.servo-ns.stddev", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_p50, comp_id, "%s.metrics.servo-ns.p50", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_p95, comp_id, "%s.metrics.servo-ns.p95", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_p99, comp_id, "%s.metrics.servo-ns.p99", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_ns_p99_9, comp_id, "%s.metrics.servo-ns.p99_9", prefix)))
    return -1;

  // UART RTT stats (us)
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_min, comp_id, "%s.metrics.uart-rtt-us.min", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_max, comp_id, "%s.metrics.uart-rtt-us.max", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_mean, comp_id, "%s.metrics.uart-rtt-us.mean", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_stddev, comp_id, "%s.metrics.uart-rtt-us.stddev", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_p50, comp_id, "%s.metrics.uart-rtt-us.p50", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_p95, comp_id, "%s.metrics.uart-rtt-us.p95", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_p99, comp_id, "%s.metrics.uart-rtt-us.p99", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_p99_9, comp_id, "%s.metrics.uart-rtt-us.p99_9", prefix)))
    return -1;

  // counters under metrics as well
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_frames_ok, comp_id, "%s.metrics.frames-ok", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_bad_header, comp_id, "%s.metrics.bad-header", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_read_errors, comp_id, "%s.metrics.read-errors", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_write_errors, comp_id, "%s.metrics.write-errors", prefix)))
    return -1;

  // initial values
  *state->metrics_reset = 0;
  *state->metrics_window_size = 256;

  *state->servo_ns_min = 0;
  *state->servo_ns_max = 0;
  *state->servo_ns_mean = 0;
  *state->servo_ns_stddev = 0;
  *state->servo_ns_p50 = 0;
  *state->servo_ns_p95 = 0;
  *state->servo_ns_p99 = 0;
  *state->servo_ns_p99_9 = 0;

  *state->rtt_us_min = 0;
  *state->rtt_us_max = 0;
  *state->rtt_us_mean = 0;
  *state->rtt_us_stddev = 0;
  *state->rtt_us_p50 = 0;
  *state->rtt_us_p95 = 0;
  *state->rtt_us_p99 = 0;
  *state->rtt_us_p99_9 = 0;

  *state->metric_frames_ok = 0;
  *state->metric_bad_header = 0;
  *state->metric_read_errors = 0;
  *state->metric_write_errors = 0;

  return 0;
}

void metrics_rtt_sample(state_t *state, int32_t rtt_us) {
  metrics_apply_window_size(state);
  rtt_push_internal(rtt_us);
  metrics_update_rtt_outputs(state);
}

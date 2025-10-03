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

// Window sizing is derived from servo period:
// - 1s window: ~ 1s / period
// - 1m window: ~ 60s / period

#define METRIC_1M_MAX_SAMPLES 120000
// forward decl for RTT 1s window setter used from servo side
static inline void rtt_set_1s_window_internal(int n);

// forward decl for AGE 1s window setter used from servo side
static inline void age_set_1s_window_internal(int n);

// Servo timing (us) histogram centered around requested period
#define SERVO_RANGE_US 500                                                 // +/- 500 us around period
#define SERVO_BIN_WIDTH_US 2                                               // 2 us bins
#define SERVO_BIN_COUNT (((SERVO_RANGE_US * 2) / SERVO_BIN_WIDTH_US) + 2)  // +under/+over

static int servo_win_size = 1000; // 1s @1kHz; auto-adjusted from period
static int servo_count = 0;
static int servo_head = 0;  // next insert index
static int32_t servo_ring[METRIC_1M_MAX_SAMPLES];    // microseconds
static int64_t servo_sum = 0;                        // sum of samples (us)
static uint64_t servo_sumsq = 0;                     // sum of squares (us^2)
static int64_t servo_center_us = 0;              // last l_period_ns converted to us
static uint32_t servo_hist[SERVO_BIN_COUNT];
static uint64_t servo_prev_ts_ns = 0;  // previous tick timestamp

// 1-minute sliding window for servo (us)

static int metric_1m_size = 60000; // default 60s @1kHz; adjusted from servo period
static int servo_1m_count = 0, servo_1m_head = 0;
static int32_t servo_1m_ring[METRIC_1M_MAX_SAMPLES];
static int64_t servo_1m_sum = 0; static uint64_t servo_1m_sumsq = 0;
static uint32_t servo_hist_1m[SERVO_BIN_COUNT];

static inline int servo_bin_index(const int32_t sample_us) {
  if (servo_center_us == 0) return 0;  // until initialized, dump into underflow
  int64_t delta = (int64_t)sample_us - servo_center_us;
  if (delta < -(int64_t)SERVO_RANGE_US) return 0;                   // underflow
  if (delta > (int64_t)SERVO_RANGE_US) return SERVO_BIN_COUNT - 1;  // overflow
  int64_t shifted = delta + SERVO_RANGE_US;                         // [0 .. 2*RANGE]
  int idx = (int)(shifted / SERVO_BIN_WIDTH_US) + 1;                // +1 to reserve 0 for underflow
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

  memset(servo_hist, 0, sizeof(servo_hist));
  // reset 1m sliding window
  servo_1m_count = 0;
  servo_1m_head = 0;
  servo_1m_sum = 0;
  servo_1m_sumsq = 0;

  memset(servo_hist_1m, 0, sizeof(servo_hist_1m));
}

static inline void servo_set_1s_window_internal(int n) {
  if (n <= 0) n = 1;
  if (n > METRIC_1M_MAX_SAMPLES) n = METRIC_1M_MAX_SAMPLES;
  if (n != servo_win_size) {
    // shrink logical window immediately by dropping oldest if needed
    if (servo_count > n) {
      int drop = servo_count - n;
      for (int i = 0; i < drop; ++i) {
        int oldest_idx = servo_head - servo_count;
        while (oldest_idx < 0) oldest_idx += METRIC_1M_MAX_SAMPLES;
        int32_t ov = servo_ring[oldest_idx];
        servo_sum -= ov;
        servo_sumsq -= (uint64_t)((int64_t)ov * (int64_t)ov);
        int ob = servo_bin_index(ov);
        if (servo_hist[ob] > 0) servo_hist[ob]--;
        servo_count--;
      }
    }
    servo_win_size = n;
    if (servo_head >= servo_win_size) servo_head %= servo_win_size;
  }
}

static inline void servo_push_internal(const int32_t sample_us, const long l_period_ns) {
  int64_t period_us = (int64_t)(l_period_ns / 1000);
  if (servo_center_us != period_us) {
    servo_center_us = period_us;
    servo_reset_all_internal();
  }
  // eviction for finite-window stats
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
  servo_ring[servo_head] = sample_us;
  servo_sum += sample_us;
  servo_sumsq += (uint64_t)((int64_t)sample_us * (int64_t)sample_us);
  int bin = servo_bin_index(sample_us);
  servo_hist[bin]++;
  servo_head = (servo_head + 1) % servo_win_size;

  // adjust window sizes from servo period
  /* period_us already computed above */
  int new_1s_size = (int)((1000000LL + period_us / 2) / (period_us ? period_us : 1));
  int new_1m_size = (int)((60000000LL + period_us / 2) / (period_us ? period_us : 1));
  if (new_1s_size > METRIC_1M_MAX_SAMPLES) new_1s_size = METRIC_1M_MAX_SAMPLES;
  if (new_1m_size > METRIC_1M_MAX_SAMPLES) new_1m_size = METRIC_1M_MAX_SAMPLES;
  if (new_1s_size <= 0) new_1s_size = 1;
  if (new_1m_size <= 0) new_1m_size = 1;

  servo_set_1s_window_internal(new_1s_size);
  rtt_set_1s_window_internal(new_1s_size);
  age_set_1s_window_internal(new_1s_size);

  if (new_1m_size != metric_1m_size) {
    // shrink logical window immediately by dropping oldest if needed
    if (servo_1m_count > new_1m_size) {
      int drop = servo_1m_count - new_1m_size;
      for (int i = 0; i < drop; ++i) {
        int oldest_idx = servo_1m_head - servo_1m_count;
        while (oldest_idx < 0) oldest_idx += METRIC_1M_MAX_SAMPLES;
        int32_t ov = servo_1m_ring[oldest_idx];
        servo_1m_sum -= ov;
        servo_1m_sumsq -= (uint64_t)((int64_t)ov * (int64_t)ov);
        int ob = servo_bin_index(ov);
        if (servo_hist_1m[ob] > 0) servo_hist_1m[ob]--;
        servo_1m_count--;
      }
    }
    metric_1m_size = new_1m_size;
  }

  // 1m sliding window update (O(1))
  if (servo_1m_count == metric_1m_size) {
    int32_t old = servo_1m_ring[servo_1m_head];
    servo_1m_sum -= old;
    servo_1m_sumsq -= (uint64_t)((int64_t)old * (int64_t)old);
    int ob = servo_bin_index(old);
    if (servo_hist_1m[ob] > 0) servo_hist_1m[ob]--;
  } else {
    servo_1m_count++;
  }
  servo_1m_ring[servo_1m_head] = sample_us;
  servo_1m_sum += sample_us;
  servo_1m_sumsq += (uint64_t)((int64_t)sample_us * (int64_t)sample_us);
  servo_hist_1m[bin]++;
  servo_1m_head = (servo_1m_head + 1) % METRIC_1M_MAX_SAMPLES;
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
      if (b == 0) return (int32_t)(servo_center_us - SERVO_RANGE_US);
      if (b == SERVO_BIN_COUNT - 1) return (int32_t)(servo_center_us + SERVO_RANGE_US);
      int64_t lower = servo_center_us - SERVO_RANGE_US + (int64_t)(b - 1) * SERVO_BIN_WIDTH_US;
      return (int32_t)lower;  // lower edge of bin
    }
  }
  return (int32_t)servo_center_us;
}

static inline int32_t servo_percentile_1m(double p) {
  if (servo_1m_count <= 0) return 0;
  uint32_t target = (uint32_t)ceil(p * servo_1m_count);
  if (target < 1) target = 1;
  uint32_t acc = 0;
  for (int b = 0; b < SERVO_BIN_COUNT; ++b) {
    acc += servo_hist_1m[b];
    if (acc >= target) {
      if (b == 0) return (int32_t)(servo_center_us - SERVO_RANGE_US);
      if (b == SERVO_BIN_COUNT - 1) return (int32_t)(servo_center_us + SERVO_RANGE_US);
      int64_t lower = servo_center_us - SERVO_RANGE_US + (int64_t)(b - 1) * SERVO_BIN_WIDTH_US;
      return (int32_t)lower;
    }
  }
  return (int32_t)servo_center_us;
}

// UART RTT (us) histogram: 0..2000us with 5us bins
#define RTT_MAX_US 2000
#define RTT_BIN_WIDTH_US 5
#define RTT_BIN_COUNT ((RTT_MAX_US / RTT_BIN_WIDTH_US) + 2)  // +under/+over

static int rtt_win_size = 1000; // align to 1s window size
static int rtt_count = 0;
static int rtt_head = 0;
static int32_t rtt_ring[METRIC_1M_MAX_SAMPLES];
static int64_t rtt_sum = 0;
static uint64_t rtt_sumsq = 0;
static uint32_t rtt_hist[RTT_BIN_COUNT];
// 1-minute sliding window for RTT (us)
static uint32_t rtt_hist_1m[RTT_BIN_COUNT];
static int rtt_1m_size = 60000, rtt_1m_count = 0, rtt_1m_head = 0;
static int32_t rtt_1m_ring[METRIC_1M_MAX_SAMPLES];
static int64_t rtt_1m_sum = 0; static uint64_t rtt_1m_sumsq = 0;

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

  memset(rtt_hist, 0, sizeof(rtt_hist));
  // reset 1m sliding window
  rtt_1m_size = rtt_1m_size; // keep size
  rtt_1m_count = 0;
  rtt_1m_head = 0;
  memset(rtt_hist_1m, 0, sizeof(rtt_hist_1m));

  rtt_1m_sum = 0;
  rtt_1m_sumsq = 0;
}

static inline void rtt_set_1s_window_internal(int n) {
  if (n <= 0) n = 1;
  if (n > METRIC_1M_MAX_SAMPLES) n = METRIC_1M_MAX_SAMPLES;
  if (n != rtt_win_size) {
    if (rtt_count > n) {
      int drop = rtt_count - n;
      for (int i = 0; i < drop; ++i) {
        int oldest_idx = rtt_head - rtt_count;
        while (oldest_idx < 0) oldest_idx += METRIC_1M_MAX_SAMPLES;
        int32_t ov = rtt_ring[oldest_idx];
        rtt_sum -= ov;
        rtt_sumsq -= (uint64_t)((int64_t)ov * (int64_t)ov);
        int ob = rtt_bin_index(ov);
        if (rtt_hist[ob] > 0) rtt_hist[ob]--;
        rtt_count--;
      }
    }
    rtt_win_size = n;
    if (rtt_head >= rtt_win_size) rtt_head %= rtt_win_size;
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
  int bin = rtt_bin_index(us);
  rtt_hist[bin]++;
  rtt_head = (rtt_head + 1) % rtt_win_size;

  // align 1m window size to metric_1m_size
  if (rtt_1m_size != metric_1m_size) {
    if (rtt_1m_count > metric_1m_size) {
      int drop = rtt_1m_count - metric_1m_size;
      for (int i = 0; i < drop; ++i) {
        int oldest_idx = rtt_1m_head - rtt_1m_count;
        while (oldest_idx < 0) oldest_idx += METRIC_1M_MAX_SAMPLES;
        int32_t ov = rtt_1m_ring[oldest_idx];
        rtt_1m_sum -= ov;
        rtt_1m_sumsq -= (uint64_t)((int64_t)ov * (int64_t)ov);
        int ob = rtt_bin_index(ov);
        if (rtt_hist_1m[ob] > 0) rtt_hist_1m[ob]--;
        rtt_1m_count--;
      }
    }
    rtt_1m_size = metric_1m_size;
  }
  // 1m sliding window update
  if (rtt_1m_count == rtt_1m_size) {
    int32_t old = rtt_1m_ring[rtt_1m_head];
    rtt_1m_sum -= old;
    rtt_1m_sumsq -= (uint64_t)((int64_t)old * (int64_t)old);
    int ob = rtt_bin_index(old);
    if (rtt_hist_1m[ob] > 0) rtt_hist_1m[ob]--;
  } else {
    rtt_1m_count++;
  }
  rtt_1m_ring[rtt_1m_head] = us;
  rtt_1m_sum += us;
  rtt_1m_sumsq += (uint64_t)((int64_t)us * (int64_t)us);
  rtt_hist_1m[bin]++;
  rtt_1m_head = (rtt_1m_head + 1) % METRIC_1M_MAX_SAMPLES;
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

static inline int32_t rtt_percentile_1m(double p) {
  if (rtt_1m_count <= 0) return 0;
  uint32_t target = (uint32_t)ceil(p * rtt_1m_count);
  if (target < 1) target = 1;
  uint32_t acc = 0;
  for (int b = 0; b < RTT_BIN_COUNT; ++b) {
    acc += rtt_hist_1m[b];
    if (acc >= target) {
      if (b == 0) return 0;
      if (b == RTT_BIN_COUNT - 1) return RTT_MAX_US;
      int32_t lower = (int32_t)((b - 1) * RTT_BIN_WIDTH_US);
      return lower;
    }
  }
  return 0;
}



// Frame age (us) histogram — 1s window
#define AGE_MAX_US 6000
#define AGE_BIN_WIDTH_US 10
#define AGE_BIN_COUNT ((AGE_MAX_US / AGE_BIN_WIDTH_US) + 2) // +under/+over

static int age_win_size = 1000; // 1s @1kHz; auto-adjusted from period
static int age_count = 0, age_head = 0;
static int32_t age_ring[METRIC_1M_MAX_SAMPLES];
static uint32_t age_hist[AGE_BIN_COUNT];

static inline int age_bin_index_age(const int32_t age_us) {
  // Treat > 3 servo periods as infinity
  int32_t thr = (int32_t)((servo_center_us > 0 ? 3 * servo_center_us : 3000));
  if (thr > AGE_MAX_US) thr = AGE_MAX_US;
  if (age_us < 0) return 0;
  if (age_us > thr) return AGE_BIN_COUNT - 1;
  int idx = (age_us / AGE_BIN_WIDTH_US) + 1;
  if (idx < 1) idx = 1;
  if (idx > AGE_BIN_COUNT - 2) idx = AGE_BIN_COUNT - 2;
  return idx;
}

static inline void age_reset_all_internal(void) {
  age_count = 0;
  age_head = 0;
  memset(age_hist, 0, sizeof(age_hist));
}

static inline void age_set_1s_window_internal(int n) {
  if (n <= 0) n = 1;
  if (n > METRIC_1M_MAX_SAMPLES) n = METRIC_1M_MAX_SAMPLES;
  if (n != age_win_size) {
    if (age_count > n) {
      int drop = age_count - n;
      for (int i = 0; i < drop; ++i) {
        int oldest_idx = age_head - age_count;
        while (oldest_idx < 0) oldest_idx += METRIC_1M_MAX_SAMPLES;
        int32_t ov = age_ring[oldest_idx];
        int ob = age_bin_index_age(ov);
        if (age_hist[ob] > 0) age_hist[ob]--;
        age_count--;
      }
    }
    age_win_size = n;
    if (age_head >= age_win_size) age_head %= age_win_size;
  }
}

static inline void age_push_internal(const int32_t age_us) {
  if (age_count == age_win_size) {
    int32_t old = age_ring[age_head];
    int ob = age_bin_index_age(old);
    if (age_hist[ob] > 0) age_hist[ob]--;
  } else {
    age_count++;
  }
  age_ring[age_head] = age_us;
  int nb = age_bin_index_age(age_us);
  age_hist[nb]++;
  age_head = (age_head + 1) % age_win_size;
}

static inline void metrics_update_age_outputs(state_t *state) {
  if (age_count <= 0) {
    *state->age_us_1m_p50 = 0; // reuse fields for now; will export as 1s pins
    *state->age_us_1m_p95 = 0;
    *state->age_us_1m_p99 = 0;
    return;
  }
  uint32_t target50 = (uint32_t)ceil(0.50 * age_count);
  uint32_t target95 = (uint32_t)ceil(0.95 * age_count);
  uint32_t target99 = (uint32_t)ceil(0.99 * age_count);
  uint32_t acc = 0; int p50 = 0, p95 = 0, p99 = 0;
  for (int b = 0; b < AGE_BIN_COUNT; ++b) {
    acc += age_hist[b];
    if (!p50 && acc >= target50) {
      if (b == 0) p50 = 0; else if (b == AGE_BIN_COUNT - 1) p50 = (int)(3 * (servo_center_us ? servo_center_us : 1000)); else p50 = (b - 1) * AGE_BIN_WIDTH_US;
    }
    if (!p95 && acc >= target95) {
      if (b == 0) p95 = 0; else if (b == AGE_BIN_COUNT - 1) p95 = (int)(3 * (servo_center_us ? servo_center_us : 1000)); else p95 = (b - 1) * AGE_BIN_WIDTH_US;
    }
    if (!p99 && acc >= target99) {
      if (b == 0) p99 = 0; else if (b == AGE_BIN_COUNT - 1) p99 = (int)(3 * (servo_center_us ? servo_center_us : 1000)); else p99 = (b - 1) * AGE_BIN_WIDTH_US;
      break;
    }
  }
  *state->age_us_1m_p50 = p50;
  *state->age_us_1m_p95 = p95;
  *state->age_us_1m_p99 = p99;
}

void metrics_age_sample(state_t *state, const int32_t age_us) {
  age_push_internal(age_us);
  metrics_update_age_outputs(state);
}


static inline void metrics_update_servo_outputs(state_t *state) {
  if (servo_count <= 0) {
    *state->servo_us_1s_min = 0;
    *state->servo_us_1s_max = 0;
    *state->servo_us_1s_mean = 0;
    *state->servo_us_1s_stddev = 0;
    *state->servo_us_1s_p50 = 0;
    *state->servo_us_1s_p95 = 0;
    *state->servo_us_1s_p99 = 0;
    *state->servo_us_1s_p99_9 = 0;
  } else {
    int32_t mn, mx;
    servo_compute_minmax(&mn, &mx);
    *state->servo_us_1s_min = mn;
    *state->servo_us_1s_max = mx;
    int32_t mean = (int32_t)(servo_sum / (servo_count ? servo_count : 1));
    *state->servo_us_1s_mean = mean;
    double ex2 = (double)servo_sumsq / (double)(servo_count ? servo_count : 1);
    double var = ex2 - ((double)mean * (double)mean);
    if (var < 0.0) var = 0.0;
    *state->servo_us_1s_stddev = (int32_t)(sqrt(var) + 0.5);
    *state->servo_us_1s_p50 = servo_percentile(0.50);
    *state->servo_us_1s_p95 = servo_percentile(0.95);
    *state->servo_us_1s_p99 = servo_percentile(0.99);
    *state->servo_us_1s_p99_9 = servo_percentile(0.999);
  }
  // 1-minute sliding window outputs
  if (servo_1m_count <= 0) {
    *state->servo_us_1m_mean = 0;
    *state->servo_us_1m_stddev = 0;
    *state->servo_us_1m_p50 = 0;
    *state->servo_us_1m_p95 = 0;
    *state->servo_us_1m_p99 = 0;
    *state->servo_us_1m_p99_9 = 0;
  } else {
    double mean = (double)servo_1m_sum / (double)servo_1m_count;
    double ex2 = (double)servo_1m_sumsq / (double)servo_1m_count;
    double var = ex2 - mean * mean;
    if (var < 0.0) var = 0.0;
    *state->servo_us_1m_mean = (int32_t)(mean + 0.5);
    *state->servo_us_1m_stddev = (int32_t)(sqrt(var) + 0.5);
    *state->servo_us_1m_p50 = servo_percentile_1m(0.50);
    *state->servo_us_1m_p95 = servo_percentile_1m(0.95);
    *state->servo_us_1m_p99 = servo_percentile_1m(0.99);
    *state->servo_us_1m_p99_9 = servo_percentile_1m(0.999);
  }
}

static inline void metrics_update_rtt_outputs(state_t *state) {
  if (rtt_count <= 0) {
    *state->rtt_us_1s_min = 0;
    *state->rtt_us_1s_max = 0;
    *state->rtt_us_1s_mean = 0;
    *state->rtt_us_1s_stddev = 0;
    *state->rtt_us_1s_p50 = 0;
    *state->rtt_us_1s_p95 = 0;
    *state->rtt_us_1s_p99 = 0;
    *state->rtt_us_1s_p99_9 = 0;
  } else {
    int32_t mn, mx;
    rtt_compute_minmax(&mn, &mx);
    *state->rtt_us_1s_min = mn;
    *state->rtt_us_1s_max = mx;
    int32_t mean = (int32_t)(rtt_sum / (rtt_count ? rtt_count : 1));
    *state->rtt_us_1s_mean = mean;
    double ex2 = (double)rtt_sumsq / (double)(rtt_count ? rtt_count : 1);
    double var = ex2 - ((double)mean * (double)mean);
    if (var < 0.0) var = 0.0;
    *state->rtt_us_1s_stddev = (int32_t)(sqrt(var) + 0.5);
    *state->rtt_us_1s_p50 = rtt_percentile(0.50);
    *state->rtt_us_1s_p95 = rtt_percentile(0.95);
    *state->rtt_us_1s_p99 = rtt_percentile(0.99);
    *state->rtt_us_1s_p99_9 = rtt_percentile(0.999);
  }
  // 1-minute sliding window outputs
  if (rtt_1m_count <= 0) {
    *state->rtt_us_1m_mean = 0;
    *state->rtt_us_1m_stddev = 0;
    *state->rtt_us_1m_p50 = 0;
    *state->rtt_us_1m_p95 = 0;
    *state->rtt_us_1m_p99 = 0;

    *state->rtt_us_1m_p99_9 = 0;
  } else {
    double mean = (double)rtt_1m_sum / (double)rtt_1m_count;
    double ex2 = (double)rtt_1m_sumsq / (double)rtt_1m_count;
    double var = ex2 - mean * mean;
    if (var < 0.0) var = 0.0;
    *state->rtt_us_1m_mean = (int32_t)(mean + 0.5);
    *state->rtt_us_1m_stddev = (int32_t)(sqrt(var) + 0.5);
    *state->rtt_us_1m_p50 = rtt_percentile_1m(0.50);
    *state->rtt_us_1m_p95 = rtt_percentile_1m(0.95);
    *state->rtt_us_1m_p99 = rtt_percentile_1m(0.99);
    *state->rtt_us_1m_p99_9 = rtt_percentile_1m(0.999);
  }
}

void metrics_reset_all(state_t *state) {
  servo_reset_all_internal();
  rtt_reset_all_internal();
  age_reset_all_internal();
  servo_center_us = 0;
  servo_prev_ts_ns = 0;
  // zero outputs
  metrics_update_servo_outputs(state);
  metrics_update_rtt_outputs(state);
  *state->metrics_reset = 0;
}

void metrics_servo_tick(state_t *state, long l_period_ns) {

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
    uint64_t dt_ns = now_ns - servo_prev_ts_ns;
    uint64_t dt_us64 = (dt_ns + 500ull) / 1000ull;  // round to nearest us
    int32_t dt_us = (dt_us64 > (uint64_t)INT32_MAX) ? INT32_MAX : (int32_t)dt_us64;
    servo_push_internal(dt_us, l_period_ns);
    metrics_update_servo_outputs(state);
  }
  servo_prev_ts_ns = now_ns;
}
static int metrics_export_pins(int comp_id, const char *prefix, state_t *state) {
  // control
  if (pin_err(hal_pin_bit_newf(HAL_IN, &state->metrics_reset, comp_id, "%s.metrics.reset", prefix))) return -1;


  // servo timing stats (us) — 1s window
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_min, comp_id, "%s.metrics.servo-us.1s.min", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_max, comp_id, "%s.metrics.servo-us.1s.max", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_mean, comp_id, "%s.metrics.servo-us.1s.mean", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_stddev, comp_id, "%s.metrics.servo-us.1s.stddev", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_p50, comp_id, "%s.metrics.servo-us.1s.p50", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_p95, comp_id, "%s.metrics.servo-us.1s.p95", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_p99, comp_id, "%s.metrics.servo-us.1s.p99", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1s_p99_9, comp_id, "%s.metrics.servo-us.1s.p99_9", prefix)))
    return -1;
  // 1-minute decayed servo aggregates (us)
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1m_mean, comp_id, "%s.metrics.servo-us.1m.mean", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1m_stddev, comp_id, "%s.metrics.servo-us.1m.stddev", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1m_p50, comp_id, "%s.metrics.servo-us.1m.p50", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1m_p95, comp_id, "%s.metrics.servo-us.1m.p95", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1m_p99, comp_id, "%s.metrics.servo-us.1m.p99", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->servo_us_1m_p99_9, comp_id, "%s.metrics.servo-us.1m.p99_9", prefix))) return -1;

  // UART RTT stats (us) — 1s window
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_min, comp_id, "%s.metrics.uart-rtt-us.1s.min", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_max, comp_id, "%s.metrics.uart-rtt-us.1s.max", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_mean, comp_id, "%s.metrics.uart-rtt-us.1s.mean", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_stddev, comp_id, "%s.metrics.uart-rtt-us.1s.stddev", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_p50, comp_id, "%s.metrics.uart-rtt-us.1s.p50", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_p95, comp_id, "%s.metrics.uart-rtt-us.1s.p95", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_p99, comp_id, "%s.metrics.uart-rtt-us.1s.p99", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1s_p99_9, comp_id, "%s.metrics.uart-rtt-us.1s.p99_9", prefix)))
    return -1;
  // 1-minute decayed UART RTT aggregates (us)
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1m_mean, comp_id, "%s.metrics.uart-rtt-us.1m.mean", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1m_stddev, comp_id, "%s.metrics.uart-rtt-us.1m.stddev", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1m_p50, comp_id, "%s.metrics.uart-rtt-us.1m.p50", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1m_p95, comp_id, "%s.metrics.uart-rtt-us.1m.p95", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1m_p99, comp_id, "%s.metrics.uart-rtt-us.1m.p99", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->rtt_us_1m_p99_9, comp_id, "%s.metrics.uart-rtt-us.1m.p99_9", prefix))) return -1;

  // counters under metrics as well
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_frames_ok, comp_id, "%s.metrics.frames-ok", prefix))) return -1;
  // 1-minute frame age (us)
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->age_us_1m_p50, comp_id, "%s.metrics.frame-age-us.1s.p50", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->age_us_1m_p95, comp_id, "%s.metrics.frame-age-us.1s.p95", prefix))) return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->age_us_1m_p99, comp_id, "%s.metrics.frame-age-us.1s.p99", prefix))) return -1;

  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_bad_header, comp_id, "%s.metrics.bad-header", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_read_errors, comp_id, "%s.metrics.read-errors", prefix)))
    return -1;
  if (pin_err(hal_pin_s32_newf(HAL_OUT, &state->metric_write_errors, comp_id, "%s.metrics.write-errors", prefix)))
    return -1;

  // initial values
  *state->metrics_reset = 0;


  *state->servo_us_1s_min = 0;
  *state->servo_us_1s_max = 0;
  *state->servo_us_1s_mean = 0;
  *state->servo_us_1s_stddev = 0;
  *state->servo_us_1s_p50 = 0;
  *state->servo_us_1s_p95 = 0;
  *state->servo_us_1s_p99 = 0;
  *state->servo_us_1s_p99_9 = 0;
  *state->servo_us_1m_mean = 0;
  *state->servo_us_1m_stddev = 0;
  *state->servo_us_1m_p50 = 0;
  *state->servo_us_1m_p95 = 0;
  *state->servo_us_1m_p99 = 0;
  *state->servo_us_1m_p99_9 = 0;

  *state->rtt_us_1s_min = 0;
  *state->rtt_us_1s_max = 0;
  *state->rtt_us_1s_mean = 0;
  *state->rtt_us_1s_stddev = 0;
  *state->rtt_us_1s_p50 = 0;
  *state->rtt_us_1s_p95 = 0;
  *state->rtt_us_1s_p99 = 0;
  *state->rtt_us_1s_p99_9 = 0;
  *state->rtt_us_1m_mean = 0;
  *state->rtt_us_1m_stddev = 0;
  *state->rtt_us_1m_p50 = 0;
  *state->rtt_us_1m_p95 = 0;
  *state->rtt_us_1m_p99 = 0;
  *state->rtt_us_1m_p99_9 = 0;

  *state->age_us_1m_p50 = 0;
  *state->age_us_1m_p95 = 0;
  *state->age_us_1m_p99 = 0;


  *state->metric_frames_ok = 0;
  *state->metric_bad_header = 0;
  *state->metric_read_errors = 0;
  *state->metric_write_errors = 0;

  return 0;
}

void metrics_rtt_sample(state_t *state, int32_t rtt_us) {

  rtt_push_internal(rtt_us);
  metrics_update_rtt_outputs(state);
}

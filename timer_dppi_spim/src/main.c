/*
 * timer_dppi_spim - accelerometer read at a fixed rate set by a TIMER,
 * nRF Connect SDK v3.4.1
 *
 *   TIMER COMPARE0 --DPPI--> SPIM START
 *   SPIM (hardware CSN, EasyDMA) --END--DPPI--> TIMER counter
 *
 * The board overlay selects sensor, bus, pins and timers (see app_dt.h);
 * Kconfig selects period and consumption mode. This file only reports and,
 * when APP_SWEEP_PERIODS_US is set, sweeps the period (bench).
 */
#include <stdlib.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensor.h"
#include "spim_dppi.h"

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

#define G_MS2 9.80665f

static float to_ms2(int16_t raw)
{
	return raw * sensor->mg_per_lsb * G_MS2 / 1000.0f;
}

/* Per-report window statistics (QUEUE mode) */
struct stats {
	uint32_t n, fresh;
	int16_t zmin, zmax;
	int32_t zsum;
};

static void stats_reset(struct stats *s)
{
	s->n = s->fresh = 0;
	s->zmin = INT16_MAX;
	s->zmax = INT16_MIN;
	s->zsum = 0;
}

#if defined(CONFIG_APP_CONSUME_LATEST)
static void report_latest(uint32_t elapsed_ms)
{
	uint8_t raw[SENSOR_BURST_LEN];
	struct sensor_sample s;
	bool ok = spim_dppi_latest(raw);

	if (ok) {
		sensor->decode(raw, &s);
	}
	LOG_INF("t=%u ms xfers=%u %s X=%.2f Y=%.2f Z=%.2f m/s^2 %s", elapsed_ms,
		spim_dppi_total_xfers(), ok ? "latest" : "torn",
		ok ? (double)to_ms2(s.x) : 0.0, ok ? (double)to_ms2(s.y) : 0.0,
		ok ? (double)to_ms2(s.z) : 0.0, (ok && s.fresh) ? "(fresh)" : "");
}

/* Sleep for one report period, then print the latest sample */
static void run_window(uint32_t *elapsed_ms)
{
	k_msleep(CONFIG_APP_REPORT_PERIOD_MS);
	*elapsed_ms += CONFIG_APP_REPORT_PERIOD_MS;
	report_latest(*elapsed_ms);
}
#else
static struct stats win;

/* Drain the queue for one report period, then print the window statistics */
static void run_window(uint32_t *elapsed_ms)
{
	uint8_t raw[SENSOR_BURST_LEN];
	struct sensor_sample s;
	int64_t end = k_uptime_get() + CONFIG_APP_REPORT_PERIOD_MS;

	while (k_uptime_get() < end) {
		/* One sample per get, in order; the ISR fills the queue N at a time */
		if (k_msgq_get(&sample_q, raw, K_MSEC(5)) != 0) {
			continue;
		}
		sensor->decode(raw, &s);
		win.n++;
		win.fresh += s.fresh;
		win.zmin = MIN(win.zmin, s.z);
		win.zmax = MAX(win.zmax, s.z);
		win.zsum += s.z;
	}
	*elapsed_ms += CONFIG_APP_REPORT_PERIOD_MS;
	LOG_INF("t=%u ms xfers=%u queued=%u fresh=%u skipped=%u dropped=%u late=%u Z avg=%.2f min=%.2f max=%.2f m/s^2",
		*elapsed_ms, spim_dppi_total_xfers(), win.n, win.fresh, spim_dppi_skipped(),
		spim_dppi_dropped(), spim_dppi_late_wraps(),
		win.n ? (double)to_ms2(win.zsum / (int32_t)win.n) : 0.0,
		win.n ? (double)to_ms2(win.zmin) : 0.0, win.n ? (double)to_ms2(win.zmax) : 0.0);
	stats_reset(&win);
}
#endif

#if defined(CONFIG_APP_CONSUME_QUEUE)
/*
 * Sweep: run each period for APP_SWEEP_STEP_S seconds and print the totals
 * of the step (first second discarded as settling). Fresh per second vs the
 * sensor's real ODR shows whether the timer rate loses samples.
 */
static void run_sweep(const char *list)
{
	char buf[128];
	uint32_t elapsed_ms = 0;

	strncpy(buf, list, sizeof(buf) - 1);
	buf[sizeof(buf) - 1] = '\0';

	for (char *tok = strtok(buf, ","); tok; tok = strtok(NULL, ",")) {
		uint32_t period = strtoul(tok, NULL, 10);

		if (period == 0) {
			continue;
		}
		spim_dppi_set_period_us(period);
		LOG_INF("=== sweep: period %u us (%u.%u Hz) for %d s", period,
			1000000 / period, (10000000 / period) % 10, CONFIG_APP_SWEEP_STEP_S);
		run_window(&elapsed_ms);   /* settling second, not counted */
#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
		uint32_t d1, d2, d3;

		spim_dppi_wrap_latency(&d1, &d2, &d3, true);   /* discard the settling second */
#endif

		uint32_t x0 = spim_dppi_total_xfers(), q = 0, f = 0;
		uint32_t s0 = spim_dppi_skipped(), d0 = spim_dppi_dropped(), l0 = spim_dppi_late_wraps();
		uint8_t raw[SENSOR_BURST_LEN];
		struct sensor_sample s;
		int64_t end = k_uptime_get() + (CONFIG_APP_SWEEP_STEP_S - 1) * 1000;

		while (k_uptime_get() < end) {
			if (k_msgq_get(&sample_q, raw, K_MSEC(5)) == 0) {
				sensor->decode(raw, &s);
				q++;
				f += s.fresh;
			}
		}
		uint32_t secs = CONFIG_APP_SWEEP_STEP_S - 1;

		LOG_INF("=== sweep result: period %u us: xfers/s=%u queued/s=%u fresh/s=%u.%u skipped=%u dropped=%u late_wraps=%u",
			period, (spim_dppi_total_xfers() - x0) / secs, q / secs, f / secs,
			(f * 10 / secs) % 10, spim_dppi_skipped() - s0, spim_dppi_dropped() - d0,
			spim_dppi_late_wraps() - l0);
#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
		uint32_t mn, av, mx;

		if (spim_dppi_wrap_latency(&mn, &av, &mx, true)) {
			LOG_INF("=== wrap latency (trigger -> wrap ISR): min=%u.%02u avg=%u.%02u max=%u.%02u us",
				mn / 1000, (mn % 1000) / 10, av / 1000, (av % 1000) / 10,
				mx / 1000, (mx % 1000) / 10);
		}
#endif
	}
	LOG_INF("=== sweep done");
}
#endif

int main(void)
{
	int err;
	uint32_t elapsed_ms = 0;

	LOG_INF("timer_dppi_spim: %s, trigger=timer %u us, consume=%s%s%s", sensor->name,
		CONFIG_APP_SAMPLE_PERIOD_US,
		IS_ENABLED(CONFIG_APP_CONSUME_QUEUE) ? "queue N=" : "latest",
		IS_ENABLED(CONFIG_APP_CONSUME_QUEUE) ? STRINGIFY(CONFIG_APP_BLOCK_SAMPLES) : "",
		IS_ENABLED(CONFIG_APP_QUEUE_FRESH_ONLY) ? " fresh-only" : "");

	err = spim_dppi_init();
	if (err) {
		LOG_ERR("init failed (%d)", err);
		return err;
	}
	LOG_INF("%s ODR %u.%u Hz", sensor->name, sensor_odr_hz_x10() / 10, sensor_odr_hz_x10() % 10);

	err = spim_dppi_start();
	if (err) {
		LOG_ERR("start failed (%d)", err);
		return err;
	}

#if defined(CONFIG_APP_CONSUME_QUEUE)
	stats_reset(&win);
#endif
#if defined(CONFIG_APP_CONSUME_QUEUE)
	if (CONFIG_APP_SWEEP_PERIODS_US[0] != '\0') {
		run_sweep(CONFIG_APP_SWEEP_PERIODS_US);
	}
#endif
	while (1) {
		run_window(&elapsed_ms);
	}
	return 0;
}

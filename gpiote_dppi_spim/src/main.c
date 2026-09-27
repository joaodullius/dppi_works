/*
 * gpiote_dppi_spim - accelerometer read started by its data-ready pin,
 * nRF Connect SDK v3.4.1
 *
 *   sensor INT pin --GPIOTE IN event--DPPI--> SPIM START
 *   SPIM (hardware CSN, EasyDMA, RX pointer post-increment) fills a ring;
 *   a drain thread moves the new slots to a k_msgq every APP_DRAIN_PERIOD_US.
 *
 * The board overlay selects sensor, bus and pins (see app_dt.h); Kconfig
 * selects the drain period and the ring size. This file only reports.
 */
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
	LOG_INF("t=%u ms xfers=%u queued=%u fresh=%u dropped=%u late=%u ovf=%u Z avg=%.2f min=%.2f max=%.2f m/s^2",
		*elapsed_ms, spim_dppi_total_xfers(), win.n, win.fresh,
		spim_dppi_dropped(), spim_dppi_late_wraps(), spim_dppi_overflows(),
		win.n ? (double)to_ms2(win.zsum / (int32_t)win.n) : 0.0,
		win.n ? (double)to_ms2(win.zmin) : 0.0, win.n ? (double)to_ms2(win.zmax) : 0.0);
	stats_reset(&win);
}

int main(void)
{
	int err;
	uint32_t elapsed_ms = 0;

	LOG_INF("gpiote_dppi_spim: %s, trigger=data-ready pin, drain every %u us", sensor->name,
		CONFIG_APP_DRAIN_PERIOD_US);

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

	stats_reset(&win);
	while (1) {
		run_window(&elapsed_ms);
	}
	return 0;
}

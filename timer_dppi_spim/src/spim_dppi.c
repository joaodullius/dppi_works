#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>
#if defined(CONFIG_APP_REQUEST_HFXO)
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/onoff.h>
#endif

#include <nrfx_spim.h>
#include <nrfx_timer.h>
#if defined(CONFIG_APP_RRAM_STANDBY)
#include <hal/nrf_rramc.h>
#endif
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_spim.h>
#include <hal/nrf_timer.h>

#include "app_dt.h"
#include "sensor.h"
#include "spim_dppi.h"

LOG_MODULE_REGISTER(spim_dppi, LOG_LEVEL_INF);

/* ---- Instances from the devicetree ------------------------------------ */

static nrfx_spim_t spim = NRFX_SPIM_INSTANCE(DT_REG_ADDR(BUS_NODE));
PINCTRL_DT_DEFINE(BUS_NODE);

/* Trigger: free-running TIMER whose COMPARE0 starts every transaction */
static nrfx_timer_t timer_trig = NRFX_TIMER_INSTANCE(DT_REG_ADDR(TRIG_TIMER_NODE));

/* Event after which the .PTR register may be rewritten for the next transfer:
 * DMA.RX.READY on nRF54L ("EasyDMA has buffered the .PTR and .MAXCNT
 * registers"), STARTED on nRF53/52 ("can be updated immediately after having
 * received the STARTED event"). nrfx names the former RXSTARTED. */
#if NRF_SPIM_HAS_DMA_REG && !defined(CONFIG_APP_COUNT_STARTED)
#define READY_EVENT      NRF_SPIM_EVENT_RXSTARTED
#define READY_INT_MASK   NRF_SPIM_INT_RXREADY_MASK
#define READY_EVENT_NAME "DMA.RX.READY"
#else
#define READY_EVENT      NRF_SPIM_EVENT_STARTED
#define READY_INT_MASK   NRF_SPIM_INT_STARTED_MASK
#define READY_EVENT_NAME "STARTED"
#endif

/* ---- Ring ---------------------------------------------------------------- */

/*
 * The EasyDMA array list writes one burst per transaction into consecutive
 * slots of the ring, on its own. A drain thread wakes up every
 * APP_DRAIN_PERIOD_US, reads the DMA pointer to know how many slots arrived,
 * pushes them into the message queue and then arms the wrap: the next READY
 * interrupt rewrites the pointer to slot 0, inside the window the datasheet
 * allows (right after READY/STARTED, before the next START). No TIMER
 * counter, no EGU, no zero-latency ISR.
 *
 * GUARD slots follow the ring so that a drain that runs late overflows into
 * unused memory (counted as overflow) instead of whatever follows the array.
 */
#define RING_SLOTS  CONFIG_APP_RING_SLOTS
#define GUARD_SLOTS 8

K_MSGQ_DEFINE(sample_q, SENSOR_BURST_LEN, CONFIG_APP_QUEUE_DEPTH, 1);

static uint8_t ring[RING_SLOTS + GUARD_SLOTS][SENSOR_BURST_LEN];
/* EasyDMA source: the backend's TX prefix copied to RAM (EasyDMA cannot read flash) */
static uint8_t burst_tx[SENSOR_BURST_LEN];

static uint32_t dropped;      /* samples that did not fit in the queue */
static uint32_t late_wraps;   /* wraps written after the next START (one sample to slack) */
static uint32_t overflows;    /* drains that found the DMA past the ring end */
static uint32_t lap_base;     /* transactions completed in previous laps (for xfers) */
static uint32_t skipped;      /* repeated samples dropped by the fresh filter */
#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
/* Trigger-timer ticks (16 MHz) from the trigger COMPARE to the wrap ISR */
static uint32_t lat_min = UINT32_MAX, lat_max, lat_sum, lat_n;
#endif

/* Drain state, only touched by the drain thread and the wrap ISR */
static uint32_t tail;              /* next slot to deliver */
static volatile bool wrap_armed;   /* READY interrupt enabled, waiting to rewrite PTR */
static volatile bool wrap_done;    /* ISR wrote PTR = ring[0] */
static volatile uint32_t wrap_last;/* last slot of the lap that ended with that wrap */

static inline uint32_t rx_ptr_get(void)
{
#if NRF_SPIM_HAS_DMA_REG
	return spim.p_reg->DMA.RX.PTR;
#else
	return spim.p_reg->RXD.PTR;
#endif
}

/* Index of the slot the DMA will write next (the one before may be in flight) */
static inline uint32_t head_index(void)
{
	return (rx_ptr_get() - (uint32_t)ring[0]) / SENSOR_BURST_LEN;
}

/* ---- Blocking SPI for sensor configuration ----------------------------- */

static K_SEM_DEFINE(xfer_done, 0, 1);
static bool blocking_phase = true;

static void spim_evt_handler(nrfx_spim_event_t const *p_event, void *p_context)
{
	ARG_UNUSED(p_context);
	if (p_event->type == NRFX_SPIM_EVENT_DONE) {
		k_sem_give(&xfer_done);
	}
}

/*
 * Wrap ISR: READY of transaction k just fired, the register already points
 * at slot k+1. Point it at slot 0 instead: transaction k+1 writes slot 0. If
 * another READY arrives before the write, the next transaction had already
 * consumed slot k+1 (slack): counted as late, no corruption.
 */
static void wrap_isr(void)
{
	uint32_t h = head_index();          /* k + 1 */
#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
	/* The trigger timer was cleared by the COMPARE that started transaction k */
	uint32_t lt = nrfx_timer_capture(&timer_trig, NRF_TIMER_CC_CHANNEL1);

	lat_min = MIN(lat_min, lt);
	lat_max = MAX(lat_max, lt);
	lat_sum += lt;
	lat_n++;
#endif

	nrf_spim_int_disable(spim.p_reg, READY_INT_MASK);
	nrf_spim_event_clear(spim.p_reg, READY_EVENT);
	nrf_spim_rx_buffer_set(spim.p_reg, ring[0], SENSOR_BURST_LEN);
	if (nrf_spim_event_check(spim.p_reg, READY_EVENT)) {
		late_wraps++;
		nrf_spim_event_clear(spim.p_reg, READY_EVENT);
	}
	wrap_last = h - 1;
	wrap_armed = false;
	wrap_done = true;
}

/* The SPIM IRQ belongs to the nrfx driver during the blocking phase and to
 * the wrap logic afterwards. */
static void spim_irq_wrapper(const void *arg)
{
	if (blocking_phase) {
		nrfx_spim_irq_handler((nrfx_spim_t *)arg);
		return;
	}
	if (wrap_armed && nrf_spim_event_check(spim.p_reg, READY_EVENT)) {
		wrap_isr();
		return;
	}
	/* Anything else (e.g. END left enabled by the driver): silence it */
	nrf_spim_int_disable(spim.p_reg, 0xFFFFFFFF);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_STARTED);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_END);
}

int spim_xfer_blocking(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len)
{
	nrfx_spim_xfer_desc_t xfer = {
		.p_tx_buffer = tx, .tx_length = tx_len,
		.p_rx_buffer = rx, .rx_length = rx_len,
	};
	int err = nrfx_spim_xfer(&spim, &xfer, 0);

	if (err < 0) {
		LOG_ERR("nrfx_spim_xfer: %d", err);
		return err;
	}
	if (k_sem_take(&xfer_done, K_MSEC(100)) != 0) {
		LOG_ERR("SPI transfer timeout");
		return -ETIMEDOUT;
	}
	return 0;
}

/* ---- Init ---------------------------------------------------------------- */

#if defined(CONFIG_APP_REQUEST_HFXO)
static int hfxo_request(void)
{
	static struct onoff_client cli;
	struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
	int res;

	sys_notify_init_spinwait(&cli.notify);
	int err = onoff_request(mgr, &cli);

	if (err < 0) {
		return err;
	}
	while (sys_notify_fetch_result(&cli.notify, &res) == -EAGAIN) {
		k_msleep(1);
	}
	LOG_INF("HFXO running");
	return res < 0 ? res : 0;
}
#endif

static void timer_noop_handler(nrf_timer_event_t event_type, void *p_context)
{
	ARG_UNUSED(event_type);
	ARG_UNUSED(p_context);
}

int spim_dppi_init(void)
{
	int err;

#if defined(CONFIG_APP_RRAM_STANDBY)
	/* Fast RRAM wake-up: the ISR code lives in RRAM, and after an idle
	 * period the first fetch otherwise waits for the RRAM to power up */
	nrf_rramc_lp_mode_set(NRF_RRAMC, NRF_RRAMC_LP_STANDBY);
	LOG_INF("RRAMC low-power mode: standby");
#endif
#if defined(CONFIG_APP_REQUEST_HFXO)
	err = hfxo_request();
	if (err) {
		LOG_ERR("HFXO request failed: %d", err);
		return err;
	}
#endif

	/* Pins (SCK/MOSI/MISO/CSN) come from the bus node's pinctrl */
	err = pinctrl_apply_state(PINCTRL_DT_DEV_CONFIG_GET(BUS_NODE), PINCTRL_STATE_DEFAULT);
	if (err) {
		LOG_ERR("pinctrl_apply_state: %d", err);
		return err;
	}

	IRQ_CONNECT(DT_IRQN(BUS_NODE), DT_IRQ(BUS_NODE, priority), spim_irq_wrapper, &spim, 0);

	nrfx_spim_config_t cfg = NRFX_SPIM_DEFAULT_CONFIG(NRF_SPIM_PIN_NOT_CONNECTED,
							  NRF_SPIM_PIN_NOT_CONNECTED,
							  NRF_SPIM_PIN_NOT_CONNECTED,
							  CS_PIN_ABS);
	cfg.skip_gpio_cfg = true;   /* pinctrl did it */
	cfg.skip_psel_cfg = true;
	cfg.use_hw_ss = true;       /* CSN asserted by the SPIM around every transaction */
	cfg.ss_duration = CONFIG_APP_SPI_CSN_DURATION;
	cfg.frequency = CONFIG_APP_SPI_FREQ_HZ;
#if NRF_SPIM_HAS_RXDELAY
	if (CONFIG_APP_SPI_RX_DELAY >= 0) {
		cfg.rx_delay = CONFIG_APP_SPI_RX_DELAY;
	}
#endif

	err = nrfx_spim_init(&spim, &cfg, spim_evt_handler, NULL);
	if (err < 0) {
		LOG_ERR("nrfx_spim_init: %d", err);
		return err;
	}
	LOG_INF("SPIM @%p, hardware CSN on pin %d, %u Hz, CSNDUR %u, RXDELAY %d", spim.p_reg,
		CS_PIN_ABS, (unsigned)cfg.frequency, (unsigned)cfg.ss_duration,
#if NRF_SPIM_HAS_RXDELAY
		(int)cfg.rx_delay);
#else
		-1);
#endif

	err = sensor->init();
	if (err) {
		LOG_ERR("%s init failed: %d", sensor->name, err);
		return err;
	}
	return 0;
}

/* ---- Drain: ring -> message queue, then arm the wrap -------------------- */

static void deliver(uint32_t from, uint32_t to)
{
	for (uint32_t i = from; i < to; i++) {
#if defined(CONFIG_APP_QUEUE_FRESH_ONLY)
		/* TIMER faster than the ODR re-reads the previous sample: the
		 * data-ready bit in the burst's STATUS byte tells them apart */
		if ((ring[i][sensor->fresh_offset] & sensor->fresh_mask) == 0) {
			skipped++;
			continue;
		}
#endif
		if (k_msgq_put(&sample_q, ring[i], K_NO_WAIT) != 0) {
			dropped++;
		}
	}
}

/*
 * Called every APP_DRAIN_PERIOD_US from the drain thread. Slot h-1 may be in
 * flight, so only slots below h-1 are delivered; the remainder goes out on
 * the next drain. After a wrap, the lap that ended at wrap_last is finished
 * first, then the new lap from slot 0.
 */
/* Samples per drain above which the wrap is awaited awake (see below) */
#define SPIN_WRAP_MIN_ARRIVED 32

static void drain(void)
{
	uint32_t h = head_index();
	uint32_t arrived = 0;

	if (h > RING_SLOTS) {
		/* The drain came too late: the DMA is in the guard slots */
		overflows++;
	}
	if (wrap_done) {
		uint32_t last = wrap_last;

		/* Everything up to and including wrap_last is complete once the new
		 * lap has started (h >= 1); otherwise wrap_last may still be in flight */
		if (h >= 1) {
			deliver(tail, last + 1);
			lap_base += last + 1;
			tail = 0;
			wrap_done = false;
		} else {
			deliver(tail, last);
			tail = last;
			return;
		}
	}
	if (h >= 1 && h - 1 > tail) {
		arrived = h - 1 - tail;
		deliver(tail, h - 1);
		tail = h - 1;
	}
	/* Arm the wrap at the next READY (only if something is going to arrive
	 * beyond slot 0, i.e. the DMA left slot 0) */
	if (!wrap_armed && !wrap_done && h >= 1) {
		wrap_armed = true;
		nrf_spim_event_clear(spim.p_reg, READY_EVENT);
		nrf_spim_int_enable(spim.p_reg, READY_INT_MASK);
		/* At high rates the next READY is microseconds away and the wrap
		 * must land before the START after it. Waking the core from idle
		 * for that IRQ costs ~17 us on the nRF54L15 (RRAM) and a few us on
		 * the nRF5340, more than one period: stay awake for it instead.
		 * At low rates the IRQ path is used and the core sleeps. */
		if (arrived >= SPIN_WRAP_MIN_ARRIVED) {
			uint32_t t0 = k_cycle_get_32();
			uint32_t limit = k_us_to_cyc_ceil32(CONFIG_APP_DRAIN_PERIOD_US / 4);

			while (!wrap_done && (k_cycle_get_32() - t0) < limit) {
			}
		}
	}
}

static void drain_thread(void *a, void *b, void *c)
{
	ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
	while (1) {
		k_sleep(K_USEC(CONFIG_APP_DRAIN_PERIOD_US));
		if (!blocking_phase) {
			drain();
		}
	}
}

K_THREAD_DEFINE(drain_tid, 1024, drain_thread, NULL, NULL, NULL, -1, 0, 0);

uint32_t spim_dppi_total_xfers(void)
{
	/* Transactions started: previous laps plus the current head. Between the
	 * wrap ISR and its drain the head already restarted from 0, so add the
	 * finished lap that has not been folded into lap_base yet. */
	uint32_t h = head_index();

	return lap_base + h + (wrap_done ? wrap_last + 1 : 0);
}

uint32_t spim_dppi_dropped(void)
{
	return dropped;
}

uint32_t spim_dppi_late_wraps(void)
{
	return late_wraps;
}

uint32_t spim_dppi_overflows(void)
{
	return overflows;
}

uint32_t spim_dppi_skipped(void)
{
	return skipped;
}

#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
bool spim_dppi_wrap_latency(uint32_t *min_ns, uint32_t *avg_ns, uint32_t *max_ns, bool reset)
{
	unsigned int key = irq_lock();
	uint32_t n = lat_n, mn = lat_min, mx = lat_max, sum = lat_sum;

	if (reset) {
		lat_min = UINT32_MAX;
		lat_max = lat_sum = lat_n = 0;
	}
	irq_unlock(key);
	if (n == 0) {
		return false;
	}
	/* 16 MHz ticks -> ns (62.5 ns each) */
	*min_ns = mn * 125 / 2;
	*max_ns = mx * 125 / 2;
	*avg_ns = (uint32_t)((uint64_t)sum * 125 / 2 / n);
	return true;
}
#endif

void spim_dppi_set_period_us(uint32_t period_us)
{
	nrfx_timer_disable(&timer_trig);
	nrfx_timer_clear(&timer_trig);
	nrfx_timer_compare(&timer_trig, NRF_TIMER_CC_CHANNEL0,
			   nrfx_timer_us_to_ticks(&timer_trig, period_us), false);
	nrfx_timer_enable(&timer_trig);
}

/* ---- Trigger (TIMER COMPARE0 event), DPPI, start ----------------------- */

static int trigger_init(uint32_t *eep)
{
	/* 1 MHz is enough for the period; the latency bench needs 16 MHz resolution */
	nrfx_timer_config_t cfg = NRFX_TIMER_DEFAULT_CONFIG(
		IS_ENABLED(CONFIG_APP_WRAP_LATENCY_STATS) ? NRFX_MHZ_TO_HZ(16) : NRFX_MHZ_TO_HZ(1));
	int err;

	cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
	err = nrfx_timer_init(&timer_trig, &cfg, timer_noop_handler);
	if (err < 0) {
		return err;
	}
	nrfx_timer_extended_compare(&timer_trig, NRF_TIMER_CC_CHANNEL0,
				    nrfx_timer_us_to_ticks(&timer_trig, CONFIG_APP_SAMPLE_PERIOD_US),
				    NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, false);
	*eep = nrfx_timer_compare_event_address_get(&timer_trig, NRF_TIMER_CC_CHANNEL0);
	LOG_INF("trigger: TIMER @%p every %u us", timer_trig.p_reg, CONFIG_APP_SAMPLE_PERIOD_US);
	return 0;
}

int spim_dppi_start(void)
{
	nrfx_gppi_handle_t h_start;
	uint32_t eep;
	int err;

	err = trigger_init(&eep);
	if (err) {
		LOG_ERR("trigger init: %d", err);
		return err;
	}

	/* Repeated burst: the driver sets the buffers once and steps aside */
	memcpy(burst_tx, sensor->burst_tx, sensor->burst_tx_len);
	nrfx_spim_xfer_desc_t xfer = {
		.p_tx_buffer = burst_tx, .tx_length = sensor->burst_tx_len,
		.p_rx_buffer = ring[0], .rx_length = SENSOR_BURST_LEN,
	};
	uint32_t flags = NRFX_SPIM_FLAG_HOLD_XFER | NRFX_SPIM_FLAG_REPEATED_XFER |
			 NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER | NRFX_SPIM_FLAG_RX_POSTINC;
	err = nrfx_spim_xfer(&spim, &xfer, flags);
	if (err < 0) {
		LOG_ERR("arming repeated transfer: %d", err);
		return err;
	}
	/* From here on the peripheral is driven by DPPI. The driver may have left
	 * STARTED (nRF54L errata workaround) or END interrupts enabled; with
	 * nobody servicing them they would storm. */
	nrf_spim_int_disable(spim.p_reg, 0xFFFFFFFF);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_STARTED);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_END);
	nrf_spim_event_clear(spim.p_reg, READY_EVENT);
	blocking_phase = false;

	/* The only DPPI connection: trigger event -> SPIM START */
	err = nrfx_gppi_conn_alloc(eep, nrfx_spim_start_task_address_get(&spim), &h_start);
	if (err < 0) {
		return err;
	}
	nrfx_gppi_conn_enable(h_start);

	nrfx_timer_clear(&timer_trig);
	nrfx_timer_enable(&timer_trig);
	LOG_INF("DPPI connected, burst %u bytes, ring %u slots, drain every %u us, wrap on %s",
		SENSOR_BURST_LEN, RING_SLOTS, CONFIG_APP_DRAIN_PERIOD_US, READY_EVENT_NAME);
	return 0;
}

#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

#include <nrfx_spim.h>
#include <nrfx_gpiote.h>
#include <gpiote_nrfx.h>          /* Zephyr's exported GPIOTE driver instances */
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_spim.h>

#include "app_dt.h"
#include "sensor.h"
#include "spim_dppi.h"

LOG_MODULE_REGISTER(spim_dppi, LOG_LEVEL_INF);

/* ---- Instances from the devicetree ------------------------------------ */

static nrfx_spim_t spim = NRFX_SPIM_INSTANCE(DT_REG_ADDR(BUS_NODE));
PINCTRL_DT_DEFINE(BUS_NODE);

/* GPIOTE instance that serves the port of the sensor's data-ready pin */
static nrfx_gpiote_t *const gpiote = &GPIOTE_NRFX_INST_BY_NODE(INT_GPIOTE_NODE);

/* Event after which the .PTR register may be rewritten for the next transfer:
 * DMA.RX.READY on nRF54L ("EasyDMA has buffered the .PTR and .MAXCNT
 * registers"), STARTED on nRF53/52 ("can be updated immediately after having
 * received the STARTED event"). nrfx names the former RXSTARTED. */
#if NRF_SPIM_HAS_DMA_REG
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
 * slots of the ring, on its own. Two ways to move the slots into the message
 * queue:
 *  - time-drained (default): a thread wakes up every APP_DRAIN_PERIOD_US,
 *    reads the DMA pointer, pushes the slots that arrived (except the one
 *    that may still be in flight) and, once the DMA is past half of the
 *    ring, arms the wrap;
 *  - per-sample (APP_PER_SAMPLE_IRQ): one buffer instead of the ring; the
 *    SPIM END interrupt copies each burst into the queue as soon as its
 *    transaction has ended. One interrupt per sample, no drain, no wrap.
 * The wrap (time-drained only) is the READY interrupt, enabled once,
 * rewriting the pointer to slot 0 inside the window the datasheet allows
 * (right after READY/STARTED, before the next START).
 *
 * GUARD slots follow the ring so that a drain that runs late overflows into
 * unused memory (counted as overflow) instead of whatever follows the array.
 */
#if defined(CONFIG_APP_PER_SAMPLE_IRQ)
#define RING_SLOTS  1          /* one buffer, see below */
#define GUARD_SLOTS 0
#else
#define RING_SLOTS  CONFIG_APP_RING_SLOTS
#define GUARD_SLOTS 8
#endif
/* The wrap is armed once the DMA is this far into the ring, so the slots of
 * the old lap still to be delivered are at least half a ring ahead of the
 * new lap. Hence the sizing rule: ring >= 2 x rate x drain period. */
#define WRAP_MIN_HEAD (RING_SLOTS / 2)

K_MSGQ_DEFINE(sample_q, SENSOR_BURST_LEN, CONFIG_APP_QUEUE_DEPTH, 1);

static uint8_t ring[RING_SLOTS + GUARD_SLOTS][SENSOR_BURST_LEN];
/* EasyDMA source: the backend's TX prefix copied to RAM (EasyDMA cannot read flash) */
static uint8_t burst_tx[SENSOR_BURST_LEN];

static uint32_t dropped;      /* samples that did not fit in the queue */
#if !defined(CONFIG_APP_PER_SAMPLE_IRQ)
static uint32_t late_wraps;   /* wraps written after the next START (slot k+1 used) */
static uint32_t overflows;    /* laps in which the DMA reached the guard slots */
static uint32_t lap_base;     /* transactions completed in previous laps (for xfers) */
#endif
#if defined(CONFIG_APP_QUEUE_FRESH_ONLY)
static uint32_t skipped;      /* repeated samples dropped by the fresh filter */
#endif
#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
/* Trigger-timer ticks (16 MHz) from the trigger COMPARE to the delivering ISR */
static uint32_t lat_min = UINT32_MAX, lat_max, lat_sum, lat_n;
#endif

/* Delivery state, only touched by the drain thread (or the END ISR) and the
 * wrap ISR */
#if !defined(CONFIG_APP_PER_SAMPLE_IRQ)
static uint32_t tail;              /* next slot to deliver */
static bool in_guard;              /* the DMA is past the ring end (overflow counted) */
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
#endif

#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
static inline void latency_record(void)
{
	/* The trigger timer was cleared by the COMPARE that started the transaction */
	uint32_t lt = nrfx_timer_capture(&timer_trig, NRF_TIMER_CC_CHANNEL1);

	lat_min = MIN(lat_min, lt);
	lat_max = MAX(lat_max, lt);
	lat_sum += lt;
	lat_n++;
}
#endif

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

#if !defined(CONFIG_APP_PER_SAMPLE_IRQ)
/*
 * Wrap ISR: READY of transaction k just fired, the register already points
 * at slot k+1. Point it at slot 0 instead, so transaction k+1 writes slot 0.
 * If a START sneaks in between the clear and the write, transaction k+1 had
 * already taken slot k+1 (in the ring or in the guard) and slot 0 goes to
 * k+2: the sample is not lost, the wrap was late (counted).
 */
static void wrap_isr(void)
{
	uint32_t h, h2;
	bool started;

#if defined(CONFIG_APP_WRAP_LATENCY_STATS) && !defined(CONFIG_APP_PER_SAMPLE_IRQ)
	latency_record();
#endif
	nrf_spim_int_disable(spim.p_reg, READY_INT_MASK);
	nrf_spim_event_clear(spim.p_reg, READY_EVENT);
	h = head_index();                                   /* k + 1 */
	nrf_spim_rx_buffer_set(spim.p_reg, ring[0], SENSOR_BURST_LEN);
	started = nrf_spim_event_check(spim.p_reg, READY_EVENT);
	h2 = head_index();
	if (started && h2 == 0) {
		/* The START came before the write: it latched slot k+1 (= h) */
		late_wraps++;
		wrap_last = h;
	} else {
		/* No START before the write (a START right after it already
		 * latched slot 0 and shows as h2 == 1: the new lap began) */
		wrap_last = h - 1;
	}
	wrap_armed = false;
	wrap_done = true;
}

/* Enable the READY interrupt once the DMA is deep enough into the ring */
static bool arm_wrap(uint32_t h)
{
	if (wrap_armed || wrap_done || h < WRAP_MIN_HEAD) {
		return false;
	}
	wrap_armed = true;
	nrf_spim_event_clear(spim.p_reg, READY_EVENT);
	nrf_spim_int_enable(spim.p_reg, READY_INT_MASK);
	return true;
}

static void deliver(uint32_t from, uint32_t to)
{
	to = MIN(to, RING_SLOTS + GUARD_SLOTS);
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

static void overflow_check(uint32_t h)
{
	if (h > RING_SLOTS) {
		if (!in_guard) {
			in_guard = true;
			overflows++;
		}
	} else {
		in_guard = false;
	}
}
#endif /* !CONFIG_APP_PER_SAMPLE_IRQ */

#if defined(CONFIG_APP_PER_SAMPLE_IRQ)

/* ---- Per-sample delivery from the SPIM END interrupt -------------------- */

/*
 * One buffer, no ring, no wrap: the END ISR copies the burst into the queue
 * before the next transaction overwrites it. That holds while the sample
 * period exceeds the transaction time plus the ISR latency (~17 us from
 * idle on the nRF54L15 M33, 14-25 us on the nRF5340) plus the copy. A START
 * during the copy is detected (READY event set after being cleared) and the
 * sample is dropped as torn; a START before the ISR entry looks like the
 * normal one and cannot be told apart, so above that rate the data may be
 * stale: use the drain instead.
 */
static uint32_t ends;      /* END interrupts served (= transactions unless late) */
static uint32_t torn;      /* samples overwritten while being copied */

static void sample_isr(void)
{
	uint8_t burst[SENSOR_BURST_LEN];

#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
	latency_record();
#endif
	ends++;
	nrf_spim_event_clear(spim.p_reg, READY_EVENT);
	memcpy(burst, ring[0], SENSOR_BURST_LEN);
	if (nrf_spim_event_check(spim.p_reg, READY_EVENT)) {
		torn++;                /* the next transaction started mid-copy */
		return;
	}
#if defined(CONFIG_APP_QUEUE_FRESH_ONLY)
	if ((burst[sensor->fresh_offset] & sensor->fresh_mask) == 0) {
		skipped++;
		return;
	}
#endif
	if (k_msgq_put(&sample_q, burst, K_NO_WAIT) != 0) {
		dropped++;
	}
}

#else /* time-drained */

/* ---- Drain: ring -> message queue, then arm the wrap -------------------- */

/*
 * Called every APP_DRAIN_PERIOD_US from the drain thread. Slot h-1 may be in
 * flight, so only slots below h-1 are delivered; the remainder goes out on
 * the next drain. After a wrap, the lap that ended at wrap_last is finished
 * first, then the new lap from slot 0.
 */
static void drain(void)
{
	uint32_t h = head_index();
	uint32_t arrived = 0;

	overflow_check(h);
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
	if (arm_wrap(h) && arrived > 0 && CONFIG_APP_WRAP_AWAKE_BELOW_US > 0) {
		/* The wrap must land before the START that follows the next READY.
		 * Waking the core from idle for that IRQ costs ~17 us on the
		 * nRF54L15 M33 (RRAM) and more than 14-25 us on the nRF5340: when
		 * the samples are closer than that, stay awake for it instead. At
		 * low rates the IRQ path is used and the core sleeps. */
		uint32_t period_us = CONFIG_APP_DRAIN_PERIOD_US / arrived;

		if (period_us < CONFIG_APP_WRAP_AWAKE_BELOW_US) {
			uint32_t limit_us = MIN(CONFIG_APP_DRAIN_PERIOD_US / 4, 8 * period_us + 8);
			uint32_t t0 = k_cycle_get_32();
			uint32_t limit = k_us_to_cyc_ceil32(limit_us);

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

/* Cooperative priority: a drain is never preempted, so its delivery cannot
 * be delayed past the ring by application threads (only by ISRs) */
K_THREAD_DEFINE(drain_tid, 1024, drain_thread, NULL, NULL, NULL, -1, 0, 0);

#endif /* CONFIG_APP_PER_SAMPLE_IRQ */

/* The SPIM IRQ belongs to the nrfx driver during the blocking phase and to
 * the wrap (and, per-sample, the END delivery) afterwards. */
static void spim_irq_wrapper(const void *arg)
{
	if (blocking_phase) {
		nrfx_spim_irq_handler((nrfx_spim_t *)arg);
		return;
	}
#if defined(CONFIG_APP_PER_SAMPLE_IRQ)
	if (nrf_spim_event_check(spim.p_reg, NRF_SPIM_EVENT_END)) {
		nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_END);
		sample_isr();
	}
#else
	if (wrap_armed && nrf_spim_event_check(spim.p_reg, READY_EVENT)) {
		wrap_isr();
	}
	/* Anything else (e.g. END left enabled by the driver): silence it */
	nrf_spim_int_disable(spim.p_reg, ~READY_INT_MASK);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_STARTED);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_END);
#endif
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

#if defined(TRIG_TIMER_NODE)
static void timer_noop_handler(nrf_timer_event_t event_type, void *p_context)
{
	ARG_UNUSED(event_type);
	ARG_UNUSED(p_context);
}
#endif

/* ---- Init ---------------------------------------------------------------- */

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
#if defined(INT_PIN_ABS)
	/* Route data-ready to the sensor's interrupt pin (one pulse/level per sample) */
	err = sensor->enable_drdy_int();
	if (err) {
		return err;
	}
#endif
	return 0;
}

/* ---- Statistics ---------------------------------------------------------- */

uint32_t spim_dppi_total_xfers(void)
{
#if defined(CONFIG_APP_PER_SAMPLE_IRQ)
	return ends;
#else
	/* Transactions started: previous laps plus the current head. Between the
	 * wrap ISR and its drain the head already restarted from 0, so add the
	 * finished lap that has not been folded into lap_base yet. */
	uint32_t h = head_index();

	return lap_base + h + (wrap_done ? wrap_last + 1 : 0);
#endif
}

uint32_t spim_dppi_torn(void)
{
#if defined(CONFIG_APP_PER_SAMPLE_IRQ)
	return torn;
#else
	return 0;
#endif
}

uint32_t spim_dppi_dropped(void)
{
	return dropped;
}

uint32_t spim_dppi_late_wraps(void)
{
#if defined(CONFIG_APP_PER_SAMPLE_IRQ)
	return 0;
#else
	return late_wraps;
#endif
}

uint32_t spim_dppi_overflows(void)
{
#if defined(CONFIG_APP_PER_SAMPLE_IRQ)
	return 0;
#else
	return overflows;
#endif
}

#if defined(CONFIG_APP_QUEUE_FRESH_ONLY) || defined(TRIG_TIMER_NODE)
uint32_t spim_dppi_skipped(void)
{
#if defined(CONFIG_APP_QUEUE_FRESH_ONLY)
	return skipped;
#else
	return 0;
#endif
}
#endif

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

#if defined(TRIG_TIMER_NODE)
void spim_dppi_set_period_us(uint32_t period_us)
{
	nrfx_timer_disable(&timer_trig);
	nrfx_timer_clear(&timer_trig);
	nrfx_timer_compare(&timer_trig, NRF_TIMER_CC_CHANNEL0,
			   nrfx_timer_us_to_ticks(&timer_trig, period_us), false);
	nrfx_timer_enable(&timer_trig);
}
#endif

/* ---- Trigger (GPIOTE IN event on the data-ready pin), DPPI, start ------ */

static int trigger_init(uint32_t *eep)
{
	static uint8_t ch;
	int err;

	/* With CONFIG_GPIO=y gpio_nrfx already initialized (and owns) this instance */
	if (!nrfx_gpiote_init_check(gpiote)) {
		err = nrfx_gpiote_init(gpiote, 0);
		if (err < 0) {
			return err;
		}
	}
	err = nrfx_gpiote_channel_alloc(gpiote, &ch);
	if (err < 0) {
		return err;
	}
	static const nrf_gpio_pin_pull_t pull = NRF_GPIO_PIN_NOPULL;
	nrfx_gpiote_trigger_config_t trig = {
		.trigger = INT_ACTIVE_LOW ? NRFX_GPIOTE_TRIGGER_HITOLO : NRFX_GPIOTE_TRIGGER_LOTOHI,
		.p_in_channel = &ch,
	};
	nrfx_gpiote_input_pin_config_t in = {
		.p_pull_config = &pull,
		.p_trigger_config = &trig,
		.p_handler_config = NULL,   /* event only, no CPU interrupt */
	};
	err = nrfx_gpiote_input_configure(gpiote, INT_PIN_ABS, &in);
	if (err < 0) {
		return err;
	}
	nrfx_gpiote_trigger_enable(gpiote, INT_PIN_ABS, false);
	*eep = nrfx_gpiote_in_event_address_get(gpiote, INT_PIN_ABS);
	LOG_INF("trigger: %s data-ready on pin %d, %s edge -> GPIOTE IN event", sensor->name,
		INT_PIN_ABS, INT_ACTIVE_LOW ? "falling" : "rising");
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
			 NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;

	if (!IS_ENABLED(CONFIG_APP_PER_SAMPLE_IRQ)) {
		flags |= NRFX_SPIM_FLAG_RX_POSTINC;   /* array list: one slot per transaction */
	}
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

	/* Data-ready is a level: it is already high (nobody read the data since
	 * measurement started) so no rising edge would ever come. One software
	 * START reads and clears it; every next sample raises a fresh edge. */
	nrf_spim_task_trigger(spim.p_reg, NRF_SPIM_TASK_START);
#if defined(CONFIG_APP_PER_SAMPLE_IRQ)
	nrf_spim_int_enable(spim.p_reg, NRF_SPIM_INT_END_MASK);
	LOG_INF("DPPI connected, burst %u bytes, one buffer, one END interrupt per sample",
		SENSOR_BURST_LEN);
#else
	LOG_INF("DPPI connected, burst %u bytes, ring %u slots, drain every %u us, wrap on %s",
		SENSOR_BURST_LEN, RING_SLOTS, CONFIG_APP_DRAIN_PERIOD_US, READY_EVENT_NAME);
#endif
	return 0;
}

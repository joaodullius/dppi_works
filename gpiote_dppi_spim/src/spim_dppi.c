#include <string.h>
#include <errno.h>

#include <zephyr/kernel.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/pinctrl.h>

#include <nrfx_spim.h>
#include <nrfx_timer.h>
#include <nrfx_egu.h>
#include <nrfx_gpiote.h>
#include <gpiote_nrfx.h>          /* Zephyr's exported GPIOTE driver instances */
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

static nrfx_timer_t timer_cnt = NRFX_TIMER_INSTANCE(DT_REG_ADDR(CNT_TIMER_NODE));
#if defined(CONFIG_APP_CONSUME_QUEUE)
static nrfx_egu_t egu = NRFX_EGU_INSTANCE(DT_REG_ADDR(EGU_NODE));
#endif
/* GPIOTE instance that serves the port of the sensor's data-ready pin */
static nrfx_gpiote_t *const gpiote = &GPIOTE_NRFX_INST_BY_NODE(INT_GPIOTE_NODE);

/* Event counted per transaction: the one after which .PTR may be rewritten */
#if NRF_SPIM_HAS_DMA_REG
#define CNT_EVENT      NRF_SPIM_EVENT_RXSTARTED   /* DMA.RX.READY (nRF54L) */
#define CNT_EVENT_NAME "DMA.RX.READY"
#else
#define CNT_EVENT      NRF_SPIM_EVENT_STARTED
#define CNT_EVENT_NAME "STARTED"
#endif

/* ---- Buffers ------------------------------------------------------------ */

#if defined(CONFIG_APP_CONSUME_QUEUE)
/* Ping-pong of 2N slots plus N slots of slack: if the wrap ISR (COMPARE1)
 * runs later than one trigger period, the array list keeps writing past the
 * second block instead of corrupting whatever follows the buffer. */
#define RING_SLOTS (3 * CONFIG_APP_BLOCK_SAMPLES)
K_MSGQ_DEFINE(sample_q, SENSOR_BURST_LEN, CONFIG_APP_QUEUE_DEPTH, 1);
static uint32_t dropped;
static uint32_t late_wraps;
static uint32_t cycles_done;   /* completed 2N-slot ring cycles */
#else
#define RING_SLOTS 1
#endif

/* EasyDMA target: RING_SLOTS consecutive bursts (array list increments PTR by MAXCNT) */
static uint8_t ring[RING_SLOTS][SENSOR_BURST_LEN];
/* EasyDMA source: the backend's TX prefix copied to RAM (EasyDMA cannot read flash) */
static uint8_t burst_tx[SENSOR_BURST_LEN];

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

/* The SPIM IRQ belongs to the nrfx driver only during the blocking phase.
 * In the repeated phase no SPIM interrupt is needed (the END counter does the
 * work), so everything is disabled; this branch only exists for safety. */
static void spim_irq_wrapper(const void *arg)
{
	if (blocking_phase) {
		nrfx_spim_irq_handler((nrfx_spim_t *)arg);
	} else {
		nrf_spim_int_disable(spim.p_reg, 0xFFFFFFFF);
		nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_STARTED);
		nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_END);
	}
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

static void __unused timer_irq_wrapper(const void *arg)
{
	nrfx_timer_irq_handler((nrfx_timer_t *)arg);
}

static void __unused timer_noop_handler(nrf_timer_event_t event_type, void *p_context)
{
	ARG_UNUSED(event_type);
	ARG_UNUSED(p_context);
}

int spim_dppi_init(void)
{
	int err;

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
	/* Route data-ready to the sensor's interrupt pin (one pulse/level per sample) */
	err = sensor->enable_drdy_int();
	if (err) {
		return err;
	}
	return 0;
}

/* ---- Counter of SPIM END events (and block interrupts in QUEUE mode) --- */

#if defined(CONFIG_APP_CONSUME_QUEUE)
/* Current EasyDMA RX pointer (array list mode advances it after each transfer) */
static inline uint32_t spim_rx_ptr_get(void)
{
#if NRF_SPIM_HAS_DMA_REG
	return spim.p_reg->DMA.RX.PTR;
#else
	return spim.p_reg->RXD.PTR;
#endif
}

/*
 * The counter counts the event that marks the pointer registers as free
 * (not END): slot k of a ring cycle is in flight when the count is k+1. The
 * array-list pointer register is double-buffered and the hardware rewrites
 * it (PTR += MAXCNT) at every START, so the only safe moment for the CPU to
 * write it is right after that, with the whole transaction (>= 11 us here)
 * as margin. A write made around END, as an END-counted design does, can
 * coincide with the next START and the hardware update: the pointer is then
 * torn and the DMA writes elsewhere (bus/MPU fault at 15 us on the nRF5340).
 *
 * nRF54L: DMA.RX.READY ("EasyDMA has buffered the .PTR and .MAXCNT
 * registers, allowing them to be written to prepare for the next sequence")
 * is the literal definition of that window; nrfx names it RXSTARTED.
 * nRF53/52: no such event, STARTED ("can be updated immediately after
 * having received the STARTED event") is the reference.
 *
 *   COMPARE1 = 2N   START of slot 2N-1  -> wrap: PTR = ring[0] (this ISR,
 *                                          zero-latency), short CLEAR
 *   COMPARE0 = N+1  START of slot N     -> block A complete -> EGU T0
 *   COMPARE2 = 1    START of slot 0     -> block B complete -> EGU T1
 *                                          (skipped on the very first cycle)
 */
static void cnt_handler(nrf_timer_event_t event_type, void *p_context)
{
	ARG_UNUSED(p_context);

	if (event_type == NRF_TIMER_EVENT_COMPARE1) {
		/* Slot 2N-1 just started and the register already points at slot
		 * 2N (slack). If it points further, the next START happened before
		 * this ISR ran (one full transaction late): those samples went to
		 * the slack area, not over memory. */
		if (spim_rx_ptr_get() != (uint32_t)ring[2 * CONFIG_APP_BLOCK_SAMPLES]) {
			late_wraps++;
		}
		nrf_spim_rx_buffer_set(spim.p_reg, ring[0], SENSOR_BURST_LEN);
		cycles_done++;
	}
}

/*
 * EGU ISR (normal priority): COMPARE0/COMPARE2 reach the EGU through DPPI, so
 * the queue work runs here without holding up the wrap above.
 */
static void egu_handler(uint8_t event_idx, void *p_context)
{
	ARG_UNUSED(p_context);
	static bool first_cycle = true;
	const uint8_t *block = event_idx == 0 ? ring[0] : ring[CONFIG_APP_BLOCK_SAMPLES];

	if (event_idx == 1 && first_cycle) {
		/* COMPARE2 = 1 also fires at the very first transaction */
		first_cycle = false;
		return;
	}

	for (int i = 0; i < CONFIG_APP_BLOCK_SAMPLES; i++) {
		const uint8_t *s = block + i * SENSOR_BURST_LEN;

		/* One transaction per data-ready edge: every burst is a new sample
		 * (the consumer checks the STATUS bit anyway, as a sanity count). */
		if (k_msgq_put(&sample_q, s, K_NO_WAIT) != 0) {
			dropped++;
		}
	}
}

static void egu_irq_wrapper(const void *arg)
{
	nrfx_egu_irq_handler((nrfx_egu_t *)arg);
}

#if defined(CONFIG_ZERO_LATENCY_IRQS)
/* Zero-latency IRQs must be direct ISRs (no kernel entry/exit, no scheduling) */
ISR_DIRECT_DECLARE(cnt_direct_isr)
{
	nrfx_timer_irq_handler(&timer_cnt);
	return 0;
}
#endif
#endif

static int counter_init(void)
{
	nrfx_timer_config_t cfg = NRFX_TIMER_DEFAULT_CONFIG(NRFX_MHZ_TO_HZ(1));
	int err;

	cfg.mode = NRF_TIMER_MODE_COUNTER;
	cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
#if defined(CONFIG_APP_CONSUME_QUEUE)
#if defined(CONFIG_ZERO_LATENCY_IRQS)
	IRQ_DIRECT_CONNECT(DT_IRQN(CNT_TIMER_NODE), 0, cnt_direct_isr, IRQ_ZERO_LATENCY);
#else
	IRQ_CONNECT(DT_IRQN(CNT_TIMER_NODE), DT_IRQ(CNT_TIMER_NODE, priority),
		    timer_irq_wrapper, &timer_cnt, 0);
#endif
	err = nrfx_timer_init(&timer_cnt, &cfg, cnt_handler);
	if (err < 0) {
		return err;
	}
	/* Only COMPARE1 (wrap) interrupts; block-complete compares go to the EGU */
	nrfx_timer_extended_compare(&timer_cnt, NRF_TIMER_CC_CHANNEL0, CONFIG_APP_BLOCK_SAMPLES + 1,
				    0, false);
	nrfx_timer_extended_compare(&timer_cnt, NRF_TIMER_CC_CHANNEL1, 2 * CONFIG_APP_BLOCK_SAMPLES,
				    NRF_TIMER_SHORT_COMPARE1_CLEAR_MASK, true);
	nrfx_timer_extended_compare(&timer_cnt, NRF_TIMER_CC_CHANNEL2, 1, 0, false);

	/* Queue work: block-complete events reach the EGU through DPPI */
	IRQ_CONNECT(DT_IRQN(EGU_NODE), DT_IRQ(EGU_NODE, priority), egu_irq_wrapper, &egu, 0);
	err = nrfx_egu_init(&egu, 0 /* priority set by IRQ_CONNECT */, egu_handler, NULL);
	if (err < 0) {
		return err;
	}
	nrfx_egu_int_enable(&egu, NRF_EGU_INT_TRIGGERED0 | NRF_EGU_INT_TRIGGERED1);
#else
	IRQ_CONNECT(DT_IRQN(CNT_TIMER_NODE), DT_IRQ(CNT_TIMER_NODE, priority),
		    timer_irq_wrapper, &timer_cnt, 0);
	err = nrfx_timer_init(&timer_cnt, &cfg, timer_noop_handler);
	if (err < 0) {
		return err;
	}
#endif
	nrfx_timer_clear(&timer_cnt);
	nrfx_timer_enable(&timer_cnt);
	return 0;
}

uint32_t spim_dppi_total_xfers(void)
{
	/* Transactions started (the one in flight, if any, included) */
	uint32_t in_cycle = nrfx_timer_capture(&timer_cnt, NRF_TIMER_CC_CHANNEL3);
#if defined(CONFIG_APP_CONSUME_QUEUE)
	return cycles_done * 2 * CONFIG_APP_BLOCK_SAMPLES + in_cycle;
#else
	return in_cycle;
#endif
}

#if defined(CONFIG_APP_CONSUME_QUEUE)
uint32_t spim_dppi_dropped(void)
{
	return dropped;
}

uint32_t spim_dppi_late_wraps(void)
{
	return late_wraps;
}
#endif

#if defined(CONFIG_APP_CONSUME_LATEST)
bool spim_dppi_latest(uint8_t out[SENSOR_BURST_LEN])
{
	/* A transaction may be rewriting the slot: take two copies bracketed by
	 * the END counter and accept only when nothing completed in between and
	 * both copies match. */
	for (int attempt = 0; attempt < 4; attempt++) {
		uint32_t before = spim_dppi_total_xfers();
		uint8_t a[SENSOR_BURST_LEN], b[SENSOR_BURST_LEN];

		memcpy(a, ring[0], SENSOR_BURST_LEN);
		memcpy(b, ring[0], SENSOR_BURST_LEN);
		if (spim_dppi_total_xfers() == before && memcmp(a, b, SENSOR_BURST_LEN) == 0) {
			memcpy(out, a, SENSOR_BURST_LEN);
			return true;
		}
	}
	return false;
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
	nrfx_gppi_handle_t h_start, h_count;
	uint32_t eep;
	int err;

	err = counter_init();
	if (err) {
		LOG_ERR("counter init: %d", err);
		return err;
	}
	err = trigger_init(&eep);
	if (err) {
		LOG_ERR("trigger init: %d", err);
		return err;
	}

	/* Repeated burst: the driver sets the buffers once and steps aside */
	blocking_phase = false;
	memcpy(burst_tx, sensor->burst_tx, sensor->burst_tx_len);
	nrfx_spim_xfer_desc_t xfer = {
		.p_tx_buffer = burst_tx, .tx_length = sensor->burst_tx_len,
		.p_rx_buffer = ring[0], .rx_length = SENSOR_BURST_LEN,
	};
	uint32_t flags = NRFX_SPIM_FLAG_HOLD_XFER | NRFX_SPIM_FLAG_REPEATED_XFER |
			 NRFX_SPIM_FLAG_NO_XFER_EVT_HANDLER;
#if defined(CONFIG_APP_CONSUME_QUEUE)
	flags |= NRFX_SPIM_FLAG_RX_POSTINC;   /* EasyDMA array list on RX */
#endif
	err = nrfx_spim_xfer(&spim, &xfer, flags);
	if (err < 0) {
		LOG_ERR("arming repeated transfer: %d", err);
		return err;
	}
	/* From here on the peripheral is driven by DPPI only. The driver may have
	 * left STARTED (nRF54L errata workaround) or END interrupts enabled; with
	 * nobody servicing them they would storm. */
	nrf_spim_int_disable(spim.p_reg, 0xFFFFFFFF);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_STARTED);
	nrf_spim_event_clear(spim.p_reg, NRF_SPIM_EVENT_END);

	/* trigger event -> SPIM START ; SPIM STARTED / DMA.RX.READY -> counter COUNT */
	err = nrfx_gppi_conn_alloc(eep, nrfx_spim_start_task_address_get(&spim), &h_start);
	if (err < 0) {
		return err;
	}
	err = nrfx_gppi_conn_alloc(nrf_spim_event_address_get(spim.p_reg, CNT_EVENT),
				   nrf_timer_task_address_get(timer_cnt.p_reg, NRF_TIMER_TASK_COUNT),
				   &h_count);
	if (err < 0) {
		return err;
	}
#if defined(CONFIG_APP_CONSUME_QUEUE)
	/* counter COMPARE0/2 (block A/B complete) -> EGU TRIGGER0/1 -> queue ISR */
	nrfx_gppi_handle_t h_blk0, h_blk1;

	err = nrfx_gppi_conn_alloc(
		nrfx_timer_compare_event_address_get(&timer_cnt, NRF_TIMER_CC_CHANNEL0),
		nrfx_egu_task_address_get(&egu, NRF_EGU_TASK_TRIGGER0), &h_blk0);
	if (err < 0) {
		return err;
	}
	err = nrfx_gppi_conn_alloc(
		nrfx_timer_compare_event_address_get(&timer_cnt, NRF_TIMER_CC_CHANNEL2),
		nrfx_egu_task_address_get(&egu, NRF_EGU_TASK_TRIGGER1), &h_blk1);
	if (err < 0) {
		return err;
	}
	nrfx_gppi_conn_enable(h_blk0);
	nrfx_gppi_conn_enable(h_blk1);
#endif
	nrfx_gppi_conn_enable(h_count);
	nrfx_gppi_conn_enable(h_start);

	/* Data-ready is a level: it is already high (nobody read the data since
	 * measurement started) so no rising edge would ever come. One software
	 * START reads and clears it; every next sample raises a fresh edge. */
	nrf_spim_task_trigger(spim.p_reg, NRF_SPIM_TASK_START);
	LOG_INF("DPPI connected, %s consumption, burst %u bytes, counting %s%s",
		IS_ENABLED(CONFIG_APP_CONSUME_QUEUE) ? "queue (wrap ISR + EGU)" : "latest",
		SENSOR_BURST_LEN, CNT_EVENT_NAME,
		IS_ENABLED(CONFIG_ZERO_LATENCY_IRQS) ? ", wrap ISR zero-latency" : "");
	return 0;
}

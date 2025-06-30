#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/timing/timing.h>

#include <nrfx_timer.h>
#include <nrfx_gpiote.h>

// GPPI + PPI/DPPI includes
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
    #include <nrfx_dppi.h>
#else
    #include <nrfx_ppi.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

/* The devicetree node identifier for the "led0" alias. */
#define LED0_PIN	DT_GPIO_PIN(DT_ALIAS(led0), gpios)

/* GPIOTE instance for NCS 2.7.0 */
static const nrfx_gpiote_t p_gpiote_inst = NRFX_GPIOTE_INSTANCE(0);
static const nrfx_gpiote_t* p_gpiote = &p_gpiote_inst;

/* Timer instance */
static const nrfx_timer_t m_sample_timer = NRFX_TIMER_INSTANCE(1);

void timer_handler(nrf_timer_event_t event_type, void* p_context)
{
	// Nothing here due to DPPI/PPI
}

static void timer_init(void)
{
    nrfx_timer_config_t timer_config = {
        .frequency          = NRF_TIMER_FREQ_1MHz,
        .mode               = NRF_TIMER_MODE_TIMER,
        .bit_width          = NRF_TIMER_BIT_WIDTH_32,
        .interrupt_priority = 6,
        .p_context          = NULL,
    };

    nrfx_err_t err_code = nrfx_timer_init(&m_sample_timer, &timer_config, timer_handler);
    if (err_code != NRFX_SUCCESS) {
        LOG_ERR("nrfx_timer_init error: %08x", err_code);
        return;
    }

    nrfx_timer_extended_compare(&m_sample_timer,
                                NRF_TIMER_CC_CHANNEL0,
                                nrfx_timer_ms_to_ticks(&m_sample_timer, 500),
                                NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                true);

    LOG_INF("Timer 1 initialized.");
    nrfx_timer_enable(&m_sample_timer);

    IRQ_CONNECT(TIMER1_IRQn, 0, nrfx_timer_1_irq_handler, NULL, 0);
}

static void gpiote_init(void)
{
	nrfx_err_t err;
	uint8_t out_channel;


	#ifndef CONFIG_GPIO
	err = nrfx_gpiote_init(p_gpiote, 0);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_init error: 0x%08X", err);
		return;
	}
	#endif

	err = nrfx_gpiote_channel_alloc(p_gpiote, &out_channel);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to allocate out_channel, error: 0x%08X", err);
		return;
	}

	static const nrfx_gpiote_output_config_t output_config = {
		.drive = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull = NRF_GPIO_PIN_NOPULL,
	};
	const nrfx_gpiote_task_config_t task_config = {
		.task_ch = out_channel,
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = 1,
	};

	err = nrfx_gpiote_output_configure(p_gpiote, LED0_PIN, &output_config, &task_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_output_configure error: 0x%08X", err);
		return;
	}

	nrfx_gpiote_out_task_enable(p_gpiote, LED0_PIN);
	LOG_INF("nrfx_gpiote initialized");
}

static void gppi_init(void)
{
	nrfx_err_t err;
	uint8_t ppi_channel;

	err = nrfx_gppi_channel_alloc(&ppi_channel);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gppi_channel_alloc error: 0x%08X", err);
		return;
	}

	nrfx_gppi_channel_endpoints_setup(ppi_channel,
		nrf_timer_event_address_get(m_sample_timer.p_reg, NRF_TIMER_EVENT_COMPARE0),
		nrfx_gpiote_out_task_address_get(p_gpiote, LED0_PIN));

	nrfx_gppi_channels_enable(BIT(ppi_channel));
}

int main(void)
{
	#if defined(DPPI_PRESENT)
		LOG_INF("Starting Timer + DPPI + GPIOTE Application...");
	#else
		LOG_INF("Starting Timer + PPI + GPIOTE Application...");
	#endif

	timer_init();
	gpiote_init();
	gppi_init();
	return 0;
}

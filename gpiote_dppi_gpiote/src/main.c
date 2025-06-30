/*
 * Copyright (c) 2019 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/irq.h>

#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>

LOG_MODULE_REGISTER(nrfx_sample, LOG_LEVEL_INF);

#define INPUT_PIN	NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(sw0), gpios)
#define OUTPUT_PIN	NRF_DT_GPIOS_TO_PSEL(DT_ALIAS(led0), gpios)

#define GPIOTE_INST	NRF_DT_GPIOTE_INST(DT_ALIAS(sw0), gpios)
#define GPIOTE_NODE	DT_NODELABEL(_CONCAT(gpiote, GPIOTE_INST))

BUILD_ASSERT(NRF_DT_GPIOTE_INST(DT_ALIAS(led0), gpios) == GPIOTE_INST,
	"Both sw0 and led0 GPIOs must use the same GPIOTE instance");
BUILD_ASSERT(IS_ENABLED(_CONCAT(CONFIG_, _CONCAT(NRFX_GPIOTE, GPIOTE_INST))),
	"NRFX_GPIOTE" STRINGIFY(GPIOTE_INST) " must be enabled in Kconfig");

static void button_handler(nrfx_gpiote_pin_t pin,
			   nrfx_gpiote_trigger_t trigger,
			   void *context)
{
	LOG_INF("GPIO input event callback");
}

static nrfx_err_t configure_input_pin(const nrfx_gpiote_t * p_gpiote,
					nrfx_gpiote_pin_t input_pin,
					uint8_t *p_in_channel)
{
	nrfx_err_t err;

	/* Allocate a channel for the input pin */
	err = nrfx_gpiote_channel_alloc(p_gpiote, p_in_channel);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to allocate in_channel, error: 0x%08X", err);
		return err;
	}

	static const nrf_gpio_pin_pull_t pull_config = NRF_GPIO_PIN_PULLUP;
	nrfx_gpiote_trigger_config_t trigger_config = {
		.trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
		.p_in_channel = p_in_channel,
	};
	static const nrfx_gpiote_handler_config_t handler_config = {
		.handler = button_handler,
	};
	nrfx_gpiote_input_pin_config_t input_config = {
		.p_pull_config    = &pull_config,
		.p_trigger_config = &trigger_config,
		.p_handler_config = &handler_config
	};

	err = nrfx_gpiote_input_configure(p_gpiote, input_pin, &input_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_input_configure error: 0x%08X", err);
		return err;
	}

	/* Enable the input trigger */
	nrfx_gpiote_trigger_enable(p_gpiote, input_pin, true);
	return NRFX_SUCCESS;
}

static nrfx_err_t configure_output_pin(const nrfx_gpiote_t * p_gpiote,
					 nrfx_gpiote_pin_t output_pin,
					 uint8_t *p_out_channel)
{
	nrfx_err_t err;

	/* Allocate a channel for the output pin */
	err = nrfx_gpiote_channel_alloc(p_gpiote, p_out_channel);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to allocate out_channel, error: 0x%08X", err);
		return err;
	}

	static const nrfx_gpiote_output_config_t output_config = {
		.drive         = NRF_GPIO_PIN_S0S1,
		.input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
		.pull          = NRF_GPIO_PIN_NOPULL,
	};
	const nrfx_gpiote_task_config_t task_config = {
		.task_ch  = *p_out_channel,
		.polarity = NRF_GPIOTE_POLARITY_TOGGLE,
		.init_val = 1,
	};

	err = nrfx_gpiote_output_configure(p_gpiote, output_pin, &output_config,
					   &task_config);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_output_configure error: 0x%08X", err);
		return err;
	}

	/* Enable the output task */
	nrfx_gpiote_out_task_enable(p_gpiote, output_pin);
	return NRFX_SUCCESS;
}

static nrfx_err_t configure_dppi(const nrfx_gpiote_t * p_gpiote,
				 nrfx_gpiote_pin_t input_pin,
				 nrfx_gpiote_pin_t output_pin)
{
	nrfx_err_t err;
	uint8_t ppi_channel;

	/* Allocate a DPPI channel */
	err = nrfx_gppi_channel_alloc(&ppi_channel);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gppi_channel_alloc error: 0x%08X", err);
		return err;
	}

	/* Setup endpoints so that the input pin event triggers the output pin task */
	nrfx_gppi_channel_endpoints_setup(ppi_channel,
		nrfx_gpiote_in_event_address_get(p_gpiote, input_pin),
		nrfx_gpiote_out_task_address_get(p_gpiote, output_pin));

	/* Enable the DPPI channel */
	nrfx_gppi_channels_enable(BIT(ppi_channel));
	return NRFX_SUCCESS;
}

int main(void)
{
	LOG_INF("nrfx_gpiote sample on %s", CONFIG_BOARD);

	nrfx_err_t err;
	uint8_t in_channel, out_channel;
	const nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(GPIOTE_INST);

	/* Connect GPIOTE instance IRQ to the interrupt handler */
	IRQ_CONNECT(DT_IRQN(GPIOTE_NODE), DT_IRQ(GPIOTE_NODE, priority), nrfx_isr,
			NRFX_CONCAT(nrfx_gpiote_, GPIOTE_INST, _irq_handler), 0);

	err = nrfx_gpiote_init(&gpiote, 0);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gpiote_init error: 0x%08X", err);
		return 0;
	}

	err = configure_input_pin(&gpiote, INPUT_PIN, &in_channel);
	if (err != NRFX_SUCCESS) {
		return 0;
	}

	err = configure_output_pin(&gpiote, OUTPUT_PIN, &out_channel);
	if (err != NRFX_SUCCESS) {
		return 0;
	}

	err = configure_dppi(&gpiote, INPUT_PIN, OUTPUT_PIN);
	if (err != NRFX_SUCCESS) {
		return 0;
	}

	LOG_INF("nrfx_gpiote and DPPI configured, leaving main()");
	return 0;
}

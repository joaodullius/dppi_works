#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>

#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#if defined(DPPI_PRESENT)
#include <nrfx_dppi.h>
#else
#include <nrfx_ppi.h>
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_DBG);

#define LED0_PIN        DT_GPIO_PIN(DT_ALIAS(led0), gpios)
#define BUTTON0_PIN     DT_GPIO_PIN(DT_ALIAS(sw0), gpios)

static const nrfx_gpiote_t p_gpiote_inst = NRFX_GPIOTE_INSTANCE(0);
static const nrfx_gpiote_t *p_gpiote = &p_gpiote_inst;

static void gpiote_init(void)
{
    nrfx_err_t err;
    uint8_t out_channel;
    uint8_t in_channel;

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

    err = nrfx_gpiote_channel_alloc(p_gpiote, &in_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to allocate in_channel, error: 0x%08X", err);
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
        .init_val = 0,
    };

    err = nrfx_gpiote_output_configure(p_gpiote, LED0_PIN, &output_config, &task_config);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gpiote_output_configure error: 0x%08X", err);
        return;
    }

    static const nrfx_gpiote_input_config_t input_config = {
        .pull = NRF_GPIO_PIN_PULLUP,
    };
    const nrfx_gpiote_trigger_config_t trigger_config = {
        .trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
        .p_in_channel = &in_channel,
    };

    err = nrfx_gpiote_input_configure(p_gpiote, BUTTON0_PIN, &input_config, &trigger_config, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gpiote_input_configure error: 0x%08X", err);
        return;
    }

    nrfx_gpiote_out_task_enable(p_gpiote, LED0_PIN);
    nrfx_gpiote_trigger_enable(BUTTON0_PIN, true);

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
        nrfx_gpiote_in_event_address_get(p_gpiote, BUTTON0_PIN),
        nrfx_gpiote_out_task_address_get(p_gpiote, LED0_PIN));

    nrfx_gppi_channels_enable(BIT(ppi_channel));
}

int main(void)
{
#if defined(DPPI_PRESENT)
    LOG_INF("Starting GPIOTE -> DPPI -> GPIOTE Application...");
#else
    LOG_INF("Starting GPIOTE -> PPI -> GPIOTE Application...");
#endif

    gpiote_init();
    gppi_init();

    return 0;
}

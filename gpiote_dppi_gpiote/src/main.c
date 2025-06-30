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

static const nrfx_gpiote_t gpiote_inst = NRFX_GPIOTE_INSTANCE(0);
static const nrfx_gpiote_t *p_gpiote = &gpiote_inst;

static void gpiote_init(void)
{
    nrfx_err_t err;
    uint8_t led_channel;

#ifndef CONFIG_GPIO
    err = nrfx_gpiote_init(p_gpiote, 0);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gpiote_init error: 0x%08X", err);
        return;
    }
#endif

    err = nrfx_gpiote_channel_alloc(p_gpiote, &led_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to allocate LED channel: 0x%08X", err);
        return;
    }

    static const nrfx_gpiote_output_config_t out_cfg = {
        .drive = NRF_GPIO_PIN_S0S1,
        .input_connect = NRF_GPIO_PIN_INPUT_DISCONNECT,
        .pull = NRF_GPIO_PIN_NOPULL,
    };
    const nrfx_gpiote_task_config_t task_cfg = {
        .task_ch = led_channel,
        .polarity = NRF_GPIOTE_POLARITY_TOGGLE,
        .init_val = 0,
    };

    err = nrfx_gpiote_output_configure(p_gpiote, LED0_PIN, &out_cfg, &task_cfg);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gpiote_output_configure error: 0x%08X", err);
        return;
    }
    nrfx_gpiote_out_task_enable(p_gpiote, LED0_PIN);

    nrfx_gpiote_in_config_t in_cfg = NRFX_GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
    in_cfg.pull = NRF_GPIO_PIN_PULLUP;

    err = nrfx_gpiote_in_init(BUTTON0_PIN, &in_cfg, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gpiote_in_init error: 0x%08X", err);
        return;
    }
    nrfx_gpiote_in_event_enable(BUTTON0_PIN, true);

    LOG_INF("nrfx_gpiote initialized");
}

static void gppi_init(void)
{
    nrfx_err_t err;
    uint8_t dppi_channel;

    err = nrfx_gppi_channel_alloc(&dppi_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("nrfx_gppi_channel_alloc error: 0x%08X", err);
        return;
    }

    nrfx_gppi_channel_endpoints_setup(dppi_channel,
        nrfx_gpiote_in_event_address_get(p_gpiote, BUTTON0_PIN),
        nrfx_gpiote_out_task_address_get(p_gpiote, LED0_PIN));

    nrfx_gppi_channels_enable(BIT(dppi_channel));
}

int main(void)
{
#if defined(DPPI_PRESENT)
    LOG_INF("Starting GPIOTE -> DPPI -> GPIOTE application...");
#else
    LOG_INF("Starting GPIOTE -> PPI -> GPIOTE application...");
#endif

    gpiote_init();
    gppi_init();
    return 0;
}

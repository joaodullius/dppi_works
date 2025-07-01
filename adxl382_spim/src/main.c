#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_spim.h>
#include <hal/nrf_gpio.h>

LOG_MODULE_REGISTER(adxl382, LOG_LEVEL_INF);

// SPI3 hardware instance
#define SPIM_INST NRFX_SPIM_INSTANCE(3)

// SPI CS pin
#define ADXL382_CS_PIN 47

// ADXL382 register address for DEVID (Chip ID)
#define ADXL382_REG_DEVID 0x00

static const nrfx_spim_t spim = SPIM_INST;

static uint8_t tx_buf[] = { ADXL382_REG_DEVID | 0x80 }; // read bit = 1
static uint8_t rx_buf[2] = { 0 };

int main(void)
{
    LOG_INF("Starting ADXL382 SPI test");

    // Configure CS pin manually
    nrf_gpio_cfg_output(ADXL382_CS_PIN);
    nrf_gpio_pin_set(ADXL382_CS_PIN); // inactive

    nrfx_spim_config_t config = {
        .sck_pin      = 44,
        .mosi_pin     = 45,
        .miso_pin     = 46,
        .ss_pin       = NRF_SPIM_PIN_NOT_CONNECTED,
        .irq_priority = NRFX_SPIM_DEFAULT_CONFIG_IRQ_PRIORITY,
        .orc          = 0xFF,
        .frequency    = NRF_SPIM_FREQ_8M,
        .mode         = NRF_SPIM_MODE_0,
        .bit_order    = NRF_SPIM_BIT_ORDER_MSB_FIRST,
    };

	nrfx_err_t err = nrfx_spim_init(&spim, &config, NULL, NULL);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to init SPIM, error: 0x%08X", err);
		return -1;
	}

    nrf_gpio_pin_clear(ADXL382_CS_PIN); // assert CS

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length = 1,
        .p_rx_buffer = rx_buf,
        .rx_length = 2, // read 1 byte + dummy
    };

    if (nrfx_spim_xfer(&spim, &xfer, 0) != NRFX_SUCCESS) {
        LOG_ERR("SPI transfer failed");
    } else {
        LOG_INF("ADXL382 CHIP ID: 0x%02X", rx_buf[1]);
    }

    nrf_gpio_pin_set(ADXL382_CS_PIN); // deassert CS

	return 0;
}

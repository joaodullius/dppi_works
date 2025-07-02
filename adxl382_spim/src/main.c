#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_spim.h>
#include <hal/nrf_gpio.h>

LOG_MODULE_REGISTER(adxl382, LOG_LEVEL_INF);

// SPI3 hardware instance
#define SPIM_INST NRFX_SPIM_INSTANCE(3)

// SPI CS pin
#define ADXL382_CS_PIN 22
#define ADXL382_INT_PIN 19
#define SPIM_SCK 29
#define SPIM_MOSI 28
#define SPIM_MISO 26

// ADXL382 register address for DEVID (Chip ID)
#define ADXL362_CMD_WRITE_REG   0x0A
#define ADXL362_CMD_READ_REG    0x0B
#define ADXL362_REG_DEVID_AD    0x00

#define ADXL362_REG_POWER_CTL  0x2D
#define ADXL362_MEASURE_MODE   0x02

#define ADXL362_REG_XDATA_L   0x0E
#define ADXL362_REG_YDATA_L   0x10
#define ADXL362_REG_ZDATA_L   0x12

static const nrfx_spim_t spim = SPIM_INST;

int adxl362_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[3] = {
        ADXL362_CMD_READ_REG,
        reg,
        0x00 // Dummy
    };
    uint8_t rx_buf[3] = {0};

    LOG_HEXDUMP_DBG(tx_buf, sizeof(tx_buf), "TX Buffer");
    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "RX Buffer (before)");


    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length = sizeof(tx_buf),
        .p_rx_buffer = rx_buf,
        .rx_length = sizeof(rx_buf),
    };

    nrf_gpio_pin_clear(ADXL382_CS_PIN); // assert CS

    int ret = nrfx_spim_xfer(&spim, &xfer, 0);
    if (ret == NRFX_SUCCESS) {
        *value = rx_buf[2]; // O valor lido está na terceira posição
        LOG_DBG("*value = %xh", rx_buf[2]);
    }
    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "RX Buffer (after)");
    nrf_gpio_pin_set(ADXL382_CS_PIN); // deassert CS
    return NRFX_SUCCESS;
}

int main(void)
{

    int ret;
    uint8_t devid = 0;

    LOG_INF("Starting ADXL382 SPI test");

    // Configure CS pin manually
    nrf_gpio_cfg_output(ADXL382_CS_PIN);
    nrf_gpio_pin_set(ADXL382_CS_PIN); // inactive

    nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(SPIM_SCK,
                                                            SPIM_MOSI,
                                                            SPIM_MISO,
                                                            NRF_SPIM_PIN_NOT_CONNECTED);

	nrfx_err_t err = nrfx_spim_init(&spim, &config, NULL, NULL);
	if (err != NRFX_SUCCESS) {
		LOG_WRN("Failed to init SPIM, error: 0x%08X", err);
		return -1;
	}

    ret = adxl362_read_reg(ADXL362_REG_DEVID_AD, &devid);
    if (ret == NRFX_SUCCESS) {
        LOG_INF("ADXL362 Device ID: 0x%02X", devid);
        if (devid != 0xAD) {
            LOG_WRN("Unexpected device ID!");
        }
    } else {
        LOG_ERR("Failed to read from ADXL362 (err %d)", ret);
    }


	return 0;
}

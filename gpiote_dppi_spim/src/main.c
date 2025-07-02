#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <nrfx_spim.h>
#include <nrfx_gpiote.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>

LOG_MODULE_REGISTER(adxl382, LOG_LEVEL_INF);

// SPI3 hardware instance
#define SPIM_INST NRFX_SPIM_INSTANCE(3)

// SPI CS pin
#define ADXL382_CS_PIN 22
#define ADXL382_INT_PIN 19
#define SPIM_SCK 29
#define SPIM_MOSI 28
#define SPIM_MISO 26

// Button SW0 pin for Thingy53
#define BUTTON_SW0_PIN NRF_GPIO_PIN_MAP(1, 14)  // P1.14

// ADXL382 register address for DEVID (Chip ID)
#define ADXL362_CMD_WRITE_REG   0x0A
#define ADXL362_CMD_READ_REG    0x0B
#define ADXL362_REG_DEVID_AD    0x00

#define ADXL362_REG_POWER_CTL  0x2D
#define ADXL362_MEASURE_MODE   0x02

#define ADXL362_REG_XDATA_L   0x0E
#define ADXL362_REG_YDATA_L   0x10
#define ADXL362_REG_ZDATA_L   0x12

#define GRAVITY_M_S2 9.80665f

static const nrfx_spim_t spim = SPIM_INST;

// Global variable to track button press
static volatile bool button_pressed = false;


int16_t adxl362_read_accel(uint8_t reg_base)
{

   uint8_t tx_buf[3] = {
        ADXL362_CMD_READ_REG,
        reg_base,
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
        LOG_DBG("*value = %xh", rx_buf[2]);
    }
    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "RX Buffer (after)");
    nrf_gpio_pin_set(ADXL382_CS_PIN); // deassert CS

    return ret;
}


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
    return ret;
}


int adxl362_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[3] = {
        ADXL362_CMD_WRITE_REG,
        reg,
        value
    };
    uint8_t rx_buf[3] = {0};

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length = sizeof(tx_buf),
        .p_rx_buffer = rx_buf,
        .rx_length = sizeof(rx_buf),
    };

    LOG_HEXDUMP_DBG(tx_buf, sizeof(tx_buf), "TX Buffer");
    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "RX Buffer (before)");

    nrf_gpio_pin_clear(ADXL382_CS_PIN); // assert CS

    int ret = nrfx_spim_xfer(&spim, &xfer, 0);
    if (ret == NRFX_SUCCESS) {
        LOG_DBG("Wrote reg 0x%02X: 0x%02X", reg, value);
    }
    nrf_gpio_pin_set(ADXL382_CS_PIN); // deassert CS

    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "RX Buffer (after)");

    return ret;
}

int adxl362_init()
{
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
    LOG_INF("ADXL362 SPI initialized successfully");

    // Configure INT pin
    nrf_gpio_cfg_input(ADXL382_INT_PIN, NRF_GPIO_PIN_PULLUP);
    LOG_INF("ADXL362 INT pin configured");

    return 0;
}


// Button event handler
static void button_handler(nrfx_gpiote_pin_t pin,
                          nrfx_gpiote_trigger_t trigger,
                          void *context)
{
    button_pressed = true;
    LOG_INF("Button SW0 pressed - triggering accel read");
}

// Function to read accelerometer data
void read_accelerometer_data(void)
{
    int ret;
    uint8_t axl, axh, ayl, ayh, azl, azh;

    uint8_t tx_buf[8] = {
        ADXL362_CMD_READ_REG,
        ADXL362_REG_XDATA_L, // First register address
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // Dummy
    };
    uint8_t rx_buf[8] = {0};

    LOG_HEXDUMP_DBG(tx_buf, sizeof(tx_buf), "TX Buffer");

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length = sizeof(tx_buf),
        .p_rx_buffer = rx_buf,
        .rx_length = sizeof(rx_buf),
    };

    nrf_gpio_pin_clear(ADXL382_CS_PIN); // assert CS

    ret = nrfx_spim_xfer(&spim, &xfer, 0);
    if (ret == NRFX_SUCCESS) {
        axl = rx_buf[2]; // X low byte
        axh = rx_buf[3]; // X high byte
        ayl = rx_buf[4]; // Y low byte
        ayh = rx_buf[5]; // Y high byte
        azl = rx_buf[6]; // Z low byte
        azh = rx_buf[7]; // Z high byte
    }
    LOG_HEXDUMP_DBG(rx_buf, sizeof(rx_buf), "RX Buffer (after)");
    nrf_gpio_pin_set(ADXL382_CS_PIN); // deassert CS

    if (ret == NRFX_SUCCESS) {
        int16_t ax = ((int16_t)axh << 8) | axl;  // Combine bytes with correct sign extension
        float ax_mg = ax * 1.0f; // Convert to mg
        float ax_ms2 = ax_mg * GRAVITY_M_S2 / 1000.0f; // Convert to m/s^2

        int16_t ay = (int16_t)((ayh << 8) | ayl);  // Sign-extended by hardware
        float ay_mg = ay * 1.0f; // Convert to mg
        float ay_ms2 = ay_mg * GRAVITY_M_S2 / 1000.0f; // Convert to m/s^2

        int16_t az = (int16_t)((azh << 8) | azl);  // Sign-extended by hardware
        float az_mg = az * 1.0f; // Convert to mg
        float az_ms2 = az_mg * GRAVITY_M_S2 / 1000.0f; // Convert to m/s^2

        LOG_INF("Accel [m/s^2]: X=%.2f Y=%.2f Z=%.2f", (double)ax_ms2, (double)ay_ms2, (double)az_ms2);
    } else {
        LOG_ERR("Failed to read accelerometer data (err %d)", ret);
    }
}

// Function to initialize button
int button_init(void)
{
    nrfx_err_t err;
    uint8_t in_channel;
    static const nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);

    // Initialize GPIOTE if not already done
    #if !defined(CONFIG_GPIO)
    err = nrfx_gpiote_init(&gpiote, 0);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to initialize GPIOTE (err 0x%08X)", err);
        return -1;
    }
    #endif

    // Allocate a channel for the input pin
    err = nrfx_gpiote_channel_alloc(&gpiote, &in_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to allocate channel (err 0x%08X)", err);
        return -1;
    }

    // Configure button pin
    static const nrf_gpio_pin_pull_t pull_config = NRF_GPIO_PIN_PULLUP;
    nrfx_gpiote_trigger_config_t trigger_config = {
        .trigger = NRFX_GPIOTE_TRIGGER_HITOLO, // Button press (high to low)
        .p_in_channel = &in_channel,
    };
    static const nrfx_gpiote_handler_config_t handler_config = {
        .handler = button_handler,
    };
    nrfx_gpiote_input_pin_config_t input_config = {
        .p_pull_config    = &pull_config,
        .p_trigger_config = &trigger_config,
        .p_handler_config = &handler_config
    };

    err = nrfx_gpiote_input_configure(&gpiote, BUTTON_SW0_PIN, &input_config);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to configure button pin (err 0x%08X)", err);
        return -1;
    }

    // Enable the input trigger
    nrfx_gpiote_trigger_enable(&gpiote, BUTTON_SW0_PIN, true);

    LOG_INF("Button SW0 initialized");
    return 0;
}


int main(void)
{

    int ret;
    uint8_t devid = 0;

    LOG_INF("Starting ADXL382 SPI test");

    adxl362_init();

    ret = adxl362_read_reg(ADXL362_REG_DEVID_AD, &devid);
    if (ret == NRFX_SUCCESS) {
        LOG_INF("ADXL362 Device ID: 0x%02X", devid);
        if (devid != 0xAD) {
            LOG_WRN("Unexpected device ID!");
        }
    } else {
        LOG_ERR("Failed to read from ADXL362 (err %d)", ret);
    }

    ret = adxl362_write_reg(ADXL362_REG_POWER_CTL, ADXL362_MEASURE_MODE);
    if (ret == NRFX_SUCCESS) {
        LOG_INF("Measurement mode enabled");
        k_msleep(10);  // Pequeno delay para o sensor estabilizar
    } else {
        LOG_ERR("Failed to set measurement mode");
    }

    uint8_t power_ctl;
    adxl362_read_reg(ADXL362_REG_POWER_CTL, &power_ctl);
    LOG_INF("POWER_CTL = 0x%02X", power_ctl);

    // Initialize button
    ret = button_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize button");
        return ret;
    }

    LOG_INF("System ready. Press SW0 button to read accelerometer data");

    while (1) {
        if (button_pressed) {
            button_pressed = false; // Reset flag
            read_accelerometer_data();
        }
        
        k_msleep(10); // Small delay to prevent busy waiting
    }

	return 0;
}

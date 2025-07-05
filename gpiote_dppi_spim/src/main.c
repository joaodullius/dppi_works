#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <nrfx_gpiote.h>
#include <nrfx_spim.h>
#include <helpers/nrfx_gppi.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_gpiote.h>

LOG_MODULE_REGISTER(adxl362, LOG_LEVEL_DBG);

/* Hardware Pin Definitions */

#define ADXL362_CS_PIN      22
#define ADXL362_INT1_PIN    19
#define SPIM_SCK            29
#define SPIM_MOSI           28
#define SPIM_MISO           26

/* SPI Instance */
#define SPIM_INST NRFX_SPIM_INSTANCE(3)
// Button SW0 pin for Thingy53
#define BUTTON_SW0_PIN  NRF_GPIO_PIN_MAP(1, 14)  // P1.14

static const nrfx_spim_t spim = SPIM_INST;

/* ADXL362 Register Definitions */
#define ADXL362_CMD_WRITE_REG           0x0A
#define ADXL362_CMD_READ_REG            0x0B
#define ADXL362_REG_DEVID_AD            0x00
#define ADXL362_REG_PARTID_AD           0x02
#define ADXL362_REG_INTMAP1             0x2A
#define ADXL362_INTMAP1_DATA_READY      (1 << 0)
#define ADXL362_REG_FILTER_CTL          0x2C
#define ADXL362_REG_POWER_CTL           0x2D
#define ADXL362_MEASURE_MODE            0x02
#define ADXL362_REG_XDATA_L             0x0E
#define ADXL362_REG_YDATA_L             0x10
#define ADXL362_REG_ZDATA_L             0x12

/* ADXL362 ODR (Output Data Rate) Configuration */
#define ADXL362_ODR_12_5_HZ             0x00  // 12.5 Hz (lowest ODR)
#define ADXL362_ODR_25_HZ               0x01  // 25 Hz
#define ADXL362_ODR_50_HZ               0x02  // 50 Hz
#define ADXL362_ODR_100_HZ              0x03  // 100 Hz
#define ADXL362_ODR_200_HZ              0x04  // 200 Hz
#define ADXL362_ODR_400_HZ              0x05  // 400 Hz

/* ADXL362 Range Configuration */
#define ADXL362_RANGE_2G                0x00  // ±2g range
#define ADXL362_RANGE_4G                0x40  // ±4g range
#define ADXL362_RANGE_8G                0x80  // ±8g range

/* ADXL362 Anti-aliasing Filter Configuration */
#define ADXL362_HALF_BW                 0x10  // Half bandwidth (anti-aliasing filter)

/* Combined Filter Control Register Value for Lowest ODR */
#define ADXL362_FILTER_CTL_LOWEST_ODR   (ADXL362_RANGE_2G | ADXL362_ODR_12_5_HZ)

/*
 * ODR Configuration Notes:
 * - The ADXL362 supports ODR from 12.5 Hz to 400 Hz
 * - 12.5 Hz is the lowest possible ODR, providing maximum power efficiency
 * - At 12.5 Hz ODR, data ready interrupts will occur every 80ms (1/12.5 = 0.08s)
 * - This configuration uses ±2g range for maximum sensitivity
 * - The anti-aliasing filter can be enabled by OR'ing with ADXL362_HALF_BW
 */

/* Constants */
#define GRAVITY_M_S2 9.80665f

// SPI transfer buffers for accelerometer reading
static uint8_t spi_tx_buf[8] = {
    ADXL362_CMD_READ_REG,
    ADXL362_REG_XDATA_L, // First register address
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00 // Dummy
};
static uint8_t spi_rx_buf[8] = {0};

//Create semaphore for SPIM transfer
static K_SEM_DEFINE(spim_sem, 0, 1);

//Create bool to indicate a transfer prepared for start event
static bool spim_dppi_transfer = false;


/* Function Prototypes */
static int adxl362_init(void);
static int adxl362_read_reg(uint8_t reg, uint8_t *value);
static int adxl362_write_reg(uint8_t reg, uint8_t value);
int prepare_spim_transfer(void);
static void adxl362_gpiote_handler(nrfx_gpiote_pin_t pin, nrfx_gpiote_trigger_t trigger, void *context);
static void spim_event_handler(nrfx_spim_evt_t const *p_event, void *p_context);

/* SPI Event Handler for Non-blocking Operation */
static void spim_event_handler(nrfx_spim_evt_t const *p_event, void *p_context)
{
    ARG_UNUSED(p_context);

    if (p_event->type == NRFX_SPIM_EVENT_DONE) {
        LOG_DBG("SPIM transfer done");
        if(!spim_dppi_transfer) {
            LOG_DBG("Regular SPIM transfer, doing semaphore release");
  			k_sem_give(&spim_sem);
            return; // If transfer was not prepared, skip semaphore release
        }
        else {
            LOG_DBG("SPIM transfer was prepared for DPPI, not releasing semaphore");
            // Process the received accelerometer data
            uint8_t axl = spi_rx_buf[2]; // X low byte
            uint8_t axh = spi_rx_buf[3]; // X high byte
            uint8_t ayl = spi_rx_buf[4]; // Y low byte
            uint8_t ayh = spi_rx_buf[5]; // Y high byte
            uint8_t azl = spi_rx_buf[6]; // Z low byte
            uint8_t azh = spi_rx_buf[7]; // Z high byte

            int16_t ax = ((int16_t)axh << 8) | axl;  // Combine bytes
            float ax_mg = ax * 1.0f; // Convert to mg
            float ax_ms2 = ax_mg * GRAVITY_M_S2 / 1000.0f; // Convert to m/s^2

            int16_t ay = (int16_t)((ayh << 8) | ayl);  
            float ay_mg = ay * 1.0f; // Convert to mg
            float ay_ms2 = ay_mg * GRAVITY_M_S2 / 1000.0f; // Convert to m/s^2

            int16_t az = (int16_t)((azh << 8) | azl);  
            float az_mg = az * 1.0f; // Convert to mg
            float az_ms2 = az_mg * GRAVITY_M_S2 / 1000.0f; // Convert to m/s^2

            LOG_INF("SPIM task-triggered read complete!");
            LOG_INF("Accel [m/s^2]: X=%.2f Y=%.2f Z=%.2f", (double)ax_ms2, (double)ay_ms2, (double)az_ms2);
            
            // Deassert CS after transfer
            nrf_gpio_pin_set(ADXL362_CS_PIN);
            
            // Prepare the next transfer for the next button press
            spim_dppi_transfer = false; // Reset flag for next transfer
            prepare_spim_transfer();
            
        }
       
    }
}

/* ADXL362 SPI Functions */
int adxl362_read_reg(uint8_t reg, uint8_t *value)
{
    uint8_t tx_buf[3] = {
        ADXL362_CMD_READ_REG,
        reg,
        0x00 // Dummy
    };
    uint8_t rx_buf[3] = {0};
	
    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length = sizeof(tx_buf),
        .p_rx_buffer = rx_buf,
        .rx_length = sizeof(rx_buf),
    };

    nrf_gpio_pin_clear(ADXL362_CS_PIN); // assert CS

    /* Start non-blocking SPI transfer */
    nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer, 0);
    if (err != NRFX_SUCCESS) {
        nrf_gpio_pin_set(ADXL362_CS_PIN);  // Deassert CS on error
        LOG_ERR("SPI transfer start failed (reg 0x%02X): %d", reg, err);
        return err;
    }

    /* Wait for transfer completion with timeout */
    if (k_sem_take(&spim_sem, K_MSEC(100)) != 0) {
        nrf_gpio_pin_set(ADXL362_CS_PIN);  // Deassert CS on timeout
        LOG_ERR("SPI transfer timeout (reg 0x%02X)", reg);
        return -ETIMEDOUT;
    }

    nrf_gpio_pin_set(ADXL362_CS_PIN); // deassert CS

    *value = rx_buf[2];  // Valor lido
    LOG_DBG("Read reg 0x%02X = 0x%02X", reg, *value);
    return err;
}

int adxl362_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t tx_buf[3] = {
        ADXL362_CMD_WRITE_REG,
        reg,
        value
    };

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = tx_buf,
        .tx_length = sizeof(tx_buf),
        .p_rx_buffer = NULL,
        .rx_length = 0,
    };

    /* Assert CS */
    nrf_gpio_pin_clear(ADXL362_CS_PIN);

    /* Start non-blocking SPI transfer */
    nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer, 0);
    if (err != NRFX_SUCCESS) {
        nrf_gpio_pin_set(ADXL362_CS_PIN);  // Deassert CS on error
        LOG_ERR("SPI transfer start failed (reg 0x%02X): %d", reg, err);
        return err;
    }

    /* Wait for transfer completion with timeout */
    if (k_sem_take(&spim_sem, K_MSEC(100)) != 0) {
        nrf_gpio_pin_set(ADXL362_CS_PIN);  // Deassert CS on timeout
        LOG_ERR("SPI transfer timeout (reg 0x%02X)", reg);
        return -ETIMEDOUT;
    }

    /* Deassert CS */
    nrf_gpio_pin_set(ADXL362_CS_PIN);
    return err; 
}

int adxl362_init(void)
{
    nrfx_err_t err;

    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_SPIM_INST_GET(3)), IRQ_PRIO_LOWEST,
        NRFX_SPIM_INST_HANDLER_GET(3), 0, 0);


    /* Configure CS pin manually */
    nrf_gpio_cfg_output(ADXL362_CS_PIN);
    nrf_gpio_pin_set(ADXL362_CS_PIN); // inactive

    /* Initialize SPI with event handler for non-blocking operation */
    nrfx_spim_config_t config = NRFX_SPIM_DEFAULT_CONFIG(SPIM_SCK,
                                                          SPIM_MOSI,
                                                          SPIM_MISO,
                                                          NRF_SPIM_PIN_NOT_CONNECTED);
    
	err = nrfx_spim_init(&spim, &config, spim_event_handler, NULL);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to init SPIM, error: 0x%08X", err);
        return -1;
    }
    LOG_INF("ADXL362 SPI initialized successfully (non-blocking mode)");

    return 0;
}

static nrfx_gpiote_pin_t gpiote_input_pin = BUTTON_SW0_PIN;

// Function to initialize gpiote input
int gpiote_input_init(void)
{
    nrfx_err_t err;
    uint8_t gpiote_input_channel;
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
    err = nrfx_gpiote_channel_alloc(&gpiote, &gpiote_input_channel);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to allocate channel (err 0x%08X)", err);
        return -1;
    }

    /* Configure GPIOTE Input / Interrupt */
    static const nrf_gpio_pin_pull_t pull_cfg = NRF_GPIO_PIN_PULLUP;
    nrfx_gpiote_trigger_config_t trigger_cfg = {
        .trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
        .p_in_channel = &gpiote_input_channel,
    };
    static const nrfx_gpiote_handler_config_t handler_cfg = {
        .handler = adxl362_gpiote_handler,
    };
    nrfx_gpiote_input_pin_config_t input_cfg = {
        .p_pull_config    = &pull_cfg,
        .p_trigger_config = &trigger_cfg,
        .p_handler_config = &handler_cfg
    };

    err = nrfx_gpiote_input_configure(&gpiote, gpiote_input_pin, &input_cfg);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to configure button pin (err 0x%08X)", err);
        return -1;
    }

    nrfx_gpiote_trigger_enable(&gpiote, gpiote_input_pin, true);
    LOG_INF("GPIOTE Input configured successfully on P0.%d", gpiote_input_pin);

    return 0;
}

/* Interrupt Handlers and Workqueue */
static void adxl362_gpiote_handler(nrfx_gpiote_pin_t pin,
                                   nrfx_gpiote_trigger_t trigger,
                                   void *context)
{
    ARG_UNUSED(trigger);
    ARG_UNUSED(context);

    LOG_INF("*** GPIOTE interrupt triggered on pin (P0.%d)! ***", pin);
}

// Function to read accelerometer data
int prepare_spim_transfer(void)
{

	
    // Assert CS before transfer
    nrf_gpio_pin_clear(ADXL362_CS_PIN);

    nrfx_spim_xfer_desc_t xfer = {
        .p_tx_buffer = spi_tx_buf,
        .tx_length = sizeof(spi_tx_buf),
        .p_rx_buffer = spi_rx_buf,
        .rx_length = sizeof(spi_rx_buf),
    };

    // Configure the transfer
    nrfx_err_t err = nrfx_spim_xfer(&spim, &xfer, NRFX_SPIM_FLAG_HOLD_XFER);
    if (err != NRFX_SUCCESS) {
        LOG_ERR("Failed to prepare SPIM transfer (err 0x%08X)", err);
        nrf_gpio_pin_set(ADXL362_CS_PIN); // Release CS on error
        return -1;
    }
    
    spim_dppi_transfer = true; // Indicate transfer is prepared

    LOG_DBG("SPIM transfer prepared and ready for task trigger");
    return 0;
}

int configure_dppi(void)
{
	nrfx_err_t err;
	uint8_t ppi_channel;
    static const nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(0);

	/* Allocate a DPPI channel */
	err = nrfx_gppi_channel_alloc(&ppi_channel);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("nrfx_gppi_channel_alloc error: 0x%08X", err);
		return err;
	}

	/* Setup endpoints so that the input pin event triggers the SPIM start task */
	nrfx_gppi_channel_endpoints_setup(ppi_channel,
		nrfx_gpiote_in_event_address_get(&gpiote, BUTTON_SW0_PIN),
		nrfx_spim_start_task_address_get(&spim));

	/* Enable the DPPI channel */
	nrfx_gppi_channels_enable(BIT(ppi_channel));
    LOG_INF("DPPI configured: Button SW0 -> SPIM start");
	return NRFX_SUCCESS;
}


/* Main Function */
int main(void)
{

    int ret;
    uint8_t devid = 0;

    LOG_INF("Starting GPIOTE-DPPI-SPIM test");
    if (adxl362_init() != 0) {
        LOG_ERR("Failed to initialize ADXL362");
        return 1;
    }

    // Initialize GPIOTE Input
    ret = gpiote_input_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize button");
        return ret;
    }

    /* Read and verify Device ID */
    ret = adxl362_read_reg(ADXL362_REG_DEVID_AD, &devid);
    if (ret == NRFX_SUCCESS) {
        LOG_INF("ADXL362 Device ID: 0x%02X", devid);
        if (devid != 0xAD) {
            LOG_WRN("Unexpected device ID!");
        }
    } else {
        LOG_ERR("Failed to read from ADXL362 (err %d)", ret);
    }

    /* Read and verify Part ID */
    ret = adxl362_read_reg(ADXL362_REG_PARTID_AD, &devid);
    if (ret == NRFX_SUCCESS) {
        LOG_INF("ADXL362 Part ID: 0x%02X", devid);
        if (devid != 0xF2) {
            LOG_WRN("Unexpected part ID!");
        }
    } else {
        LOG_ERR("Failed to read from ADXL362 (err %d)", ret);
    }

    /* Enable measurement mode */
    ret = adxl362_write_reg(ADXL362_REG_POWER_CTL, ADXL362_MEASURE_MODE);
    if (ret == NRFX_SUCCESS) {
        LOG_INF("Measurement mode enabled");
        k_msleep(10);  // Small delay for sensor stabilization
    } else {
        LOG_ERR("Failed to set measurement mode");
        return ret;
    }
    k_msleep(50);  // Additional delay to ensure sensor is ready

    /* Verify power control register */
    uint8_t power_ctl;
    adxl362_read_reg(ADXL362_REG_POWER_CTL, &power_ctl);
    LOG_DBG("POWER_CTL = 0x%02X", power_ctl);

    prepare_spim_transfer();

    ret = configure_dppi();
    if (ret != NRFX_SUCCESS) {
        LOG_ERR("Failed to configure DPPI");
        return ret;
    }

    LOG_INF("System ready. Press SW0 button to read accelerometer data");

	return 0;
}

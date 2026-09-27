/*
 * BMI270 backend (nRF54L15 TAG, SPI). SPI protocol: read = (reg | 0x80), then
 * one dummy byte, then data; write = reg, data. The sensor needs its ~8 KB
 * configuration file uploaded after every power-on before it measures.
 * Register map and config blob from the Zephyr bmi270 driver (Apache-2.0).
 */
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensor.h"
#include "bmi270_config_file.h"

LOG_MODULE_REGISTER(bmi270, LOG_LEVEL_INF);

#define REG_CHIP_ID         0x00
#define REG_STATUS          0x03   /* bit7 drdy_acc, cleared when data regs are read */
#define REG_ACC_X_LSB       0x0C
#define REG_INTERNAL_STATUS 0x21
#define REG_ACC_CONF        0x40
#define REG_ACC_RANGE       0x41
#define REG_INT1_IO_CTRL    0x53
#define REG_INT_LATCH       0x55
#define REG_INT_MAP_DATA    0x58
#define REG_INIT_CTRL       0x59
#define REG_INIT_ADDR_0     0x5B
#define REG_INIT_DATA       0x5E
#define REG_PWR_CONF        0x7C
#define REG_PWR_CTRL        0x7D
#define REG_CMD             0x7E

#define CHIP_ID             0x24
#define CMD_SOFT_RESET      0xB6
#define STATUS_DRDY_ACC     0x80
#define INTERNAL_STATUS_MSK 0x0F
#define INTERNAL_STATUS_OK  0x01
#define PWR_CONF_ADV_PWR_SAVE 0x01
#define PWR_CTRL_ACC_EN     0x04
#define ACC_CONF_FILT_PERF  0x80
#define ACC_CONF_BWP_NORM_AVG4 (0x02 << 4)
#define ACC_RANGE_2G        0x00
#define INT_IO_CTRL_LVL_HIGH 0x02
#define INT_IO_CTRL_OUTPUT_EN 0x08
#define INT_MAP_DATA_DRDY_INT1 0x04
#define INT_LATCH_NONE      0x00

#define CONFIG_CHUNK        32
#define WRITE_DELAY_US      1000   /* after each write while adv_pwr_save may be on */
#define SOFT_RESET_US       2000
#define POWER_ON_US         500

/*
 * Burst: STATUS (0x03) through ACC_Z_MSB (0x11) = 15 registers, plus the
 * address byte and the dummy byte the BMI270 clocks out before data.
 *   rx[0]     clocked while sending the address (ignored)
 *   rx[1]     dummy
 *   rx[2]     STATUS      (0x03)
 *   rx[11..16] ACC_X/Y/Z LSB,MSB (0x0C..0x11)
 */
static const uint8_t burst_tx[1] = { REG_STATUS | 0x80 };
BUILD_ASSERT(SENSOR_BURST_LEN == 2 + (0x11 - 0x03 + 1));

static int reg_read(uint8_t reg, uint8_t *val)
{
	uint8_t tx[1] = { reg | 0x80 };
	uint8_t rx[3];
	int err = spim_xfer_blocking(tx, sizeof(tx), rx, sizeof(rx));

	*val = rx[2];
	return err;
}

static int reg_write(uint8_t reg, uint8_t val)
{
	uint8_t tx[2] = { reg, val };
	int err = spim_xfer_blocking(tx, sizeof(tx), NULL, 0);

	k_busy_wait(WRITE_DELAY_US);
	return err;
}

static int reg_write_burst(uint8_t reg, const uint8_t *data, size_t len)
{
	uint8_t tx[1 + CONFIG_CHUNK];

	tx[0] = reg;
	memcpy(&tx[1], data, len);
	int err = spim_xfer_blocking(tx, 1 + len, NULL, 0);

	k_busy_wait(WRITE_DELAY_US);
	return err;
}

static int upload_config(void)
{
	const uint8_t *cfg = bmi270_config_file_max_fifo;
	const size_t cfg_len = sizeof(bmi270_config_file_max_fifo);
	int64_t t0 = k_uptime_get();
	size_t chunks = 0;
	int err;

	for (size_t index = 0; index < cfg_len; index += CONFIG_CHUNK) {
		uint8_t addr[2] = { (uint8_t)((index / 2) & 0x0F), (uint8_t)((index / 2) >> 4) };

		err = reg_write_burst(REG_INIT_ADDR_0, addr, 2);
		if (err) {
			return err;
		}
		err = reg_write_burst(REG_INIT_DATA, &cfg[index], CONFIG_CHUNK);
		if (err) {
			return err;
		}
		chunks++;
	}
	uint8_t a0, a1;

	reg_read(REG_INIT_ADDR_0, &a0);
	reg_read(REG_INIT_ADDR_0 + 1, &a1);
	LOG_INF("config upload: %u bytes in %u chunks, %lld ms; INIT_ADDR readback 0x%02X%02X",
		(unsigned)cfg_len, (unsigned)chunks, k_uptime_get() - t0, a1, a0);
	return 0;
}

/* ODR code and effective rate (x10) for ACC_CONF[3:0] */
struct odr_entry { uint16_t hz_x10; uint8_t code; };
static const struct odr_entry odr_table[] = {
	{ 16000, 0x0C }, { 8000, 0x0B }, { 4000, 0x0A }, { 2000, 0x09 },
	{ 1000, 0x08 }, { 500, 0x07 }, { 250, 0x06 },
};

static uint16_t odr_hz_x10;

static int bmi270_init(void)
{
	uint8_t id, v;
	int err;

	k_busy_wait(POWER_ON_US);
	/* First read switches the interface to SPI mode; its value is not trusted */
	reg_read(REG_CHIP_ID, &id);
	k_msleep(2);
	err = reg_read(REG_CHIP_ID, &id);
	if (err) {
		return err;
	}
	if (id != CHIP_ID) {
		LOG_ERR("CHIP_ID = 0x%02X, expected 0x%02X", id, CHIP_ID);
		return -ENODEV;
	}
	LOG_INF("BMI270 CHIP_ID 0x%02X", id);

	err = reg_write(REG_CMD, CMD_SOFT_RESET);
	if (err) {
		return err;
	}
	k_busy_wait(SOFT_RESET_US);
	reg_read(REG_CHIP_ID, &id);  /* back to SPI mode after reset */
	k_msleep(2);

	err = reg_write(REG_PWR_CONF, 0x00);   /* adv_pwr_save off for config load */
	if (err) {
		return err;
	}
	err = reg_write(REG_INIT_CTRL, 0x00);
	if (err) {
		return err;
	}
	err = upload_config();
	if (err) {
		return err;
	}
	err = reg_write(REG_INIT_CTRL, 0x01);
	if (err) {
		return err;
	}
	for (int tries = 0; ; tries++) {
		reg_read(REG_INTERNAL_STATUS, &v);
		if ((v & INTERNAL_STATUS_MSK) == INTERNAL_STATUS_OK) {
			break;
		}
		if (tries >= 30) {
			LOG_ERR("config load failed, INTERNAL_STATUS 0x%02X after %d ms", v, tries * 10);
			return -EIO;
		}
		k_msleep(10);
	}
	LOG_INF("config file loaded (%u bytes)", (unsigned)sizeof(bmi270_config_file_max_fifo));

	const struct odr_entry *sel = &odr_table[ARRAY_SIZE(odr_table) - 1];

	for (size_t i = 0; i < ARRAY_SIZE(odr_table); i++) {
		if (odr_table[i].hz_x10 <= CONFIG_APP_SENSOR_ODR_HZ * 10) {
			sel = &odr_table[i];
			break;
		}
	}
	odr_hz_x10 = sel->hz_x10;

	err = reg_write(REG_ACC_CONF, ACC_CONF_FILT_PERF | ACC_CONF_BWP_NORM_AVG4 | sel->code);
	if (err) {
		return err;
	}
	err = reg_write(REG_ACC_RANGE, ACC_RANGE_2G);
	if (err) {
		return err;
	}
	/* adv_pwr_save stays off: continuous performance mode */
	err = reg_write(REG_PWR_CTRL, PWR_CTRL_ACC_EN);
	if (err) {
		return err;
	}
	k_msleep(50);

	reg_read(REG_ACC_CONF, &v);
	LOG_INF("ACC_CONF 0x%02X (+/-2 g, ODR %u Hz)", v, odr_hz_x10 / 10);
	reg_read(REG_PWR_CTRL, &v);
	LOG_INF("PWR_CTRL 0x%02X (%s)", v, (v & PWR_CTRL_ACC_EN) ? "accel on" : "accel OFF");
	return 0;
}

static int bmi270_enable_drdy_int(void)
{
	uint8_t v;
	int err;

	err = reg_write(REG_INT1_IO_CTRL, INT_IO_CTRL_LVL_HIGH | INT_IO_CTRL_OUTPUT_EN);
	if (err) {
		return err;
	}
	err = reg_write(REG_INT_LATCH, INT_LATCH_NONE);
	if (err) {
		return err;
	}
	err = reg_write(REG_INT_MAP_DATA, INT_MAP_DATA_DRDY_INT1);
	if (err) {
		return err;
	}
	reg_read(REG_STATUS, &v);
	LOG_INF("INT1: data-ready, push-pull active high, non-latched; STATUS 0x%02X", v);
	return 0;
}

static void bmi270_decode(const uint8_t *rx, struct sensor_sample *out)
{
	out->fresh = (rx[2] & STATUS_DRDY_ACC) != 0;
	out->x = (int16_t)((rx[12] << 8) | rx[11]);
	out->y = (int16_t)((rx[14] << 8) | rx[13]);
	out->z = (int16_t)((rx[16] << 8) | rx[15]);
}

static struct sensor_ops ops = {
	.name = "BMI270",
	.init = bmi270_init,
	.enable_drdy_int = bmi270_enable_drdy_int,
	.burst_tx = burst_tx,
	.burst_tx_len = sizeof(burst_tx),
	.decode = bmi270_decode,
	.fresh_offset = 2,
	.fresh_mask = STATUS_DRDY_ACC,
	.mg_per_lsb = 2000.0f / 32768.0f,   /* +/-2 g over 16 bits */
};

const struct sensor_ops *const sensor = &ops;

uint16_t sensor_odr_hz_x10(void)
{
	return odr_hz_x10;
}

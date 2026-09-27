/* ADXL362 backend (Thingy:53). SPI: command byte, address, data; no dummy. */
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "sensor.h"

LOG_MODULE_REGISTER(adxl362, LOG_LEVEL_INF);

#define CMD_WRITE_REG   0x0A
#define CMD_READ_REG    0x0B

#define REG_DEVID_AD    0x00
#define REG_PARTID      0x02
#define REG_STATUS      0x0B   /* bit0 DATA_READY, cleared when data regs are read */
#define REG_XDATA_L     0x0E
#define REG_INTMAP1     0x2A
#define REG_FILTER_CTL  0x2C
#define REG_POWER_CTL   0x2D

#define DEVID_AD        0xAD
#define PARTID          0xF2
#define INTMAP1_DATA_READY 0x01
#define POWER_CTL_MEASURE  0x02
#define RANGE_2G        0x00
#define STATUS_DATA_READY 0x01

/*
 * Burst: read STATUS..ZDATA_H in one transaction. STATUS comes first so the
 * DATA_READY bit tells whether this burst carries a new sample (it is cleared
 * by the data-register read that follows in the same transaction).
 *   rx[0..1]  clocked while sending cmd/addr (ignored)
 *   rx[2]     STATUS
 *   rx[3..4]  FIFO_ENTRIES_L/H (ignored)
 *   rx[5..10] XDATA_L/H, YDATA_L/H, ZDATA_L/H
 */
static const uint8_t burst_tx[2] = { CMD_READ_REG, REG_STATUS };
BUILD_ASSERT(SENSOR_BURST_LEN == 11);

static int reg_read(uint8_t reg, uint8_t *val)
{
	uint8_t tx[3] = { CMD_READ_REG, reg, 0x00 };
	uint8_t rx[3];
	int err = spim_xfer_blocking(tx, sizeof(tx), rx, sizeof(rx));

	*val = rx[2];
	return err;
}

static int reg_write(uint8_t reg, uint8_t val)
{
	uint8_t tx[3] = { CMD_WRITE_REG, reg, val };

	return spim_xfer_blocking(tx, sizeof(tx), NULL, 0);
}

/* ODR code and effective rate (x10) for FILTER_CTL[2:0] */
struct odr_entry { uint16_t hz_x10; uint8_t code; };
static const struct odr_entry odr_table[] = {
	{ 4000, 5 }, { 2000, 4 }, { 1000, 3 }, { 500, 2 }, { 250, 1 }, { 125, 0 },
};

static uint16_t odr_hz_x10;

static int adxl362_init(void)
{
	uint8_t id, v;
	int err;

	err = reg_read(REG_DEVID_AD, &id);
	if (err) {
		return err;
	}
	if (id != DEVID_AD) {
		LOG_ERR("DEVID_AD = 0x%02X, expected 0x%02X", id, DEVID_AD);
		return -ENODEV;
	}
	reg_read(REG_PARTID, &v);
	LOG_INF("ADXL362 DEVID_AD 0x%02X PARTID 0x%02X", id, v);

	/* ODR: nearest supported rate not above the requested one */
	const struct odr_entry *sel = &odr_table[ARRAY_SIZE(odr_table) - 1];

	for (size_t i = 0; i < ARRAY_SIZE(odr_table); i++) {
		if (odr_table[i].hz_x10 <= CONFIG_APP_SENSOR_ODR_HZ * 10) {
			sel = &odr_table[i];
			break;
		}
	}
	odr_hz_x10 = sel->hz_x10;

	/* The sensor keeps its registers across MCU resets: always write them */
	err = reg_write(REG_FILTER_CTL, RANGE_2G | sel->code);
	if (err) {
		return err;
	}
	err = reg_write(REG_POWER_CTL, POWER_CTL_MEASURE);
	if (err) {
		return err;
	}
	k_msleep(50);

	reg_read(REG_FILTER_CTL, &v);
	LOG_INF("FILTER_CTL 0x%02X (+/-2 g, ODR %u.%u Hz)", v, odr_hz_x10 / 10, odr_hz_x10 % 10);
	reg_read(REG_POWER_CTL, &v);
	LOG_INF("POWER_CTL 0x%02X (%s)", v, (v & 0x03) == POWER_CTL_MEASURE ? "measurement" : "NOT measuring");
	return 0;
}

static int adxl362_enable_drdy_int(void)
{
	uint8_t v;
	int err = reg_write(REG_INTMAP1, INTMAP1_DATA_READY);

	if (err) {
		return err;
	}
	reg_read(REG_INTMAP1, &v);
	reg_read(REG_STATUS, &v);
	/* DATA_READY is a level: high until the data registers are read */
	LOG_INF("INTMAP1 = DATA_READY on INT1 (active high); STATUS 0x%02X", v);
	return 0;
}

static void adxl362_decode(const uint8_t *rx, struct sensor_sample *out)
{
	out->fresh = (rx[2] & STATUS_DATA_READY) != 0;
	out->x = (int16_t)((rx[6] << 8) | rx[5]);
	out->y = (int16_t)((rx[8] << 8) | rx[7]);
	out->z = (int16_t)((rx[10] << 8) | rx[9]);
}

static struct sensor_ops ops = {
	.name = "ADXL362",
	.init = adxl362_init,
	.enable_drdy_int = adxl362_enable_drdy_int,
	.burst_tx = burst_tx,
	.burst_tx_len = sizeof(burst_tx),
	.decode = adxl362_decode,
	.fresh_offset = 2,
	.fresh_mask = STATUS_DATA_READY,
	.mg_per_lsb = 1.0f,    /* +/-2 g range */
};

const struct sensor_ops *const sensor = &ops;

uint16_t sensor_odr_hz_x10(void)
{
	return odr_hz_x10;
}

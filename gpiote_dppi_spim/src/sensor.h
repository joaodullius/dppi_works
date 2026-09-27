/*
 * Minimal accelerometer backend interface. A backend only knows how to
 * configure its sensor over SPI (blocking transfers, at init) and how to
 * describe/decode the burst read that the hardware repeats afterwards.
 */
#ifndef SENSOR_H_
#define SENSOR_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Length of one burst transaction (bytes clocked = bytes received) */
#if defined(CONFIG_APP_SENSOR_ADXL362)
#define SENSOR_BURST_LEN 11   /* cmd, addr, STATUS, FIFO_L, FIFO_H, X/Y/Z L+H */
#elif defined(CONFIG_APP_SENSOR_BMI270)
#define SENSOR_BURST_LEN 17   /* addr, dummy, STATUS(0x03) .. ACC_Z_MSB(0x11) */
#else
#error "select a sensor backend"
#endif

struct sensor_sample {
	int16_t x, y, z;   /* raw counts */
	bool fresh;        /* data-ready bit was set in this burst */
};

struct sensor_ops {
	const char *name;
	/* Full sensor bring-up over blocking SPI transfers; returns errno */
	int (*init)(void);
	/* Route data-ready to the interrupt pin used by the trigger */
	int (*enable_drdy_int)(void);
	/* Burst read descriptor: TX prefix and total transaction length */
	const uint8_t *burst_tx;
	size_t burst_tx_len;
	/* Decode one received burst into a sample */
	void (*decode)(const uint8_t *rx, struct sensor_sample *out);
	/* Data-ready bit inside the raw burst (for the ISR-side filter) */
	uint8_t fresh_offset;
	uint8_t fresh_mask;
	/* Scale of the raw counts, in mg per LSB (for the report) */
	float mg_per_lsb;
};

extern const struct sensor_ops *const sensor;

/* Effective ODR after rounding APP_SENSOR_ODR_HZ, in Hz x10 (keeps 12.5) */
uint16_t sensor_odr_hz_x10(void);

/* Blocking SPI helpers provided by spim_dppi.c for the backends' init code */
int spim_xfer_blocking(const uint8_t *tx, size_t tx_len, uint8_t *rx, size_t rx_len);

#endif /* SENSOR_H_ */

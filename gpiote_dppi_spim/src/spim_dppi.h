/*
 * Hardware engine: SPIM with hardware CSN, repeated EasyDMA burst started
 * through DPPI by the GPIOTE IN event of the sensor's data-ready pin. The
 * bursts land in a ring (RX pointer post-increment); a periodic drain thread
 * moves them into a k_msgq and, once per lap, re-arms the ring wrap on the
 * SPIM READY IRQ.
 * With APP_PER_SAMPLE_IRQ there is one buffer instead and the SPIM END ISR
 * copies each burst into the k_msgq.
 */
#ifndef SPIM_DPPI_H_
#define SPIM_DPPI_H_

#include <stdint.h>
#include <zephyr/kernel.h>

#include "sensor.h"

/* Bring up SPIM (pinctrl from the devicetree bus node) and the sensor */
int spim_dppi_init(void);

/* Arm the repeated burst, connect DPPI, start the trigger. Returns errno. */
int spim_dppi_start(void);

/* Total SPIM transactions started since start (finished laps + DMA pointer) */
uint32_t spim_dppi_total_xfers(void);


/* One SENSOR_BURST_LEN-byte raw burst per message, in order */
extern struct k_msgq sample_q;
/* Samples that could not be queued because the queue was full */
uint32_t spim_dppi_dropped(void);
/* Wraps written after the next START had begun (that transaction's sample is skipped) */
uint32_t spim_dppi_late_wraps(void);
/* Laps in which the DMA reached the guard slots (ring too small for the drain period) */
uint32_t spim_dppi_overflows(void);
/* Per-sample mode: samples the next transaction overwrote during the copy (dropped) */
uint32_t spim_dppi_torn(void);

#endif /* SPIM_DPPI_H_ */

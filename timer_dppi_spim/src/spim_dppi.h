/*
 * Hardware engine: SPIM with hardware CSN, repeated EasyDMA burst started
 * through DPPI by a TIMER COMPARE event, transaction starts counted by a second TIMER
 * in counter mode; every sample goes to a k_msgq in blocks of N.
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

/* Total SPIM transactions completed since start (hardware counter) */
uint32_t spim_dppi_total_xfers(void);


/* One SENSOR_BURST_LEN-byte raw burst per message, in order */
extern struct k_msgq sample_q;
/* Samples the ISR could not queue because the queue was full */
uint32_t spim_dppi_dropped(void);
/* Samples the ISR skipped as repeats (APP_QUEUE_FRESH_ONLY) */
uint32_t spim_dppi_skipped(void);
/* Block wraps that ran later than one trigger period (samples lost) */
uint32_t spim_dppi_late_wraps(void);

/* Change the trigger TIMER period at runtime (restarts the timer) */
void spim_dppi_set_period_us(uint32_t period_us);

#if defined(CONFIG_APP_WRAP_LATENCY_STATS)
/* Trigger COMPARE -> wrap ISR latency since the last reset; false if no sample */
bool spim_dppi_wrap_latency(uint32_t *min_ns, uint32_t *avg_ns, uint32_t *max_ns, bool reset);
#endif

#endif /* SPIM_DPPI_H_ */

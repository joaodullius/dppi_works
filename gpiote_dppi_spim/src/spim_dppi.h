/*
 * Hardware engine: SPIM with hardware CSN, repeated EasyDMA burst started
 * through DPPI by the GPIOTE IN event of the sensor's data-ready pin, transaction starts
 * counted by a TIMER in counter mode; every sample goes to a k_msgq in
 * blocks of N.
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
/* Block wraps that ran later than one sample period (samples lost) */
uint32_t spim_dppi_late_wraps(void);

#endif /* SPIM_DPPI_H_ */

/*
 * Everything board specific comes from the devicetree, through chosen nodes
 * set by the board overlay:
 *   app,accel        the accelerometer node on its SPI bus (int1-gpios /
 *                    irq-gpios = data-ready pin, cs-gpios on the bus = CSN)
 *   app,timer-count  TIMER used as hardware counter of SPIM END events
 *   app,egu          EGU that turns block-complete events into the queue ISR
 */
#ifndef APP_DT_H_
#define APP_DT_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/gpio/gpio.h>

#define ACCEL_NODE      DT_CHOSEN(app_accel)
#define BUS_NODE        DT_BUS(ACCEL_NODE)
#define CNT_TIMER_NODE  DT_CHOSEN(app_timer_count)
#define EGU_NODE        DT_CHOSEN(app_egu)

BUILD_ASSERT(DT_NODE_EXISTS(EGU_NODE), "chosen app,egu is missing in the board overlay");

BUILD_ASSERT(DT_NODE_EXISTS(ACCEL_NODE), "chosen app,accel is missing in the board overlay");
BUILD_ASSERT(DT_NODE_HAS_PROP(BUS_NODE, cs_gpios), "the SPI bus needs cs-gpios (hardware CSN pin)");

/* Absolute nRF pin number (port * 32 + pin) of a gpio phandle-array entry */
#define NRF_PIN_ABS(node, prop) \
	(DT_PROP(DT_GPIO_CTLR(node, prop), port) * 32 + DT_GPIO_PIN(node, prop))

#define CS_PIN_ABS NRF_PIN_ABS(BUS_NODE, cs_gpios)

/* Data-ready pin: adi,adxl362 calls it int1-gpios, bosch,bmi270 irq-gpios */
#if DT_NODE_HAS_PROP(ACCEL_NODE, int1_gpios)
#define INT_GPIO_PROP int1_gpios
#elif DT_NODE_HAS_PROP(ACCEL_NODE, irq_gpios)
#define INT_GPIO_PROP irq_gpios
#else
#error "the chosen app,accel node needs int1-gpios or irq-gpios (data-ready pin)"
#endif

#define INT_PIN_ABS     NRF_PIN_ABS(ACCEL_NODE, INT_GPIO_PROP)
#define INT_ACTIVE_LOW  ((DT_GPIO_FLAGS(ACCEL_NODE, INT_GPIO_PROP) & GPIO_ACTIVE_LOW) != 0)
/* GPIOTE instance serving the port of the data-ready pin (gpiote-instance) */
#define INT_GPIOTE_NODE DT_PHANDLE(DT_GPIO_CTLR(ACCEL_NODE, INT_GPIO_PROP), gpiote_instance)

#endif /* APP_DT_H_ */

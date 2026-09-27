/*
 * Everything board specific comes from the devicetree, through chosen nodes
 * set by the board overlay:
 *   app,accel          the accelerometer node on its SPI bus (cs-gpios on the
 *                      bus = hardware CSN pin)
 *   app,timer-trigger  TIMER whose COMPARE0 starts every SPIM transaction
 *   app,timer-count    TIMER used as hardware counter of SPIM END events
 *   app,egu            EGU that turns block-complete events into the queue ISR
 */
#ifndef APP_DT_H_
#define APP_DT_H_

#include <zephyr/devicetree.h>
#include <zephyr/dt-bindings/gpio/gpio.h>

#define ACCEL_NODE      DT_CHOSEN(app_accel)
#define BUS_NODE        DT_BUS(ACCEL_NODE)
#define TRIG_TIMER_NODE DT_CHOSEN(app_timer_trigger)
#define CNT_TIMER_NODE  DT_CHOSEN(app_timer_count)
#define EGU_NODE        DT_CHOSEN(app_egu)

BUILD_ASSERT(DT_NODE_EXISTS(ACCEL_NODE), "chosen app,accel is missing in the board overlay");
BUILD_ASSERT(DT_NODE_EXISTS(TRIG_TIMER_NODE), "chosen app,timer-trigger is missing in the board overlay");
BUILD_ASSERT(DT_NODE_HAS_PROP(BUS_NODE, cs_gpios), "the SPI bus needs cs-gpios (hardware CSN pin)");
#if defined(CONFIG_APP_CONSUME_QUEUE)
BUILD_ASSERT(DT_NODE_EXISTS(EGU_NODE), "chosen app,egu is missing in the board overlay");
#endif

/* Absolute nRF pin number (port * 32 + pin) of a gpio phandle-array entry */
#define NRF_PIN_ABS(node, prop) \
	(DT_PROP(DT_GPIO_CTLR(node, prop), port) * 32 + DT_GPIO_PIN(node, prop))

#define CS_PIN_ABS NRF_PIN_ABS(BUS_NODE, cs_gpios)

#endif /* APP_DT_H_ */

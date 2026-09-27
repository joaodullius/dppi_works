/*
 * hfxo_launcher: application-core side of the FLPR builds.
 *
 * The FLPR is started by the nordic-flpr snippet's VPR launcher at boot. The
 * FLPR has no clock-control driver in NCS 3.4.1, so the HFXO is requested
 * here and held forever; the FLPR's TIMER then runs from the crystal.
 */
#include <zephyr/kernel.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/sys/onoff.h>

int main(void)
{
	static struct onoff_client cli;
	struct onoff_manager *mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);

	sys_notify_init_spinwait(&cli.notify);
	(void)onoff_request(mgr, &cli);

	while (1) {
		k_sleep(K_FOREVER);
	}
	return 0;
}

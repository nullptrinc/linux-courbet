/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef COURBET_FAST_MODEL_H
#define COURBET_FAST_MODEL_H

/* Third-party PPS is limited to 3 A cell current, as in the OEM policy. */
#define COURBET_FAST_FCC_UA 3000000
#define COURBET_BUCK_FCC_UA 3000000
#define COURBET_PPS_MAX_UV 10000000
#define COURBET_PPS_STEP_UV 20000

static inline int courbet_fast_thermal_block(int temp, int previous)
{
	if (temp < 150 || (previous < 0 && temp < 170))
		return -1;
	if (temp >= 400 || (previous > 0 && temp > 380))
		return 1;
	return 0;
}

static inline int courbet_pump_allowed(int temp, int voltage, int target,
				       int fcc, int icl, int percent, int end,
				       int enable, int die)
{
	return enable && temp >= 150 && temp < 400 && die < 75000 &&
		voltage >= 3500000 && voltage < target - 100000 &&
		fcc >= 2000000 && icl >= 1500000 && percent < end && percent < 90;
}

/* CC/CV control: a single 20 mV step, with independent current/voltage vetoes. */
static inline int courbet_pps_step(int requested_uv, int battery_uv, int target_uv,
				   int cell_ua, int target_ua, int bus_ua,
				   int bus_limit_ua, int error_low, int error_high)
{
	int step = 0;

	if (battery_uv >= target_uv - 20000 || cell_ua > target_ua ||
	    bus_ua > bus_limit_ua || error_high)
		step = -COURBET_PPS_STEP_UV;
	else if (error_low || (cell_ua < target_ua - 100000 &&
			      bus_ua < bus_limit_ua - 100000))
		step = COURBET_PPS_STEP_UV;
	return requested_uv + step;
}

static inline int courbet_pump_window(int bus_uv, int cell_uv)
{
	return cell_uv >= 3500000 && cell_uv <= 4450000 &&
		bus_uv >= 2 * cell_uv + 100000 &&
		bus_uv <= 2 * cell_uv + 700000 && bus_uv <= COURBET_PPS_MAX_UV;
}

/* Once switching, permit the loaded output to approach half the input. */
static inline int courbet_pump_running_window(int bus_uv, int cell_uv)
{
	return cell_uv >= 3500000 && cell_uv <= 4450000 &&
		bus_uv >= 2 * cell_uv - 100000 &&
		bus_uv <= 2 * cell_uv + 700000 && bus_uv <= COURBET_PPS_MAX_UV;
}
#endif

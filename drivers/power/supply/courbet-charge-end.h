/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef COURBET_CHARGE_END_H
#define COURBET_CHARGE_END_H
#include "courbet-power-model.h"

enum courbet_end_reason {
	COURBET_END_NONE,
	COURBET_END_FULL,
	COURBET_END_PREMATURE,
	COURBET_END_VOLTAGE,
	COURBET_END_TARGET,
};

struct courbet_charge_end {
	s64 taper_start_ms, taper_last_ms, recharge_start_ms, recharge_last_ms;
	int hold_percent, reason, full, last_state;
};

static inline int courbet_recharge_uv(int target_uv)
{
	int voltage = target_uv - 250000;

	if (voltage > 4200000)
		voltage = 4200000;
	if (voltage < 3350000)
		voltage = 3350000;
	return voltage;
}

static inline void courbet_end_reset_taper(struct courbet_charge_end *end)
{
	end->taper_start_ms = 0;
	end->taper_last_ms = 0;
}

/* Current is positive into the cell. Time, not poll count, proves the tail.
 * A stopped converter's zero-current sample can never establish a full charge.
 */
static inline int courbet_end_update(struct courbet_charge_end *end, s64 now_ms,
		int eligible, int normal_full, int state, int voltage_uv,
		int current_ua, int target_uv, int percent)
{
	int taper, proven;

	end->last_state = state;
	if (end->hold_percent >= 0) {
		courbet_end_reset_taper(end);
		/* The relaxed voltage AND counted discharge must justify a restart. */
		if (percent <= (end->hold_percent >= 5 ? end->hold_percent - 5 : 0) &&
		    voltage_uv <= courbet_recharge_uv(target_uv) &&
		    current_ua >= -50000 && current_ua <= 10000) {
			if (!end->recharge_start_ms || now_ms - end->recharge_last_ms > 5000)
				end->recharge_start_ms = now_ms;
			end->recharge_last_ms = now_ms;
			if (now_ms - end->recharge_start_ms >= 30000) {
				end->hold_percent = -1;
				end->reason = COURBET_END_NONE;
				end->full = 0;
			}
		} else {
			end->recharge_start_ms = 0;
			end->recharge_last_ms = 0;
		}
		return end->hold_percent >= 0;
	}
	end->recharge_start_ms = 0;
	if (voltage_uv > 4480000 && current_ua > 10000) {
		end->hold_percent = percent;
		end->reason = COURBET_END_VOLTAGE;
		end->full = 0;
		courbet_end_reset_taper(end);
		return 1;
	}
	if (!eligible) {
		courbet_end_reset_taper(end);
		return 0;
	}
	proven = end->taper_start_ms && end->taper_last_ms &&
		now_ms - end->taper_last_ms <= 5000 &&
		now_ms - end->taper_start_ms >= 60000;
	taper = (state == 3 || state == 4) && current_ua >= 10000 &&
		current_ua <= 200000 && voltage_uv >= target_uv - 30000 &&
		voltage_uv <= target_uv + 30000;
	if (taper) {
		if (!end->taper_start_ms || now_ms - end->taper_last_ms > 5000)
			end->taper_start_ms = now_ms;
		end->taper_last_ms = now_ms;
		proven = now_ms - end->taper_start_ms >= 60000;
	} else if (state != 5) {
		courbet_end_reset_taper(end);
		proven = 0;
	}
	if ((taper && proven) || state == 5) {
		end->full = normal_full && proven && current_ua >= -10000 &&
			current_ua <= 200000 && voltage_uv >= target_uv - 50000 &&
			voltage_uv <= target_uv + 30000;
		end->hold_percent = end->full ? 100 : percent;
		end->reason = end->full ? COURBET_END_FULL :
			(target_uv < COURBET_MAX_FV_UV || (taper && proven) ?
			 COURBET_END_TARGET : COURBET_END_PREMATURE);
		courbet_end_reset_taper(end);
		return 1;
	}
	return 0;
}
#endif

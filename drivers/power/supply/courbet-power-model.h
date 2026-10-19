/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef COURBET_POWER_MODEL_H
#define COURBET_POWER_MODEL_H
#ifdef __KERNEL__
#include <linux/types.h>
#else
#include <stdint.h>
typedef int64_t s64;
#endif

#define COURBET_MAX_FCC_UA 3000000
#define COURBET_MAX_FV_UV 4450000
#define COURBET_MAX_ICL_UA 2000000
#define COURBET_UAMS_PER_UAH 3600000LL

struct courbet_soc {
	s64 charge_uams;
	int full_uah;
};

static inline void courbet_soc_add(struct courbet_soc *soc, s64 delta_uams)
{
	s64 max = (s64)soc->full_uah * COURBET_UAMS_PER_UAH;

	soc->charge_uams += delta_uams;
	if (soc->charge_uams < 0)
		soc->charge_uams = 0;
	if (soc->charge_uams > max)
		soc->charge_uams = max;
}

static inline int courbet_soc_percent(const struct courbet_soc *soc)
{
	s64 max = (s64)soc->full_uah * COURBET_UAMS_PER_UAH;

	return (soc->charge_uams * 100 + max / 2) / max;
}

static inline int courbet_ocv_uv(int voltage_uv, int current_ua, int resistance_uohm)
{
	return voltage_uv - (s64)current_ua * resistance_uohm / 1000000;
}

struct courbet_charge_limits {
	int fcc_ua;
	int fv_uv;
	int thermal_block;
};

/* Thermal block: -1 cold, +1 hot, 0 charging window. */
static inline struct courbet_charge_limits
courbet_charge_limits(int temp_deci_c, int previous_block, int fcc_ua, int fv_uv)
{
	struct courbet_charge_limits result = { fcc_ua, fv_uv, 0 };

	if (temp_deci_c <= 50 || (previous_block < 0 && temp_deci_c < 70))
		result.thermal_block = -1;
	if (temp_deci_c >= 450 || (previous_block > 0 && temp_deci_c > 430))
		result.thermal_block = 1;
	if (temp_deci_c < 150 && result.fcc_ua > 400000)
		result.fcc_ua = 400000;
	if (temp_deci_c >= 430) {
		if (result.fcc_ua > 500000)
			result.fcc_ua = 500000;
		if (result.fv_uv > 4100000)
			result.fv_uv = 4100000;
	}
	if (result.thermal_block)
		result.fcc_ua = 0;
	return result;
}
#endif

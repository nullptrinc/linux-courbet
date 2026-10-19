// SPDX-License-Identifier: GPL-2.0-only
/* Native PM6150 charging policy and QGauge FIFO coulomb accounting.
 * Register layout/conversions: Qualcomm courbet-r-oss SMB5 and QG drivers.
 * Android SDAM calibration is never modified. Linux SOC state lives on /data.
 */
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/module.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/pm_wakeirq.h>
#include "courbet-power-model.h"
#include "courbet-battery-profile.h"
#include "courbet-fast-model.h"
#include "courbet-charge-end.h"

#define QG_TYPE 0x4804
#define QG_SUBTYPE 0x4805
#define QG_PRESENT 0x4808
#define QG_V_AVG 0x4880
#define QG_I_AVG 0x4882
#define QG_V_NOW 0x48c0
#define QG_I_NOW 0x48c2
#define CHG_STATE 0x1006
#define CHG_FAULTS 0x1007
#define CHG_TEMP 0x100d
#define USB_RT_STS 0x1310
#define USB_ICL 0x1370

struct courbet_power {
	struct device *dev;
	struct regmap *regmap;
	struct iio_channel *therm, *vbus, *die;
	struct power_supply *source, *pump;
	struct mutex lock;
	struct courbet_soc soc;
	struct courbet_charge_end end;
	s64 last_fifo_ns, boot_ns, rest_ns, next_pps_ns;
	u32 fifo_count, fifo_errors;
	int requested_fcc, requested_fv, requested_icl, end_threshold;
	int applied_fcc, applied_fv, source_limit, behaviour, thermal_block;
	int last_online, policy_error, ocv_uv, resistance_uohm, safety_test;
	int pps_state, pps_uv, fast_error, contract_uv, contract_type, fast_thermal;
	bool pps_attempted, fast_allowed;
	bool limit_block, restored, ready, suspended;
	int applied_recharge;
	struct power_supply *battery, *usb;
	struct power_supply_battery_info *info;
	struct delayed_work poll;
	int current_scale;
	int design_vmin, design_vmax, design_full;
};

static int courbet_sample(struct courbet_power *p, unsigned int reg,
			 bool is_current, int *value)
{
	u8 raw[2];
	int ret = regmap_bulk_read(p->regmap, reg, raw, sizeof(raw));

	if (ret)
		return ret;
	if (is_current)
		/* PSY: positive means net current into the cell. */
		*value = -div_s64((s64)(s16)get_unaligned_le16(raw) *
				 p->current_scale, 1000);
	else
		*value = div_u64((u64)get_unaligned_le16(raw) * 194637, 1000);
	return 0;
}

static int courbet_online(struct courbet_power *p, int *value)
{
	unsigned int status;
	int ret = regmap_read(p->regmap, USB_RT_STS, &status);

	if (!ret)
		*value = !!(status & BIT(4));
	return ret;
}

static int courbet_status(struct courbet_power *p, int *value)
{
	unsigned int state;
	int online, current_ua, ret;

	ret = courbet_online(p, &online);
	if (ret)
		return ret;
	if (!online) {
		*value = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	}
	ret = regmap_read(p->regmap, CHG_STATE, &state);
	if (ret)
		return ret;
	ret = courbet_sample(p, QG_I_NOW, true, &current_ua);
	if (ret)
		return ret;
	if (current_ua < -10000)
		*value = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (READ_ONCE(p->pps_state) == 2 && current_ua > 10000)
		*value = POWER_SUPPLY_STATUS_CHARGING;
	else if (READ_ONCE(p->end.full) && READ_ONCE(p->end.hold_percent) >= 0)
		*value = POWER_SUPPLY_STATUS_FULL;
	else if ((state & 7) >= 1 && (state & 7) <= 4 && current_ua > 10000)
		*value = POWER_SUPPLY_STATUS_CHARGING;
	else
		*value = POWER_SUPPLY_STATUS_NOT_CHARGING;
	return 0;
}

/* All policy register writes are checked; incomplete setup leaves charging off. */
static int courbet_write(struct courbet_power *p, unsigned int reg,
			 unsigned int mask, unsigned int value)
{
	unsigned int actual;
	int ret = regmap_update_bits(p->regmap, reg, mask, value);

	if (!ret)
		ret = regmap_read(p->regmap, reg, &actual);
	if (!ret && (actual & mask) != (value & mask))
		ret = -EIO;
	if (ret)
		dev_err_ratelimited(p->dev, "Register %04x write/readback failed: %d\n",
				    reg, ret);
	return ret;
}

static int courbet_write_be16(struct courbet_power *p, unsigned int reg, u16 value)
{
	int ret = courbet_write(p, reg, 0xff, value >> 8);

	return ret ? ret : courbet_write(p, reg + 1, 0xff, value & 0xff);
}

/* Same ADC5 1.875 V / 0x70e4 scale and NTC table as the thermistor reader. */
static int courbet_therm_raw(int temp_deci_c)
{
	static const int temperature[] = { 0, 50, 100, 150, 200, 250,
					  300, 350, 400, 450, 500 };
	static const int voltage_mv[] = { 1463, 1370, 1268, 1160, 1049, 937,
					 828, 726, 630, 544, 467 };
	int i, mv;

	temp_deci_c = clamp(temp_deci_c, 0, 500);
	for (i = 0; i < ARRAY_SIZE(temperature) - 1; i++)
		if (temp_deci_c <= temperature[i + 1])
			break;
	mv = voltage_mv[i] + (voltage_mv[i + 1] - voltage_mv[i]) *
		(temp_deci_c - temperature[i]) / 50;
	return DIV_ROUND_CLOSEST(mv * 0x70e4, 1875);
}

static int courbet_charger_init(struct courbet_power *p)
{
	static const struct { u16 reg; u8 mask, value; } init[] = {
		{ 0x1042, 0x01, 0x00 }, /* charging off during configuration */
		{ 0x1043, 0x01, 0x00 }, /* clear an inherited charging pause */
		{ 0x10c0, 0x08, 0x00 }, /* use ADC current termination */
		{ 0x1060, 0xff, 0x03 }, /* OEM normal precharge setting */
		{ 0x1061, 0xff, 0x08 }, /* 400 mA until policy validates sensors */
		{ 0x1070, 0xff, 0x55 }, /* OEM normal FV: 3.6 V + 85 * 10 mV */
		{ 0x106c, 0x01, 0x00 }, /* PM6150 OEM synchronous ADC termination */
		{ 0x106b, 0x0f, 0x0a }, /* 3 voltage samples for recharge */
		{ 0x1051, 0x07, 0x04 }, /* voltage-based automatic recharge */
		{ 0x10a2, 0xff, 0x02 }, /* 768 minute fast-charge safety timer */
		{ 0x1090, 0x1f, 0x10 }, /* autonomous hard JEITA; SW handles derating */
		{ 0x1092, 0xff, 0x00 }, /* no inherited soft FCC compensation */
		{ 0x1093, 0xff, 0x00 }, /* software derating uses the explicit FCC cap */
		{ 0x1362, 0x0c, 0x08 }, /* BC1.2 on, autonomous HVDCP off */
		{ 0x1344, 0x0f, 0x08 }, /* PD/PPS continuous input allowance */
		{ 0x1360, 0x0f, 0x0c }, /* 5..12 V HW range; sink PDOs capped at10 V */
		{ 0x1150, 0x0f, 0x08 }, /* OEM 9 V buck frequency, before PD attach */
		{ 0x1365, 0x10, 0x10 }, /* software ICL survives APSD */
		{ 0x1366, 0x03, 0x01 }, /* high-current mode, software current cap */
		{ 0x1342, 0x01, 0x01 }, /* ICL software override */
		{ 0x1370, 0xff, 0x0a }, /* 500 mA fallback until source known */
		{ 0x1380, 0xbf, 0x17 }, /* AICL, periodic rerun, collapse response */
		{ 0x1381, 0x07, 0x03 }, /* 4.3 V AICL threshold */
		{ 0x1384, 0x3f, 0x03 }, /* 4.3 V continuous AICL threshold */
		{ 0x1661, 0x03, 0x01 }, /* rerun AICL every 12 seconds */
		{ 0x1670, 0xb7, 0x01 }, /* native die-temperature comparator */
		{ 0x1653, 0xff, 0x8b }, /* watchdog bite disables charge, no AFP */
		{ 0x1651, 0xc2, 0x02 }, /* watchdog on plug; worker pets it */
		{ 0x1340, 0x01, 0x00 }, /* input powers system even when inhibited */
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(init); i++) {
		ret = courbet_write(p, init[i].reg, init[i].mask, init[i].value);
		if (ret)
			goto fail;
	}
	/* Signed ADC termination thresholds, PM6150 10 A range, big endian. */
	ret = courbet_write_be16(p, 0x1067, (u16)(s16)(-100 * 32767 / 10000));
	if (ret)
		goto fail;
	ret = courbet_write_be16(p, 0x1069, (u16)(s16)(50 * 32767 / 10000));
	if (ret)
		goto fail;
	ret = courbet_write_be16(p, 0x107e, 4200 * 1000000LL / 194637);
	if (ret)
		goto fail;
	/* Autonomous 5..45 C hard window; 15/43 C soft health indications. */
	ret = courbet_write_be16(p, 0x1094, courbet_therm_raw(430));
	if (ret)
		goto fail;
	ret = courbet_write_be16(p, 0x1096, courbet_therm_raw(150));
	if (ret)
		goto fail;
	ret = courbet_write_be16(p, 0x1098, courbet_therm_raw(450));
	if (ret)
		goto fail;
	ret = courbet_write_be16(p, 0x109a, courbet_therm_raw(50));
	if (ret)
		goto fail;
	p->applied_recharge = 4200000;
	p->last_online = -1;
	p->policy_error = 0;
	return 0;
fail:
	regmap_update_bits(p->regmap, 0x1042, BIT(0), 0);
	return ret;
}

static int courbet_resistance(struct courbet_power *p, int temp_deci_c, int soc)
{
	int row, col, next, value, other;

	for (row = 0; row < ARRAY_SIZE(courbet_profile_soc) - 2; row++)
		if (soc >= courbet_profile_soc[row + 1])
			break;
	for (col = 0; col < ARRAY_SIZE(courbet_profile_temp) - 2; col++)
		if (temp_deci_c <= courbet_profile_temp[col + 1] * 10)
			break;
	next = courbet_profile_soc[row] - courbet_profile_soc[row + 1];
	soc = clamp(soc, courbet_profile_soc[row + 1], courbet_profile_soc[row]);
	value = courbet_profile_r_uohm[row][col] +
		(courbet_profile_r_uohm[row + 1][col] - courbet_profile_r_uohm[row][col]) *
		(courbet_profile_soc[row] - soc) / next;
	other = courbet_profile_r_uohm[row][col + 1] +
		(courbet_profile_r_uohm[row + 1][col + 1] - courbet_profile_r_uohm[row][col + 1]) *
		(courbet_profile_soc[row] - soc) / next;
	temp_deci_c = clamp(temp_deci_c, courbet_profile_temp[col] * 10,
			   courbet_profile_temp[col + 1] * 10);
	return value + (other - value) * (temp_deci_c - courbet_profile_temp[col] * 10) /
		((courbet_profile_temp[col + 1] - courbet_profile_temp[col]) * 10);
}

static int courbet_source_limit(struct courbet_power *p)
{
	union power_supply_propval online, current_max;
	unsigned int done, type;

	/* TCPM's source PSY describes the attached source when we are a sink. */
	if (!power_supply_get_property(p->source, POWER_SUPPLY_PROP_ONLINE, &online) &&
	    online.intval &&
	    !power_supply_get_property(p->source, POWER_SUPPLY_PROP_CURRENT_MAX, &current_max) &&
	    current_max.intval > 0)
		return min(current_max.intval, COURBET_MAX_ICL_UA);
	if (!regmap_read(p->regmap, 0x1307, &done) && (done & BIT(0)) &&
	    !regmap_read(p->regmap, 0x1308, &type) && (type & (BIT(2) | BIT(3))))
		return 1500000; /* positively detected BC1.2 CDP/DCP */
	return 500000; /* unknown/floating sources never get an elevated cap */
}

/* Native TCPM PPS control. Never enable a direct-charge pump on fixed PD. */
static int courbet_psy_set(struct power_supply *psy, enum power_supply_property prop, int value)
{
	union power_supply_propval v = { .intval = value };

	return power_supply_set_property(psy, prop, &v);
}

static int courbet_psy_get(struct power_supply *psy, enum power_supply_property prop, int *value)
{
	union power_supply_propval v;
	int ret = power_supply_get_property(psy, prop, &v);

	if (!ret)
		*value = v.intval;
	return ret;
}

static int courbet_fast_stop(struct courbet_power *p)
{
	int ret, source_ret = 0;

	if (!p->pump)
		return 0;
	ret = courbet_psy_set(p->pump, POWER_SUPPLY_PROP_ONLINE, 0);
	/* Stop CP before asking the adapter to change to any fixed PDO. */
	if (!ret && p->pps_state)
		source_ret = courbet_psy_set(p->source, POWER_SUPPLY_PROP_ONLINE, 1);
	if (!ret) {
		p->pps_state = 0;
		p->pps_uv = 0;
	}
	return ret ? ret : source_ret;
}

/* Returns 1 when CP owns cell charging, 0 for SMB5, negative on failure. */
static int courbet_fast_update(struct courbet_power *p, bool allow, int voltage,
			       int current_ua, int fcc, int fv, int icl, int source_type)
{
	int ret, online, min_uv, max_uv, max_ua, cp_online, cp_ibus, target, source_ua;

	if (!allow || !p->fast_allowed || !p->pump || (source_type != POWER_SUPPLY_USB_TYPE_PD_PPS &&
	     source_type != POWER_SUPPLY_USB_TYPE_PD)) {
		if (p->pps_state) {
			ret = courbet_fast_stop(p);
			if (!ret)
				p->pps_attempted = false;
			return ret;
		}
		return 0;
	}
	if (p->pps_state && ktime_get_boottime_ns() < p->next_pps_ns)
		return 1;
	if (!p->pps_state) {
		if (p->pps_attempted)
			return 0;
		p->pps_attempted = true;
		ret = courbet_write(p, 0x1042, 1, 0);
		if (!ret)
			ret = courbet_write(p, 0x1370, 0xff, 6); /* 300 mA system reserve */
		if (!ret)
			ret = courbet_psy_set(p->source, POWER_SUPPLY_PROP_ONLINE, 2);
		if (ret == -EOPNOTSUPP)
			return 0; /* Fixed PD source: ordinary buck fast charging. */
		if (ret)
			goto fail;
		p->pps_state = 1;
		ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_ONLINE, &online);
		if (!ret)
			ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_VOLTAGE_MIN, &min_uv);
		if (!ret)
			ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_VOLTAGE_MAX, &max_uv);
		if (!ret)
			ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_CURRENT_MAX, &max_ua);
		if (ret)
			goto fail;
		target = (voltage * 2 + 300000) / 20000 * 20000;
		if (online != 2 || target < min_uv || target > max_uv ||
		    target > COURBET_PPS_MAX_UV || max_ua < 1500000) {
			ret = -ERANGE;
			goto fail;
		}
		source_ua = min(min(icl, max_ua), fcc / 2 + 300000) / 50000 * 50000;
		ret = courbet_psy_set(p->source, POWER_SUPPLY_PROP_CURRENT_NOW, source_ua);
		if (!ret)
			ret = courbet_psy_set(p->source, POWER_SUPPLY_PROP_VOLTAGE_NOW, target);
		if (ret)
			goto fail;
		p->pps_uv = target;
		p->next_pps_ns = ktime_get_boottime_ns() + NSEC_PER_SEC;
		return 1; /* Allow fresh ADC conversions before enabling CP. */
	}
	ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_ONLINE, &online);
	if (!ret && online != 2)
		ret = -ENOTCONN;
	if (!ret)
		ret = courbet_psy_get(p->pump, POWER_SUPPLY_PROP_ONLINE, &cp_online);
	if (!ret && p->pps_state == 2 && !cp_online)
		ret = -EIO;
	if (ret)
		goto fail;
	if (p->pps_state == 1) {
		ret = courbet_psy_set(p->pump, POWER_SUPPLY_PROP_ONLINE, 1);
		if (ret)
			goto fail;
		p->pps_state = 2;
		p->next_pps_ns = ktime_get_boottime_ns() + NSEC_PER_SEC;
		return 1;
	}
	source_ua = min(icl, fcc / 2 + 300000) / 50000 * 50000;
	ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_CURRENT_NOW, &max_ua);
	if (!ret && max_ua != source_ua)
		ret = courbet_psy_set(p->source, POWER_SUPPLY_PROP_CURRENT_NOW, source_ua);
	if (ret)
		goto fail;
	ret = courbet_psy_get(p->pump, POWER_SUPPLY_PROP_CURRENT_NOW, &cp_ibus);
	if (ret)
		goto fail;
	target = courbet_pps_step(p->pps_uv, voltage, fv, current_ua,
				 fcc, cp_ibus, min(1500000, icl - 300000), 0, 0);
	/* A broken/stale cell measurement must never ramp the source unboundedly. */
	if (!courbet_pump_running_window(target, voltage)) {
		ret = -ERANGE;
		goto fail;
	}
	if (target != p->pps_uv) {
		ret = courbet_psy_set(p->source, POWER_SUPPLY_PROP_VOLTAGE_NOW, target);
		if (ret)
			goto fail;
		p->pps_uv = target;
	}
	/* Do not restart a pump stopped by its independent supervisor mid-request. */
	ret = courbet_psy_get(p->pump, POWER_SUPPLY_PROP_ONLINE, &cp_online);
	if (!ret && !cp_online)
		ret = -EIO;
	if (!ret)
		ret = courbet_psy_set(p->pump, POWER_SUPPLY_PROP_ONLINE, 1); /* heartbeat */
	if (ret)
		goto fail;
	p->next_pps_ns = ktime_get_boottime_ns() + NSEC_PER_SEC;
	return 1;
fail:
	p->fast_error = ret;
	if (courbet_fast_stop(p))
		return -EIO;
	dev_warn_ratelimited(p->dev, "PPS stopped, using SMB5: %d\n", ret);
	return 0;
}

static int courbet_adapter_config(struct courbet_power *p, int *source_type, int *vbus)
{
	int ret, online, threshold;

	*source_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
	*vbus = 5000000;
	ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_ONLINE, &online);
	if (ret || !online)
		goto configure;
	ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_USB_TYPE, source_type);
	if (!ret)
		ret = courbet_psy_get(p->source, POWER_SUPPLY_PROP_VOLTAGE_NOW, vbus);
	if (ret)
		return ret;
	if (*vbus < 4500000 || *vbus > COURBET_PPS_MAX_UV)
		return -ERANGE;
configure:
	p->contract_uv = *vbus;
	p->contract_type = *source_type;
	/* Continuous input allowance covers attach at 5 V, fixed 9 V and PPS. */
	ret = courbet_write(p, 0x1344, 0x0f, 8);
	if (!ret)
		ret = courbet_write(p, 0x1150, 0x0f, *vbus >= 9000000 ? 8 :
				    *vbus > 5500000 ? 11 : 15);
	threshold = clamp(*vbus / 1000 - 1000, 4300, 8800);
	threshold = threshold < 5600 ? (threshold - 4000) / 100 :
		(threshold - 5600) / 200 + 16;
	if (!ret)
		ret = courbet_write(p, 0x1384, 0x3f, threshold);
	return ret;
}

static ssize_t fast_charge_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&p->lock);
	n = sysfs_emit(buf, "mode=%s allowed=%d contract_uv=%d pps_uv=%d error=%d\n",
			p->pps_state == 2 ? "pps-pump" : p->pps_state == 1 ? "pps-start" :
			p->contract_type == POWER_SUPPLY_USB_TYPE_PD_PPS ? "pps-buck" :
			p->contract_type == POWER_SUPPLY_USB_TYPE_PD ? "pd-buck" : "usb",
			p->fast_allowed, p->contract_uv, p->pps_uv, p->fast_error);
	mutex_unlock(&p->lock);
	return n;
}

/* 0 disables pump requests; 1 permits supervised PPS. Fixed PD stays with TCPM. */
static ssize_t fast_charge_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	bool allow;
	int ret = kstrtobool(buf, &allow);

	if (ret)
		return ret;
	mutex_lock(&p->lock);
	p->fast_allowed = allow;
	p->pps_attempted = false;
	p->fast_error = 0;
	if (!allow)
		ret = courbet_fast_stop(p);
	mutex_unlock(&p->lock);
	mod_delayed_work(system_wq, &p->poll, 0);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(fast_charge);

static int courbet_policy(struct courbet_power *p, int temp_deci_c,
			  int die_millic, int voltage_uv, int current_ua)
{
	struct courbet_charge_limits limits;
	unsigned int faults, thermal, state;
	int online, ret, icl, percent, source_type, source_uv, pump, buck_icl;
	bool enable, was_full = p->end.full;

	ret = courbet_online(p, &online);
	if (ret)
		goto fail;
	ret = regmap_read(p->regmap, CHG_FAULTS, &faults);
	if (ret)
		goto fail;
	ret = regmap_read(p->regmap, CHG_TEMP, &thermal);
	if (!ret)
		ret = regmap_read(p->regmap, CHG_STATE, &state);
	if (ret)
		goto fail;
	if (online != p->last_online) {
		/* Reassert the driver-owned path settings on each attach. */
		ret = courbet_write(p, 0x1365, BIT(4), BIT(4));
		if (ret)
			goto fail;
		ret = courbet_write(p, 0x1366, 3, 1);
		if (ret)
			goto fail;
		ret = courbet_write(p, 0x1342, 1, 1);
		if (ret)
			goto fail;
		p->last_online = online;
		if (!online) {
			p->pps_attempted = false;
			p->fast_error = 0;
		}
	}
	limits = courbet_charge_limits(temp_deci_c, p->thermal_block,
				       p->requested_fcc, p->requested_fv);
	p->thermal_block = limits.thermal_block;
	percent = courbet_soc_percent(&p->soc);
	/* Conservative last-ten-percent cap while the independent taper is measured. */
	if (percent >= 90)
		limits.fcc_ua = min(limits.fcc_ua, 500000);
	if (p->end_threshold < 100 && percent >= p->end_threshold)
		p->limit_block = true;
	if (percent <= max(p->end_threshold - 5, 0) || p->end_threshold == 100)
		p->limit_block = false;
	enable = online && limits.fcc_ua >= 50000 && !p->limit_block &&
		p->behaviour == POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		!(faults & BIT(1)) && !(thermal & 0x0f) &&
		die_millic < 85000 && voltage_uv < 4500000 &&
		ktime_get_boottime_ns() - p->boot_ns >= 5LL * NSEC_PER_SEC &&
		(p->restored || ktime_get_boottime_ns() - p->boot_ns > 60LL * NSEC_PER_SEC);
	ret = courbet_adapter_config(p, &source_type, &source_uv);
	if (ret)
		goto fail;
	icl = min(p->requested_icl, courbet_source_limit(p));
	if (source_uv <= 5500000)
		icl = min(icl, 1500000);
	if (die_millic >= 75000)
		icl = min(icl, 500000);
	if (courbet_end_update(&p->end, div_s64(ktime_get_boottime_ns(), NSEC_PER_MSEC),
		enable && !p->pps_state,
		p->end_threshold == 100 && limits.fv_uv == COURBET_MAX_FV_UV &&
		limits.fcc_ua >= 500000 &&
		icl >= 500000 && temp_deci_c >= 150 && temp_deci_c < 430,
		state & 7, voltage_uv, current_ua, limits.fv_uv, percent))
		enable = false;
	if (p->end.full && !was_full)
		p->soc.charge_uams = (s64)p->soc.full_uah * COURBET_UAMS_PER_UAH;
	if (p->applied_recharge != courbet_recharge_uv(limits.fv_uv)) {
		ret = courbet_write_be16(p, 0x107e,
			(s64)courbet_recharge_uv(limits.fv_uv) * 1000 / 194637);
		if (ret)
			goto fail;
		p->applied_recharge = courbet_recharge_uv(limits.fv_uv);
	}
	p->fast_thermal = courbet_fast_thermal_block(temp_deci_c, p->fast_thermal);
	pump = courbet_fast_update(p,
		!p->fast_thermal && courbet_pump_allowed(temp_deci_c, voltage_uv, limits.fv_uv,
			limits.fcc_ua, icl, percent, p->end_threshold, enable, die_millic),
		voltage_uv, current_ua, limits.fcc_ua, limits.fv_uv, icl, source_type);
	if (pump < 0) {
		ret = pump;
		goto fail;
	}
	buck_icl = pump ? 300000 : icl;
	limits.fcc_ua = min(limits.fcc_ua, COURBET_BUCK_FCC_UA);
	ret = courbet_write(p, 0x1370, 0xff, buck_icl / 50000);
	if (ret)
		goto fail;
	ret = courbet_write(p, 0x1061, 0xff, limits.fcc_ua / 50000);
	if (ret)
		goto fail;
	ret = courbet_write(p, 0x1070, 0xff, (limits.fv_uv - 3600000) / 10000);
	if (ret)
		goto fail;
	ret = courbet_write(p, 0x1340, 1, icl < 50000 ? 1 : 0);
	if (ret)
		goto fail;
	ret = courbet_write(p, 0x1042, 1, enable && !pump ? 1 : 0);
	if (ret)
		goto fail;
	ret = regmap_write(p->regmap, 0x1643, BIT(0)); /* self-clearing pet command */
	if (ret)
		goto fail;
	p->policy_error = 0;
	p->applied_fcc = limits.fcc_ua / 50000 * 50000;
	p->applied_fv = limits.fv_uv / 10000 * 10000;
	p->source_limit = icl;
	return 0;
fail:
	p->policy_error = ret;
	courbet_end_reset_taper(&p->end);
	courbet_fast_stop(p);
	regmap_update_bits(p->regmap, 0x1042, 1, 0);
	return ret;
}

static int courbet_qg_init(struct courbet_power *p)
{
	int ret;

	/* Reset the old FIFO before choosing our own accumulation interval. */
	ret = courbet_write(p, 0x4841, 1, 0);
	if (ret)
		return ret;
	ret = courbet_write(p, 0x4841, 1, 1);
	if (ret)
		return ret;
	ret = courbet_write(p, 0x4851, 0x3f, 0x3c); /* 8 records x 32 samples */
	if (ret)
		goto release;
	ret = courbet_write(p, 0x4852, 0xff, 10); /* nominal 100 ms per sample */
	if (ret)
		goto release;
	/* Keep S2 counting; an untracked S3 transition would lose charge data. */
	ret = courbet_write(p, 0x485e, 0xff, 0);
	if (ret)
		goto release;
	ret = courbet_write(p, 0x485f, 0xff, 1);
release:
	if (courbet_write(p, 0x4841, 1, 0) && !ret)
		ret = -EIO;
	p->last_fifo_ns = ktime_get_boottime_ns();
	return ret;
}

static irqreturn_t courbet_fifo_irq(int irq, void *data)
{
	struct courbet_power *p = data;
	u8 raw[16];
	s64 delta = 0, now = ktime_get_boottime_ns(), elapsed;
	int i, current_ua, ret;

	/* Snapshot promptly even if a policy/selftest operation holds the SOC mutex. */
	ret = regmap_bulk_read(p->regmap, 0x48a0, raw, sizeof(raw));
	mutex_lock(&p->lock);
	elapsed = div_s64(now - p->last_fifo_ns, NSEC_PER_MSEC);
	/* Ignore an old latched event from the pre-driver configuration. */
	if (elapsed < 12000)
		goto out;
	if (ret)
		goto error;
	for (i = 0; i < 8; i++) {
		if (get_unaligned_le16(raw + i * 2) == 0x8000)
			goto error;
		current_ua = -div_s64((s64)(s16)get_unaligned_le16(raw + i * 2) *
				       p->current_scale, 1000);
		/* PM6150 QG clock correction: 32 kHz nominal / 32764 Hz. */
		delta += div_s64((s64)current_ua * 32 * 100 * 32000, 32764);
	}
	courbet_soc_add(&p->soc, delta);
	p->last_fifo_ns = now;
	p->fifo_count++;
	if (p->soc.charge_uams < (s64)p->soc.full_uah * COURBET_UAMS_PER_UAH * 995 / 1000)
		p->end.full = false;
	if (elapsed > 40000)
		p->fifo_errors++; /* missing coverage remains visible to diagnostics */
	goto out;
error:
	p->fifo_errors++;
out:
	mutex_unlock(&p->lock);
	power_supply_changed(p->battery);
	return IRQ_HANDLED;
}

static ssize_t soc_state_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&p->lock);
	len = sysfs_emit(buf, "2 %lld %lld %d %d %d %d\n", (s64)ktime_get_real_seconds(),
			 div_s64(p->soc.charge_uams, COURBET_UAMS_PER_UAH), p->ocv_uv,
			 p->end.full, p->end.hold_percent, p->end.reason);
	mutex_unlock(&p->lock);
	return len;
}

static ssize_t soc_state_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	long long saved_time, charge;
	int version, ocv, full, hold = -1, reason = 0, fields, ret = 0;
	s64 now = ktime_get_real_seconds();
	char extra;

	if (sscanf(buf, "%d", &version) != 1)
		return -EINVAL;
	fields = version == 1 ?
		sscanf(buf, "%d %lld %lld %d %d %c",
		       &version, &saved_time, &charge, &ocv, &full, &extra) :
		sscanf(buf, "%d %lld %lld %d %d %d %d %c",
		       &version, &saved_time, &charge, &ocv, &full, &hold, &reason, &extra);
	if (!((version == 1 && fields == 5) || (version == 2 && fields == 7)))
		return -EINVAL;
	mutex_lock(&p->lock);
	if (full < 0 || full > 1 || charge < 0 || charge > p->soc.full_uah ||
	    hold < -1 || hold > 100 || reason < 0 || reason > COURBET_END_TARGET ||
	    ((hold == -1) != (reason == COURBET_END_NONE)) ||
	    (version == 2 && full && (hold != 100 || reason != COURBET_END_FULL ||
				     charge < (s64)p->soc.full_uah * 985 / 1000)) ||
	    saved_time < 0 || saved_time > now + 5 || now - saved_time > 3600 ||
	    ocv < 2500000 || ocv > 4600000 || abs(ocv - p->ocv_uv) > 150000)
		ret = -EINVAL;
	else if (p->restored || ktime_get_boottime_ns() - p->boot_ns > 60LL * NSEC_PER_SEC)
		ret = -EPERM;
	else if (version == 1 && full) {
		/* Version1 may contain the known premature-termination100% anchor.
		 * Keep the fresh OCV seed instead of importing that charge or flag.
		 */
		ret = -ESTALE;
	} else {
		p->soc.charge_uams = charge * COURBET_UAMS_PER_UAH;
		p->restored = true;
		/* Older version2 snapshots may retain full between98.5 and99.5%.
		 * Preserve the counter and completion hold, but display99% honestly.
		 */
		p->end.full = full && charge >= (s64)p->soc.full_uah * 995 / 1000;
		p->end.hold_percent = hold;
		p->end.reason = reason;
		courbet_end_reset_taper(&p->end);
	}
	mutex_unlock(&p->lock);
	if (!ret)
		power_supply_changed(p->battery);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(soc_state);

static ssize_t gauge_diagnostics_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&p->lock);
	len = sysfs_emit(buf, "fifo_count=%u fifo_errors=%u restored=%d ocv_uv=%d "
			 "resistance_uohm=%d thermal_block=%d policy_error=%d source_limit_ua=%d\n",
			 p->fifo_count, p->fifo_errors, p->restored, p->ocv_uv,
			 p->resistance_uohm, p->thermal_block, p->policy_error, p->source_limit);
	mutex_unlock(&p->lock);
	return len;
}
static DEVICE_ATTR_RO(gauge_diagnostics);

static ssize_t charge_end_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	ssize_t len;

	mutex_lock(&p->lock);
	len = sysfs_emit(buf, "hold=%d hold_percent=%d full=%d reason=%d state=%d "
			 "taper_ms=%lld recharge_uv=%d\n",
			 p->end.hold_percent >= 0, p->end.hold_percent, p->end.full,
			 p->end.reason, p->end.last_state,
			 p->end.taper_start_ms ? p->end.taper_last_ms - p->end.taper_start_ms : 0,
			 p->applied_recharge);
	mutex_unlock(&p->lock);
	return len;
}
static DEVICE_ATTR_RO(charge_end);

/* Tightening a hardware cutoff exercises the real JEITA comparator without
 * heating the cell or allowing any temperature outside the production window.
 */
static ssize_t charger_safety_test_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct courbet_power *p = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", READ_ONCE(p->safety_test));
}

static ssize_t charger_safety_test_store(struct device *dev,
					struct device_attribute *attr,
					const char *buf, size_t count)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	unsigned int thermal, state, enabled_modes;
	int temp, i, ret, restore_ret;

	if (!sysfs_streq(buf, "1"))
		return -EINVAL;
	mutex_lock(&p->lock);
	ret = iio_read_channel_processed(p->therm, &temp);
	if (ret < 0)
		goto out;
	ret = regmap_read(p->regmap, CHG_STATE, &state);
	if (ret)
		goto out;
	if (!p->ready || p->suspended || p->policy_error || temp < 15000 ||
	    temp > 40000 || (state & 7) < 1 || (state & 7) > 4) {
		ret = -EBUSY;
		goto out;
	}
	regmap_write(p->regmap, 0x1643, BIT(0));
	/* The only temporary setting is stricter than the normal 45 C cutoff. */
	ret = courbet_write_be16(p, 0x1098, courbet_therm_raw(temp / 100 - 20));
	if (!ret) {
		ret = -ETIMEDOUT;
		for (i = 0; i < 50; i++) {
			msleep(100);
			if (!regmap_read(p->regmap, CHG_TEMP, &thermal) && (thermal & BIT(3)) &&
			    !regmap_read(p->regmap, 0x100b, &enabled_modes) && !(enabled_modes & 7)) {
				ret = 0;
				break;
			}
		}
	}
	restore_ret = courbet_write_be16(p, 0x1098, courbet_therm_raw(450));
	if (restore_ret) {
		regmap_update_bits(p->regmap, 0x1042, 1, 0);
		ret = restore_ret;
	}
	p->safety_test = ret ? ret : 1;
	dev_info(p->dev, "Hardware JEITA tightened-cutoff selftest: %d\n", p->safety_test);
out:
	mutex_unlock(&p->lock);
	mod_delayed_work(system_wq, &p->poll, 0);
	return ret ? ret : count;
}
static DEVICE_ATTR_RW(charger_safety_test);

static struct attribute *courbet_power_attrs[] = {
	&dev_attr_soc_state.attr, &dev_attr_gauge_diagnostics.attr,
	&dev_attr_charger_safety_test.attr, &dev_attr_fast_charge.attr,
	&dev_attr_charge_end.attr, NULL,
};
ATTRIBUTE_GROUPS(courbet_power);

static enum power_supply_property courbet_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT, POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH, POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP, POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_CHARGE_NOW, POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_VOLTAGE_OCV,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD,
	POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN, POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int courbet_battery_get(struct power_supply *psy,
			      enum power_supply_property prop,
			      union power_supply_propval *value)
{
	struct courbet_power *p = power_supply_get_drvdata(psy);
	unsigned int status;
	int ret, temp;

	switch (prop) {
	case POWER_SUPPLY_PROP_PRESENT:
		ret = regmap_read(p->regmap, QG_PRESENT, &status);
		if (!ret)
			value->intval = !!(status & BIT(0));
		return ret;
	case POWER_SUPPLY_PROP_STATUS:
		return courbet_status(p, &value->intval);
	case POWER_SUPPLY_PROP_HEALTH:
		ret = regmap_read(p->regmap, CHG_FAULTS, &status);
		if (ret)
			return ret;
		if (status & BIT(1)) {
			value->intval = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
			return 0;
		}
		ret = regmap_read(p->regmap, CHG_TEMP, &status);
		if (ret)
			return ret;
		if (status & (BIT(3) | BIT(1)))
			value->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
		else if (status & (BIT(2) | BIT(0)))
			value->intval = POWER_SUPPLY_HEALTH_COLD;
		else if (status & BIT(5))
			value->intval = POWER_SUPPLY_HEALTH_WARM;
		else if (status & BIT(4))
			value->intval = POWER_SUPPLY_HEALTH_COOL;
		else
			value->intval = POWER_SUPPLY_HEALTH_GOOD;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		value->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return courbet_sample(p, QG_V_NOW, false, &value->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_AVG:
		return courbet_sample(p, QG_V_AVG, false, &value->intval);
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		return courbet_sample(p, QG_I_NOW, true, &value->intval);
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		return courbet_sample(p, QG_I_AVG, true, &value->intval);
	case POWER_SUPPLY_PROP_TEMP:
		ret = iio_read_channel_processed(p->therm, &temp);
		if (ret < 0)
			return ret;
		value->intval = temp / 100;
		return 0;
	default:
		break;
	}
	/* Registration can emit an event before the battery info is loaded. */
	if (!p->info)
		return -EAGAIN;
	switch (prop) {
	case POWER_SUPPLY_PROP_CAPACITY:
		mutex_lock(&p->lock);
		if (!p->ready) {
			mutex_unlock(&p->lock);
			return -EAGAIN;
		}
		value->intval = min(courbet_soc_percent(&p->soc), p->end.full ? 100 : 99);
		mutex_unlock(&p->lock);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		value->intval = READ_ONCE(p->soc.full_uah);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		mutex_lock(&p->lock);
		value->intval = div_s64(p->soc.charge_uams, COURBET_UAMS_PER_UAH);
		mutex_unlock(&p->lock);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_OCV:
		value->intval = READ_ONCE(p->ocv_uv);
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		value->intval = READ_ONCE(p->requested_fcc);
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		value->intval = READ_ONCE(p->requested_fv);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		value->intval = READ_ONCE(p->end_threshold);
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		value->intval = READ_ONCE(p->behaviour);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		value->intval = p->design_vmin;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		value->intval = p->design_vmax;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		value->intval = p->design_full;
		return 0;
	default:
		return -EINVAL;
	}
}

static enum power_supply_property courbet_usb_props[] = {
	POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
};

static int courbet_usb_get(struct power_supply *psy,
			  enum power_supply_property prop,
			  union power_supply_propval *value)
{
	struct courbet_power *p = power_supply_get_drvdata(psy);
	unsigned int raw;
	int ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		return courbet_online(p, &value->intval);
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = iio_read_channel_processed(p->vbus, &value->intval);
		return ret < 0 ? ret : 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = regmap_read(p->regmap, USB_ICL, &raw);
		if (!ret)
			value->intval = READ_ONCE(p->pps_state) ? READ_ONCE(p->source_limit) : raw * 50000;
		return ret;
	default:
		return -EINVAL;
	}
}

static int courbet_battery_writable(struct power_supply *psy,
				    enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX ||
		prop == POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX ||
		prop == POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD ||
		prop == POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR;
}

static int courbet_battery_set(struct power_supply *psy,
			      enum power_supply_property prop,
			      const union power_supply_propval *value)
{
	struct courbet_power *p = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&p->lock);
	if (!p->ready || p->suspended) {
		ret = -EAGAIN;
		goto out;
	}
	switch (prop) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (value->intval < 0 || value->intval > COURBET_MAX_FCC_UA)
			ret = -EINVAL;
		else
			p->requested_fcc = value->intval / 50000 * 50000;
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		if (value->intval < 3600000 || value->intval > COURBET_MAX_FV_UV)
			ret = -EINVAL;
		else {
			/* A higher requested target starts a new lower-FV completion cycle.
			 * Never use this to clear a full, fault or premature-stop hold.
			 */
			if (value->intval > p->requested_fv && p->end.reason == COURBET_END_TARGET) {
				p->end.hold_percent = -1;
				p->end.reason = COURBET_END_NONE;
				p->end.full = 0;
			}
			p->requested_fv = value->intval / 10000 * 10000;
			courbet_end_reset_taper(&p->end);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_END_THRESHOLD:
		if (value->intval < 5 || value->intval > 100)
			ret = -EINVAL;
		else
			p->end_threshold = value->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_BEHAVIOUR:
		if (value->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO &&
		    value->intval != POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE)
			ret = -EINVAL;
		else
			p->behaviour = value->intval;
		break;
	default:
		ret = -EINVAL;
	}
out:
	/* A successful inhibit request takes effect before returning to userspace. */
	if (!ret && (p->behaviour == POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE ||
		     p->requested_fcc == 0)) {
		ret = courbet_fast_stop(p);
		if (!ret) {
			p->pps_attempted = false;
			ret = courbet_write(p, 0x1042, 1, 0);
		}
	}
	mutex_unlock(&p->lock);
	if (!ret)
		mod_delayed_work(system_wq, &p->poll, 0);
	return ret;
}

static int courbet_usb_writable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT;
}

static int courbet_usb_set(struct power_supply *psy, enum power_supply_property prop,
			  const union power_supply_propval *value)
{
	struct courbet_power *p = power_supply_get_drvdata(psy);

	if (prop != POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT || value->intval < 0 ||
	    value->intval > COURBET_MAX_ICL_UA)
		return -EINVAL;
	mutex_lock(&p->lock);
	p->requested_icl = value->intval / 50000 * 50000;
	if (!p->requested_icl)
		courbet_fast_stop(p);
	mutex_unlock(&p->lock);
	mod_delayed_work(system_wq, &p->poll, 0);
	return 0;
}

static void courbet_external_changed(struct power_supply *psy)
{
	struct courbet_power *p = power_supply_get_drvdata(psy);

	if (READ_ONCE(p->ready) && !READ_ONCE(p->suspended))
		mod_delayed_work(system_wq, &p->poll, 0);
}

static const struct power_supply_desc courbet_battery_desc = {
	.name = "battery", .type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = courbet_battery_props,
	.num_properties = ARRAY_SIZE(courbet_battery_props),
	.get_property = courbet_battery_get,
	.set_property = courbet_battery_set,
	.property_is_writeable = courbet_battery_writable,
	.external_power_changed = courbet_external_changed,
	.charge_behaviours = BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_AUTO) |
		BIT(POWER_SUPPLY_CHARGE_BEHAVIOUR_INHIBIT_CHARGE),
};
static const struct power_supply_desc courbet_usb_desc = {
	.name = "usb", .type = POWER_SUPPLY_TYPE_USB,
	.properties = courbet_usb_props,
	.num_properties = ARRAY_SIZE(courbet_usb_props),
	.get_property = courbet_usb_get,
	.set_property = courbet_usb_set,
	.property_is_writeable = courbet_usb_writable,
	.external_power_changed = courbet_external_changed,
};

/* A missing/failed pump must not prevent ordinary SMB5 policy or telemetry. */
static void courbet_find_pump(struct courbet_power *p)
{
	struct power_supply *pump;
	struct device_link *link;

	if (p->pump)
		return;
	pump = power_supply_get_by_name("courbet-charge-pump");
	if (!pump)
		return;
	link = device_link_add(p->dev, pump->dev.parent, DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!link) {
		power_supply_put(pump);
		return;
	}
	p->pump = pump;
}

static void courbet_pump_release(void *data)
{
	struct courbet_power *p = data;

	if (p->pump)
		power_supply_put(p->pump);
}

static void courbet_poll(struct work_struct *work)
{
	struct courbet_power *p = container_of(to_delayed_work(work),
					     struct courbet_power, poll);
	int voltage, current_ua, temp, die, target, ret;
	s64 now = ktime_get_boottime_ns(), correction, max_correction;

	mutex_lock(&p->lock);
	if (!p->ready || p->suspended)
		goto out;
	ret = courbet_sample(p, QG_V_NOW, false, &voltage);
	if (!ret)
		ret = courbet_sample(p, QG_I_NOW, true, &current_ua);
	if (!ret)
		ret = iio_read_channel_processed(p->therm, &temp);
	if (ret >= 0)
		ret = iio_read_channel_processed(p->die, &die);
	if (ret < 0 || voltage < 2500000 || voltage > 4700000 || abs(current_ua) > 10000000) {
		p->policy_error = ret < 0 ? ret : -ERANGE;
		courbet_end_reset_taper(&p->end);
		courbet_fast_stop(p);
		regmap_update_bits(p->regmap, 0x1042, 1, 0);
		goto reschedule;
	}
	temp /= 100;
	p->resistance_uohm = courbet_resistance(p, temp, courbet_soc_percent(&p->soc));
	p->ocv_uv = courbet_ocv_uv(voltage, current_ua, p->resistance_uohm);
	/* Coulomb accounting drives SOC. Relaxed OCV only slowly corrects drift. */
	if (abs(current_ua) < 50000 && p->end.hold_percent < 0) {
		if (!p->rest_ns)
			p->rest_ns = now;
		if (now - p->rest_ns > 300LL * NSEC_PER_SEC) {
			target = power_supply_batinfo_ocv2cap(p->info, p->ocv_uv, temp / 10);
			if (target >= 0) {
				correction = (s64)p->soc.full_uah * COURBET_UAMS_PER_UAH;
				correction = correction * target / 100 - p->soc.charge_uams;
				max_correction = (s64)p->soc.full_uah * 2000 / 25;
				correction = clamp(correction, -max_correction, max_correction);
				courbet_soc_add(&p->soc, correction);
			}
		}
	} else {
		p->rest_ns = 0;
	}
	courbet_find_pump(p);
	courbet_policy(p, temp, die, voltage, current_ua);
reschedule:
	schedule_delayed_work(&p->poll, HZ);
out:
	mutex_unlock(&p->lock);
	power_supply_changed(p->battery);
	power_supply_changed(p->usb);
}

static void courbet_info_release(void *data)
{
	struct courbet_power *p = data;

	struct power_supply_battery_info *info = p->info;

	WRITE_ONCE(p->info, NULL);
	power_supply_put_battery_info(p->battery, info);
}

static void courbet_power_stop(void *data)
{
	struct courbet_power *p = data;

	WRITE_ONCE(p->ready, false);
	cancel_delayed_work_sync(&p->poll);
	courbet_fast_stop(p);
	regmap_update_bits(p->regmap, 0x1042, 1, 0);
	regmap_update_bits(p->regmap, 0x1651, 0xc2, 0);
}

static void courbet_power_shutdown(struct platform_device *pdev)
{
	courbet_power_stop(platform_get_drvdata(pdev));
}

static int courbet_power_suspend(struct device *dev)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	int ret;

	WRITE_ONCE(p->suspended, true);
	cancel_delayed_work_sync(&p->poll);
	mutex_lock(&p->lock);
	courbet_end_reset_taper(&p->end);
	/* CP stops for suspend; SMB5 JEITA/termination and QG FIFO stay active. */
	ret = courbet_fast_stop(p);
	if (!ret)
		ret = courbet_write(p, 0x1651, 0xc2, 0);
	mutex_unlock(&p->lock);
	if (ret) {
		WRITE_ONCE(p->suspended, false);
		schedule_delayed_work(&p->poll, 0);
	}
	return ret;
}

static int courbet_power_resume(struct device *dev)
{
	struct courbet_power *p = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&p->lock);
	ret = courbet_write(p, 0x1651, 0xc2, 2);
	if (!ret)
		ret = regmap_write(p->regmap, 0x1643, BIT(0));
	if (ret)
		regmap_update_bits(p->regmap, 0x1042, 1, 0);
	mutex_unlock(&p->lock);
	WRITE_ONCE(p->suspended, false);
	schedule_delayed_work(&p->poll, 0);
	return ret;
}
static DEFINE_SIMPLE_DEV_PM_OPS(courbet_power_pm, courbet_power_suspend, courbet_power_resume);

static int courbet_power_probe(struct platform_device *pdev)
{
	struct courbet_power *p;
	struct power_supply_config cfg = {};
	unsigned int type, subtype, valid;
	u8 learned[2], magic[4];
	int ret, voltage, current_ua, temp, capacity, irq;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = &pdev->dev;
	mutex_init(&p->lock);
	p->boot_ns = ktime_get_boottime_ns();
	p->soc.full_uah = 4250000;
	p->design_full = 4250000;
	p->design_vmin = 3400000;
	p->design_vmax = 4480000;
	p->requested_fcc = COURBET_MAX_FCC_UA;
	p->requested_fv = COURBET_MAX_FV_UV;
	p->requested_icl = COURBET_MAX_ICL_UA;
	p->fast_allowed = false; /* PPS hardware validation still pending. */
	p->end.hold_percent = -1;
	p->end_threshold = 100;
	p->source = devm_power_supply_get_by_reference(p->dev, "power-supplies");
	if (IS_ERR(p->source))
		return dev_err_probe(p->dev, PTR_ERR(p->source), "No TCPM source supply\n");
	if (!p->source)
		return -EPROBE_DEFER;
	ret = devm_add_action_or_reset(p->dev, courbet_pump_release, p);
	if (ret)
		return ret;
	ret = devm_delayed_work_autocancel(p->dev, &p->poll, courbet_poll);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, p);
	p->regmap = dev_get_regmap(p->dev->parent, NULL);
	if (!p->regmap)
		return -ENODEV;
	ret = regmap_read(p->regmap, QG_TYPE, &type);
	if (ret)
		return ret;
	ret = regmap_read(p->regmap, QG_SUBTYPE, &subtype);
	if (ret)
		return ret;
	if (type != 0x0d || (subtype != 3 && subtype != 4))
		return dev_err_probe(p->dev, -ENODEV, "Unsupported QGauge %x/%x\n",
				     type, subtype);
	p->current_scale = subtype == 3 ? 152588 : 305176;
	p->therm = devm_iio_channel_get(p->dev, "batt-therm");
	if (IS_ERR(p->therm))
		return dev_err_probe(p->dev, PTR_ERR(p->therm), "No battery ADC\n");
	p->vbus = devm_iio_channel_get(p->dev, "usbin-v");
	if (IS_ERR(p->vbus))
		return dev_err_probe(p->dev, PTR_ERR(p->vbus), "No USB ADC\n");
	p->die = devm_iio_channel_get(p->dev, "die-temp");
	if (IS_ERR(p->die))
		return dev_err_probe(p->dev, PTR_ERR(p->die), "No die temperature ADC\n");
	cfg.drv_data = p;
	cfg.fwnode = dev_fwnode(p->dev);
	p->battery = devm_power_supply_register(p->dev, &courbet_battery_desc, &cfg);
	if (IS_ERR(p->battery))
		return PTR_ERR(p->battery);
	ret = power_supply_get_battery_info(p->battery, &p->info);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(p->dev, courbet_info_release, p);
	if (ret)
		return ret;
	if (p->info->charge_full_design_uah <= 0 ||
	    p->info->voltage_max_design_uv <= p->info->voltage_min_design_uv ||
	    !p->info->ocv_table[0])
		return -EINVAL;
	p->usb = devm_power_supply_register(p->dev, &courbet_usb_desc, &cfg);
	if (IS_ERR(p->usb))
		return PTR_ERR(p->usb);
	p->soc.full_uah = p->info->charge_full_design_uah;
	p->design_full = p->info->charge_full_design_uah;
	p->design_vmin = p->info->voltage_min_design_uv;
	p->design_vmax = p->info->voltage_max_design_uv;
	/* Optional existing per-cell calibration is read-only; no Android daemon needed. */
	if (!regmap_read(p->regmap, 0xb646, &valid) && valid == 1 &&
	    !regmap_bulk_read(p->regmap, 0xb680, magic, sizeof(magic)) &&
	    get_unaligned_le32(magic) == 0x12345678 &&
	    !regmap_bulk_read(p->regmap, 0xb668, learned, sizeof(learned))) {
		int full_uah = get_unaligned_le16(learned) * 1000;

		if (full_uah >= p->soc.full_uah * 60 / 100 && full_uah <= p->soc.full_uah)
			p->soc.full_uah = full_uah;
	}
	ret = courbet_charger_init(p);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(p->dev, courbet_power_stop, p);
	if (ret)
		return ret;
	ret = courbet_qg_init(p);
	if (ret)
		return ret;
	/* Seed from a fresh conversion under the driver-owned QG timing. */
	msleep(120);
	ret = courbet_sample(p, QG_V_NOW, false, &voltage);
	if (!ret)
		ret = courbet_sample(p, QG_I_NOW, true, &current_ua);
	if (!ret)
		ret = iio_read_channel_processed(p->therm, &temp);
	if (ret < 0)
		return ret;
	if (voltage < 2500000 || voltage > 4600000 || abs(current_ua) > 10000000)
		return dev_err_probe(p->dev, -EPROBE_DEFER, "QGauge sample not ready\n");
	p->resistance_uohm = courbet_resistance(p, temp / 100, 50);
	p->ocv_uv = courbet_ocv_uv(voltage, current_ua, p->resistance_uohm);
	capacity = power_supply_batinfo_ocv2cap(p->info, p->ocv_uv, temp / 1000);
	if (capacity < 0)
		return capacity;
	p->soc.charge_uams = (s64)p->soc.full_uah * COURBET_UAMS_PER_UAH * capacity / 100;
	irq = platform_get_irq_byname(pdev, "qg-fifo-done");
	if (irq < 0)
		return irq;
	ret = devm_request_threaded_irq(p->dev, irq, NULL, courbet_fifo_irq,
				       IRQF_ONESHOT, "courbet-qg-fifo", p);
	if (ret)
		return ret;
	devm_device_init_wakeup(p->dev);
	ret = devm_pm_set_wake_irq(p->dev, irq);
	if (ret)
		return ret;
	mutex_lock(&p->lock);
	p->ready = true;
	mutex_unlock(&p->lock);
	schedule_delayed_work(&p->poll, 0);
	dev_info(p->dev, "Native SMB5 policy and %u A QGauge FIFO accounting, initial SOC %d%%\n",
		 subtype == 3 ? 5 : 10, capacity);
	return 0;
}

static const struct of_device_id courbet_power_match[] = {
	{ .compatible = "xiaomi,courbet-power" }, {}
};
MODULE_DEVICE_TABLE(of, courbet_power_match);
static struct platform_driver courbet_power_driver = {
	.probe = courbet_power_probe,
	.shutdown = courbet_power_shutdown,
	.driver = { .name = "courbet-power", .of_match_table = courbet_power_match,
		.dev_groups = courbet_power_groups, .pm = pm_sleep_ptr(&courbet_power_pm) },
};
module_platform_driver(courbet_power_driver);
MODULE_DESCRIPTION("Xiaomi courbet native PM6150 charging and QGauge coulomb accounting");
MODULE_LICENSE("GPL");

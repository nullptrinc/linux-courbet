// SPDX-License-Identifier: GPL-2.0-only
/* Read-only PM6150 battery/USB telemetry for Xiaomi courbet.
 * QGauge conversion factors: Qualcomm qg-util.c (courbet-r-oss).
 * This driver deliberately leaves charger policy and QG calibration intact.
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
	struct iio_channel *therm, *vbus;
	struct power_supply *battery, *usb;
	struct power_supply_battery_info *info;
	struct delayed_work poll;
	int current_scale;
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
	else if ((state & 7) == 5)
		*value = POWER_SUPPLY_STATUS_FULL;
	else if ((state & 7) >= 1 && (state & 7) <= 4 && current_ua > 10000)
		*value = POWER_SUPPLY_STATUS_CHARGING;
	else
		*value = POWER_SUPPLY_STATUS_NOT_CHARGING;
	return 0;
}

static enum power_supply_property courbet_battery_props[] = {
	POWER_SUPPLY_PROP_PRESENT, POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_HEALTH, POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_VOLTAGE_AVG,
	POWER_SUPPLY_PROP_CURRENT_NOW, POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_TEMP, POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN, POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static int courbet_battery_get(struct power_supply *psy,
			      enum power_supply_property prop,
			      union power_supply_propval *value)
{
	struct courbet_power *p = power_supply_get_drvdata(psy);
	unsigned int status;
	int ret, voltage, temp;

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
		ret = courbet_sample(p, QG_V_AVG, false, &voltage);
		if (ret)
			return ret;
		ret = iio_read_channel_processed(p->therm, &temp);
		if (ret < 0)
			return ret;
		/* Estimate from terminal voltage, not a learned coulomb count. */
		ret = power_supply_batinfo_ocv2cap(p->info, voltage, temp / 1000);
		if (ret < 0)
			return ret;
		value->intval = clamp(ret, 0, 100);
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		value->intval = p->info->voltage_min_design_uv;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		value->intval = p->info->voltage_max_design_uv;
		return 0;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		value->intval = p->info->charge_full_design_uah;
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
			value->intval = raw * 50000;
		return ret;
	default:
		return -EINVAL;
	}
}

static const struct power_supply_desc courbet_battery_desc = {
	.name = "battery", .type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = courbet_battery_props,
	.num_properties = ARRAY_SIZE(courbet_battery_props),
	.get_property = courbet_battery_get,
};
static const struct power_supply_desc courbet_usb_desc = {
	.name = "usb", .type = POWER_SUPPLY_TYPE_USB,
	.properties = courbet_usb_props,
	.num_properties = ARRAY_SIZE(courbet_usb_props),
	.get_property = courbet_usb_get,
};

static void courbet_poll(struct work_struct *work)
{
	struct courbet_power *p = container_of(to_delayed_work(work),
					     struct courbet_power, poll);

	power_supply_changed(p->battery);
	power_supply_changed(p->usb);
	schedule_delayed_work(&p->poll, 10 * HZ);
}

static void courbet_info_release(void *data)
{
	struct courbet_power *p = data;

	power_supply_put_battery_info(p->battery, p->info);
}

static int courbet_power_probe(struct platform_device *pdev)
{
	struct courbet_power *p;
	struct power_supply_config cfg = {};
	unsigned int type, subtype;
	int ret;

	p = devm_kzalloc(&pdev->dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = &pdev->dev;
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
	ret = devm_delayed_work_autocancel(p->dev, &p->poll, courbet_poll);
	if (ret)
		return ret;
	schedule_delayed_work(&p->poll, 0);
	dev_info(p->dev, "Read-only PM6150 telemetry, QGauge %u A\n",
		 subtype == 3 ? 5 : 10);
	return 0;
}

static const struct of_device_id courbet_power_match[] = {
	{ .compatible = "xiaomi,courbet-power" }, {}
};
MODULE_DEVICE_TABLE(of, courbet_power_match);
static struct platform_driver courbet_power_driver = {
	.probe = courbet_power_probe,
	.driver = { .name = "courbet-power", .of_match_table = courbet_power_match },
};
module_platform_driver(courbet_power_driver);
MODULE_DESCRIPTION("Xiaomi courbet PM6150 read-only battery and USB telemetry");
MODULE_LICENSE("GPL");

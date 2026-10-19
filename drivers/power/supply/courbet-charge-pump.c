// SPDX-License-Identifier: GPL-2.0-only
/* Courbet BQ25968 / SC8551 charge pump. Register layout: Xiaomi courbet-r-oss
 * ti/bq2597x_charger.c and TI BQ25968 datasheet SLUSE31A.
 * The board's external pump NTC inputs are not wired. Battery temperature is
 * independently supervised through PM6150 ADC5, in addition to HW OVP/OCP/die
 * protections and a 5 s watchdog. A stale policy heartbeat always disables CP.
 */
#include <linux/delay.h>
#include <linux/devm-helpers.h>
#include <linux/i2c.h>
#include <linux/iio/consumer.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/regmap.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>
#include "courbet-fast-model.h"

struct courbet_cp {
	struct device *dev;
	struct regmap *map;
	struct iio_channel *therm;
	struct power_supply *source, *psy;
	struct mutex lock;
	struct delayed_work guard;
	s64 heartbeat_ns, stop_failed_ns;
	int vbus, vbat, ibus, ibat, die, fault, error, bus_status;
	bool sc8551, enabled, ready, suspended;
};

static int cp_write(struct courbet_cp *p, unsigned int reg,
		    unsigned int mask, unsigned int value)
{
	unsigned int actual;
	int ret = regmap_update_bits(p->map, reg, mask, value);

	if (!ret)
		ret = regmap_read(p->map, reg, &actual);
	if (!ret && (actual & mask) != (value & mask))
		ret = -EIO;
	return ret;
}

static int cp_disable(struct courbet_cp *p)
{
	int ret = cp_write(p, 0x0c, BIT(7), 0);

	p->enabled = false;
	p->stop_failed_ns = ret ? ktime_get_boottime_ns() : 0;
	return ret;
}

static int cp_adc(struct courbet_cp *p, int reg, int scale, int *result)
{
	u8 raw[2];
	int ret = regmap_bulk_read(p->map, reg, raw, sizeof(raw));

	if (!ret)
		*result = (s16)get_unaligned_be16(raw) * scale;
	return ret;
}

static int cp_sample(struct courbet_cp *p)
{
	unsigned int fault, status;
	int raw_bus, ret;

	/* SC8551 IBUS is 1.5625 mA/LSB; all other scaling is exact integer. */
	ret = cp_adc(p, 0x16, 1, &raw_bus);
	if (!ret)
		p->ibus = p->sc8551 ? raw_bus * 15625 / 10 : raw_bus * 1000;
	if (!ret)
		ret = cp_adc(p, 0x18, p->sc8551 ? 3750 : 1000, &p->vbus);
	if (!ret)
		ret = cp_adc(p, 0x1e, p->sc8551 ? 1250 : 1000, &p->vbat);
	if (!ret)
		ret = cp_adc(p, 0x20, p->sc8551 ? 3125 : 1000, &p->ibat);
	if (!ret)
		ret = cp_adc(p, 0x26, p->sc8551 ? 10 : 5, &p->die);
	if (!ret)
		ret = regmap_read(p->map, 0x10, &fault);
	if (!ret)
		ret = regmap_read(p->map, 0x0a, &status);
	if (ret)
		return ret;
	p->fault = fault;
	p->bus_status = status;
	return 0;
}

/* All callers hold lock, including diagnostic reads. Watchdog reads therefore
 * cannot keep CP charging behind a wedged supervisor/expired policy heartbeat.
 */
static int cp_validate(struct courbet_cp *p)
{
	union power_supply_propval online, type, voltage;
	unsigned int adc, watchdog;
	int temp, ret;

	ret = regmap_read(p->map, 0x14, &adc);
	if (!ret)
		ret = regmap_read(p->map, 0x0b, &watchdog);
	if (ret)
		return ret;
	if (!(adc & BIT(7)) || (watchdog & 7) != 2)
		return -EIO;
	ret = iio_read_channel_processed(p->therm, &temp);
	if (ret < 0)
		return ret;
	if (temp < 15000 || temp >= 40000)
		return -ERANGE;
	ret = cp_sample(p);
	if (ret)
		return ret;
	if (p->fault || (p->bus_status & BIT(6)) || p->die >= 750 ||
	    p->die < 100 || p->ibus < -100000 || p->ibus > 2000000 ||
	    !(p->enabled ? courbet_pump_running_window(p->vbus, p->vbat) :
	      courbet_pump_window(p->vbus, p->vbat)))
		return -ERANGE;
	ret = power_supply_get_property(p->source, POWER_SUPPLY_PROP_ONLINE, &online);
	if (!ret)
		ret = power_supply_get_property(p->source, POWER_SUPPLY_PROP_USB_TYPE, &type);
	if (!ret)
		ret = power_supply_get_property(p->source, POWER_SUPPLY_PROP_VOLTAGE_NOW, &voltage);
	if (ret)
		return ret;
	/* A fixed 9 V PDO is never an acceptable source for the direct-charge path. */
	if (online.intval != 2 || type.intval != POWER_SUPPLY_USB_TYPE_PD_PPS ||
	    abs(voltage.intval - p->vbus) > 400000)
		return -EOPNOTSUPP;
	return 0;
}

static void cp_guard(struct work_struct *work)
{
	struct courbet_cp *p = container_of(to_delayed_work(work), struct courbet_cp, guard);
	int ret;

	mutex_lock(&p->lock);
	if (!p->ready || p->suspended)
		goto out;
	if (p->stop_failed_ns) {
		/* No I2C traffic during this interval: let the 5 s HW watchdog expire. */
		if (ktime_get_boottime_ns() - p->stop_failed_ns > 6LL * NSEC_PER_SEC)
			cp_disable(p);
	} else if (p->enabled) {
		ret = ktime_get_boottime_ns() - p->heartbeat_ns > 3LL * NSEC_PER_SEC ?
			-ETIMEDOUT : cp_validate(p);
		if (!ret && !(p->bus_status & BIT(2)))
			ret = -EIO;
		if (ret) {
			p->error = ret;
			cp_disable(p);
			dev_warn_ratelimited(p->dev, "Charge pump stopped by supervisor: %d\n", ret);
		}
	} else {
		cp_sample(p);
	}
	schedule_delayed_work(&p->guard, msecs_to_jiffies(p->enabled ? 250 : 2000));
out:
	mutex_unlock(&p->lock);
}

static enum power_supply_property cp_props[] = {
	POWER_SUPPLY_PROP_MODEL_NAME, POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW, POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_TEMP,
};

static int cp_get(struct power_supply *psy, enum power_supply_property prop,
		  union power_supply_propval *v)
{
	struct courbet_cp *p = power_supply_get_drvdata(psy);
	int ret = 0;

	mutex_lock(&p->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_MODEL_NAME:
		v->strval = p->sc8551 ? "SC8551" : "BQ25968";
		break;
	case POWER_SUPPLY_PROP_PRESENT: v->intval = p->ready; break;
	case POWER_SUPPLY_PROP_ONLINE: v->intval = p->enabled; break;
	case POWER_SUPPLY_PROP_HEALTH:
		v->intval = p->fault || p->error ? POWER_SUPPLY_HEALTH_UNSPEC_FAILURE :
			POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW: v->intval = p->vbus; break;
	case POWER_SUPPLY_PROP_CURRENT_NOW: v->intval = p->ibus; break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX: v->intval = 4450000; break;
	case POWER_SUPPLY_PROP_TEMP: v->intval = p->die; break;
	default: ret = -EINVAL;
	}
	mutex_unlock(&p->lock);
	return ret;
}

static int cp_set(struct power_supply *psy, enum power_supply_property prop,
		  const union power_supply_propval *v)
{
	struct courbet_cp *p = power_supply_get_drvdata(psy);
	int ret;

	if (prop != POWER_SUPPLY_PROP_ONLINE || v->intval < 0 || v->intval > 1)
		return -EINVAL;
	mutex_lock(&p->lock);
	if (p->stop_failed_ns) {
		ret = -EIO;
	} else if (!v->intval) {
		ret = cp_disable(p);
	} else if (!p->ready || p->suspended) {
		ret = -EAGAIN;
	} else {
		ret = cp_validate(p);
		if (!ret)
			ret = cp_write(p, 0x0c, BIT(7), BIT(7));
		if (!ret) {
			p->enabled = true;
			p->error = 0;
			p->heartbeat_ns = ktime_get_boottime_ns();
		} else {
			p->error = ret;
			cp_disable(p);
		}
	}
	mutex_unlock(&p->lock);
	if (!ret)
		mod_delayed_work(system_wq, &p->guard, msecs_to_jiffies(250));
	return ret;
}

static int cp_writable(struct power_supply *psy, enum power_supply_property prop)
{
	return prop == POWER_SUPPLY_PROP_ONLINE;
}

static const struct power_supply_desc cp_desc = {
	.name = "courbet-charge-pump", .type = POWER_SUPPLY_TYPE_MAINS,
	.properties = cp_props, .num_properties = ARRAY_SIZE(cp_props),
	.get_property = cp_get, .set_property = cp_set, .property_is_writeable = cp_writable,
};

static ssize_t diagnostics_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct courbet_cp *p = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&p->lock);
	n = sysfs_emit(buf, "model=%s enabled=%d vbus_uv=%d vbat_uv=%d ibus_ua=%d "
			"ibat_ua=%d die_deciC=%d faults=0x%x bus_status=0x%x error=%d\n",
			p->sc8551 ? "SC8551" : "BQ25968", p->enabled, p->vbus, p->vbat,
			p->ibus, p->ibat, p->die, p->fault, p->bus_status, p->error);
	mutex_unlock(&p->lock);
	return n;
}
static DEVICE_ATTR_RO(diagnostics);
static struct attribute *cp_attrs[] = { &dev_attr_diagnostics.attr, NULL };
ATTRIBUTE_GROUPS(cp);

static void cp_stop(void *data)
{
	struct courbet_cp *p = data;

	WRITE_ONCE(p->ready, false);
	cancel_delayed_work_sync(&p->guard);
	cp_disable(p);
}

static int cp_suspend(struct device *dev)
{
	struct courbet_cp *p = dev_get_drvdata(dev);
	int ret;

	WRITE_ONCE(p->suspended, true);
	cancel_delayed_work_sync(&p->guard);
	mutex_lock(&p->lock);
	ret = cp_disable(p);
	mutex_unlock(&p->lock);
	return ret;
}

static int cp_resume(struct device *dev)
{
	struct courbet_cp *p = dev_get_drvdata(dev);

	WRITE_ONCE(p->suspended, false);
	mod_delayed_work(system_wq, &p->guard, 0);
	return 0;
}
static DEFINE_SIMPLE_DEV_PM_OPS(cp_pm, cp_suspend, cp_resume);

static int cp_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct power_supply_config cfg = {};
	static const struct regmap_config map = { .reg_bits = 8, .val_bits = 8,
		.max_register = 0x36, .cache_type = REGCACHE_NONE };
	static const struct { u8 reg, mask, value; } init[] = {
		{ 0x0c, 0x87, 0x06 }, /* off; unused external NTCs off, die guard on */
		{ 0x0b, 0x77, 0x42 }, /* 500 kHz, watchdog enabled, 5 seconds */
		{ 0x01, 0xbf, 0x24 }, /* 4.4 V alarm, enabled */
		{ 0x02, 0xff, 0x14 }, /* cell OCP 4 A, enabled */
		{ 0x03, 0xff, 0x0a }, /* cell alarm 3 A */
		{ 0x04, 0xff, 0x80 }, /* unused cell undercurrent alarm */
		{ 0x05, 0x07, 0x00 }, /* VAC OVP 11 V */
		{ 0x06, 0xff, 0x5b }, /* bus OVP 10.5 V, no pulldown */
		{ 0x07, 0xff, 0x50 }, /* bus alarm 10 V */
		{ 0x08, 0x8f, 0x04 }, /* integrated bus OCP 2 A, enabled */
		{ 0x09, 0xff, 0x22 }, /* bus alarm 1.7 A */
		{ 0x0f, 0xff, 0xff }, /* polled alarm status; no interrupt storm */
		{ 0x12, 0xff, 0xff }, /* polled fault status */
		{ 0x14, 0xff, 0xa8 }, /* continuous averaged ADC; IBUS on */
		{ 0x15, 0xff, 0x06 }, /* all ADC channels except unused external NTCs */
		{ 0x2a, 0xff, 0x5a }, /* die alarm 75 C */
		{ 0x2b, 0xff, 0x80 }, /* 100 ms soft start, 2 mOhm, no pulldown */
	};
	struct courbet_cp *p;
	unsigned int id, mode;
	int i, ret;

	p = devm_kzalloc(dev, sizeof(*p), GFP_KERNEL);
	if (!p)
		return -ENOMEM;
	p->dev = dev;
	mutex_init(&p->lock);
	i2c_set_clientdata(client, p);
	p->map = devm_regmap_init_i2c(client, &map);
	if (IS_ERR(p->map))
		return PTR_ERR(p->map);
	ret = regmap_read(p->map, 0x13, &id);
	if (ret)
		return ret;
	if (id != 0x00 && id != 0x16)
		return dev_err_probe(dev, -ENODEV, "Unknown pump ID %02x\n", id);
	p->sc8551 = id == 0;
	ret = regmap_read(p->map, 0x0c, &mode);
	if (ret)
		return ret;
	if (mode & 0x60)
		return dev_err_probe(dev, -EINVAL, "Pump is not standalone\n");
	/* Stop an inherited pump before waiting for any supplier. */
	ret = cp_disable(p);
	if (ret)
		return ret;
	p->source = devm_power_supply_get_by_reference(dev, "power-supplies");
	if (IS_ERR(p->source))
		return PTR_ERR(p->source);
	if (!p->source)
		return -EPROBE_DEFER;
	p->therm = devm_iio_channel_get(dev, "batt-therm");
	if (IS_ERR(p->therm))
		return PTR_ERR(p->therm);
	for (i = 0; i < ARRAY_SIZE(init); i++) {
		if (p->sc8551 && init[i].reg == 0x14)
			ret = cp_write(p, 0x14, 0xc1, 0x80); /* SC has no BQ averaging/bits */
		else if (p->sc8551 && init[i].reg == 0x15)
			ret = cp_write(p, 0x15, 0xf9, 0x00); /* SC has seven ADC channels */
		else
			ret = cp_write(p, init[i].reg, init[i].mask, init[i].value);
		if (ret)
			return dev_err_probe(dev, ret, "Pump init register %02x failed\n", init[i].reg);
	}
	ret = cp_write(p, 0x00, 0xbf, p->sc8551 ? 38 : 39); /* 4.45 V hard OVP */
	if (!ret && p->sc8551)
		ret = cp_write(p, 0x31, 1, 0); /* 2:1 only; no bypass */
	if (!ret && p->sc8551)
		ret = cp_write(p, 0x34, 1, 1); /* OEM ADC optimization */
	if (ret)
		return ret;
	ret = devm_delayed_work_autocancel(dev, &p->guard, cp_guard);
	if (ret)
		return ret;
	cfg.drv_data = p;
	cfg.fwnode = dev_fwnode(dev);
	p->psy = devm_power_supply_register(dev, &cp_desc, &cfg);
	if (IS_ERR(p->psy))
		return PTR_ERR(p->psy);
	ret = devm_add_action_or_reset(dev, cp_stop, p);
	if (ret)
		return ret;
	p->ready = true;
	msleep(100);
	cp_sample(p);
	schedule_delayed_work(&p->guard, 0);
	dev_info(dev, "%s charge pump ready; disabled until supervised PPS\n",
		 p->sc8551 ? "SC8551" : "BQ25968");
	return 0;
}

static void cp_shutdown(struct i2c_client *client)
{
	cp_stop(i2c_get_clientdata(client));
}
static const struct of_device_id cp_of_match[] = {
	{ .compatible = "xiaomi,courbet-charge-pump" }, {}
};
MODULE_DEVICE_TABLE(of, cp_of_match);
static struct i2c_driver cp_driver = {
	.driver = { .name = "courbet-charge-pump", .of_match_table = cp_of_match,
		.dev_groups = cp_groups, .pm = pm_sleep_ptr(&cp_pm) },
	.probe = cp_probe, .shutdown = cp_shutdown,
};
module_i2c_driver(cp_driver);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Courbet BQ25968/SC8551 supervised PPS charge pump");

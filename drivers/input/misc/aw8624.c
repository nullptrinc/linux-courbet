// SPDX-License-Identifier: GPL-2.0-only
/*
 * AW8624 continuous-mode haptics.
 * Register sequence based on the Awinic AW8624 reference driver.
 * Copyright (c) 2024 Shanghai Awinic Technology Co., Ltd.
 * Copyright (c) 2026 moe-takasaki
 *
 * Continuous mode needs no vendor waveform firmware. Motor characteristics
 * must come from the board; this driver does not perform OTP programming.
 */
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/workqueue.h>

#define AW8624_ID		0x00
#define AW8624_SYSINT		0x02
#define AW8624_SYSCTRL		0x04
#define AW8624_STANDBY		BIT(0)
#define AW8624_PLAY_MODE	GENMASK(3, 2)
#define AW8624_CONT_MODE	BIT(3)
#define AW8624_GO		0x05
#define AW8624_DATCTRL		0x2b
#define AW8624_PWMDBG		0x2e
#define AW8624_BRAKE0		0x32
#define AW8624_SW_BRAKE		0x39
#define AW8624_GAIN		0x3b
#define AW8624_STATE		0x47
#define AW8624_CONT_CTRL	0x48
#define AW8624_F_PRE_H		0x49
#define AW8624_TD_H		0x4b
#define AW8624_TSET		0x4d
#define AW8624_BRA_END		0x4f
#define AW8624_R_SPARE		0x5d
#define AW8624_ADCTEST		0x66
#define AW8624_ZC_THR_H		0x72
#define AW8624_BEMF_VTHH_H	0x74
#define AW8624_BEMF_NUM		0x78
#define AW8624_TIME_NZC		0x7a
#define AW8624_DRV_LVL		0x7b
#define AW8624_DRV_LVL_OV	0x7c

struct aw8624 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	struct work_struct work;
	/* Serialize playback, reset and system suspend. */
	struct mutex lock;
	u32 frequency;
	u32 drive_level;
	u32 overdrive_level;
	u32 delay;
	u32 zero_cross;
	u16 magnitude;
	bool suspended;
	bool running;
	bool initialized;
};

static void aw8624_reset(struct aw8624 *aw)
{
	gpiod_set_value_cansleep(aw->reset, 1);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(aw->reset, 0);
	usleep_range(5000, 6000);
	aw->running = false;
	aw->initialized = false;
}

static int aw8624_init(struct aw8624 *aw)
{
	/* Courbet's factory braking/BEMF profile; only this board is matched. */
	const struct reg_sequence init[] = {
		{ AW8624_SW_BRAKE, 0x2c },
		{ AW8624_BRA_END, 0x00 },
		{ AW8624_TSET, 0x11 },
		{ AW8624_BRAKE0, 0x5a },
		{ AW8624_BRAKE0 + 1, 0xaa },
		{ AW8624_BRAKE0 + 2, 0x94 },
		{ AW8624_BRAKE0 + 3, 0x95 },
		{ AW8624_BEMF_VTHH_H, 0x00 },
		{ AW8624_BEMF_VTHH_H + 1, 0x08 },
		{ AW8624_BEMF_VTHH_H + 2, 0x03 },
		{ AW8624_BEMF_VTHH_H + 3, 0xf8 },
		{ AW8624_CONT_CTRL, 0x88 },
		{ AW8624_TIME_NZC, 0x1f },
		{ AW8624_GAIN, 0x80 },
	};
	unsigned int id, frequency;
	u8 data[2];
	int ret;

	aw8624_reset(aw);
	ret = regmap_read(aw->regmap, AW8624_ID, &id);
	if (ret)
		return ret;
	if (id != 0x24)
		return -ENODEV;

	ret = regmap_multi_reg_write(aw->regmap, init, ARRAY_SIZE(init));
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_PWMDBG, 0x60, 0x40);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_R_SPARE, 0x80, 0x80);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_ADCTEST, 0x40, 0x40);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_DATCTRL, 0xe0, 0xe0);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_BEMF_NUM, 0x07, 0x03);
	if (ret)
		return ret;

	/* Awinic's coefficient is 260 with frequency expressed in 0.1 Hz. */
	frequency = 100000000U / (aw->frequency * 260U);
	data[0] = frequency >> 8;
	data[1] = frequency;
	ret = regmap_bulk_write(aw->regmap, AW8624_F_PRE_H, data, 2);
	if (ret)
		return ret;
	data[0] = aw->delay >> 8;
	data[1] = aw->delay;
	ret = regmap_bulk_write(aw->regmap, AW8624_TD_H, data, 2);
	if (ret)
		return ret;
	data[0] = aw->zero_cross >> 8;
	data[1] = aw->zero_cross;
	ret = regmap_bulk_write(aw->regmap, AW8624_ZC_THR_H, data, 2);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_SYSCTRL,
				 AW8624_STANDBY, AW8624_STANDBY);
	if (!ret)
		aw->initialized = true;
	return ret;
}

static int aw8624_stop(struct aw8624 *aw)
{
	unsigned int state;
	int ret;

	ret = regmap_write(aw->regmap, AW8624_GO, 0);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(aw->regmap, AW8624_STATE, state,
				       !(state & 0xf), 2000, 200000);
	if (ret)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_SYSCTRL,
				 AW8624_STANDBY, AW8624_STANDBY);
	if (!ret)
		aw->running = false;
	return ret;
}

static int aw8624_play(struct aw8624 *aw, u16 magnitude)
{
	unsigned int ignored;
	u8 levels[] = {
		DIV_ROUND_CLOSEST(aw->drive_level * magnitude, 0xffff),
		DIV_ROUND_CLOSEST(aw->overdrive_level * magnitude, 0xffff),
	};
	int ret;

	if (!aw->initialized) {
		ret = aw8624_init(aw);
		if (ret)
			return ret;
	}
	ret = regmap_bulk_write(aw->regmap, AW8624_DRV_LVL, levels, 2);
	if (ret || aw->running)
		return ret;
	ret = regmap_update_bits(aw->regmap, AW8624_SYSCTRL,
				 AW8624_PLAY_MODE | AW8624_STANDBY,
				 AW8624_CONT_MODE);
	if (ret)
		return ret;
	usleep_range(2000, 2500);
	ret = regmap_read(aw->regmap, AW8624_SYSINT, &ignored);
	if (ret)
		return ret;
	ret = regmap_write(aw->regmap, AW8624_GO, 1);
	if (!ret)
		aw->running = true;
	return ret;
}

static void aw8624_work(struct work_struct *work)
{
	struct aw8624 *aw = container_of(work, struct aw8624, work);
	u16 magnitude;
	int ret = 0;

	mutex_lock(&aw->lock);
	if (!aw->suspended) {
		magnitude = READ_ONCE(aw->magnitude);
		if (magnitude)
			ret = aw8624_play(aw, magnitude);
		else if (aw->initialized)
			ret = aw8624_stop(aw);
		if (ret) {
			/* Reset is also a stop path if the I2C link has failed. */
			gpiod_set_value_cansleep(aw->reset, 1);
			aw->initialized = false;
			aw->running = false;
			dev_err_ratelimited(aw->dev, "Playback failed: %d\n", ret);
		}
	}
	mutex_unlock(&aw->lock);
}

static int aw8624_effect(struct input_dev *input, void *data,
			 struct ff_effect *effect)
{
	struct aw8624 *aw = input_get_drvdata(input);
	u16 magnitude = max(effect->u.rumble.strong_magnitude,
			    effect->u.rumble.weak_magnitude);

	WRITE_ONCE(aw->magnitude, magnitude);
	schedule_work(&aw->work);
	return 0;
}

static void aw8624_close(struct input_dev *input)
{
	struct aw8624 *aw = input_get_drvdata(input);

	WRITE_ONCE(aw->magnitude, 0);
	cancel_work_sync(&aw->work);
	mutex_lock(&aw->lock);
	gpiod_set_value_cansleep(aw->reset, 1);
	aw->initialized = false;
	aw->running = false;
	mutex_unlock(&aw->lock);
}

static void aw8624_cleanup(void *data)
{
	struct aw8624 *aw = data;

	cancel_work_sync(&aw->work);
	gpiod_set_value_cansleep(aw->reset, 1);
}

static int aw8624_property(struct device *dev, const char *name, u32 *value,
			   u32 min, u32 max)
{
	int ret = device_property_read_u32(dev, name, value);

	if (ret)
		return dev_err_probe(dev, ret, "Missing %s\n", name);
	if (*value < min || *value > max)
		return dev_err_probe(dev, -EINVAL, "Invalid %s\n", name);
	return 0;
}

static const struct regmap_config aw8624_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x7f,
	.use_single_read = true,
	.use_single_write = true,
};

static int aw8624_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct input_dev *input;
	struct aw8624 *aw;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	aw->dev = dev;
	mutex_init(&aw->lock);
	INIT_WORK(&aw->work, aw8624_work);
	i2c_set_clientdata(client, aw);
	ret = aw8624_property(dev, "awinic,resonant-frequency-hz", &aw->frequency,
			      100, 300);
	if (ret)
		return ret;
	ret = aw8624_property(dev, "awinic,drive-level", &aw->drive_level, 1, 255);
	if (ret)
		return ret;
	ret = aw8624_property(dev, "awinic,overdrive-level", &aw->overdrive_level,
			      1, 255);
	if (ret)
		return ret;
	ret = aw8624_property(dev, "awinic,continuous-delay", &aw->delay, 0, 65535);
	if (ret)
		return ret;
	ret = aw8624_property(dev, "awinic,zero-cross-threshold", &aw->zero_cross,
			      0, 65535);
	if (ret)
		return ret;
	aw->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(dev, PTR_ERR(aw->reset), "Reset GPIO unavailable\n");
	aw->regmap = devm_regmap_init_i2c(client, &aw8624_regmap);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(dev, PTR_ERR(aw->regmap), "Regmap unavailable\n");
	ret = devm_add_action_or_reset(dev, aw8624_cleanup, aw);
	if (ret)
		return ret;
	ret = aw8624_init(aw);
	if (ret)
		return dev_err_probe(dev, ret, "AW8624 initialization failed\n");
	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;
	input->name = "AW8624 Haptics";
	input->id.bustype = BUS_I2C;
	input->close = aw8624_close;
	input_set_drvdata(input, aw);
	input_set_capability(input, EV_FF, FF_RUMBLE);
	ret = input_ff_create_memless(input, NULL, aw8624_effect);
	if (ret)
		return ret;
	ret = input_register_device(input);
	if (ret)
		return ret;
	dev_info(dev, "AW8624 continuous haptics, %u Hz\n", aw->frequency);
	return 0;
}

static int aw8624_suspend(struct device *dev)
{
	struct aw8624 *aw = dev_get_drvdata(dev);

	mutex_lock(&aw->lock);
	aw->suspended = true;
	WRITE_ONCE(aw->magnitude, 0);
	gpiod_set_value_cansleep(aw->reset, 1);
	aw->initialized = false;
	aw->running = false;
	mutex_unlock(&aw->lock);
	cancel_work_sync(&aw->work);
	return 0;
}

static int aw8624_resume(struct device *dev)
{
	struct aw8624 *aw = dev_get_drvdata(dev);

	mutex_lock(&aw->lock);
	aw->suspended = false;
	mutex_unlock(&aw->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(aw8624_pm, aw8624_suspend, aw8624_resume);

static void aw8624_shutdown(struct i2c_client *client)
{
	aw8624_suspend(&client->dev);
}

static const struct of_device_id aw8624_of_match[] = {
	{ .compatible = "xiaomi,courbet-aw8624" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw8624_of_match);

static struct i2c_driver aw8624_driver = {
	.probe = aw8624_probe,
	.shutdown = aw8624_shutdown,
	.driver = {
		.name = "aw8624",
		.of_match_table = aw8624_of_match,
		.pm = pm_sleep_ptr(&aw8624_pm),
	},
};
module_i2c_driver(aw8624_driver);

MODULE_AUTHOR("moe-takasaki");
MODULE_DESCRIPTION("AW8624 continuous-mode haptics for Xiaomi courbet");
MODULE_LICENSE("GPL");

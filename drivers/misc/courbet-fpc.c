// SPDX-License-Identifier: GPL-2.0-only
/* Xiaomi courbet FPC electrical control and interrupt transport.
 *
 * Fingerprint images, templates and decisions remain in the trusted app.
 * This driver only powers the sensor and reports its interrupt sequence.
 */
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

struct courbet_fpc {
	struct miscdevice misc;
	struct gpio_desc *power;
	struct gpio_desc *reset;
	struct gpio_desc *irq_gpio;
	wait_queue_head_t waitq;
	atomic64_t irq_seq;
	struct mutex lock;
	bool in_use;
};

static irqreturn_t courbet_fpc_irq(int irq, void *cookie)
{
	struct courbet_fpc *fpc = cookie;

	atomic64_inc(&fpc->irq_seq);
	wake_up_interruptible(&fpc->waitq);
	return IRQ_HANDLED;
}

static int courbet_fpc_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct courbet_fpc *fpc = container_of(misc, struct courbet_fpc, misc);
	int ret = 0;

	mutex_lock(&fpc->lock);
	if (fpc->in_use) {
		ret = -EBUSY;
		goto out;
	}

	fpc->in_use = true;
	/* Start from reset asserted and power off on every open. */
	gpiod_set_value_cansleep(fpc->reset, 0);
	gpiod_set_value_cansleep(fpc->power, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(fpc->power, 1);
	usleep_range(10000, 11000);
	gpiod_set_value_cansleep(fpc->reset, 1);
	usleep_range(30000, 31000);
	file->private_data = fpc;
	file->f_pos = atomic64_read(&fpc->irq_seq);
	out:
	mutex_unlock(&fpc->lock);
	return ret;
}

static int courbet_fpc_release(struct inode *inode, struct file *file)
{
	struct courbet_fpc *fpc = file->private_data;

	mutex_lock(&fpc->lock);
	gpiod_set_value_cansleep(fpc->reset, 0);
	usleep_range(1000, 2000);
	gpiod_set_value_cansleep(fpc->power, 0);
	fpc->in_use = false;
	mutex_unlock(&fpc->lock);
	return 0;
}

static ssize_t courbet_fpc_read(struct file *file, char __user *buffer,
				size_t count, loff_t *offset)
{
	struct courbet_fpc *fpc = file->private_data;
	u64 seq;
	int ret;

	if (count < sizeof(seq))
		return -EINVAL;
	if (file->f_pos == atomic64_read(&fpc->irq_seq)) {
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(fpc->waitq,
			file->f_pos != atomic64_read(&fpc->irq_seq));
		if (ret)
			return ret;
	}
	seq = atomic64_read(&fpc->irq_seq);
	if (copy_to_user(buffer, &seq, sizeof(seq)))
		return -EFAULT;
	/* VFS writes this offset back to file->f_pos after ->read returns. */
	*offset = seq;
	return sizeof(seq);
}

static __poll_t courbet_fpc_poll(struct file *file, poll_table *wait)
{
	struct courbet_fpc *fpc = file->private_data;

	poll_wait(file, &fpc->waitq, wait);
	if (file->f_pos != atomic64_read(&fpc->irq_seq))
		return EPOLLIN | EPOLLRDNORM;
	return 0;
}

static const struct file_operations courbet_fpc_fops = {
	.owner = THIS_MODULE,
	.open = courbet_fpc_open,
	.release = courbet_fpc_release,
	.read = courbet_fpc_read,
	.poll = courbet_fpc_poll,
	.llseek = noop_llseek,
};

static int courbet_fpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct courbet_fpc *fpc;
	int irq, ret;

	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc)
		return -ENOMEM;
	fpc->power = devm_gpiod_get(dev, "power", GPIOD_OUT_LOW);
	if (IS_ERR(fpc->power))
		return dev_err_probe(dev, PTR_ERR(fpc->power), "power GPIO\n");
	fpc->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(fpc->reset))
		return dev_err_probe(dev, PTR_ERR(fpc->reset), "reset GPIO\n");
	fpc->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(fpc->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(fpc->irq_gpio), "IRQ GPIO\n");
	irq = gpiod_to_irq(fpc->irq_gpio);
	if (irq < 0)
		return dev_err_probe(dev, irq, "map IRQ GPIO\n");

	init_waitqueue_head(&fpc->waitq);
	mutex_init(&fpc->lock);
	atomic64_set(&fpc->irq_seq, 0);
	ret = devm_request_irq(dev, irq, courbet_fpc_irq,
			       IRQF_TRIGGER_RISING, dev_name(dev), fpc);
	if (ret)
		return dev_err_probe(dev, ret, "request IRQ\n");

	fpc->misc.minor = MISC_DYNAMIC_MINOR;
	fpc->misc.name = "courbet-fpc";
	fpc->misc.fops = &courbet_fpc_fops;
	fpc->misc.parent = dev;
	fpc->misc.mode = 0600;
	ret = misc_register(&fpc->misc);
	if (ret)
		return dev_err_probe(dev, ret, "register character device\n");
	platform_set_drvdata(pdev, fpc);
	return 0;
}

static void courbet_fpc_remove(struct platform_device *pdev)
{
	struct courbet_fpc *fpc = platform_get_drvdata(pdev);

	misc_deregister(&fpc->misc);
}

static const struct of_device_id courbet_fpc_of_match[] = {
	{ .compatible = "nullptrinc,courbet-fpc" },
	{ }
};
MODULE_DEVICE_TABLE(of, courbet_fpc_of_match);

static struct platform_driver courbet_fpc_driver = {
	.probe = courbet_fpc_probe,
	.remove = courbet_fpc_remove,
	.driver = {
		.name = "courbet-fpc",
		.of_match_table = courbet_fpc_of_match,
		/* Open file descriptors retain devm-managed state. This built-in
		 * board driver must not be detached through sysfs while in use.
		 */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(courbet_fpc_driver);

MODULE_DESCRIPTION("Xiaomi courbet FPC power and IRQ transport");
MODULE_LICENSE("GPL");

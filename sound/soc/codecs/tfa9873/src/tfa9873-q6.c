// SPDX-License-Identifier: GPL-2.0-only
/* Courbet: bind the paired TFA9873 vendor core to the native Q6AFE transport.
 * Parameter IDs are from NXP tfa_platform_interface_definition.h in the
 * Android reference pinned in research/audio-bringup.json.
 */
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/errno.h>
#include "../../../qcom/qdsp6/q6afe.h"
#include "../inc/tfa9873-q6.h"

#define TFA_RX_MODULE 0x1000b911
#define TFA_RX_CONFIG 0x1000b921
#define TFA_RX_BYPASS 0x1000b923

static DEFINE_MUTEX(tfa_afe_lock);
/* The machine link owns this reference and detaches it before releasing it. */
static struct q6afe_port *tfa_rx;

int tfa9873_attach_afe(struct q6afe_port *port)
{
	int ret = 0;
	mutex_lock(&tfa_afe_lock);
	if (tfa_rx)
		ret = -EBUSY;
	else
		tfa_rx = port;
	mutex_unlock(&tfa_afe_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(tfa9873_attach_afe);

void tfa9873_detach_afe(struct q6afe_port *port)
{
	mutex_lock(&tfa_afe_lock);
	if (tfa_rx == port)
		tfa_rx = NULL;
	mutex_unlock(&tfa_afe_lock);
}
EXPORT_SYMBOL_GPL(tfa9873_detach_afe);

static int tfa_send(void *buf, int size, u32 param)
{
	int ret;
	if (!buf || size <= 0 || size > 4096)
		return -EINVAL;
	mutex_lock(&tfa_afe_lock);
	ret = tfa_rx ? q6afe_port_set_vendor_param(tfa_rx, TFA_RX_MODULE,
						 param, buf, size) : -ENODEV;
	mutex_unlock(&tfa_afe_lock);
	return ret;
}

int send_tfa_cal_in_band(void *buf, int size)
{
	return tfa_send(buf, size, TFA_RX_CONFIG);
}

int send_tfa_cal_apr(void *buf, int size, bool read)
{
	/* Diagnostic result reads require an additional AFE response protocol. */
	return read ? -EOPNOTSUPP : tfa_send(buf, size, TFA_RX_CONFIG);
}

int send_tfa_cal_set_bypass(void *buf, int size)
{
	if (size != sizeof(u32))
		return -EINVAL;
	return tfa_send(buf, size, TFA_RX_BYPASS);
}

int send_tfa_cal_set_tx_enable(void *buf, int size)
{
	/* No feedback-capture link has been validated yet. */
	return -EOPNOTSUPP;
}

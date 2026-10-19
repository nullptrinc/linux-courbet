// SPDX-License-Identifier: GPL-2.0-only
/* Xiaomi courbet speakers: Q6ASM -> Q6ADM -> primary MI2S -> two TFA9873. */
#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <dt-bindings/sound/qcom,q6afe.h>
#include "common.h"
#include "sdw.h"
#include <linux/soundwire/sdw.h>
#include "qdsp6/q6afe.h"
#include "../codecs/tfa9873/inc/tfa9873-q6.h"

struct courbet_audio {
	struct snd_soc_card card;
	struct q6afe_port *rx;
	struct sdw_stream_runtime *mic_stream;
	bool mic_prepared;
};

static int courbet_be_init(struct snd_soc_pcm_runtime *rtd)
{
	struct courbet_audio *data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	int ret;

	data->rx = q6afe_port_get_from_id(cpu->dev, PRIMARY_MI2S_RX);
	if (IS_ERR(data->rx))
		return PTR_ERR(data->rx);
	ret = tfa9873_attach_afe(data->rx);
	if (ret) {
		q6afe_port_put(data->rx);
		data->rx = NULL;
	}
	return ret;
}

static void courbet_be_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct courbet_audio *data = snd_soc_card_get_drvdata(rtd->card);
	if (data->rx) {
		tfa9873_detach_afe(data->rx);
		q6afe_port_put(data->rx);
		data->rx = NULL;
	}
}

static int courbet_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *format = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	rate->min = rate->max = 96000;
	channels->min = channels->max = 2;
	snd_mask_none(format);
	snd_mask_set_format(format, SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static int courbet_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0), *codec;
	int ret, i;

	ret = snd_soc_dai_set_fmt(cpu, SND_SOC_DAIFMT_BP_FP);
	if (ret)
		return ret;
	for_each_rtd_codec_dais(rtd, i, codec) {
		ret = snd_soc_dai_set_fmt(codec, SND_SOC_DAIFMT_I2S |
				SND_SOC_DAIFMT_NB_NF | SND_SOC_DAIFMT_BC_FC);
		if (ret)
			return ret;
	}
	return snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
				     96000 * 2 * 32, SNDRV_PCM_STREAM_PLAYBACK);
}

static void courbet_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_soc_substream_to_rtd(substream);
	struct snd_soc_dai *cpu = snd_soc_rtd_to_cpu(rtd, 0);
	int ret = snd_soc_dai_set_sysclk(cpu, Q6AFE_LPASS_CLK_ID_PRI_MI2S_IBIT,
				       0, SNDRV_PCM_STREAM_PLAYBACK);
	if (ret)
		dev_warn(rtd->card->dev, "MI2S clock stop failed: %d\n", ret);
}

static const struct snd_soc_ops courbet_ops = {
	.startup = courbet_startup,
	.shutdown = courbet_shutdown,
};

static int courbet_mic_fixup(struct snd_soc_pcm_runtime *rtd,
                             struct snd_pcm_hw_params *params)
{
 struct snd_interval *rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
 struct snd_interval *channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
 struct snd_mask *format = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
 rate->min = rate->max = 48000;
 channels->min = channels->max = 2;
 snd_mask_none(format);
 snd_mask_set_format(format, SNDRV_PCM_FORMAT_S16_LE);
 return 0;
}
static int courbet_mic_startup(struct snd_pcm_substream *s)
{
 struct courbet_audio *d = snd_soc_card_get_drvdata(snd_soc_substream_to_rtd(s)->card);
 int ret = qcom_snd_sdw_startup(s);
 if (ret)
  return ret;
 return qcom_snd_sdw_hw_params(s, NULL, &d->mic_stream);
}
static int courbet_mic_hw_params(struct snd_pcm_substream *s, struct snd_pcm_hw_params *p)
{
 struct courbet_audio *d = snd_soc_card_get_drvdata(snd_soc_substream_to_rtd(s)->card);
 return qcom_snd_sdw_hw_params(s, p, &d->mic_stream);
}
static int courbet_mic_prepare(struct snd_pcm_substream *s)
{
 struct courbet_audio *d = snd_soc_card_get_drvdata(snd_soc_substream_to_rtd(s)->card);
 return qcom_snd_sdw_prepare(s, d->mic_stream, &d->mic_prepared);
}
static int courbet_mic_hw_free(struct snd_pcm_substream *s)
{
 struct courbet_audio *d = snd_soc_card_get_drvdata(snd_soc_substream_to_rtd(s)->card);
 return qcom_snd_sdw_hw_free(s, d->mic_stream, &d->mic_prepared);
}
static void courbet_mic_shutdown(struct snd_pcm_substream *s)
{
 struct courbet_audio *d = snd_soc_card_get_drvdata(snd_soc_substream_to_rtd(s)->card);
 sdw_release_stream(d->mic_stream);
 d->mic_stream = NULL;
}
static const struct snd_soc_ops courbet_mic_ops = {
 .startup = courbet_mic_startup, .shutdown = courbet_mic_shutdown,
 .hw_params = courbet_mic_hw_params, .hw_free = courbet_mic_hw_free,
 .prepare = courbet_mic_prepare,
};

static int courbet_probe(struct platform_device *pdev)
{
	struct courbet_audio *data;
	struct snd_soc_dai_link *link;
	int ret, i;

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	data->card.dev = &pdev->dev;
	data->card.owner = THIS_MODULE;
	snd_soc_card_set_drvdata(&data->card, data);
	ret = qcom_snd_parse_of(&data->card);
	if (ret)
		return ret;
	for_each_card_prelinks(&data->card, i, link) {
		if (!link->no_pcm)
			continue;
		if (link->id == TX_CODEC_DMA_TX_3) {
			link->be_hw_params_fixup = courbet_mic_fixup;
			link->ops = &courbet_mic_ops;
			continue;
		}
		if (link->id != PRIMARY_MI2S_RX)
			return -EINVAL;
		link->init = courbet_be_init;
		link->exit = courbet_be_exit;
		link->be_hw_params_fixup = courbet_fixup;
		link->ops = &courbet_ops;
	}
	return devm_snd_soc_register_card(&pdev->dev, &data->card);
}

static const struct of_device_id courbet_match[] = {
	{ .compatible = "xiaomi,courbet-audio" },
	{}
};
MODULE_DEVICE_TABLE(of, courbet_match);
static struct platform_driver courbet_driver = {
	.probe = courbet_probe,
	.driver = { .name = "courbet-audio", .of_match_table = courbet_match },
};
module_platform_driver(courbet_driver);
MODULE_DESCRIPTION("Xiaomi courbet primary MI2S speaker audio");
MODULE_LICENSE("GPL");

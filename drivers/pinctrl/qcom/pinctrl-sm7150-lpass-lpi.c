// SPDX-License-Identifier: GPL-2.0-only
/* SM7150 LPASS pin routing verified against courbet Android DT.
 * Unverified alternate functions and slew controls are deliberately absent.
 */
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include "pinctrl-lpass-lpi.h"
enum { LPI_MUX_swr_tx_clk, LPI_MUX_swr_tx_data, LPI_MUX_swr_rx_clk,
       LPI_MUX_swr_rx_data, LPI_MUX_wcd_reset, LPI_MUX_gpio, LPI_MUX__ };
static const struct pinctrl_pin_desc sm7150_pins[] = {
	PINCTRL_PIN(0, "gpio0"),
	PINCTRL_PIN(1, "gpio1"),
	PINCTRL_PIN(2, "gpio2"),
	PINCTRL_PIN(3, "gpio3"),
	PINCTRL_PIN(4, "gpio4"),
	PINCTRL_PIN(5, "gpio5"),
	PINCTRL_PIN(6, "gpio6"),
	PINCTRL_PIN(7, "gpio7"),
	PINCTRL_PIN(8, "gpio8"),
	PINCTRL_PIN(9, "gpio9"),
	PINCTRL_PIN(10, "gpio10"),
	PINCTRL_PIN(11, "gpio11"),
	PINCTRL_PIN(12, "gpio12"),
	PINCTRL_PIN(13, "gpio13"),
	PINCTRL_PIN(14, "gpio14"),
	PINCTRL_PIN(15, "gpio15"),
	PINCTRL_PIN(16, "gpio16"),
	PINCTRL_PIN(17, "gpio17"),
	PINCTRL_PIN(18, "gpio18"),
	PINCTRL_PIN(19, "gpio19"),
	PINCTRL_PIN(20, "gpio20"),
	PINCTRL_PIN(21, "gpio21"),
	PINCTRL_PIN(22, "gpio22"),
	PINCTRL_PIN(23, "gpio23"),
	PINCTRL_PIN(24, "gpio24"),
	PINCTRL_PIN(25, "gpio25"),
	PINCTRL_PIN(26, "gpio26"),
	PINCTRL_PIN(27, "gpio27"),
	PINCTRL_PIN(28, "gpio28"),
	PINCTRL_PIN(29, "gpio29"),
	PINCTRL_PIN(30, "gpio30"),
	PINCTRL_PIN(31, "gpio31"),
};
static const char * const swr_tx_clk_groups[] = { "gpio18" };
static const char * const swr_tx_data_groups[] = { "gpio19", "gpio20" };
static const char * const swr_rx_clk_groups[] = { "gpio21" };
static const char * const swr_rx_data_groups[] = { "gpio22", "gpio23" };
static const char * const wcd_reset_groups[] = { "gpio24" };
static const struct lpi_function sm7150_functions[] = {
 LPI_FUNCTION(swr_tx_clk), LPI_FUNCTION(swr_tx_data),
 LPI_FUNCTION(swr_rx_clk), LPI_FUNCTION(swr_rx_data), LPI_FUNCTION(wcd_reset),
};
static const struct lpi_pingroup sm7150_groups[] = {
	LPI_PINGROUP(0, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(1, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(2, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(3, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(4, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(5, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(6, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(7, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(8, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(9, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(10, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(11, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(12, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(13, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(14, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(15, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(16, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(17, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(18, LPI_NO_SLEW, _, swr_tx_clk, _, _),
	LPI_PINGROUP(19, LPI_NO_SLEW, _, _, swr_tx_data, _),
	LPI_PINGROUP(20, LPI_NO_SLEW, _, swr_tx_data, _, _),
	LPI_PINGROUP(21, LPI_NO_SLEW, _, swr_rx_clk, _, _),
	LPI_PINGROUP(22, LPI_NO_SLEW, _, swr_rx_data, _, _),
	LPI_PINGROUP(23, LPI_NO_SLEW, _, swr_rx_data, _, _),
	LPI_PINGROUP(24, LPI_NO_SLEW, _, wcd_reset, _, _),
	LPI_PINGROUP(25, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(26, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(27, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(28, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(29, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(30, LPI_NO_SLEW, _, _, _, _),
	LPI_PINGROUP(31, LPI_NO_SLEW, _, _, _, _),
};
static const struct lpi_pinctrl_variant_data sm7150_data = {
 .pins=sm7150_pins, .npins=ARRAY_SIZE(sm7150_pins),
 .groups=sm7150_groups, .ngroups=ARRAY_SIZE(sm7150_groups),
 .functions=sm7150_functions, .nfunctions=ARRAY_SIZE(sm7150_functions),
 /* No separate slew mapping is used; all groups have LPI_NO_SLEW. */
 .flags=LPI_FLAG_SLEW_RATE_SAME_REG,
};
static const struct of_device_id sm7150_match[] = {
 { .compatible="qcom,sm7150-lpass-lpi-pinctrl", .data=&sm7150_data }, {}
};
MODULE_DEVICE_TABLE(of, sm7150_match);
static struct platform_driver sm7150_driver = {
 .probe=lpi_pinctrl_probe, .remove=lpi_pinctrl_remove,
 .driver={ .name="sm7150-lpass-lpi", .of_match_table=sm7150_match },
};
module_platform_driver(sm7150_driver);
MODULE_DESCRIPTION("SM7150 LPASS pin controller");
MODULE_LICENSE("GPL");

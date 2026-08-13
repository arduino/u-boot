// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 */

#include <clk.h>
#include <dm.h>
#include <dm/device_compat.h>
#include <dm/devres.h>
#include <dm/uclass.h>
#include <generic-phy.h>
#include <reset.h>
#include <power/regulator.h>
#include <usb/tcpm.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/compat.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/err.h>

#include "phy-qcom-qmp-common.h"

#include "phy-qcom-qmp.h"
#include "phy-qcom-qmp-pcs-misc-v3.h"

/* QPHY_V3_PCS_MISC_TYPEC_CTRL-equivalent bit, legacy USB3-only PHY */
#define SW_PORTSELECT_MUX			BIT(1)
#define SW_PORTSELECT_VAL			BIT(0)

#define PHY_INIT_COMPLETE_TIMEOUT		10000

#define RESET_SETTLE_DELAY_US			10

struct qmp_usb3_legacy_offsets {
	u16 serdes;
	u16 tx;
	u16 rx;
	u16 tx2;
	u16 rx2;
	u16 pcs;
	u16 pcs_misc;
};

static const struct qmp_usb3_legacy_offsets qmp_usb3_legacy_offsets_qcm2290 = {
	.serdes		= 0x000,
	.tx		= 0x200,
	.rx		= 0x400,
	.tx2		= 0x600,
	.rx2		= 0x800,
	.pcs_misc	= 0xa00,
	.pcs		= 0xc00,
};

static const struct qmp_phy_init_tbl qcm2290_usb3_serdes_tbl[] = {
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SYSCLK_EN_SEL, 0x14),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_BIAS_EN_CLKBUFLR_EN, 0x08),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_CLK_SELECT, 0x30),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SYS_CLK_CTRL, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_RESETSM_CNTRL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_RESETSM_CNTRL2, 0x08),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_BG_TRIM, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SVS_MODE_CLK_SEL, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_HSCLK_SEL, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_DEC_START_MODE0, 0x82),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_DIV_FRAC_START1_MODE0, 0x55),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_DIV_FRAC_START2_MODE0, 0x55),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_DIV_FRAC_START3_MODE0, 0x03),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_CP_CTRL_MODE0, 0x0b),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_PLL_RCTRL_MODE0, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_PLL_CCTRL_MODE0, 0x28),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_INTEGLOOP_GAIN0_MODE0, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_INTEGLOOP_GAIN1_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_CORECLK_DIV, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_LOCK_CMP1_MODE0, 0x15),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_LOCK_CMP2_MODE0, 0x34),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_LOCK_CMP3_MODE0, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_LOCK_CMP_EN, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_CORE_CLK_EN, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_LOCK_CMP_CFG, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_VCO_TUNE_MAP, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_BG_TIMER, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SSC_EN_CENTER, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SSC_PER1, 0x31),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SSC_PER2, 0x01),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SSC_ADJ_PER1, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SSC_ADJ_PER2, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SSC_STEP_SIZE1, 0xde),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_SSC_STEP_SIZE2, 0x07),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_PLL_IVCO, 0x0f),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_CMN_CONFIG, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_INTEGLOOP_INITVAL, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V2_COM_BIAS_EN_CTRL_BY_PSM, 0x01),
};

static const struct qmp_phy_init_tbl qcm2290_usb3_tx_tbl[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_HIGHZ_DRVR_EN, 0x10),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RCV_DETECT_LVL_2, 0x12),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_LANE_MODE_1, 0xc6),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RES_CODE_LANE_OFFSET_TX, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_TX_RES_CODE_LANE_OFFSET_RX, 0x00),
};

static const struct qmp_phy_init_tbl qcm2290_usb3_rx_tbl[] = {
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_FO_GAIN, 0x0b),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_PI_CONTROLS, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_COUNT_LOW, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FASTLOCK_COUNT_HIGH, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_FO_GAIN, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SO_GAIN, 0x06),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_UCDR_SO_SATURATION_AND_ENABLE, 0x75),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL2, 0x02),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL3, 0x4e),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQU_ADAPTOR_CNTRL4, 0x18),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_EQ_OFFSET_ADAPTOR_CNTRL1, 0x77),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_OFFSET_ADAPTOR_CNTRL2, 0x80),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_VGA_CAL_CNTRL2, 0x0a),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_CNTRL, 0x03),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_DEGLITCH_CNTRL, 0x16),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_SIGDET_ENABLES, 0x00),
	QMP_PHY_INIT_CFG(QSERDES_V3_RX_RX_MODE_00, 0x00),
};

static const struct qmp_phy_init_tbl qcm2290_usb3_pcs_tbl[] = {
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TXMGN_V0, 0x9f),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TXDEEMPH_M6DB_V0, 0x17),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TXDEEMPH_M3P5DB_V0, 0x0f),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNTRL2, 0x83),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNTRL1, 0x02),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNT_VAL_L, 0x09),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_CNT_VAL_H_TOL, 0xa2),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_FLL_MAN_CODE, 0x85),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LOCK_DETECT_CONFIG1, 0xd1),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LOCK_DETECT_CONFIG2, 0x1f),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LOCK_DETECT_CONFIG3, 0x47),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RXEQTRAINING_WAIT_TIME, 0x75),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RXEQTRAINING_RUN_TIME, 0x13),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_LFPS_TX_ECSTART_EQTLOCK, 0x86),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_PWRUP_RESET_DLY_TIME_AUXCLK, 0x04),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_TSYNC_RSYNC_TIME, 0x44),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_P1U2_L, 0xe7),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_P1U2_H, 0x03),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_U3_L, 0x40),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RCVR_DTCT_DLY_U3_H, 0x00),
	QMP_PHY_INIT_CFG(QPHY_V3_PCS_RX_SIGDET_LVL, 0x88),
};

/* Per-compatible config; add a new instance here to support another SoC */
struct qmp_usb3_legacy_cfg {
	const struct qmp_usb3_legacy_offsets *offsets;

	const struct qmp_phy_init_tbl *serdes_tbl;
	int serdes_tbl_num;
	const struct qmp_phy_init_tbl *tx_tbl;
	int tx_tbl_num;
	const struct qmp_phy_init_tbl *rx_tbl;
	int rx_tbl_num;
	const struct qmp_phy_init_tbl *pcs_tbl;
	int pcs_tbl_num;

	/* clock names fetched by clk_get_by_name(); "pipe" is fetched separately */
	const char * const *clk_list;
	int num_clks;

	const char * const *vreg_list;
	int num_vregs;
};

static const char * const shikra_usb3_clk_l[] = {
	"cfg_ahb", "ref", "com_aux",
};

static const struct qmp_usb3_legacy_cfg shikra_usb3phy_cfg = {
	.offsets	= &qmp_usb3_legacy_offsets_qcm2290,
	.serdes_tbl	= qcm2290_usb3_serdes_tbl,
	.serdes_tbl_num	= ARRAY_SIZE(qcm2290_usb3_serdes_tbl),
	.tx_tbl		= qcm2290_usb3_tx_tbl,
	.tx_tbl_num	= ARRAY_SIZE(qcm2290_usb3_tx_tbl),
	.rx_tbl		= qcm2290_usb3_rx_tbl,
	.rx_tbl_num	= ARRAY_SIZE(qcm2290_usb3_rx_tbl),
	.pcs_tbl	= qcm2290_usb3_pcs_tbl,
	.pcs_tbl_num	= ARRAY_SIZE(qcm2290_usb3_pcs_tbl),
	.clk_list	= shikra_usb3_clk_l,
	.num_clks	= ARRAY_SIZE(shikra_usb3_clk_l),
	.vreg_list	= NULL,
	.num_vregs	= 0,
};

static const char * const talos_usb3_clk_l[] = {
	"aux", "ref", "cfg_ahb",
};

static const char * const talos_usb3_vreg_l[] = {
	"vdda-phy-supply", "vdda-pll-supply",
};

static const struct qmp_usb3_legacy_cfg talos_usb3phy_cfg = {
	.offsets	= &qmp_usb3_legacy_offsets_qcm2290,
	.serdes_tbl	= qcm2290_usb3_serdes_tbl,
	.serdes_tbl_num	= ARRAY_SIZE(qcm2290_usb3_serdes_tbl),
	.tx_tbl		= qcm2290_usb3_tx_tbl,
	.tx_tbl_num	= ARRAY_SIZE(qcm2290_usb3_tx_tbl),
	.rx_tbl		= qcm2290_usb3_rx_tbl,
	.rx_tbl_num	= ARRAY_SIZE(qcm2290_usb3_rx_tbl),
	.pcs_tbl	= qcm2290_usb3_pcs_tbl,
	.pcs_tbl_num	= ARRAY_SIZE(qcm2290_usb3_pcs_tbl),
	.clk_list	= talos_usb3_clk_l,
	.num_clks	= ARRAY_SIZE(talos_usb3_clk_l),
	.vreg_list	= talos_usb3_vreg_l,
	.num_vregs	= ARRAY_SIZE(talos_usb3_vreg_l),
};

struct qmp_usb3_legacy {
	struct udevice *dev;
	void __iomem *serdes;
	void __iomem *tx;
	void __iomem *rx;
	void __iomem *tx2;
	void __iomem *rx2;
	void __iomem *pcs;
	void __iomem *pcs_misc;
	struct clk *clks;
	struct clk pipe_clk;
	int num_clks;
	struct reset_ctl_bulk resets;
	struct udevice **vregs;
	int num_vregs;
	const struct qmp_usb3_legacy_cfg *cfg;
};

static inline void qphy_setbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg |= val;
	writel(reg, base + offset);
	readl(base + offset);
}

static inline void qphy_clrbits(void __iomem *base, u32 offset, u32 val)
{
	u32 reg;

	reg = readl(base + offset);
	reg &= ~val;
	writel(reg, base + offset);
	readl(base + offset);
}

/* Quiesce the PHY; shared by power_off() and power_on()'s error path */
static void qmp_usb3_legacy_quiesce(struct qmp_usb3_legacy *qmp)
{
	qphy_clrbits(qmp->pcs, QPHY_V3_PCS_START_CONTROL, SERDES_START | PCS_START);
	qphy_setbits(qmp->pcs, QPHY_V3_PCS_SW_RESET, SW_RESET);
	qphy_clrbits(qmp->pcs, QPHY_V3_PCS_POWER_DOWN_CONTROL, SW_PWRDN);
}

/* Fixed orientation by default; use TCPM orientation if available */
static u32 qmp_usb3_legacy_get_portselect(struct qmp_usb3_legacy *qmp)
{
	u32 val = SW_PORTSELECT_MUX;

	if (IS_ENABLED(CONFIG_TYPEC_TCPM)) {
		struct udevice *tcpm_dev;
		enum typec_orientation orientation;
		int ret;

		ret = uclass_first_device_check(UCLASS_TCPM, &tcpm_dev);
		if (!ret && tcpm_dev) {
			orientation = tcpm_get_orientation(tcpm_dev);
			if (orientation == TYPEC_ORIENTATION_REVERSE)
				val |= SW_PORTSELECT_VAL;
		}
	}

	return val;
}

static int qmp_usb3_legacy_power_on(struct phy *phy)
{
	struct qmp_usb3_legacy *qmp = dev_get_priv(phy->dev);
	const struct qmp_usb3_legacy_cfg *cfg = qmp->cfg;
	void __iomem *status;
	u32 val;
	int ret;

	qphy_setbits(qmp->pcs, QPHY_V3_PCS_POWER_DOWN_CONTROL, SW_PWRDN);

	writel(qmp_usb3_legacy_get_portselect(qmp), qmp->pcs_misc);

	qmp_configure(qmp->dev, qmp->serdes, cfg->serdes_tbl, cfg->serdes_tbl_num);

	ret = clk_enable(&qmp->pipe_clk);
	if (ret) {
		dev_err(qmp->dev, "pipe_clk enable failed err=%d\n", ret);
		return ret;
	}

	qmp_configure_lane(qmp->dev, qmp->tx, cfg->tx_tbl, cfg->tx_tbl_num, 1);
	qmp_configure_lane(qmp->dev, qmp->rx, cfg->rx_tbl, cfg->rx_tbl_num, 1);

	qmp_configure_lane(qmp->dev, qmp->tx2, cfg->tx_tbl, cfg->tx_tbl_num, 2);
	qmp_configure_lane(qmp->dev, qmp->rx2, cfg->rx_tbl, cfg->rx_tbl_num, 2);

	qmp_configure(qmp->dev, qmp->pcs, cfg->pcs_tbl, cfg->pcs_tbl_num);

	qphy_clrbits(qmp->pcs, QPHY_V3_PCS_SW_RESET, SW_RESET);

	qphy_setbits(qmp->pcs, QPHY_V3_PCS_START_CONTROL, SERDES_START | PCS_START);

	status = qmp->pcs + QPHY_V3_PCS_PCS_STATUS;
	ret = readl_poll_timeout(status, val, !(val & PHYSTATUS),
				 PHY_INIT_COMPLETE_TIMEOUT);
	if (ret) {
		dev_err(qmp->dev, "phy initialization timed-out\n");
		clk_disable(&qmp->pipe_clk);
		qmp_usb3_legacy_quiesce(qmp);
		return ret;
	}

	return 0;
}

static int qmp_usb3_legacy_power_off(struct phy *phy)
{
	struct qmp_usb3_legacy *qmp = dev_get_priv(phy->dev);

	clk_disable(&qmp->pipe_clk);
	qmp_usb3_legacy_quiesce(qmp);

	return 0;
}

static void qmp_usb3_legacy_clk_disable_upto(struct qmp_usb3_legacy *qmp, int upto)
{
	int i;

	for (i = 0; i < upto; i++)
		clk_disable(&qmp->clks[i]);
}

static int qmp_usb3_legacy_clk_init(struct qmp_usb3_legacy *qmp)
{
	const struct qmp_usb3_legacy_cfg *cfg = qmp->cfg;
	struct udevice *dev = qmp->dev;
	int num = cfg->num_clks;
	int i, ret;

	qmp->clks = devm_kcalloc(dev, num, sizeof(*qmp->clks), GFP_KERNEL);
	if (!qmp->clks)
		return -ENOMEM;

	qmp->num_clks = 0;

	for (i = 0; i < num; i++) {
		ret = clk_get_by_name(dev, cfg->clk_list[i], &qmp->clks[i]);
		if (ret) {
			dev_err(dev, "failed to get %s clock: %d\n",
				cfg->clk_list[i], ret);
			qmp_usb3_legacy_clk_disable_upto(qmp, i);
			return ret;
		}

		ret = clk_enable(&qmp->clks[i]);
		if (ret) {
			dev_err(dev, "failed to enable %s clock: %d\n",
				cfg->clk_list[i], ret);
			qmp_usb3_legacy_clk_disable_upto(qmp, i);
			return ret;
		}
	}
	qmp->num_clks = num;

	ret = clk_get_by_name(dev, "pipe", &qmp->pipe_clk);
	if (ret) {
		dev_err(dev, "failed to get pipe clock: %d\n", ret);
		qmp_usb3_legacy_clk_disable_upto(qmp, qmp->num_clks);
		return ret;
	}

	return 0;
}

static int qmp_usb3_legacy_vreg_init(struct qmp_usb3_legacy *qmp)
{
	const struct qmp_usb3_legacy_cfg *cfg = qmp->cfg;
	struct udevice *dev = qmp->dev;
	int i, ret;

	qmp->num_vregs = 0;

	if (!cfg->num_vregs)
		return 0;

	qmp->vregs = devm_kcalloc(dev, cfg->num_vregs, sizeof(*qmp->vregs), GFP_KERNEL);
	if (!qmp->vregs)
		return -ENOMEM;

	for (i = 0; i < cfg->num_vregs; i++) {
		ret = device_get_supply_regulator(dev, cfg->vreg_list[i], &qmp->vregs[i]);
		if (ret) {
			dev_err(dev, "failed to get regulator %s: %d\n",
				cfg->vreg_list[i], ret);
			return ret;
		}
	}

	qmp->num_vregs = cfg->num_vregs;
	return 0;
}

static int qmp_usb3_legacy_vreg_enable(struct qmp_usb3_legacy *qmp)
{
	int i, ret;

	for (i = 0; i < qmp->num_vregs; i++) {
		ret = regulator_set_enable(qmp->vregs[i], true);
		if (ret) {
			dev_err(qmp->dev, "failed to enable regulator %d: %d\n", i, ret);
			while (--i >= 0)
				regulator_set_enable(qmp->vregs[i], false);
			return ret;
		}
	}

	return 0;
}

static void qmp_usb3_legacy_vreg_disable(struct qmp_usb3_legacy *qmp)
{
	int i;

	for (i = qmp->num_vregs - 1; i >= 0; i--)
		regulator_set_enable(qmp->vregs[i], false);
}

static int qmp_usb3_legacy_parse_dt(struct qmp_usb3_legacy *qmp)
{
	const struct qmp_usb3_legacy_cfg *cfg = qmp->cfg;
	const struct qmp_usb3_legacy_offsets *offs = cfg->offsets;
	struct udevice *dev = qmp->dev;
	void __iomem *base;
	int ret;

	base = (void __iomem *)dev_read_addr(dev);
	if (IS_ERR(base))
		return PTR_ERR(base);

	qmp->serdes = base + offs->serdes;
	qmp->tx = base + offs->tx;
	qmp->rx = base + offs->rx;
	qmp->tx2 = base + offs->tx2;
	qmp->rx2 = base + offs->rx2;
	qmp->pcs = base + offs->pcs;
	qmp->pcs_misc = base + offs->pcs_misc;

	ret = qmp_usb3_legacy_vreg_init(qmp);
	if (ret)
		return ret;

	ret = qmp_usb3_legacy_vreg_enable(qmp);
	if (ret)
		return ret;

	ret = reset_get_bulk(dev, &qmp->resets);
	if (ret) {
		dev_err(dev, "failed to get resets: %d\n", ret);
		goto err_disable_vregs;
	}

	ret = reset_assert_bulk(&qmp->resets);
	if (ret) {
		dev_err(dev, "failed to assert resets: %d\n", ret);
		goto err_disable_vregs;
	}

	udelay(RESET_SETTLE_DELAY_US);

	ret = reset_deassert_bulk(&qmp->resets);
	if (ret) {
		dev_err(dev, "failed to deassert resets: %d\n", ret);
		goto err_assert_resets;
	}

	ret = qmp_usb3_legacy_clk_init(qmp);
	if (ret)
		goto err_assert_resets;

	return 0;

err_assert_resets:
	reset_assert_bulk(&qmp->resets);
err_disable_vregs:
	qmp_usb3_legacy_vreg_disable(qmp);

	return ret;
}

static int qmp_usb3_legacy_probe(struct udevice *dev)
{
	struct qmp_usb3_legacy *qmp = dev_get_priv(dev);

	qmp->dev = dev;
	qmp->cfg = (const struct qmp_usb3_legacy_cfg *)dev_get_driver_data(dev);
	if (!qmp->cfg) {
		dev_err(dev, "failed to get PHY configuration\n");
		return -EINVAL;
	}

	return qmp_usb3_legacy_parse_dt(qmp);
}

static struct phy_ops qmp_usb3_legacy_ops = {
	.power_on = qmp_usb3_legacy_power_on,
	.power_off = qmp_usb3_legacy_power_off,
};

static const struct udevice_id qmp_usb3_legacy_ids[] = {
	{
		.compatible = "qcom,shikra-qmp-usb3-phy",
		.data = (ulong)&shikra_usb3phy_cfg,
	},
	{ }
};

U_BOOT_DRIVER(qcom_qmp_usb3_legacy_phy) = {
	.name = "qcom-qmp-usb3-legacy-phy",
	.id = UCLASS_PHY,
	.of_match = qmp_usb3_legacy_ids,
	.ops = &qmp_usb3_legacy_ops,
	.probe = qmp_usb3_legacy_probe,
	.priv_auto = sizeof(struct qmp_usb3_legacy),
};

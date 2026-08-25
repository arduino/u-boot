// SPDX-License-Identifier: GPL-2.0+
/*
 * Qualcomm SDHCI driver - SD/eMMC controller
 *
 * (C) Copyright 2015 Mateusz Kulikowski <mateusz.kulikowski@gmail.com>
 *
 * Based on Linux driver
 */

#include <clk.h>
#include <dm.h>
#include <malloc.h>
#include <reset.h>
#include <sdhci.h>
#include <wait_bit.h>
#include <asm/io.h>
#include <linux/bitops.h>
#include <linux/iopoll.h>
#include <power/regulator.h>
#include <memalign.h>

/* Non-standard registers needed for SDHCI startup */
#define SDCC_MCI_POWER   0x0
#define SDCC_MCI_POWER_SW_RST BIT(7)

/* This is undocumented register */
#define SDCC_MCI_VERSION		0x50
#define SDCC_V5_VERSION			0x318

#define SDCC_VERSION_MAJOR_SHIFT	28
#define SDCC_VERSION_MAJOR_MASK		(0xf << SDCC_VERSION_MAJOR_SHIFT)
#define SDCC_VERSION_MINOR_MASK		0xff

#define SDCC_MCI_STATUS2 0x6C
#define SDCC_MCI_STATUS2_MCI_ACT 0x1
#define SDCC_MCI_HC_MODE 0x78

#define CORE_VENDOR_SPEC_POR_VAL 0xa9c

#define CORE_DLL_PDN		BIT(29)
#define CORE_DLL_RST		BIT(30)

/* DLL configuration */
#define CORE_DLL_EN		BIT(16)
#define CORE_CDR_EN		BIT(17)
#define CORE_CK_OUT_EN		BIT(18)
#define CORE_CDR_EXT_EN		BIT(19)
#define CORE_DLL_LOCK		BIT(7)
#define CORE_CMD_DAT_TRACK_SEL	BIT(0)

#define CDR_SELEXT_SHIFT	20
#define CDR_SELEXT_MASK		(0xf << CDR_SELEXT_SHIFT)

/* MCLK frequency selection (CMUX_SHIFT_PHASE) - required for the DLL to lock */
#define CMUX_SHIFT_PHASE_SHIFT	24
#define CMUX_SHIFT_PHASE_MASK	(7 << CMUX_SHIFT_PHASE_SHIFT)

/* 14LPP DLL reset / Tassadar DLL extras, for SDCC minor >= 0x42 / 0x71 */
#define CORE_DLL_CLOCK_DISABLE	BIT(21)

#define DLL_USR_CTL_POR_VAL	0x10800
#define ENABLE_DLL_LOCK_STATUS	BIT(26)
#define FINE_TUNE_MODE_EN	BIT(27)
#define BIAS_OK_SIGNAL		BIT(29)

#define DLL_CONFIG_3_LOW_FREQ_VAL	0x08
#define DLL_CONFIG_3_HIGH_FREQ_VAL	0x10

#define CORE_CLK_PWRSAVE	BIT(1)

/* Timing mode selection */
#define CORE_HC_MCLK_SEL_DFLT	(2 << 8)
#define CORE_HC_MCLK_SEL_HS400	(3 << 8)
#define CORE_HC_MCLK_SEL_MASK	(3 << 8)
#define CORE_HC_SELECT_IN_EN	BIT(18)
#define CORE_HC_SELECT_IN_HS400	(6 << 19)
#define CORE_HC_SELECT_IN_MASK	(7 << 19)

/* HS400 DDR/SDC4 DLL calibration, needed alongside the SDR CM_DLL above */
#define CORE_DDR_CAL_EN		BIT(0)
#define CORE_DDR_DLL_LOCK	BIT(11)
#define CORE_PWRSAVE_DLL	BIT(3)
#define CORE_CMDIN_RCLK_EN	BIT(1)
#define DDR_CONFIG_POR_VAL	0x80040873

#define CORE_FREQ_100MHZ	(100 * 1000000)
#define MAX_PHASES		16

#define MHZ(X) ((X) * 1000000UL)

struct msm_sdhc_plat {
	struct mmc_config cfg;
	struct mmc mmc;
};

struct msm_sdhc {
	struct sdhci_host host;
	void *base;
	struct clk_bulk clks;
	struct udevice *vqmmc;

	/* HS200/HS400 tuning and calibration state */
	bool tuning_done;
	bool calibration_done;
	u8 saved_tuning_phase;

	/* DLL init sequence variant selection, from the SDCC core version */
	u32 dll_config;		/* qcom,dll-config DT override, 0 if absent */
	bool use_14lpp_dll_reset;	/* core_minor >= 0x42 */
	bool uses_tassadar_dll;		/* core_minor >= 0x71 */

	/* HS400 support */
	bool use_cdclp533;		/* core_minor < 0x34, legacy calibration path */
	bool updated_ddr_cfg;		/* core_minor >= 0x49 */
	u32 ddr_config;			/* qcom,ddr-config DT override, else POR value */
};

struct msm_sdhc_variant_info {
	bool mci_removed;

	u32 core_dll_config;
	u32 core_dll_status;
	u32 core_dll_config_2;
	u32 core_dll_config_3;
	u32 core_dll_usr_ctl; /* Present on SDCC5.1 onwards */
	u32 core_vendor_spec;
	u32 core_vendor_spec_capabilities0;

	/* HS400 DDR/SDC4 calibration registers */
	u32 core_ddr_200_cfg;
	u32 core_vendor_spec3;
	u32 core_ddr_config_old; /* Applicable to sdcc minor ver < 0x49; 0 if N/A */
	u32 core_ddr_config;
};

static int msm_sdc_clk_init(struct udevice *dev)
{
	struct msm_sdhc *prv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info;
	ulong clk_rate;
	int ret, i = 0, n_clks;
	const char *clk_name;

	var_info = (void *)dev_get_driver_data(dev);

	clk_rate = dev_read_u32_default(dev, "max-frequency", 201500000);

	ret = clk_get_bulk(dev, &prv->clks);
	if (ret) {
		log_warning("Couldn't get mmc clocks: %d\n", ret);
		return ret;
	}

	ret = clk_enable_bulk(&prv->clks);
	if (ret) {
		log_warning("Couldn't enable mmc clocks: %d\n", ret);
		return ret;
	}

	/* If clock-names is unspecified, then the first clock is the core clock */
	if (!dev_read_prop(dev, "clock-names", &n_clks)) {
		if (!clk_set_rate(&prv->clks.clks[0], clk_rate)) {
			log_warning("Couldn't set core clock rate: %d\n", ret);
			return -EINVAL;
		}
	}

	/* Find the index of the "core" clock */
	while (i < n_clks) {
		dev_read_string_index(dev, "clock-names", i, &clk_name);
		if (!strcmp(clk_name, "core"))
			break;
		i++;
	}

	if (i >= prv->clks.count) {
		log_warning("Couldn't find core clock (index %d but only have %d clocks)\n", i,
		       prv->clks.count);
		return -EINVAL;
	}

	/* The clock is already enabled by the clk_bulk above */
	clk_rate = clk_set_rate(&prv->clks.clks[i], clk_rate);
	/* If we get a rate of 0 then something has probably gone wrong. */
	if (clk_rate == 0 || IS_ERR((void *)clk_rate)) {
		log_warning("Couldn't set MMC core clock rate: %dE\n", clk_rate ? (int)PTR_ERR((void *)clk_rate) : 0);
		return -EINVAL;
	}

	/* This is the base clock sdhci core will use to configure the SDCLK */
	prv->host.max_clk = clk_rate;

	writel_relaxed(CORE_VENDOR_SPEC_POR_VAL,
		       prv->host.ioaddr + var_info->core_vendor_spec);

	return 0;
}

static int msm_sdc_mci_init(struct msm_sdhc *prv)
{
	/* Reset the core and Enable SDHC mode */
	writel(readl(prv->base + SDCC_MCI_POWER) | SDCC_MCI_POWER_SW_RST,
	       prv->base + SDCC_MCI_POWER);

	/* Wait for reset to be written to register */
	if (wait_for_bit_le32(prv->base + SDCC_MCI_STATUS2,
			      SDCC_MCI_STATUS2_MCI_ACT, false, 10, false)) {
		printf("msm_sdhci: reset request failed\n");
		return -EIO;
	}

	/* SW reset can take upto 10HCLK + 15MCLK cycles. (min 40us) */
	if (wait_for_bit_le32(prv->base + SDCC_MCI_POWER,
			      SDCC_MCI_POWER_SW_RST, false, 2, false)) {
		printf("msm_sdhci: stuck in reset\n");
		return -ETIMEDOUT;
	}

	/* Enable host-controller mode */
	writel(1, prv->base + SDCC_MCI_HC_MODE);

	return 0;
}

static int msm_dll_poll_ck_out_en(struct sdhci_host *host, u8 poll)
{
	u32 ck_out_en;
	struct udevice *dev = host->mmc->dev;
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	int ret;

	ret = read_poll_timeout(readl, ck_out_en,
				(!!(ck_out_en & CORE_CK_OUT_EN) == poll),
				1, 50,
				host->ioaddr + var_info->core_dll_config);
	if (ret) {
		printf("%s: CK_OUT_EN bit is not %d\n",
		       host->name, poll);
		return -ETIMEDOUT;
	}

	return 0;
}

static int msm_config_cm_dll_phase(struct sdhci_host *host, u8 phase)
{
	int rc;
	static const u8 grey_coded_phase_table[] = {
		0x0, 0x1, 0x3, 0x2, 0x6, 0x7, 0x5, 0x4,
		0xc, 0xd, 0xf, 0xe, 0xa, 0xb, 0x9, 0x8
	};
	u32 config;
	struct udevice *dev = host->mmc->dev;
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);

	if (phase > 0xf)
		return -EINVAL;

	config = readl(host->ioaddr + var_info->core_dll_config);
	config &= ~(CORE_CDR_EN | CORE_CK_OUT_EN);
	config |= (CORE_CDR_EXT_EN | CORE_DLL_EN);
	writel(config, host->ioaddr + var_info->core_dll_config);

	rc = msm_dll_poll_ck_out_en(host, 0);
	if (rc)
		return rc;

	config = readl(host->ioaddr + var_info->core_dll_config);
	config &= ~CDR_SELEXT_MASK;
	config |= grey_coded_phase_table[phase] << CDR_SELEXT_SHIFT;
	writel(config, host->ioaddr + var_info->core_dll_config);

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_CK_OUT_EN;
	writel(config, host->ioaddr + var_info->core_dll_config);

	rc = msm_dll_poll_ck_out_en(host, 1);
	if (rc)
		return rc;

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_CDR_EN;
	config &= ~CORE_CDR_EXT_EN;
	writel(config, host->ioaddr + var_info->core_dll_config);

	return 0;
}

/*
 * Programs the MCLK_FREQ (CMUX_SHIFT_PHASE) field, required before enabling
 * CORE_DLL_EN/CORE_CK_OUT_EN or the DLL never locks. Matches the kernel
 * driver's msm_cm_dll_set_freq().
 *
 * host->clock is never updated by this U-Boot's generic sdhci.c, so
 * mmc->clock is used instead.
 */
static void msm_cm_dll_set_freq(struct sdhci_host *host)
{
	struct udevice *dev = host->mmc->dev;
	struct mmc *mmc = host->mmc;
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	u32 mclk_freq = 0, config;
	unsigned int clock = mmc->clock;

	if (clock <= 112000000)
		mclk_freq = 0;
	else if (clock <= 125000000)
		mclk_freq = 1;
	else if (clock <= 137000000)
		mclk_freq = 2;
	else if (clock <= 150000000)
		mclk_freq = 3;
	else if (clock <= 162000000)
		mclk_freq = 4;
	else if (clock <= 175000000)
		mclk_freq = 5;
	else if (clock <= 187000000)
		mclk_freq = 6;
	else if (clock <= 200000000)
		mclk_freq = 7;

	config = readl(host->ioaddr + var_info->core_dll_config);
	config &= ~CMUX_SHIFT_PHASE_MASK;
	config |= mclk_freq << CMUX_SHIFT_PHASE_SHIFT;
	writel(config, host->ioaddr + var_info->core_dll_config);
}

static int msm_dll_poll_lock_status(struct sdhci_host *host)
{
	u32 dll_status;
	struct udevice *dev = host->mmc->dev;
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	int ret;

	ret = read_poll_timeout(readl, dll_status,
				(dll_status & CORE_DLL_LOCK),
				1, 50,
				host->ioaddr + var_info->core_dll_status);
	if (ret) {
		printf("%s: DLL failed to LOCK (DLL_STATUS=0x%08x)\n",
		       host->name,
		       readl(host->ioaddr + var_info->core_dll_status));
		return -ETIMEDOUT;
	}

	return 0;
}

/* Matches the kernel sdhci-msm.c msm_init_cm_dll() sequence */
static int msm_init_cm_dll(struct sdhci_host *host)
{
	struct udevice *dev = host->mmc->dev;
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	u32 config;
	int ret;

	/*
	 * Keep the clock enabled while DLL tuning is in progress; PWRSAVE
	 * may otherwise turn it off.
	 */
	config = readl(host->ioaddr + var_info->core_vendor_spec);
	config &= ~CORE_CLK_PWRSAVE;
	writel(config, host->ioaddr + var_info->core_vendor_spec);

	if (priv->dll_config)
		writel(priv->dll_config, host->ioaddr + var_info->core_dll_config);

	if (priv->use_14lpp_dll_reset) {
		config = readl(host->ioaddr + var_info->core_dll_config);
		config &= ~CORE_CK_OUT_EN;
		writel(config, host->ioaddr + var_info->core_dll_config);

		if (var_info->core_dll_config_2) {
			config = readl(host->ioaddr + var_info->core_dll_config_2);
			config |= CORE_DLL_CLOCK_DISABLE;
			writel(config, host->ioaddr + var_info->core_dll_config_2);
		}
	}

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_DLL_RST;
	writel(config, host->ioaddr + var_info->core_dll_config);

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_DLL_PDN;
	writel(config, host->ioaddr + var_info->core_dll_config);

	/*
	 * MCLK_FREQ must be programmed while the DLL is reset and powered
	 * down, unless a DT dll-config override is in effect.
	 */
	if (!priv->dll_config)
		msm_cm_dll_set_freq(host);

	config = readl(host->ioaddr + var_info->core_dll_config);
	config &= ~CORE_DLL_RST;
	writel(config, host->ioaddr + var_info->core_dll_config);

	config = readl(host->ioaddr + var_info->core_dll_config);
	config &= ~CORE_DLL_PDN;
	writel(config, host->ioaddr + var_info->core_dll_config);

	if (priv->use_14lpp_dll_reset) {
		if (!priv->dll_config)
			msm_cm_dll_set_freq(host);

		if (var_info->core_dll_config_2) {
			config = readl(host->ioaddr + var_info->core_dll_config_2);
			config &= ~CORE_DLL_CLOCK_DISABLE;
			writel(config, host->ioaddr + var_info->core_dll_config_2);
		}
	}

	/* Applicable to SDCC v5.1 onwards only */
	if (priv->uses_tassadar_dll && var_info->core_dll_usr_ctl) {
		config = DLL_USR_CTL_POR_VAL | FINE_TUNE_MODE_EN |
			 ENABLE_DLL_LOCK_STATUS | BIAS_OK_SIGNAL;
		writel(config, host->ioaddr + var_info->core_dll_usr_ctl);

		if (var_info->core_dll_config_3) {
			config = readl(host->ioaddr + var_info->core_dll_config_3);
			config &= ~0xFF;
			if (host->mmc->clock < 150000000)
				config |= DLL_CONFIG_3_LOW_FREQ_VAL;
			else
				config |= DLL_CONFIG_3_HIGH_FREQ_VAL;
			writel(config, host->ioaddr + var_info->core_dll_config_3);
		}
	}

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_DLL_EN;
	writel(config, host->ioaddr + var_info->core_dll_config);

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_CK_OUT_EN;
	writel(config, host->ioaddr + var_info->core_dll_config);

	ret = msm_dll_poll_lock_status(host);
	if (ret)
		return ret;

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_CDR_EN;
	config &= ~CORE_CDR_EXT_EN;
	writel(config, host->ioaddr + var_info->core_dll_config);

	priv->calibration_done = false;

	return 0;
}

/*
 * HS400 DDR calibration via SDC4 CM_DLL, used when use_cdclp533 is false
 * (core_minor >= 0x34). Matches the kernel's
 * sdhci_msm_cm_dll_sdc4_calibration().
 */
static int sdhci_msm_cm_dll_sdc4_calibration(struct sdhci_host *host)
{
	struct udevice *dev = host->mmc->dev;
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	u32 config, ddr_cfg_offset, wait_cnt;
	int ret;

	/*
	 * core_ddr_config defaults to the desired configuration on reset;
	 * reprogram the POR value in case an earlier bootloader stage
	 * modified it.
	 */
	if (priv->updated_ddr_cfg)
		ddr_cfg_offset = var_info->core_ddr_config;
	else
		ddr_cfg_offset = var_info->core_ddr_config_old;
	writel(priv->ddr_config, host->ioaddr + ddr_cfg_offset);

	config = readl(host->ioaddr + var_info->core_ddr_200_cfg);
	config &= ~CORE_CMDIN_RCLK_EN;
	writel(config, host->ioaddr + var_info->core_ddr_200_cfg);

	config = readl(host->ioaddr + var_info->core_dll_config_2);
	config |= CORE_DDR_CAL_EN;
	writel(config, host->ioaddr + var_info->core_dll_config_2);

	ret = -ETIMEDOUT;
	wait_cnt = 100;
	while (wait_cnt--) {
		if (readl(host->ioaddr + var_info->core_dll_status) &
		    CORE_DDR_DLL_LOCK) {
			ret = 0;
			break;
		}
		udelay(10);
	}

	if (ret) {
		printf("%s: CM_DLL_SDC4 calibration was not completed\n",
		       host->name);
		return ret;
	}

	/*
	 * Skip CORE_PWRSAVE_DLL for the 14lpp DLL reset variant: it cannot
	 * guarantee the MCLK-gating timing PWRSAVE_DLL depends on.
	 */
	if (!priv->use_14lpp_dll_reset) {
		config = readl(host->ioaddr + var_info->core_vendor_spec3);
		config |= CORE_PWRSAVE_DLL;
		writel(config, host->ioaddr + var_info->core_vendor_spec3);
	}

	return 0;
}

/*
 * HS400 DLL calibration, performed once when transitioning into HS400 at
 * clock > 100MHz. Matches the kernel's sdhci_msm_hs400_dll_calibration().
 */
static int sdhci_msm_hs400_dll_calibration(struct sdhci_host *host)
{
	struct udevice *dev = host->mmc->dev;
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	u32 config;
	int ret;

	ret = msm_init_cm_dll(host);
	if (ret)
		return ret;

	/* Restore the phase found during HS200 tuning */
	ret = msm_config_cm_dll_phase(host, priv->saved_tuning_phase);
	if (ret)
		return ret;

	config = readl(host->ioaddr + var_info->core_dll_config);
	config |= CORE_CMD_DAT_TRACK_SEL;
	writel(config, host->ioaddr + var_info->core_dll_config);

	/*
	 * use_cdclp533 only applies to legacy SDCC (core_minor < 0x34); our
	 * supported targets always use the SDC4 calibration path.
	 */
	if (priv->use_cdclp533) {
		printf("%s: CDCLP533 HS400 calibration path is not implemented\n",
		       host->name);
		return -EOPNOTSUPP;
	}

	return sdhci_msm_cm_dll_sdc4_calibration(host);
}

static int msm_find_most_appropriate_phase(struct sdhci_host *host,
					   u8 *phase_table,
					   u8 total_phases)
{
	int ret;
	u8 ranges[MAX_PHASES][MAX_PHASES] = { {0}, {0} };
	u8 phases_per_row[MAX_PHASES] = { 0 };
	int row_index = 0, col_index = 0, selected_row_index = 0;
	int i, longest_range_len = 0;
	bool found = false;

	if (!total_phases || total_phases > MAX_PHASES) {
		printf("%s: Invalid argument: total_phases=%d\n",
		       host->name, total_phases);
		return -EINVAL;
	}

	for (i = 0; i < total_phases; i++) {
		ranges[row_index][col_index] = phase_table[i];
		phases_per_row[row_index] += 1;
		col_index++;

		if ((i + 1) == total_phases)
			continue;

		if (phase_table[i] + 1 != phase_table[i + 1]) {
			row_index++;
			col_index = 0;
		}
	}

	if (row_index == 0) {
		ret = phase_table[total_phases / 2];
		goto exit;
	}

	for (i = 0; i <= row_index; i++) {
		if (phases_per_row[i] > longest_range_len) {
			longest_range_len = phases_per_row[i];
			selected_row_index = i;
			found = true;
		}
	}

	if (found) {
		ret = ranges[selected_row_index][longest_range_len / 2];
	} else {
		ret = -EIO;
		printf("%s: Failed to find a valid phase\n", host->name);
	}

exit:
	return ret;
}

static bool sdhci_msm_is_tuning_needed(struct sdhci_host *host)
{
	struct mmc *mmc = host->mmc;

	return mmc->selected_mode == MMC_HS_200;
}

static int sdhci_msm_execute_tuning(struct mmc *mmc, u8 opcode)
{
	struct sdhci_host *host = mmc->priv;
	int tuning_seq_cnt = 10;
	u8 phase, tuned_phases[MAX_PHASES], tuned_phase_cnt = 0;
	int rc;
	struct udevice *dev = mmc->dev;
	struct msm_sdhc *priv = dev_get_priv(dev);

	if (!sdhci_msm_is_tuning_needed(host))
		return 0;

	/*
	 * The SDHCI core may call execute_tuning before host->clock is
	 * updated to match mmc->clock.
	 */
	if (host->clock != mmc->clock) {
		rc = sdhci_set_clock(mmc, mmc->clock);
		if (rc) {
			printf("%s: Failed to set clock for tuning\n", host->name);
			return rc;
		}
	}

	priv->tuning_done = false;

retry:
	rc = msm_init_cm_dll(host);
	if (rc) {
		printf("%s: Failed to init DLL\n", host->name);
		return rc;
	}

	phase = 0;
	tuned_phase_cnt = 0;

	do {
		rc = msm_config_cm_dll_phase(host, phase);
		if (rc) {
			printf("%s: Failed to set DLL phase %d\n",
			       host->name, phase);
			return rc;
		}

		rc = mmc_send_tuning(mmc, opcode);
		if (!rc)
			tuned_phases[tuned_phase_cnt++] = phase;
	} while (++phase < MAX_PHASES);

	if (tuned_phase_cnt) {
		if (tuned_phase_cnt == MAX_PHASES) {
			/*
			 * All phases valid is close to as bad as none valid:
			 * likely no phase is really reliable. Retry a few
			 * times rather than guessing.
			 */
			if (--tuning_seq_cnt) {
				tuned_phase_cnt = 0;
				goto retry;
			}
		}

		rc = msm_find_most_appropriate_phase(host, tuned_phases,
						     tuned_phase_cnt);
		if (rc < 0) {
			printf("%s: Failed to find appropriate phase\n",
			       host->name);
			return rc;
		}
		phase = rc;

		rc = msm_config_cm_dll_phase(host, phase);
		if (rc) {
			printf("%s: Failed to set final phase %d\n",
			       host->name, phase);
			return rc;
		}

		priv->saved_tuning_phase = phase;
	} else {
		if (--tuning_seq_cnt)
			goto retry;
		printf("%s: No tuning point found\n", host->name);
		rc = -EIO;
	}

	if (!rc)
		priv->tuning_done = true;

	return rc;
}

/*
 * Configure HC mode selection. Runs from set_control_reg(), which the
 * generic sdhci_set_ios() always calls before sdhci_set_clock() - and it is
 * sdhci_set_clock() that triggers config_dll(), which performs the SDC4
 * CM_DLL HS400 calibration. On the first HS400 transition, calibration_done
 * is still false here (it only becomes true after config_dll() runs later
 * in the same set_ios() call), so the HS400 mux-select bits are deferred
 * until calibration has actually completed - selecting HS400 in the mux
 * beforehand would switch the data path into HS400 timing while the DLL is
 * still configured for the previous mode/clock. The first write of these
 * bits happens from msm_sdhci_config_dll() via sdhci_msm_hs400_select_in()
 * once calibration succeeds.
 */
static void sdhci_msm_hc_select_mode(struct sdhci_host *host)
{
	struct mmc *mmc = host->mmc;
	struct udevice *dev = mmc->dev;
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	bool is_hs400 = mmc->selected_mode == MMC_HS_400;
	u32 config;

	config = readl(host->ioaddr + var_info->core_vendor_spec);
	config &= ~CORE_HC_MCLK_SEL_MASK;

	if (mmc->selected_mode == MMC_HS_200)
		config |= CORE_HC_MCLK_SEL_DFLT;
	else if (is_hs400)
		config |= CORE_HC_MCLK_SEL_HS400;
	else
		config |= CORE_HC_MCLK_SEL_DFLT;

	writel(config, host->ioaddr + var_info->core_vendor_spec);

	if (is_hs400) {
		if (priv->calibration_done) {
			config = readl(host->ioaddr + var_info->core_vendor_spec);
			config |= CORE_HC_SELECT_IN_HS400;
			config |= CORE_HC_SELECT_IN_EN;
			writel(config, host->ioaddr + var_info->core_vendor_spec);
		}
	} else {
		/*
		 * Matches the kernel's msm_hc_select_default(): explicitly
		 * clear these bits for every non-HS400 mode so a previous
		 * HS400 attempt never leaves them stuck set.
		 */
		if (!priv->use_cdclp533) {
			config = readl(host->ioaddr + var_info->core_vendor_spec3);
			config &= ~CORE_PWRSAVE_DLL;
			writel(config, host->ioaddr + var_info->core_vendor_spec3);
		}

		config = readl(host->ioaddr + var_info->core_vendor_spec);
		config &= ~CORE_HC_SELECT_IN_EN;
		config &= ~CORE_HC_SELECT_IN_MASK;
		writel(config, host->ioaddr + var_info->core_vendor_spec);
	}
}

/*
 * Selects HS400 in the HC_SELECT_IN mux. Must only be called immediately
 * after sdhci_msm_hs400_dll_calibration() succeeds - see the comment in
 * sdhci_msm_hc_select_mode() for why this can't be done there.
 */
static void sdhci_msm_hs400_select_in(struct sdhci_host *host)
{
	struct udevice *dev = host->mmc->dev;
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	u32 config;

	config = readl(host->ioaddr + var_info->core_vendor_spec);
	config |= CORE_HC_SELECT_IN_HS400;
	config |= CORE_HC_SELECT_IN_EN;
	writel(config, host->ioaddr + var_info->core_vendor_spec);
}

static void sdhci_msm_set_control_reg(struct sdhci_host *host)
{
	struct mmc *mmc = host->mmc;
	struct udevice *dev = mmc->dev;
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info =
		(void *)dev_get_driver_data(dev);
	u32 config;

	/*
	 * The Qualcomm SDHCI controller does not implement the generic
	 * SDHCI_CTRL_HS400 (0x5) HOST_CONTROL2 encoding that
	 * sdhci_set_uhs_timing() would write for HS400. Per the kernel
	 * driver's sdhci_msm_set_uhs_signaling(), it instead keeps
	 * HOST_CONTROL2's UHS field at SDR104 (identical to HS200) for
	 * HS400 too, relying entirely on the vendor-specific
	 * CORE_HC_MCLK_SEL/CORE_HC_SELECT_IN bits (sdhci_msm_hc_select_mode()
	 * below) to switch the data path into HS400 timing.
	 */
	sdhci_set_voltage(host);
	if (mmc->selected_mode == MMC_HS_400) {
		u32 ctrl2 = sdhci_readw(host, SDHCI_HOST_CONTROL2);

		ctrl2 &= ~SDHCI_CTRL_UHS_MASK;
		ctrl2 |= SDHCI_CTRL_UHS_SDR104;
		sdhci_writew(host, ctrl2, SDHCI_HOST_CONTROL2);
	} else {
		sdhci_set_uhs_timing(host);
	}

	sdhci_msm_hc_select_mode(host);

	/*
	 * Below 100MHz the feedback clock must be provided without the DLL,
	 * so tuning can be skipped.
	 */
	if (mmc->clock && mmc->clock <= CORE_FREQ_100MHZ) {
		if (mmc->selected_mode == MMC_HS_200 ||
		    mmc->selected_mode == MMC_HS_400) {
			config = readl(host->ioaddr + var_info->core_dll_config);
			config |= CORE_DLL_RST;
			writel(config, host->ioaddr + var_info->core_dll_config);

			config = readl(host->ioaddr + var_info->core_dll_config);
			config |= CORE_DLL_PDN;
			writel(config, host->ioaddr + var_info->core_dll_config);

			/*
			 * Calibration must be redone once the clock is set
			 * back to HS400 speed, matching the kernel's
			 * sdhci_msm_set_uhs_signaling().
			 */
			priv->calibration_done = false;
		}
	}
}

static int msm_sdhci_config_dll(struct sdhci_host *host, u32 clock, bool enable)
{
	struct udevice *dev = mmc_to_dev(host->mmc);
	struct mmc *mmc = host->mmc;
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info = (void *)dev_get_driver_data(dev);
	u32 config;

	if (clock == 0)
		return 0;

	if (enable && clock < MHZ(100)) {
		/*
		 * DLL is not required for clock <= 100MHz
		 * Thus, make sure DLL is disabled when not required
		 */
		config = readl(host->ioaddr + var_info->core_dll_config);
		config |= CORE_DLL_RST;
		writel(config, host->ioaddr + var_info->core_dll_config);

		config = readl(host->ioaddr + var_info->core_dll_config);
		config |= CORE_DLL_PDN;
		writel(config, host->ioaddr + var_info->core_dll_config);
	}

	/*
	 * HS400 requires a dedicated DDR/SDC4 DLL calibration step,
	 * performed once per calibration cycle after the divider has been
	 * programmed and the clock is running above 100MHz - the analogue
	 * of the kernel's sdhci_msm_set_uhs_signaling() -> sdhci_msm_hs400()
	 * trigger, which runs right after the SDCLK divider is set.
	 */
	if (enable && clock > MHZ(100) &&
	    mmc->selected_mode == MMC_HS_400 && priv->tuning_done &&
	    !priv->calibration_done) {
		int ret = sdhci_msm_hs400_dll_calibration(host);

		if (!ret) {
			priv->calibration_done = true;
			/*
			 * Only now that calibration has actually succeeded
			 * is it safe to switch the data-path mux into HS400
			 * timing - see the comment in
			 * sdhci_msm_hc_select_mode().
			 */
			sdhci_msm_hs400_select_in(host);
		} else {
			printf("%s: Failed to calibrate DLL for HS400 mode (%d)\n",
			       host->name, ret);
			return ret;
		}
	}

	return 0;
}

struct sdhci_ops msm_sdhci_ops = {
	.config_dll = &msm_sdhci_config_dll,
	.set_control_reg = &sdhci_msm_set_control_reg,
	.platform_execute_tuning = &sdhci_msm_execute_tuning,
};

static int msm_sdc_probe(struct udevice *dev)
{
	struct mmc_uclass_priv *upriv = dev_get_uclass_priv(dev);
	struct msm_sdhc_plat *plat = dev_get_plat(dev);
	struct msm_sdhc *prv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info;
	struct sdhci_host *host = &prv->host;
	u32 core_version, core_minor, core_major;
	struct reset_ctl bcr_rst;
	struct blk_desc *bdesc;
	u32 caps;
	int ret;

	ret = reset_get_by_index(dev, 0, &bcr_rst);
	if (!ret) {
		reset_assert(&bcr_rst);
		udelay(200);
		reset_deassert(&bcr_rst);
		udelay(200);
	}

	host->quirks = SDHCI_QUIRK_WAIT_SEND_CMD | SDHCI_QUIRK_BROKEN_R1B;

	host->max_clk = 0;

	/* Init clocks */
	ret = msm_sdc_clk_init(dev);
	if (ret)
		return ret;

	/* Get the vqmmc regulator and enable it if available */
	device_get_supply_regulator(dev, "vqmmc-supply", &prv->vqmmc);
	if (prv->vqmmc) {
		ret = regulator_set_enable_if_allowed(prv->vqmmc, true);
		if (ret) {
			printf("Failed to enable the VQMMC regulator\n");
			return ret;
		}
	}

	var_info = (void *)dev_get_driver_data(dev);
	if (!var_info->mci_removed) {
		ret = msm_sdc_mci_init(prv);
		if (ret)
			return ret;
	}

	if (!var_info->mci_removed)
		core_version = readl(prv->base + SDCC_MCI_VERSION);
	else
		core_version = readl(host->ioaddr + SDCC_V5_VERSION);

	core_major = (core_version & SDCC_VERSION_MAJOR_MASK);
	core_major >>= SDCC_VERSION_MAJOR_SHIFT;

	core_minor = core_version & SDCC_VERSION_MINOR_MASK;

	log_debug("SDCC version %d.%d\n", core_major, core_minor);

	/*
	 * Match the kernel driver's version-gated feature detection so the
	 * DLL init sequence matches what this SDCC IP revision needs.
	 */
	if (core_major == 1 && core_minor >= 0x42)
		prv->use_14lpp_dll_reset = true;

	if (core_major == 1 && core_minor >= 0x71)
		prv->uses_tassadar_dll = true;

	if (core_major == 1 && core_minor < 0x34)
		prv->use_cdclp533 = true;

	if (core_major == 1 && core_minor >= 0x49)
		prv->updated_ddr_cfg = true;

	dev_read_u32(dev, "qcom,dll-config", &prv->dll_config);

	if (dev_read_u32(dev, "qcom,ddr-config", &prv->ddr_config))
		prv->ddr_config = DDR_CONFIG_POR_VAL;

	/*
	 * Support for some capabilities is not advertised by newer
	 * controller versions and must be explicitly enabled.
	 */
	if (core_major >= 1 && core_minor != 0x11 && core_minor != 0x12) {
		caps = readl(host->ioaddr + SDHCI_CAPABILITIES);
		caps |= SDHCI_CAN_VDD_300 | SDHCI_CAN_DO_8BIT;
		writel(caps, host->ioaddr + var_info->core_vendor_spec_capabilities0);
	}

	ret = mmc_of_parse(dev, &plat->cfg);
	if (ret)
		return ret;

	if (plat->cfg.host_caps & MMC_CAP_NONREMOVABLE) {
		bdesc = mmc_get_blk_desc(&plat->mmc);
		if (!bdesc)
			return -ENODEV;
		bdesc->removable = 0;
	}

	host->mmc = &plat->mmc;
	host->mmc->dev = dev;
	host->ops = &msm_sdhci_ops;
	ret = sdhci_setup_cfg(&plat->cfg, host, 0, 0);
	if (ret)
		return ret;
	host->mmc->priv = &prv->host;
	upriv->mmc = host->mmc;

	return sdhci_probe(dev);
}

static int msm_sdc_remove(struct udevice *dev)
{
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info;

	var_info = (void *)dev_get_driver_data(dev);

	/* Disable host-controller mode */
	if (!var_info->mci_removed && priv->base)
		writel(0, priv->base + SDCC_MCI_HC_MODE);

	clk_release_bulk(&priv->clks);

	return 0;
}

static int msm_of_to_plat(struct udevice *dev)
{
	struct msm_sdhc *priv = dev_get_priv(dev);
	const struct msm_sdhc_variant_info *var_info;
	struct sdhci_host *host = &priv->host;
	int ret;

	var_info = (void*)dev_get_driver_data(dev);

	host->name = strdup(dev->name);
	host->ioaddr = dev_read_addr_ptr(dev);
	ret = dev_read_u32(dev, "bus-width", &host->bus_width);
	if (ret)
		host->bus_width = 4;
	ret = dev_read_u32(dev, "index", &host->index);
	if (ret)
		host->index = 0;
	priv->base = dev_read_addr_index_ptr(dev, 1);

	if (!host->ioaddr)
		return -EINVAL;

	if (!var_info->mci_removed && !priv->base) {
		printf("msm_sdhci: MCI base address not found\n");
		return -EINVAL;
	}

	return 0;
}

static int msm_sdc_bind(struct udevice *dev)
{
	struct msm_sdhc_plat *plat = dev_get_plat(dev);

	return sdhci_bind(dev, &plat->mmc, &plat->cfg);
}

static const struct msm_sdhc_variant_info msm_sdhc_mci_var = {
	.mci_removed = false,

	.core_dll_config = 0x100,
	.core_dll_status = 0x108,
	.core_dll_config_2 = 0x1b4,
	.core_vendor_spec = 0x10c,
	.core_vendor_spec_capabilities0 = 0x11c,

	.core_ddr_200_cfg = 0x184,
	.core_vendor_spec3 = 0x1b0,
	.core_ddr_config_old = 0x1b8,
	.core_ddr_config = 0x1bc,
};

static const struct msm_sdhc_variant_info msm_sdhc_v5_var = {
	.mci_removed = true,

	.core_dll_config = 0x200,
	.core_dll_status = 0x208,
	.core_dll_config_2 = 0x254,
	.core_dll_config_3 = 0x258,
	.core_dll_usr_ctl = 0x388,
	.core_vendor_spec = 0x20c,
	.core_vendor_spec_capabilities0 = 0x21c,

	.core_ddr_200_cfg = 0x224,
	.core_vendor_spec3 = 0x250,
	.core_ddr_config = 0x25c,
	/*
	 * core_ddr_config_old not present on V5 - updated_ddr_cfg is
	 * effectively always true here since V5 implies core_minor >= 0x49
	 */
};

static const struct udevice_id msm_mmc_ids[] = {
	{ .compatible = "qcom,sdhci-msm-v4", .data = (ulong)&msm_sdhc_mci_var },
	{ .compatible = "qcom,sdhci-msm-v5", .data = (ulong)&msm_sdhc_v5_var },
	{ }
};

U_BOOT_DRIVER(msm_sdc_drv) = {
	.name		= "msm_sdc",
	.id		= UCLASS_MMC,
	.of_match	= msm_mmc_ids,
	.of_to_plat = msm_of_to_plat,
	.ops		= &sdhci_ops,
	.bind		= msm_sdc_bind,
	.probe		= msm_sdc_probe,
	.remove		= msm_sdc_remove,
	.priv_auto	= sizeof(struct msm_sdhc),
	.plat_auto	= sizeof(struct msm_sdhc_plat),
};

// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm RPM power domains (CX, MX and friends) voted over SMD RPM.
 *
 * Based on the Linux rpmpd driver.
 */

#include <dm.h>
#include <power-domain-uclass.h>
#include <dm/device_compat.h>
#include <dt-bindings/power/qcom-rpmpd.h>
#include <linux/errno.h>
#include <soc/qcom/smd-rpm.h>

#define RPMPD_RWCX	0x78637772
#define RPMPD_RWMX	0x786d7772
#define RPMPD_RWLC	0x636c7772
#define RPMPD_RWLM	0x6d6c7772

#define KEY_ENABLE		0x6e657773 /* swen */
#define KEY_FLOOR_LEVEL		0x6c6676   /* vfl */
#define KEY_LEVEL		0x6c766c76 /* vlvl */

struct rpmpd_req {
	__le32 key;
	__le32 nbytes;
	__le32 value;
};

struct rpmpd {
	u32 res_type;
	u32 res_id;
	u32 key;
	bool active_only;
};

struct rpmpd_desc {
	const struct rpmpd *rpmpds;
	unsigned int num_pds;
	unsigned int max_state;
};

struct rpmpd_priv {
	const struct rpmpd_desc *desc;
	unsigned long enabled;
};

#define RPMPD(_type, _key)		{ .res_type = _type, .key = _key }
#define RPMPD_AO(_type, _key)		{ .res_type = _type, .key = _key, \
					  .active_only = true }

static const struct rpmpd qcm2290_rpmpds[] = {
	[QCM2290_VDDCX] =	RPMPD(RPMPD_RWCX, KEY_LEVEL),
	[QCM2290_VDDCX_AO] =	RPMPD_AO(RPMPD_RWCX, KEY_LEVEL),
	[QCM2290_VDDCX_VFL] =	RPMPD(RPMPD_RWCX, KEY_FLOOR_LEVEL),
	[QCM2290_VDDMX] =	RPMPD(RPMPD_RWMX, KEY_LEVEL),
	[QCM2290_VDDMX_AO] =	RPMPD_AO(RPMPD_RWMX, KEY_LEVEL),
	[QCM2290_VDDMX_VFL] =	RPMPD(RPMPD_RWMX, KEY_FLOOR_LEVEL),
	[QCM2290_VDD_LPI_CX] =	RPMPD(RPMPD_RWLC, KEY_LEVEL),
	[QCM2290_VDD_LPI_MX] =	RPMPD(RPMPD_RWLM, KEY_LEVEL),
};

static const struct rpmpd_desc qcm2290_desc = {
	.rpmpds = qcm2290_rpmpds,
	.num_pds = ARRAY_SIZE(qcm2290_rpmpds),
	.max_state = RPM_SMD_LEVEL_TURBO_NO_CPR,
};

static const struct rpmpd sm6125_rpmpds[] = {
	[RPMPD_VDDCX] =		RPMPD(RPMPD_RWCX, KEY_LEVEL),
	[RPMPD_VDDCX_AO] =	RPMPD_AO(RPMPD_RWCX, KEY_LEVEL),
	[RPMPD_VDDCX_VFL] =	RPMPD(RPMPD_RWCX, KEY_FLOOR_LEVEL),
	[RPMPD_VDDMX] =		RPMPD(RPMPD_RWMX, KEY_LEVEL),
	[RPMPD_VDDMX_AO] =	RPMPD_AO(RPMPD_RWMX, KEY_LEVEL),
	[RPMPD_VDDMX_VFL] =	RPMPD(RPMPD_RWMX, KEY_FLOOR_LEVEL),
};

static const struct rpmpd_desc shikra_desc = {
	.rpmpds = sm6125_rpmpds,
	.num_pds = ARRAY_SIZE(sm6125_rpmpds),
	.max_state = RPM_SMD_LEVEL_TURBO_NO_CPR,
};

static int rpmpd_send(struct udevice *dev, const struct rpmpd *pd, int state,
		      u32 key, u32 value)
{
	struct rpmpd_req req = {
		.key = cpu_to_le32(key),
		.nbytes = cpu_to_le32(sizeof(u32)),
		.value = cpu_to_le32(value),
	};

	return qcom_rpm_smd_write(dev_get_parent(dev), state, pd->res_type,
				  pd->res_id, &req, sizeof(req));
}

static int rpmpd_request(struct power_domain *power_domain)
{
	struct rpmpd_priv *priv = dev_get_priv(power_domain->dev);

	if (power_domain->id >= priv->desc->num_pds ||
	    !priv->desc->rpmpds[power_domain->id].res_type)
		return -EINVAL;

	return 0;
}

/*
 * Vote the highest level, as Linux does for each domain until its
 * sync_state: U-Boot has no performance states, and the kernel replaces
 * the vote once its own driver probes.
 */
static int rpmpd_on(struct power_domain *power_domain)
{
	struct udevice *dev = power_domain->dev;
	struct rpmpd_priv *priv = dev_get_priv(dev);
	const struct rpmpd *pd = &priv->desc->rpmpds[power_domain->id];
	unsigned int level = priv->desc->max_state;
	int ret;

	if (priv->enabled & BIT(power_domain->id))
		return 0;

	ret = rpmpd_send(dev, pd, QCOM_SMD_RPM_ACTIVE_STATE, KEY_ENABLE, 1);
	if (ret)
		goto err;

	ret = rpmpd_send(dev, pd, QCOM_SMD_RPM_ACTIVE_STATE, pd->key, level);
	if (ret)
		goto err;

	ret = rpmpd_send(dev, pd, QCOM_SMD_RPM_SLEEP_STATE, pd->key,
			 pd->active_only ? 0 : level);
	if (ret)
		goto err;

	priv->enabled |= BIT(power_domain->id);

	return 0;

err:
	dev_err(dev, "failed to enable domain %lu: %d\n", power_domain->id, ret);
	return ret;
}

/*
 * Keep the vote: the RPM holds the last one until the kernel's driver
 * sends its own, and dropping CX or MX in between would take the buses
 * down under the booting kernel.
 */
static int rpmpd_off(struct power_domain *power_domain)
{
	return 0;
}

static int rpmpd_probe(struct udevice *dev)
{
	struct rpmpd_priv *priv = dev_get_priv(dev);

	priv->desc = (const struct rpmpd_desc *)dev_get_driver_data(dev);

	return 0;
}

static const struct power_domain_ops rpmpd_ops = {
	.request = rpmpd_request,
	.on = rpmpd_on,
	.off = rpmpd_off,
};

static const struct udevice_id rpmpd_ids[] = {
	{ .compatible = "qcom,qcm2290-rpmpd", .data = (ulong)&qcm2290_desc },
	{ .compatible = "qcom,shikra-rpmpd", .data = (ulong)&shikra_desc },
	{ }
};

U_BOOT_DRIVER(qcom_rpmpd) = {
	.name = "qcom_rpmpd",
	.id = UCLASS_POWER_DOMAIN,
	.of_match = rpmpd_ids,
	.probe = rpmpd_probe,
	.priv_auto = sizeof(struct rpmpd_priv),
	.ops = &rpmpd_ops,
	.flags = DM_FLAG_DEFAULT_PD_CTRL_OFF,
};

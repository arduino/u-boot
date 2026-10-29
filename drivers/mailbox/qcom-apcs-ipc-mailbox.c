// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm APCS IPC mailbox: a write-only doorbell register in the APCS
 * global (or APSS shared) block, one bit per remote processor and channel.
 *
 * Based on the Linux qcom-apcs-ipc-mailbox driver.
 */

#include <dm.h>
#include <mailbox-uclass.h>
#include <asm/io.h>
#include <linux/bitops.h>

struct qcom_apcs_ipc {
	void __iomem *reg;
};

static int qcom_apcs_ipc_request(struct mbox_chan *chan)
{
	return chan->id < 32 ? 0 : -EINVAL;
}

static int qcom_apcs_ipc_send(struct mbox_chan *chan, const void *data)
{
	struct qcom_apcs_ipc *apcs = dev_get_priv(chan->dev);

	writel(BIT(chan->id), apcs->reg);

	return 0;
}

static int qcom_apcs_ipc_probe(struct udevice *dev)
{
	struct qcom_apcs_ipc *apcs = dev_get_priv(dev);
	fdt_addr_t base;

	base = dev_read_addr(dev);
	if (base == FDT_ADDR_T_NONE)
		return -EINVAL;

	apcs->reg = (void __iomem *)(base + dev_get_driver_data(dev));

	return 0;
}

static const struct mbox_ops qcom_apcs_ipc_ops = {
	.request = qcom_apcs_ipc_request,
	.send = qcom_apcs_ipc_send,
};

/* .data is the offset of the IPC register in the block */
static const struct udevice_id qcom_apcs_ipc_ids[] = {
	{ .compatible = "qcom,qcm2290-apcs-hmss-global", .data = 8 },
	{ .compatible = "qcom,sdm845-apss-shared", .data = 12 },
	{ }
};

U_BOOT_DRIVER(qcom_apcs_ipc) = {
	.name = "qcom_apcs_ipc",
	.id = UCLASS_MAILBOX,
	.of_match = qcom_apcs_ipc_ids,
	.probe = qcom_apcs_ipc_probe,
	.priv_auto = sizeof(struct qcom_apcs_ipc),
	.ops = &qcom_apcs_ipc_ops,
	.flags = DM_FLAG_DEFAULT_PD_CTRL_OFF,
};

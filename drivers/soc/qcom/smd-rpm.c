// SPDX-License-Identifier: GPL-2.0-only
/*
 * Qualcomm RPM requests over the GLINK RPM transport.
 *
 * The RPM exposes two FIFOs in its message RAM, listed in a table of
 * contents at the end of that RAM, and is interrupted through the APCS IPC
 * doorbell. GLINK runs on top of them with a single channel,
 * "rpm_requests", which carries the resource requests. U-Boot has no
 * interrupts, so the receive FIFO is polled.
 *
 * Based on the Linux qcom_glink_rpm, qcom_glink_native and smd-rpm drivers.
 */

#include <dm.h>
#include <mailbox.h>
#include <time.h>
#include <asm/io.h>
#include <asm/unaligned.h>
#include <dm/device_compat.h>
#include <dm/ofnode.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <soc/qcom/smd-rpm.h>

#define RPM_TOC_SIZE			256
#define RPM_TOC_MAGIC			0x67727430 /* grt0 */
#define RPM_TOC_HDR_SIZE		8
#define RPM_TOC_ENTRY_SIZE		12
#define RPM_TX_FIFO_ID			0x61703272 /* ap2r */
#define RPM_RX_FIFO_ID			0x72326170 /* r2ap */

#define GLINK_VERSION_1			1

#define GLINK_CMD_VERSION		0
#define GLINK_CMD_VERSION_ACK		1
#define GLINK_CMD_OPEN			2
#define GLINK_CMD_CLOSE			3
#define GLINK_CMD_OPEN_ACK		4
#define GLINK_CMD_INTENT		5
#define GLINK_CMD_RX_DONE		6
#define GLINK_CMD_RX_INTENT_REQ		7
#define GLINK_CMD_RX_INTENT_REQ_ACK	8
#define GLINK_CMD_TX_DATA		9
#define GLINK_CMD_CLOSE_ACK		11
#define GLINK_CMD_TX_DATA_CONT		12
#define GLINK_CMD_READ_NOTIF		13
#define GLINK_CMD_RX_DONE_W_REUSE	14
#define GLINK_CMD_SIGNALS		15

#define GLINK_NAME_SIZE			32
#define GLINK_INTENT_PAIR_SIZE		8

#define RPM_CHANNEL_NAME		"rpm_requests"
#define RPM_CHANNEL_LCID		1
#define SSR_CHANNEL_NAME		"glink_ssr"
#define SSR_CHANNEL_LCID		2
#define GLINK_SSR_DO_CLEANUP		0
#define GLINK_SSR_APSS			"apss"
#define RPM_TIMEOUT_US			(5 * 1000 * 1000)
#define RPM_RESET_TIMEOUT_US		(100 * 1000)
/* Packets to and from the RPM may not exceed 256 bytes */
#define RPM_MAX_MSG_SIZE		256

#define RPM_SERVICE_TYPE_REQUEST	0x00716572 /* "req\0" */
#define RPM_MSG_TYPE_ERR		0x00727265 /* "err\0" */

struct glink_msg {
	__le16 cmd;
	__le16 param1;
	__le32 param2;
} __packed;

struct glink_data_hdr {
	struct glink_msg msg;
	__le32 chunk_size;
	__le32 left_size;
} __packed;

struct qcom_rpm_header {
	__le32 service_type;
	__le32 length;
};

struct qcom_rpm_request {
	__le32 msg_id;
	__le32 flags;
	__le32 type;
	__le32 id;
	__le32 data_len;
};

struct glink_rpm_pipe {
	void __iomem *tail;
	void __iomem *head;
	void __iomem *fifo;
	u32 length;
};

struct glink_rpm {
	struct glink_rpm_pipe rx;
	struct glink_rpm_pipe tx;
	struct mbox_chan mbox;
	u32 features;
	u16 rcid;
	bool remote_open;
	bool open_acked;
	u16 ssr_rcid;
	bool ssr_open_acked;
	bool msg_ready;
	u32 msg_id;
	u32 msg_len;
	u8 msg[RPM_MAX_MSG_SIZE];
};

/* The message RAM only takes 32-bit accesses. */
static u32 glink_rpm_rx_avail(struct glink_rpm_pipe *pipe)
{
	u32 head = readl(pipe->head);
	u32 tail = readl(pipe->tail);

	return head < tail ? pipe->length - tail + head : head - tail;
}

static void glink_rpm_rx_peek(struct glink_rpm_pipe *pipe, u32 offset,
			      void *data, u32 count)
{
	u32 pos = (readl(pipe->tail) + offset) % pipe->length;
	u8 *buf = data;
	u32 word, n;

	while (count) {
		word = readl(pipe->fifo + pos);
		n = min_t(u32, count, sizeof(word));
		memcpy(buf, &word, n);
		buf += n;
		count -= n;
		pos += sizeof(word);
		if (pos >= pipe->length)
			pos = 0;
	}
}

static void glink_rpm_rx_advance(struct glink_rpm_pipe *pipe, u32 count)
{
	writel((readl(pipe->tail) + count) % pipe->length, pipe->tail);
}

static u32 glink_rpm_tx_avail(struct glink_rpm_pipe *pipe)
{
	u32 head = readl(pipe->head);
	u32 tail = readl(pipe->tail);

	return tail <= head ? pipe->length - head + tail : tail - head;
}

static int glink_rpm_tx(struct glink_rpm *rpm, const void *hdr, u32 hlen,
			const void *data, u32 dlen)
{
	struct glink_rpm_pipe *pipe = &rpm->tx;
	u8 buf[sizeof(struct glink_data_hdr) + RPM_MAX_MSG_SIZE];
	u32 len = ALIGN(hlen + dlen, 8);
	ulong start = timer_get_us();
	u32 head, word, i;

	if (len > sizeof(buf) || len >= pipe->length)
		return -EINVAL;

	memset(buf, 0, len);
	memcpy(buf, hdr, hlen);
	if (dlen)
		memcpy(buf + hlen, data, dlen);

	/* Never fill the FIFO completely: head == tail means empty */
	while (glink_rpm_tx_avail(pipe) <= len) {
		if (timer_get_us() - start > RPM_TIMEOUT_US)
			return -ETIMEDOUT;
		udelay(10);
	}

	head = readl(pipe->head);
	for (i = 0; i < len; i += sizeof(word)) {
		memcpy(&word, buf + i, sizeof(word));
		writel(word, pipe->fifo + head);
		head += sizeof(word);
		if (head >= pipe->length)
			head = 0;
	}
	writel(head, pipe->head);

	return mbox_send(&rpm->mbox, NULL);
}

static int glink_rpm_send_cmd(struct glink_rpm *rpm, u16 cmd, u16 param1,
			      u32 param2)
{
	struct glink_msg msg = {
		.cmd = cpu_to_le16(cmd),
		.param1 = cpu_to_le16(param1),
		.param2 = cpu_to_le32(param2),
	};

	return glink_rpm_tx(rpm, &msg, sizeof(msg), NULL, 0);
}

static int glink_rpm_rx_data(struct udevice *dev, struct glink_rpm *rpm,
			     u32 avail)
{
	struct glink_data_hdr hdr;
	u32 chunk, left;
	u16 cmd, rcid;

	if (avail < sizeof(hdr))
		return -EAGAIN;

	glink_rpm_rx_peek(&rpm->rx, 0, &hdr, sizeof(hdr));
	cmd = le16_to_cpu(hdr.msg.cmd);
	rcid = le16_to_cpu(hdr.msg.param1);
	chunk = le32_to_cpu(hdr.chunk_size);
	left = le32_to_cpu(hdr.left_size);

	if (avail < sizeof(hdr) + chunk)
		return -EAGAIN;

	if (rpm->rcid && rcid == rpm->rcid) {
		if (cmd == GLINK_CMD_TX_DATA)
			rpm->msg_len = 0;

		if (rpm->msg_len + chunk > sizeof(rpm->msg)) {
			dev_err(dev, "RPM message too long, dropped\n");
			rpm->msg_len = 0;
		} else {
			glink_rpm_rx_peek(&rpm->rx, sizeof(hdr),
					  rpm->msg + rpm->msg_len, chunk);
			rpm->msg_len += chunk;
			if (!left)
				rpm->msg_ready = true;
		}
	}

	glink_rpm_rx_advance(&rpm->rx, ALIGN(sizeof(hdr) + chunk, 8));

	return 0;
}

/* Handle everything the RPM has sent so far */
static int glink_rpm_rx(struct udevice *dev, struct glink_rpm *rpm)
{
	char name[GLINK_NAME_SIZE];
	struct glink_msg msg;
	u32 avail, extra, param2;
	u16 cmd, param1;
	int ret;

	for (;;) {
		avail = glink_rpm_rx_avail(&rpm->rx);
		if (avail < sizeof(msg))
			return 0;

		glink_rpm_rx_peek(&rpm->rx, 0, &msg, sizeof(msg));
		cmd = le16_to_cpu(msg.cmd);
		param1 = le16_to_cpu(msg.param1);
		param2 = le32_to_cpu(msg.param2);

		switch (cmd) {
		case GLINK_CMD_VERSION:
			glink_rpm_rx_advance(&rpm->rx, sizeof(msg));
			if (param1 == GLINK_VERSION_1)
				rpm->features &= param2;
			if (param1)
				glink_rpm_send_cmd(rpm, GLINK_CMD_VERSION_ACK,
						   GLINK_VERSION_1,
						   rpm->features);
			break;
		case GLINK_CMD_VERSION_ACK:
			glink_rpm_rx_advance(&rpm->rx, sizeof(msg));
			if (!param1 || (param1 == GLINK_VERSION_1 &&
					param2 == rpm->features))
				break;
			if (param1 == GLINK_VERSION_1)
				rpm->features &= param2;
			glink_rpm_send_cmd(rpm, GLINK_CMD_VERSION,
					   GLINK_VERSION_1, rpm->features);
			break;
		case GLINK_CMD_OPEN:
			/* the upper 16 bits of param2 are the priority */
			extra = ALIGN(param2 & 0xffff, 8);
			if (avail < sizeof(msg) + extra)
				return 0;
			memset(name, 0, sizeof(name));
			glink_rpm_rx_peek(&rpm->rx, sizeof(msg), name,
					  min_t(u32, extra, sizeof(name) - 1));
			glink_rpm_rx_advance(&rpm->rx, sizeof(msg) + extra);
			if (!strcmp(name, RPM_CHANNEL_NAME)) {
				rpm->rcid = param1;
				rpm->remote_open = true;
			} else if (!strcmp(name, SSR_CHANNEL_NAME)) {
				rpm->ssr_rcid = param1;
			} else {
				dev_dbg(dev, "ignoring channel %s\n", name);
			}
			break;
		case GLINK_CMD_OPEN_ACK:
			glink_rpm_rx_advance(&rpm->rx, sizeof(msg));
			if (param1 == RPM_CHANNEL_LCID)
				rpm->open_acked = true;
			if (param1 == SSR_CHANNEL_LCID)
				rpm->ssr_open_acked = true;
			break;
		case GLINK_CMD_CLOSE:
			glink_rpm_rx_advance(&rpm->rx, sizeof(msg));
			if (rpm->rcid && param1 == rpm->rcid) {
				rpm->remote_open = false;
				rpm->open_acked = false;
				rpm->rcid = 0;
			}
			glink_rpm_send_cmd(rpm, GLINK_CMD_CLOSE_ACK, param1, 0);
			break;
		case GLINK_CMD_INTENT:
			extra = param2 * GLINK_INTENT_PAIR_SIZE;
			if (avail < sizeof(msg) + extra)
				return 0;
			glink_rpm_rx_advance(&rpm->rx,
					     ALIGN(sizeof(msg) + extra, 8));
			break;
		case GLINK_CMD_READ_NOTIF:
			glink_rpm_rx_advance(&rpm->rx, sizeof(msg));
			mbox_send(&rpm->mbox, NULL);
			break;
		case GLINK_CMD_CLOSE_ACK:
		case GLINK_CMD_RX_DONE:
		case GLINK_CMD_RX_DONE_W_REUSE:
		case GLINK_CMD_RX_INTENT_REQ:
		case GLINK_CMD_RX_INTENT_REQ_ACK:
		case GLINK_CMD_SIGNALS:
			glink_rpm_rx_advance(&rpm->rx, sizeof(msg));
			break;
		case GLINK_CMD_TX_DATA:
		case GLINK_CMD_TX_DATA_CONT:
			ret = glink_rpm_rx_data(dev, rpm, avail);
			if (ret == -EAGAIN)
				return 0;
			break;
		default:
			dev_err(dev, "unhandled GLINK command %u\n", cmd);
			return -EPROTO;
		}
	}
}

static int glink_rpm_wait(struct udevice *dev, struct glink_rpm *rpm,
			  const bool *done)
{
	ulong start = timer_get_us();
	int ret;

	for (;;) {
		ret = glink_rpm_rx(dev, rpm);
		if (ret)
			return ret;
		if (*done)
			return 0;
		if (timer_get_us() - start > RPM_TIMEOUT_US)
			return -ETIMEDOUT;
		udelay(10);
	}
}

static int glink_rpm_parse_toc(struct udevice *dev, struct glink_rpm *rpm,
			       void __iomem *msg_ram, u32 size)
{
	void __iomem *toc = msg_ram + size - RPM_TOC_SIZE;
	struct glink_rpm_pipe *pipe;
	u32 count, id, offset, len, i;

	if (readl(toc) != RPM_TOC_MAGIC) {
		dev_err(dev, "RPM TOC has invalid magic\n");
		return -EINVAL;
	}

	count = readl(toc + 4);
	if (count > (RPM_TOC_SIZE - RPM_TOC_HDR_SIZE) / RPM_TOC_ENTRY_SIZE) {
		dev_err(dev, "invalid number of RPM TOC entries\n");
		return -EINVAL;
	}

	for (i = 0; i < count; i++) {
		void __iomem *entry = toc + RPM_TOC_HDR_SIZE +
				      i * RPM_TOC_ENTRY_SIZE;

		id = readl(entry);
		offset = readl(entry + 4);
		len = readl(entry + 8);

		if (offset > size || len > size - offset) {
			dev_err(dev, "RPM TOC entry with invalid size\n");
			continue;
		}

		if (id == RPM_RX_FIFO_ID)
			pipe = &rpm->rx;
		else if (id == RPM_TX_FIFO_ID)
			pipe = &rpm->tx;
		else
			continue;

		/* tail and head indices, then the FIFO itself */
		pipe->length = len;
		pipe->tail = msg_ram + offset;
		pipe->head = msg_ram + offset + 4;
		pipe->fifo = msg_ram + offset + 8;
	}

	if (!rpm->rx.fifo || !rpm->tx.fifo) {
		dev_err(dev, "RPM TOC has no rx/tx FIFO\n");
		return -EINVAL;
	}

	return 0;
}

static int glink_rpm_probe(struct udevice *dev)
{
	struct glink_rpm *rpm = dev_get_priv(dev);
	fdt_addr_t base;
	fdt_size_t size;
	ofnode node;
	int ret;

	node = ofnode_parse_phandle(dev_ofnode(dev), "qcom,rpm-msg-ram", 0);
	if (!ofnode_valid(node))
		return -EINVAL;

	base = ofnode_get_addr_size_index(node, 0, &size);
	if (base == FDT_ADDR_T_NONE || size < RPM_TOC_SIZE)
		return -EINVAL;

	ret = glink_rpm_parse_toc(dev, rpm, (void __iomem *)base, size);
	if (ret)
		return ret;

	ret = mbox_get_by_index(dev, 0, &rpm->mbox);
	if (ret) {
		dev_err(dev, "failed to get the IPC mailbox: %d\n", ret);
		return ret;
	}

	writel(0, rpm->tx.head);
	writel(0, rpm->rx.tail);

	return glink_rpm_send_cmd(rpm, GLINK_CMD_VERSION, GLINK_VERSION_1,
				  rpm->features);
}

static const struct udevice_id glink_rpm_ids[] = {
	{ .compatible = "qcom,glink-rpm" },
	{ }
};

U_BOOT_DRIVER(qcom_glink_rpm) = {
	.name = "qcom_glink_rpm",
	.id = UCLASS_NOP,
	.of_match = glink_rpm_ids,
	.bind = dm_scan_fdt_dev,
	.probe = glink_rpm_probe,
	.priv_auto = sizeof(struct glink_rpm),
	.flags = DM_FLAG_DEFAULT_PD_CTRL_OFF,
};

static int smd_rpm_parse_ack(struct udevice *dev, const u8 *data, u32 len)
{
	u32 type, mlen, hdr_len, off, end;
	char err[32];
	int status = 0;

	if (len < sizeof(struct qcom_rpm_header))
		return -EINVAL;

	hdr_len = get_unaligned_le32(data + 4);
	if (get_unaligned_le32(data) != RPM_SERVICE_TYPE_REQUEST ||
	    hdr_len > len - sizeof(struct qcom_rpm_header)) {
		dev_err(dev, "invalid RPM response\n");
		return -EINVAL;
	}

	off = sizeof(struct qcom_rpm_header);
	end = off + hdr_len;
	while (off + 8 <= end) {
		type = get_unaligned_le32(data + off);
		mlen = get_unaligned_le32(data + off + 4);
		if (mlen > end - off - 8)
			break;

		if (type == RPM_MSG_TYPE_ERR) {
			memset(err, 0, sizeof(err));
			memcpy(err, data + off + 8,
			       min_t(u32, mlen, sizeof(err) - 1));
			dev_dbg(dev, "RPM error: %s\n", err);
			status = strcmp(err, "resource does not exist") ?
				 -EINVAL : -ENXIO;
		}

		off = ALIGN(off + 8 + mlen, 4);
	}

	return status;
}

static int glink_rpm_send_data(struct glink_rpm *rpm, u16 lcid,
			       const void *data, u32 len)
{
	struct glink_data_hdr hdr;

	hdr.msg.cmd = cpu_to_le16(GLINK_CMD_TX_DATA);
	hdr.msg.param1 = cpu_to_le16(lcid);
	hdr.msg.param2 = 0;
	hdr.chunk_size = cpu_to_le32(len);
	hdr.left_size = 0;

	rpm->msg_ready = false;

	return glink_rpm_tx(rpm, &hdr, sizeof(hdr), data, len);
}

/* Acknowledge the RPM's open of a channel and open our end of it */
static int glink_rpm_open(struct udevice *edge, struct glink_rpm *rpm,
			  u16 rcid, u16 lcid, const char *name,
			  const bool *acked)
{
	struct {
		struct glink_msg msg;
		char name[GLINK_NAME_SIZE];
	} req;
	u32 name_len = strlen(name) + 1;
	int ret;

	ret = glink_rpm_send_cmd(rpm, GLINK_CMD_OPEN_ACK, rcid, 0);
	if (ret)
		return ret;

	memset(&req, 0, sizeof(req));
	req.msg.cmd = cpu_to_le16(GLINK_CMD_OPEN);
	req.msg.param1 = cpu_to_le16(lcid);
	req.msg.param2 = cpu_to_le32(name_len);
	strlcpy(req.name, name, sizeof(req.name));

	ret = glink_rpm_tx(rpm, &req, ALIGN(sizeof(req.msg) + name_len, 8),
			   NULL, 0);
	if (ret)
		return ret;

	return glink_rpm_wait(edge, rpm, acked);
}

int qcom_rpm_smd_write(struct udevice *dev, int state, u32 type, u32 id,
		       void *buf, size_t count)
{
	struct udevice *edge = dev_get_parent(dev);
	struct glink_rpm *rpm = dev_get_priv(edge);
	struct {
		struct qcom_rpm_header hdr;
		struct qcom_rpm_request req;
		u8 payload[RPM_MAX_MSG_SIZE];
	} pkt;
	u32 size = sizeof(pkt.hdr) + sizeof(pkt.req) + count;
	int ret;

	if (size >= RPM_MAX_MSG_SIZE)
		return -EINVAL;

	pkt.hdr.service_type = cpu_to_le32(RPM_SERVICE_TYPE_REQUEST);
	pkt.hdr.length = cpu_to_le32(sizeof(pkt.req) + count);
	pkt.req.msg_id = cpu_to_le32(++rpm->msg_id);
	pkt.req.flags = cpu_to_le32(state);
	pkt.req.type = cpu_to_le32(type);
	pkt.req.id = cpu_to_le32(id);
	pkt.req.data_len = cpu_to_le32(count);
	memcpy(pkt.payload, buf, count);

	ret = glink_rpm_send_data(rpm, RPM_CHANNEL_LCID, &pkt, size);
	if (ret)
		return ret;

	ret = glink_rpm_wait(edge, rpm, &rpm->msg_ready);
	if (ret) {
		dev_err(dev, "no RPM response: %d\n", ret);
		return ret;
	}

	return smd_rpm_parse_ack(dev, rpm->msg, rpm->msg_len);
}

static int smd_rpm_probe(struct udevice *dev)
{
	struct udevice *edge = dev_get_parent(dev);
	struct glink_rpm *rpm = dev_get_priv(edge);
	int ret;

	/* The RPM opens the channel once the versions are agreed on */
	ret = glink_rpm_wait(edge, rpm, &rpm->remote_open);
	if (ret) {
		dev_err(dev, "RPM did not open %s: %d\n", RPM_CHANNEL_NAME,
			ret);
		return ret;
	}

	ret = glink_rpm_open(edge, rpm, rpm->rcid, RPM_CHANNEL_LCID,
			     RPM_CHANNEL_NAME, &rpm->open_acked);
	if (ret)
		dev_err(dev, "RPM did not acknowledge %s: %d\n",
			RPM_CHANNEL_NAME, ret);

	return ret;
}

/*
 * Hand the link over in the state the kernel expects. The RPM announces its
 * channels once per link, and the kernel's GLINK driver waits for that
 * announcement before it opens rpm_requests. Ask the RPM, over its glink_ssr
 * channel as Linux does after a remote processor crash, to clean up its
 * channels to the application processor: it then resets the whole link,
 * FIFO indices included, and announces the channels again when the kernel
 * connects.
 */
static int smd_rpm_remove(struct udevice *dev)
{
	struct udevice *edge = dev_get_parent(dev);
	struct glink_rpm *rpm = dev_get_priv(edge);
	struct {
		__le32 version;
		__le32 command;
		__le32 seq_num;
		__le32 name_len;
		char name[32];
	} cleanup;
	ulong start;
	int ret;

	if (!rpm->ssr_rcid) {
		ret = -ENOENT;
		goto err;
	}

	ret = glink_rpm_open(edge, rpm, rpm->ssr_rcid, SSR_CHANNEL_LCID,
			     SSR_CHANNEL_NAME, &rpm->ssr_open_acked);
	if (ret)
		goto err;

	memset(&cleanup, 0, sizeof(cleanup));
	cleanup.command = cpu_to_le32(GLINK_SSR_DO_CLEANUP);
	cleanup.seq_num = cpu_to_le32(1);
	cleanup.name_len = cpu_to_le32(strlen(GLINK_SSR_APSS));
	strlcpy(cleanup.name, GLINK_SSR_APSS, sizeof(cleanup.name));

	ret = glink_rpm_send_data(rpm, SSR_CHANNEL_LCID, &cleanup,
				  sizeof(cleanup));
	if (ret)
		goto err;

	/*
	 * The RPM answers by resetting the link rather than with
	 * cleanup_done; do not read the receive FIFO meanwhile, the reset
	 * discards it.
	 */
	start = timer_get_us();
	while (readl(rpm->tx.head) || readl(rpm->tx.tail) ||
	       readl(rpm->rx.tail)) {
		if (timer_get_us() - start > RPM_RESET_TIMEOUT_US) {
			ret = -ETIMEDOUT;
			goto err;
		}
		udelay(10);
	}

	return 0;

err:
	dev_err(dev, "failed to reset the RPM link, the kernel may not reach the RPM: %d\n",
		ret);
	return 0;
}

static const struct udevice_id smd_rpm_ids[] = {
	{ .compatible = "qcom,glink-smd-rpm" },
	{ }
};

U_BOOT_DRIVER(qcom_smd_rpm) = {
	.name = "qcom_smd_rpm",
	.id = UCLASS_NOP,
	.of_match = smd_rpm_ids,
	.bind = dm_scan_fdt_dev,
	.probe = smd_rpm_probe,
	.remove = smd_rpm_remove,
	.flags = DM_FLAG_DEFAULT_PD_CTRL_OFF | DM_FLAG_OS_PREPARE,
};

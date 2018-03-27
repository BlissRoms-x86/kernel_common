/*
 *
 *  Generic Bluetooth 3-Wire driver
 *
 *  Copyright (C) 2015-2018  Intel Corporation
 *
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/skbuff.h>
#include <linux/serdev.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/acpi.h>
#include <linux/gpio/consumer.h>
#include <linux/firmware.h>
#include <linux/crc-ccitt.h>
#include <linux/bitrev.h>
#include <asm/unaligned.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#include "btrtl.h"

#define VERSION "1.0"

struct bt3wire_dev {
	struct hci_dev *hdev;
	struct serdev_device *serdev;

	struct work_struct tx_work;
	unsigned long tx_state;
	struct sk_buff_head tx_queue;
	u8 tx_seq;			/* Next seq number to send */
	u8 tx_ack;			/* Next seq number to receive */

	enum {
		SLIP_WAIT_DELIM,
		SLIP_PACKET,
		SLIP_ESCAPE,
	} rx_slip_state;
	struct sk_buff *rx_skb;

	enum {
		LINK_UNINITIALIZED,
		LINK_INITIALIZED,
		LINK_ACTIVE,
	} link_state;
	struct delayed_work link_timer;
	wait_queue_head_t link_wait;

	const struct bt3wire_vnd *vnd;
};

struct bt3wire_vnd {
	int (*setup)(struct bt3wire_dev *bdev);
	int (*open)(struct bt3wire_dev *bdev);
	int (*close)(struct bt3wire_dev *bdev);
	const struct acpi_gpio_mapping *acpi_gpios;
};

#define BT3WIRE_TX_STATE_ACTIVE	1
#define BT3WIRE_TX_STATE_WAKEUP	2

#define MAX_PACKET_SIZE	(4 + 4095 + 2)

#define PKT_TYPE_ACK		0x00
#define PKT_TYPE_HCI_COMMAND	0x01
#define PKT_TYPE_HCI_ACLDATA	0x02
#define PKT_TYPE_HCI_SCODATA	0x03
#define PKT_TYPE_HCI_EVENT	0x04
#define PKT_TYPE_LINK_CTRL	0x0f

#define LINK_PERIODIC_TIMEOUT	msecs_to_jiffies(250 * 4)
#define LINK_ACTIVATION_TIMEOUT	msecs_to_jiffies(8000)

#define LINK_MSG_SYNC		0x01
#define LINK_MSG_SYNC_RSP	0x02
#define LINK_MSG_CONF		0x03
#define LINK_MSG_CONF_RSP	0x04

#define SLIP_DELIM	0xc0
#define SLIP_ESC	0xdb
#define SLIP_XON	0x11
#define SLIP_XOFF	0x13
#define SLIP_ESC_DELIM	0xdc
#define SLIP_ESC_ESC	0xdd
#define SLIP_ESC_XON	0xde
#define SLIP_ESC_XOFF	0xdf

static void slip_buf_into_skb(struct sk_buff *skb, const u8 *buf, u16 len)
{
	int i;

	for (i = 0; i < len; i++) {
		switch (buf[i]) {
		case SLIP_DELIM:
			skb_put_u8(skb, SLIP_ESC);
			skb_put_u8(skb, SLIP_ESC_DELIM);
			break;
		case SLIP_ESC:
			skb_put_u8(skb, SLIP_ESC);
			skb_put_u8(skb, SLIP_ESC_ESC);
			break;
		default:
			skb_put_u8(skb, buf[i]);
			break;
		}
	}
}

static int bt3wire_queue_pkt(struct bt3wire_dev *bdev, u8 pkt_type,
			     const u8 *buf, u16 len)
{
	struct sk_buff *skb;
	bool add_crc;
	u8 hdr[4];

	switch (pkt_type) {
	case PKT_TYPE_HCI_COMMAND:
	case PKT_TYPE_HCI_ACLDATA:
		/* Reliable packet */
		hdr[0] = (bdev->tx_seq & 0x07) |
			 (bdev->tx_ack & 0x07) << 3 | 0x40 | 0x80;
		add_crc = true;
		break;

	case PKT_TYPE_HCI_SCODATA:
	case PKT_TYPE_ACK:
		/* Unreliable packet */
		hdr[0] = (bdev->tx_ack & 0x07) << 3;
		add_crc = false;
		break;

	case PKT_TYPE_LINK_CTRL:
		/* Link control packet */
		hdr[0] = 0x00;
		add_crc = false;
		break;

	default:
		bt_dev_err(bdev->hdev, "Invalid packet type %u", pkt_type);
		return -EILSEQ;
	}

	hdr[1] = (pkt_type & 0x0f) | (len & 0x0f) << 4;
	hdr[2] = (len & 0x0ff0) >> 4;
	hdr[3] = ~((hdr[0] + hdr[1] + hdr[2]) & 0xff);

	/* Maximum length of a packet after SLIP encoding the 0xC0 and 0xDB
	 * octets is (original len + 4 (header) + 2 (CRC)) * 2 + 2 (delimiter).
	 */
	skb = alloc_skb((len + 6) * 2 + 2, GFP_ATOMIC);
	if (!skb)
		return -ENOMEM;

	skb_put_u8(skb, SLIP_DELIM);
	slip_buf_into_skb(skb, hdr, 4);
	slip_buf_into_skb(skb, buf, len);
	if (add_crc) {
		u8 crc_buf[2];
		u16 crc;

		crc = crc_ccitt(0xffff, hdr, 4);
		crc = crc_ccitt(crc, buf, len);
		crc = bitrev16(crc);
		crc_buf[0] = (crc & 0xff00) >> 8;
		crc_buf[1] = (crc & 0x00ff);

		slip_buf_into_skb(skb, crc_buf, 2);
	}
	skb_put_u8(skb, SLIP_DELIM);

	switch (pkt_type) {
	case PKT_TYPE_HCI_COMMAND:
	case PKT_TYPE_HCI_ACLDATA:
		bdev->tx_seq = (bdev->tx_seq + 1) & 0x07;
		break;
	}

	skb_queue_tail(&bdev->tx_queue, skb);
	return 0;
}

static void bt3wire_tx_work(struct work_struct *work)
{
	struct bt3wire_dev *bdev = container_of(work, struct bt3wire_dev,
					       tx_work);
	struct serdev_device *serdev = bdev->serdev;
	struct hci_dev *hdev = bdev->hdev;

	while (1) {
		clear_bit(BT3WIRE_TX_STATE_WAKEUP, &bdev->tx_state);

		while (1) {
			struct sk_buff *skb = skb_dequeue(&bdev->tx_queue);
			int len;

			if (!skb)
				break;

			len = serdev_device_write_buf(serdev, skb->data,
						      skb->len);
			hdev->stat.byte_tx += len;

			skb_pull(skb, len);
			if (skb->len > 0) {
				skb_queue_head(&bdev->tx_queue, skb);
				break;
			}

			kfree_skb(skb);
		}

		if (!test_bit(BT3WIRE_TX_STATE_WAKEUP, &bdev->tx_state))
			break;
	}

	clear_bit(BT3WIRE_TX_STATE_ACTIVE, &bdev->tx_state);
}

static int bt3wire_tx_wakeup(struct bt3wire_dev *bdev)
{
	if (test_and_set_bit(BT3WIRE_TX_STATE_ACTIVE, &bdev->tx_state)) {
		set_bit(BT3WIRE_TX_STATE_WAKEUP, &bdev->tx_state);
		return 0;
	}

	schedule_work(&bdev->tx_work);
	return 0;
}

static void bt3wire_link_timer(struct work_struct *work)
{
	struct bt3wire_dev *bdev = container_of(work, struct bt3wire_dev,
						link_timer.work);
	static const unsigned char sync_pkt[] = { 0x01, 0x7e };
	//static const unsigned char conf_pkt[] = { 0x03, 0xfc, 0x17 };
	static const unsigned char conf_pkt[] = { 0x03, 0xfc, 0x11 };

	switch (bdev->link_state) {
	case LINK_UNINITIALIZED:
		bt3wire_queue_pkt(bdev, PKT_TYPE_LINK_CTRL,
				  sync_pkt, sizeof(sync_pkt));
		bt3wire_tx_wakeup(bdev);
		break;
	case LINK_INITIALIZED:
		bt3wire_queue_pkt(bdev, PKT_TYPE_LINK_CTRL,
				  conf_pkt, sizeof(conf_pkt));
		bt3wire_tx_wakeup(bdev);
		break;
	case LINK_ACTIVE:
		return;
        }

	schedule_delayed_work(&bdev->link_timer, LINK_PERIODIC_TIMEOUT);
}

static int bt3wire_open(struct hci_dev *hdev)
{
	struct bt3wire_dev *bdev = hci_get_drvdata(hdev);
	int err;

	bdev->tx_seq = 0;
	bdev->tx_ack = 0;

	bdev->rx_slip_state = SLIP_WAIT_DELIM;
	bdev->rx_skb = NULL;

	err = serdev_device_open(bdev->serdev);
	if (err) {
		bt_dev_err(hdev, "Unable to open UART device %s",
			   dev_name(&bdev->serdev->dev));
		return err;
	}

	if (bdev->vnd->open) {
		err = bdev->vnd->open(bdev);
		if (err) {
			bt_dev_err(hdev, "Vendor setup failed");
			serdev_device_close(bdev->serdev);
			return err;
		}
	}

	serdev_device_set_baudrate(bdev->serdev, 115200);

	bdev->link_state = LINK_UNINITIALIZED;
	schedule_delayed_work(&bdev->link_timer, 0);

	if (!wait_event_interruptible_timeout(bdev->link_wait,
					      bdev->link_state == LINK_ACTIVE,
					      LINK_ACTIVATION_TIMEOUT)) {
		bt_dev_err(hdev, "Link activation timeout");
		err = -ETIMEDOUT;

		cancel_delayed_work_sync(&bdev->link_timer);
		serdev_device_close(bdev->serdev);
	} else {
		bt_dev_info(hdev, "Link activation successful");
		err = 0;
	}

	return err;
}

static int bt3wire_close(struct hci_dev *hdev)
{
	struct bt3wire_dev *bdev = hci_get_drvdata(hdev);
	int err = 0;

	if (bdev->vnd->close)
		err = bdev->vnd->close(bdev);

	cancel_delayed_work_sync(&bdev->link_timer);
	serdev_device_close(bdev->serdev);

	return 0;
}

static int bt3wire_flush(struct hci_dev *hdev)
{
	struct bt3wire_dev *bdev = hci_get_drvdata(hdev);

	/* Flush any pending characters */
	serdev_device_write_flush(bdev->serdev);
	skb_queue_purge(&bdev->tx_queue);

	cancel_work_sync(&bdev->tx_work);

	kfree_skb(bdev->rx_skb);
	bdev->rx_skb = NULL;

	return 0;
}

static int bt3wire_setup(struct hci_dev *hdev)
{
	struct bt3wire_dev *bdev = hci_get_drvdata(hdev);

	if (bdev->vnd->setup)
		return bdev->vnd->setup(bdev);

	return 0;
}

static int bt3wire_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct bt3wire_dev *bdev = hci_get_drvdata(hdev);

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	}

	bt3wire_queue_pkt(bdev, hci_skb_pkt_type(skb), skb->data, skb->len);
	kfree_skb(skb);

	bt3wire_tx_wakeup(bdev);
	return 0;
}

static void bt3wire_process_link_ctrl(struct bt3wire_dev *bdev,
				      const u8 *buf, unsigned int len)
{
	static const unsigned char sync_rsp_pkt[] = { 0x02, 0x7d };
	static const unsigned char conf_rsp_pkt[] = { 0x04, 0x7b };

	/* The link control message consists at least of 2 octets and
	 * any packet smaller is invalid.
	 */
	if (len < 2) {
		bt_dev_warn(bdev->hdev, "Link control data too small");
		return;
	}

	/* The second octet for all link control packets equals the
	 * least significant 7 bits of the first octet, inverted,
	 * with the most significant bit set to ensure even parity.
	 *
	 * Unfortunately the 3-Wire specification got it wrong and
	 * has 0x01 0x7E, 0x02 0x7D, 0x03 0xFC and 0x04 0x7B octet
	 * sequences defined. So ignore the most significant bit
	 * since the specification has that all wrong.
	 */
	if (((buf[1] & 0x7f) != ((~buf[0]) & 0x7f))) {
		bt_dev_warn(bdev->hdev, "Invalid link control data");
		return;
	}

	switch (bdev->link_state) {
	case LINK_UNINITIALIZED:
		switch (buf[0]) {
		case LINK_MSG_SYNC:
			bt3wire_queue_pkt(bdev, PKT_TYPE_LINK_CTRL,
					  sync_rsp_pkt, sizeof(sync_rsp_pkt));
			bt3wire_tx_wakeup(bdev);
			break;
		case LINK_MSG_SYNC_RSP:
			bdev->link_state = LINK_INITIALIZED;
			cancel_delayed_work_sync(&bdev->link_timer);
			schedule_delayed_work(&bdev->link_timer, 0);
			break;
		}
		break;

	case LINK_INITIALIZED:
		switch (buf[0]) {
		case LINK_MSG_SYNC:
			bt3wire_queue_pkt(bdev, PKT_TYPE_LINK_CTRL,
					  sync_rsp_pkt, sizeof(sync_rsp_pkt));
			bt3wire_tx_wakeup(bdev);
			break;
		case LINK_MSG_CONF:
			bt3wire_queue_pkt(bdev, PKT_TYPE_LINK_CTRL,
					  conf_rsp_pkt, sizeof(conf_rsp_pkt));
			bt3wire_tx_wakeup(bdev);
			break;
		case LINK_MSG_CONF_RSP:
			if (len < 3) {
				bt_dev_warn(bdev->hdev, "Missing link control "
							"configuration data");
				return;
			}
			bt_dev_info(bdev->hdev, "Sliding window: %u", buf[2] & 0x07);
			bt_dev_info(bdev->hdev, "OOF flow control: %u", !!(buf[2] & 0x08));
			bt_dev_info(bdev->hdev, "Data integrity check type: %u", !!(buf[2] & 0x10));
			bt_dev_info(bdev->hdev, "Version number: %u", (buf[2] & 0xe0) >> 5);
			bdev->link_state = LINK_ACTIVE;
			cancel_delayed_work_sync(&bdev->link_timer);
			wake_up_interruptible(&bdev->link_wait);
			break;
		}
		break;

	case LINK_ACTIVE:
		switch (buf[0]) {
		case LINK_MSG_SYNC:
			/* TODO: Handle sync loss */
			break;
		case LINK_MSG_CONF:
			bt3wire_queue_pkt(bdev, PKT_TYPE_LINK_CTRL,
					  conf_rsp_pkt, sizeof(conf_rsp_pkt));
			bt3wire_tx_wakeup(bdev);
			break;
		}
	}
}

static void bt3wire_process_pkt(struct bt3wire_dev *bdev, struct sk_buff *skb)
{
	u8 *hdr;
	u16 len;
	bool rel;

	/* The packet header consists of 4 octets and any packet smaller than
	 * that is an invalid packet.
	 */
	if (skb->len < 4) {
		bt_dev_warn(bdev->hdev, "Packet too small (%u bytes)",
			    skb->len);
		kfree_skb(skb);
		return;
	}

	hdr = skb->data;

	/* The packet header checksum is calculated by setting it to a value
	 * such that the 2’s complement sum modulo 256 of the four header
	 * octets is 0xFF.
	 */
	if (((hdr[0] + hdr[1] + hdr[2] + hdr[3]) & 0xff) != 0xff) {
		bt_dev_warn(bdev->hdev, "Packet with wrong header checksum");
		kfree_skb(skb);
		return;
	}

	rel = !!(hdr[0] & 0x80);
	len = ((hdr[1] & 0xf0) >> 4) + (hdr[2] << 4);

	if (rel) {
		/* Store sequence number of next expected reliable packet */
		bdev->tx_ack = ((hdr[0] & 0x07) + 1) & 0x07;

		/* If there are no pending frames in the TX queue, then
		 * schedule an ACK packet to acknowledge the receiption
		 * of this reliable packet.
		 */
		if (!skb_queue_len(&bdev->tx_queue)) {
			bt3wire_queue_pkt(bdev, PKT_TYPE_ACK, NULL, 0);
			bt3wire_tx_wakeup(bdev);
		}
	}

	/* The data integrity check present bit indicates the payload is
	 * appended with a 16 bit CCITT-CRC data integrity check.
	 */
	if (hdr[0] & 0x40) {
		u16 crc;

		if (skb->len != len + 4 + 2) {
			bt_dev_warn(bdev->hdev, "Packet with missing checksum");
			kfree_skb(skb);
			return;
		}

		crc = crc_ccitt(0xffff, skb->data, len + 4);
		crc = bitrev16(crc);

		if (crc != get_unaligned_be16(skb->data + 4 + len)) {
			bt_dev_warn(bdev->hdev, "Packet failed integrity check");
			kfree_skb(skb);
			return;
		}

		skb_trim(skb, len + 4);
	} else if (skb->len != len + 4) {
		bt_dev_warn(bdev->hdev, "Packet with invalid payload length");
		kfree_skb(skb);
		return;
	}

	skb_pull(skb, 4);

	/* The packet payload length does not include the length of the packet
	 * header or the length of the optional data integrity check. The range
	 * is defined as 0-4095 for maixmum payload size.
	 */
	if (skb->len > 4095) {
		bt_dev_warn(bdev->hdev, "Packet payload too large (%u bytes)",
			    skb->len);
		kfree_skb(skb);
		return;
	}

	/* The packet type differentiates between the four HCI packet types
	 * and transport specific acknoledgments, link control and vendor
	 * specific packet types.
	 */
	switch (hdr[1] & 0x0f) {
	case PKT_TYPE_ACK:
		bt_dev_info(bdev->hdev, "Acknowledgement packet");
		break;
	case PKT_TYPE_HCI_ACLDATA:
		hci_skb_pkt_type(skb) = HCI_ACLDATA_PKT;
		hci_recv_frame(bdev->hdev, skb);
		break;
	case PKT_TYPE_HCI_EVENT:
		hci_skb_pkt_type(skb) = HCI_EVENT_PKT;
		hci_recv_frame(bdev->hdev, skb);
		break;
	case PKT_TYPE_LINK_CTRL:
		bt3wire_process_link_ctrl(bdev, skb->data, skb->len);
		kfree_skb(skb);
		break;
	default:
		bt_dev_err(bdev->hdev, "Unknown packet type %u", hdr[1] & 0x0f);
		kfree_skb(skb);
		break;
	}
}

static int bt3wire_receive_buf(struct serdev_device *serdev, const u8 *buf,
			      size_t count)
{
	struct bt3wire_dev *bdev = serdev_device_get_drvdata(serdev);
	size_t i;

	for (i = 0; i < count; i++) {
		const unsigned char *ptr = buf + i;

		switch (bdev->rx_slip_state) {
		case SLIP_WAIT_DELIM:
			/* The delimiter octet 0xC0 is placed at the start and
			 * end of every packet. To synchronize with the packet
			 * stream, wait for the delimiter octet and drop any
			 * other octets.
			 */
			if (*ptr == SLIP_DELIM) {
				bdev->rx_skb = bt_skb_alloc(MAX_PACKET_SIZE,
							    GFP_ATOMIC);
				if (bdev->rx_skb)
					bdev->rx_slip_state = SLIP_PACKET;
			}
			break;
		case SLIP_PACKET:
			/* Receiving the delimiter octet 0xC0 indicates a
			 * complete packet, the escape octet 0xDB indicates
			 * the switch to an escape sequence, and all other
			 * octets are copied into the result.
			 */
			if (*ptr == SLIP_DELIM) {
				bt3wire_process_pkt(bdev, bdev->rx_skb);
				bdev->rx_skb = NULL;
				bdev->rx_slip_state = SLIP_WAIT_DELIM;
			} else if (*ptr == SLIP_ESC) {
				bdev->rx_slip_state = SLIP_ESCAPE;
			} else {
				if (bdev->rx_skb->len < MAX_PACKET_SIZE) {
					skb_put_u8(bdev->rx_skb, *ptr);
				} else {
					kfree_skb(bdev->rx_skb);
					bdev->rx_skb = NULL;
					bdev->rx_slip_state = SLIP_WAIT_DELIM;
				}
			}
			break;
		case SLIP_ESCAPE:
			/* As part of the escape sequence, only the octets
			 * 0xDC for the delimiter and 0xDD for the escape
			 * sequence are valid. All other octets are causing
			 * re-sychronization with the delimter.
			 */
			if (*ptr == SLIP_ESC_DELIM) {
				if (bdev->rx_skb->len < MAX_PACKET_SIZE) {
					skb_put_u8(bdev->rx_skb, SLIP_DELIM);
				} else {
					kfree_skb(bdev->rx_skb);
					bdev->rx_skb = NULL;
					bdev->rx_slip_state = SLIP_WAIT_DELIM;
				}
				bdev->rx_slip_state = SLIP_PACKET;
			} else if (*ptr == SLIP_ESC_ESC) {
				if (bdev->rx_skb->len < MAX_PACKET_SIZE) {
					skb_put_u8(bdev->rx_skb, SLIP_ESC);
				} else {
					kfree_skb(bdev->rx_skb);
					bdev->rx_skb = NULL;
					bdev->rx_slip_state = SLIP_WAIT_DELIM;
				}
				bdev->rx_slip_state = SLIP_PACKET;
			} else {
				kfree_skb(bdev->rx_skb);
				bdev->rx_skb = NULL;
				bdev->rx_slip_state = SLIP_WAIT_DELIM;
			}
			break;
		}
	}

	bdev->hdev->stat.byte_rx += count;

	return count;
}

static void bt3wire_write_wakeup(struct serdev_device *serdev)
{
	struct bt3wire_dev *bdev = serdev_device_get_drvdata(serdev);

	bt3wire_tx_wakeup(bdev);
}

static const struct serdev_device_ops bt3wire_client_ops = {
	.receive_buf = bt3wire_receive_buf,
	.write_wakeup = bt3wire_write_wakeup,
};

static int bt3wire_probe(struct serdev_device *serdev)
{
	struct bt3wire_dev *bdev;
	struct hci_dev *hdev;

	bdev = devm_kzalloc(&serdev->dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;

	bdev->serdev = serdev;
	serdev_device_set_drvdata(serdev, bdev);

	if (has_acpi_companion(&serdev->dev)) {
		bdev->vnd = acpi_device_get_match_data(&serdev->dev);
		if (bdev->vnd->acpi_gpios)
			devm_acpi_dev_add_driver_gpios(&serdev->dev,
						       bdev->vnd->acpi_gpios);
	} else {
		bdev->vnd = of_device_get_match_data(&serdev->dev);
	}

	serdev_device_set_client_ops(serdev, &bt3wire_client_ops);

	INIT_WORK(&bdev->tx_work, bt3wire_tx_work);
	skb_queue_head_init(&bdev->tx_queue);

	INIT_DELAYED_WORK(&bdev->link_timer, bt3wire_link_timer);
	init_waitqueue_head(&bdev->link_wait);

	/* Initialize and register HCI device */
	hdev = hci_alloc_dev();
	if (!hdev) {
		dev_err(&serdev->dev, "Can't allocate HCI device\n");
		return -ENOMEM;
	}

	bdev->hdev = hdev;

	hdev->bus = HCI_UART;
	hci_set_drvdata(hdev, bdev);

	hdev->manufacturer = 15;

	hdev->open  = bt3wire_open;
	hdev->close = bt3wire_close;
	hdev->flush = bt3wire_flush;
	hdev->setup = bt3wire_setup;
	hdev->send  = bt3wire_send_frame;
	SET_HCIDEV_DEV(hdev, &serdev->dev);

	if (hci_register_dev(hdev) < 0) {
		dev_err(&serdev->dev, "Can't register HCI device\n");
		hci_free_dev(hdev);
		return -ENODEV;
	}

	return 0;
}

static void bt3wire_remove(struct serdev_device *serdev)
{
	struct bt3wire_dev *bdev = serdev_device_get_drvdata(serdev);
	struct hci_dev *hdev = bdev->hdev;

	hci_unregister_dev(hdev);
	hci_free_dev(hdev);
}

static int bt3wire_btrtl_setup(struct bt3wire_dev *bdev)
{
	struct btrtl_device_info *btrtl_dev;
	struct sk_buff *skb;
	__le32 baudrate_data;
	u32 device_baudrate;
	unsigned int controller_baudrate;
	bool flow_control;
	int err;

	btrtl_dev = btrtl_initialize(bdev->hdev);
	if (IS_ERR(btrtl_dev))
		return PTR_ERR(btrtl_dev);

	err = btrtl_get_uart_settings(bdev->hdev, btrtl_dev,
				      &controller_baudrate, &device_baudrate,
				      &flow_control);
	if (err)
		goto out_free;

	baudrate_data = cpu_to_le32(device_baudrate);
	skb = __hci_cmd_sync(bdev->hdev, 0xfc17, sizeof(baudrate_data),
			     &baudrate_data, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(bdev->hdev, "set baud rate command failed");
		err = PTR_ERR(skb);
		goto out_free;
	} else {
		kfree_skb(skb);
	}
	/* Give the device some type to set up the new baudrate. This value was
	 * chosen by manually testing on a RTL8723BS. Shorter wait times lead to
	 * a non-responsive device when downloading the firmware.
	 */
	msleep(10);

	serdev_device_set_baudrate(bdev->serdev, controller_baudrate);
	serdev_device_set_flow_control(bdev->serdev, flow_control);

	err = btrtl_download_firmware(bdev->hdev, btrtl_dev);

out_free:
	btrtl_free(btrtl_dev);

	return err;
}

static int bt3wire_btrtl_open(struct bt3wire_dev *bdev)
{
	struct gpio_desc *enable_gpio;
	struct gpio_desc *reset_gpio;
	int err;

	/* devices always start with flow control disabled and even parity */
	serdev_device_set_flow_control(bdev->serdev, false);
	err = serdev_device_set_parity(bdev->serdev, SERDEV_PARITY_EVEN);
	if (err)
		return err;

	enable_gpio = devm_gpiod_get_optional(&bdev->serdev->dev, "enable",
					      GPIOD_OUT_HIGH);
	if (IS_ERR(enable_gpio))
		return PTR_ERR(enable_gpio);
	reset_gpio = devm_gpiod_get_optional(&bdev->serdev->dev, "reset",
					     GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio))
		return PTR_ERR(reset_gpio);

	/* Disable the device and put it into reset. Some devices only have one
	 * of these lines, so toggle both to support all combinations. After
	 * toggling, we need to wait for the device to disable/reset. 500ms was
	 * chosen by manually testing on a RTL8723BS. Shorter wait times lead to
	 * a non-responsible device.
	 */
	gpiod_set_value_cansleep(reset_gpio, 1);
	gpiod_set_value_cansleep(enable_gpio, 0);
	msleep(500);

	/* Re-enable the device and wait another 500ms, otherwise the device
	 * might not respond in all cases. This value was chosen by manually
	 * testing on an RTL8723BS.
	 */
	gpiod_set_value_cansleep(reset_gpio, 0);
	gpiod_set_value_cansleep(enable_gpio, 1);
	msleep(500);

	return 0;
}

static int bt3wire_btrtl_close(struct bt3wire_dev *bdev)
{
	struct gpio_desc *enable_gpio;
	enable_gpio = devm_gpiod_get_optional(&bdev->serdev->dev, "enable",
					      GPIOD_OUT_HIGH);
	if (IS_ERR(enable_gpio))
		return PTR_ERR(enable_gpio);
	gpiod_set_value_cansleep(enable_gpio, 0);
	return 0;
}

static const struct acpi_gpio_params btrtl_enable_gpios = { 0, 0, false };
static const struct acpi_gpio_mapping acpi_btrtl_gpios[] = {
	{ "enable-gpios", &btrtl_enable_gpios, 1 },
	{},
};

static struct bt3wire_vnd rtl_vnd = {
	.setup		= bt3wire_btrtl_setup,
	.open		= bt3wire_btrtl_open,
	.close		= bt3wire_btrtl_close,
	.acpi_gpios	= acpi_btrtl_gpios,
};

#ifdef CONFIG_ACPI
static const struct acpi_device_id bt3wire_acpi_match[] = {
	{ "OBDA8723", (kernel_ulong_t)&rtl_vnd },
	{ },
};
MODULE_DEVICE_TABLE(acpi, bt3wire_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id bt3wire_of_match[] = {
	{ .compatible = "brcm,bcm43438-bt" },
	{ .compatible = "realtek,rtl8723bs-bluetooth", .data = &rtl_vnd },
	{ .compatible = "realtek,rtl8723ds-bluetooth", .data = &rtl_vnd },
	{ },
};
MODULE_DEVICE_TABLE(of, bt3wire_of_match);
#endif

static struct serdev_device_driver bt3wire_driver = {
	.probe = bt3wire_probe,
	.remove = bt3wire_remove,
	.driver = {
		.name = "bt3wire",
		.of_match_table = of_match_ptr(bt3wire_of_match),
		.acpi_match_table = ACPI_PTR(bt3wire_acpi_match),
	},
};

module_serdev_device_driver(bt3wire_driver);

MODULE_AUTHOR("Marcel Holtmann <marcel@holtmann.org>");
MODULE_DESCRIPTION("Generic Bluetooth 3-Wire driver ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");

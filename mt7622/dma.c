// SPDX-License-Identifier: ISC
/* Copyright (C) 2019 MediaTek Inc.
 *
 * Author: Ryder Lee <ryder.lee@mediatek.com>
 *         Lorenzo Bianconi <lorenzo@kernel.org>
 *         Felix Fietkau <nbd@nbd.name>
 *         Shayne Chen <shayne.chen@mediatek.com>
 */

#include "mt7622.h"
#include "../dma.h"
#include "mac.h"

static int
mt7622_init_tx_queue(struct mt7622_dev *dev, struct mt76_sw_queue *q,
		     int idx, int n_desc)
{
	struct mt76_queue *hwq;
	int err;

	hwq = devm_kzalloc(dev->mt76.dev, sizeof(*hwq), GFP_KERNEL);
	if (!hwq)
		return -ENOMEM;

	err = mt76_queue_alloc(dev, hwq, idx, n_desc, 0, MT_TX_RING_BASE);
	if (err < 0)
		return err;

	INIT_LIST_HEAD(&q->swq);
	q->q = hwq;

	return 0;
}

void mt7622_queue_rx_skb(struct mt76_dev *mdev, enum mt76_rxq_id q,
			 struct sk_buff *skb)
{
	struct mt7622_dev *dev = container_of(mdev, struct mt7622_dev, mt76);
	__le32 *rxd = (__le32 *)skb->data;
	__le32 *end = (__le32 *)&skb->data[skb->len];
	enum rx_pkt_type type;

	type = FIELD_GET(MT_RXD0_PKT_TYPE, le32_to_cpu(rxd[0]));

	switch (type) {
	case PKT_TYPE_TXS:
		for (rxd++; rxd + 7 <= end; rxd += 7)
			mt7622_mac_add_txs(dev, rxd);
		dev_kfree_skb(skb);
		break;
	case PKT_TYPE_TXRX_NOTIFY:
		mt7622_mac_tx_free(dev, skb);
		break;
	case PKT_TYPE_RX_EVENT:
		mt7622_mcu_rx_event(dev, skb);
		break;
	case PKT_TYPE_NORMAL:
		if (!mt7622_mac_fill_rx(dev, skb)) {
			mt76_rx(&dev->mt76, q, skb);
			return;
		}
		/* fall through */
	default:
		dev_kfree_skb(skb);
		break;
	}
}

static int mt7622_poll_tx(struct napi_struct *napi, int budget)
{
	struct mt7622_dev *dev;
	int i;

	dev = container_of(napi, struct mt7622_dev, mt76.tx_napi);

	for (i = MT_TXQ_MCU; i >= 0; i--)
		mt76_queue_tx_cleanup(dev, i, false);

	if (napi_complete_done(napi, 0))
		mt7622_irq_enable(dev, MT_INT_TX_DONE_ALL);

	for (i = MT_TXQ_MCU; i >= 0; i--)
		mt76_queue_tx_cleanup(dev, i, false);

	mt7622_mac_sta_poll(dev);

	tasklet_schedule(&dev->mt76.tx_tasklet);

	return 0;
}

static u32 
mt7622_dma_sched_rr(struct mt7622_dev *dev, u32 offset)
{
	u32 val, ori_addr;

	ori_addr = mt76_rr(dev, MT_DMASHDL_REMAP);
	mt76_wr(dev, MT_DMASHDL_REMAP, MT_DMASHDL_BASE);
	val = mt76_rr(dev, MT_DMASHDL_OFS(offset));
	mt76_wr(dev, MT_DMASHDL_REMAP, ori_addr);

	return val;
}

static void 
mt7622_dma_sched_wr(struct mt7622_dev *dev, u32 offset, u32 val)
{
	u32 ori_addr = mt76_rr(dev, MT_DMASHDL_REMAP);
	mt76_wr(dev, MT_DMASHDL_REMAP, MT_DMASHDL_BASE);
	mt76_wr(dev, MT_DMASHDL_OFS(offset), val);
	mt76_wr(dev, MT_DMASHDL_REMAP, ori_addr);
}

int mt7622_dma_sched_init(struct mt7622_dev *dev)
{
	u32 val;

	val = mt7622_dma_sched_rr(dev, MT_DMASHDL_PKT_MAX_SIZE);
	val &= ~(PLE_PKT_MAX_SIZE | PSE_PKT_MAX_SIZE);
	val |= (FIELD_PREP(PLE_PKT_MAX_SIZE, 0x1) |
			FIELD_PREP(PSE_PKT_MAX_SIZE, 0x8));
	mt7622_dma_sched_wr(dev, MT_DMASHDL_PKT_MAX_SIZE, val);

	/* Enable refill Control Group 0, 1, 2, 4, 5 */
	mt7622_dma_sched_wr(dev, MT_DMASHDL_REFILL_CTRL, 0xffc80000);
	val = FIELD_PREP(DMASHDL_MIN_QUOTA, 0x10) |
		FIELD_PREP(DMASHDL_MAX_QUOTA, 0x800);

	mt7622_dma_sched_wr(dev, MT_DMASHDL_GROUP0_CTRL, val);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_GROUP1_CTRL, val);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_GROUP2_CTRL, val);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_GROUP4_CTRL, val);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_GROUP5_CTRL, val);

	mt7622_dma_sched_wr(dev, MT_DMASHDL_Q_MAP0, 0x42104210);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_Q_MAP1, 0x42104210);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_Q_MAP2, 0x00000005);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_Q_MAP3, 0x0);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_SET0, 0x6012345f);
	mt7622_dma_sched_wr(dev, MT_DMASHDL_SET1, 0xedcba987);

	return 0;
}

int mt7622_dma_init(struct mt7622_dev *dev)
{
	int i, ret;
	static const u8 wmm_queue_map[] = {
		[IEEE80211_AC_BK] = 0,
		[IEEE80211_AC_BE] = 1,
		[IEEE80211_AC_VI] = 2,
		[IEEE80211_AC_VO] = 4,
	};

	mt76_dma_attach(&dev->mt76);

	mt76_wr(dev, MT_WPDMA_GLO_CFG,
		MT_WPDMA_GLO_CFG_TX_WRITEBACK_DONE |
		MT_WPDMA_GLO_CFG_FIFO_LITTLE_ENDIAN |
		MT_WPDMA_GLO_CFG_OMIT_TX_INFO);

	mt76_rmw_field(dev, MT_WPDMA_GLO_CFG,
		       MT_WPDMA_GLO_CFG_FW_RING_BP_TX_SCH, 0x1);

	mt76_rmw_field(dev, MT_WPDMA_GLO_CFG,
		       MT_WPDMA_GLO_CFG_TX_BT_SIZE_BIT21, 0x1);

	mt76_rmw_field(dev, MT_WPDMA_GLO_CFG,
		       MT_WPDMA_GLO_CFG_DMA_BURST_SIZE, 0x3);

	mt76_rmw_field(dev, MT_WPDMA_GLO_CFG,
		       MT_WPDMA_GLO_CFG_MULTI_DMA_EN, 0x3);

	mt76_wr(dev, MT_WPDMA_RST_IDX, ~0);

	for (i = 0; i < ARRAY_SIZE(wmm_queue_map); i++) {
		ret = mt7622_init_tx_queue(dev, &dev->mt76.q_tx[i],
					   wmm_queue_map[i],
					   MT7622_TX_RING_SIZE);
		if (ret)
			return ret;
	}

	ret = mt7622_init_tx_queue(dev, &dev->mt76.q_tx[MT_TXQ_PSD],
				   MT7622_TXQ_MGMT, MT7622_TX_RING_SIZE);
	if (ret)
		return ret;

	ret = mt7622_init_tx_queue(dev, &dev->mt76.q_tx[MT_TXQ_MCU],
				   MT7622_TXQ_MCU, MT7622_TX_MCU_RING_SIZE);
	if (ret)
		return ret;

	ret = mt7622_init_tx_queue(dev, &dev->mt76.q_tx[MT_TXQ_FWDL],
				   MT7622_TXQ_FWDL, MT7622_TX_FWDL_RING_SIZE);
	if (ret)
		return ret;

	/* init rx queues */
	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MCU], 1,
			       MT7622_RX_MCU_RING_SIZE, MT_RX_BUF_SIZE,
			       MT_RX_RING_BASE);
	if (ret)
		return ret;

	ret = mt76_queue_alloc(dev, &dev->mt76.q_rx[MT_RXQ_MAIN], 0,
			       MT7622_RX_RING_SIZE, MT_RX_BUF_SIZE,
			       MT_RX_RING_BASE);
	if (ret)
		return ret;

	mt76_wr(dev, MT_DELAY_INT_CFG, 0);

	ret = mt76_init_queues(dev);
	if (ret < 0)
		return ret;

	netif_tx_napi_add(&dev->mt76.napi_dev, &dev->mt76.tx_napi,
			  mt7622_poll_tx, NAPI_POLL_WEIGHT);
	napi_enable(&dev->mt76.tx_napi);

	mt76_poll(dev, MT_WPDMA_GLO_CFG,
		  MT_WPDMA_GLO_CFG_TX_DMA_BUSY |
		  MT_WPDMA_GLO_CFG_RX_DMA_BUSY, 0, 1000);

	/* start dma engine */
	mt76_set(dev, MT_WPDMA_GLO_CFG,
		 MT_WPDMA_GLO_CFG_TX_DMA_EN |
		 MT_WPDMA_GLO_CFG_RX_DMA_EN);

	/* enable interrupts for TX/RX rings */
	mt7622_irq_enable(dev, MT_INT_RX_DONE_ALL | MT_INT_TX_DONE_ALL);

	return 0;
}

void mt7622_dma_cleanup(struct mt7622_dev *dev)
{
	mt76_clear(dev, MT_WPDMA_GLO_CFG,
		   MT_WPDMA_GLO_CFG_TX_DMA_EN |
		   MT_WPDMA_GLO_CFG_RX_DMA_EN);
	mt76_set(dev, MT_WPDMA_GLO_CFG, MT_WPDMA_GLO_CFG_SW_RESET);

	tasklet_kill(&dev->mt76.tx_tasklet);
	mt76_dma_cleanup(&dev->mt76);
}

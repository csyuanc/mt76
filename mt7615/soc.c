// SPDX-License-Identifier: ISC
/* Copyright (C) 2019 MediaTek Inc.
 *
 * Author: Ryder Lee <ryder.lee@mediatek.com>
 *         Shayne Chen <shayne.chen@mediatek.com>
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include "mt7615.h"
#include "mac.h"

static int
mt7622_wmac_init(struct mt7615_dev *dev)
{
	struct device_node *np = dev->mt76.dev->of_node;

	if (!is_mt7622(&dev->mt76))
		return 0;

	dev->infracfg = syscon_regmap_lookup_by_phandle(np, "infracfg");
	if (IS_ERR(dev->infracfg)) {
		dev_err(dev->mt76.dev, "Cannot find infracfg controller\n");
		return PTR_ERR(dev->infracfg);
	}

	return 0;
}

static int mt76_wmac_probe(struct platform_device *pdev)
{
	static const struct mt76_driver_ops drv_ops = {
		/* txwi_size = txd size + txp size */
		.txwi_size = MT_TXD_SIZE + sizeof(struct mt7622_txp),
		.drv_flags = MT_DRV_TXWI_NO_FREE,
		.tx_prepare_skb = mt7622_tx_prepare_skb,
		.tx_complete_skb = mt7622_tx_complete_skb,
		.rx_skb = mt7615_queue_rx_skb,
		.rx_poll_complete = mt7615_rx_poll_complete,
		.sta_ps = mt7615_sta_ps,
		.sta_add = mt7615_mac_sta_add,
		.sta_remove = mt7615_mac_sta_remove,
		.update_survey = mt7615_update_channel,
	};
	struct resource *res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	struct mt7615_dev *dev;
	void __iomem *mem_base;
	struct mt76_dev *mdev;
	int irq;
	int ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(&pdev->dev, "Failed to get device IRQ\n");
		return irq;
	}

	mem_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(mem_base)) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return PTR_ERR(mem_base);
	}

	mdev = mt76_alloc_device(&pdev->dev, sizeof(*dev), &mt7615_ops,
				 &drv_ops);
	if (!mdev)
		return -ENOMEM;

	dev = container_of(mdev, struct mt7615_dev, mt76);
	mt76_mmio_init(mdev, mem_base);

	mdev->rev = (mt76_rr(dev, MT_HW_CHIPID) << 16) |
		(mt76_rr(dev, MT_HW_REV) & 0xff);
	dev_info(mdev->dev, "ASIC revision: %04x\n", mdev->rev);

	mt7622_wmac_init(dev);

	ret = devm_request_irq(mdev->dev, irq, mt7615_irq_handler,
			       IRQF_SHARED, KBUILD_MODNAME, dev);
	if (ret)
		goto error;

	ret = mt7615_register_device(dev);
	if (ret)
		goto error;

	return 0;
error:
	ieee80211_free_hw(mt76_hw(dev));
	return ret;
}

static int mt76_wmac_remove(struct platform_device *pdev)
{
	struct mt76_dev *mdev = platform_get_drvdata(pdev);
	struct mt7615_dev *dev = container_of(mdev, struct mt7615_dev, mt76);

	mt7615_unregister_device(dev);

	return 0;
}

static const struct of_device_id of_wmac_match[] = {
	{ .compatible = "mediatek,mt7622-wmac" },
	{},
};

struct platform_driver mt76_wmac_driver = {
	.probe		= mt76_wmac_probe,
	.remove		= mt76_wmac_remove,
	.driver = {
		.name = "mt7622_wmac",
		.of_match_table = of_wmac_match,
	},
};
module_platform_driver(mt76_wmac_driver);

MODULE_DEVICE_TABLE(of, of_wmac_match);
MODULE_FIRMWARE(MT7622_FIRMWARE_N9);
MODULE_FIRMWARE(MT7622_ROM_PATCH);
MODULE_LICENSE("Dual BSD/GPL");

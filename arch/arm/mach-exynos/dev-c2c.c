/* linux/arch/arm/mach-exynos/dev-c2c.c
 *
 * Copyright (c) 2012 Samsung Electronics
 *
 * Base EXYNOS C2C resource and device definitions
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/io.h>

#include <mach/map.h>
#include <mach/regs-pmu.h>
#include <mach/pmu.h>
#include <mach/c2c.h>
#include <plat/irqs.h>
#include <plat/cpu.h>

static struct resource exynos_c2c_resource[] = {
	[0] = {
		.start  = EXYNOS_PA_C2C,
		.end    = EXYNOS_PA_C2C + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start  = EXYNOS_PA_C2C_CP,
		.end    = EXYNOS_PA_C2C_CP + SZ_64K - 1,
		.flags  = IORESOURCE_MEM,
	},
	[2] = {
		.start  = IRQ_C2C_SSCM0,
		.end    = IRQ_C2C_SSCM0,
		.flags  = IORESOURCE_IRQ,
	},
	[3] = {
		.start  = IRQ_C2C_SSCM1,
		.end    = IRQ_C2C_SSCM1,
		.flags  = IORESOURCE_IRQ,
	},
};

static u64 exynos_c2c_dma_mask = DMA_BIT_MASK(32);

struct platform_device exynos_device_c2c = {
	.name		= "samsung-c2c",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(exynos_c2c_resource),
	.resource	= exynos_c2c_resource,
	.dev		= {
		.dma_mask		= &exynos_c2c_dma_mask,
		.coherent_dma_mask	= DMA_BIT_MASK(32),
	},
};

void __init exynos_c2c_set_platdata(struct exynos_c2c_platdata *pd)
{
	struct exynos_c2c_platdata *npd = pd;
	pr_err("[C2C] %s: +++\n", __func__);

	if (!npd->setup_gpio)
		npd->setup_gpio = exynos_c2c_cfg_gpio;

	if (!npd->set_cprst)
		npd->set_cprst = exynos_c2c_set_cprst;

	if (!npd->clear_cprst)
		npd->clear_cprst = exynos_c2c_clear_cprst;

	/* Set C2C_CTRL Register */
	writel(0x1, S5P_C2C_CTRL);

	exynos_device_c2c.dev.platform_data = npd;
}


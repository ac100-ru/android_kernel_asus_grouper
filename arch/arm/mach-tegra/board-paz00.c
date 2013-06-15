/*
 * arch/arm/mach-tegra/board-paz00.c
 *
 * Copyright (C) 2011 Marc Dietrich <marvin24@gmx.de>
 *
 * Based on board-harmony.c
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/serial_8250.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/gpio_keys.h>
#include <linux/pda_power.h>
#include <linux/memblock.h>
#include <linux/mfd/core.h>
#include <linux/io.h>
#include <linux/input.h>
#include <linux/i2c.h>
#include <linux/i2c-tegra.h>
#include <linux/tegra_uart.h>
#include <linux/mfd/core.h>
#include <linux/rfkill-gpio.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/mach/time.h>
#include <asm/setup.h>

#include <mach/iomap.h>
#include <mach/irqs.h>
#include <mach/sdhci.h>
#include <mach/gpio.h>
#include <mach/usb_phy.h>
#include <mach/tegra_alc5632_pdata.h>
#include <mach/clk.h>
#include <mach/tegra_fiq_debugger.h>

#include "board.h"
#include "board-paz00.h"
#include "clock.h"
#include "devices.h"
#include "gpio-names.h"
#include "pm.h"
#include "../../../drivers/staging/nvec/nvec.h"
#include "wakeups-t2.h"

#define ATAG_NVIDIA	0x41000801
#define MAX_MEMHDL	8

struct tag_tegra {
	__u32 bootarg_len;
	__u32 bootarg_key;
	__u32 bootarg_nvkey;
	__u32 bootarg[];
};

struct memhdl {
	__u32 id;
	__u32 start;
	__u32 size;
};

enum {
	RM = 1,
	DISPLAY,
	FRAMEBUFFER,
	CHIPSHMOO,
	CHIPSHMOO_PHYS,
	CARVEOUT,
	WARMBOOT,
};

static int num_memhdl = 0;

static struct memhdl nv_memhdl[MAX_MEMHDL];

static const char atag_ids[][16] = {
	"RM             ",
	"DISPLAY        ",
	"FRAMEBUFFER    ",
	"CHIPSHMOO      ",
	"CHIPSHMOO_PHYS ",
	"CARVEOUT       ",
	"WARMBOOT       ",
};

static int __init parse_tag_nvidia(const struct tag *tag)
{
	int i;
	struct tag_tegra *nvtag = (struct tag_tegra *)tag;
	__u32 id;

	switch (nvtag->bootarg_nvkey) {
	case FRAMEBUFFER:
		id = nvtag->bootarg[1];
		for (i=0; i<num_memhdl; i++)
			if (nv_memhdl[i].id == id) {
				tegra_bootloader_fb_start = nv_memhdl[i].start;
				tegra_bootloader_fb_size = nv_memhdl[i].size;
			}
		break;
	case WARMBOOT:
		id = nvtag->bootarg[1];
		for (i=0; i<num_memhdl; i++) {
			if (nv_memhdl[i].id == id) {
				tegra_lp0_vec_start = nv_memhdl[i].start;
				tegra_lp0_vec_size = nv_memhdl[i].size;
			}
		}
		break;
	}

	if (nvtag->bootarg_nvkey & 0x10000) {
		char pmh[] = " PreMemHdl     ";
		id = nvtag->bootarg_nvkey;
		if (num_memhdl < MAX_MEMHDL) {
			nv_memhdl[num_memhdl].id = id;
			nv_memhdl[num_memhdl].start = nvtag->bootarg[1];
			nv_memhdl[num_memhdl].size = nvtag->bootarg[2];
			num_memhdl++;
		}
		pmh[11] = '0' + id;
		print_hex_dump(KERN_INFO, pmh, DUMP_PREFIX_NONE,
				32, 4, &nvtag->bootarg[0], 4*(tag->hdr.size-2), false);
	}
	else if (nvtag->bootarg_nvkey <= ARRAY_SIZE(atag_ids))
		print_hex_dump(KERN_INFO, atag_ids[nvtag->bootarg_nvkey-1], DUMP_PREFIX_NONE,
				32, 4, &nvtag->bootarg[0], 4*(tag->hdr.size-2), false);
	else
		pr_warning("unknown ATAG key %d\n", nvtag->bootarg_nvkey);

	return 0;
}
__tagtable(ATAG_NVIDIA, parse_tag_nvidia);

static struct platform_device *paz00_uart_devices[] __initdata = {
	&tegra_uarta_device,
	&tegra_uartb_device,
	&tegra_uartc_device,
	&tegra_uartd_device,
	&tegra_uarte_device,
};

static struct uart_clk_parent uart_parent_clk[] = {
	[0] = {.name = "pll_p"},
	[1] = {.name = "pll_m"},
	[2] = {.name = "clk_m"},
};

static struct tegra_uart_platform_data paz00_uart_pdata;
static unsigned long debug_uart_port_clk_rate;
static void __init uart_debug_init(void)
{
	int debug_port_id;

	debug_port_id = get_tegra_uart_debug_port_id();
	if (debug_port_id < 0)
		debug_port_id = 3;

	switch (debug_port_id) {
	case 0:
		/* UARTA is the debug port. */
		pr_info("Selecting UARTA as the debug console\n");
		paz00_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		debug_uart_port_clk_rate = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->uartclk;

		break;

	case 1:
		/* UARTB is the debug port. */
		pr_info("Selecting UARTB as the debug console\n");
		paz00_uart_devices[1] = &debug_uartb_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartb");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->mapbase;
		debug_uart_port_clk_rate = ((struct plat_serial8250_port *)(
			debug_uartb_device.dev.platform_data))->uartclk;

		break;

	case 2:
		/* UARTC is the debug port. */
		pr_info("Selecting UARTC as the debug console\n");
		paz00_uart_devices[2] = &debug_uartc_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartc");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->mapbase;
		debug_uart_port_clk_rate = ((struct plat_serial8250_port *)(
			debug_uartc_device.dev.platform_data))->uartclk;

		break;

	case 3:
		/* UARTD is the debug port. */
		pr_info("Selecting UARTD as the debug console\n");
		paz00_uart_devices[3] = &debug_uartd_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uartd");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->mapbase;
		debug_uart_port_clk_rate = ((struct plat_serial8250_port *)(
			debug_uartd_device.dev.platform_data))->uartclk;

		break;

	default:
		pr_info("The debug console id %d is invalid, Assuming UARTA",
			debug_port_id);
		paz00_uart_devices[0] = &debug_uarta_device;
		debug_uart_clk = clk_get_sys("serial8250.0", "uarta");
		debug_uart_port_base = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->mapbase;
		debug_uart_port_clk_rate = ((struct plat_serial8250_port *)(
			debug_uarta_device.dev.platform_data))->uartclk;

		break;
	}
}

static void __init paz00_uart_init(void)
{
	struct clk *c;
	int i;

	for (i = 0; i < ARRAY_SIZE(uart_parent_clk); ++i) {
		c = tegra_get_clock_by_name(uart_parent_clk[i].name);
		if (IS_ERR_OR_NULL(c)) {
			pr_err("Not able to get the clock for %s\n",
						uart_parent_clk[i].name);
			continue;
		}
		uart_parent_clk[i].parent_clk = c;
		uart_parent_clk[i].fixed_clk_rate = clk_get_rate(c);
	}
	paz00_uart_pdata.parent_clk_list = uart_parent_clk;
	paz00_uart_pdata.parent_clk_count = ARRAY_SIZE(uart_parent_clk);

	tegra_uarta_device.dev.platform_data = &paz00_uart_pdata;
	tegra_uartb_device.dev.platform_data = &paz00_uart_pdata;
	tegra_uartc_device.dev.platform_data = &paz00_uart_pdata;
	tegra_uartd_device.dev.platform_data = &paz00_uart_pdata;
	tegra_uarte_device.dev.platform_data = &paz00_uart_pdata;

	/* Register low speed only if it is selected */
	if (!is_tegra_debug_uartport_hs()) {

		uart_debug_init();

		/* Clock enable for the debug channel */
		if (!IS_ERR_OR_NULL(debug_uart_clk)) {

			pr_info("The debug console clock name is %s\n",
						debug_uart_clk->name);
			c = tegra_get_clock_by_name("pll_p");
			if (IS_ERR_OR_NULL(c))
				pr_err("Not getting the parent clock pll_p\n");
			else
				clk_set_parent(debug_uart_clk, c);

			clk_enable(debug_uart_clk);
			clk_set_rate(debug_uart_clk, debug_uart_port_clk_rate);
		} else {
			pr_err("Not getting the clock %s for debug console\n",
					debug_uart_clk->name);
		}
	}

	return platform_add_devices(paz00_uart_devices,
				 ARRAY_SIZE(paz00_uart_devices));
};

static struct tegra_utmip_config utmi_phy_config[] = {
	[0] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 15,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
        },
	[1] = {
			.hssync_start_delay = 9,
			.idle_wait_delay = 17,
			.elastic_limit = 16,
			.term_range_adj = 6,
			.xcvr_setup = 8,
			.xcvr_setup_offset = 0,
			.xcvr_use_fuses = 1,
			.xcvr_lsfslew = 2,
			.xcvr_lsrslew = 2,
	},
};

static struct tegra_ulpi_config ulpi_phy_config = {
	.reset_gpio = TEGRA_ULPI_RST,
	.clk = "cdev2",
};

static struct tegra_ehci_platform_data tegra_ehci_pdata[] = {
	[0] = {
			.phy_config = &utmi_phy_config[0],
			.operating_mode = TEGRA_USB_OTG,
			.power_down_on_bus_suspend = 1,
			.hotplug = 1,
			.default_enable = true,
	},
	[1] = {
			.phy_config = &ulpi_phy_config,
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.phy_type = TEGRA_USB_PHY_TYPE_LINK_ULPI,
			.default_enable = true,
	},
	[2] = {
			.phy_config = &utmi_phy_config[1],
			.operating_mode = TEGRA_USB_HOST,
			.power_down_on_bus_suspend = 1,
			.default_enable = true,
	},
};

static struct tegra_otg_platform_data tegra_otg_pdata = {
        .ehci_device = &tegra_ehci1_device,
        .ehci_pdata = &tegra_ehci_pdata[0],
};

static struct usb_phy_plat_data tegra_usb_phy_pdata[] = {
        [0] = {
                        .instance = 0,
                        .vbus_gpio = -1,
        },
        [1] = {
                        .instance = 1,
                        .vbus_gpio = -1,
        },
        [2] = {
                        .instance = 2,
                        .vbus_gpio = -1,
        },
};

static struct rfkill_gpio_platform_data wifi_rfkill_platform_data = {
	.name		= "wifi_rfkill",
	.reset_gpio	= TEGRA_WIFI_RST,
	.shutdown_gpio	= TEGRA_WIFI_PWRN,
	.type	= RFKILL_TYPE_WLAN,
};

static struct platform_device wifi_rfkill_device = {
	.name	= "rfkill_gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &wifi_rfkill_platform_data,
	},
};

static struct gpio_led gpio_leds[] = {
	{
		.name			= "wifi-led",
		.default_trigger	= "rfkill0",
		.gpio			= TEGRA_WIFI_LED,
	},
};

static struct gpio_led_platform_data gpio_led_info = {
	.leds		= gpio_leds,
	.num_leds	= ARRAY_SIZE(gpio_leds),
};

static struct platform_device leds_gpio = {
	.name	= "leds-gpio",
	.id	= -1,
	.dev	= {
		.platform_data = &gpio_led_info,
	},
};

#define NVEC_GPIO_BASE TEGRA_NR_GPIOS + 4 /* 4 is number of tps6586x gpios */
static int nvec_gpio_base = NVEC_GPIO_BASE;

static struct nvec_events_platform_data nvec_ev_pdata[] = {
	{
		.name = "lid switch",
		.input_type = EV_SW,
		.key_code = SW_LID,
		.status_mask = BIT(1),
		.enabled = true,
	},
	{
		.name = "power key",
		.input_type = EV_KEY,
		.key_code = KEY_POWER,
		.status_mask = BIT(7),
		.enabled = true,
	},
	{	/* keep last entry */
		.status_mask = 0,
	},
};

static struct mfd_cell paz00_nvec_devices[] = {
	{
		.name = "nvec-kbd",
		.id = 1,
	},
	{
		.name = "nvec-mouse",
		.id = 1,
	},
	{
		.name = "nvec-power",
		.id = 1,
	},
	{
		.name = "nvec-power",
		.id = 2,
	},
	{
		.name = "nvec-paz00",
		.id = 1,
		.platform_data = &nvec_gpio_base,
		.pdata_size = sizeof(int),
	},
	{
		.name = "nvec-event",
		.id = 1,
		.platform_data = &nvec_ev_pdata,
		.pdata_size = sizeof(nvec_ev_pdata),
	},
};

static struct nvec_platform_data nvec_pdata = {
	.i2c_addr	= 0x8a,
	.gpio		= TEGRA_NVEC_REQ,
	.nvec_devices	= paz00_nvec_devices,
	.nr_nvec_devs	= ARRAY_SIZE(paz00_nvec_devices),
	.has_poweroff	= true,
};

static struct resource i2c_resource3[] = {
	[0] = {
		.start	= INT_I2C3,
		.end	= INT_I2C3,
		.flags	= IORESOURCE_IRQ,
	},
	[1] = {
		.start	= TEGRA_I2C3_BASE,
		.end	= TEGRA_I2C3_BASE + TEGRA_I2C3_SIZE-1,
		.flags	= IORESOURCE_MEM,
	},
};

static struct platform_device nvec_device = {
	.name		= "nvec",
	.id		= 2,
	.resource	= i2c_resource3,
	.num_resources	= ARRAY_SIZE(i2c_resource3),
	.dev		= {
		.platform_data = &nvec_pdata,
	}
};

static struct gpio_keys_button paz00_gpio_keys_buttons[] = {
	{
		.code		= KEY_POWER,
		.gpio		= TEGRA_GPIO_POWERKEY,
		.active_low	= 1,
		.desc		= "Power",
		.type		= EV_KEY,
		.wakeup		= 1,
	},
};

#define PMC_WAKE_STATUS 0x14

static int paz00_wakeup_key(void)
{
	unsigned long status =
		readl(IO_ADDRESS(TEGRA_PMC_BASE) + PMC_WAKE_STATUS);

	return (status & (1 << TEGRA_WAKE_GPIO_PJ7)) ?
		KEY_POWER : KEY_RESERVED;
}

static struct gpio_keys_platform_data paz00_gpio_keys = {
	.buttons	= paz00_gpio_keys_buttons,
	.nbuttons	= ARRAY_SIZE(paz00_gpio_keys_buttons),
	.wakeup_key	= paz00_wakeup_key,
};

static struct platform_device gpio_keys_device = {
	.name	= "gpio-keys",
	.id	= -1,
	.dev	= {
		.platform_data = &paz00_gpio_keys,
	},
};

static struct tegra_alc5632_audio_platform_data audio_pdata = {
	/* speaker enable goes via nvec */
	.gpio_hp_det	= TEGRA_HP_DET,
	.gpio_spk_en	= NVEC_GPIO_BASE,
};

static struct platform_device audio_device = {
	.name	= "tegra-alc5632",
	.id	= 0,
	.dev	= {
		.platform_data = &audio_pdata,
	},
};

static struct i2c_board_info __initdata alc5632_board_info = {
	I2C_BOARD_INFO("alc5632", 0x1e),
};

static struct i2c_board_info __initdata adt7421_board_info = {
	I2C_BOARD_INFO("adt7461", 0x4c),
};

static struct tegra_i2c_platform_data paz00_i2c1_platform_data = {
	.adapter_nr	= 0,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.slave_addr	= 0x00fc,
};

static const struct tegra_pingroup_config i2c2_ddc = {
	.pingroup	= TEGRA_PINGROUP_DDC,
	.func		= TEGRA_MUX_I2C2,
};

static const struct tegra_pingroup_config i2c2_gen2 = {
	.pingroup	= TEGRA_PINGROUP_PTA,
	.func		= TEGRA_MUX_I2C2,
};

static struct tegra_i2c_platform_data paz00_i2c2_platform_data = {
	.adapter_nr	= 1,
	.bus_count	= 2,
	.bus_clk_rate	= { 100000, 100000 },
	.bus_mux	= { &i2c2_ddc, &i2c2_gen2 },
	.bus_mux_len	= { 1, 1 },
	.slave_addr	= 0x00fc,
};

static struct tegra_i2c_platform_data paz00_dvc_platform_data = {
	.adapter_nr	= 4,
	.bus_count	= 1,
	.bus_clk_rate	= { 400000, 0 },
	.is_dvc		= true,
};

static struct platform_device *paz00_devices[] __initdata = {
	&nvec_device,
	&tegra_sdhci_device4,
	&tegra_sdhci_device1,
	&tegra_pmu_device,
	&wifi_rfkill_device,
	&leds_gpio,
	&gpio_keys_device,
	&tegra_gart_device,
	&audio_device,
	&tegra_i2s_device1,
	&tegra_spdif_device,
	&tegra_das_device,
	&spdif_dit_device,
	&tegra_pcm_device,
	&tegra_avp_device,
};

static void paz00_i2c_init(void)
{
	tegra_i2c_device1.dev.platform_data = &paz00_i2c1_platform_data;
	tegra_i2c_device2.dev.platform_data = &paz00_i2c2_platform_data;
	tegra_i2c_device4.dev.platform_data = &paz00_dvc_platform_data;

	platform_device_register(&tegra_i2c_device1);
	platform_device_register(&tegra_i2c_device2);
	platform_device_register(&tegra_i2c_device4);

	i2c_register_board_info(0, &alc5632_board_info, 1);
	i2c_register_board_info(4, &adt7421_board_info, 1);
}

static void paz00_usb_init(void)
{
	tegra_usb_phy_init(tegra_usb_phy_pdata, ARRAY_SIZE(tegra_usb_phy_pdata));
        /* OTG should be the first to be registered */
//        tegra_otg_device.dev.platform_data = &tegra_otg_pdata;
//        platform_device_register(&tegra_otg_device);

        platform_device_register(&tegra_udc_device);

	tegra_ehci2_device.dev.platform_data = &tegra_ehci_pdata[1];
	tegra_ehci3_device.dev.platform_data = &tegra_ehci_pdata[2];

	platform_device_register(&tegra_ehci2_device);
	platform_device_register(&tegra_ehci3_device);
}

static void __init tegra_paz00_fixup(struct machine_desc *desc,
	struct tag *tags, char **cmdline, struct meminfo *mi)
{
	mi->nr_banks = 1;
	mi->bank[0].start = PHYS_OFFSET;
	mi->bank[0].size = SZ_512M;
}

static __initdata struct tegra_clk_init_table paz00_clk_init_table[] = {
	/* name		parent		rate		enabled */
	{ "uarta",	"pll_p",	216000000,	true  },
	{ "uartc",	"pll_p",	216000000,	true  },

	{ "pll_p_out4",	"pll_p",	24000000,	true  },
	{ "usbd",	"clk_m",	12000000,	false },
	{ "usb2",	"clk_m",	12000000,	false },
	{ "usb3",	"clk_m",	12000000,	false },

	{ "pwm",	"clk_32k",	32768,		false },

	{ "pll_a",	"pll_p_out1",	56448000,	true  },
	{ "pll_a_out0",	"pll_a",	11289600,	true  },
	{ "cdev1",	NULL,		0,		true  },
	{ "audio",	"pll_a_out0",	11289600,	true  },
	{ "audio_2x",	"audio",	22579200,	false },
	{ "i2s1",	"pll_a_out0",	0	,	false },
	{ "spdif_out",  "pll_a_out0",   0,              false},

	{ NULL,		NULL,		0,		0     },
};

static struct tegra_sdhci_platform_data sdhci_pdata1 = {
	.cd_gpio	= TEGRA_GPIO_SD1_CD,
	.wp_gpio	= TEGRA_GPIO_SD1_WP,
	.power_gpio	= TEGRA_GPIO_SD1_POWER,
};

static struct tegra_sdhci_platform_data sdhci_pdata4 = {
	.cd_gpio	= -1,
	.wp_gpio	= -1,
	.power_gpio	= -1,
	.is_8bit	= 1,
	.mmc_data = {
		.built_in = 1,
	},
};

static void __init tegra_paz00_init(void)
{
	tegra_clk_init_from_table(paz00_clk_init_table);

	paz00_pinmux_init();

	tegra_sdhci_device1.dev.platform_data = &sdhci_pdata1;
	tegra_sdhci_device4.dev.platform_data = &sdhci_pdata4;

	platform_add_devices(paz00_devices, ARRAY_SIZE(paz00_devices));

	paz00_emc_init();
	paz00_i2c_init();
	paz00_regulator_init();
	paz00_suspend_init();
	paz00_uart_init();
	paz00_panel_init();
	paz00_usb_init();
	tegra_ram_console_debug_init();
}

static void __init paz00_ramconsole_reserve(unsigned long size)
{
        tegra_ram_console_debug_reserve(SZ_1M);
}

void __init tegra_paz00_reserve(void)
{
	if (memblock_reserve(0x0, 4096) < 0)
		pr_warn("Cannot reserve first 4K of memory for safety\n");

	tegra_reserve(SZ_128M, SZ_8M, SZ_16M);
	paz00_ramconsole_reserve(SZ_1M);
}

MACHINE_START(PAZ00, "paz00")
	.boot_params	= 0x00000100,
	.fixup		= tegra_paz00_fixup,
	.map_io         = tegra_map_common_io,
	.reserve	= tegra_paz00_reserve,
	.init_early	= tegra_init_early,
	.init_irq       = tegra_init_irq,
	.timer          = &tegra_timer,
	.init_machine   = tegra_paz00_init,
MACHINE_END

/* linux/arch/arm/mach-exynos/mach-kcppk.c
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/platform_device.h>
#include <linux/serial_core.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_gpio.h>
#include <linux/clk.h>
#include <linux/lcd.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>
#include <linux/pwm_backlight.h>
#include <linux/input.h>
#include <linux/mmc/host.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/fixed.h>
#include <linux/v4l2-mediabus.h>
#include <linux/memblock.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include <asm/mach/arch.h>
#include <asm/mach-types.h>

#include <plat/regs-serial.h>
#include <plat/exynos4.h>
#include <plat/cpu.h>
#include <plat/clock.h>
#include <plat/keypad.h>
#include <plat/devs.h>
#include <plat/fb.h>
#include <plat/fb-s5p.h>
#include <plat/fb-core.h>
#include <plat/regs-fb-v4.h>
#include <plat/backlight.h>
#include <plat/gpio-cfg.h>
#include <plat/regs-adc.h>
#include <plat/adc.h>
#include <plat/iic.h>
#include <plat/pd.h>
#include <plat/sdhci.h>
#include <plat/mshci.h>
#include <plat/ehci.h>
#include <plat/usbgadget.h>
#include <plat/s3c64xx-spi.h>
#if defined(CONFIG_VIDEO_FIMC)
#include <plat/fimc.h>
#endif
#if defined(CONFIG_VIDEO_FIMC_MIPI)
#include <plat/csis.h>
#endif
#include <plat/tvout.h>
#include <plat/media.h>
#include <plat/regs-srom.h>
#include <plat/sysmmu.h>
#include <plat/tv-core.h>
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC) || defined(CONFIG_VIDEO_MFC5X)
#include <plat/s5p-mfc.h>
#endif

#include <media/exynos_flite.h>
#include <media/exynos_fimc_is.h>
#include <video/platform_lcd.h>
#include <mach/board_rev.h>
#include <mach/map.h>
#include <mach/spi-clocks.h>
#include <mach/exynos-ion.h>
#include <mach/regs-pmu.h>


#ifdef CONFIG_EXYNOS4_DEV_DWMCI
#include <linux/scatterlist.h>
#include <mach/dwmci.h>
#endif

#include <mach/dev.h>
#include <mach/ppmu.h>


#include <plat/fimg2d.h>
#include <mach/dev-sysmmu.h>

#ifdef CONFIG_VIDEO_JPEG_V2X
#include <plat/jpeg.h>
#endif
#ifdef CONFIG_REGULATOR_S5M8767
#include <linux/mfd/s5m87xx/s5m-core.h>
#include <linux/mfd/s5m87xx/s5m-pmic.h>
#endif
#if defined(CONFIG_EXYNOS_THERMAL)
#include <mach/tmu.h>
#endif
#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
#include <mach/gpio-kcppk.h>
#endif

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
#include <linux/wakelock.h>
#include <linux/hrtimer.h>
#include<linux/switch.h>
#include <linux/mfd/kcppk/kcppk_buttons.h>
#include <linux/leds_pwm.h>
#endif

#define REG_INFORM4            (S5P_INFORM4)

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define KCPPK_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define KCPPK_ULCON_DEFAULT	S3C2410_LCON_CS8

#define KCPPK_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)



static struct s3c2410_uartcfg kcppk_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= KCPPK_UCON_DEFAULT,
		.ulcon		= KCPPK_ULCON_DEFAULT,
		.ufcon		= KCPPK_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= KCPPK_UCON_DEFAULT,
		.ulcon		= KCPPK_ULCON_DEFAULT,
		.ufcon		= KCPPK_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= KCPPK_UCON_DEFAULT,
		.ulcon		= KCPPK_ULCON_DEFAULT,
		.ufcon		= KCPPK_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= KCPPK_UCON_DEFAULT,
		.ulcon		= KCPPK_ULCON_DEFAULT,
		.ufcon		= KCPPK_UFCON_DEFAULT,
	},
};


#define WRITEBACK_ENABLED

#define CAM_CHECK_ERR_RET(x, msg)					\
	if (unlikely((x) < 0)) {					\
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x);	\
		return x;						\
	}
#define CAM_CHECK_ERR(x, msg)						\
	if (unlikely((x) < 0)) {					\
		printk(KERN_ERR "\nfail to %s: err = %d\n", msg, x);	\
	}
#define CAM_CHECK_ERR_GOTO(x, out, fmt, ...) \
	if (unlikely((x) < 0)) { \
		printk(KERN_ERR fmt, ##__VA_ARGS__); \
		goto out; \
	}
#if CONFIG_MFD_SOAP_KCPPK_BUTTONS
#include <media/adv7280.h>

#define ADV7280_LINE_LENGTH	714
#define ADV7280_RESOLUTION_W	710
#define ADV7280_RESOLUTION_H	942
#define ADV7280_PREVIEW_W	714  
#define ADV7280_PREVIEW_H	942 

static struct adv7280_platform_data ADV7280_plat = {
    .default_width = ADV7280_RESOLUTION_W,
    .default_height = ADV7280_RESOLUTION_H,
    .pixelformat = V4L2_PIX_FMT_YUYV,   
    .freq = 27000000,
    .is_mipi = 0,
 //   .csi_tx_addr = 0x51,
};

struct s3c2410_platform_i2c ADV7280_i2c1_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 100*1000,
	.sda_delay	= 100,
	.bus_num 	= 1,
};


static struct i2c_board_info ADV7280_i2c_info = {
    I2C_BOARD_INFO("adv7280", 0x42 >> 1), 
    .platform_data = &ADV7280_plat,
};

int return_value_for_bus_num(void) {
	return 1;
}

static struct s3c_platform_camera ADV7280 = {
        .id             = CAMERA_PAR_A, //CAMERA_CSI_C, CAMERA_PAR_A
        .clk_name       = "sclk_cam0",
        .i2c_busnum     = 1,
        .get_i2c_busnum = return_value_for_bus_num,
        .type           = CAM_TYPE_ITU,
        .fmt            = ITU_656_YCBCR422_8BIT, 
        .order422	= CAM_ORDER422_8BIT_CBYCRY, 
        .info           = &ADV7280_i2c_info,
        .pixelformat    = V4L2_PIX_FMT_YUYV,        
        .srclk_name     = "xusbxti",
        .clk_rate       = 14318180,       
        .line_length    = ADV7280_LINE_LENGTH,
        .width          = ADV7280_PREVIEW_W,
        .height         = ADV7280_PREVIEW_H,
        .window         = {
                .left   = 0,
                .top    = 8,
                .width  = ADV7280_PREVIEW_W,
                .height = ADV7280_PREVIEW_H,
        },
        /* Polarity */
		.inv_pclk	= 1,
		.inv_vsync	= 0,
		.inv_href	= 0,
		.inv_hsync	= 0,
        .initialized    = 0,
};

#endif

#ifdef WRITEBACK_ENABLED
static struct i2c_board_info writeback_i2c_info = {
	I2C_BOARD_INFO("WriteBack", 0x0),
};

static struct s3c_platform_camera writeback = {
	.id		= CAMERA_WB,
	.fmt		= ITU_601_YCBCR422_8BIT,
	.order422	= CAM_ORDER422_8BIT_CBYCRY,
	.i2c_busnum	= 0,
	.info		= &writeback_i2c_info,
	.pixelformat	= V4L2_PIX_FMT_YUV444,
	.line_length	= 800,
	.width		= 480,
	.height		= 800,
	.window		= {
		.left	= 0,
		.top	= 0,
		.width	= 480,
		.height	= 800,
	},

	.initialized	= 0,
};
#endif

/* Interface setting */
static struct s3c_platform_fimc fimc_plat = {
#ifdef CONFIG_ITU_A
		.default_cam	= CAMERA_PAR_A,
#endif
#ifdef CONFIG_ITU_B
		.default_cam	= CAMERA_PAR_B,
#endif
#ifdef CONFIG_CSI_C
		.default_cam	= CAMERA_CSI_C,
#endif
#ifdef CONFIG_CSI_D
		.default_cam	= CAMERA_CSI_D,
#endif
#ifdef WRITEBACK_ENABLED
	.default_cam	= CAMERA_WB,
#endif
	.camera		= {
#if defined(CONFIG_VIDEO_ADV7280)
        &ADV7280,
#endif
#ifdef WRITEBACK_ENABLED
		&writeback,
#endif
	},
	.hw_ver		= 0x51,
};


static int exynos4_notifier_call(struct notifier_block *this,
					unsigned long code, void *_cmd)
{
	int mode = 0;

	if ((code == SYS_RESTART) && _cmd) {
		if (!strcmp((char *)_cmd, "recovery"))
			mode = 0xf;
		else if(!strcmp((char *)_cmd, "fastboot")) //leehc add reboot fastboot mode
			mode = 0xfa;
		else if(!strcmp((char *)_cmd, "uboot"))
			mode = 0xfb;
		else if(!strcmp((char *)_cmd, "erase"))
			mode = 0xfc;
		else if(!strcmp((char *)_cmd, "wipe"))
			mode = 0xff;
		
		
	}
	printk(KERN_ERR "%s - reboot mode %x\n", __func__, mode);

	__raw_writel(mode, REG_INFORM4);

	return NOTIFY_DONE;
}

static struct notifier_block exynos4_reboot_notifier = {
	.notifier_call = exynos4_notifier_call,
};

#ifdef CONFIG_EXYNOS4_DEV_DWMCI
static void exynos_dwmci_cfg_gpio(int width)
{
	unsigned int gpio;

	for (gpio = EXYNOS4_GPK0(0); gpio < EXYNOS4_GPK0(2); gpio++) {
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_NONE);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	}

	switch (width) {
	case 8:
		for (gpio = EXYNOS4_GPK1(3); gpio <= EXYNOS4_GPK1(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(4));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
	case 4:
		for (gpio = EXYNOS4_GPK0(3); gpio <= EXYNOS4_GPK0(6); gpio++) {
			s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
			s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
			s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
		}
		break;
	case 1:
		gpio = EXYNOS4_GPK0(3);
		s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(3));
		s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
		s5p_gpio_set_drvstr(gpio, S5P_GPIO_DRVSTR_LV2);
	default:
		break;
	}
}

static struct dw_mci_board exynos_dwmci_pdata __initdata = {
	.num_slots		= 1,
	.quirks			= DW_MCI_QUIRK_BROKEN_CARD_DETECTION | DW_MCI_QUIRK_HIGHSPEED,
	.bus_hz			= 100 * 1000 * 1000,
	.caps			= MMC_CAP_UHS_DDR50 | MMC_CAP_8_BIT_DATA | MMC_CAP_CMD23,
	.fifo_depth		= 0x80,
	.detect_delay_ms	= 200,
	.hclk_name		= "dwmci",
	.cclk_name		= "sclk_dwmci",
	.cfg_gpio		= exynos_dwmci_cfg_gpio,
};
#endif

#ifdef CONFIG_S3C_DEV_HSMMC2
static struct s3c_sdhci_platdata kcppk_hsmmc2_pdata __initdata = {
	.cd_type		= S3C_SDHCI_CD_INTERNAL,
	.clk_type		= S3C_SDHCI_CLK_DIV_EXTERNAL,
#ifdef CONFIG_EXYNOS4_SDHCI_CH2_8BIT
	.max_width		= 8,
	.host_caps		= MMC_CAP_8_BIT_DATA,
#endif
};
#endif


#ifdef CONFIG_USB_EHCI_S5P
static struct s5p_ehci_platdata kcppk_ehci_pdata;

static void __init kcppk_ehci_init(void)
{
	struct s5p_ehci_platdata *pdata = &kcppk_ehci_pdata;

	s5p_ehci_set_platdata(pdata);
}
#endif

#ifdef CONFIG_USB_OHCI_S5P
static struct s5p_ohci_platdata kcppk_ohci_pdata;

static void __init kcppk_ohci_init(void)
{
	struct s5p_ohci_platdata *pdata = &kcppk_ohci_pdata;

	s5p_ohci_set_platdata(pdata);
}
#endif

/* USB GADGET */
#ifdef CONFIG_USB_GADGET
static struct s5p_usbgadget_platdata kcppk_usbgadget_pdata;

static void __init kcppk_usbgadget_init(void)
{
	struct s5p_usbgadget_platdata *pdata = &kcppk_usbgadget_pdata;

	s5p_usbgadget_set_platdata(pdata);
}
#endif

#ifdef CONFIG_REGULATOR_S5M8767
/* S5M8767 Regulator */
static int s5m_cfg_irq(void)
{	
	s3c_gpio_cfgpin(GPIO_PMIC_INT, S3C_GPIO_SFN(0xF));
	s3c_gpio_setpull(GPIO_PMIC_INT, S3C_GPIO_PULL_UP);
	return 0;
}
static struct regulator_consumer_supply s5m8767_ldo1_supply[] = {
	REGULATOR_SUPPLY("vdd_alive", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo2_supply[] = {
	REGULATOR_SUPPLY("vddq_m12", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo3_supply[] = {
	REGULATOR_SUPPLY("vddioap_18", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo4_supply[] = {
	REGULATOR_SUPPLY("vddq_pre", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo5_supply[] = {
	REGULATOR_SUPPLY("VDD_IO_18", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo6_supply[] = {
	REGULATOR_SUPPLY("vdd10_mpll", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo7_supply[] = {
	REGULATOR_SUPPLY("vdd10_xpll", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo8_supply[] = {
	REGULATOR_SUPPLY("vdd10_mipi", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo9_supply[] = {
	REGULATOR_SUPPLY("vdd33_lcd", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo10_supply[] = {
	REGULATOR_SUPPLY("vdd18_mipi", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo11_supply[] = {
	REGULATOR_SUPPLY("vdd18_abb1", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo12_supply[] = {
	REGULATOR_SUPPLY("vdd33_uotg", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo13_supply[] = {
	REGULATOR_SUPPLY("CAM_IO_1V8", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo14_supply[] = {
	REGULATOR_SUPPLY("vdd18_abb02", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo15_supply[] = {
	REGULATOR_SUPPLY("vdd10_ush", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo16_supply[] = {
	REGULATOR_SUPPLY("vdd18_hsic", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo17_supply[] = {
	REGULATOR_SUPPLY("vmmc", "dw_mmc"),
};
//static struct regulator_consumer_supply s5m8767_ldo18_supply[] = {
//	REGULATOR_SUPPLY("vmmc", "s3c-sdhci.2"),
//};
static struct regulator_consumer_supply s5m8767_ldo18_supply[] = {
	REGULATOR_SUPPLY("bt_3v3", NULL),
};

static struct regulator_consumer_supply s5m8767_ldo19_supply[] = {
	REGULATOR_SUPPLY("CAM_2V8", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo20_supply[] = {
	REGULATOR_SUPPLY("aud_3v0", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo21_supply[] = {
	REGULATOR_SUPPLY("vdd28_af", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo22_supply[] = {
	REGULATOR_SUPPLY("gps_3v3", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo23_supply[] = {
	REGULATOR_SUPPLY("AF_3V3", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo24_supply[] = {
	REGULATOR_SUPPLY("vdd33_a31", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo25_supply[] = {
	REGULATOR_SUPPLY("HUB0_1V2", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo26_supply[] = {
	REGULATOR_SUPPLY("MAC_2V0", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo27_supply[] = {
	REGULATOR_SUPPLY("aud_1v8", NULL),
};
static struct regulator_consumer_supply s5m8767_ldo28_supply[] = {
	REGULATOR_SUPPLY("MOTOR_1V3", NULL),
};

static struct regulator_consumer_supply s5m8767_buck1_consumer =
	REGULATOR_SUPPLY("vdd_mif", NULL);

static struct regulator_consumer_supply s5m8767_buck2_consumer =
	REGULATOR_SUPPLY("vdd_arm", NULL);

static struct regulator_consumer_supply s5m8767_buck3_consumer =
	REGULATOR_SUPPLY("vdd_int", NULL);

static struct regulator_consumer_supply s5m8767_buck4_consumer =
	REGULATOR_SUPPLY("vdd_g3d", NULL);
static struct regulator_consumer_supply s5m8767_buck5_consumer =
	REGULATOR_SUPPLY("vdd_m12", NULL);
static struct regulator_consumer_supply s5m8767_buck6_consumer =
	REGULATOR_SUPPLY("CAM_1V5", NULL);
static struct regulator_consumer_supply s5m8767_buck9_consumer =
	REGULATOR_SUPPLY("vddf28_emmc", NULL);

#define REGULATOR_INIT(_ldo, _name, _min_uV, _max_uV, _always_on, _ops_mask,\
		_disabled) \
	static struct regulator_init_data s5m8767_##_ldo##_init_data = {		\
		.constraints = {					\
			.name	= _name,				\
			.min_uV = _min_uV,				\
			.max_uV = _max_uV,				\
			.always_on	= _always_on,			\
			.boot_on	= _always_on,			\
			.apply_uV	= 1,				\
			.valid_ops_mask = _ops_mask,			\
			.state_mem	= {				\
				.disabled	= _disabled,		\
				.enabled	= !(_disabled),		\
			}						\
		},							\
		.num_consumer_supplies = ARRAY_SIZE(s5m8767_##_ldo##_supply),	\
		.consumer_supplies = &s5m8767_##_ldo##_supply[0],			\
	};

REGULATOR_INIT(ldo1, "VDD_ALIVE", 1100000, 1100000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo2, "VDDQ_M12", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo3, "VDDIOAP_18", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo4, "VDDQ_PRE", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo5, "VDD_IO_18", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo6, "VDD10_MPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo7, "VDD10_XPLL", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo8, "VDD10_MIPI", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo9, "LAN_3V3", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo10, "VDD18_MIPI", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo11, "VDD18_ABB1", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo12, "VDD33_UOTG", 3000000, 3000000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo13, "CAM_IO_1V8", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo14, "VDD18_ABB02", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo15, "VDD10_USH", 1000000, 1000000, 1,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo16, "VDD18_HSIC", 1800000, 1800000, 1,
		REGULATOR_CHANGE_STATUS, 1);
//REGULATOR_INIT(ldo17, "dw-mmc", 2800000, 2800000, 1,
//		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo17, "dw-mmc", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 0);

//REGULATOR_INIT(ldo18, "s3c-sdhci.2", 3300000, 3300000, 0,
//		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo18, "BT_3V3", 3300000, 3300000, 0,
		REGULATOR_CHANGE_STATUS, 0);

REGULATOR_INIT(ldo19, "CAM_2V8", 2800000, 2800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
//REGULATOR_INIT(ldo20, "VDD28_CAM", 3000000, 3000000, 1,
//		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo20, "AUD_3V0", 3000000, 3000000, 0,
		REGULATOR_CHANGE_STATUS, 1);

REGULATOR_INIT(ldo21, "VDD28_AF", 3000000, 3000000, 0,
		REGULATOR_CHANGE_STATUS, 1);
//REGULATOR_INIT(ldo22, "VDDA28_2M", 3300000, 3300000, 1,
//		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo22, "GPS_3V3", 3300000, 3300000, 1,
		REGULATOR_CHANGE_STATUS, 1);

REGULATOR_INIT(ldo23, "AF_3V3", 3300000, 3300000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo24, "VDD33_A31", 3000000, 3000000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo25, "HUB0_1V2", 1200000, 1200000, 1,
		REGULATOR_CHANGE_STATUS, 0);
REGULATOR_INIT(ldo26, "MAC_2V0", 2000000, 2000000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo27, "AUD_1V8", 1800000, 1800000, 0,
		REGULATOR_CHANGE_STATUS, 1);
REGULATOR_INIT(ldo28, "MOTOR_1V3", 1300000, 1300000, 0,
		REGULATOR_CHANGE_STATUS, 1);

static struct regulator_init_data s5m8767_buck1_data = {
	.constraints	= {
		.name		= "vdd_mif range",
		.min_uV		= 800000,
		.max_uV		= 1100000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck1_consumer,
};

static struct regulator_init_data s5m8767_buck2_data = {
	.constraints	= {
		.name		= "vdd_arm range",
		.min_uV		=  800000,
		.max_uV		= 1350000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				  REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck2_consumer,
};

static struct regulator_init_data s5m8767_buck3_data = {
	.constraints	= {
		.name		= "vdd_int range",
		.min_uV		=  800000,
		.max_uV		= 1150000,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.uV		= 1100000,
			.mode		= REGULATOR_MODE_NORMAL,
			.disabled	= 1,
		},
	},
	.num_consumer_supplies	= 1,
	.consumer_supplies	= &s5m8767_buck3_consumer,
};

static struct regulator_init_data s5m8767_buck4_data = {
	.constraints	= {
		.name		= "vdd_g3d range",
		.min_uV		= 850000,
		.max_uV		= 1200000,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.always_on = 1,
		.boot_on = 1,
		.state_mem	= {
			.disabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck4_consumer,
};

static struct regulator_init_data s5m8767_buck5_data = {
	.constraints	= {
		.name		= "vdd_m12 range",
		.min_uV		= 750000,
		.max_uV		= 1500000,
		.apply_uV	= 1,
	//	.boot_on	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.enabled	= 1,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck5_consumer,
};

static struct regulator_init_data s5m8767_buck6_data = {
	.constraints	= {
		.name		= "CAM_1V5",
		.min_uV		= 1500000,
		.max_uV		= 1500000,
//		.boot_on	= 1,
//		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE |
				REGULATOR_CHANGE_STATUS,
		.state_mem	= {
			.disabled	= 1,
			.uV	= 1500000,
			.mode	= REGULATOR_MODE_NORMAL,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck6_consumer,
};

static struct regulator_init_data s5m8767_buck9_data = {
	.constraints	= {
		.name		= "vddf28_emmc range",
		.min_uV		= 2800000,
		.max_uV		= 2800000,
		.boot_on	= 1,
		.apply_uV	= 1,
		.valid_ops_mask	= REGULATOR_CHANGE_VOLTAGE,
		.state_mem	= {
			.disabled	= 1,				
			.uV	= 2800000,
			.mode	= REGULATOR_MODE_NORMAL,
		},
	},
	.num_consumer_supplies = 1,
	.consumer_supplies = &s5m8767_buck9_consumer,
};

static struct s5m_regulator_data pegasus_regulators[] = {
	{ S5M8767_BUCK1, &s5m8767_buck1_data },
	{ S5M8767_BUCK2, &s5m8767_buck2_data },
	{ S5M8767_BUCK3, &s5m8767_buck3_data },
	{ S5M8767_BUCK4, &s5m8767_buck4_data },
	{ S5M8767_BUCK5, &s5m8767_buck5_data },
	{ S5M8767_BUCK6, &s5m8767_buck6_data },
	{ S5M8767_BUCK9, &s5m8767_buck9_data },

	{ S5M8767_LDO1, &s5m8767_ldo1_init_data },
	{ S5M8767_LDO2, &s5m8767_ldo2_init_data },
	{ S5M8767_LDO3, &s5m8767_ldo3_init_data },
	{ S5M8767_LDO4, &s5m8767_ldo4_init_data },
	{ S5M8767_LDO5, &s5m8767_ldo5_init_data },
	{ S5M8767_LDO6, &s5m8767_ldo6_init_data },
	{ S5M8767_LDO7, &s5m8767_ldo7_init_data },
	{ S5M8767_LDO8, &s5m8767_ldo8_init_data },
	{ S5M8767_LDO9, &s5m8767_ldo9_init_data },
	{ S5M8767_LDO10, &s5m8767_ldo10_init_data },

	{ S5M8767_LDO11, &s5m8767_ldo11_init_data },
	{ S5M8767_LDO12, &s5m8767_ldo12_init_data },
	{ S5M8767_LDO13, &s5m8767_ldo13_init_data },
	{ S5M8767_LDO14, &s5m8767_ldo14_init_data },
	{ S5M8767_LDO15, &s5m8767_ldo15_init_data },
	{ S5M8767_LDO16, &s5m8767_ldo16_init_data },
	{ S5M8767_LDO17, &s5m8767_ldo17_init_data },
	{ S5M8767_LDO18, &s5m8767_ldo18_init_data },
	{ S5M8767_LDO19, &s5m8767_ldo19_init_data },
	{ S5M8767_LDO20, &s5m8767_ldo20_init_data },

	{ S5M8767_LDO21, &s5m8767_ldo21_init_data },
	{ S5M8767_LDO22, &s5m8767_ldo22_init_data },
	{ S5M8767_LDO23, &s5m8767_ldo23_init_data },
	{ S5M8767_LDO24, &s5m8767_ldo24_init_data },
	{ S5M8767_LDO25, &s5m8767_ldo25_init_data },
	{ S5M8767_LDO26, &s5m8767_ldo26_init_data },
	{ S5M8767_LDO27, &s5m8767_ldo27_init_data },
	{ S5M8767_LDO28, &s5m8767_ldo28_init_data },
};

struct s5m_opmode_data s5m8767_opmode_data[S5M8767_REG_MAX] = {
	[S5M8767_BUCK1] = {S5M8767_BUCK1, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK2] = {S5M8767_BUCK2, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK3] = {S5M8767_BUCK3, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK4] = {S5M8767_BUCK4, S5M_OPMODE_STANDBY},
	[S5M8767_BUCK5] = {S5M8767_BUCK5, S5M_OPMODE_STANDBY},

	[S5M8767_BUCK9] = {S5M8767_BUCK9, S5M_OPMODE_STANDBY},
	//CAM
	[S5M8767_BUCK6] = {S5M8767_BUCK6, S5M_OPMODE_NORMAL},
	[S5M8767_LDO19] = {S5M8767_LDO19, S5M_OPMODE_NORMAL},
	[S5M8767_LDO23] = {S5M8767_LDO23, S5M_OPMODE_NORMAL},
	[S5M8767_LDO13] = {S5M8767_LDO13, S5M_OPMODE_NORMAL},
	//LAN
	[S5M8767_LDO9] = {S5M8767_LDO9, S5M_OPMODE_NORMAL},
};

static struct s5m_platform_data exynos4_s5m8767_pdata = {
	.device_type		= S5M8767X,
	.irq_base		= IRQ_BOARD_START,
	.num_regulators		= ARRAY_SIZE(pegasus_regulators),
	.regulators		= pegasus_regulators,
	.cfg_pmic_irq		= s5m_cfg_irq,
	.wakeup			= 1,
	.opmode_data		= s5m8767_opmode_data,
	.wtsr_smpl		= 1,

	.buck2_voltage[2]	= 1150000,
	.buck2_voltage[3]	= 1100000,
	.buck2_voltage[4]	= 1050000,
	.buck2_voltage[5]	= 1000000,
	.buck2_voltage[6]	=  950000,
	.buck2_voltage[7]	=  900000,

	.buck3_voltage[1]	= 1000000,
	.buck3_voltage[2]	= 950000,
	.buck3_voltage[3]	= 900000,
	.buck3_voltage[5]	= 1000000,
	.buck3_voltage[6]	= 950000,
	.buck3_voltage[7]	= 900000,

	.buck4_voltage[1]	= 1150000,
	.buck4_voltage[3]	= 1100000,
	.buck4_voltage[4]	= 1100000,
	.buck4_voltage[5]	= 1100000,
	.buck4_voltage[6]	= 1100000,
	.buck4_voltage[7]	= 1100000,

	.buck_default_idx	= 1,

	.buck_gpios[0]		= EXYNOS4_GPL0(3),
	.buck_gpios[1]		= EXYNOS4_GPL0(4),
	.buck_gpios[2]		= EXYNOS4_GPL0(6),

	.buck_ds[0]		= EXYNOS4_GPL0(0),
	.buck_ds[1]		= EXYNOS4_GPL0(1),
	.buck_ds[2]		= EXYNOS4_GPL0(2),	
	
	.buck_ramp_delay        = 25,
	.buck2_ramp_enable      = true,
	.buck3_ramp_enable      = true,
	.buck4_ramp_enable      = true,

	.buck2_init		= 1100000,
	.buck3_init		= 1000000,
	.buck4_init		= 1000000,
};
/* End of S5M8767 */
#endif

#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
static struct regulator_consumer_supply mipi_csi_fixed_voltage_supplies[] = {
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.0"),
	REGULATOR_SUPPLY("mipi_csi", "s5p-mipi-csis.1"),
};

static struct regulator_init_data mipi_csi_fixed_voltage_init_data = {
	.constraints = {
		.always_on = 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(mipi_csi_fixed_voltage_supplies),
	.consumer_supplies	= mipi_csi_fixed_voltage_supplies,
};

static struct fixed_voltage_config mipi_csi_fixed_voltage_config = {
	.supply_name	= "DC_5V",
	.microvolts	= 5000000,
	.gpio		= -EINVAL,
	.init_data	= &mipi_csi_fixed_voltage_init_data,
};

static struct platform_device mipi_csi_fixed_voltage = {
	.name		= "reg-fixed-voltage",
	.id		= 3,
	.dev		= {
		.platform_data	= &mipi_csi_fixed_voltage_config,
	},
};
#endif

static struct i2c_board_info i2c_devs0[] __initdata = {
#if 0 //def CONFIG_SND_SOC_WM8960
	{ I2C_BOARD_INFO("wm8960", 0x1a), },
#endif
#if 0 //def CONFIG_SND_SOC_WM1800
	{ I2C_BOARD_INFO("wm1800", 0x1a), },
#endif
#ifdef CONFIG_REGULATOR_S5M8767
	{
		I2C_BOARD_INFO("s5m87xx", 0xCC >> 1),
		.platform_data = &exynos4_s5m8767_pdata,
		.irq		= GPIO_PMIC_IRQ,
	},
#endif
};

static struct i2c_board_info i2c_devs1[] __initdata = {
};
#ifdef CONFIG_VIDEO_TVOUT
static struct i2c_board_info i2c_devs2[] __initdata = {
	{
		I2C_BOARD_INFO("s5p_ddc", (0x74 >> 1)),
	},
};
#endif

static struct i2c_board_info i2c_devs3[] __initdata = {
{
		I2C_BOARD_INFO("kcppk-light", (0xaa >> 1)),   //if AD pin == pull down: 0x52, pull-up: 0x55
		.irq		= GPIO_SI1133_INT,
}
};

struct s3c2410_platform_i2c i2c_3_data __initdata = {
	.flags		= 0,
	.slave_addr	= 0x10,
	.frequency	= 100*1000,
	.sda_delay	= 100,
	.bus_num 	= 3,
};

#ifdef CONFIG_TOUCH_SCREEN
static struct i2c_board_info i2c_devs5[] __initdata = {
};
#endif

static struct led_pwm kcppk_pwm_leds[] = {
	{
		.name = "f_led_pwm",
		.max_brightness = 64,
		.pwm_id = 0,
//		.pwm_period_ns = 16 * 1000 * 1000, //30Hz
		.pwm_period_ns = 1 * 1000 * 1000, //1Khz		
	},
};

static struct led_pwm_platform_data kcppk_led_pwm_info = {
	.leds = kcppk_pwm_leds,
	.num_leds = ARRAY_SIZE(kcppk_pwm_leds),
};

static struct platform_device kcppk_leds_pwm = {
	.name = "leds_pwm",
	.id   = -1,
	.dev  = {
		.platform_data = &kcppk_led_pwm_info,
	},
};

static int kcppk_led_pwm_gpio_enable(void){
	s3c_gpio_cfgpin(BUTTON_LED_PWM, S3C_GPIO_SFN(2));
	 return 0;
}

struct platform_device *get_bt_let_pdev() {
       return &kcppk_leds_pwm;
}

static struct gpio_led kcppk_gpio_leds[] = {
	{
		.name = "red",
		.default_trigger  = "timer",
		.gpio = EXYNOS4_GPL2(1),
	},
	{
		.name = "green",
		.default_trigger  = "timer",
		.gpio = EXYNOS4_GPL2(4),
	},
	{
		.name = "blue",
		.default_trigger  = "timer",
		.gpio = EXYNOS4_GPL2(0),
	},
};


static struct gpio_led_platform_data kcppk_gpio_led_info = {
	.leds = kcppk_gpio_leds,
	.num_leds = ARRAY_SIZE(kcppk_gpio_leds),
};

static struct platform_device kcppk_leds_gpio = {
	.name = "leds-gpio",
	.id   = -1,
	.dev  = {
		.platform_data = &kcppk_gpio_led_info,
	},
};

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
static struct platform_device kcppk_buttons_device = {
	.name = "kcppk_buttons_drv",
	.id   = -1,
};
#endif

#define GPIO_KEYS(_code, _gpio, _active_low, _iswake, _hook)	\
	{\
		.code = _code,\
		.gpio = _gpio,\
		.active_low = _active_low,\
		.type = EV_KEY,\
		.wakeup = _iswake,\
		.debounce_interval = 10,\
		.value = 0,\
	}

/*		.isr_hook = _hook,\ */
static struct gpio_keys_button kcppk_buttons[] = {
//	GPIO_KEYS(KEY_POWER, GPIO_POWER_KEY,  // 
//			1, 1, 0),
};

struct gpio_keys_platform_data kcppk_gpiokeys_platform_data = {
	.buttons = kcppk_buttons,
	.nbuttons = ARRAY_SIZE(kcppk_buttons),
};

static struct platform_device kcppk_keypad = {
	.name	= "gpio-keys",
	.id	= -1,
	.num_resources	= 0,
	.dev	= {
		.platform_data = &kcppk_gpiokeys_platform_data,
	},
};

static void __init kcppk_gpio_power_init(void)
{

	s3c_gpio_setpull(GPIO_POWER_KEY, S3C_GPIO_PULL_NONE);
	s3c_gpio_cfgpin(GPIO_POWER_KEY, S3C_GPIO_SFN(0xF));
}

#ifdef CONFIG_VIDEO_FIMG2D
static struct fimg2d_platdata fimg2d_data __initdata = {
	.hw_ver = 0x41,
	.parent_clkname = "mout_g2d0",
	.clkname = "sclk_fimg2d",
	.gate_clkname = "fimg2d",
	.clkrate = 201 * 1000000,	/* 200 Mhz */
};
#endif

#ifdef CONFIG_USB_EXYNOS_SWITCH
#include <plat/usb-switch.h>
static struct s5p_usbswitch_platdata kcppk_usbswitch_pdata;

static void __init kcppk_usbswitch_init(void)
{
	struct s5p_usbswitch_platdata *pdata = &kcppk_usbswitch_pdata;
	int err;

	pdata->gpio_device_detect = GPIO_OTG_EN_H; /* high active */
	err = gpio_request_one(pdata->gpio_device_detect, GPIOF_OUT_INIT_LOW, "DEVICE_DETECT");
	if (err) {
		printk(KERN_ERR "failed to request gpio_host_detect for\n");
		return;
	}

	s3c_gpio_cfgpin(pdata->gpio_device_detect, S3C_GPIO_OUTPUT);
	s3c_gpio_setpull(pdata->gpio_device_detect, S3C_GPIO_PULL_NONE);
	gpio_free(pdata->gpio_device_detect);

	s5p_usbswitch_set_platdata(pdata);
}
#endif

static struct platform_device exynos4_busfreq = {
	.id = -1,
	.name = "exynos-busfreq",
};

static struct platform_device *smdk4412_devices[] __initdata = {
	&s3c_device_adc,
};

static struct platform_device *kcppk_devices[] __initdata = {
	/* Samsung Power Domain */
	&exynos4_device_pd[PD_MFC],
	&exynos4_device_pd[PD_G3D],
	&exynos4_device_pd[PD_LCD0],
	&exynos4_device_pd[PD_CAM],
	&exynos4_device_pd[PD_TV],
	&exynos4_device_pd[PD_GPS],
	&exynos4_device_pd[PD_GPS_ALIVE],
	/* legacy fimd */
#ifdef CONFIG_FB_S5P
	&s3c_device_fb,
#endif
	&s3c_device_wdt,
	&s3c_device_rtc,
	&s3c_device_i2c0, //pmic
	&s3c_device_i2c1, //adv7280
    &s3c_device_i2c3, //lightsensor
#ifdef CONFIG_USB_EHCI_S5P
	&s5p_device_ehci,
#endif
#ifdef CONFIG_USB_OHCI_S5P
	&s5p_device_ohci,
#endif
#ifdef CONFIG_USB_GADGET
	&s3c_device_usbgadget,
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	&s3c_device_hsmmc2,
#endif
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	&exynos_device_dwmci,
#endif

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
#if defined(CONFIG_VIDEO_FIMC)
	&s3c_device_fimc0,
	&s3c_device_fimc1,
	&s3c_device_fimc2,
	&s3c_device_fimc3,
#endif
#if defined(CONFIG_VIDEO_FIMC_MIPI)
	&s3c_device_csis0,
	&s3c_device_csis1,
#endif

#if defined(CONFIG_VIDEO_MFC5X) || defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	&s5p_device_mfc,
#endif
#ifdef CONFIG_S5P_SYSTEM_MMU
	&SYSMMU_PLATDEV(g2d_acp),
	&SYSMMU_PLATDEV(fimc0),
	&SYSMMU_PLATDEV(fimc1),
	&SYSMMU_PLATDEV(fimc2),
	&SYSMMU_PLATDEV(fimc3),
	&SYSMMU_PLATDEV(jpeg),
	&SYSMMU_PLATDEV(mfc_l),
	&SYSMMU_PLATDEV(mfc_r),
	&SYSMMU_PLATDEV(tv),
#endif
#endif /* CONFIG_S5P_SYSTEM_MMU */
#ifdef CONFIG_VIDEO_FIMG2D
	&s5p_device_fimg2d,
#endif
#ifdef CONFIG_VIDEO_JPEG_V2X
	&s5p_device_jpeg,
#endif	
	&samsung_asoc_dma,
	&samsung_asoc_idma,
	&samsung_device_keypad,
#ifdef CONFIG_EXYNOS_THERMAL
	&exynos_device_tmu,
#endif
	&exynos4_busfreq,

};

#ifdef CONFIG_EXYNOS_THERMAL
/* below temperature base on the celcius degree */
struct tmu_data exynos_tmu_data __initdata = {
	.ts = {
		.stop_throttle  = 82,
		.start_throttle = 85,
		.stop_warning  = 97, // 102 -> 97
		.start_warning = 100, // 105 -> 100
		.start_tripping = 110,		/* temp to do tripping */
		.start_hw_tripping = 113,	/* temp to do hw_trpping*/
		.stop_mem_throttle = 80,
		.start_mem_throttle = 85,
		.stop_tc = 13,
		.start_tc = 10,
	},
	.cpulimit = {
		.throttle_freq = 900000, // 800000 -> 400000
		.warning_freq = 800000,
	},
	.temp_compensate = {
		.arm_volt = 925000, /* vdd_arm in uV for temperature compensation */
		.bus_volt = 900000, /* vdd_bus in uV for temperature compensation */
		.g3d_volt = 900000, /* vdd_g3d in uV for temperature compensation */
	},
	.efuse_value = 55,
	.slope = 0x10008802,
	.mode = 0,
};
#endif

#if defined(CONFIG_CMA)
static void __init exynos4_reserve_mem(void)
{
	static struct cma_region regions[] = {
#ifndef CONFIG_VIDEOBUF2_ION
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV
		{
			.name = "tv",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_TV * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG
		{
			.name = "jpeg",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_JPEG * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP
		{
			.name = "srp",
			.size = CONFIG_AUDIO_SAMSUNG_MEMSIZE_SRP * SZ_1K,
			.start = 0,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D
		{
			.name = "fimg2d",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMG2D * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD
		{
			.name = "fimd",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0
		{
			.name = "fimc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC0 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2
		{
			.name = "fimc2",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC2 * SZ_1K,
			.start = 0
		},
#endif
#if !defined(CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION) && \
	defined(CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3)
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1
		{
			.name = "fimc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC1 * SZ_1K,
			.start = 0
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_NORMAL
		{
			.name = "mfc-normal",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_NORMAL * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1
		{
			.name = "mfc1",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC1 * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0
		{
			.name = "mfc0",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC0 * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC
		{
			.name = "mfc",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC * SZ_1K,
			{ .alignment = 1 << 17 },
		},
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
		{
			.name = "fimc_is",
			.size = CONFIG_VIDEO_EXYNOS_MEMSIZE_FIMC_IS * SZ_1K,
			{
				.alignment = 1 << 26,
			},
			.start = 0
		},
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS_BAYER
		{
			.name = "fimc_is_isp",
			.size = CONFIG_VIDEO_EXYNOS_MEMSIZE_FIMC_IS_ISP * SZ_1K,
			.start = 0
		},
#endif
#endif
#if !defined(CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION) && \
	defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
		{
			.name		= "b2",
			.size		= 32 << 20,
			{ .alignment	= 128 << 10 },
		},
		{
			.name		= "b1",
			.size		= 32 << 20,
			{ .alignment	= 128 << 10 },
		},
		{
			.name		= "fw",
			.size		= 1 << 20,
			{ .alignment	= 128 << 10 },
		},
#endif
#else /* !CONFIG_VIDEOBUF2_ION */
#ifdef CONFIG_FB_S5P
#error CONFIG_FB_S5P is defined. Select CONFIG_FB_S3C, instead
#endif
		{
			.name	= "ion",
			.size	= CONFIG_ION_EXYNOS_CONTIGHEAP_SIZE * SZ_1K,
		},
#endif /* !CONFIG_VIDEOBUF2_ION */
		{
			.size = 0
		},
	};
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
	static struct cma_region regions_secure[] = {
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3
		{
			.name = "fimc3",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMC3 * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD_VIDEO
		{
			.name = "video",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_FIMD_VIDEO * SZ_1K,
		},
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_SECURE
		{
			.name = "mfc-secure",
			.size = CONFIG_VIDEO_SAMSUNG_MEMSIZE_MFC_SECURE * SZ_1K,
		},
#endif
		{
			.name = "sectbl",
			.size = SZ_1M,
		},
		{
			.size = 0
		},
	};
#else /* !CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION */
	struct cma_region *regions_secure = NULL;
#endif
	static const char map[] __initconst =
#ifdef CONFIG_EXYNOS_C2C
		"samsung-c2c=c2c_shdmem;"
#endif
		"s3cfb.0/fimd=fimd;exynos4-fb.0/fimd=fimd;"
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
		"s3cfb.0/video=video;exynos4-fb.0/video=video;"
#endif
		"s3c-fimc.0=fimc0;s3c-fimc.1=fimc1;s3c-fimc.2=fimc2;s3c-fimc.3=fimc3;"
		"exynos4210-fimc.0=fimc0;exynos4210-fimc.1=fimc1;exynos4210-fimc.2=fimc2;exynos4210-fimc.3=fimc3;"
#ifdef CONFIG_VIDEO_MFC5X
		"s3c-mfc/A=mfc0,mfc-secure;"
		"s3c-mfc/B=mfc1,mfc-normal;"
		"s3c-mfc/AB=mfc;"
#endif
#ifdef CONFIG_VIDEO_SAMSUNG_S5P_MFC
		"s5p-mfc/f=fw;"
		"s5p-mfc/a=b1;"
		"s5p-mfc/b=b2;"
#endif
		"samsung-rp=srp;"
		"s5p-jpeg=jpeg;"
		"exynos4-fimc-is/f=fimc_is;"
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS_BAYER
		"exynos4-fimc-is/i=fimc_is_isp;"
#endif
		"s5p-mixer=tv;"
		"s5p-fimg2d=fimg2d;"
		"ion-exynos=ion,fimd,fimc0,fimc1,fimc2,fimc3,fw,b1,b2;"
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
		"s5p-smem/video=video;"
		"s5p-smem/sectbl=sectbl;"
#endif
		"s5p-smem/mfc=mfc0,mfc-secure;"
		"s5p-smem/fimc=fimc3;"
		"s5p-smem/mfc-shm=mfc1,mfc-normal;"
		"s5p-smem/fimd=fimd;";

	s5p_cma_region_reserve(regions, regions_secure, 0, map);
}
#else
static inline void exynos4_reserve_mem(void)
{
}
#endif /* CONFIG_CMA */

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
/* LCD Backlight data */
static struct samsung_bl_gpio_info kcppk_bl_gpio_info = {
	.no = GPIO_LCD_PWM,
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data kcppk_bl_data = {
	.pwm_id = 1,
//	.pwm_period_ns  = 1*1000*1000, // 2Khz
	.pwm_period_ns  = 78770, // 25Khz
};
#endif

static void __init kcppk_map_io(void)
{
	clk_xusbxti.rate = 24000000;
	s5p_init_io(NULL, 0, S5P_VA_CHIPID);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(kcppk_uartcfgs, ARRAY_SIZE(kcppk_uartcfgs));

	exynos4_reserve_mem();
}

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
static void kcppk_5v_power_on(void)
{
    gpio_request_one(GPIO_5V_EN_H, GPIOF_OUT_INIT_HIGH, "GPX0_3");
	mdelay(50);
	gpio_free(GPIO_5V_EN_H);
}
#endif

static void __init exynos_sysmmu_init(void)
{
	ASSIGN_SYSMMU_POWERDOMAIN(fimc0, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc1, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc2, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(fimc3, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(jpeg, &exynos4_device_pd[PD_CAM].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_l, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(mfc_r, &exynos4_device_pd[PD_MFC].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(tv, &exynos4_device_pd[PD_TV].dev);
#ifdef CONFIG_VIDEO_FIMG2D
	sysmmu_set_owner(&SYSMMU_PLATDEV(g2d_acp).dev, &s5p_device_fimg2d.dev);
#endif
#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC) || defined(CONFIG_VIDEO_MFC5X)
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_l).dev, &s5p_device_mfc.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(mfc_r).dev, &s5p_device_mfc.dev);
#endif
#if defined(CONFIG_VIDEO_FIMC)
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s3c_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s3c_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s3c_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s3c_device_fimc3.dev);
#elif defined(CONFIG_VIDEO_SAMSUNG_S5P_FIMC)
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc0).dev, &s5p_device_fimc0.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc1).dev, &s5p_device_fimc1.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc2).dev, &s5p_device_fimc2.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(fimc3).dev, &s5p_device_fimc3.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_TV
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_mixer.dev);
#endif
#if 0 
#ifdef CONFIG_VIDEO_TVOUT
	sysmmu_set_owner(&SYSMMU_PLATDEV(tv).dev, &s5p_device_tvout.dev);
#endif
#endif
#ifdef CONFIG_VIDEO_JPEG_V2X
	sysmmu_set_owner(&SYSMMU_PLATDEV(jpeg).dev, &s5p_device_jpeg.dev);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	ASSIGN_SYSMMU_POWERDOMAIN(is_isp, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_drc, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_fd, &exynos4_device_pd[PD_ISP].dev);
	ASSIGN_SYSMMU_POWERDOMAIN(is_cpu, &exynos4_device_pd[PD_ISP].dev);

	sysmmu_set_owner(&SYSMMU_PLATDEV(is_isp).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_drc).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_fd).dev,
						&exynos4_device_fimc_is.dev);
	sysmmu_set_owner(&SYSMMU_PLATDEV(is_cpu).dev,
						&exynos4_device_fimc_is.dev);
#endif
}

#define SMDK4412_REV_0_0_ADC_VALUE 0
#define SMDK4412_REV_0_1_ADC_VALUE 443
int samsung_board_rev;

static int get_samsung_board_rev(void)
{
	int				adc_val = 0;
	struct clk		*adc_clk;
	struct resource	*res;
	void __iomem	*adc_regs;
	unsigned int	con;
	int		ret;

	if ((soc_is_exynos4412() && samsung_rev() < EXYNOS4412_REV_1_0) ||
		(soc_is_exynos4212() && samsung_rev() < EXYNOS4212_REV_1_0))
		return SAMSUNG_BOARD_REV_0_0;

	adc_clk = clk_get(NULL, "adc");
	if (unlikely(IS_ERR(adc_clk)))
		return SAMSUNG_BOARD_REV_0_0;

	clk_enable(adc_clk);

	res = platform_get_resource(&s3c_device_adc, IORESOURCE_MEM, 0);
	if (unlikely(!res))
		goto err_clk;

	adc_regs = ioremap(res->start, resource_size(res));
	if (unlikely(!adc_regs))
		goto err_clk;

	writel(S5PV210_ADCCON_SELMUX(3), adc_regs + S5P_ADCMUX);

	con = readl(adc_regs + S3C2410_ADCCON);
	con &= ~S3C2410_ADCCON_MUXMASK;
	con &= ~S3C2410_ADCCON_STDBM;
	con &= ~S3C2410_ADCCON_STARTMASK;
	con |=  S3C2410_ADCCON_PRSCEN;

	con |= S3C2410_ADCCON_ENABLE_START;
	writel(con, adc_regs + S3C2410_ADCCON);

	udelay(50);

	adc_val = readl(adc_regs + S3C2410_ADCDAT0) & 0xFFF;
	writel(0, adc_regs + S3C64XX_ADCCLRINT);

	iounmap(adc_regs);
err_clk:
	clk_disable(adc_clk);
	clk_put(adc_clk);

	ret = (adc_val < SMDK4412_REV_0_1_ADC_VALUE/2) ?
			SAMSUNG_BOARD_REV_0_0 : SAMSUNG_BOARD_REV_0_1;

	pr_info("SMDK MAIN Board Rev 0.%d (ADC value:%d)\n", ret, adc_val);
	return ret;
}

int kcppk_pshold_activate(int on){
    uint val = readl(S5P_PS_HOLD_CONTROL);
    if(on){
        val |= (3 << 8);
    }else{/* Sleep forever */
        val &= ~(1 << 8);
        mdelay(100);
    }
    writel(val, S5P_PS_HOLD_CONTROL);
    if(!on)
        while(1);
    return 0;
}
EXPORT_SYMBOL(kcppk_pshold_activate);

static void kcppk_export_gpios(void)
{
	int iRet,ret;


#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
	// kcppk LCD Power ONOFF 12V
	iRet =gpio_request(LCD_POWER_UPDOWN,"LCD_12V");
    if(iRet == 0) {
		s3c_gpio_cfgpin(LCD_POWER_UPDOWN, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(LCD_POWER_UPDOWN, S3C_GPIO_PULL_DOWN);
	    iRet = gpio_direction_output(LCD_POWER_UPDOWN,0);
        printk(KERN_INFO "GPIO %d Assinged to LCD_12V (ret: %d)\n", LCD_POWER_UPDOWN,ret);	
		gpio_export(LCD_POWER_UPDOWN, 0);
	} else {
		printk(KERN_ERR "Failed : GPIO %d Assinged to LCD_12V (ret: %d)\n",
			LCD_POWER_UPDOWN, iRet);
		
	}

	// kcppk CAMERA ONOFF 24V
	iRet =gpio_request(HTC_CAM_POWER,"CAM_24V");
    if(iRet == 0) {
		s3c_gpio_cfgpin(HTC_CAM_POWER, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(HTC_CAM_POWER, S3C_GPIO_PULL_DOWN);
	    iRet = gpio_direction_output(HTC_CAM_POWER,1);
        printk(KERN_INFO "GPIO %d Assinged to CAM_24V (ret: %d)\n", HTC_CAM_POWER,ret);
		gpio_export(HTC_CAM_POWER, 0);		
	} else {
		printk(KERN_ERR "Failed : GPIO %d Assinged to CAM_24V (ret: %d)\n",
			HTC_CAM_POWER, iRet);
		
	}

	// kcppk Key Button LED
	iRet =gpio_request(BUTTON_LED_ON,"Button_LED");
    if(iRet == 0) {
		s3c_gpio_cfgpin(BUTTON_LED_ON, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(BUTTON_LED_ON, S3C_GPIO_PULL_DOWN);
	    iRet = gpio_direction_output(BUTTON_LED_ON,1);
        printk(KERN_INFO "GPIO %d Assinged to Button_LED (ret: %d)\n", BUTTON_LED_ON,ret);	
		gpio_export(BUTTON_LED_ON, 0);			
	} else {
		printk(KERN_ERR "Failed : GPIO %d Assinged to Button_LED (ret: %d)\n",
			BUTTON_LED_ON, iRet);
		
	}
#endif

	//otg detect pin export
	iRet =gpio_request(GPIO_OTG_EN_H, "GPL2-3");
	iRet = gpio_direction_output(GPIO_OTG_EN_H, 0);
	printk(KERN_ERR "otg detect gpio = %d, ret:%i\n",
			GPIO_OTG_EN_H, iRet);
	iRet = gpio_export(GPIO_OTG_EN_H, 0);
	printk(KERN_ERR "otg gpio_export gpio = %d, ret:%i\n",
			GPIO_OTG_EN_H, iRet);

//kcppk_leds_ppwm
    kcppk_led_pwm_gpio_enable();


}

static void __init kcppk_machine_init(void)
{

#ifdef CONFIG_S3C64XX_DEV_SPI
	struct clk *sclk = NULL;
	struct clk *prnt = NULL;
	struct device *spi0_dev = &exynos_device_spi0.dev;
#ifndef CONFIG_FB_S5P_LMS501KF03
	struct device *spi1_dev = &exynos_device_spi1.dev;
#endif
	struct device *spi2_dev = &exynos_device_spi2.dev;
#endif
	samsung_board_rev = get_samsung_board_rev();
#if defined(CONFIG_EXYNOS_DEV_PD) && defined(CONFIG_PM_RUNTIME)
	exynos_pd_disable(&exynos4_device_pd[PD_MFC].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_G3D].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_LCD0].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_CAM].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_TV].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_GPS_ALIVE].dev);
	exynos_pd_disable(&exynos4_device_pd[PD_ISP].dev);
#elif defined(CONFIG_EXYNOS_DEV_PD)
	/*
	 * These power domains should be always on
	 * without runtime pm support.
	 */
	exynos_pd_enable(&exynos4_device_pd[PD_MFC].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_G3D].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_LCD0].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_CAM].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_TV].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_GPS].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_GPS_ALIVE].dev);
	exynos_pd_enable(&exynos4_device_pd[PD_ISP].dev);
#endif
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, i2c_devs0, ARRAY_SIZE(i2c_devs0));

#if 1
	s3c_i2c1_set_platdata(&ADV7280_i2c1_data);
	i2c_register_board_info(1, i2c_devs1, ARRAY_SIZE(i2c_devs1));	
#endif

#if defined(CONFIG_S3C_DEV_I2C6)
	s3c_i2c6_set_platdata(NULL);
	i2c_register_board_info(6, i2c_devs6, ARRAY_SIZE(i2c_devs6));
	
#endif

	s3c_i2c3_set_platdata(&i2c_3_data);
	i2c_register_board_info(3, i2c_devs3, ARRAY_SIZE(i2c_devs3));

	s3c_i2c7_set_platdata(NULL);

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
        kcppk_5v_power_on();
#endif

#if defined(CONFIG_FB_S5P_MIPI_DSIM)
	mipi_fb_init();
#endif

#ifdef CONFIG_FB_S5P
	s3cfb_set_platdata(NULL);
#endif
#ifdef CONFIG_FB_S5P_MIPI_DSIM
	s5p_device_dsim.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_fb.dev.parent = &exynos4_device_pd[PD_LCD0].dev;
#endif
#ifdef CONFIG_USB_EHCI_S5P
	kcppk_ehci_init();
#endif
#ifdef CONFIG_USB_OHCI_S5P
	kcppk_ohci_init();
#endif
#ifdef CONFIG_USB_GADGET
	kcppk_usbgadget_init();
#endif

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
	kcppk_usbswitch_init();
	samsung_bl_set(&kcppk_bl_gpio_info, &kcppk_bl_data);
#endif
#ifdef CONFIG_EXYNOS4_DEV_DWMCI
	exynos_dwmci_set_platdata(&exynos_dwmci_pdata,0);
#endif

#ifdef CONFIG_VIDEO_EXYNOS_FIMC_IS
	exynos4_fimc_is_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	exynos4_device_fimc_is.dev.parent = &exynos4_device_pd[PD_ISP].dev;
#endif
#endif
#ifdef CONFIG_S3C_DEV_HSMMC
	s3c_sdhci0_set_platdata(&kcppk_hsmmc0_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC1
	s3c_sdhci1_set_platdata(&kcppk_hsmmc1_pdata);
#endif
#ifdef CONFIG_S3C_DEV_HSMMC2
	s3c_sdhci2_set_platdata(&kcppk_hsmmc2_pdata);
#endif
#ifdef CONFIG_VIDEO_EXYNOS_FIMC_LITE
	kcppk_set_camera_flite_platdata();
	s3c_set_platdata(&exynos_flite0_default_data,
			sizeof(exynos_flite0_default_data), &exynos_device_flite0);
	s3c_set_platdata(&exynos_flite1_default_data,
			sizeof(exynos_flite1_default_data), &exynos_device_flite1);
#ifdef CONFIG_EXYNOS_DEV_PD
	exynos_device_flite0.dev.parent = &exynos4_device_pd[PD_ISP].dev;
	exynos_device_flite1.dev.parent = &exynos4_device_pd[PD_ISP].dev;
#endif
#endif
#ifdef CONFIG_EXYNOS_THERMAL
	exynos_tmu_set_platdata(&exynos_tmu_data);
#endif
#ifdef CONFIG_VIDEO_FIMC
	s3c_fimc0_set_platdata(&fimc_plat);
	s3c_fimc1_set_platdata(&fimc_plat);
	s3c_fimc2_set_platdata(NULL);
	s3c_fimc3_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#ifdef CONFIG_EXYNOS4_CONTENT_PATH_PROTECTION
	secmem.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#ifdef CONFIG_VIDEO_FIMC_MIPI
	s3c_csis0_set_platdata(NULL);
	s3c_csis1_set_platdata(NULL);
#ifdef CONFIG_EXYNOS_DEV_PD
	s3c_device_csis0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s3c_device_csis1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif

#endif /* CONFIG_VIDEO_FIMC */

#ifdef CONFIG_VIDEO_SAMSUNG_S5P_FIMC
	kcppk_camera_config();
	kcppk_subdev_config();

	dev_set_name(&s5p_device_fimc0.dev, "s3c-fimc.0");
	dev_set_name(&s5p_device_fimc1.dev, "s3c-fimc.1");
	dev_set_name(&s5p_device_fimc2.dev, "s3c-fimc.2");
	dev_set_name(&s5p_device_fimc3.dev, "s3c-fimc.3");

	clk_add_alias("fimc", "exynos4210-fimc.0", "fimc", &s5p_device_fimc0.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.0", "sclk_fimc",
			&s5p_device_fimc0.dev);
	clk_add_alias("fimc", "exynos4210-fimc.1", "fimc", &s5p_device_fimc1.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.1", "sclk_fimc",
			&s5p_device_fimc1.dev);
	clk_add_alias("fimc", "exynos4210-fimc.2", "fimc", &s5p_device_fimc2.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.2", "sclk_fimc",
			&s5p_device_fimc2.dev);
	clk_add_alias("fimc", "exynos4210-fimc.3", "fimc", &s5p_device_fimc3.dev);
	clk_add_alias("sclk_fimc", "exynos4210-fimc.3", "sclk_fimc",
			&s5p_device_fimc3.dev);

	s3c_fimc_setname(0, "exynos4210-fimc");
	s3c_fimc_setname(1, "exynos4210-fimc");
	s3c_fimc_setname(2, "exynos4210-fimc");
	s3c_fimc_setname(3, "exynos4210-fimc");
	/* FIMC */
	s3c_set_platdata(&s3c_fimc0_default_data,
			 sizeof(s3c_fimc0_default_data), &s5p_device_fimc0);
	s3c_set_platdata(&s3c_fimc1_default_data,
			 sizeof(s3c_fimc1_default_data), &s5p_device_fimc1);
	s3c_set_platdata(&s3c_fimc2_default_data,
			 sizeof(s3c_fimc2_default_data), &s5p_device_fimc2);
	s3c_set_platdata(&s3c_fimc3_default_data,
			 sizeof(s3c_fimc3_default_data), &s5p_device_fimc3);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_fimc0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc2.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_fimc3.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#ifdef CONFIG_VIDEO_S5P_MIPI_CSIS
	dev_set_name(&s5p_device_mipi_csis0.dev, "s3c-csis.0");
	dev_set_name(&s5p_device_mipi_csis1.dev, "s3c-csis.1");
	clk_add_alias("csis", "s5p-mipi-csis.0", "csis",
			&s5p_device_mipi_csis0.dev);
	clk_add_alias("sclk_csis", "s5p-mipi-csis.0", "sclk_csis",
			&s5p_device_mipi_csis0.dev);
	clk_add_alias("csis", "s5p-mipi-csis.1", "csis",
			&s5p_device_mipi_csis1.dev);
	clk_add_alias("sclk_csis", "s5p-mipi-csis.1", "sclk_csis",
			&s5p_device_mipi_csis1.dev);
	dev_set_name(&s5p_device_mipi_csis0.dev, "s5p-mipi-csis.0");
	dev_set_name(&s5p_device_mipi_csis1.dev, "s5p-mipi-csis.1");

	s3c_set_platdata(&s5p_mipi_csis0_default_data,
			sizeof(s5p_mipi_csis0_default_data), &s5p_device_mipi_csis0);
	s3c_set_platdata(&s5p_mipi_csis1_default_data,
			sizeof(s5p_mipi_csis1_default_data), &s5p_device_mipi_csis1);
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_mipi_csis0.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	s5p_device_mipi_csis1.dev.parent = &exynos4_device_pd[PD_CAM].dev;
#endif
#endif
#endif

#ifdef CONFIG_VIDEO_JPEG_V2X
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_jpeg.dev.parent = &exynos4_device_pd[PD_CAM].dev;
	exynos4_jpeg_setup_clock(&s5p_device_jpeg.dev, 160000000);
#endif
#endif

#ifdef CONFIG_ION_EXYNOS
	exynos_ion_set_platdata();
#endif

#if defined(CONFIG_VIDEO_MFC5X) || defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
#ifdef CONFIG_EXYNOS_DEV_PD
	s5p_device_mfc.dev.parent = &exynos4_device_pd[PD_MFC].dev;
#endif

		exynos4_mfc_setup_clock(&s5p_device_mfc.dev, 200 * MHZ);
#endif

#if defined(CONFIG_VIDEO_SAMSUNG_S5P_MFC)
	dev_set_name(&s5p_device_mfc.dev, "s3c-mfc");
	clk_add_alias("mfc", "s5p-mfc", "mfc", &s5p_device_mfc.dev);
	s5p_mfc_setname(&s5p_device_mfc, "s5p-mfc");
#endif

#ifdef CONFIG_VIDEO_FIMG2D
	s5p_fimg2d_set_platdata(&fimg2d_data);
#endif

	exynos_sysmmu_init();

	platform_add_devices(kcppk_devices, ARRAY_SIZE(kcppk_devices));
	if (soc_is_exynos4412())
		platform_add_devices(smdk4412_devices, ARRAY_SIZE(smdk4412_devices));

	platform_device_register(&kcppk_keypad);
	platform_device_register(&kcppk_leds_pwm);		
	kcppk_gpio_power_init();


#ifdef CONFIG_BUSFREQ_OPP
	dev_add(&busfreq, &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC0], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_DMC1], &exynos4_busfreq.dev);
	ppmu_init(&exynos_ppmu[PPMU_CPU], &exynos4_busfreq.dev);
#endif

	register_reboot_notifier(&exynos4_reboot_notifier);

	platform_device_register(&s3c_device_timer[0]);
	
	platform_device_register(&kcppk_leds_gpio);

#ifdef CONFIG_MFD_SOAP_KCPPK_BUTTONS
    platform_device_register(&kcppk_buttons_device);
#endif

	kcppk_export_gpios();
}

MACHINE_START(SMDK4412, "VALCAN")
	.boot_params	= S5P_PA_SDRAM + 0x100,
	.init_irq	= exynos4_init_irq,
	.map_io		= kcppk_map_io,
	.init_machine	= kcppk_machine_init,
	.timer		= &exynos4_timer,
#ifdef CONFIG_EXYNOS_C2C
	.reserve	= &exynos_c2c_reserve,
#endif
MACHINE_END


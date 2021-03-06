/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <generated/autoconf.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/kthread.h>
#include <linux/wakelock.h>
#include <linux/device.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/proc_fs.h>
#include <linux/syscalls.h>
#include <linux/sched.h>
#include <linux/writeback.h>
#include <linux/seq_file.h>

#include <asm/uaccess.h>
#include <mt-plat/charging.h>
#include <mt-plat/upmu_common.h>
#include <mach/upmu_sw.h>
#include <mach/upmu_hw.h>
#include <mach/mt_pmic_wrap.h>
#if defined CONFIG_MTK_LEGACY
/*#include <mach/mt_gpio.h> TBD*/
#endif
/*#include <mach/mtk_rtc.h> TBD*/
#include <mach/mt_spm_mtcmos.h>
#if defined(CONFIG_MTK_SMART_BATTERY)
#include <mt-plat/battery_common.h>
#endif
#include <linux/time.h>

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
#include <mt-plat/mt_usb2jtag.h>
#endif

#if defined(CONFIG_TCPC_CLASS)
#include <tcpm.h>
static struct tcpc_device *tcpc_dev;
#endif

/* ============================================================ // */
/* extern function */
/* ============================================================ // */
bool is_dcp_type = false;
#if defined(CONFIG_POWER_EXT) || defined(CONFIG_MTK_FPGA)

int hw_charging_get_charger_type(void)
{
	return STANDARD_HOST;
}

#else

static void hw_bc11_dump_register(void)
{
/*
    battery_log(BAT_LOG_FULL, "Reg[0x%x]=0x%x,Reg[0x%x]=0x%x\n",
	MT6325_CHR_CON20, upmu_get_reg_value(MT6325_CHR_CON20),
	MT6325_CHR_CON21, upmu_get_reg_value(MT6325_CHR_CON21)
	);
*/
}

static void hw_bc11_init(void)
{
	msleep(200);
#if defined(CONFIG_MTK_SMART_BATTERY)
	Charger_Detect_Init();
#endif
	/* RG_bc11_BIAS_EN=1 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 1);
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0);
	/* bc11_RST=1 */
	bc11_set_register_value(PMIC_RG_BC11_RST, 1);
	/* bc11_BB_CTRL=1 */
	bc11_set_register_value(PMIC_RG_BC11_BB_CTRL, 1);

	msleep(50);
	/* mdelay(50); */

	if (Enable_BATDRV_LOG == BAT_LOG_FULL) {
		battery_log(BAT_LOG_FULL, "hw_bc11_init() \r\n");
		hw_bc11_dump_register();
	}

}


static unsigned int hw_bc11_DCD(void)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_IPU_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x2);
	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=01 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x1);
	/* RG_bc11_CMP_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);

	msleep(80);
	/* mdelay(80); */

	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	if (Enable_BATDRV_LOG == BAT_LOG_FULL) {
		battery_log(BAT_LOG_FULL, "hw_bc11_DCD() \r\n");
		hw_bc11_dump_register();
	}
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);


	return wChargerAvail;
}


static unsigned int hw_bc11_stepA1(void)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_IPD_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);

	msleep(80);
	/* mdelay(80); */

	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	if (Enable_BATDRV_LOG == BAT_LOG_FULL) {
		battery_log(BAT_LOG_FULL, "hw_bc11_stepA1() \r\n");
		hw_bc11_dump_register();
	}
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);

	return wChargerAvail;
}


static unsigned int hw_bc11_stepA2(void)
{
	unsigned int wChargerAvail = 0;

	/* RG_bc11_VSRC_EN[1.0] = 10 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	/* RG_bc11_IPD_EN[1:0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x1);
	/* RG_bc11_VREF_VTH = [1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 01 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x1);

	msleep(80);
	/* mdelay(80); */

	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);

	if (Enable_BATDRV_LOG == BAT_LOG_FULL) {
		battery_log(BAT_LOG_FULL, "hw_bc11_stepA2() \r\n");
		hw_bc11_dump_register();
	}
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);

	return wChargerAvail;
}


static unsigned int hw_bc11_stepB2(void)
{
	unsigned int wChargerAvail = 0;

	/*enable the voltage source to DM*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x1);
	/* enable the pull-down current to DP */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x2);
	/* VREF threshold voltage for comparator  =0.325V */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* enable the comparator to DP */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x2);
	msleep(80);
	wChargerAvail = bc11_get_register_value(PMIC_RGS_BC11_CMP_OUT);
	if (!wChargerAvail) {
		/* RG_bc11_VSRC_EN[1.0] = 10 */
		bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x2);
	}
	/*reset to default value*/
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);

	return wChargerAvail;
}


static void hw_bc11_done(void)
{
	/* RG_bc11_VSRC_EN[1:0]=00 */
	bc11_set_register_value(PMIC_RG_BC11_VSRC_EN, 0x0);
	/* RG_bc11_VREF_VTH = [1:0]=0 */
	bc11_set_register_value(PMIC_RG_BC11_VREF_VTH, 0x0);
	/* RG_bc11_CMP_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_CMP_EN, 0x0);
	/* RG_bc11_IPU_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPU_EN, 0x0);
	/* RG_bc11_IPD_EN[1.0] = 00 */
	bc11_set_register_value(PMIC_RG_BC11_IPD_EN, 0x0);
	/* RG_bc11_BIAS_EN=0 */
	bc11_set_register_value(PMIC_RG_BC11_BIAS_EN, 0x0);


	Charger_Detect_Release();

	if (Enable_BATDRV_LOG == BAT_LOG_FULL) {
		battery_log(BAT_LOG_FULL, "hw_bc11_done() \r\n");
		hw_bc11_dump_register();
	}

}

int hw_charging_get_charger_type(void)
{
#if 0
	return STANDARD_HOST;
	/* return STANDARD_CHARGER; //adaptor */
#else
	CHARGER_TYPE CHR_Type_num = CHARGER_UNKNOWN;

#ifdef CONFIG_MTK_USB2JTAG_SUPPORT
	if (usb2jtag_mode()) {
		pr_err("[USB2JTAG] in usb2jtag mode, skip charger detection\n");
		return STANDARD_HOST;
	}
#endif

	/********* Step initial  ***************/
	hw_bc11_init();

	/********* Step DCD ***************/
	if (1 == hw_bc11_DCD()) {
		/********* Step A1 ***************/
		if (1 == hw_bc11_stepA1()) {
			CHR_Type_num = APPLE_2_1A_CHARGER;
			battery_log(BAT_LOG_CRTI, "step A1 : Apple 2.1A CHARGER!\r\n");
		} else {
			CHR_Type_num = NONSTANDARD_CHARGER;
			battery_log(BAT_LOG_CRTI, "step A1 : Non STANDARD CHARGER!\r\n");
			#if defined(CONFIG_TCPC_CLASS)
			{
				uint8_t cc1;
				uint8_t cc2;
				tcpc_dev = tcpc_dev_get_by_name("type_c_port0");
				if (tcpc_dev) {
					tcpm_inquire_remote_cc(tcpc_dev, &cc1, &cc2, true);
					battery_log(BAT_LOG_CRTI, "cc1=%d, cc2=%d\r\n", cc1, cc2);
					if ((cc1 == 0x7) || (cc2 == 0x07)) {
						battery_log(BAT_LOG_CRTI, "step A1_1 : Suppose is Type-C 3A PD Charger\r\n");
						is_dcp_type = true;
						CHR_Type_num = STANDARD_CHARGER;
						battery_log(BAT_LOG_CRTI, "step A1_1 : STANDARD CHARGER!\r\n");
					} else if ((cc1 == 0x6) || (cc2 == 0x06)) {
						battery_log(BAT_LOG_CRTI, "step A1_2 : Suppose is Type-C 1.5A PD Charger\r\n");
						is_dcp_type = true;
						CHR_Type_num = STANDARD_CHARGER;
						battery_log(BAT_LOG_CRTI, "step A1_2 : STANDARD CHARGER!\r\n");
					}
				}
			}
			#endif
		}
	} else {
	/********* Step A2 ***************/
	if (1 == hw_bc11_stepA2()) {
		/********* Step B2 ***************/
			if (1 == hw_bc11_stepB2()) {
				is_dcp_type = true;
				CHR_Type_num = STANDARD_CHARGER;
				battery_log(BAT_LOG_CRTI, "step B2 : STANDARD CHARGER!\r\n");
			} else {
				CHR_Type_num = CHARGING_HOST;
				battery_log(BAT_LOG_CRTI, "step B2 :  Charging Host!\r\n");
			}
		} else {
			CHR_Type_num = STANDARD_HOST;
			battery_log(BAT_LOG_CRTI, "step A2 : Standard USB Host!\r\n");
		}

	}

    /********* Finally setting *******************************/
	hw_bc11_done();

	return CHR_Type_num;
#endif
}
#endif

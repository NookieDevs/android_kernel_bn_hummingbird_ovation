/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
//#define DEBUG

#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_ts.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/usb/otg.h>

#include <linux/gpio.h>
#include <linux/regulator/consumer.h>
#include <asm/uaccess.h>
#if defined(CONFIG_DEBUG_FS)
#include <linux/debugfs.h>
#endif /* defined(CONFIG_DEBUG_FS) */

#define MXT_TAG "MXT"

/* Family ID */
#define MXT224_ID		0x80
#define MXT768E_ID		0xA1
#define MXT1188S_ID		0xA2
#define MXT1386_ID		0xA0

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* I2C slave address pairs */
struct mxt_address_pair {
	int bootloader;
	int application;
};

static const struct mxt_address_pair mxt_slave_addresses[] = {
	{ 0x26, 0x4a },
	{ 0x25, 0x4b },
	{ 0x25, 0x4b },
	{ 0x26, 0x4c },
	{ 0x27, 0x4d },
	{ 0x34, 0x5a },
	{ 0x35, 0x5b },
	{ 0 },
};

enum mxt_device_state { INIT, APPMODE, BOOTLOADER };

/* Firmware files */
#define MXT_FW_NAME		"maxtouch.fw"
#define MXT_BIN_CFG_NAME	"maxtouch_cfg.bin"
#define MXT_TXT_CFG_NAME	"maxtouch_cfg.txt"
#define MXT_CFG_MAGIC		"OBP_RAW V1"

/* Registers */
#define MXT_FAMILY_ID		(0x00)
#define MXT_VARIANT_ID		(0x01)
#define MXT_VERSION		(0x02)
#define MXT_BUILD		(0x03)
#define MXT_MATRIX_X_SIZE	(0x04)
#define MXT_MATRIX_Y_SIZE	(0x05)
#define MXT_OBJECT_NUM		(0x06)
#define MXT_OBJECT_START	(0x07)

#define MXT_OBJECT_SIZE		(6)

/* Object types */
#define MXT_GEN_MESSAGE_T5		(5)
#define MXT_GEN_COMMAND_T6		(6)
#define MXT_GEN_POWER_T7		(7)
#define MXT_GEN_ACQUIRE_T8		(8)
#define MXT_TOUCH_MULTI_T9		(9)
#define MXT_TOUCH_KEYARRAY_T15		(15)
#define MXT_SPT_COMMSCONFIG_T18		(18)
#define MXT_SPT_GPIOPWM_T19		(19)
#define MXT_PROCI_GRIPFACE_T20		(20)
#define MXT_PROCG_NOISE_T22		(22)
#define MXT_TOUCH_PROXIMITY_T23		(23)
#define MXT_PROCI_ONETOUCH_T24		(24)
#define MXT_SPT_SELFTEST_T25		(25)
#define MXT_PROCI_TWOTOUCH_T27		(27)
#define MXT_SPT_CTECONFIG_T28		(28)
#define MXT_DEBUG_DIAGNOSTIC_T37	(37)
#define MXT_SPT_USERDATA_T38		(38)
#define MXT_PROCI_GRIP_T40		(40)
#define MXT_PROCI_PALM_T41		(41)
#define MXT_PROCI_TOUCHSUPPRESSION_T42	(42)
#define MXT_SPT_DIGITIZER_T43		(43)
#define MXT_SPT_MESSAGECOUNT_T44	(44)
#define MXT_SPT_CTECONFIG_T46		(46)
#define MXT_PROCI_STYLUS_T47		(47)
#define MXT_PROCG_NOISESUPPRESSION_T48	(48)
#define MXT_TOUCH_PROXKEY_T52		(52)
#define MXT_GEN_DATASOURCE_T53		(53)
#define MXT_PROCG_NOISESUPPRESSION_T62	(62)


/* MXT_GEN_MESSAGE_T5 object */
#define MXT_RPTID_NOMSG		(0xff)
#define MXT_MSG_MAX_SIZE	(9)

/* MXT_SPT_MESSAGECOUNT_T44 object */
#define MXT_MESSAGECOUNT_COUNT	(0x00)


/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	(0)
#define MXT_COMMAND_BACKUPNV	(1)
#define MXT_COMMAND_CALIBRATE	(2)
#define MXT_COMMAND_REPORTALL	(3)
#define MXT_COMMAND_DIAGNOSTIC	(5)

/* MXT_GEN_POWER_T7 field */
#define MXT_POWER_IDLEACQINT	(0)
#define MXT_POWER_ACTVACQINT	(1)
#define MXT_POWER_ACTV2IDLETO	(2)

#define MXT_POWER_CFG_RUN	(0)
#define MXT_POWER_CFG_DEEPSLEEP	(1)

/* MXT_GEN_ACQUIRE_T8 field */
#define MXT_ACQUIRE_CHRGTIME	(0)
#define MXT_ACQUIRE_TCHDRIFT	(2)
#define MXT_ACQUIRE_DRIFTST	(3)
#define MXT_ACQUIRE_TCHAUTOCAL	(4)
#define MXT_ACQUIRE_SYNC	(5)
#define MXT_ACQUIRE_ATCHCALST	(6)
#define MXT_ACQUIRE_ATCHCALSTHR	(7)

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_TOUCH_CTRL		(0)
#define MXT_TOUCH_XORIGIN	(1)
#define MXT_TOUCH_YORIGIN	(2)
#define MXT_TOUCH_XSIZE		(3)
#define MXT_TOUCH_YSIZE		(4)
#define MXT_TOUCH_BLEN		(6)
#define MXT_TOUCH_TCHTHR	(7)
#define MXT_TOUCH_TCHDI		(8)
#define MXT_TOUCH_ORIENT	(9)
#define MXT_TOUCH_MOVHYSTI	(11)
#define MXT_TOUCH_MOVHYSTN	(12)
#define MXT_TOUCH_NUMTOUCH	(14)
#define MXT_TOUCH_MRGHYST	(15)
#define MXT_TOUCH_MRGTHR	(16)
#define MXT_TOUCH_AMPHYST	(17)
#define MXT_TOUCH_XRANGE_LSB	(18)
#define MXT_TOUCH_XRANGE_MSB	(19)
#define MXT_TOUCH_YRANGE_LSB	(20)
#define MXT_TOUCH_YRANGE_MSB	(21)
#define MXT_TOUCH_XLOCLIP	(22)
#define MXT_TOUCH_XHICLIP	(23)
#define MXT_TOUCH_YLOCLIP	(24)
#define MXT_TOUCH_YHICLIP	(25)
#define MXT_TOUCH_XEDGECTRL	(26)
#define MXT_TOUCH_XEDGEDIST	(27)
#define MXT_TOUCH_YEDGECTRL	(28)
#define MXT_TOUCH_YEDGEDIST	(29)
#define MXT_TOUCH_JUMPLIMIT	(30)

#define MXT_TOUCH_CTRL_ENABLE	(1<<0)
#define MXT_TOUCH_CTRL_RPTEN	(1<<1)
#define MXT_TOUCH_CTRL_DISAMP	(1<<2)
#define MXT_TOUCH_CTRL_DISVECT	(1<<3)
#define MXT_TOUCH_CTRL_DISMOVE	(1<<4)
#define MXT_TOUCH_CTRL_DISREL	(1<<5)
#define MXT_TOUCH_CTRL_DISPRSS	(1<<6)
#define MXT_TOUCH_CTRL_SCANEN	(1<<7)

/* MXT_PROCI_GRIPFACE_T20 field */
#define MXT_GRIPFACE_CTRL	(0)
#define MXT_GRIPFACE_XLOGRIP	(1)
#define MXT_GRIPFACE_XHIGRIP	(2)
#define MXT_GRIPFACE_YLOGRIP	(3)
#define MXT_GRIPFACE_YHIGRIP	(4)
#define MXT_GRIPFACE_MAXTCHS	(5)
#define MXT_GRIPFACE_SZTHR1	(7)
#define MXT_GRIPFACE_SZTHR2	(8)
#define MXT_GRIPFACE_SHPTHR1	(9)
#define MXT_GRIPFACE_SHPTHR2	(10)
#define MXT_GRIPFACE_SUPEXTTO	(11)

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		(0)
#define MXT_NOISE_OUTFLEN	(1)
#define MXT_NOISE_GCAFUL_LSB	(3)
#define MXT_NOISE_GCAFUL_MSB	(4)
#define MXT_NOISE_GCAFLL_LSB	(5)
#define MXT_NOISE_GCAFLL_MSB	(6)
#define MXT_NOISE_ACTVGCAFVALID	(7)
#define MXT_NOISE_NOISETHR	(8)
#define MXT_NOISE_FREQHOPSCALE	(10)
#define MXT_NOISE_FREQ0		(11)
#define MXT_NOISE_FREQ1		(12)
#define MXT_NOISE_FREQ2		(13)
#define MXT_NOISE_FREQ3		(14)
#define MXT_NOISE_FREQ4		(15)
#define MXT_NOISE_IDLEGCAFVALID	(16)

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		(0)
#define MXT_COMMS_CMD		(1)

/* MXT_SPT_CTECONFIG_T28 field */
#define MXT_CTE_CTRL		(0)
#define MXT_CTE_CMD		(1)
#define MXT_CTE_MODE		(2)
#define MXT_CTE_IDLEGCAFDEPTH	(3)
#define MXT_CTE_ACTVGCAFDEPTH	(4)
#define MXT_CTE_VOLTAGE		(5)

#define MXT_VOLTAGE_DEFAULT	(2700000)
#define MXT_VOLTAGE_STEP	(10000)

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		(0xa5)
#define MXT_RESET_VALUE		(0x01)
#define MXT_BACKUP_VALUE	(0x55)
#define MXT_BACKUP_TIME		(25)	/* msec */
#define MXT224_RESET_TIME	(65)	/* msec */
#define MXT768E_RESET_TIME	(250)	/* msec */
#define MXT1188S_RESET_TIME	(250)	/* msec */
#define MXT1386_RESET_TIME	(200)	/* msec */
#define MXT_RESET_TIME		(200)	/* msec */
#define MXT_RESET_NOCHGREAD	(400)	/* msec */

#define MXT_FWRESET_TIME	(1000)	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	(0xaa)
#define MXT_UNLOCK_CMD_LSB	(0xdc)

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	(0xc0)	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA		(0x80)	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK		(0x02)
#define MXT_FRAME_CRC_FAIL		(0x03)
#define MXT_FRAME_CRC_PASS		(0x04)
#define MXT_APP_CRC_FAIL		(0x40)	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK		(0x3f)
#define MXT_BOOT_EXTENDED_ID		(1 << 5)
#define MXT_BOOT_ID_MASK		(0x1f)

/* Command process status */
#define MXT_STATUS_CFGERROR	(1 << 3)

/* Touch status */
#define MXT_TOUCH_STATUS_SUPPRESS	(1 << 1)
#define MXT_TOUCH_STATUS_AMP		(1 << 2)
#define MXT_TOUCH_STATUS_VECTOR		(1 << 3)
#define MXT_TOUCH_STATUS_MOVE		(1 << 4)
#define MXT_TOUCH_STATUS_RELEASE	(1 << 5)
#define MXT_TOUCH_STATUS_PRESS		(1 << 6)
#define MXT_TOUCH_STATUS_DETECT		(1 << 7)

/* MXT_PROCI_ONETOUCH_T24 field */
#define MXT_ONETOUCH_CTRL	(0)
#define MXT_ONETOUCH_NUMGEST	(1)
#define MXT_ONETOUCH_GESTEN_0	(2)
#define MXT_ONETOUCH_GESTEN_1	(3)
#define MXT_ONETOUCH_PROCESS	(4)
#define MXT_ONETOUCH_TAPTO	(5)
#define MXT_ONETOUCH_FLICKTO	(6)
#define MXT_ONETOUCH_DRAGTO	(7)
#define MXT_ONETOUCH_SPRESSTO	(8)
#define MXT_ONETOUCH_LPRESSTO	(9)
#define MXT_ONETOUCH_REPPRESSTO	(10)
#define MXT_ONETOUCH_FLICKTHR_0	(11)
#define MXT_ONETOUCH_FLICKTHR_1	(12)
#define MXT_ONETOUCH_DRAGTHR_0	(13)
#define MXT_ONETOUCH_DRAGTHR_1	(14)
#define MXT_ONETOUCH_TAPTHR_0	(15)
#define MXT_ONETOUCH_TAPTHR_1	(16)
#define MXT_ONETOUCH_THROWTHR_0	(17)
#define MXT_ONETOUCH_THROWTHR_1	(18)

#define MXT_ONETOUCH_CTRL_ENABLE	(1 << 0)
#define MXT_ONETOUCH_CTRL_RPTEN		(1 << 1)

#define MXT_ONETOUCH_GESTEN_0_PRESS	(1 << 0)
#define MXT_ONETOUCH_GESTEN_0_RELEASE	(1 << 1)
#define MXT_ONETOUCH_GESTEN_0_TAP	(1 << 2)
#define MXT_ONETOUCH_GESTEN_0_DBLTAP	(1 << 3)
#define MXT_ONETOUCH_GESTEN_0_FLICK	(1 << 4)
#define MXT_ONETOUCH_GESTEN_0_DRAG	(1 << 5)
#define MXT_ONETOUCH_GESTEN_0_SPRESS	(1 << 6)
#define MXT_ONETOUCH_GESTEN_0_LPRESS	(1 << 7)

#define MXT_ONETOUCH_GESTEN_1_RPRESS	(1 << 0)
#define MXT_ONETOUCH_GESTEN_1_THROW	(1 << 1)

#define MXT_ONETOUCH_PROCESS_SHORTEN	(1 << 0)
#define MXT_ONETOUCH_PROCESS_LONGEN	(1 << 1)
#define MXT_ONETOUCH_PROCESS_REPTEN	(1 << 2)
#define MXT_ONETOUCH_PROCESS_DBLTAPEN	(1 << 3)
#define MXT_ONETOUCH_PROCESS_FLICKEN	(1 << 4)
#define MXT_ONETOUCH_PROCESS_THROWEN	(1 << 5)

#define MXT_ONETOUCH_MSG_STATUS		(0)
#define MXT_ONETOUCH_MSG_XPOSMSB	(1)
#define MXT_ONETOUCH_MSG_YPOSMSB	(2)
#define MXT_ONETOUCH_MSG_XYPOSLSB	(3)
#define MXT_ONETOUCH_MSG_DIR		(4)
#define MXT_ONETOUCH_MSG_DIST_0		(5)
#define MXT_ONETOUCH_MSG_DIST_1		(6)

#define MXT_ONETOUCH_MSG_EVENT_MASK            (0x0F)
#define MXT_ONETOUCH_MSG_EVENT_PRESS           (0x01)
#define MXT_ONETOUCH_MSG_EVENT_RELEASE         (0x02)
#define MXT_ONETOUCH_MSG_EVENT_TAP             (0x03)
#define MXT_ONETOUCH_MSG_EVENT_DOUBLETAP       (0x04)
#define MXT_ONETOUCH_MSG_EVENT_FLICK           (0x05)
#define MXT_ONETOUCH_MSG_EVENT_DRAG            (0x06)
#define MXT_ONETOUCH_MSG_EVENT_SHORTPRESS      (0x07)
#define MXT_ONETOUCH_MSG_EVENT_LONGPRESS       (0x08)
#define MXT_ONETOUCH_MSG_EVENT_REPEATTPRESS    (0x09)
#define MXT_ONETOUCH_MSG_EVENT_TAPANDPRESS     (0x0A)
#define MXT_ONETOUCH_MSG_EVENT_THROW           (0x0B)

#define MXT_DIR_E      (0x00)
#define MXT_DIR_NE     (0x20)
#define MXT_DIR_N      (0x40)
#define MXT_DIR_NW     (0x60)
#define MXT_DIR_W      (0x80)
#define MXT_DIR_SW     (0xA0)
#define MXT_DIR_S      (0xC0)
#define MXT_DIR_SE     (0xE0)

#define MXT_IS_DIR_E(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_E + 0x10)) \
		|| \
	  ((u8)(dir) >= (MXT_DIR_SE + 0x10)) \
	)
#define MXT_IS_DIR_NE(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_NE + 0x10)) \
		&& \
	  ((u8)(dir) >= (MXT_DIR_E + 0x10)) \
	)
#define MXT_IS_DIR_N(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_N + 0x10)) \
		&& \
	  ((u8)(dir) >= (MXT_DIR_NE + 0x10)) \
	)
#define MXT_IS_DIR_NW(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_NW + 0x10)) \
		&& \
	  ((u8)(dir) >= (MXT_DIR_N + 0x10)) \
	)
#define MXT_IS_DIR_W(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_W + 0x10)) \
		&& \
	  ((u8)(dir) >= (MXT_DIR_NW + 0x10)) \
	)
#define MXT_IS_DIR_SW(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_SW + 0x10)) \
		&& \
	  ((u8)(dir) >= (MXT_DIR_W + 0x10)) \
	)
#define MXT_IS_DIR_S(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_S + 0x10)) \
		&& \
	  ((u8)(dir) >= (MXT_DIR_SW + 0x10)) \
	)
#define MXT_IS_DIR_SE(dir) \
	( \
	  ((u8)(dir) < (MXT_DIR_SE + 0x10)) \
		&& \
	  ((u8)(dir) >= (MXT_DIR_S + 0x10)) \
	)

/* MXT_PROCI_TWOTOUCH_T27 field */
#define MXT_TWOTOUCH_CTRL	(0)
#define MXT_TWOTOUCH_NUMGEST	(1)
#define MXT_TWOTOUCH_GESTEN	(3)
#define MXT_TWOTOUCH_ROTATETHR	(4)
#define MXT_TWOTOUCH_ZOOMTHR_0	(5)
#define MXT_TWOTOUCH_ZOOMTHR_1	(6)

#define MXT_TWOTOUCH_CTRL_ENABLE	(1 << 0)
#define MXT_TWOTOUCH_CTRL_RPTEN		(1 << 1)
#define MXT_TWOTOUCH_GESTEN_PINCH	(1 << 5)
#define MXT_TWOTOUCH_GESTEN_ROTATE	(1 << 6)
#define MXT_TWOTOUCH_GESTEN_STRETCH	(1 << 7)

#define MXT_TWOTOUCH_MSG_STATUS		(0)
#define MXT_TWOTOUCH_MSG_XPOSMSB	(1)
#define MXT_TWOTOUCH_MSG_YPOSMSB	(2)
#define MXT_TWOTOUCH_MSG_XYPOSLSB	(3)
#define MXT_TWOTOUCH_MSG_ANGLE		(4)
#define MXT_TWOTOUCH_MSG_SEP_0		(5)
#define MXT_TWOTOUCH_MSG_SEP_1		(6)

#define MXT_TWOTOUCH_MSG_STATUS_MASK		(0xF0)
#define MXT_TWOTOUCH_MSG_STATUS_ROTATEDIR	(1 << 4)
#define MXT_TWOTOUCH_MSG_STATUS_PINCH		(1 << 5)
#define MXT_TWOTOUCH_MSG_STATUS_ROTATE		(1 << 6)
#define MXT_TWOTOUCH_MSG_STATUS_STRETCH		(1 << 7)

#define GESTURE_NONE	0x00
#define GESTURE_ST_N	0x10
#define GESTURE_ST_NE	0x12
#define GESTURE_ST_E	0x14
#define GESTURE_ST_SE	0x16
#define GESTURE_ST_S	0x18
#define GESTURE_ST_SW	0x1A
#define GESTURE_ST_W	0x1C
#define GESTURE_ST_NW	0x1E
#define GESTURE_SC	0x20
#define GESTURE_DC	0x22
#define GESTURE_TD	0x2F
#define GESTURE_DT	0x31
#define GESTURE_ZI	0x48
#define GESTURE_ZO	0x49
#define GESTURE_LO	0x4F

/* MXT_PROCG_NOISESUPPRESSION_T62 field */
#define MXT_CHARGER_CTRL	(0)
#define MXT_CHARGER_CALCFG1	(1)
#define MXT_CHARGER_CALCFG2	(2)
#define MXT_CHARGER_CALCFG3	(3)
#define MXT_CHARGER_CFG1	(4)
#define MXT_CHARGER_BASEFREQ	(7)
#define MXT_CHARGER_MAXSELFREQ	(8)
#define MXT_CHARGER_FREQ_0	(9)
#define MXT_CHARGER_FREQ_1	(10)
#define MXT_CHARGER_FREQ_2	(11)
#define MXT_CHARGER_FREQ_3	(12)
#define MXT_CHARGER_FREQ_4	(13)
#define MXT_CHARGER_HOPCNT	(14)
#define MXT_CHARGER_HOPCNTPER	(16)

#define MXT_CHARGER_CTRL_ENABLE		(1 << 0)
#define MXT_CHARGER_CTRL_RPTEN		(1 << 1)
#define MXT_CHARGER_CTRL_RPTSELFREQ	(1 << 2)
#define MXT_CHARGER_CTRL_RPTADCSPERX	(1 << 3)
#define MXT_CHARGER_CTRL_RPTGCLIMIT	(1 << 4)
#define MXT_CHARGER_CTRL_RPTNOISELVL	(1 << 5)
#define MXT_CHARGER_CTRL_RPTNLTHR	(1 << 6)

#define MXT_CHARGER_CALCFG1_CHRGON	(1 << 0)
#define MXT_CHARGER_CALCFG1_DISGC	(1 << 1)
#define MXT_CHARGER_CALCFG1_STAYOFF	(1 << 2)
#define MXT_CHARGER_CALCFG1_DISAUTOFREQ	(1 << 3)

#define MXT_CHARGER_CALCFG3_DUALX	(1 << 0)
#define MXT_CHARGER_CALCFG3_INCRST	(1 << 2)
#define MXT_CHARGER_CALCFG3_CHRGIN	(1 << 3)
#define MXT_CHARGER_CALCFG3_FRQDRFT	(1 << 4)

#define MXT_CHARGER_CFG1_CHRGON		(1 << 0)
#define MXT_CHARGER_CFG1_DISGC		(1 << 1)
#define MXT_CHARGER_CFG1_STAYOFF	(1 << 2)
#define MXT_CHARGER_CFG1_DISAUTOFREQ	(1 << 3)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_MAX_FINGER		10

#define MXT_MEMACCESS_SIZE 32768
#define MXT_I2C_MAX_REQ_SIZE 256

/* Forward declaration of the structure */
struct mxt_data;

enum mxt_power_mode
{
	MXT_POWER_MODE_DEEP_SLEEP = 0,
	MXT_POWER_MODE_ACTIVE
};

enum mxt_power_supply_state
{
	MXT_POWER_SUPPLY_OFF = 0,
	MXT_POWER_SUPPLY_ON
};

enum mxt_mt_protocol
{
	MXT_MT_NONE = 0,
	MXT_MT_A,
	MXT_MT_B
};

struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *es);
static void mxt_late_resume(struct early_suspend *es);
#endif

struct mxt_object {
	u16 type;
	u16 start_address;
	u16 size;
	u16 instances;
	u16 num_report_ids;

	/* to map object and message */
	u16 min_reportid;
	u16 max_reportid;
#if defined(CONFIG_DEBUG_FS)
	struct mxt_data *priv_data;
#endif /* defined(CONFIG_DEBUG_FS) */
};

struct mxt_message {
	u8 reportid;
	u8 message[MXT_MSG_MAX_SIZE - 2];
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
	int pressure;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	atomic_t  mxt_state;
	struct mxt_object *object_table;
	struct mxt_info info;

	struct mutex mutex_fingers;
	struct mxt_finger finger[MXT_MAX_FINGER];
	int mxt_n_active_fingers;

	struct workqueue_struct *mxt_workqueue;
	unsigned int irq;
	atomic_t  mxt_irq_enabled;
	struct work_struct mxt_irq_work;
	unsigned int max_x;
	unsigned int max_y;
	struct bin_attribute mem_access_attr;
	int debug_enabled;
	atomic_t uses_mt_slots;
	u8 actv_cycle_time;
	u8 idle_cycle_time;
	atomic_t mxt_power_mode;
	atomic_t mxt_power_supply_state;
	atomic_t do_not_suspend_mxt;

	u8 max_reportid;

#if 0
	/* Charger support */
	struct otg_transceiver *otg;
	struct notifier_block otg_nb;
	atomic_t mxt_charger_state;
	struct work_struct mxt_charger_work;
#endif

	/* Gesture support */
	struct mutex mutex_onetouch_gestures;
	u8 current_mxt_onetouch_gesture;
	u8 current_bn_onetouch_gesture;
	u8 onetouch_gesture_count;
	u8 mxt_onetouch_gesture_active;

	struct mutex mutex_twotouch_gestures;
	u8 current_mxt_twotouch_gesture;
	u8 current_bn_twotouch_gesture;
	u8 twotouch_gesture_count;
	u8 mxt_twotouch_gesture_active;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend early_suspend;
#endif
#if defined(CONFIG_DEBUG_FS)
	struct dentry *dbgfs_root;
	struct dentry **dbgfs_files;
	unsigned int dbgfs_n_entries;
#endif /* defined(CONFIG_DEBUG_FS) */
};

/* HOW-NOT: USB event comes before we register in the notifiers,
 * so we do not know that there is SDP/DCP connected on boot, so
 * we get it manually on probe. */
extern int twl6030_usbotg_get_status();


static int mxt_switch_to_bootloader_address(struct mxt_data *data);
static int mxt_switch_to_appmode_address(struct mxt_data *data);
static int mxt_get_bootloader_version(struct i2c_client *client, u8 val);
static int mxt_check_bootloader(struct i2c_client *client, unsigned int state);
static int mxt_unlock_bootloader(struct i2c_client *client);
static int mxt_fw_write(struct i2c_client *client, const u8 *data, unsigned int frame_size);
static int __mxt_read_reg(struct i2c_client *client, u16 reg, u16 len, void *val);
static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val);
static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val);
static int mxt_read_object_table(struct i2c_client *client, u16 reg, u16 obj_size, u8 *object_buf);
static struct mxt_object * mxt_get_object(struct mxt_data *data, u8 type);
static int mxt_check_message_length(struct mxt_data *data);
static int mxt_read_message(struct mxt_data *data, struct mxt_message *message);
static int mxt_read_message_reportid(struct mxt_data *data, struct mxt_message *message, u8 reportid);
static int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *val);
static int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, u8 val);
static int mxt_make_highchg(struct mxt_data *data);
//static void mxt_input_report(struct mxt_data *data, int single_id);
static void mxt_input_report(struct mxt_data *data);
static void mxt_process_touchevent(struct mxt_data *data, struct mxt_message *message, int id);
static void mxt_process_onetouch_gesture(struct mxt_data *data, struct mxt_message *message, int id);
static void mxt_process_twotouch_gesture(struct mxt_data *data, struct mxt_message *message, int id);
static irqreturn_t mxt_interrupt(int irq, void *dev_id);
static void mxt_process_irq(struct work_struct *work);
static int mxt_read_current_crc(struct mxt_data *data, unsigned long *crc);
static int mxt_download_txt_config(struct mxt_data *data, const char *fn);
static int mxt_download_bin_config(struct mxt_data *data, const char *fn);
static int mxt_bkup_nv(struct mxt_data *data);
static int mxt_soft_reset(struct mxt_data *data, u8 value);
static int mxt_set_power_cfg(struct mxt_data *data, u8 mode);
static int mxt_read_power_cfg(struct mxt_data *data, u8 *actv_cycle_time, u8 *idle_cycle_time);
static int mxt_check_power_cfg_post_reset(struct mxt_data *data);
static int mxt_probe_power_cfg(struct mxt_data *data);
static int mxt_check_reg_init(struct mxt_data *data);
static int mxt_configure_volatile_settings(struct mxt_data *data);
static void mxt_handle_pdata(struct mxt_data *data);
static int mxt_get_info(struct mxt_data *data);
static int mxt_get_object_table(struct mxt_data *data);
static int mxt_read_resolution(struct mxt_data *data);
static int mxt_init_mt_slots(struct mxt_data *data, int use_slots);
static int mxt_initialize(struct mxt_data *data);
static int mxt_load_fw(struct device *dev, const char *fn);
static ssize_t mxt_update_fw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_update_txt_cfg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_update_bin_cfg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_pause_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t mxt_pause_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_debug_enable_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t mxt_debug_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count);
static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value);
static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count);
static ssize_t mxt_info_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t mxt_object_table_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t mxt_debug_trigger_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t mxt_debug_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#if defined(CONFIG_DEBUG_FS)
static int mxt_dbgfs_open(struct inode *inode, struct file *file);
static int mxt_dbgfs_release(struct inode *inode, struct file *file);
static ssize_t mxt_dbgfs_object_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);
static ssize_t mxt_dbgfs_object_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static int mxt_dbgfs_create(struct mxt_data *data);
static int mxt_dbgfs_destroy(struct mxt_data *data);
#endif /* dfined(CONFIG_DEBUG_FS) */
static void mxt_start(struct mxt_data *data);
static void mxt_stop(struct mxt_data *data);
static int mxt_input_open(struct input_dev *dev);
static void mxt_input_close(struct input_dev *dev);
static int __devinit mxt_probe(struct i2c_client *client, const struct i2c_device_id *id);
static int __devexit mxt_remove(struct i2c_client *client);
#if defined(CONFIG_PM)
static int mxt_suspend(struct device *dev);
static int mxt_resume(struct device *dev);
#endif /* defined(CONFIG_PM) */
#if defined (CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es);
static void mxt_late_resume(struct early_suspend *es);
#endif /* defined(CONFIG_HAS_EARLYSUSPEND) */
static int __init mxt_init(void);
static void __exit mxt_exit(void);

typedef enum __dbg_level
{
	dbg_level_highest = 0,
	dbg_level_critical = dbg_level_highest,
	dbg_level_error,
	dbg_level_warning,
	dbg_level_info,
	dbg_level_debug,
	dbg_level_verbose,
	dbg_level_lowest
}dbg_level;
#define DBG_PRINT(msg_level,...) \
	{ \
		if((msg_level) <= (cur_dbg_level)) \
		{ \
			printk(KERN_INFO __VA_ARGS__); \
		} \
	}
#define DBG_PRINT_HEX(msg_level,...) \
	{ \
		if((msg_level) <= (cur_dbg_level)) \
		{ \
			print_hex_dump(KERN_INFO, ##__VA_ARGS__); \
		} \
	}

static int cur_dbg_level = dbg_level_info;
module_param_named(debug_level, cur_dbg_level, int, S_IRUGO | S_IWUSR | S_IWGRP);
MODULE_PARM_DESC(debug_level, "Debug Level");

static int mxt_switch_to_bootloader_address(struct mxt_data *data)
{
	int i;
	struct i2c_client *client = data->client;

	if (atomic_read(&data->mxt_state) == BOOTLOADER)
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Already in bootloader state.\n", dev_name(&client->dev),__func__);
		return -EINVAL;
	}

	for (i = 0; mxt_slave_addresses[i].application != 0;  i++)
	{
		if (mxt_slave_addresses[i].application == client->addr)
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Changing to bootloader address: 0x%02x -> 0x%02x\n",
					dev_name(&client->dev),	__func__, client->addr,	mxt_slave_addresses[i].bootloader);
			client->addr = mxt_slave_addresses[i].bootloader;
			atomic_set(&data->mxt_state, BOOTLOADER);
			return 0;
		}
	}

	DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Address 0x%02x not found in address table.\n", dev_name(&client->dev),__func__, client->addr);
	return -EINVAL;
}

static int mxt_switch_to_appmode_address(struct mxt_data *data)
{
	int i;
	struct i2c_client *client = data->client;

	if (atomic_read(&data->mxt_state) == APPMODE)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Already in appmode state.\n", dev_name(&client->dev),__func__);
		return -EINVAL;
	}

	for (i = 0; mxt_slave_addresses[i].application != 0;  i++)
	{
		if (mxt_slave_addresses[i].bootloader == client->addr)
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Changing to appmode address: 0x%02x -> 0x%02x\n",
					dev_name(&client->dev),	__func__, client->addr,	mxt_slave_addresses[i].application);
			client->addr = mxt_slave_addresses[i].application;
			atomic_set(&data->mxt_state, APPMODE);
			return 0;
		}
	}

	DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Address 0x%02x not found in address table.\n", dev_name(&client->dev),__func__, client->addr);
	return -EINVAL;
}

static int mxt_get_bootloader_version(struct i2c_client *client, u8 val)
{
	u8 buf[3];

//	if (val | MXT_BOOT_EXTENDED_ID) /* Shouldn't this be AND */
	if (val & MXT_BOOT_EXTENDED_ID)
	{
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Retrieving extended mode ID information...\n", dev_name(&client->dev),__func__);
		if (i2c_master_recv(client, &buf[0], 3) != 3)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: I2C recv failed.\n", dev_name(&client->dev),__func__);
			return -EIO;
		}
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Bootloader ID: %d, Version: %d\n", dev_name(&client->dev),__func__, buf[1], buf[2]);
		return buf[0];
	}
	else
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Bootloader ID: %d\n", dev_name(&client->dev),__func__, val & MXT_BOOT_ID_MASK);
		return val;
	}
}

static int mxt_check_bootloader(struct i2c_client *client, unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: I2C recv failed.\n", dev_name(&client->dev),__func__);
		return -EIO;
	}

	switch (state)
	{
	case MXT_WAITING_BOOTLOAD_CMD:
		val = mxt_get_bootloader_version(client, val);
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_WAITING_FRAME_DATA:
	case MXT_APP_CRC_FAIL:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
		{
			goto recheck;
		}
		if (val == MXT_FRAME_CRC_FAIL)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bootloader CRC failed.\n", dev_name(&client->dev),__func__);
			return -EINVAL;
		}
		break;
	default:
		return -EINVAL;
	}

	if (val != state)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Invalid bootloader mode state 0x%x.\n", dev_name(&client->dev),__func__, val);
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: I2C send failed.\n", dev_name(&client->dev),__func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client, const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: I2C send failed.\n", dev_name(&client->dev),__func__);
		return -EIO;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client, u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: I2C transfer failed. reg=0x%02x\n", dev_name(&client->dev),__func__, reg);
		//dump_stack();
		return -EIO;
	}

	return 0;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: I2C send failed. reg=0x%02x\n", dev_name(&client->dev),__func__, reg);
		dump_stack();
		return -EIO;
	}

	return 0;
}

static int mxt_read_object_table(struct i2c_client *client, u16 reg, u16 obj_size, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, obj_size, object_buf);
}

static struct mxt_object * mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;

	for (i = 0; i < data->info.object_num; i++)
	{
		object = data->object_table + i;
		if (object->type == type)
			return object;
	}

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Invalid object type T%03d.\n", dev_name(&data->client->dev),__func__, type);
	return NULL;
}

static int mxt_check_message_length(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_object *object;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
	{
		return -EINVAL;
	}

	if (object->size > MXT_MSG_MAX_SIZE)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: msg size exceeded: max=%d, current=%d.\n", dev_name(dev),__func__, MXT_MSG_MAX_SIZE, object->size);
		return -EINVAL;
	}
	return 0;
}

static int mxt_read_message(struct mxt_data *data, struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;
	int ret;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
	{
		return -EINVAL;
	}
	reg = object->start_address;

	/* Do not read last byte which contains CRC */
	ret = __mxt_read_reg(data->client, reg, object->size - 1, message);

#if 1
	if (ret == 0 && message->reportid != MXT_RPTID_NOMSG
	    && data->debug_enabled)
		print_hex_dump(KERN_DEBUG, "MXT MSG:", DUMP_PREFIX_NONE, 16, 1,
			       message, object->size - 1, false);
#endif
	return ret;
}

static int mxt_read_message_reportid(struct mxt_data *data, struct mxt_message *message, u8 reportid)
{
	int try = 20;
	int error;

	do
	{
		error = mxt_read_message(data, message);
		if (error)
		{
			return error;
		}
		if (message->reportid == 0xff)
		{
			return -EINVAL;
		}
		if (message->reportid == reportid)
		{
			return 0;
		}
	}while(try--);
	return -EINVAL;
}

static int mxt_read_object(struct mxt_data *data, u8 type, u8 offset, u8 *val)
{
	int ret;
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
	{
		return -EINVAL;
	}
	reg = object->start_address;
	ret = __mxt_read_reg(data->client, reg + offset, 1, val);
	if(ret < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: read object %d failed: reg=%u, offset=%u.\n", dev_name(&data->client->dev),__func__, object->type, reg, offset);
	}
	return ret;
}

static int mxt_read_object_block(struct mxt_data *data, u8 type, u8 offset, u8 len, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
	{
		return -EINVAL;
	}
	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, len, val);
}

static int mxt_write_object(struct mxt_data *data, u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
	{
		return -EINVAL;
	}

	if (offset >= object->size * object->instances)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Tried to write outside object T%03d: object_size=%d, given_offset=%d.\n",
				dev_name(&data->client->dev),__func__, type, object->size, offset);
		return -EINVAL;
	}
	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message;
	int count;
	int error;

	if (mxt_read_object(data, MXT_SPT_MESSAGECOUNT_T44, MXT_MESSAGECOUNT_COUNT, (u8 *)&count))
	{
		/*
		 * If all objects report themselves then a number of messages equal to
		 * the number of report ids may be generated. Therefore a good safety
		 * heuristic is twice this number
		 */
		count = data->max_reportid * 2;
	}
	else
	{
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Pending messages: %d.\n", dev_name(dev),__func__, count);
		count += 1; /* Add one more to make sure we read all the messages */
	}
	if (data->pdata->read_chg)
	{
		for(;count > 0; count--)
		{
			error = mxt_read_message(data, &message);
			if (error)
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read message %d.\n", dev_name(dev),__func__, count);
				return error;
			}
			if(data->pdata->read_chg())
			{
				DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: CHG pin cleared.\n", dev_name(dev),__func__);
				break;
			}
		}
		if(data->pdata->read_chg())
		{
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: CHG pin cleared.\n", dev_name(dev),__func__);
		}
	}
	else
	{
		/* Read dummy message to make high CHG pin */
		do
		{
			error = mxt_read_message(data, &message);
			if (error)
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read message.\n", dev_name(dev),__func__);
				return error;
			}
		}while ((message.reportid != MXT_RPTID_NOMSG) && --count);
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: CHG pin cleared.\n", dev_name(dev),__func__);
	}

	return 0;
}

//static void mxt_input_report(struct mxt_data *data, int single_id)
static void mxt_input_report(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
//	int finger_status = finger[single_id].status;
	int finger_num = 0;
	int event_num = 0;
	int id;

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting touch event.\n", dev_name(dev),__func__);
	if(atomic_read(&data->uses_mt_slots) == MXT_MT_B) /* Type B device */
	{
		for (id = 0; id < MXT_MAX_FINGER; id++)
		{
			if (!finger[id].status)
			{
				continue;
			}
			/* Mark the beginning of the new contact packet */
			input_mt_slot(input_dev, id);
			switch(finger[id].status)
			{
			case MXT_TOUCH_STATUS_RELEASE:
				{
					DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: event[%d]: tool_finger=false.\n", dev_name(dev), __func__, id);
					input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, false);
					finger[id].status = 0;
					event_num++;
				}
				break;
			case MXT_TOUCH_STATUS_PRESS:
			case MXT_TOUCH_STATUS_MOVE:
				{
					DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: event[%d]: tool_finger=true.\n", dev_name(dev), __func__, id);
					input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, true);
				}
				/*
				 * Fall through
				 */
//			case MXT_TOUCH_STATUS_MOVE:
				{
					DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: event[%d]: touch_major=%d, mt_x=%d, mt_y=%d, mt_pressure=%d.\n",
							dev_name(dev),__func__,	id, finger[id].area, finger[id].x, finger[id].y, finger[id].pressure);
					input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[id].area);
					input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
					input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
					//input_report_abs(input_dev, ABS_MT_PRESSURE, finger[id].pressure);
					finger_num++;
					event_num++;
				}
				break;
			default:
				break;
			}
		}
	}
	else if(atomic_read(&data->uses_mt_slots) == MXT_MT_A) /* Type A device */
	{
		for (id = 0; id < MXT_MAX_FINGER; id++)
		{
			if (!finger[id].status)
			{
				continue;
			}

			if (finger[id].status == MXT_TOUCH_STATUS_RELEASE)
			{
				finger[id].status = 0;
			}
			else
			{
				DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: event[%d]: touch_major=%d, mt_x=%d, mt_y=%d, mt_pressure=%d.\n",
						dev_name(dev),__func__,	id, finger[id].area, finger[id].x, finger[id].y, finger[id].pressure);
				input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR, finger[id].area);
				input_report_abs(input_dev, ABS_MT_POSITION_X, finger[id].x);
				input_report_abs(input_dev, ABS_MT_POSITION_Y, finger[id].y);
				//input_report_abs(input_dev, ABS_MT_PRESSURE, finger[id].pressure);
				/* Mark the end of the contact packet */
				input_mt_sync(input_dev);
				finger_num++;
				event_num++;
			}
		}
	}
	else /* Does not use MT protocol */
	{
#if 0
		if(finger_status == MXT_TOUCH_STATUS_RELEASE)
		{
			finger[single_id].status = 0;
		}
		else
		{
			finger_num++;
			event_num++;
		}
#endif
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

#if defined(MXT_SUPPORT_ST)
#if 0
	if((finger_status != 0 ) && (finger_status != MXT_TOUCH_STATUS_RELEASE))
	{
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
		//input_report_abs(input_dev, ABS_PRESSURE, finger[single_id].pressure);
	}
#endif
#endif /* defined(MXT_SUPPORT_ST) */

	/* Gesture processing */
	if(data->pdata->use_fw_gestures != 0)
	{
		if((finger_num > data->mxt_n_active_fingers))
		{
			/*Generate a touch down event */
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT1X event: hat1x=%u\n", dev_name(&(data->client->dev)), __func__, GESTURE_TD);
			input_report_abs(input_dev, ABS_HAT1X, GESTURE_TD);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2X event: hat2x=%u\n", dev_name(&(data->client->dev)), __func__, finger_num);
			input_report_abs(input_dev, ABS_HAT2X, finger_num);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2Y event: hat2y=%u\n", dev_name(&(data->client->dev)), __func__, 1);
			input_report_abs(input_dev, ABS_HAT2Y, 1);
		}

		if((finger_num <= 1) && (data->current_bn_onetouch_gesture != GESTURE_NONE) && (data->mxt_onetouch_gesture_active != 0))
		{
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT1X event: hat1x=%u\n", dev_name(&(data->client->dev)), __func__, data->current_bn_onetouch_gesture);
			input_report_abs(input_dev, ABS_HAT1X, data->current_bn_onetouch_gesture);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2X event: hat2x=%u\n", dev_name(&(data->client->dev)), __func__, finger_num);
			input_report_abs(input_dev, ABS_HAT2X, finger_num);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2Y event: hat2y=%u\n", dev_name(&(data->client->dev)), __func__, data->onetouch_gesture_count);
			input_report_abs(input_dev, ABS_HAT2Y, data->onetouch_gesture_count);
			data->mxt_onetouch_gesture_active = 0;
		}
		else if((finger_num == 2) && (data->current_bn_twotouch_gesture != GESTURE_NONE) && (data->mxt_twotouch_gesture_active != 0))
		{
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT1X event: hat1x=%u\n", dev_name(&(data->client->dev)), __func__, data->current_bn_twotouch_gesture);
			input_report_abs(input_dev, ABS_HAT1X, data->current_bn_twotouch_gesture);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2X event: hat2x=%u\n", dev_name(&(data->client->dev)), __func__, finger_num);
			input_report_abs(input_dev, ABS_HAT2X, finger_num);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2Y event: hat2y=%u\n", dev_name(&(data->client->dev)), __func__, data->twotouch_gesture_count);
			input_report_abs(input_dev, ABS_HAT2Y, data->twotouch_gesture_count);
			data->mxt_twotouch_gesture_active = 0;
		}

		if((finger_num < data->mxt_n_active_fingers))
		{
			/*Generate a lift off event */
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT1X event: hat1x=%u\n", dev_name(&(data->client->dev)), __func__, GESTURE_LO);
			input_report_abs(input_dev, ABS_HAT1X, GESTURE_LO);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2X event: hat2x=%u\n", dev_name(&(data->client->dev)), __func__, finger_num);
			input_report_abs(input_dev, ABS_HAT2X, finger_num);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2Y event: hat2y=%u\n", dev_name(&(data->client->dev)), __func__, 1);
			input_report_abs(input_dev, ABS_HAT2Y, 1);
		}

		if((finger_num == 0) || (finger_num > 2))
		{
			/* Mark the end of transfer */
			input_sync(input_dev);

			/* Report GESTURE_NONE */
			data->current_bn_onetouch_gesture = GESTURE_NONE;
			data->current_bn_twotouch_gesture = GESTURE_NONE;
			data->current_mxt_onetouch_gesture = 0x00;
			data->current_mxt_twotouch_gesture = 0x00;
			data->onetouch_gesture_count = 0;
			data->twotouch_gesture_count = 0;
			data->mxt_onetouch_gesture_active = 0;
			data->mxt_twotouch_gesture_active = 0;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT1X event: hat1x=%u\n", dev_name(&(data->client->dev)), __func__, data->current_bn_onetouch_gesture);
			input_report_abs(input_dev, ABS_HAT1X, data->current_bn_onetouch_gesture);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2X event: hat2x=0\n", dev_name(&(data->client->dev)), __func__);
			input_report_abs(input_dev, ABS_HAT2X, 0);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reporting HAT2Y event: hat2y=%u\n", dev_name(&(data->client->dev)), __func__, data->onetouch_gesture_count);
			input_report_abs(input_dev, ABS_HAT2Y, data->onetouch_gesture_count);
		}
	}

	data->mxt_n_active_fingers = finger_num;

	/* Mark the end of transfer */
	input_sync(input_dev);

}

/* WESN */
#define DIR_NORTH_BIT	(0)
#define DIR_SOUTH_BIT	(1)
#define DIR_EAST_BIT	(2)
#define DIR_WEST_BIT	(3)
#define DIR_NORTH_MASK	(1<<DIR_NORTH_BIT)
#define DIR_SOUTH_MASK	(1<<DIR_SOUTH_BIT)
#define DIR_EAST_MASK	(1<<DIR_EAST_BIT)
#define DIR_WEST_MASK	(1<<DIR_WEST_BIT)
enum
{
	DIR_NONE = 0x00,
	DIR_EAST = DIR_EAST_MASK,
	DIR_NORTH_EAST = DIR_NORTH_MASK | DIR_EAST_MASK,
	DIR_NORTH = DIR_NORTH_MASK,
	DIR_NORTH_WEST = DIR_NORTH_MASK | DIR_WEST_MASK,
	DIR_WEST = DIR_WEST_MASK,
	DIR_SOUTH_WEST = DIR_SOUTH_MASK | DIR_WEST_MASK,
	DIR_SOUTH = DIR_SOUTH_MASK,
	DIR_SOUTH_EAST = DIR_SOUTH_MASK | DIR_EAST_MASK,
};

static int mxt_gesture_get_dir(struct mxt_data * data, u8 dir)
{
	u8 converted_dir;
	u8 orient = data->pdata->orient;
	int ret;

	if(MXT_IS_DIR_E((dir)))
	{
		converted_dir = DIR_EAST;
	}
	else if(MXT_IS_DIR_NE((dir)))
	{
		converted_dir = DIR_NORTH_EAST;
	}
	else if(MXT_IS_DIR_N((dir)))
	{
		converted_dir = DIR_NORTH;
	}
	else if(MXT_IS_DIR_NW((dir)))
	{
		converted_dir = DIR_NORTH_WEST;
	}
	else if(MXT_IS_DIR_W((dir)))
	{
		converted_dir = DIR_WEST;
	}
	else if(MXT_IS_DIR_SW((dir)))
	{
		converted_dir = DIR_SOUTH_WEST;
	}
	else if(MXT_IS_DIR_S((dir)))
	{
		converted_dir = DIR_SOUTH;
	}
	else if(MXT_IS_DIR_SE((dir)))
	{
		converted_dir = DIR_SOUTH_EAST;
	}
	else
	{
		converted_dir = DIR_NONE;
	}

#if 0
	if(data->pdata->orient & MXT_Y_INVERT)
	{
		/* Switch NORTH and SOUTH bits */
		converted_dir =	(converted_dir & ~(DIR_NORTH_MASK | DIR_SOUTH_MASK)) | ((converted_dir & DIR_NORTH_MASK) << (DIR_SOUTH_BIT - DIR_NORTH_BIT)) | ((converted_dir & DIR_SOUTH_MASK) >> (DIR_SOUTH_BIT - DIR_NORTH_BIT));
	}

	if(data->pdata->orient & MXT_X_INVERT)
	{
		/* Switch EAST and WEST bits */
		converted_dir =	(converted_dir & ~(DIR_WEST_MASK | DIR_EAST_MASK)) | ((converted_dir & DIR_EAST_MASK) << (DIR_WEST_BIT - DIR_EAST_BIT)) | ((converted_dir & DIR_WEST_MASK) >> (DIR_WEST_BIT - DIR_EAST_BIT));
	}

	if(data->pdata->orient & MXT_XY_SWITCH)
	{
		/* Switch NORTH and EAST bits */
		converted_dir =	(converted_dir & ~(DIR_NORTH_MASK | DIR_EAST_MASK)) | ((converted_dir & DIR_NORTH_MASK) << (DIR_EAST_BIT - DIR_NORTH_BIT)) | ((converted_dir & DIR_EAST_MASK) >> (DIR_EAST_BIT - DIR_NORTH_BIT));
		/* Switch SOUTH and WEST bits */
		converted_dir =	(converted_dir & ~(DIR_SOUTH_MASK | DIR_WEST_MASK)) | ((converted_dir & DIR_SOUTH_MASK) << (DIR_WEST_BIT - DIR_SOUTH_BIT)) | ((converted_dir & DIR_WEST_MASK) >> (DIR_WEST_BIT - DIR_SOUTH_BIT));
	}
#else
	/* We just need to just invert the Y axis because the (0,0) is on the top-left of the screen, but the mxt reports the direction based on bottom-left origin */
	converted_dir =	(converted_dir & ~(DIR_NORTH_MASK | DIR_SOUTH_MASK)) | ((converted_dir & DIR_NORTH_MASK) << (DIR_SOUTH_BIT - DIR_NORTH_BIT)) | ((converted_dir & DIR_SOUTH_MASK) >> (DIR_SOUTH_BIT - DIR_NORTH_BIT));
#endif
	//DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: dir=0x%02x,\tconverted_dir=0x%02x.\n", dev_name(&data->client->dev),__func__, dir, converted_dir);

	switch(converted_dir)
	{
	case DIR_EAST:
		{
			ret = 0x14;
		}
		break;
	case DIR_NORTH_EAST:
		{
			ret = 0x12;
		}
		break;
	case DIR_NORTH:
		{
			ret = 0x10;
		}
		break;
	case DIR_NORTH_WEST:
		{
			ret = 0x1E;
		}
		break;
	case DIR_WEST:
		{
			ret = 0x1C;
		}
		break;
	case DIR_SOUTH_WEST:
		{
			ret = 0x1A;
		}
		break;
	case DIR_SOUTH:
		{
			ret = 0x18;
		}
		break;
	case DIR_SOUTH_EAST:
		{
			ret = 0x16;
		}
		break;
	default:
		{
			ret = 0x00;
		}
		break;
	}


	return ret;
}
static void mxt_process_onetouch_gesture(struct mxt_data *data, struct mxt_message *message, int id)
{
	u8 event = message->message[0] & MXT_ONETOUCH_MSG_EVENT_MASK;
	int bn_event;
	int x;
	int y;
	u8 dir = message->message[4];
	int dist;

	if (atomic_read(&data->mxt_power_mode) == MXT_POWER_MODE_DEEP_SLEEP)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Device inactive.\n", dev_name(&data->client->dev),__func__);
		return;
	}

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0x0f);
	y = (message->message[2] << 4) | ((message->message[3] & 0x0f));
	dist = (message->message[6] << 8) | (message->message[5]);

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: event=0x%02x, x=0x%04x, y=0x%04x, dir=0x%02x, dist=0x%02x.\n", dev_name(&data->client->dev),__func__,
			event, x, y, dir, dist);


	switch(event)
	{
	case MXT_ONETOUCH_MSG_EVENT_PRESS:
		{
			bn_event = GESTURE_TD;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_PRESS: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_RELEASE:
		{
			bn_event = GESTURE_LO;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_RELEASE: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_TAP:
		{
			bn_event = GESTURE_SC;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_TAP: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_DOUBLETAP:
		{
			bn_event = GESTURE_DC;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_DOUBLETAP: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_FLICK:
		{
			bn_event = mxt_gesture_get_dir(data, dir);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_FLICK: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_DRAG:
		{
			bn_event = mxt_gesture_get_dir(data, dir);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_DRAG: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_SHORTPRESS:
		{
			bn_event = GESTURE_NONE;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_SHORTPRESS: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_LONGPRESS:
		{
			bn_event = GESTURE_NONE;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_LONGPRESS: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_REPEATTPRESS:
		{
			bn_event = GESTURE_NONE;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_REPEATTPRESS: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_TAPANDPRESS:
		{
			bn_event = GESTURE_NONE;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_TAPANDPRESS: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	case MXT_ONETOUCH_MSG_EVENT_THROW:
		{
			bn_event = mxt_gesture_get_dir(data, dir);
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_THROW: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	default:
		{
			bn_event = GESTURE_NONE;
			event = 0x00;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_ONETOUCH_MSG_EVENT_DEFAULT: 0x%02x.\n", dev_name(&data->client->dev),__func__, bn_event);
		}
		break;
	}

	data->current_mxt_onetouch_gesture = event;
	if(bn_event == data->current_bn_onetouch_gesture)
	{
		data->onetouch_gesture_count++;
	}
	else
	{
		data->current_bn_onetouch_gesture = bn_event;
		data->onetouch_gesture_count = 1;
	}

	if(data->current_bn_onetouch_gesture != GESTURE_NONE)
	{
		data->mxt_onetouch_gesture_active = 1;
	}
	else
	{
		data->mxt_onetouch_gesture_active = 0;
		data->onetouch_gesture_count = 0;
		data->current_mxt_onetouch_gesture = 0;
	}
}

static void mxt_process_twotouch_gesture(struct mxt_data *data, struct mxt_message *message, int id)
{
	u8 event = message->message[0] & MXT_TWOTOUCH_MSG_STATUS_MASK;
	int bn_event;
	int x;
	int y;
	u8 angle = message->message[4];
	int sep;

	if (atomic_read(&data->mxt_power_mode) == MXT_POWER_MODE_DEEP_SLEEP)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Device inactive.\n", dev_name(&data->client->dev),__func__);
		return;
	}

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0x0f);
	y = (message->message[2] << 4) | ((message->message[3] & 0x0f));
	sep = (message->message[6] << 8) | (message->message[5]);

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: event=0x%02x, x=0x%04x, y=0x%04x, angle=0x%02x, sep=0x%02x.\n", dev_name(&data->client->dev),__func__,
			event, x, y, angle, sep);

	if(event & MXT_TWOTOUCH_MSG_STATUS_STRETCH)
	{
		bn_event = GESTURE_ZI;
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_TWOTOUCH_MSG_STATUS_STRETCH.\n", dev_name(&data->client->dev),__func__);
	}
	else if(event & MXT_TWOTOUCH_MSG_STATUS_PINCH)
	{
		bn_event = GESTURE_ZO;
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_TWOTOUCH_MSG_STATUS_PINCH.\n", dev_name(&data->client->dev),__func__);
	}
	else if(event & MXT_TWOTOUCH_MSG_STATUS_ROTATE)
	{
		bn_event = GESTURE_NONE;
		if(event & MXT_TWOTOUCH_MSG_STATUS_ROTATEDIR)
		{
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_TWOTOUCH_MSG_STATUS_ROTATE ANTICLK.\n", dev_name(&data->client->dev),__func__);
		}
		else
		{
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT_TWOTOUCH_MSG_STATUS_ROTATE CLK.\n", dev_name(&data->client->dev),__func__);
		}
		event = MXT_TWOTOUCH_MSG_STATUS_ROTATE;
	}
	else
	{
		event = 0x00;
		data->current_bn_twotouch_gesture = GESTURE_NONE;
	}

	data->current_mxt_twotouch_gesture = event;
	if(bn_event == data->current_bn_twotouch_gesture)
	{
		data->twotouch_gesture_count++;
	}
	else
	{
		data->current_bn_twotouch_gesture = bn_event;
		data->twotouch_gesture_count = 1;
	}

	if(data->current_bn_twotouch_gesture != GESTURE_NONE)
	{
		data->mxt_twotouch_gesture_active = 1;
	}
	else
	{
		data->mxt_twotouch_gesture_active = 0;
		data->twotouch_gesture_count = 0;
		data->current_mxt_twotouch_gesture = 0;
	}
}

static void mxt_process_touchevent(struct mxt_data *data, struct mxt_message *message, int id)
{
	struct mxt_finger *finger = data->finger;
	u8 status = message->message[0];
	int x;
	int y;
	int area;
	int pressure;

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Processing touch event.\n", dev_name(&data->client->dev),__func__);
	if (atomic_read(&data->mxt_power_mode) == MXT_POWER_MODE_DEEP_SLEEP)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Device inactive.\n", dev_name(&data->client->dev),__func__);
		return;
	}

	/* Check the touch is present on the screen */
	if (!(status & MXT_TOUCH_STATUS_DETECT))
	{
		if (status & MXT_TOUCH_STATUS_RELEASE)
		{
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: [%d] released.\n", dev_name(&data->client->dev),__func__, id);
			finger[id].status = MXT_TOUCH_STATUS_RELEASE;
//			mxt_input_report(data, id);
		}
		return;
	}

	/* Check only AMP detection */
	if (!(status & (MXT_TOUCH_STATUS_PRESS | MXT_TOUCH_STATUS_MOVE)))
		return;

	x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
	y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	if (data->max_x <= 1024)
	{
		x = x >> 2;
	}
	if (data->max_y <= 1024)
	{
		y = y >> 2;
	}

	area = message->message[4];
	pressure = message->message[5];

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: [%d] %s: x=%d, y=%d, area=%d, pressure=%d.\n",
			dev_name(&data->client->dev),__func__, id, status & MXT_TOUCH_STATUS_MOVE ? "moved" : "pressed", x, y, area, pressure);

	finger[id].status = status & MXT_TOUCH_STATUS_MOVE ?	MXT_TOUCH_STATUS_MOVE : MXT_TOUCH_STATUS_PRESS;
	finger[id].x = x;
	finger[id].y = y;
	finger[id].area = area;
	finger[id].pressure = pressure;

	//mxt_input_report(data, id);
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;

	if (atomic_read(&data->mxt_state) != APPMODE)
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Ignoring IRQ: device not in appmode state.\n", dev_name(&data->client->dev),__func__);
		return IRQ_HANDLED;
	}

	if(0 != queue_work(data->mxt_workqueue, &(data->mxt_irq_work)))
	{
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Interrupts coming in too fast.\n", dev_name(&data->client->dev),__func__);
	}

	return IRQ_HANDLED;
}

static void mxt_process_irq(struct work_struct *irq_work)
{
	struct mxt_data *data = container_of(irq_work, struct mxt_data, mxt_irq_work);
	struct mxt_object *touch_object, *command_object, *onetouch_gesture_object, *twotouch_gesture_object;
	struct mxt_message message;
	int event_id;
	u8 reportid;
	u8 nMsg;
	struct device *dev = &(data->client->dev);

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Processing IRQ.\n", dev_name(dev),__func__);
	command_object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	touch_object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
	onetouch_gesture_object = mxt_get_object(data, MXT_PROCI_ONETOUCH_T24);
	twotouch_gesture_object = mxt_get_object(data, MXT_PROCI_TWOTOUCH_T27);
	if((touch_object == NULL) || (command_object == NULL))
	{
		/* Just read all the messages */
		nMsg = data->max_reportid * 2;
		do
		{
			if (mxt_read_message(data, &message))
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read message.\n", dev_name(dev),__func__);
				return;
			}
		} while((message.reportid != MXT_RPTID_NOMSG) && --nMsg);
		return;
	}
	else if (mxt_read_object(data, MXT_SPT_MESSAGECOUNT_T44, MXT_MESSAGECOUNT_COUNT, &nMsg))
	{
		/* We dont have the number of pending messages. Just read till the report id is invalid */
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Unable to read the number of pending messages.\n", dev_name(dev),__func__);
		nMsg = data->max_reportid * 2;
		do
		{
			if (mxt_read_message(data, &message))
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read message.\n", dev_name(dev),__func__);
				message.reportid = MXT_RPTID_NOMSG;
				break;
			}
			reportid = message.reportid;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Got message with report_id=%d.\n", dev_name(dev),__func__, reportid);
			if (data->debug_enabled)
			{
				DBG_PRINT_HEX(dbg_level_debug, MXT_TAG ": mxt_process_irq(): ", DUMP_PREFIX_NONE, 16, 1, &message, sizeof(struct mxt_message), false);
			}

			if ((reportid >= touch_object->min_reportid) && (reportid <= touch_object->max_reportid))
			{
				DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Dispatching touch event.\n", dev_name(dev),__func__);
				event_id = reportid - touch_object->min_reportid;
				mutex_lock(&data->mutex_fingers);
				mxt_process_touchevent(data, &message, event_id);
				mutex_unlock(&data->mutex_fingers);
			}
			else if ((reportid == command_object->max_reportid))
			{
				DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Command object error message: 0x%02x.\n", dev_name(dev),__func__, message.message[1]);
			}
			else
			{
				if((onetouch_gesture_object != NULL) && (reportid >= onetouch_gesture_object->min_reportid) && (reportid <= onetouch_gesture_object->max_reportid))
				{
					DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Dispatching one touch gesture event.\n", dev_name(dev),__func__);
					event_id = reportid - onetouch_gesture_object->min_reportid;
					mutex_lock(&data->mutex_onetouch_gestures);
					mxt_process_onetouch_gesture(data, &message, event_id);
					mutex_unlock(&data->mutex_onetouch_gestures);
				}
				else if((twotouch_gesture_object != NULL) && (reportid >= twotouch_gesture_object->min_reportid) && (reportid <= twotouch_gesture_object->max_reportid))
				{
					DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Dispatching two touch gesture event.\n", dev_name(dev),__func__);
					event_id = reportid - twotouch_gesture_object->min_reportid;
					mutex_lock(&data->mutex_twotouch_gestures);
					mxt_process_twotouch_gesture(data, &message, event_id);
					mutex_unlock(&data->mutex_twotouch_gestures);
				}
			}
		} while ((message.reportid != MXT_RPTID_NOMSG) && --nMsg);
	}
	else
	{
		dev_dbg(dev, MXT_TAG ": %s(): Pending messages %d\n", __func__, nMsg);
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Pending messages=%d.\n", dev_name(dev),__func__, nMsg);
		for(;nMsg > 0; nMsg--)
		{
			if (mxt_read_message(data, &message))
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read message.\n", dev_name(dev),__func__);
				message.reportid = MXT_RPTID_NOMSG;
				break;
			}
			reportid = message.reportid;
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Got message with report_id=%d.\n", dev_name(dev),__func__, reportid);
			if (data->debug_enabled)
			{
				DBG_PRINT_HEX(dbg_level_debug, MXT_TAG ": mxt_process_irq(): ", DUMP_PREFIX_NONE, 16, 1, &message, sizeof(struct mxt_message), false);
			}

			if ((reportid >= touch_object->min_reportid) && (reportid <= touch_object->max_reportid))
			{
				DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Dispatching touch event.\n", dev_name(dev),__func__);
				event_id = reportid - touch_object->min_reportid;
				mutex_lock(&data->mutex_fingers);
				mxt_process_touchevent(data, &message, event_id);
				mutex_unlock(&data->mutex_fingers);
			}
			else if ((reportid == command_object->max_reportid))
			{
				DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Command object error message: 0x%02x.\n", dev_name(dev),__func__, message.message[1]);
			}
			else
			{
				if((onetouch_gesture_object != NULL) && (reportid >= onetouch_gesture_object->min_reportid) && (reportid <= onetouch_gesture_object->max_reportid))
				{
					DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Dispatching one touch gesture event.\n", dev_name(dev),__func__);
					event_id = reportid - onetouch_gesture_object->min_reportid;
					mutex_lock(&data->mutex_onetouch_gestures);
					mxt_process_onetouch_gesture(data, &message, event_id);
					mutex_unlock(&data->mutex_onetouch_gestures);
				}
				else if((twotouch_gesture_object != NULL) && (reportid >= twotouch_gesture_object->min_reportid) && (reportid <= twotouch_gesture_object->max_reportid))
				{
					DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Dispatching two touch gesture event.\n", dev_name(dev),__func__);
					event_id = reportid - twotouch_gesture_object->min_reportid;
					mutex_lock(&data->mutex_twotouch_gestures);
					mxt_process_twotouch_gesture(data, &message, event_id);
					mutex_unlock(&data->mutex_twotouch_gestures);
				}
			}
		}
	}

	/* Report all the input events */
	mutex_lock(&data->mutex_fingers);
	mutex_lock(&data->mutex_onetouch_gestures);
	mutex_lock(&data->mutex_twotouch_gestures);
	mxt_input_report(data);
	mutex_unlock(&data->mutex_twotouch_gestures);
	mutex_unlock(&data->mutex_onetouch_gestures);
	mutex_unlock(&data->mutex_fingers);
}

static int mxt_read_current_crc(struct mxt_data *data, unsigned long *crc)
{
	struct device *dev = &data->client->dev;
	int error;
	struct mxt_message message;
	struct mxt_object *object;
	int nRetries = 100;

	object = mxt_get_object(data, MXT_GEN_COMMAND_T6);
	if (!object)
	{
		return -EIO;
	}

	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Disabling touch irq.\n", dev_name(dev),__func__);
	disable_irq(data->irq);

	error = mxt_make_highchg(data);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Could not deassert touch irq.\n", dev_name(dev),__func__);
	}
	/* Try to read the config checksum of the existing cfg */
	mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_REPORTALL, 1);

	do
	{
		msleep(10);
		if(data->pdata->read_chg != NULL)
		{
			if(!data->pdata->read_chg())
			{
				break;
			}

		}
	} while(nRetries--);

	/* Read message from command processor, which only has one report ID */
	error = mxt_read_message_reportid(data, &message, object->max_reportid);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to retrieve CRC.\n", dev_name(dev),__func__);
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Enabling touch irq.\n", dev_name(dev),__func__);
		enable_irq(data->irq);
		return error;
	}
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Enabling touch irq.\n", dev_name(dev),__func__);
	enable_irq(data->irq);

	/* Bytes 1-3 are the checksum. */
	*crc =     message.message[1]
		| (message.message[2] << 8)
		| (message.message[3] << 16);

	return 0;
}

static int mxt_download_txt_config(struct mxt_data *data, const char *fn)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	struct mxt_object *object;
	const struct firmware *cfg = NULL;
	int ret;
	int offset;
	int pos;
	int i;
	//unsigned long current_crc;
	unsigned long info_crc;
	unsigned long config_crc;
	unsigned int type, instance, size;
	u8 val;
	u16 reg;
	int suspend_enabled;

	/* Disable suspend during cfg upgrade */
	suspend_enabled = atomic_cmpxchg(&data->do_not_suspend_mxt, 0, 1); /* After this do_not_suspend_mxt will be 1, and old value will be in suspend_enabled */

	/* Disable cfg upgrade if we are in suspend */
	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_ON)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Controller is powered off...Make sure the controller and display is on during cfg upgrade.\n", dev_name(dev), __func__);
		ret = -EAGAIN;
		goto err_return;
	}

	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Requesting config file %s.\n", dev_name(dev),__func__, fn);
	ret = request_firmware(&cfg, fn, dev);
	if (ret < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Config file %s does not exist.\n", dev_name(dev),__func__, fn);
		return 0;
	}
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Config file %s downloaded successfully.\n", dev_name(dev),__func__, fn);

#if 0
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO:\n\t", dev_name(dev),__func__);
	for(i=0; i < cfg->size; i++)
	{
		DBG_PRINT(dbg_level_info, "%c", cfg->data[i]);
	}
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO:\n", dev_name(dev),__func__);
#endif /* 0 */

#if 0
	ret = mxt_read_current_crc(data, &current_crc);
	if (ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read current crc\n", dev_name(dev),__func__);
		goto release;
	}
	else
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Current crc = 0x%06x\n", dev_name(dev),__func__, current_crc);
	}
#endif /* 0 */

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC)))
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Wrong magic number.\n", dev_name(dev),__func__);
		ret = -EINVAL;
		goto release;
	}
	pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	for (i = 0; i < sizeof(struct mxt_info); i++)
	{
		ret = sscanf(cfg->data + pos, "%hhx%n", ((unsigned char *)&cfg_info) + i, &offset);
		if (ret <= 0)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for info.\n", dev_name(dev),__func__, pos);
			ret = -EINVAL;
			goto release;
		}
		pos += offset;
	}
	DBG_PRINT_HEX(dbg_level_debug, MXT_TAG ": mxt_download_txt_config(): info: ", DUMP_PREFIX_NONE, 16, 1, &cfg_info, sizeof(cfg_info), false);

	if (cfg_info.family_id != data->info.family_id)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Family ID mismatch: expected=%d given=%d.\n", dev_name(dev),__func__, data->info.family_id, cfg_info.family_id);
		ret = -EINVAL;
		goto release;
	}

	if (cfg_info.variant_id != data->info.variant_id)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Variant ID mismatch: expected=%d given=%d.\n", dev_name(dev),__func__, data->info.variant_id, cfg_info.variant_id);
		ret = -EINVAL;
		goto release;
	}

	if (cfg_info.version != data->info.version)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Version mismatch: expected=%d given=%d ... still proceeding with the update.\n", dev_name(dev),__func__, data->info.version, cfg_info.version);
	}

	if (cfg_info.build != data->info.build)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Build mismatch: expected=%d given=%d ... still proceeding with the update.\n", dev_name(dev),__func__, data->info.build, cfg_info.build);
	}

	ret = sscanf(cfg->data + pos, "%lx%n", &info_crc, &offset);
	if (ret <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for info crc.\n", dev_name(dev),__func__, pos);
		ret = -EINVAL;
		goto release;
	}
	pos += offset;

	/* Check config CRC */
	ret = sscanf(cfg->data + pos, "%lx%n", &config_crc, &offset);
	if (ret <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for config crc.\n", dev_name(dev),__func__, pos);
		ret = -EINVAL;
		goto release;
	}
	pos += offset;
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: info_crc=0x%06x, config_crc=0x%06x.\n", dev_name(dev),__func__, (unsigned int)info_crc, (unsigned int)config_crc);

#if 0
	if (current_crc == config_crc)
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: config_crc 0x%06x ... MATCH.\n", dev_name(dev),__func__, (unsigned int) current_crc);
		ret = 0;
		goto release;
	}
	else
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Config crc mismatch: current=0x%06x given=0x%06x ... proceeding with the update.\n", dev_name(dev),__func__, (unsigned int) current_crc, (unsigned int) config_crc);
	}
#endif /* 0 */

	while (pos < cfg->size)
	{
		/* Read type, instance, length */
		ret = sscanf(cfg->data + pos, "%x %x %x%n", &type, &instance, &size, &offset);
		if (ret == 0)
		{
			/* EOF */
			ret = 1;
			goto release;
		}
		else if (ret < 0)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for object type/instance/size.\n", dev_name(dev),__func__, pos);
			ret = -EINVAL;
			goto release;
		}
		pos += offset;
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: type=%03d, instance=%02d, size=0x%08x.\n", dev_name(dev),__func__, type, instance, size);

		object = mxt_get_object(data, type);
		if (!object)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Object T%03d not present...skipping.\n", dev_name(dev),__func__, type);
			for (i = 0; i < size; i++)
			{
				ret = sscanf(cfg->data + pos, "%hhx%n", &val, &offset);
				if (ret <= 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for object addr/value.\n", dev_name(dev),__func__, pos);
					goto release;
				}
				pos += offset;
			}
			ret = -EINVAL;
			continue;
		}

		if (size > object->size)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Size overflow for T%03d: actual_size=%d, cfg_size=%d.\n", dev_name(dev),__func__, type, object->size, size);
			for (i = 0; i < size; i++)
			{
				ret = sscanf(cfg->data + pos, "%hhx%n", &val, &offset);
				if (ret <= 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for object addr/value.\n", dev_name(dev),__func__, pos);
					goto release;
				}
				pos += offset;
			}
			ret = -EINVAL;
			continue;
		}

		if (instance >= object->instances)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Instance overflow for T%03d: actual_instances=%d, cfg_instance=%d.\n", dev_name(dev),__func__, type, object->instances, instance);
			for (i = 0; i < size; i++)
			{
				ret = sscanf(cfg->data + pos, "%hhx%n", &val, &offset);
				if (ret <= 0)
				{
					DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for object addr/value.\n", dev_name(dev),__func__, pos);
					goto release;
				}
				pos += offset;
			}
			ret = -EINVAL;
			continue;
		}

		reg = object->start_address + object->size * instance;
		for (i = 0; i < size; i++)
		{
			ret = sscanf(cfg->data + pos, "%hhx%n", &val, &offset);
			if (ret <= 0)
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Bad format @ offset %d for object addr/value.\n", dev_name(dev),__func__, pos);
				ret = -EINVAL;
				goto release;
			}
			pos += offset;

			ret = mxt_write_reg(data->client, reg + i, val);
			if (ret)
			{
				goto release;
			}
		}

		/* If firmware is upgraded, new bytes may be added to end of
		 * objects. It is generally forward compatible to zero these
		 * bytes - previous behaviour will be retained. However
		 * this does invalidate the CRC and will force a config
		 * download every time until the configuration is updated */
		if (size < object->size)
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Zeroing %d byte(s) in T%03d.\n", dev_name(dev),__func__, object->size - size, type);
			for (i = size + 1; i < object->size; i++)
			{
				ret = mxt_write_reg(data->client, reg + i, 0);
				if (ret)
				{
					goto release;
				}
			}
		}
	}

release:
	release_firmware(cfg);
err_return:
	/* Restore the do_not_suspend_mxt */
	atomic_cmpxchg(&data->do_not_suspend_mxt, !suspend_enabled, suspend_enabled); /* After this do_not_suspend_mxt will be suspend_enabled */
	return ret;
}


static int mxt_download_bin_config(struct mxt_data *data, const char *fn)
{
	struct device *dev = &data->client->dev;
	struct mxt_info cfg_info;
	struct mxt_object *object;
	const struct firmware *cfg = NULL;
	int ret;
	int pos;
	int i;
	//unsigned long current_crc;
	unsigned long info_crc;
	unsigned long config_crc;
	unsigned int type, instance, size;
	u8 val;
	u16 reg;
	int suspend_enabled;

	/* Disable suspend during cfg upgrade */
	suspend_enabled = atomic_cmpxchg(&data->do_not_suspend_mxt, 0, 1); /* After this do_not_suspend_mxt will be 1, and old value will be in suspend_enabled */

	/* Disable cfg upgrade if we are in suspend */
	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_ON)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Controller is powered off...Make sure the controller and display is on during cfg upgrade.\n", dev_name(dev), __func__);
		ret = -EAGAIN;
		goto err_return;
	}

	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Requesting config file %s.\n", dev_name(dev),__func__, fn);
	ret = request_firmware(&cfg, fn, dev);
	if (ret < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Config file %s does not exist.\n", dev_name(dev),__func__, fn);
		return 0;
	}
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Config file %s downloaded successfully.\n", dev_name(dev),__func__, fn);
	DBG_PRINT_HEX(dbg_level_debug, MXT_TAG ": mxt_download_bin_config(): ", DUMP_PREFIX_NONE, 16, 1, cfg->data, cfg->size, false);

#if 0
	ret = mxt_read_current_crc(data, &current_crc);
	if (ret)
	{
		goto release;
	}
#endif /* 0 */

	if (strncmp(cfg->data, MXT_CFG_MAGIC, strlen(MXT_CFG_MAGIC)))
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Wrong magic number.\n", dev_name(dev),__func__);
		ret = -EINVAL;
		goto release;
	}
	pos = strlen(MXT_CFG_MAGIC);

	/* Load information block and check */
	memcpy(&cfg_info, cfg->data + pos, sizeof(cfg_info));
	pos += sizeof(cfg_info);
	DBG_PRINT_HEX(dbg_level_debug, MXT_TAG ": mxt_download_bin_config(): info: ", DUMP_PREFIX_NONE, 16, 1, &cfg_info, sizeof(cfg_info), false);
	if (cfg_info.family_id != data->info.family_id)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Family ID mismatch: expected=%d given=%d.\n", dev_name(dev),__func__, data->info.family_id, cfg_info.family_id);
		ret = -EINVAL;
		goto release;
	}

	if (cfg_info.variant_id != data->info.variant_id)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Variant ID mismatch: expected=%d given=%d.\n", dev_name(dev),__func__, data->info.variant_id, cfg_info.variant_id);
		ret = -EINVAL;
		goto release;
	}

	if (cfg_info.version != data->info.version)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Version mismatch: expected=%d given=%d ... still proceeding with the update.\n", dev_name(dev),__func__, data->info.version, cfg_info.version);
	}

	if (cfg_info.build != data->info.build)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Build mismatch: expected=%d given=%d ... still proceeding with the update.\n", dev_name(dev),__func__, data->info.build, cfg_info.build);
	}

	memcpy(&info_crc, cfg->data + pos, sizeof(info_crc));
	pos += sizeof(info_crc);
	memcpy(&config_crc, cfg->data + pos, sizeof(config_crc));
	pos += sizeof(config_crc);

	while (pos < cfg->size)
	{
		/* Read type, instance, length */
		memcpy(&type, cfg->data + pos, sizeof(type));
		pos += sizeof(type);
		memcpy(&instance, cfg->data + pos, sizeof(instance));
		pos += sizeof(instance);
		memcpy(&size, cfg->data + pos, sizeof(size));
		pos += sizeof(size);
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: type=%03d, instance=%02d, size=0x%08x.\n", dev_name(dev),__func__, type, instance, size);

		object = mxt_get_object(data, type);
		if (!object)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Object T%03d not present...skipping.\n", dev_name(dev),__func__, type);
			pos += size * sizeof(val);
			ret = -EINVAL;
			continue;
		}

		if (size > object->size)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Size overflow for T%03d: actual_size=%d, cfg_size=%d.\n", dev_name(dev),__func__, type, object->size, size);
			pos += size * sizeof(val);
			ret = -EINVAL;
			continue;
		}

		if (instance >= object->instances)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Instance overflow for T%03d: actual_instances=%d, cfg_instance=%d.\n", dev_name(dev),__func__, type, object->instances, instance);
			pos += size * sizeof(val);
			ret = -EINVAL;
			continue;
		}

		reg = object->start_address + object->size * instance;
		for (i = 0; i < size; i++)
		{
			memcpy(&val, cfg->data + pos, sizeof(val));
			pos += sizeof(val);

			ret = mxt_write_reg(data->client, reg + i, val);
			if (ret)
			{
				goto release;
			}
		}

		/* If firmware is upgraded, new bytes may be added to end of
		 * objects. It is generally forward compatible to zero these
		 * bytes - previous behaviour will be retained. However
		 * this does invalidate the CRC and will force a config
		 * download every time until the configuration is updated */
		if (size < object->size)
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Zeroing %d byte(s) in T%03d.\n", dev_name(dev),__func__, object->size - size, type);
			for (i = size + 1; i < object->size; i++)
			{
				ret = mxt_write_reg(data->client, reg + i, 0);
				if (ret)
				{
					goto release;
				}
			}
		}
	}

release:
	release_firmware(cfg);
err_return:
	/* Restore the do_not_suspend_mxt */
	atomic_cmpxchg(&data->do_not_suspend_mxt, !suspend_enabled, suspend_enabled); /* After this do_not_suspend_mxt will be suspend_enabled */
	return ret;
}

static int mxt_bkup_nv(struct mxt_data *data)
{
	int ret;
	int timeout_counter = 1000;
	u8 command_register;

	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_BACKUPNV, MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);
	do
	{
		ret =  mxt_read_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_BACKUPNV, &command_register);
		if (ret)
		{
			return ret;
		}
		msleep(10);
	} while ((command_register != 0) && (timeout_counter-- > 0));
	if (timeout_counter <= 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: No response after backup.\n", dev_name(&data->client->dev),__func__);
		return -EIO;
	}
	return 0;
}

static int mxt_soft_reset(struct mxt_data *data, u8 value)
{
	int timeout_counter = 0;
	struct device *dev = &data->client->dev;
	int n_retries;

	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Resetting chip to 0x%02x.\n", dev_name(dev),__func__, value);

	disable_irq(data->irq);

	mxt_write_object(data, MXT_GEN_COMMAND_T6, MXT_COMMAND_RESET, value);

	if (data->pdata->read_chg == NULL)
	{
		msleep(MXT_RESET_NOCHGREAD);
	}
	else
	{
		switch (data->info.family_id)
		{
		case MXT224_ID:
			n_retries = (3*MXT224_RESET_TIME)/10;
			break;
		case MXT768E_ID:
			n_retries = (3*MXT768E_RESET_TIME)/10;
			break;
		case MXT1386_ID:
			n_retries = (3*MXT1386_RESET_TIME)/10;
			break;
		case MXT1188S_ID:
			n_retries = (3*MXT1188S_RESET_TIME)/10;
			break;
		default:
			n_retries = (3*MXT_RESET_TIME)/10;
			break;
		}
		timeout_counter = 0;
		while ((timeout_counter++ <= n_retries) && data->pdata->read_chg())
		{
			msleep(10);
		}
		if (timeout_counter > n_retries)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: No response after reset.\n", dev_name(dev),__func__);
			enable_irq(data->irq);
			return -EIO;
		}
	}
	enable_irq(data->irq);
	return 0;
}

static int mxt_set_power_cfg(struct mxt_data *data, u8 mode)
{
	struct device *dev = &data->client->dev;
	int error;
	u8 actv_cycle_time;
	u8 idle_cycle_time;

	if (atomic_read(&data->mxt_state) != APPMODE)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Not in app mode.\n", dev_name(dev),__func__);
		return -EINVAL;
	}

	switch (mode)
	{
	case MXT_POWER_CFG_DEEPSLEEP:
		actv_cycle_time = 0;
		idle_cycle_time = 0;
		break;
	case MXT_POWER_CFG_RUN:
	default:
		actv_cycle_time = data->actv_cycle_time;
		idle_cycle_time = data->idle_cycle_time;
	}

	error = mxt_write_object(data, MXT_GEN_POWER_T7, MXT_POWER_ACTVACQINT, actv_cycle_time);
	if (error)
	{
		goto i2c_error;
	}

	error = mxt_write_object(data, MXT_GEN_POWER_T7, MXT_POWER_IDLEACQINT, idle_cycle_time);
	if (error)
	{
		goto i2c_error;
	}

	dev_dbg(dev, MXT_TAG ": %s(): Set ACTV %d, IDLE %d", __func__, actv_cycle_time, idle_cycle_time);

	if((actv_cycle_time == 0) || (idle_cycle_time == 0))
	{
		atomic_set(&data->mxt_power_mode, MXT_POWER_MODE_DEEP_SLEEP);
	}
	else
	{
		atomic_set(&data->mxt_power_mode, MXT_POWER_MODE_ACTIVE);
	}

//	atomic_set(&data->mxt_power_mode, (mode == MXT_POWER_CFG_DEEPSLEEP) ? MXT_POWER_MODE_DEEP_SLEEP : MXT_POWER_MODE_ACTIVE);

	return 0;

i2c_error:
	DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to set power cfg.\n", dev_name(dev),__func__);
	return error;
}

static int mxt_read_power_cfg(struct mxt_data *data, u8 *actv_cycle_time, u8 *idle_cycle_time)
{
	struct device *dev = &data->client->dev;
	int error;

	error = mxt_read_object(data, MXT_GEN_POWER_T7, MXT_POWER_ACTVACQINT, actv_cycle_time);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read active cycle time.\n", dev_name(dev),__func__);
		return error;
	}

	error = mxt_read_object(data, MXT_GEN_POWER_T7,	MXT_POWER_IDLEACQINT, idle_cycle_time);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read idle cycle time.\n", dev_name(dev),__func__);
		return error;
	}

	if((actv_cycle_time == 0) || (idle_cycle_time == 0))
	{
		atomic_set(&data->mxt_power_mode, MXT_POWER_MODE_DEEP_SLEEP);
	}
	else
	{
		atomic_set(&data->mxt_power_mode, MXT_POWER_MODE_ACTIVE);
	}


	return 0;
}

static int mxt_check_power_cfg_post_reset(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	error = mxt_read_power_cfg(data, &data->actv_cycle_time, &data->idle_cycle_time);
	if (error)
	{
		return error;
	}

	/* If the Power config is zero, select free run */
	if((data->actv_cycle_time == 0) || (data->idle_cycle_time == 0))
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Overriding power cfg to free run.\n", dev_name(dev),__func__);
		data->actv_cycle_time = 255;
		data->idle_cycle_time = 255;
		error = mxt_set_power_cfg(data, MXT_POWER_CFG_RUN);
		if (error)
		{
			return error;
		}
	}
//	atomic_set(&data->mxt_power_mode, MXT_POWER_MODE_ACTIVE);
	return 0;
}

static int mxt_probe_power_cfg(struct mxt_data *data)
{
	int error;

	error = mxt_read_power_cfg(data, &data->actv_cycle_time, &data->idle_cycle_time);
	if (error)
	{
		return error;
	}

	/* If the controller is in deep sleep mode, attempt reset */
	if((data->actv_cycle_time == 0) || (data->idle_cycle_time == 0))
	{
		error = mxt_soft_reset(data, MXT_RESET_VALUE);
		if (error)
		{
			return error;
		}

		error = mxt_check_power_cfg_post_reset(data);
		if (error)
		{
			return error;
		}
	}
//	else
//	{
//		atomic_set(&data->mxt_power_mode, MXT_POWER_MODE_ACTIVE);
//	}
	return 0;
}

static int mxt_check_reg_init(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	int ret;
#if 0
	/*
	 * Ideally the config should be stored in the controller NV during FW update.
	 * In case there is no cfg info, use the parameters from platform data as a fall back.
	 */
	#define CFG_ITEM_SIZE 16
	#define CFG_ITEM_MAX 2
	u8 usr_data[CFG_ITEM_SIZE];
	int cfg_data_not_found = -1;
	int iLoop;
	for(iLoop=0; iLoop < CFG_ITEM_MAX; iLoop++)
	{
		ret= mxt_read_object_block(data, MXT_SPT_USERDATA_T38, iLoop * CFG_ITEM_SIZE, CFG_ITEM_SIZE, usr_data);
		if(ret == 0)
		{
			switch(iLoop)
			{
			case 0:
				if(strncmp("ATMEL_MXT", usr_data, strlen("ATMEL_MXT")) != 0)
				{
					cfg_data_not_found = 0;
				}
				break;
			case 1:
				if(strncmp("CFG_VERSION", usr_data, strlen("CFG_VERSION")) != 0)
				{
					cfg_data_not_found = 1;
				}
				break;
			default:
				break;
			}
		}
		else
		{
			/* Just indicate that the data is not found */
			cfg_data_not_found = 0;
			break;
		}
		if(cfg_data_not_found >=0)
		{
			dev_dbg(dev, MXT_TAG ": %s(): Using values from platfrom data\n", __func__);
			mxt_handle_pdata(data);
			break;
		}
	}

//	ret = mxt_download_config(data, MXT_CFG_NAME);
//	if (ret < 0)
//	{
//		return ret;
//	}
//	else if (ret == 0)
//	{
//		/* CRC matched, or no config file, no need to reset */
//		return 0;
//	}
//	mxt_bkup_nv(data);
//	ret = mxt_soft_reset(data, MXT_RESET_VALUE);
//	if (ret)
//	{
//		return ret;
//	}
#endif
	ret = mxt_probe_power_cfg(data);
	if (ret)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to initialize power cfg.\n", dev_name(dev),__func__);
		return ret;
	}
	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Power cfg: active_cycle=0x%02x, idle_cycle=0x%02x.\n", dev_name(dev),__func__, data->actv_cycle_time, data->idle_cycle_time);

	mxt_configure_volatile_settings(data);

	return 0;
}

static int mxt_configure_volatile_settings(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;

	//if(pdata->use_fw_gestures)
	//{
	//	mxt_gestures_init(data);
	//}

	return 0;
}

static void mxt_handle_pdata(struct mxt_data *data)
{
	const struct mxt_platform_data *pdata = data->pdata;
	u8 ctrl;

	/* Set touchscreen lines */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XSIZE, pdata->x_line);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YSIZE, pdata->y_line);

	/* Set touchscreen orient */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_ORIENT, pdata->orient);

	/* Set touchscreen burst length */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_BLEN, pdata->blen);

	/* Set touchscreen threshold */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_TCHTHR, pdata->threshold);

	/* Set touchscreen resolution */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XRANGE_LSB, (pdata->x_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XRANGE_MSB, (pdata->x_size - 1) >> 8);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YRANGE_LSB, (pdata->y_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YRANGE_MSB, (pdata->y_size - 1) >> 8);

	/* Disable touch vector change calculation */
	if(0 != mxt_read_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, &ctrl))
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: reading CTRL failed.\n", dev_name(&data->client->dev),__func__);
	}
	else
	{
		ctrl &=  ~(MXT_TOUCH_CTRL_DISVECT);
		mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, ctrl);
	}
}

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;

	error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
	if (error)
	{
		return error;
	}
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	if (error)
	{
		return error;
	}
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	if (error)
	{
		return error;
	}
	info->version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	if (error)
	{
		return error;
	}
	info->build = val;

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	if (error)
	{
		return error;
	}
	info->object_num = val;

	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];
	u16 obj_size = MXT_OBJECT_SIZE;

	for (i = 0; i < data->info.object_num; i++)
	{
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, obj_size, buf);
		if (error)
		{
			return error;
		}

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		object->size = buf[3] + 1;
		object->instances = buf[4] + 1;
		object->num_report_ids = buf[5];

		if (object->num_report_ids)
		{
			reportid += object->num_report_ids * object->instances;
			object->max_reportid = reportid;
			object->min_reportid = object->max_reportid - object->instances * object->num_report_ids + 1;
		}

		dev_dbg(&(data->client->dev), MXT_TAG ": %s(): T%03d,\tstart = %03d,\tsize = %03d,\tinstances = %02d,\tmin_reportid = %03d,\tmax_reportid = %03d\n",
			__func__, object->type, object->start_address, object->size,
			object->instances, object->min_reportid, object->max_reportid);
	}

	/* Store maximum reportid */
	data->max_reportid = reportid;

	return 0;
}

static int mxt_read_resolution(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	int error;
	unsigned int x_range, y_range;
	unsigned int max_x, max_y;
	unsigned char orient;
	unsigned char val;

	/* Update matrix size in info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
	{
		return error;
	}
	data->info.matrix_xsize = val;

	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
	{
		return error;
	}
	data->info.matrix_ysize = val;

	/* Read X/Y size of touchscreen */
	error =  mxt_read_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XRANGE_MSB, &val);
	if (error)
	{
		return error;
	}
	x_range = val << 8;

	error =  mxt_read_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XRANGE_LSB, &val);
	if (error)
	{
		return error;
	}
	x_range |= val;

	error =  mxt_read_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YRANGE_MSB, &val);
	if (error)
	{
		return error;
	}
	y_range = val << 8;

	error =  mxt_read_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YRANGE_LSB, &val);
	if (error)
	{
		return error;
	}
	y_range |= val;

	error =  mxt_read_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_ORIENT, &orient);
	if (error)
	{
		return error;
	}

	/* Handle default values */
	if (x_range == 0)
	{
		x_range = 1023;
	}
	if (y_range == 0)
	{
		y_range = 1023;
	}

	max_x = x_range + 1;
	max_y = y_range + 1;

	if (orient & MXT_XY_SWITCH)
	{
		data->max_x = max_y;
		data->max_y = max_x;
	}
	else
	{
		data->max_x = max_x;
		data->max_y = max_y;
	}

	return 0;
}

static int mxt_init_mt_slots(struct mxt_data *data, int use_slots)
{
	struct input_dev *input_dev = data->input_dev;
	struct i2c_client *client = data->client;
	struct device *dev = &client->dev;
	int ret;

	switch(use_slots)
	{
	case MXT_MT_NONE:
		/* Check if slots are already initialized. Destroy the slots if we are using them. */
		if(atomic_read(&data->uses_mt_slots) == MXT_MT_B)
		{
			input_mt_destroy_slots(input_dev);
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Destroyed slots.\n", dev_name(dev),__func__);
		}
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Using only ST protocol.\n", dev_name(dev),__func__);
		atomic_set(&data->uses_mt_slots, MXT_MT_NONE);
		break;
	case MXT_MT_A:
		/* Check if slots are already initialized. Destroy the slots if we are using them. */
		if(atomic_read(&data->uses_mt_slots) == MXT_MT_B)
		{
			input_mt_destroy_slots(input_dev);
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Destroyed slots.\n", dev_name(dev),__func__);
		}
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Using type A protocol.\n", dev_name(dev),__func__);
		atomic_set(&data->uses_mt_slots, MXT_MT_A);
		break;
	case MXT_MT_B:
		/* Check if slots are not initialized. Initialize them if we dont have them. */
		if(atomic_read(&data->uses_mt_slots) != MXT_MT_B)
		{
			ret = input_mt_init_slots(input_dev, MXT_MAX_FINGER);
			if(ret)
			{
				DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Failed to initialize slots; using type A protocol.\n", dev_name(dev),__func__);
				atomic_set(&data->uses_mt_slots, MXT_MT_A);
			}
			else
			{
				DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Successfully initialized slots; using type B protocol.\n", dev_name(dev),__func__);
				atomic_set(&data->uses_mt_slots, MXT_MT_B);
			}
		}
		else
		{
			/* Nothing to do if slots are already initialized. */
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Already using type B protocol.\n", dev_name(dev),__func__);
		}
		break;
	}
	return atomic_read(&data->uses_mt_slots);
}

static int mxt_initialize(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct device *dev = &data->client->dev;
	struct mxt_info *info = &data->info;
	int error;

	error = mxt_get_info(data);
	if (error)
	{
		/* Try bootloader mode */
		error = mxt_switch_to_bootloader_address(data);
		if (error)
		{
			return error;
		}

		error = mxt_check_bootloader(client, MXT_APP_CRC_FAIL);
		if (error)
		{
			return error;
		}
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Application crc failure.\n", dev_name(dev),__func__);
		atomic_set(&data->mxt_state, BOOTLOADER);

		return 0;
	}
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Family ID = %d, Variant ID = %d, Version = %d.%d, Build = 0x%02X, Object Num = %d\n",
			dev_name(dev), __func__,info->family_id, info->variant_id, info->version >> 4, info->version & 0xf, info->build, info->object_num);

	atomic_set(&data->mxt_state, APPMODE);

	data->object_table = kcalloc(info->object_num, sizeof(struct mxt_object), GFP_KERNEL);
	if (!data->object_table)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to allocate memory.\n", dev_name(dev),__func__);
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read object table.\n", dev_name(dev),__func__);
		goto err_get_object_table;
	}

	error = mxt_check_message_length(data);
	if (error)
	{
		goto err_check_message;
	}

	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to initialize configuration.\n", dev_name(dev),__func__);
		goto err_check_reg_init;
	}

	error = mxt_read_resolution(data);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to initialize resolution.\n", dev_name(dev), __func__);
		goto err_read_resolution;
	}

	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Matrix Size X%dY%d, Touchscreen Size X%dY%d\n",
			dev_name(dev), __func__, data->info.matrix_xsize, data->info.matrix_ysize, data->max_x, data->max_y);

	return 0;

err_read_resolution:
err_check_reg_init:
err_check_message:
err_get_object_table:
	kfree(data->object_table);
	data->object_table = NULL;

	return error;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	unsigned int retry = 0;
	int ret;

	ret = request_firmware(&fw, fn, dev);
	if (ret < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Unable to open FW %s.\n", dev_name(dev), __func__, fn);
		return ret;
	}

	if (atomic_read(&data->mxt_state) != BOOTLOADER)
	{
		/* Change to the bootloader mode */
		ret = mxt_soft_reset(data, MXT_BOOT_VALUE);
		if (ret)
		{
			goto release_firmware;
		}
		ret = mxt_switch_to_bootloader_address(data);
		if (ret)
		{
			goto release_firmware;
		}
	}

	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
	{
		/* Bootloader may still be unlocked from previous update
		 * attempt */
		ret = mxt_check_bootloader(client, MXT_WAITING_FRAME_DATA);
		if (ret)
		{
			goto return_to_app_mode;
		}
	}
	else
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Unlocking bootloader.\n", dev_name(dev), __func__);
		/* Unlock bootloader */
		mxt_unlock_bootloader(client);
	}

	while (pos < fw->size)
	{
		ret = mxt_check_bootloader(client, MXT_WAITING_FRAME_DATA);
		if (ret)
		{
			goto release_firmware;
		}
		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);

		ret = mxt_check_bootloader(client, MXT_FRAME_CRC_PASS);
		if (ret)
		{
			retry++;

			/* Back off by 20ms per retry */
			msleep(retry * 20);
			if (retry > 20)
			{
				goto release_firmware;
			}
		}
		else
		{
			retry = 0;
			pos += frame_size;
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Updated %d/%zd bytes.\n", dev_name(dev), __func__, pos, fw->size);
		}
	}

return_to_app_mode:
	mxt_switch_to_appmode_address(data);
release_firmware:
	release_firmware(fw);

	return ret;
}

static ssize_t mxt_update_fw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int update_fw = 0;
	int error;

	if (sscanf(buf, "%u", &update_fw) > 0)
	{
		if(update_fw > 0)
		{
			if(atomic_read(&data->mxt_irq_enabled) != 0)
			{
				disable_irq(data->irq);
				atomic_set(&data->mxt_irq_enabled, 0);
			}

			error = mxt_load_fw(dev, MXT_FW_NAME);
			if (error)
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: The FW update failed(%d).\n", dev_name(dev), __func__, error);
				count = error;
			}
			else
			{
				DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: The FW update succeeded.\n", dev_name(dev), __func__);

				/* Wait for reset */
				msleep(MXT_FWRESET_TIME);
				atomic_set(&data->mxt_state, INIT);
				kfree(data->object_table);
				data->object_table = NULL;
				mxt_initialize(data);
			}

			if (atomic_read(&data->mxt_state) == APPMODE)
			{
				if(atomic_read(&data->mxt_irq_enabled) == 0)
				{
					enable_irq(data->irq);
					atomic_set(&data->mxt_irq_enabled, 1);
				}

				error = mxt_make_highchg(data);
				if (error)
				{
					count = error;
				}
			}

			return count;
		}
	}
	return 0;
}
static DEVICE_ATTR(update_fw, 0220, NULL, mxt_update_fw_store);

static ssize_t mxt_update_txt_cfg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret;
	char mxt_txt_cfg_name[256];

	// buf contains the newline-terminated file name - remove the newline.
	strncpy(mxt_txt_cfg_name, buf, count - 1);
	mxt_txt_cfg_name[count - 1] = '\0';
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Processing input file %s.\n", dev_name(dev), __func__, mxt_txt_cfg_name);

	ret = mxt_download_txt_config(data, mxt_txt_cfg_name);
	if (ret < 0)
	{
		return count;
	}
	else if (ret == 0)
	{
		/* CRC matched, or no config file, no need to reset */
		return count;
	}
	mxt_bkup_nv(data);
	ret = mxt_soft_reset(data, MXT_RESET_VALUE);
	if (ret)
	{
		return count;
	}
	ret = mxt_check_power_cfg_post_reset(data);
	if (ret)
	{
		return count;
	}

	return count;
}
static DEVICE_ATTR(update_txt_cfg, 0220, NULL, mxt_update_txt_cfg_store);

static ssize_t mxt_update_bin_cfg_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int update_cfg = 0;
	int ret;
	char mxt_bin_cfg_name[256];

	// buf contains the newline-terminated file name - remove the newline.
	strncpy(mxt_bin_cfg_name, buf, count - 1);
	mxt_bin_cfg_name[count - 1] = '\0';
	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Processing input file %s.\n", dev_name(dev), __func__, mxt_bin_cfg_name);

	ret = mxt_download_bin_config(data, mxt_bin_cfg_name);
	if (ret < 0)
	{
		return count;
	}
	else if (ret == 0)
	{
		/* CRC matched, or no config file, no need to reset */
		return count;
	}
	mxt_bkup_nv(data);
	ret = mxt_soft_reset(data, MXT_RESET_VALUE);
	if (ret)
	{
		return count;
	}
	ret = mxt_check_power_cfg_post_reset(data);
	if (ret)
	{
		return count;
	}

	return count;
}
static DEVICE_ATTR(update_bin_cfg, 0220, NULL, mxt_update_bin_cfg_store);

static ssize_t mxt_pause_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;

	count += sprintf(buf + count, "%d", (atomic_read(&data->mxt_power_mode) == MXT_POWER_MODE_DEEP_SLEEP) ? 1 : 0);
	count += sprintf(buf + count, "\n");

	return count;
}
static ssize_t mxt_pause_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int pause;

	if (sscanf(buf, "%u", &pause) == 1 && pause < 2)
	{
		if(pause)
		{
			mxt_stop(data);
		}
		else
		{
			mxt_start(data);
		}
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: %s.\n", dev_name(dev), __func__, pause ? "paused" : "unpaused");
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: pause_driver write error.\n", dev_name(dev), __func__);
	}
	return count;
}
static DEVICE_ATTR(pause_driver, 0666, mxt_pause_show, mxt_pause_store);

static ssize_t mxt_debug_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;

	count += sprintf(buf + count, "%d", data->debug_enabled);
	count += sprintf(buf + count, "\n");

	return count;
}
static ssize_t mxt_debug_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;

	if (sscanf(buf, "%u", &i) == 1 && i < 2)
	{
		data->debug_enabled = i;
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: debug %sabled.\n", dev_name(dev), __func__, i ? "en" : "dis");
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: debug_enabled write error.\n", dev_name(dev), __func__);
	}

	return count;
}
static DEVICE_ATTR(debug_enable, 0666, mxt_debug_enable_show, mxt_debug_enable_store);

static ssize_t mxt_mem_access_read(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (atomic_read(&data->mxt_state) != APPMODE)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Not in app mode.\n", dev_name(dev), __func__);
		return -EINVAL;
	}

	if (off >= MXT_MEMACCESS_SIZE)
	{
		return -EIO;
	}

	if (off + count > MXT_MEMACCESS_SIZE)
	{
		count = MXT_MEMACCESS_SIZE - off;
	}

	if (count > MXT_I2C_MAX_REQ_SIZE)
	{
		count = MXT_I2C_MAX_REQ_SIZE;
	}

	if (count > 0)
	{
		ret = __mxt_read_reg(data->client, off, count, buf);
	}
	return ret == 0 ? count : ret;
}

static int mxt_write_block(struct i2c_client *client, u16 addr, u16 length, u8 *value)
{
	int i;
	struct
	{
		__le16 le_addr;
		u8  data[MXT_I2C_MAX_REQ_SIZE];
	} i2c_block_transfer;

	if (length > MXT_I2C_MAX_REQ_SIZE)
	{
		return -EINVAL;
	}

	for (i = 0; i < length; i++)
	{
		i2c_block_transfer.data[i] = *value++;
	}
	i2c_block_transfer.le_addr = cpu_to_le16(addr);

	i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
	if (i == (length + 2))
	{
		return 0;
	}
	else
	{
		return -EIO;
	}
}

static ssize_t mxt_mem_access_write(struct file *filp, struct kobject *kobj, struct bin_attribute *bin_attr, char *buf, loff_t off, size_t count)
{
	struct device *dev = container_of(kobj, struct device, kobj);
	struct mxt_data *data = dev_get_drvdata(dev);
	int ret = 0;

	if (atomic_read(&data->mxt_state) != APPMODE)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Not in app mode.\n", dev_name(dev), __func__);
		return -EINVAL;
	}

	if (off >= MXT_MEMACCESS_SIZE)
	{
		return -EIO;
	}

	if (off + count > MXT_MEMACCESS_SIZE)
	{
		count = MXT_MEMACCESS_SIZE - off;
	}

	if (count > MXT_I2C_MAX_REQ_SIZE)
	{
		count = MXT_I2C_MAX_REQ_SIZE;
	}

	if (count > 0)
	{
		ret = mxt_write_block(data->client, off, count, buf);
	}

	return ret == 0 ? count : 0;
}

static ssize_t mxt_info_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int bytes_read = 0;

	bytes_read += sprintf(buf + bytes_read, "Family ID:  0x%02x\n", data->info.family_id);
	bytes_read += sprintf(buf + bytes_read, "Variant ID: 0x%02x\n", data->info.variant_id);
	bytes_read += sprintf(buf + bytes_read, "Version:    0x%02x\n", data->info.version);
	bytes_read += sprintf(buf + bytes_read, "Build:      0x%02x\n", data->info.build);
	bytes_read += sprintf(buf + bytes_read, "X Size:     0x%02x\n", data->info.matrix_xsize);
	bytes_read += sprintf(buf + bytes_read, "Y Size:     0x%02x\n", data->info.matrix_ysize);
	bytes_read += sprintf(buf + bytes_read, "Objects:    0x%02x\n", data->info.object_num);
	bytes_read += sprintf(buf + bytes_read, "\n");

	return bytes_read;
}
static DEVICE_ATTR(info, 0444, mxt_info_show, NULL);

static ssize_t mxt_object_table_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int bytes_read = 0;
	int iLoop;

	if(data->object_table != 0)
	{
		for (iLoop = 0; iLoop < data->info.object_num; iLoop++)
		{
			struct mxt_object *object = data->object_table + iLoop;
			bytes_read += sprintf(buf + bytes_read,
					"T%03d,\tstart = %03d,\tsize = %03d,\tinstances = %02d,\tmin_reportid = %03d,\tmax_reportid = %03d\n",
					object->type, object->start_address, object->size, object->instances, object->min_reportid, object->max_reportid);
		}
	}
	return bytes_read;
}
static DEVICE_ATTR(object_table, 0444, mxt_object_table_show, NULL);

static ssize_t mxt_debug_trigger_show(struct device *dev, struct device_attribute *attr, char *buf)
{
//	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;

	count += sprintf(buf + count,   "0 - suspend");
	count += sprintf(buf + count, "\n1 - resume");
	count += sprintf(buf + count, "\n2 - download bin cfg");
	count += sprintf(buf + count, "\n3 - download txt cfg");
	count += sprintf(buf + count, "\n4 - backup cfg");
	count += sprintf(buf + count, "\n5 - use type A protocol");
	count += sprintf(buf + count, "\n6 - use type B protocol");
	count += sprintf(buf + count, "\n7 - do not use MT protocol");
	count += sprintf(buf + count, "\n");

	return count;
}
static ssize_t mxt_debug_trigger_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int i;
	int ret;

	if (sscanf(buf, "%u", &i) == 1)
	{
		switch(i)
		{
		case 0:
			mxt_suspend(dev);
			break;
		case 1:
			mxt_resume(dev);
			break;
		case 2:
			ret = mxt_download_bin_config(data, MXT_BIN_CFG_NAME);
			if (ret < 0)
			{
				/* Error */
			}
			else if (ret == 0)
			{
				/* CRC matched, or no config file, no need to reset */
			}
			break;
		case 3:
			ret = mxt_download_txt_config(data, MXT_TXT_CFG_NAME);
			if (ret < 0)
			{
				/* Error */
			}
			else if (ret == 0)
			{
				/* CRC matched, or no config file, no need to reset */
			}
			break;
		case 4:
			/* Backup to memory */
			mxt_bkup_nv(data);
			break;
		case 5:
			/* Use type A protocol */
			mxt_init_mt_slots(data, MXT_MT_A);
			break;
		case 6:
			/* Use type B protocol */
			mxt_init_mt_slots(data, MXT_MT_B);
			break;
		case 7:
			/* Do not use MT protocol */
			mxt_init_mt_slots(data, MXT_MT_NONE);
			break;
		}
	}
	else
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: debug_trigger write error.\n", dev_name(dev), __func__);
	}

	return count;
}
static DEVICE_ATTR(debug_trigger, 0664, mxt_debug_trigger_show, mxt_debug_trigger_store);

static ssize_t mxt_driver_buildid_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int bytes_read = 0;

	bytes_read += sprintf(buf + bytes_read, "%s:%s\n", __DATE__, __TIME__);

	return bytes_read;
}
static DEVICE_ATTR(build_id, 0444, mxt_driver_buildid_show, NULL);

static ssize_t mxt_userdata_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *userdata_object;
	int count = 0;
	int retval;
	int iLoop;
	#define USERDATA_VERSION_INFO_OFFSET 0x00
	#define USERDATA_VERSION_INFO_SIZE 0x10
	u8 usr_data[USERDATA_VERSION_INFO_SIZE];

	userdata_object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!userdata_object)
	{
		return 0; /* EOF */
	}

	retval = mxt_read_object_block(data, MXT_SPT_USERDATA_T38, USERDATA_VERSION_INFO_OFFSET, USERDATA_VERSION_INFO_SIZE, usr_data);
	if(retval == 0)
	{
		for(iLoop = 0; iLoop < USERDATA_VERSION_INFO_SIZE; iLoop++)
		{
			count += sprintf(buf + count, "%02x", usr_data[iLoop]);
		}
		count += sprintf(buf + count, "\n");
	}

	return count;
}
static DEVICE_ATTR(userdata, 0444, mxt_userdata_show, NULL);

static ssize_t mxt_config_crc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	int count = 0;
	unsigned long current_crc;
	int retval;

	retval = mxt_read_current_crc(data, &current_crc);
	if (retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to read current crc\n", dev_name(dev),__func__);
		return 0; /* EOF */
	}
	count += sprintf(buf + count, "0x%06x", current_crc & 0x00FFFFFF);
	count += sprintf(buf + count, "\n");

	return count;
}
static DEVICE_ATTR(config_crc, 0444, mxt_config_crc_show, NULL);

static struct attribute *mxt_attrs[] = {
	&dev_attr_update_fw.attr,
	&dev_attr_update_txt_cfg.attr,
	&dev_attr_update_bin_cfg.attr,
	&dev_attr_pause_driver.attr,
	&dev_attr_debug_enable.attr,
	&dev_attr_info.attr,
	&dev_attr_object_table.attr,
	&dev_attr_debug_trigger.attr,
	&dev_attr_build_id.attr,
	&dev_attr_userdata.attr,
	&dev_attr_config_crc.attr,
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

#if defined(CONFIG_DEBUG_FS)
static int mxt_dbgfs_open(struct inode *inode, struct file *file)
{
//	struct mxt_object *object = inode->i_private;
	file->private_data = inode->i_private;
	return 0;
};
static int mxt_dbgfs_release(struct inode *inode, struct file *file)
{
	return 0;
};

static ssize_t mxt_dbgfs_object_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	struct mxt_object *object = file->private_data;
	char *obj_buf;
	int ret;
	unsigned int iLoop;
	unsigned int bytes_written;

	obj_buf = kzalloc(count+1, GFP_KERNEL);
	if(obj_buf == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: No memory for user buffer.\n", dev_name(&object->priv_data->client->dev), __func__);
		ret = -ENOMEM;
		goto err_alloc_obj_buf;
	}

	if(copy_from_user(obj_buf, buf, count))
	{
		ret = -EFAULT;
		goto err_copy_from_user;
	}
	bytes_written = 0;
	for(iLoop=0; iLoop < count-1; iLoop+=2)
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Writing addr=%u, val=%u to T[%03d]\n",
				dev_name(&object->priv_data->client->dev), __func__, obj_buf[iLoop], obj_buf[iLoop+1], object->type);
		ret = mxt_write_object(object->priv_data, object->type, obj_buf[iLoop], obj_buf[iLoop+1]);
		if(ret)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Error in writing to T%03d.\n", dev_name(&object->priv_data->client->dev), __func__, object->type);
			continue;
		}
		bytes_written += 2;
	}
	ret = bytes_written;

err_copy_from_user:
	kfree(obj_buf);
err_alloc_obj_buf:
	return ret;

}

static ssize_t mxt_dbgfs_object_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
	struct mxt_object *object = file->private_data;
	char *obj_buf, *usr_buf;
	int bytes_to_read;
	int iLoop;
	int ret;

	if (*offset >= object->size)
	{
		return 0;
	}

	if(((object->size - *offset)*12 + 1 ) < count)
	{
		bytes_to_read = object->size - *offset;
	}
	else
	{
		bytes_to_read = (count-1)/12;
	}
	DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Reading %d bytes from T%03d.\n", dev_name(&object->priv_data->client->dev), __func__, bytes_to_read, object->type);

	obj_buf = kzalloc(bytes_to_read, GFP_KERNEL);
	if(obj_buf == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: No memory for object buffer.\n", dev_name(&object->priv_data->client->dev), __func__);
		ret = -ENOMEM;
		goto err_alloc_obj_buf;
	}
	usr_buf = kzalloc(bytes_to_read * 12, GFP_KERNEL); /* For each value the format is "0x00 - 0x00\n" */
	if(usr_buf == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: No memory for user buffer.\n", dev_name(&object->priv_data->client->dev), __func__);
		ret = -ENOMEM;
		goto err_alloc_usr_buf;
	}

	for(iLoop = *offset; iLoop < bytes_to_read; iLoop++)
	{
		ret = mxt_read_object(object->priv_data, object->type, iLoop, obj_buf + iLoop);
		if(ret)
		{
			ret = -EIO;
			break;
		}
		sprintf(usr_buf + iLoop*12, "0x%02x - 0x%02x\n", iLoop, obj_buf[iLoop]);
	}
	usr_buf[iLoop*12] = '\0';

	if(copy_to_user(buf, usr_buf, iLoop*12 + 1))
	{
		ret = -EFAULT;
		goto err_copy_to_user;
	}
	*offset += iLoop;
	ret =  iLoop*12 + 1;

err_copy_to_user:
	kfree(usr_buf);
err_alloc_usr_buf:
	kfree(obj_buf);
err_alloc_obj_buf:

	return ret;
}

static struct file_operations mxt_dbg_object_ops = {
	.owner   = THIS_MODULE,
	.open    = mxt_dbgfs_open,
	.read    = mxt_dbgfs_object_read,
	.write   = mxt_dbgfs_object_write,
//	.llseek  = mxt_dbgfs_lseek,
	.release = mxt_dbgfs_release
};

static int mxt_dbgfs_create(struct mxt_data *data)
{
	int iLoop;
	int ret;
	char obj_name[6];
	int n_files;

	data->dbgfs_root = debugfs_create_dir("atmel_mxt", NULL);
	if(data->dbgfs_root == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Error in creating debugfs root.\n", dev_name(&data->client->dev), __func__);
		ret = -ENOMEM;
		goto err_create_root;
	}

	n_files = data->info.object_num + 1 /* Info */;
	data->dbgfs_files = kzalloc(n_files * sizeof(struct dentry *), GFP_KERNEL);
	if(data->dbgfs_files == NULL)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: No memory for dentries.\n", dev_name(&data->client->dev), __func__);
		ret = -ENOMEM;
		goto err_alloc_files;
	}

	data->dbgfs_n_entries = 0;
	for(iLoop = 0; iLoop < data->info.object_num; iLoop++)
	{
		sprintf(obj_name,"T_%03d",data->object_table[iLoop].type);
		data->object_table[iLoop].priv_data = data;
		data->dbgfs_files[iLoop] = debugfs_create_file(obj_name, 0644, data->dbgfs_root, data->object_table + iLoop, &mxt_dbg_object_ops);
		if(data->dbgfs_files[iLoop] == NULL)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Error in creating debugfs file %s.\n", dev_name(&data->client->dev), __func__, obj_name);
			continue;
		}
		data->dbgfs_n_entries++;
	}

	return data->dbgfs_n_entries;

err_alloc_files:
	debugfs_remove(data->dbgfs_root);
err_create_root:
	return ret;
}

static int mxt_dbgfs_destroy(struct mxt_data *data)
{
	int iLoop;

	for(iLoop=0; iLoop < data->dbgfs_n_entries; iLoop++)
	{
		if(data->dbgfs_files[iLoop] != NULL)
		{
			debugfs_remove(data->dbgfs_files[iLoop]);
		}
	}
	debugfs_remove(data->dbgfs_root);
	return 0;
}
#endif /* CONFIG_DEBUG_FS */


static void mxt_start(struct mxt_data *data)
{
	int error;

	//if (atomic_read(&data->mxt_power_mode) == MXT_POWER_MODE_ACTIVE)
	//{
	//	return;
	//}

	if((data->actv_cycle_time != 255) || (data->idle_cycle_time != 255))
        {

		error = mxt_set_power_cfg(data, MXT_POWER_CFG_RUN);
		if(error)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Could not start MXT.\n", dev_name(&data->client->dev), __func__);
		}
	}

	if(atomic_read(&data->mxt_irq_enabled) == 0)
	{
		enable_irq(data->irq);
		atomic_set(&data->mxt_irq_enabled, 1);
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT irq enabled.\n", dev_name(&data->client->dev), __func__);
	}

}

static void mxt_stop(struct mxt_data *data)
{
	int error;

//	if (atomic_read(&data->mxt_power_mode) == MXT_POWER_MODE_DEEP_SLEEP)
//	{
//		return;
//	}

	error = mxt_set_power_cfg(data, MXT_POWER_CFG_DEEPSLEEP);
	if(error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Could not stop MXT.\n", dev_name(&data->client->dev), __func__);
	}
	if(atomic_read(&data->mxt_irq_enabled) != 0)
	{
		disable_irq(data->irq);
		atomic_set(&data->mxt_irq_enabled, 0);
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: MXT irq disabled.\n", dev_name(&data->client->dev), __func__);
	}

}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);
	int error;


	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_ON)
	{
		if(data->pdata->power_on)
		{
			data->pdata->power_on(&data->client->dev);
		}
		atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_ON);
		msleep(50);
		/* Check register init values */
		error = mxt_check_reg_init(data);
		if (error)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to initialize configuration.\n", dev_name(&data->client->dev), __func__);
		}

		mxt_start(data);
	}
	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);


	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_OFF)
	{
		mxt_stop(data);
		if(data->pdata->power_off)
		{
			data->pdata->power_off(&data->client->dev);
		}
		atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_OFF);
	}
}

#if 0
static int mxt_set_charger_status(struct mxt_data *data, unsigned long nb_event)
{
	switch (nb_event)
	{
	case USB_EVENT_NONE: /* USB Disconnected */
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: otg notifier callbak: USB_EVENT_NONE.\n", dev_name(&data->client->dev), __func__);
			atomic_set(&data->mxt_charger_state, 0);
		}
		break;
	case USB_EVENT_VBUS: /* USB connected */
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: otg notifier callbak: USB_EVENT_VBUS.\n", dev_name(&data->client->dev), __func__);
			atomic_set(&data->mxt_charger_state, 1);
		}
		break;
	case USB_EVENT_ID:
		{
			//DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: otg notifier callbak: USB_EVENT_ID.\n", dev_name(&data->client->dev), __func__);
			atomic_set(&data->mxt_charger_state, 2);
		}
		break;
	case USB_EVENT_CHARGER: /* Wall Charger */
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: otg notifier callbak: USB_EVENT_CHARGER.\n", dev_name(&data->client->dev), __func__);
			atomic_set(&data->mxt_charger_state, 3);
		}
		break;
	case USB_EVENT_ENUMERATED:
		{
			//DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: otg notifier callbak: USB_EVENT_ENUMERATED.\n", dev_name(&data->client->dev), __func__);
			atomic_set(&data->mxt_charger_state, 4);
		}
		break;
	default:
		{
			DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: otg notifier callbak: DEFAULT.\n", dev_name(&data->client->dev), __func__);
		}
		break;
	}

	return 0;
}

static void mxt_process_charger(struct work_struct *charger_work)
{
	struct mxt_data *data = container_of(charger_work, struct mxt_data, mxt_charger_work);
	struct mxt_object *charger_object;
	u16 base_reg;
	u8 reg_val;
	int retval;

	if (atomic_read(&data->mxt_power_mode) == MXT_POWER_MODE_DEEP_SLEEP)
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Device inactive..nothing to do.\n", dev_name(&data->client->dev),__func__);
		retval = 0;
		goto err_return;
	}

	charger_object = mxt_get_object(data, MXT_PROCG_NOISESUPPRESSION_T62);
	if(charger_object != NULL)
	{
		base_reg = charger_object->start_address;
		if(atomic_read(&data->mxt_charger_state) != 0)
		{
			reg_val = MXT_CHARGER_CTRL_ENABLE;
		}
		else
		{
			reg_val = 0;
		}

		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Writing 0x%02x to T62 CTRL.\n", dev_name(&data->client->dev), __func__, reg_val);
		retval = mxt_write_reg(data->client, base_reg + MXT_CHARGER_CTRL, reg_val);
		if (retval)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: I2C write failed.\n", dev_name(&data->client->dev), __func__);
			goto err_return;
		}
	}
	else
	{
		DBG_PRINT(dbg_level_warning, "%s: " MXT_TAG ": %s(): WARN: T62 not found.\n", dev_name(&data->client->dev), __func__);
		retval = -ENODEV;
	}

err_return:
	return;
}

static int mxt_otg_notifier_call(struct notifier_block *nb, unsigned long nb_event, void *nb_data)
{
	struct mxt_data *data = container_of(nb, struct mxt_data, otg_nb);

	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: otg notifier callbak: %lu.\n", dev_name(&data->client->dev), __func__, nb_event);

	mxt_set_charger_status(data, nb_event);

	if(0 != queue_work(data->mxt_workqueue, &(data->mxt_charger_work)))
	{
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Charger callbacks coming in too fast.\n", dev_name(&data->client->dev),__func__);
	}

	return NOTIFY_OK;
}

static int mxt_register_charger_callback(struct mxt_data *data)
{
	int retval;
	int init_charger_status;

	/* Register notification callback. */
	data->otg_nb.notifier_call = mxt_otg_notifier_call;
	data->otg = otg_get_transceiver();
	if (!data->otg)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): Failed to get OTG transceiver\n", dev_name(&data->client->dev), __func__);
		retval = -ENODEV;
		goto error_return;
	}

	init_charger_status = twl6030_usbotg_get_status();
	mxt_set_charger_status(data, init_charger_status);

	if(0 != queue_work(data->mxt_workqueue, &(data->mxt_charger_work)))
	{
		DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Charger work already scheduled.\n", dev_name(&data->client->dev),__func__);
	}

	retval = otg_register_notifier(data->otg, &data->otg_nb);
	if(retval)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): Failed to register otg notifier %d\n", dev_name(&data->client->dev), __func__, retval);
		otg_put_transceiver(data->otg);
		goto error_return;
	}

error_return:
	return retval;
}
#endif

static int __devinit mxt_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
	struct input_dev *input_dev;
	int error;

	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Probing %s @ %s.\n", dev_name(&client->dev), __func__, id->name, dev_name(&client->dev));
	if (!pdata)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: No platform data supplied....exiting.\n", dev_name(&client->dev), __func__);
		return -EINVAL;
	}

	data = kzalloc(sizeof(struct mxt_data), GFP_KERNEL);
	if (!data)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to allocate memory for private data.\n", dev_name(&client->dev), __func__);
		error = -ENOMEM;
		goto err_data_alloc;
	}
	atomic_set(&data->mxt_state, INIT);
	atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_OFF);
	atomic_set(&data->do_not_suspend_mxt, 0);
	data->client = client;
	data->pdata = pdata;
	data->irq = client->irq;
	i2c_set_clientdata(client, data);

	mutex_init(&data->mutex_fingers);
	mutex_init(&data->mutex_onetouch_gestures);
	mutex_init(&data->mutex_twotouch_gestures);

	input_dev = input_allocate_device();
	if (!input_dev)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to allocate memory for input device.\n", dev_name(&client->dev), __func__);
		error = -ENOMEM;
		goto err_input_allocate;
	}
	data->input_dev = input_dev;

	if(data->pdata->request_resources)
	{
		error = data->pdata->request_resources(&client->dev);
		if(error != 0)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to acquire resources.\n", dev_name(&client->dev), __func__);
			goto err_request_resources;
		}
	}
	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_ON)
	{
		if(data->pdata->power_on)
		{
			error = data->pdata->power_on(&client->dev);
			if(error != 0)
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to power on the controller.\n", dev_name(&client->dev), __func__);
				goto err_power_on;
			}
		}
		atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_ON);
	}

       /*reset the IC after power on */
//	printk("reset IC ... \n");
	gpio_set_value(data->pdata->reset_gpio, 0);
	msleep(20);
	gpio_set_value(data->pdata->reset_gpio, 1);
	msleep(300);

	error = mxt_initialize(data);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to allocate memory for objects.\n", dev_name(&client->dev), __func__);
		goto err_object_initialize;
	}

//	/*
//	 * Do a soft reset before making the device available.
//	 * This resolves the issue of not getting the touch events before the first suspend/resume
//	 * (The resume code essentially does the same thing)
//	 */
//	error = mxt_soft_reset(data, MXT_RESET_VALUE);
//	if (error)
//	{
//		dev_err(&(client->dev), MXT_TAG ": %s(): Failed to reset the controller\n", __func__);
//		goto err_reset;
//	}

	/* Initialize the workqueue and the irq work */
	data->mxt_workqueue = create_workqueue("mxt_wq");
	if(data->mxt_workqueue == 0) /* Is this correct? */
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to create workqueue.\n", dev_name(&client->dev), __func__);
		goto err_create_wq;
	}
	INIT_WORK(&(data->mxt_irq_work), mxt_process_irq);

	error = request_irq(client->irq, mxt_interrupt, pdata->irqflags, client->dev.driver->name, data);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to register interrupt.\n", dev_name(&client->dev), __func__);
		goto err_request_irq;
	}
	atomic_set(&data->mxt_irq_enabled, 1);

	/* disable the controller until someone actually opens the event interface */
	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_OFF)
	{
		mxt_stop(data);
		if(data->pdata->power_off)
		{
			data->pdata->power_off(&data->client->dev);
		}
		atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_OFF);
	}

	/* Initialize the input device parameters */
	input_dev->name = "Atmel maXTouch Touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
	input_dev->open = mxt_input_open;
	input_dev->close = mxt_input_close;
	input_set_drvdata(input_dev, data);
	/* Events supported */
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	/* Buttons supported */
	set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
#if defined(MXT_SUPPORT_ST)
	input_set_abs_params(input_dev, ABS_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, data->max_y, 0, 0);
	//input_set_abs_params(input_dev, ABS_PRESSURE, 0, 255, 0, 0);
#endif /* defined(MXT_SUPPORT_ST) */
	/* For multi touch */
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, data->max_y, 0, 0);
	//input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);

	if(data->pdata->use_fw_gestures)
	{
		input_set_abs_params(input_dev, ABS_HAT1X, 0, 255, 0, 0);  // Gesture code.
		input_set_abs_params(input_dev, ABS_HAT2X, 0, 255, 0, 0);  // Gesture touch count.
		input_set_abs_params(input_dev, ABS_HAT2Y, 0, 255, 0, 0);  // Gesture occur count.
	}

	error = mxt_init_mt_slots(data, MXT_MT_B);
	error = input_register_device(input_dev);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to register input device.\n", dev_name(&client->dev), __func__);
		goto err_input_register;
	}

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to create sysfs entries.\n", dev_name(&client->dev), __func__);
		goto err_create_sysfs_group;
	}

	sysfs_bin_attr_init(&data->mem_access_attr);
	data->mem_access_attr.attr.name = "mem_access";
	data->mem_access_attr.attr.mode = S_IRUGO | S_IWUGO;
	data->mem_access_attr.read = mxt_mem_access_read;
	data->mem_access_attr.write = mxt_mem_access_write;
	data->mem_access_attr.size = MXT_MEMACCESS_SIZE;

	if (sysfs_create_bin_file(&client->dev.kobj, &data->mem_access_attr) < 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to create %s.\n", dev_name(&client->dev), __func__, data->mem_access_attr.attr.name);
		goto err_create_sysfs_bin;
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
#endif
#if defined(CONFIG_DEBUG_FS)
	mxt_dbgfs_create(data);
#endif /* defined(CONFIG_DEBUG_FS) */

#if 0
	atomic_set(&data->mxt_charger_state, 0);
	INIT_WORK(&(data->mxt_charger_work), mxt_process_charger);
	mxt_register_charger_callback(data);
#endif

	return 0;

err_create_sysfs_bin:
        sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
err_create_sysfs_group:
	input_unregister_device(input_dev);
err_input_register:
	free_irq(client->irq, data);
err_request_irq:
	destroy_workqueue(data->mxt_workqueue);
err_create_wq:
err_reset:
	kfree(data->object_table);
	data->object_table = NULL;
err_object_initialize:
	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_OFF)
	{
		if(data->pdata->power_off)
		{
			data->pdata->power_off(&data->client->dev);
		}
		atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_OFF);
	}
err_power_on:
	if(data->pdata->release_resources)
	{
		data->pdata->release_resources(&client->dev);
	}
err_request_resources:
	input_free_device(input_dev);
	input_dev = NULL;
err_input_allocate:
	mutex_destroy(&data->mutex_onetouch_gestures);
	mutex_destroy(&data->mutex_twotouch_gestures);
	mutex_destroy(&data->mutex_fingers);
	kfree(data);
	data = NULL;
	i2c_set_clientdata(client, NULL);
err_data_alloc:
	return error;
}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);
#if defined(CONFIG_DEBUG_FS)
	mxt_dbgfs_destroy(data);
#endif /* defined(CONFIG_DEBUG_FS) */
	sysfs_remove_bin_file(&client->dev.kobj, &data->mem_access_attr);
        sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	input_unregister_device(data->input_dev);
	free_irq(client->irq, data);
	destroy_workqueue(data->mxt_workqueue);
	kfree(data->object_table);
	data->object_table = NULL;


	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_OFF)
	{
		if(data->pdata->power_off)
		{
			data->pdata->power_off(&client->dev);
		}
		atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_OFF);
	}
	if(data->pdata->release_resources)
	{
		data->pdata->release_resources(&client->dev);
	}
	input_free_device(data->input_dev);
	data->input_dev = NULL;
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&data->early_suspend);
#endif
	kfree(data);
	data = NULL;
	i2c_set_clientdata(client, NULL);

	return 0;
}

#if defined(CONFIG_PM)
static int mxt_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_finger *finger = data->finger;
	int id;

	DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Suspending driver ...\n", dev_name(dev), __func__);

	if(atomic_read(&data->do_not_suspend_mxt) != 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: suspend is disabled....try again.\n", dev_name(dev), __func__);
		return -EAGAIN;
	}

	mxt_stop(data);
	if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_OFF)
	{
		if(data->pdata->power_off)
		{
			data->pdata->power_off(&data->client->dev);
		}
		atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_OFF);
	}

	for (id = 0; id < MXT_MAX_FINGER; id++)
	{
		finger[id].status = MXT_TOUCH_STATUS_RELEASE;
		//mxt_input_report(data, id);
		mxt_input_report(data);
	}

	return 0;
}

static int mxt_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct mxt_data *data = i2c_get_clientdata(client);
	struct input_dev *input_dev = data->input_dev;
	int error;


	mutex_lock(&input_dev->mutex);
	if(input_dev->users) /* Power on only if someone is using the device */
	{
		DBG_PRINT(dbg_level_info, "%s: " MXT_TAG ": %s(): INFO: Resuming driver...\n", dev_name(dev), __func__);
		if(atomic_read(&data->mxt_power_supply_state) != MXT_POWER_SUPPLY_ON)
		{
			if(data->pdata->power_on)
			{
				data->pdata->power_on(&data->client->dev);
			}
			atomic_set(&data->mxt_power_supply_state, MXT_POWER_SUPPLY_ON);
		}

		if ( data->pdata->reset_on_resume )
		{
			error = mxt_soft_reset(data, MXT_RESET_VALUE);

			if (error)
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: soft reset failed \n", dev_name(dev), __func__);
			}

			error = mxt_check_power_cfg_post_reset(data);

			if (error)
			{
				DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: check pwr cfg post reset failed \n", dev_name(dev), __func__);
			}
		}
		msleep(50);
		/* Check register init values */
		error = mxt_check_reg_init(data);
		if (error)
		{
			DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to initialize configuration.\n", dev_name(&data->client->dev), __func__);
		}
		mxt_start(data);

#if 0
		/* Schedule the charger work to update the T62 config */
		if(0 != queue_work(data->mxt_workqueue, &(data->mxt_charger_work)))
		{
			DBG_PRINT(dbg_level_debug, "%s: " MXT_TAG ": %s(): DEBUG: Charger work already scheduled.\n", dev_name(&data->client->dev),__func__);
		}
#endif
	}
	mutex_unlock(&input_dev->mutex);

	return 0;
}

#else /* !defined(CONFIG_PM) */
#define mxt_suspend(dev) (0)
#define mxt_resume(dev) (0)
#endif /* defined(CONFIG_PM) */

#if defined(CONFIG_HAS_EARLYSUSPEND)
static void mxt_early_suspend(struct early_suspend *es)
{
	struct mxt_data *mxt;
	mxt = container_of(es, struct mxt_data, early_suspend);
	if (mxt_suspend(&mxt->client->dev) != 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to suspend.\n", dev_name(&mxt->client->dev), __func__);
	}
}

static void mxt_late_resume(struct early_suspend *es)
{
	struct mxt_data *mxt;
	mxt = container_of(es, struct mxt_data, early_suspend);
	if (mxt_resume(&mxt->client->dev) != 0)
	{
		DBG_PRINT(dbg_level_error, "%s: " MXT_TAG ": %s(): ERROR: Failed to resume.\n", dev_name(&mxt->client->dev), __func__);
	}
}
#endif /* defined(CONFIG_HAS_EARLYSUSPEND) */

static const struct i2c_device_id mxt_id[] = {
	{ "qt602240_ts", 0 },
	{ MXT_DEVICE_224_NAME, 0 },
	{ MXT_DEVICE_768_NAME, 0 },
	{ MXT_DEVICE_1188_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= MXT_DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM
#ifndef CONFIG_HAS_EARLYSUSPEND
		.pm	= &mxt_pm_ops,
#endif
#endif
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
};


#define TOUCHPANEL_GPIO_IRQ     37
#define TOUCHPANEL_GPIO_RESET   39
static struct gpio mxt_touch_gpios[] = {
	{ TOUCHPANEL_GPIO_IRQ,		GPIOF_IN,		"touch_irq"   },
	{ TOUCHPANEL_GPIO_RESET,	GPIOF_OUT_INIT_LOW,	"touch_reset" },
};
static u8 mxt_touch_read_irq(void)
{
	return gpio_get_value(TOUCHPANEL_GPIO_IRQ);
}

#if defined(CONFIG_MACH_OMAP_OVATION)

#define MXT_TOUCH_X_RES 1280
#define MXT_TOUCH_Y_RES 1920

extern int Vdd_LCD_CT_PEN_request_supply(struct device *dev, const char *supply_name);
extern int Vdd_LCD_CT_PEN_enable(struct device *dev, const char *supply_name);
extern int Vdd_LCD_CT_PEN_disable(struct device *dev, const char *supply_name);
extern int Vdd_LCD_CT_PEN_release_supply(struct device *dev, const char *supply_name);

#elif defined(CONFIG_MACH_OMAP_HUMMINGBIRD)

#define MXT_TOUCH_X_RES 900
#define MXT_TOUCH_Y_RES 1440

static struct regulator *mxt_touch_vdd;
static struct regulator *mxt_touch_power;

#endif

static int mxt_touch_request_resources(struct device  *dev)
{
	int ret;

	/* Request GPIO lines */
	ret = gpio_request_array(mxt_touch_gpios, ARRAY_SIZE(mxt_touch_gpios));
	if (ret)
	{
		dev_err(dev, "%s: Could not get touch gpios\n", __func__);
		ret = -EBUSY;
		goto err_gpio_request;
	}
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);
	/* Request the required power supplies */
#if defined(CONFIG_MACH_OMAP_OVATION)
	ret = Vdd_LCD_CT_PEN_request_supply(NULL, "vtp");
	if(ret != 0)
	{
		dev_err(dev, "%s: Could not get touch supplies\n", __func__);
		goto err_regulator_get;
	}
	return 0;
#elif defined(CONFIG_MACH_OMAP_HUMMINGBIRD)
	mxt_touch_vdd = regulator_get(NULL, "touch_vdd");
	if(IS_ERR(mxt_touch_vdd))
	{
		dev_err(dev, "%s: Could not get touch io regulator\n", __func__);
		ret = -EBUSY;
		goto err_regulator_get;
	}
	mxt_touch_power = regulator_get(NULL, "vtp");
	if (IS_ERR(mxt_touch_power)) {
		dev_err(dev, "%s: Could not get touch power regulator\n", __func__);
		ret = -EBUSY;
		goto err_regulator_get_vtp;
	}
	return 0;

err_regulator_get_vtp:
	regulator_put(mxt_touch_vdd);
#endif
err_regulator_get:
	gpio_free_array(mxt_touch_gpios, ARRAY_SIZE(mxt_touch_gpios));
err_gpio_request:
	return ret;
}

static int mxt_touch_release_resources(struct device  *dev)
{
	int ret;

	gpio_free_array(mxt_touch_gpios, ARRAY_SIZE(mxt_touch_gpios));

	/* Release the touch power supplies */
#if defined(CONFIG_MACH_OMAP_OVATION)
	ret = Vdd_LCD_CT_PEN_release_supply(NULL, "vtp");
	if(ret != 0)
	{
		dev_err(dev, "%s: Could not release touch supplies\n", __func__);
	}
#elif defined(CONFIG_MACH_OMAP_HUMMINGBIRD)
	regulator_put(mxt_touch_vdd);
	regulator_put(mxt_touch_power);
#endif

	return 0;
}
static int mxt_touch_power_off(struct device  *dev)
{
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);
	msleep(2);

#if defined(CONFIG_MACH_OMAP_OVATION)
	Vdd_LCD_CT_PEN_disable(NULL, "vtp");
#elif defined(CONFIG_MACH_OMAP_HUMMINGBIRD)
	if(!IS_ERR(mxt_touch_vdd))
	{
		regulator_disable(mxt_touch_vdd);
	}
	else
	{
		dev_err(dev, "%s: Touch io regulator is not valid\n", __func__);
		return -ENODEV;
	}

	if (!IS_ERR(mxt_touch_power))
	{
		regulator_disable(mxt_touch_power);
	}
	else
	{
		dev_err(dev, "%s: Touch power regulator is not valid\n", __func__);
		return -ENODEV;
	}
#endif
	return 0;
}

static int mxt_touch_power_on(struct device  *dev)
{
	int ret;

	gpio_set_value(TOUCHPANEL_GPIO_RESET, 0);
#if defined(CONFIG_MACH_OMAP_OVATION)
	ret = Vdd_LCD_CT_PEN_enable(NULL, "vtp");
#elif defined(CONFIG_MACH_OMAP_HUMMINGBIRD)
	if (!IS_ERR(mxt_touch_power))
	{
		ret = regulator_enable(mxt_touch_power);
		if (ret)
		{
			dev_err(dev, "%s:Could not enable touch power regulator\n", __func__);
			return -EBUSY;
		}
	}
	else
	{
		dev_err(dev, "%s: Touch power regulator is not valid\n", __func__);
		return -ENODEV;

	}

	if(!IS_ERR(mxt_touch_vdd))
	{
		ret = regulator_enable(mxt_touch_vdd);
		if (ret)
		{
			regulator_disable(mxt_touch_power);
			dev_err(dev, "%s: Could not enable touch vdd regulator\n", __func__);
			return -EBUSY;
		}
	}
	else
	{
		regulator_disable(mxt_touch_power);
		dev_err(dev, "%s: Touch io regulator is not valid\n", __func__);
		return -ENODEV;
	}
#endif
	msleep(10);

	/* Pull the nRESET line high after the power stabilises */
	gpio_set_value(TOUCHPANEL_GPIO_RESET, 1);

	msleep(220);

	return 0;
}

static struct mxt_platform_data mxt_pdata = {
#if defined(CONFIG_MACH_OMAP_OVATION)
	.x_line            = 27,
	.y_line            = 39,
	.blen              = 200,
	.threshold         = 60,
        .use_fw_gestures   = 1,
	.reset_on_resume   = 0,
#elif defined(CONFIG_MACH_OMAP_HUMMINGBIRD)
	.x_line            = 24,
	.y_line            = 32,
	.blen              = 0xa0,
	.threshold         = 0x28,
        .use_fw_gestures   = 0,
	.reset_on_resume   = 1,
#endif
	.x_size            = MXT_TOUCH_X_RES,
	.y_size            = MXT_TOUCH_Y_RES,
	.orient            = MXT_HORIZONTAL_FLIP,
	.irqflags          = IRQF_TRIGGER_FALLING,
	.reset_gpio        = TOUCHPANEL_GPIO_RESET,
	.read_chg          = mxt_touch_read_irq,
	.request_resources = mxt_touch_request_resources,
	.release_resources = mxt_touch_release_resources,
	.power_on          = mxt_touch_power_on,
	.power_off         = mxt_touch_power_off,
};

static struct i2c_board_info __initdata mxt_i2c_3_boardinfo[] = {
        {
#if defined(CONFIG_MACH_OMAP_OVATION)
		I2C_BOARD_INFO(MXT_DEVICE_1188_NAME, MXT1188_I2C_SLAVEADDRESS),
#elif defined(CONFIG_MACH_OMAP_HUMMINGBIRD)
                I2C_BOARD_INFO(MXT_DEVICE_768_NAME, MXT768_I2C_SLAVEADDRESS),
#endif
                .platform_data = &mxt_pdata,
                .irq = OMAP_GPIO_IRQ(TOUCHPANEL_GPIO_IRQ),
        },
};
static 	struct i2c_client *mxt_i2c_client;

static int __init mxt_init(void)
{
	struct i2c_adapter *mxt_i2c_adap;
	DBG_PRINT(dbg_level_critical, MXT_TAG ": %s(): INFO: Registering touch controller device\n", __func__);

	mxt_i2c_adap = i2c_get_adapter(3);
	if(mxt_i2c_adap != NULL)
	{
		mxt_i2c_client = i2c_new_device(mxt_i2c_adap, mxt_i2c_3_boardinfo);
		i2c_put_adapter(mxt_i2c_adap);
		mxt_i2c_adap = NULL;
		if(mxt_i2c_client != NULL)
		{
			DBG_PRINT(dbg_level_critical, MXT_TAG ": %s(): INFO: Initializing MXT I2C Touchscreen Driver (Built %s @ %s)\n", __func__, __DATE__, __TIME__);
			return i2c_add_driver(&mxt_driver);
		}
		else
		{
			return -1;
		}
	}
	return -1;
}

static void __exit mxt_exit(void)
{
	DBG_PRINT(dbg_level_info, MXT_TAG ": %s(): INFO: MXT I2C Touchscreen Driver exiting (Built %s @ %s)\n", __func__, __DATE__, __TIME__);
	i2c_del_driver(&mxt_driver);

	if(mxt_i2c_client != NULL)
	{
		i2c_unregister_device(mxt_i2c_client);
	}
}

module_init(mxt_init);
module_exit(mxt_exit);

/* Module information */
MODULE_AUTHOR("Joonyoung Shim <jy0922.shim@samsung.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver");
MODULE_LICENSE("GPL");

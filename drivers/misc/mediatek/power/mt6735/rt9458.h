#ifndef __RT9458_CHARGER_H
#define __RT9458_CHARGER_H

#define RT9458_SLAVE_ADDR    0x6A

enum rt9458_reg_addr {
	RT9458_REG_CTRL1,
	RT9458_REG_CTRL2,
	RT9458_REG_CTRL3,
	RT9458_REG_DEVID,
	RT9458_REG_CTRL4,
	RT9458_REG_CTRL5,
	RT9458_REG_CTRL6,
	RT9458_REG_CTRL7,
	RT9458_REG_IRQ1,
	RT9458_REG_IRQ2,
	RT9458_REG_IRQ3,
	RT9458_REG_MASK1,
	RT9458_REG_MASK2,
	RT9458_REG_MASK3,
	RT9458_REG_CTRL8 = 0x11,
	RT9458_REG_MAX,
};

/* This enumeration defines the index of register
 * You can get register address by the following two ways
 * 1. RT9458_REG_XXXX
 * 2. rt9458_reg_addr[RT9458_REG_IDX_XXXX]
 */
enum rt9458_reg_addr_idx {
	RT9458_REG_IDX_CTRL1 = 0,
	RT9458_REG_IDX_CTRL2,
	RT9458_REG_IDX_CTRL3,
	RT9458_REG_IDX_DEVID,
	RT9458_REG_IDX_CTRL4,
	RT9458_REG_IDX_CTRL5,
	RT9458_REG_IDX_CTRL6,
	RT9458_REG_IDX_CTRL7,
	RT9458_REG_IDX_IRQ1,
	RT9458_REG_IDX_IRQ2,
	RT9458_REG_IDX_IRQ3,
	RT9458_REG_IDX_MASK1,
	RT9458_REG_IDX_MASK2,
	RT9458_REG_IDX_MASK3,
	RT9458_REG_IDX_CTRL8,
	RT9458_REG_IDX_MAX,
};


/* ========== CTRL1 0x00 ============ */
#define RT9458_SHIFT_OTG_POLARITY 1
#define RT9458_SHIFT_POWER_READY  2
#define RT9458_SHIFT_BOOST_MODE   3
#define RT9458_SHIFT_CHG_STAT     4

#define RT9458_MASK_OTG_POLARITY (1 << RT9458_SHIFT_OTG_POLARITY)
#define RT9458_MASK_POWER_READY  (1 << RT9458_SHIFT_POWER_READY)
#define RT9458_MASK_BOOST_MODE   (1 << RT9458_SHIFT_BOOST_MODE)
#define RT9458_MASK_CHG_STAT  0x30 

/* ========== CTRL2 0x01 ============ */
#define RT9458_SHIFT_OPA_MODE   0
#define RT9458_SHIFT_HZ_EN      1
#define RT9458_SHIFT_AICR_INT   2
#define RT9458_SHIFT_TE_EN      3
#define RT9458_SHIFT_HIGHER_OCP 4
#define RT9458_SHIFT_TE_SHDN_EN 5
#define RT9458_SHIFT_AICR       6

#define RT9458_MASK_OPA_MODE   (1 << RT9458_SHIFT_OPA_MODE)
#define RT9458_MASK_HZ_EN      (1 << RT9458_SHIFT_HZ_EN)
#define RT9458_MASK_AICR_INT   (1 << RT9458_SHIFT_AICR_INT)
#define RT9458_MASK_TE_EN      (1 << RT9458_SHIFT_TE_EN)
#define RT9458_MASK_HIGHER_OCP (1 << RT9458_SHIFT_HIGHER_OCP)
#define RT9458_MASK_TE_SHDN_EN (1 << RT9458_SHIFT_TE_SHDN_EN)
#define RT9458_MASK_AICR      0xC0


/* ========== CTRL3 0x02 ============ */
#define RT9458_SHIFT_OTG_EN 0
#define RT9458_SHIFT_OTG_PL 1
#define RT9458_SHIFT_VOREG  2

#define RT9458_MASK_OTG_EN (1 << RT9458_SHIFT_OTG_EN)
#define RT9458_MASK_OTG_PL (1 << RT9458_SHIFT_OTG_PL)
#define RT9458_MASK_VOREG  0xFC

/* ========== CTRL4 0x04 ============ */
#define RT9458_SHIFT_RST  7
#define RT9458_MASK_RST   (1 << RT9458_SHIFT_RST) 

/* ========== CTRL5 0x05 ============ */
#define RT9458_SHIFT_IEOC    0
#define RT9458_SHIFT_TMR_EN 7

#define RT9458_MASK_TMR_EN  (1 << RT9458_SHIFT_TMR_EN)
#define RT9458_MASK_IEOC     0x07

/* ========== CTRL6 0x06 ============ */
#define RT9458_SHIFT_VPREC    0
#define RT9458_SHIFT_FIX_FREQ 3
#define RT9458_SHIFT_ICHG     4
#define RT9458_SHIFT_AICR_SEL 7

#define RT9458_MASK_VPREC    0x07
#define RT9458_MASK_FIX_FREQ (1 << RT9458_SHIFT_FIX_FREQ)
#define RT9458_MASK_ICHG     0x70
#define RT9458_MASK_AICR_SEL (1 << RT9458_SHIFT_AICR_SEL)

/* ========== CTRL7 0x07 ============ */
#define RT9458_SHIFT_VMREG   0
#define RT9458_SHIFT_CHG_EN  4
#define RT9458_SHIFT_BATD_EN 6

#define RT9458_MASK_VMREG   0x0F
#define RT9458_MASK_CHG_EN  (1 << RT9458_SHIFT_CHG_EN)
#define RT9458_MASK_BATD_EN (1 << RT9458_SHIFT_BATD_EN)

/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
/* ========== IRQ1 0x08 ============ */
#define RT9458_SHIFT_VINOVPI 6
#define RT9458_MASK_VINOVPI   0x40
/*[Arima_8100][bozhi_lin] 20170320 end*/

/* ========== CTRL8 0x11 ============ */
#define RT9458_SHIFT_IPREC 0
#define RT9458_SHIFT_MIVR  4

#define RT9458_MASK_IPREC 0x03
#define RT9458_MASK_MIVR  0x70 

enum rt_charging_status {
	RT_CHG_STATUS_READY = 0,
	RT_CHG_STATUS_PROGRESS,
	RT_CHG_STATUS_DONE,
	RT_CHG_STATUS_FAULT,
	RT_CHG_STATUS_MAX,
};

/* You must implement all of the following interfaces */
extern int rt_charger_hw_init(void);
extern int rt_charger_sw_init(void);
extern int rt_charger_dump_register(void);
extern int rt_charger_enable_charging(const u8 enable);
extern int rt_charger_enable_hz(const u8 enable);
extern int rt_charger_enable_te(const u8 enable);
extern int rt_charger_enable_te_shutdown(const u8 enable);
extern int rt_charger_enable_timer(const u8 enable);
extern int rt_charger_enable_otg(const u8 enable);
extern int rt_charger_enable_pumpX(void);
extern int rt_charger_disable_aicr(void);
extern int rt_charger_disable_mivr(void);

extern int rt_charger_set_ichg(const u32 ichg);
extern int rt_charger_set_ieoc(const u32 ieoc);
extern int rt_charger_set_aicr(const u32 aicr);
extern int rt_charger_set_mivr(const u32 mivr);
extern int rt_charger_set_battery_voreg(const u32 voreg);
extern int rt_charger_set_boost_voreg(const u32 voreg);
extern int rt_charger_set_ta_current_pattern(const u8 is_pump_up);

extern int rt_charger_get_charging_status(enum rt_charging_status *chg_stat);
extern int rt_charger_get_ichg(u32 *ichg);
extern int rt_charger_get_ieoc(u32 *ieoc);
extern int rt_charger_get_aicr(u32 *aicr);
extern int rt_charger_get_mivr(u32 *mivr);
extern int rt_charger_get_battery_voreg(u32 *voreg);
extern int rt_charger_get_boost_voreg(u32 *voreg);

extern int rt_charger_is_charging_enable(u8 *enable);
extern int rt_charger_reset_wchdog_timer(void);
/*[Arima_8100][bozhi_lin] RID003699 OVP warning message to end-user 20170320 begin*/
extern int rt_charger_get_vinovpi(u32 *vinovpi);
/*[Arima_8100][bozhi_lin] 20170320 end*/

#endif /* __RT9458_CHARGER_H */

/*
 * Copyright (C) 2016 Richtek Technology Corp.
 *
 * TCPC Interface for timer handler
 *
 * Author: TH <tsunghan_tsai@richtek.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/atomic.h>
#include <linux/kthread.h>
#include <linux/hrtimer.h>
#include <linux/version.h>

#if 1 /* (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)) */
#include <linux/sched/rt.h>
#endif /* #if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 9, 0)) */

#include "inc/tcpci.h"
#include "inc/tcpci_timer.h"
#include "inc/tcpci_typec.h"

#define RT_MASK64(i)	(((uint64_t)1) << i)

#define TIMEOUT_VAL(val)	(val * 1000)
#define TIMEOUT_RANGE(min, max)		((min * 4000 + max * 1000)/5)
#define TIMEOUT_VAL_US(val)	(val)

/* Debug message Macro */
#if TCPC_TIMER_DBG_EN
#define TCPC_TIMER_DBG(tcpc, id)				\
{								\
	RT_DBG_INFO("Trigger %s\n", tcpc_timer_name[id]);	\
}
#else
#define TCPC_TIMER_DBG(format, args...)
#endif /* TCPC_TIMER_DBG_EN */

#if TCPC_TIMER_INFO_EN
#define TCPC_TIMER_EN_DBG(tcpc, id)				\
{								\
	RT_DBG_INFO("Enable %s\n", tcpc_timer_name[id]);	\
}
#else
#define TCPC_TIMER_EN_DBG(format, args...)
#endif /* TCPC_TIMER_INFO_EN */

static inline uint64_t rt_get_value(uint64_t *p)
{
	unsigned long flags;
	uint64_t data;

	raw_local_irq_save(flags);
	data = *p;
	raw_local_irq_restore(flags);
	return data;
}

static inline void rt_set_value(uint64_t *p, uint64_t data)
{
	unsigned long flags;

	raw_local_irq_save(flags);
	*p = data;
	raw_local_irq_restore(flags);
}

static inline void rt_clear_bit(int nr, uint64_t *addr)
{
	uint64_t mask = ((uint64_t)1) << nr;
	unsigned long flags;

	raw_local_irq_save(flags);
	*addr &= ~mask;
	raw_local_irq_restore(flags);
}

static inline void rt_set_bit(int nr, uint64_t *addr)
{
	uint64_t mask = ((uint64_t)1) << nr;
	unsigned long flags;

	raw_local_irq_save(flags);
	*addr |= mask;
	raw_local_irq_restore(flags);
}

static const char *const tcpc_timer_name[] = {
	"TYPEC_RT_TIMER_SAFE0V_DELAY",
	"TYPEC_RT_TIMER_SAFE0V_TOUT",
	"TYPEC_RT_TIMER_ROLE_SWAP_START",
	"TYPEC_RT_TIMER_ROLE_SWAP_STOP",
	"TYPEC_RT_TIMER_LEGACY",
	"TYPEC_RT_TIMER_NOT_LEGACY",
	"TYPEC_RT_TIMER_AUTO_DISCHARGE",

	"TYPEC_TRY_TIMER_DRP_TRY",
	"TYPEC_TRY_TIMER_DRP_TRYWAIT",

	"TYPEC_TIMER_CCDEBOUNCE",
	"TYPEC_TIMER_PDDEBOUNCE",
	"TYPEC_TIMER_WAKEUP_TOUT",
	"TYPEC_TIMER_DRP_SRC_TOGGLE",
};


#ifdef CONFIG_USB_PD_SAFE0V_DELAY
#define PD_TIMER_VSAFE0V_DLY_TOUT		TIMEOUT_VAL(50)
#else
#define PD_TIMER_VSAFE0V_DLY_TOUT		TIMEOUT_VAL(400)
#endif	/* CONFIG_USB_PD_SAFE0V_DELAY */

#ifdef CONFIG_TCPC_VSAFE0V_DETECT
#define TYPEC_RT_TIMER_SAFE0V_DLY_TOUT		TIMEOUT_VAL(35)
#else
#define TYPEC_RT_TIMER_SAFE0V_DLY_TOUT		TIMEOUT_VAL(100)
#endif

static const uint32_t tcpc_timer_timeout[PD_TIMER_NR] = {
	/* TYPEC-RT-TIMER */
	TYPEC_RT_TIMER_SAFE0V_DLY_TOUT,	/* TYPEC_RT_TIMER_SAFE0V_DELAY */
	TIMEOUT_VAL(650),			/* TYPEC_RT_TIMER_SAFE0V_TOUT */
	/* TYPEC_RT_TIMER_ROLE_SWAP */
	TIMEOUT_VAL(20),
	TIMEOUT_VAL(CONFIG_TYPEC_CAP_ROLE_SWAP_TOUT),
	TIMEOUT_VAL(50),	/* TYPEC_RT_TIMER_LEGACY */
	TIMEOUT_VAL(5000),	/* TYPEC_RT_TIMER_NOT_LEGACY */
	/* TYPEC_RT_TIMER_AUTO_DISCHARGE */
	TIMEOUT_VAL(CONFIG_TYPEC_CAP_AUTO_DISCHARGE_TOUT),
	TIMEOUT_VAL(500),		/* TYPEC_RT_TIMER_LOW_POWER_MODE */

	/* TYPEC-TRY-TIMER */
	TIMEOUT_RANGE(75, 150),		/* TYPEC_TRY_TIMER_DRP_TRY */
	TIMEOUT_RANGE(400, 800),	/* TYPEC_TRY_TIMER_DRP_TRYWAIT */

	TIMEOUT_RANGE(100, 200),	/* TYPEC_TIMER_CCDEBOUNCE */
	TIMEOUT_RANGE(10, 10),		/* TYPEC_TIMER_PDDEBOUNCE */

	TIMEOUT_VAL(300*1000),	/* TYPEC_TIMER_WAKEUP_TOUT */
	TIMEOUT_VAL(60),			/* TYPEC_TIMER_DRP_SRC_TOGGLE */
};

typedef enum hrtimer_restart (*tcpc_hrtimer_call)(struct hrtimer *timer);

/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
#define TCPC_TIMER_TRIGGER()	do \
{				\
	down(&tcpc_dev->timer_tick_lock);			\
	rt_set_bit(index, (uint64_t *)&tcpc_dev->timer_tick);	\
	up(&tcpc_dev->timer_tick_lock);				\
	wake_up_interruptible(&tcpc_dev->timer_wait_que);	\
} while (0)
/*[Arima_8100][bozhi_lin] 20161117 end*/

static enum hrtimer_restart tcpc_timer_rt_vsafe0v_delay(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_SAFE0V_DELAY;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_rt_vsafe0v_tout(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_SAFE0V_TOUT;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_rt_role_swap_start(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_ROLE_SWAP_START;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_rt_role_swap_stop(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_ROLE_SWAP_STOP;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_rt_legacy(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_LEGACY;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_rt_not_legacy(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_NOT_LEGACY;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_rt_auto_discharge(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_AUTO_DISCHARGE;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_rt_low_power_mode(struct hrtimer *timer)
{
	int index = TYPEC_RT_TIMER_LOW_POWER_MODE;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_try_drp_try(struct hrtimer *timer)
{
	int index = TYPEC_TRY_TIMER_DRP_TRY;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_try_drp_trywait(struct hrtimer *timer)
{
	int index = TYPEC_TRY_TIMER_DRP_TRYWAIT;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_ccdebounce(struct hrtimer *timer)
{
	int index = TYPEC_TIMER_CCDEBOUNCE;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_pddebounce(struct hrtimer *timer)
{
	int index = TYPEC_TIMER_PDDEBOUNCE;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_wakeup(struct hrtimer *timer)
{
	int index = TYPEC_TIMER_WAKEUP;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static enum hrtimer_restart tcpc_timer_drp_src_toggle(struct hrtimer *timer)
{
	int index = TYPEC_TIMER_DRP_SRC_TOGGLE;
	struct tcpc_device *tcpc_dev =
		container_of(timer, struct tcpc_device, tcpc_timer[index]);

	TCPC_TIMER_TRIGGER();
	return HRTIMER_NORESTART;
}

static tcpc_hrtimer_call tcpc_timer_call[PD_TIMER_NR] = {
	[TYPEC_RT_TIMER_SAFE0V_DELAY] = tcpc_timer_rt_vsafe0v_delay,
	[TYPEC_RT_TIMER_SAFE0V_TOUT] = tcpc_timer_rt_vsafe0v_tout,
	[TYPEC_RT_TIMER_ROLE_SWAP_START] = tcpc_timer_rt_role_swap_start,
	[TYPEC_RT_TIMER_ROLE_SWAP_STOP] = tcpc_timer_rt_role_swap_stop,
	[TYPEC_RT_TIMER_LEGACY] = tcpc_timer_rt_legacy,
	[TYPEC_RT_TIMER_NOT_LEGACY] = tcpc_timer_rt_not_legacy,
	[TYPEC_RT_TIMER_AUTO_DISCHARGE] = tcpc_timer_rt_auto_discharge,
	[TYPEC_RT_TIMER_LOW_POWER_MODE] = tcpc_timer_rt_low_power_mode,

	[TYPEC_TRY_TIMER_DRP_TRY] = tcpc_timer_try_drp_try,
	[TYPEC_TRY_TIMER_DRP_TRYWAIT] = tcpc_timer_try_drp_trywait,

	[TYPEC_TIMER_CCDEBOUNCE] = tcpc_timer_ccdebounce,
	[TYPEC_TIMER_PDDEBOUNCE] = tcpc_timer_pddebounce,
	[TYPEC_TIMER_WAKEUP] = tcpc_timer_wakeup,
	[TYPEC_TIMER_DRP_SRC_TOGGLE] = tcpc_timer_drp_src_toggle,
};

/*
 * [BLOCK] Control Timer
 */

static inline void tcpc_reset_timer_range(
		struct tcpc_device *tcpc, int start, int end)
{
	int i;
	uint64_t mask;

	down(&tcpc->timer_enable_mask_lock);
	mask = rt_get_value((uint64_t *)&tcpc->timer_enable_mask);
	up(&tcpc->timer_enable_mask_lock);

	for (i = start; i <= end; i++) {
		if (mask & (((uint64_t)1) << i)) {
			hrtimer_try_to_cancel(&tcpc->tcpc_timer[i]);
			down(&tcpc->timer_enable_mask_lock);
			rt_clear_bit(i, (uint64_t *)&tcpc->timer_enable_mask);
			up(&tcpc->timer_enable_mask_lock);
		}
	}
}

void tcpc_restart_timer(struct tcpc_device *tcpc, uint32_t timer_id)
{
	uint64_t mask;

	down(&tcpc->timer_enable_mask_lock);
	mask = rt_get_value((uint64_t *)&tcpc->timer_enable_mask);
	up(&tcpc->timer_enable_mask_lock);
	if (mask & (((uint64_t)1) << timer_id))
		tcpc_disable_timer(tcpc, timer_id);
	tcpc_enable_timer(tcpc, timer_id);
}

void tcpc_enable_timer(struct tcpc_device *tcpc, uint32_t timer_id)
{
	uint32_t r, mod, tout;

	TCPC_TIMER_EN_DBG(tcpc, timer_id);
	PD_BUG_ON(timer_id >= PD_TIMER_NR);

	mutex_lock(&tcpc->timer_lock);
	if (timer_id >= TYPEC_TIMER_START_ID)
		tcpc_reset_timer_range(tcpc, TYPEC_TIMER_START_ID, PD_TIMER_NR);

	down(&tcpc->timer_enable_mask_lock);
	rt_set_bit(timer_id, (uint64_t *)&tcpc->timer_enable_mask);
	up(&tcpc->timer_enable_mask_lock);

	tout = tcpc_timer_timeout[timer_id];

#ifdef CONFIG_USB_PD_RANDOM_FLOW_DELAY
	if (timer_id == PD_TIMER_DFP_FLOW_DELAY ||
			timer_id == PD_TIMER_UFP_FLOW_DELAY)
		tout += TIMEOUT_VAL(jiffies & 0x07);
#endif	/* CONFIG_USB_PD_RANDOM_FLOW_DELAY */

	r =  tout / 1000000;
	mod = tout % 1000000;

	mutex_unlock(&tcpc->timer_lock);
	hrtimer_start(&tcpc->tcpc_timer[timer_id],
				ktime_set(r, mod*1000), HRTIMER_MODE_REL);
}

void tcpc_disable_timer(struct tcpc_device *tcpc_dev, uint32_t timer_id)
{
	uint64_t mask;

	down(&tcpc_dev->timer_enable_mask_lock);
	mask = rt_get_value((uint64_t *)&tcpc_dev->timer_enable_mask);
	up(&tcpc_dev->timer_enable_mask_lock);

	PD_BUG_ON(timer_id >= PD_TIMER_NR);
	if (mask&(((uint64_t)1)<<timer_id)) {
		hrtimer_try_to_cancel(&tcpc_dev->tcpc_timer[timer_id]);
		rt_clear_bit(timer_id,
			(uint64_t *)&tcpc_dev->timer_enable_mask);
	}
}

void tcpc_timer_reset(struct tcpc_device *tcpc_dev)
{
	uint64_t mask;
	int i;

	down(&tcpc_dev->timer_enable_mask_lock);
	mask = rt_get_value((uint64_t *)&tcpc_dev->timer_enable_mask);
	up(&tcpc_dev->timer_enable_mask_lock);
	for (i = 0; i < PD_TIMER_NR; i++)
		if (mask & (((uint64_t)1) << i))
			hrtimer_try_to_cancel(&tcpc_dev->tcpc_timer[i]);
	rt_set_value((uint64_t *)&tcpc_dev->timer_enable_mask, 0);
}

void tcpc_reset_typec_debounce_timer(struct tcpc_device *tcpc)
{
	mutex_lock(&tcpc->timer_lock);
	tcpc_reset_timer_range(tcpc, TYPEC_TIMER_START_ID, PD_TIMER_NR);
	mutex_unlock(&tcpc->timer_lock);
}

void tcpc_reset_typec_try_timer(struct tcpc_device *tcpc)
{
	mutex_lock(&tcpc->timer_lock);
	tcpc_reset_timer_range(tcpc,
			TYPEC_TRY_TIMER_START_ID, TYPEC_TIMER_START_ID);
	mutex_unlock(&tcpc->timer_lock);
}

static void tcpc_handle_timer_triggered(struct tcpc_device *tcpc_dev)
{
	uint64_t triggered_timer;
	int i = 0;

/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
	down(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/
	triggered_timer = rt_get_value((uint64_t *)&tcpc_dev->timer_tick);
/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
	up(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/

	mutex_lock(&tcpc_dev->typec_lock);
	for (; i < PD_TIMER_NR; i++) {
		if (triggered_timer & RT_MASK64(i)) {
			TCPC_TIMER_DBG(tcpc_dev, i);
			tcpc_typec_handle_timeout(tcpc_dev, i);
/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
			down(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/
			rt_clear_bit(i, (uint64_t *)&tcpc_dev->timer_tick);
/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
			up(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/
		}
	}
	mutex_unlock(&tcpc_dev->typec_lock);

}

static int tcpc_timer_thread(void *param)
{
	struct tcpc_device *tcpc_dev = param;

	uint64_t *timer_tick;
	struct sched_param sch_param = {.sched_priority = MAX_RT_PRIO - 1};


/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
	down(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/
	timer_tick = &tcpc_dev->timer_tick;
/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
	up(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/

	sched_setscheduler(current, SCHED_FIFO, &sch_param);
	while (true) {
		wait_event_interruptible(tcpc_dev->timer_wait_que,
				((*timer_tick) ? true : false) |
				tcpc_dev->timer_thead_stop);
		if (kthread_should_stop() || tcpc_dev->timer_thead_stop)
			break;
		do {
			tcpc_handle_timer_triggered(tcpc_dev);
		} while (*timer_tick);
	}
	return 0;
}

int tcpci_timer_init(struct tcpc_device *tcpc_dev)
{
	int i;

	pr_info("PD Timer number = %d\n", PD_TIMER_NR);
	tcpc_dev->timer_task = kthread_create(tcpc_timer_thread, tcpc_dev,
			"tcpc_timer_%s.%p", dev_name(&tcpc_dev->dev), tcpc_dev);
	init_waitqueue_head(&tcpc_dev->timer_wait_que);
/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
	down(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/
	tcpc_dev->timer_tick = 0;
/*[Arima_8100][bozhi_lin] rt1711 set USB_INT_N as gpio81 and fix stack error 20161117 begin*/
	up(&tcpc_dev->timer_tick_lock);
/*[Arima_8100][bozhi_lin] 20161117 end*/
	rt_set_value((uint64_t *)&tcpc_dev->timer_enable_mask, 0);
	wake_up_process(tcpc_dev->timer_task);
	for (i = 0; i < PD_TIMER_NR; i++) {
		hrtimer_init(&tcpc_dev->tcpc_timer[i],
					CLOCK_MONOTONIC, HRTIMER_MODE_REL);
		tcpc_dev->tcpc_timer[i].function = tcpc_timer_call[i];
	}

	pr_info("%s : init OK\n", __func__);
	return 0;
}

int tcpci_timer_deinit(struct tcpc_device *tcpc_dev)
{
	uint64_t mask;
	int i;

	down(&tcpc_dev->timer_enable_mask_lock);
	mask = rt_get_value((uint64_t *)&tcpc_dev->timer_enable_mask);
	up(&tcpc_dev->timer_enable_mask_lock);

	mutex_lock(&tcpc_dev->timer_lock);
	wake_up_interruptible(&tcpc_dev->timer_wait_que);
	kthread_stop(tcpc_dev->timer_task);
	for (i = 0; i < PD_TIMER_NR; i++) {
		if (mask & (1 << i))
			hrtimer_try_to_cancel(&tcpc_dev->tcpc_timer[i]);
	}

	pr_info("%s : de init OK\n", __func__);
	mutex_unlock(&tcpc_dev->timer_lock);
	return 0;
}

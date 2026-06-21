/*
 * RTC
 * */
#define PMB887X_TRACE_ID		RTC
#define PMB887X_TRACE_PREFIX	"pmb887x-rtc"

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/hw.h"
#include "system/memory.h"
#include "cpu.h"
#include "qapi/error.h"
#include "qemu/timer.h"
#include "qemu/main-loop.h"
#include "hw/qdev-properties.h"

#include "hw/arm/pmb887x/gen/cpu_regs.h"
#include "hw/arm/pmb887x/regs_dump.h"
#include "hw/arm/pmb887x/mod.h"
#include "hw/arm/pmb887x/trace.h"

#define TYPE_PMB887X_RTC	"pmb887x-rtc"
#define PMB887X_RTC(obj)	OBJECT_CHECK(pmb887x_rtc_t, (obj), TYPE_PMB887X_RTC)

typedef struct pmb887x_rtc_t pmb887x_rtc_t;

struct pmb887x_rtc_t {
	SysBusDevice parent_obj;
	MemoryRegion mmio;
	
	pmb887x_clc_reg_t clc;
	pmb887x_src_reg_t src;
	qemu_irq irq;
	
	uint32_t ctrl;
	uint32_t con;
	uint32_t t14;
	uint32_t cnt;
	uint32_t rel;
	uint32_t isnc;
	uint32_t alarm;
	uint32_t unk0;

	uint64_t realtime_start;
	uint64_t virtual_start;

	/* Packed RTC_CNT modelling: the firmware seeds RTC_CNT/RTC_REL with a
	 * packed S-Gold2 section counter and the hardware then auto-advances it.
	 * We capture the last firmware-written seed and the virtual time at which
	 * it was written, then re-derive RTC_CNT on every read. */
	uint32_t cnt_seed;			/* packed value last written by firmware */
	uint64_t cnt_seed_ns;		/* QEMU_CLOCK_VIRTUAL ns at that write */

	QEMUTimer *tick;
};

/*
 *   bits [31:22] CNT3 = day-of-year field, encoded (1024 - yeardays + doy - 1)
 *   bits [21:16] CNT2 = hours   (field value - 40, range 0..23)
 *   bits [15:10] CNT1 = minutes (field value - 4,  range 0..59)
 *   bits [9:0]   CNT0 = seconds (field value - 964, range 0..59)
 */
#define RTC_YEARDAYS 365

static uint32_t rtc_cnt_encode(uint32_t secs_in_year) {
	uint32_t sec = secs_in_year % 60;
	uint32_t min = (secs_in_year / 60) % 60;
	uint32_t hour = (secs_in_year / 3600) % 24;
	uint32_t doy = (secs_in_year / 86400) + 1;	/* 1-based */
	uint32_t cnt3 = (1024 - RTC_YEARDAYS + doy - 1) & 0x3FF;
	return (cnt3 << 22)
		| (((hour + 40) & 0x3F) << 16)
		| (((min + 4) & 0x3F) << 10)
		| ((sec + 964) & 0x3FF);
}

static uint32_t rtc_cnt_decode(uint32_t cnt) {
	int32_t sec = (int32_t)(cnt & 0x3FF) - 964;
	int32_t min = (int32_t)((cnt >> 10) & 0x3F) - 4;
	int32_t hour = (int32_t)((cnt >> 16) & 0x3F) - 40;
	int32_t cnt3 = (int32_t)((cnt >> 22) & 0x3FF);
	int32_t doy = RTC_YEARDAYS - (1024 - cnt3) + 1;	/* 1-based */

	if (sec < 0 || sec > 59) sec = 0;
	if (min < 0 || min > 59) min = 0;
	if (hour < 0 || hour > 23) hour = 0;
	if (doy < 1 || doy > RTC_YEARDAYS) doy = 1;

	return (uint32_t)((doy - 1) * 86400 + hour * 3600 + min * 60 + sec);
}

static uint32_t rtc_cnt_now(pmb887x_rtc_t *p) {
	uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
	uint64_t elapsed = (now - p->cnt_seed_ns) / NANOSECONDS_PER_SECOND;
	uint32_t base = rtc_cnt_decode(p->cnt_seed);
	uint32_t total = (uint32_t)((base + elapsed) % (RTC_YEARDAYS * 86400ULL));
	return rtc_cnt_encode(total);
}

#define RTC_TICK_HZ 1

static void rtc_tick_rearm(pmb887x_rtc_t *p) {
	timer_mod(p->tick, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
		NANOSECONDS_PER_SECOND / RTC_TICK_HZ);
}

static void rtc_tick(void *opaque) {
	pmb887x_rtc_t *p = opaque;

	uint32_t cnt = rtc_cnt_now(p);
	uint32_t set = 0;

	/* Set the request (IR) flag for every interrupt source the firmware has
	 * enabled (IE). IR bit is the IE bit << 1. The LISR recomputes the
	 * actual time fields from RTC_CNT, so signalling on every second is
	 * safe regardless of which section actually rolled over. */
	if ((p->isnc & RTC_ISNC_T14IE))
		set |= RTC_ISNC_T14IR;
	if ((p->isnc & RTC_ISNC_RTC0IE))
		set |= RTC_ISNC_RTC0IR;
	if ((p->isnc & RTC_ISNC_RTC1IE))
		set |= RTC_ISNC_RTC1IR;
	if ((p->isnc & RTC_ISNC_RTC2IE))
		set |= RTC_ISNC_RTC2IR;
	if ((p->isnc & RTC_ISNC_RTC3IE))
		set |= RTC_ISNC_RTC3IR;
	if ((p->isnc & RTC_ISNC_ALARMIE) && p->alarm && cnt >= p->alarm)
		set |= RTC_ISNC_ALARMIR;

	if (set) {
		p->isnc |= set;
		pmb887x_src_update(&p->src, 0, MOD_SRC_SETR);
	}

	rtc_tick_rearm(p);
}

static uint64_t rtc_io_read(void *opaque, hwaddr haddr, unsigned size) {
	pmb887x_rtc_t *p = opaque;
	
	uint64_t value = 0;
	
	switch (haddr) {
		case RTC_CLC:
			value = pmb887x_clc_get(&p->clc);
			break;
		
		case RTC_ID:
			value = 0xF049C011;
			break;
		
		case RTC_CTRL:
			value = p->ctrl;
			break;
		
		case RTC_CON:
			value = p->con | RTC_CON_ACCPOS;
			break;
		
		case RTC_T14:
			value = p->t14;
			break;
		
		case RTC_CNT:
			value = rtc_cnt_now(p);
			break;
		
		case RTC_REL:
			value = p->rel;
			break;
		
		case RTC_ISNC:
			value = p->isnc;
			break;
		
		case RTC_ALARM:
			value = p->alarm;
			break;
		
		case RTC_UNK0:
			value = p->unk0;
			break;
		
		case RTC_SRC:
			value = pmb887x_src_get(&p->src);
			break;
		
		default:
			IO_DUMP(haddr + p->mmio.addr, size, 0xFFFFFFFF, false);
			EPRINTF("unknown reg access: %02"PRIX64"\n", haddr);
			exit(1);
	}
	
	IO_DUMP(haddr + p->mmio.addr, size, value, false);
	
	return value;
}

static void rtc_io_write(void *opaque, hwaddr haddr, uint64_t value, unsigned size) {
	pmb887x_rtc_t *p = opaque;
	
	IO_DUMP(haddr + p->mmio.addr, size, value, true);
	
	switch (haddr) {
		case RTC_CLC:
			pmb887x_clc_set(&p->clc, value);
			break;
		
		case RTC_CTRL:
			p->ctrl = value;
			break;
		
		case RTC_CON:
			p->con = value;
			break;
		
		case RTC_T14:
			p->t14 = value;
			break;
		
		case RTC_CNT:
			p->cnt = value;
			p->cnt_seed = value;
			p->cnt_seed_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
			break;
		
		case RTC_REL:
			p->rel = value;
			break;
		
		case RTC_ISNC:
			p->isnc = value;
			break;
		
		case RTC_ALARM:
			p->alarm = value;
			break;
		
		case RTC_UNK0:
			p->unk0 = value;
			break;
		
		case RTC_SRC:
			pmb887x_src_set(&p->src, value);
			break;
		
		default:
			EPRINTF("unknown reg access: %02"PRIX64"\n", haddr);
			exit(1);
	}
}

static const MemoryRegionOps io_ops = {
	.read			= rtc_io_read,
	.write			= rtc_io_write,
	.endianness		= DEVICE_NATIVE_ENDIAN,
	.valid			= {
		.min_access_size	= 1,
		.max_access_size	= 4
	}
};

static void rtc_init(Object *obj) {
	pmb887x_rtc_t *p = PMB887X_RTC(obj);
	memory_region_init_io(&p->mmio, obj, &io_ops, p, "pmb887x-rtc", RTC_IO_SIZE);
	sysbus_init_mmio(SYS_BUS_DEVICE(obj), &p->mmio);
	sysbus_init_irq(SYS_BUS_DEVICE(obj), &p->irq);
}

static void rtc_realize(DeviceState *dev, Error **errp) {
	pmb887x_rtc_t *p = PMB887X_RTC(dev);
	
	if (!p->irq)
		hw_error("pmb887x-rtc: irq not set");
	
	p->virtual_start = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
	p->realtime_start = qemu_clock_get_ns(QEMU_CLOCK_REALTIME);

	p->cnt_seed = 0xA4D9F9C4;
	p->cnt_seed_ns = p->virtual_start;

	pmb887x_clc_init(&p->clc);
	pmb887x_src_init(&p->src, p->irq);

	p->tick = timer_new_ns(QEMU_CLOCK_VIRTUAL, rtc_tick, p);
	rtc_tick_rearm(p);
}

static void rtc_class_init(ObjectClass *klass, const void *data) {
	DeviceClass *dc = DEVICE_CLASS(klass);
	dc->realize = rtc_realize;
}

static const TypeInfo rtc_info = {
    .name          	= TYPE_PMB887X_RTC,
    .parent        	= TYPE_SYS_BUS_DEVICE,
    .instance_size 	= sizeof(struct pmb887x_rtc_t),
    .instance_init 	= rtc_init,
    .class_init    	= rtc_class_init,
};

static void rtc_register_types(void) {
	type_register_static(&rtc_info);
}
type_init(rtc_register_types)
